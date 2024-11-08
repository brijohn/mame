// license:BSD-3-Clause
// copyright-holders:Brian Johnson


#include "emu.h"

#include "nabupc.h"

#include "bus/nabu/adapter.h"
#include "bus/nabu/ctrl/hlekeyboard.h"
#include "bus/nabu/ctrl/keyboard.h"
#include "bus/nabu/video/video.h"
#include "bus/rs232/null_modem.h"
#include "bus/rs232/pty.h"
#include "bus/rs232/rs232.h"
#include "machine/clock.h"
#include "emuopts.h"

#include "nabupc.lh"

#define HCCA_TAG "hcca"

static INPUT_PORTS_START( nabupc )
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x03, 0x02, "BIOS Size" )
	PORT_CONFSETTING( 0x02, "4k BIOS" )
	PORT_CONFSETTING( 0x01, "8k BIOS" )
INPUT_PORTS_END

static DEVICE_INPUT_DEFAULTS_START ( hcca_rs232_defaults )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_111900 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_111900 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
	DEVICE_INPUT_DEFAULTS( "FLOW_CONTROL", 0x07, 0x00 ) // No flow control
DEVICE_INPUT_DEFAULTS_END

// Bit manipulation
namespace {
	template<typename T> constexpr T BIT_SHIFT(unsigned n, unsigned v)
	{
		return (T)v << n;
	}

	template<typename T> void BIT_SET(T& w , unsigned n, unsigned v)
	{
		T mask = ~BIT_SHIFT<T>(n, 1);
		w = (w & mask) | BIT_SHIFT<T>(n, v);
	}
}

static void hcca_devices(device_slot_interface &device)
{
	device.option_add("pty",           PSEUDO_TERMINAL);
	device.option_add("null_modem",    NULL_MODEM);
	device.option_add("hcca_local",    NABU_NETWORK_LOCAL_ADAPTER);
	device.option_add("hcca_remote",   NABU_NETWORK_REMOTE_ADAPTER);
}

static void keyboard_devices(device_slot_interface &device)
{
	device.option_add("nabu", NABU_KBD);
	device.option_add("nabu_hle", NABU_HLE_KEYBOARD);
}

/** @brief F9318 input lines */
typedef enum {
	PRIO_IN_EI = (1<<8),
	PRIO_IN_I7 = (1<<7),
	PRIO_IN_I6 = (1<<6),
	PRIO_IN_I5 = (1<<5),
	PRIO_IN_I4 = (1<<4),
	PRIO_IN_I3 = (1<<3),
	PRIO_IN_I2 = (1<<2),
	PRIO_IN_I1 = (1<<1),
	PRIO_IN_I0 = (1<<0),
	/* masks */
	PRIO_I7 = PRIO_IN_I7,
	PRIO_I6_I7 = (PRIO_IN_I6 | PRIO_IN_I7),
	PRIO_I5_I7 = (PRIO_IN_I5 | PRIO_I6_I7),
	PRIO_I4_I7 = (PRIO_IN_I4 | PRIO_I5_I7),
	PRIO_I3_I7 = (PRIO_IN_I3 | PRIO_I4_I7),
	PRIO_I2_I7 = (PRIO_IN_I2 | PRIO_I3_I7),
	PRIO_I1_I7 = (PRIO_IN_I1 | PRIO_I2_I7),
	PRIO_I0_I7 = (PRIO_IN_I0 | PRIO_I1_I7),
}   f9318_in_t;

/** @brief F9318 output lines */
typedef enum {
	PRIO_OUT_Q0 = (1<<0),
	PRIO_OUT_Q1 = (1<<1),
	PRIO_OUT_Q2 = (1<<2),
	PRIO_OUT_EO = (1<<3),
	PRIO_OUT_GS = (1<<4),
	/* masks */
	PRIO_OUT_QZ = (PRIO_OUT_Q0 | PRIO_OUT_Q1 | PRIO_OUT_Q2)
}   f9318_out_t;

/**
 * @brief F9318 priority encoder 8 to 3-bit
 *
 * Emulation of the F9318 chip (pin compatible with 74348).
 *
 * <PRE>
 *            F9318
 *         +---+-+---+
 *         |   +-+   |         +---------------------------------+----------------+
 *    I4' -|1      16|-  Vcc   |              input              |     output     |
 *         |         |         +---------------------------------+----------------+
 *    I5' -|2      15|-  EO'   |      EI I0 I1 I2 I3 I4 I5 I6 I7 | GS Q0 Q1 Q2 EO |
 *         |         |         +---------------------------------+----------------+
 *    I6' -|3      14|-  GS'   | (a)  H  x  x  x  x  x  x  x  x  | H  H  H  H  H  |
 *         |         |         | (b)  L  H  H  H  H  H  H  H  H  | H  H  H  H  L  |
 *    I7' -|4      13|-  I3'   +---------------------------------+----------------+
 *         |         |         | (c)  L  x  x  x  x  x  x  x  L  | L  L  L  L  H  |
 *    EI' -|5      12|-  I2'   | (d)  L  x  x  x  x  x  x  L  H  | L  H  L  L  H  |
 *         |         |         | (e)  L  x  x  x  x  x  L  H  H  | L  L  H  L  H  |
 *    Q2' -|6      11|-  I1'   | (f)  L  x  x  x  x  L  H  H  H  | L  H  H  L  H  |
 *         |         |         | (g)  L  x  x  x  L  H  H  H  H  | L  L  L  H  H  |
 *    Q1' -|7      10|-  I0'   | (h)  L  x  x  L  H  H  H  H  H  | L  H  L  H  H  |
 *         |         |         | (i)  L  x  L  H  H  H  H  H  H  | L  L  H  H  H  |
 *   GND  -|8       9|-  Q0'   | (j)  L  L  H  H  H  H  H  H  H  | L  H  H  H  H  |
 *         |         |         +---------------------------------+----------------+
 *         +---------+
 * </PRE>
 */
static __inline f9318_out_t f9318(f9318_in_t in)
{
	int out;

	if (in & PRIO_IN_EI) {
		out = PRIO_OUT_EO | PRIO_OUT_GS | PRIO_OUT_QZ;
		return static_cast<f9318_out_t>(out);
	}

	if (0 == (in & PRIO_I7)) {
		out = PRIO_OUT_EO;
		return static_cast<f9318_out_t>(out);
	}

	if (PRIO_I7 == (in & PRIO_I6_I7)) {
		out = PRIO_OUT_EO | PRIO_OUT_Q0;
		return static_cast<f9318_out_t>(out);
	}

	if (PRIO_I6_I7 == (in & PRIO_I5_I7)) {
		out = PRIO_OUT_EO | PRIO_OUT_Q1;
		return static_cast<f9318_out_t>(out);
	}

	if (PRIO_I5_I7 == (in & PRIO_I4_I7)) {
		out = PRIO_OUT_EO | PRIO_OUT_Q0 | PRIO_OUT_Q1;
		return static_cast<f9318_out_t>(out);
	}

	if (PRIO_I4_I7 == (in & PRIO_I3_I7)) {
		out = PRIO_OUT_EO | PRIO_OUT_Q2;
		return static_cast<f9318_out_t>(out);
	}

	if (PRIO_I3_I7 == (in & PRIO_I2_I7)) {
		out = PRIO_OUT_EO | PRIO_OUT_Q0 | PRIO_OUT_Q2;
		return static_cast<f9318_out_t>(out);
	}

	if (PRIO_I2_I7 == (in & PRIO_I1_I7)) {
		out = PRIO_OUT_EO | PRIO_OUT_Q1 | PRIO_OUT_Q2;
		return static_cast<f9318_out_t>(out);
	}

	if (PRIO_I1_I7 == (in & PRIO_I0_I7)) {
		out = PRIO_OUT_EO | PRIO_OUT_Q0 | PRIO_OUT_Q1 | PRIO_OUT_Q2;
		return static_cast<f9318_out_t>(out);
	}

	out = PRIO_OUT_QZ | PRIO_OUT_GS;
	return static_cast<f9318_out_t>(out);
}

nabupc_state::nabupc_state(const machine_config &mconfig, device_type type, const char *tag)
	: driver_device(mconfig, type, tag)
	, m_maincpu(*this, "maincpu")
	, m_video_card(*this, "nabu_video")
	, m_ay8910(*this, "ay8910")
	, m_speaker(*this, "speaker")
	, m_kbduart(*this, "kbduart")
	, m_hccauart(*this, "hccauart")
	, m_ram(*this, RAM_TAG)
	, m_centronics(*this, "centronics")
	, m_bus(*this, "bus")
	, m_bios_sel(*this, "CONFIG")
	, m_leds(*this, "led%u", 0U)
	, m_irq_in_prio(0xFF)
	, m_int_lines(0)
	, m_porta(0)
	, m_portb(0)
	, m_control(0)
{

}

void nabupc_state::nabupc(machine_config &config)
{
	config.set_default_layout(layout_nabupc);

	// CPU
	Z80(config, m_maincpu, 10.738635_MHz_XTAL / 3);         // 3.579545 MHz
	m_maincpu->set_addrmap(AS_PROGRAM, &nabupc_state::memory_map);
	m_maincpu->set_addrmap(AS_IO, &nabupc_state::io_map);
	m_maincpu->set_irq_acknowledge_callback(FUNC(nabupc_state::int_ack_cb));

	// RAM
	RAM(config, RAM_TAG).set_default_size("64K");

	// Video hardware
	NABU_VIDEO_PORT(config, m_video_card, bus::nabu::video_card_devices, "tms9918a");
	m_video_card->int_callback().set(*this, FUNC(nabupc_state::vdp_int_w));

	// Sound hardware
	SPEAKER(config, m_speaker).front_center();

	AY8910(config, m_ay8910, 10.738635_MHz_XTAL / 6);
	m_ay8910->set_flags(AY8910_SINGLE_OUTPUT);
	m_ay8910->port_b_read_callback().set(FUNC(nabupc_state::psg_portb_r));
	m_ay8910->port_a_write_callback().set(FUNC(nabupc_state::psg_porta_w));
	m_ay8910->add_route(ALL_OUTPUTS, m_speaker, 0.3);

	// Keyboard
	I8251(config, m_kbduart, 10.738635_MHz_XTAL / 6);
	m_kbduart->rxrdy_handler().set(*this, FUNC(nabupc_state::rxrdy_w));

	rs232_port_device &kbd(RS232_PORT(config, "kbd", keyboard_devices, "nabu"));
	kbd.rxd_handler().set(m_kbduart, FUNC(i8251_device::write_rxd));

	// HCCA
	AY31015(config, m_hccauart);
	m_hccauart->set_auto_rdav(true);
	m_hccauart->write_dav_callback().set(FUNC(nabupc_state::hcca_dr_w));
	m_hccauart->write_tbmt_callback().set(FUNC(nabupc_state::hcca_tbre_w));
	m_hccauart->write_fe_callback().set(FUNC(nabupc_state::hcca_fe_w));
	m_hccauart->write_or_callback().set(FUNC(nabupc_state::hcca_oe_w));
	m_hccauart->write_so_callback().set(HCCA_TAG, FUNC(rs232_port_device::write_txd));

	rs232_port_device &hcca(RS232_PORT(config, HCCA_TAG, hcca_devices, "null_modem"));
	hcca.rxd_handler().set(m_hccauart, FUNC(ay31015_device::write_si));
	hcca.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(hcca_rs232_defaults));

	// Internet Adapter default socket if not set otherwise
	if (config.options().has_image_option("bitbanger")) {
		::image_option &bitb_imo(config.options().image_option("bitbanger"));
		const std::string &bitb_val(bitb_imo.value());

		if (bitb_val.empty()) {
			bitb_imo.specify("socket.127.0.0.1:5816", false);
		}
	}

	// Printer
	output_latch_device &prndata(OUTPUT_LATCH(config, "prndata"));
	CENTRONICS(config, m_centronics, centronics_devices, nullptr);
	m_centronics->set_output_latch(prndata);
	m_centronics->busy_handler().set(FUNC(nabupc_state::centronics_busy_handler));

	// Clocks
	clock_device &sclk(CLOCK(config, "sclk", 10.738635_MHz_XTAL / 96)); // 111.9 kHz Clock
	sclk.signal_handler().set(m_kbduart, FUNC(i8251_device::write_rxc));

	clock_device &pclk(CLOCK(config, "pclk", 10.738635_MHz_XTAL / 6));  // 1.79 Mhz Clock
	pclk.signal_handler().set(m_hccauart, FUNC(ay31015_device::write_rcp));
	pclk.signal_handler().append(m_hccauart, FUNC(ay31015_device::write_tcp));

	// Bus
	NABU_OPTION_BUS(config, m_bus, 10.738635_MHz_XTAL / 3);
	m_bus->out_int_callback<0>().set(FUNC(nabupc_state::j9_int_w));
	m_bus->out_int_callback<1>().set(FUNC(nabupc_state::j10_int_w));
	m_bus->out_int_callback<2>().set(FUNC(nabupc_state::j11_int_w));
	m_bus->out_int_callback<3>().set(FUNC(nabupc_state::j12_int_w));
	NABU_OPTION_BUS_SLOT(config, "option1", m_bus, 0, bus::nabu::option_bus_devices, nullptr);
	NABU_OPTION_BUS_SLOT(config, "option2", m_bus, 1, bus::nabu::option_bus_devices, nullptr);
	NABU_OPTION_BUS_SLOT(config, "option3", m_bus, 2, bus::nabu::option_bus_devices, nullptr);
	NABU_OPTION_BUS_SLOT(config, "option4", m_bus, 3, bus::nabu::option_bus_devices, nullptr);
}

void nabupc_state::machine_reset()
{
	m_irq_in_prio = 0xFF;
	m_int_lines = 0x40;
	m_porta = 0;
	m_portb = 0;
	m_control = 0;
	m_bios_size = m_bios_sel->read() == 1 ? 0x2000 : 0x1000;
	m_leds[0] = 1; // Power LED
}

void nabupc_state::machine_start()
{
	m_leds.resolve();

	m_hccauart->write_np(1);
	m_hccauart->write_nb2(1);
	m_hccauart->write_nb1(1);
	m_hccauart->write_eps(0);
	m_hccauart->write_tsb(0);
	m_hccauart->write_cs(1);
	m_hccauart->write_swe(0);

	save_item(NAME(m_irq_in_prio));
	save_item(NAME(m_int_lines));
	save_item(NAME(m_porta));
	save_item(NAME(m_portb));
	save_item(NAME(m_control));
	save_item(NAME(m_bios_size));
}

uint8_t nabupc_state::psg_portb_r()
{
	return m_portb;
}

void nabupc_state::psg_porta_w(uint8_t data)
{
	if (data != m_porta) {
		m_porta = data;
		update_irq();
	}
}

void nabupc_state::control_w(uint8_t data)
{
	m_control = data;
	m_centronics->write_strobe(BIT(m_control, 2));
	for (int i = 3 ; i < 6 ; ++i) {
		m_leds[6 - i] = BIT(m_control, i);
	}
}

void nabupc_state::centronics_busy_handler(uint8_t state)
{
	BIT_SET(m_portb, 4, state);
}

WRITE_LINE_MEMBER(nabupc_state::hcca_fe_w)
{
	BIT_SET(m_portb, 5, state);
}

WRITE_LINE_MEMBER(nabupc_state::hcca_oe_w)
{
	BIT_SET(m_portb, 6, state);
}

WRITE_LINE_MEMBER(nabupc_state::hcca_dr_w)
{
	BIT_SET(m_int_lines, 7, state);
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::hcca_tbre_w)
{
	BIT_SET(m_int_lines, 6, state);
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::rxrdy_w)
{
	BIT_SET(m_int_lines, 5, state);
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::vdp_int_w)
{
	BIT_SET(m_int_lines, 4, state);
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::j9_int_w)
{
	BIT_SET(m_int_lines, 3, !state);
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::j10_int_w)
{
	BIT_SET(m_int_lines, 2, !state);
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::j11_int_w)
{
	BIT_SET(m_int_lines, 1, !state);
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::j12_int_w)
{
	BIT_SET(m_int_lines, 0, !state);
	update_irq();
}

IRQ_CALLBACK_MEMBER(nabupc_state::int_ack_cb)
{
	uint32_t vector = m_portb  & 0xe;
	return vector;
}

void nabupc_state::update_irq()
{
	uint8_t interrupts = ~(m_porta & m_int_lines);
	m_irq_in_prio = (m_irq_in_prio & 0xff00) | interrupts;

	m_portb &= 0xf0;
	f9318_out_t out = f9318(static_cast<f9318_in_t>(m_irq_in_prio));
	m_portb |= ((out & 7) << 1);
	m_portb |= ((out >> 3) & 1);
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, !((out >> 4) & 1));
}

uint8_t nabupc_state::read_mem(offs_t offset)
{
	if (offset < m_bios_size && (m_control & 1) == 0) {
		uint8_t *rom = memregion("ipl")->base();
		return rom[offset];
	}
	return m_ram->read(offset);
}

void nabupc_state::memory_map(address_map &map)
{
	map(0x0000, 0xffff).r(FUNC(nabupc_state::read_mem)).w(m_ram, FUNC(ram_device::write));
}

void nabupc_state::io_map(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	map(0x00, 0x00).w(FUNC(nabupc_state::control_w));
	map(0x40, 0x40).r(m_ay8910, FUNC(ay8910_device::data_r));
	map(0x40, 0x41).w(m_ay8910, FUNC(ay8910_device::data_address_w));
	map(0x80, 0x80).rw(m_hccauart, FUNC(ay31015_device::receive), FUNC(ay31015_device::transmit));
	map(0x90, 0x91).rw(m_kbduart, FUNC(i8251_device::read), FUNC(i8251_device::write));
	map(0xa0, 0xa1).rw(m_video_card, FUNC(bus::nabu::video_port_device::io_read), FUNC(bus::nabu::video_port_device::io_write));
	map(0xb0, 0xb0).w("prndata", FUNC(output_latch_device::write));
	map(0xc0, 0xcf).rw(m_bus, FUNC(bus::nabu::option_bus_device::read<0>), FUNC(bus::nabu::option_bus_device::write<0>));
	map(0xd0, 0xdf).rw(m_bus, FUNC(bus::nabu::option_bus_device::read<1>), FUNC(bus::nabu::option_bus_device::write<1>));
	map(0xe0, 0xef).rw(m_bus, FUNC(bus::nabu::option_bus_device::read<2>), FUNC(bus::nabu::option_bus_device::write<2>));
	map(0xf0, 0xff).rw(m_bus, FUNC(bus::nabu::option_bus_device::read<3>), FUNC(bus::nabu::option_bus_device::write<3>));
}

ROM_START( nabupc )
	ROM_REGION( 0x2000, "ipl", 0 )
	ROM_DEFAULT_BIOS("reva")
	ROM_SYSTEM_BIOS( 0, "reva", "4k BIOS" )
	ROMX_LOAD( "nabupc-u53-90020060-reva-2732.bin", 0x0000, 0x1000, CRC(8110bde0) SHA1(57e5f34645df06d7cb6c202a6d35a442776af2cb), ROM_BIOS(0) )
	ROM_SYSTEM_BIOS( 1, "ver14", "4k BIOS - Floppy support (V14)" )
	ROMX_LOAD( "nabupc-u53-ver14-2732.bin", 0x0000, 0x1000, CRC(ca5e1ae9) SHA1(d713abd5d387a63a287d4ea51196ba5de42db052), ROM_BIOS(1) )
	ROM_SYSTEM_BIOS( 2, "ver17", "4k BIOS - Floppy support (V17)" )
	ROMX_LOAD( "nabupc-u53-ver17-2732.bin", 0x0000, 0x1000, CRC(24d4f1fa) SHA1(1b9533c709604a21aa5fcc32071d0b0630e89c20), ROM_BIOS(2) )
	ROM_SYSTEM_BIOS( 3, "revb", "8k BIOS - Floppy support (Rev B)" )
	ROMX_LOAD( "nabupc-u53-90020060-revb-2764.bin", 0x0000, 0x2000, CRC(3088f21b) SHA1(bf2f1eb5d9f5a8e9d022ce0056f2a5a8526b830e), ROM_BIOS(3) )
	ROM_SYSTEM_BIOS( 4, "ver29", "8k BIOS - Floppy support (Ver 29)" )
	ROMX_LOAD( "nabupc-u53-90037150-ver29-2764.bin", 0x0000, 0x2000, CRC(3c484e3d) SHA1(dd10ad6e0a59c54561335272d3c808b0543ba0ef), ROM_BIOS(4) )
ROM_END

/***************************************************************************
    SYSTEM DRIVERS
***************************************************************************/

/*   YEAR  NAME        PARENT    COMPAT MACHINE     INPUT     CLASS      INIT        COMPANY       FULLNAME */
/* NABUPC */
COMP(1984, nabupc,   0,        0,     nabupc,   nabupc,      nabupc_state, empty_init, "NABU", "NABU PC", 0)

