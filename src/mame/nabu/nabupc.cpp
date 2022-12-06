// license:BSD-3-Clause
// copyright-holders:Brian Johnson


#include "emu.h"

#include "nabupc.h"

#include "bus/nabu/keyboard/hlekeyboard.h"
#include "bus/nabu/keyboard/nabu_kbd.h"
#include "bus/rs232/null_modem.h"
#include "bus/rs232/pty.h"
#include "bus/rs232/rs232.h"
#include "machine/clock.h"

#include "nabupc.lh"

static INPUT_PORTS_START( nabupc )
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x03, 0x02, "BIOS Size" )
	PORT_CONFSETTING( 0x02, "4k BIOS" )
	PORT_CONFSETTING( 0x01, "8k BIOS" )
INPUT_PORTS_END

static void hcca_devices(device_slot_interface &device)
{
	device.option_add("pty",           PSEUDO_TERMINAL);
	device.option_add("null_modem",    NULL_MODEM);
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
	, m_tms9928a(*this, "tms9928a")
	, m_screen(*this, "screen")
	, m_ay8910(*this, "ay8910")
	, m_speaker(*this, "speaker")
	, m_kbduart(*this, "kbduart")
	, m_hccauart(*this, "hccauart")
	, m_ram(*this, RAM_TAG)
	, m_centronics(*this, "centronics")
	, m_bios_sel(*this, "CONFIG")
	, m_leds(*this, "led%u", 0U)
	, m_irq_in_prio(0xFEFF)
	, m_hcca_dr(0)
	, m_hcca_tbre(0)
	, m_vdpint(0)
	, m_rxrdy(0)
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
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);

	TMS9918A(config, m_tms9928a, 10.738635_MHz_XTAL);
	m_tms9928a->set_screen(m_screen);
	m_tms9928a->set_vram_size(0x4000);
	m_tms9928a->int_callback().set(*this, FUNC(nabupc_state::vdp_int_w));

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
	m_hccauart->write_so_callback().set("hcca", FUNC(rs232_port_device::write_txd));

	rs232_port_device &hcca(RS232_PORT(config, "hcca", hcca_devices, "pty"));
	hcca.rxd_handler().set(m_hccauart, FUNC(ay31015_device::write_si));

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
}

void nabupc_state::machine_reset()
{
	m_irq_in_prio = 0xFEFF;
	m_hcca_dr = 0;
	m_hcca_tbre = 0;
	m_vdpint = 0;
	m_rxrdy = 0;
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
	save_item(NAME(m_hcca_dr));
	save_item(NAME(m_hcca_tbre));
	save_item(NAME(m_vdpint));
	save_item(NAME(m_rxrdy));
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
	m_portb = (m_portb & ~(0x10)) | (state << 4);
}

WRITE_LINE_MEMBER(nabupc_state::hcca_fe_w)
{
	m_portb = (m_portb & ~(0x20)) | (state << 5);
}

WRITE_LINE_MEMBER(nabupc_state::hcca_oe_w)
{
	m_portb = (m_portb & ~(0x40)) | (state << 6);
}

WRITE_LINE_MEMBER(nabupc_state::hcca_dr_w)
{
	m_hcca_dr = state;
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::hcca_tbre_w)
{
	m_hcca_tbre = state;
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::vdp_int_w)
{
	m_vdpint = state;
	update_irq();
}

WRITE_LINE_MEMBER(nabupc_state::rxrdy_w)
{
	m_rxrdy = state;
	update_irq();
}

IRQ_CALLBACK_MEMBER(nabupc_state::int_ack_cb)
{
	uint32_t vector = (m_portb << 4) & 0xe0;
	return vector;
}

void nabupc_state::update_irq()
{
	if ((m_porta & 0x10) && m_vdpint) {
		m_irq_in_prio &= ~PRIO_IN_I4;
	} else {
		m_irq_in_prio |= PRIO_IN_I4;
	}

	if ((m_porta & 0x20) && m_rxrdy) {
		m_irq_in_prio &= ~PRIO_IN_I5;
	} else {
		m_irq_in_prio |= PRIO_IN_I5;
	}

	if ((m_porta & 0x40) && m_hcca_tbre) {
		m_irq_in_prio &= ~PRIO_IN_I6;
	} else {
		m_irq_in_prio |= PRIO_IN_I6;
	}

	if ((m_porta & 0x80) && m_hcca_dr) {
		m_irq_in_prio &= ~PRIO_IN_I7;
	} else {
		m_irq_in_prio |= PRIO_IN_I7;
	}

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
	map(0xa0, 0xa1).rw(m_tms9928a, FUNC(tms9928a_device::read), FUNC(tms9928a_device::write));
	map(0xb0, 0xb0).w("prndata", FUNC(output_latch_device::write));
}

ROM_START( nabupc )
	ROM_REGION( 0x2000, "ipl", 0 )
	ROM_SYSTEM_BIOS( 0, "reva", "4k BIOS" )
	ROMX_LOAD( "nabupc-u53-90020060-reva-2732.bin", 0x0000, 0x1000, CRC(8110bde0) SHA1(57e5f34645df06d7cb6c202a6d35a442776af2cb), ROM_BIOS(0) )
	ROM_SYSTEM_BIOS( 1, "revb", "8k BIOS - Floppy support" )
	ROMX_LOAD( "nabupc-u53-90020060-revb-2764.bin", 0x0000, 0x2000, CRC(3088f21b) SHA1(bf2f1eb5d9f5a8e9d022ce0056f2a5a8526b830e), ROM_BIOS(1) )
ROM_END

/***************************************************************************
    SYSTEM DRIVERS
***************************************************************************/

/*   YEAR  NAME        PARENT    COMPAT MACHINE     INPUT     CLASS      INIT        COMPANY       FULLNAME */
/* NABUPC */
COMP(1984, nabupc,   0,        0,     nabupc,   nabupc,      nabupc_state, empty_init, "NABU", "NABU PC", 0)

