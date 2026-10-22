// license:BSD-3-Clause
// copyright-holders:GTAMP
/*******************************************************************
 *
 * NABU PC RomWBW Option Card
 *
 * See romwbw.h for the description of the card.
 *
 * The MMU is the Zeta 2 style one that RomWBW calls MM_Z2.  With
 * paging disabled (the state after reset) the CPU address maps straight
 * onto the first 64K of the ROM.  With it enabled, each 16K window of
 * the CPU address space is backed by the physical 16K page held in its
 * register; the low 32 pages are the ROM and the high 32 the RAM.
 *
 * The IDE interface is an 82C55 in mode 0 wired as RomWBW's PPIDE
 * driver expects:
 *   port A  IDE data D0-D7          port B  IDE data D8-D15
 *   port C  bit 0-2  DA0-DA2        bit 3   CS0 (asserted when set)
 *           bit 4    CS1            bit 5   DIOW
 *           bit 6    DIOR           bit 7   RESET
 *   The control lines are inverted, so a set bit asserts the signal.
 *
 *******************************************************************/

#include "emu.h"
#include "romwbw.h"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(NABU_OPTION_ROMWBW, bus::nabu::romwbw_device, "nabupc_option_romwbw", "NABU PC RomWBW Card")

namespace bus::nabu {

namespace {

constexpr offs_t PORT_UART = 0x48;
constexpr offs_t PORT_PPI = 0x60;
constexpr offs_t PORT_MMU = 0x78;

constexpr uint8_t CTL_CS0 = 0x08;
constexpr uint8_t CTL_CS1 = 0x10;
constexpr uint8_t CTL_DIOW = 0x20;
constexpr uint8_t CTL_DIOR = 0x40;
constexpr uint8_t CTL_RESET = 0x80;

INPUT_PORTS_START( romwbw )
	PORT_START("MODE")
	PORT_CONFNAME( 0x01, 0x01, "Jumper" )
	PORT_CONFSETTING( 0x00, "NABU mode" )
	PORT_CONFSETTING( 0x01, "WBW mode" )
INPUT_PORTS_END

DEVICE_INPUT_DEFAULTS_START( terminal )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_38400 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_38400 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

ROM_START( romwbw )
	// RomWBW v3.6.0 NABU_std.rom, renamed to romwbw.rom
	ROM_REGION( 0x80000, "rom", ROMREGION_ERASEFF )
	ROM_LOAD_OPTIONAL( "romwbw.rom", 0x00000, 0x80000, CRC(e818816d) SHA1(290eab285cb8a39fc898e645da6ae074b7eb76ed) )
ROM_END

} // anonymous namespace

//-------------------------------------------------
//  romwbw_device - constructor
//-------------------------------------------------
romwbw_device::romwbw_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NABU_OPTION_ROMWBW, tag, owner, clock),
	device_option_expansion_interface(mconfig, *this),
	m_rom(*this, "rom"),
	m_mode(*this, "MODE"),
	m_uart(*this, "uart"),
	m_ppi(*this, "ppi"),
	m_ata(*this, "ata"),
	m_rbank(*this, "rbank%u", 0U),
	m_wbank(*this, "wbank%u", 0U),
	m_paging(false),
	m_wbw_mode(false),
	m_ppi_data(0),
	m_ide_data(0xffff),
	m_ppi_ctl(0)
{
	std::fill(std::begin(m_page), std::end(m_page), 0);
}

//-------------------------------------------------
//  device_add_mconfig - device-specific config
//-------------------------------------------------
void romwbw_device::device_add_mconfig(machine_config &config)
{
	// the UART clock comes from the 1.79MHz PCLK on the NABU bus
	NS16550(config, m_uart, clock() / 2);
	m_uart->out_tx_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	m_uart->out_dtr_callback().set("rs232", FUNC(rs232_port_device::write_dtr));
	m_uart->out_rts_callback().set("rs232", FUNC(rs232_port_device::write_rts));

	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, nullptr));
	rs232.rxd_handler().set(m_uart, FUNC(ins8250_uart_device::rx_w));
	rs232.dcd_handler().set(m_uart, FUNC(ins8250_uart_device::dcd_w));
	rs232.dsr_handler().set(m_uart, FUNC(ins8250_uart_device::dsr_w));
	rs232.ri_handler().set(m_uart, FUNC(ins8250_uart_device::ri_w));
	rs232.cts_handler().set(m_uart, FUNC(ins8250_uart_device::cts_w));
	rs232.set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(terminal));

	I8255(config, m_ppi);
	m_ppi->in_pa_callback().set(FUNC(romwbw_device::ppi_pa_r));
	m_ppi->in_pb_callback().set(FUNC(romwbw_device::ppi_pb_r));
	m_ppi->out_pa_callback().set(FUNC(romwbw_device::ppi_pa_w));
	m_ppi->out_pb_callback().set(FUNC(romwbw_device::ppi_pb_w));
	m_ppi->out_pc_callback().set(FUNC(romwbw_device::ppi_pc_w));

	ATA_INTERFACE(config, m_ata).options(ata_devices, "cf", nullptr, false);
}

const tiny_rom_entry *romwbw_device::device_rom_region() const
{
	return ROM_NAME( romwbw );
}

ioport_constructor romwbw_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( romwbw );
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
void romwbw_device::device_start()
{
	m_ram = std::make_unique<uint8_t []>(RAM_SIZE);
	std::fill_n(m_ram.get(), RAM_SIZE, 0);
	m_dummy = std::make_unique<uint8_t []>(0x4000);

	for (int i = 0; i < 4; i++) {
		m_rbank[i]->configure_entries(0, 32, m_rom->base(), 0x4000);
		m_rbank[i]->configure_entries(32, 32, m_ram.get(), 0x4000);
		m_wbank[i]->configure_entries(0, 32, m_dummy.get(), 0);
		m_wbank[i]->configure_entries(32, 32, m_ram.get(), 0x4000);
	}

	save_pointer(NAME(m_ram), RAM_SIZE);
	save_item(NAME(m_page));
	save_item(NAME(m_paging));
	save_item(NAME(m_wbw_mode));
	save_item(NAME(m_ppi_data));
	save_item(NAME(m_ide_data));
	save_item(NAME(m_ppi_ctl));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------
void romwbw_device::device_reset()
{
	m_wbw_mode = m_mode->read() != 0;
	// the paging enable flip-flop is cleared by /RESET, the page registers are not
	m_paging = false;
	m_ppi_ctl = 0;
	update_banks();
}

void romwbw_device::device_post_load()
{
	update_banks();
}

//-------------------------------------------------
//  slot window
//-------------------------------------------------
uint8_t romwbw_device::read(offs_t offset)
{
	return 0xff;
}

void romwbw_device::write(offs_t offset, uint8_t data)
{
}

//-------------------------------------------------
//  memory
//-------------------------------------------------
void romwbw_device::install_memory(memory_view::memory_view_entry &view)
{
	for (int i = 0; i < 4; i++) {
		const offs_t base = i * 0x4000;
		view.install_read_bank(base, base + 0x3fff, m_rbank[i]);
		view.install_write_bank(base, base + 0x3fff, m_wbank[i]);
	}
}

void romwbw_device::update_banks()
{
	for (int i = 0; i < 4; i++) {
		const int page = m_paging ? (m_page[i] & 0x3f) : i;
		m_rbank[i]->set_entry(page);
		m_wbank[i]->set_entry(page);
	}
}

//-------------------------------------------------
//  I/O
//-------------------------------------------------
void romwbw_device::install_io(address_space &space)
{
	space.install_device(0x00, 0xff, *this, &romwbw_device::io_map);
}

void romwbw_device::io_map(address_map &map)
{
	map(PORT_UART, PORT_UART + 7).rw(m_uart, FUNC(ns16550_device::ins8250_r), FUNC(ns16550_device::ins8250_w));
	map(PORT_PPI, PORT_PPI + 3).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(PORT_MMU, PORT_MMU + 3).w(FUNC(romwbw_device::page_w));
	map(PORT_MMU + 4, PORT_MMU + 4).w(FUNC(romwbw_device::paging_w));
}

void romwbw_device::page_w(offs_t offset, uint8_t data)
{
	m_page[offset] = data;
	update_banks();
}

void romwbw_device::paging_w(uint8_t data)
{
	m_paging = BIT(data, 0);
	update_banks();
}

//-------------------------------------------------
//  IDE
//-------------------------------------------------
void romwbw_device::ppi_pc_w(uint8_t data)
{
	const uint8_t old = m_ppi_ctl;
	m_ppi_ctl = data;

	const offs_t reg = data & 7;
	const uint16_t mask = reg ? 0x00ff : 0xffff;

	// a read strobe returns the addressed register on ports A and B
	if ((data & CTL_DIOR) && (!(old & CTL_DIOR) || ((old ^ data) & 0x1f))) {
		if (data & CTL_CS0) {
			m_ide_data = m_ata->cs0_r(reg, mask);
		} else if (data & CTL_CS1) {
			m_ide_data = m_ata->cs1_r(reg, mask);
		} else {
			m_ide_data = 0xffff;
		}
	}

	// a write completes when the write strobe is released
	if (!(data & CTL_DIOW) && (old & CTL_DIOW)) {
		if (data & CTL_CS0) {
			m_ata->cs0_w(reg, m_ppi_data, mask);
		} else if (data & CTL_CS1) {
			m_ata->cs1_w(reg, m_ppi_data, mask);
		}
	}

	// the drive is held in reset while the line is asserted
	if (data & CTL_RESET) {
		m_ata->reset();
	}
}

} // namespace bus::nabu
