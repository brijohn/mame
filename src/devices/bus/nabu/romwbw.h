// license:BSD-3-Clause
// copyright-holders:GTAMP
/*******************************************************************
 *
 * NABU PC RomWBW Option Card
 *
 * Les Bird's card (https://github.com/sebhc/sebhc/wiki/NABU).  It plugs
 * into an option slot and, through a cable to the Z80 socket, takes over
 * the memory bus from the motherboard while its jumper is in WBW mode.
 *
 *  - 512K flash ROM and 512K RAM behind a Zeta 2 style MMU: four 16K
 *    windows, each selecting one of 64 physical pages (0-31 ROM,
 *    32-63 RAM)
 *  - 16C550 UART with an RS232 port
 *  - 82C55 PPI wired as an IDE interface (compact flash)
 *
 *  I/O ports
 *    0x48-0x4f  16C550
 *    0x60-0x63  82C55 (A = IDE data low, B = IDE data high, C = control)
 *    0x78-0x7b  MMU page registers (write only)
 *    0x7c       MMU enable (bit 0, write only)
 *
 *******************************************************************/

#ifndef MAME_BUS_NABU_ROMWBW_H
#define MAME_BUS_NABU_ROMWBW_H

#pragma once

#include "option.h"

#include "bus/ata/atadev.h"
#include "bus/ata/ataintf.h"
#include "bus/rs232/rs232.h"
#include "machine/i8255.h"
#include "machine/ins8250.h"

namespace bus::nabu {

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class romwbw_device : public device_t, public device_option_expansion_interface
{
public:
	// construction/destruction
	romwbw_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// slot window (0xc0-0xcf and friends): the card has nothing there
	virtual uint8_t read(offs_t offset) override;
	virtual void write(offs_t offset, uint8_t data) override;

	// Z80 socket
	virtual bool cpu_socket() const override { return true; }
	virtual void install_memory(memory_view::memory_view_entry &view) override;
	virtual void install_io(address_space &space) override;
	virtual bool owns_memory() const override { return m_wbw_mode; }

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	static constexpr size_t ROM_SIZE = 0x80000;
	static constexpr size_t RAM_SIZE = 0x80000;

	void io_map(address_map &map) ATTR_COLD;
	void page_w(offs_t offset, uint8_t data);
	void paging_w(uint8_t data);
	void update_banks();

	void ppi_pa_w(uint8_t data) { m_ppi_data = (m_ppi_data & 0xff00) | data; }
	void ppi_pb_w(uint8_t data) { m_ppi_data = (m_ppi_data & 0x00ff) | (uint16_t(data) << 8); }
	uint8_t ppi_pa_r() { return uint8_t(m_ide_data); }
	uint8_t ppi_pb_r() { return uint8_t(m_ide_data >> 8); }
	void ppi_pc_w(uint8_t data);

	required_memory_region m_rom;
	required_ioport m_mode;
	required_device<ns16550_device> m_uart;
	required_device<i8255_device> m_ppi;
	required_device<ata_interface_device> m_ata;
	memory_bank_array_creator<4> m_rbank;
	memory_bank_array_creator<4> m_wbank;

	std::unique_ptr<uint8_t []> m_ram;
	std::unique_ptr<uint8_t []> m_dummy;
	uint8_t m_page[4];
	bool m_paging;
	bool m_wbw_mode;

	uint16_t m_ppi_data;   // written by the CPU through ports A and B
	uint16_t m_ide_data;   // read from the drive, returned through ports A and B
	uint8_t m_ppi_ctl;     // last value written to port C
};

} // namespace bus::nabu

// device type definition
DECLARE_DEVICE_TYPE_NS(NABU_OPTION_ROMWBW, bus::nabu, romwbw_device)

#endif // MAME_BUS_NABU_ROMWBW_H
