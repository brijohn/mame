// license:BSD-3-Clause
// copyright-holders:Brian Johnson
/*
    NABU PC BUS

    1   /INT<N>
    2   /M1
    3   3.58MHz SYSCLK
    4   AUDIO<N>
    5   /CS<N>
    6   A0
    7   A1
    8   A2
    9   A3
    10  1.79MHz PCLK
    11  /WR
    12  /RD
    13  /IORQ
    14  /WAIT<N>
    15  /RESET
    16  D0
    17  D1
    18  D2
    19  D3
    20  D4
    21  D5
    22  D6
    23  D7
    24  +5V
    25  +5V
    26  GND
    27  GND
    28  GND
    29  +12V
    30  -12V

*/
#ifndef MAME_BUS_NABU_BUS_H
#define MAME_BUS_NABU_BUS_H

#pragma once

#include <vector>

namespace bus::nabu {

//**************************************************************************
//  FORWARD DECLARATIONS
//**************************************************************************

class option_bus_device;
class device_option_expansion_interface;


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

/* NABU PC Slot Device */

class option_slot_device : public device_t, public device_single_card_slot_interface<device_option_expansion_interface>
{
public:
	// construction/destruction
	template <typename T, typename U>
	option_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&bus_tag, int slot, U &&opts, const char *dflt)
		: option_slot_device(mconfig, tag, owner, bus_tag->clock())
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
		m_bus.set_tag(std::forward<T>(bus_tag));
		m_slot = slot;
	}
	option_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	uint8_t io_read(offs_t offset);
	void io_write(offs_t offset, uint8_t data);

	device_option_expansion_interface *card() { return get_card_device(); }

	DECLARE_WRITE_LINE_MEMBER(int_w);
protected:
	// device-level overrides
	virtual void device_start() override;

	// configuration
	required_device<option_bus_device> m_bus;
	int m_slot;
};


/* NABU PC Bus Device */

class option_bus_device :  public device_t
{
public:
	// construction/destruction
	option_bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <unsigned Slot> auto out_int_callback() { return m_int_cb[Slot].bind(); }

	template <unsigned Slot> uint8_t read(offs_t offset) {
		option_slot_device *slot = (*this)[Slot];
		return slot->io_read(offset);
	}

	template <unsigned Slot> void write(offs_t offset, uint8_t data) {
		option_slot_device *slot = (*this)[Slot];
		slot->io_write(offset, data);
	}

	void set_int_line(uint8_t state, uint8_t slot) { m_int_cb[slot](state); }

	void add_slot(option_slot_device &slot);
	option_slot_device* operator[](int index) const {assert(index < m_slot_list.size()); return m_slot_list[index]; }

	// Cards that plug into the Z80 socket (see device_option_expansion_interface::cpu_socket)
	// fill the socket's memory view and install their I/O ports into the CPU's I/O space.
	void install_socket(memory_view::memory_view_entry &view);
	void install_socket_io(address_space &space);
	bool socket_owns_memory() const;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	static device_option_expansion_interface *socket_card(option_slot_device *slot);

	devcb_write_line::array<5> m_int_cb;

	std::vector<option_slot_device *> m_slot_list;
};


/* NABU PC Option Card interface */
class device_option_expansion_interface : public device_interface
{
public:
	void set_option_bus(option_bus_device &bus, int slot) { assert(!device().started()); m_bus = &bus; m_slot = slot; }

	virtual uint8_t read(offs_t offset) = 0;
	virtual void write(offs_t offset, uint8_t data) = 0;

	// A card that also plugs into the Z80 socket (e.g. the RomWBW card) returns true here and
	// may then install its memory into the socket's view, take over the memory bus (returning
	// true from owns_memory) and install I/O ports anywhere in the 0x00-0xff range.
	virtual bool cpu_socket() const { return false; }
	virtual void install_memory(memory_view::memory_view_entry &view) { }
	virtual void install_io(address_space &space) { }
	virtual bool owns_memory() const { return false; }

protected:
	device_option_expansion_interface(const machine_config &mconfig, device_t &device);

	virtual void interface_pre_start() override;
	option_slot_device* get_slot() { return (*m_bus)[m_slot]; }

	option_bus_device  *m_bus;
	int m_slot;
};

void option_bus_devices(device_slot_interface &device);

} // namespace bus::nabu


// device type definition
DECLARE_DEVICE_TYPE_NS(NABU_OPTION_BUS_SLOT, bus::nabu, option_slot_device)
DECLARE_DEVICE_TYPE_NS(NABU_OPTION_BUS, bus::nabu, option_bus_device)

#endif
