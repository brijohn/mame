// license:BSD-3-Clause
// copyright-holders:Brian Johnson

/*******************************************************************
 *
 * NABU PC Expansion emulation
 *
 *******************************************************************/
#include "emu.h"
#include "option.h"

#include "fdc.h"
#include "hdd.h"
#include "rs232.h"
#include "romwbw.h"

DEFINE_DEVICE_TYPE(NABU_OPTION_BUS_SLOT, bus::nabu::option_slot_device, "nabu_option_slot", "NABU PC Option slot")
DEFINE_DEVICE_TYPE(NABU_OPTION_BUS, bus::nabu::option_bus_device, "nabu_option_bus", "NABU PC Option Bus")

namespace bus::nabu {
//**************************************************************************
//  NABU SLOT DEVICE
//**************************************************************************

//-------------------------------------------------
//  option_bus_slot_device - constructor
//-------------------------------------------------
option_slot_device::option_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NABU_OPTION_BUS_SLOT, tag, owner, clock),
	device_single_card_slot_interface<device_option_expansion_interface>(mconfig, *this),
	m_bus(*this, finder_base::DUMMY_TAG),
	m_slot(-1)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
void option_slot_device::device_start()
{
	device_option_expansion_interface *const intf(get_card_device());
	if (intf) {
		intf->set_option_bus(*m_bus, m_slot);
	}
	m_bus->add_slot(*this);
}


uint8_t option_slot_device::io_read(offs_t offset)
{
	device_option_expansion_interface *const intf(get_card_device());
	if (intf) {
		return intf->read(offset);
	}
	return 0xFF;
}

void option_slot_device::io_write(offs_t offset, uint8_t data)
{
	device_option_expansion_interface *const intf(get_card_device());
	if (intf) {
		intf->write(offset, data);
	}
}

WRITE_LINE_MEMBER(option_slot_device::int_w) {  (*m_bus).set_int_line(state, m_slot); }


//**************************************************************************
//  NABU OPTION BUS DEVICE
//**************************************************************************

//-------------------------------------------------
//  option_bus_device - constructor
//-------------------------------------------------
option_bus_device::option_bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NABU_OPTION_BUS, tag, owner, clock),
	m_int_cb(*this)
{
}

void option_bus_device::add_slot(option_slot_device &slot)
{
	m_slot_list.push_back(&slot);
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
void option_bus_device::device_start()
{
	m_int_cb.resolve_all_safe();
}

//-------------------------------------------------
//  device_reset - device-specific startup
//-------------------------------------------------
void option_bus_device::device_reset()
{
}

device_option_expansion_interface *option_bus_device::socket_card(option_slot_device *slot)
{
	device_option_expansion_interface *const intf(slot->card());
	return (intf && intf->cpu_socket()) ? intf : nullptr;
}

void option_bus_device::install_socket(memory_view::memory_view_entry &view)
{
	for (option_slot_device *slot : m_slot_list) {
		if (device_option_expansion_interface *card = socket_card(slot)) {
			card->install_memory(view);
		}
	}
}

void option_bus_device::install_socket_io(address_space &space)
{
	for (option_slot_device *slot : m_slot_list) {
		if (device_option_expansion_interface *card = socket_card(slot)) {
			card->install_io(space);
		}
	}
}

bool option_bus_device::socket_owns_memory() const
{
	for (option_slot_device *slot : m_slot_list) {
		device_option_expansion_interface *const card = socket_card(slot);
		if (card && card->owns_memory()) {
			return true;
		}
	}
	return false;
}


//**************************************************************************
//  NABU OPTION CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  option_device_expansion_interface - constructor
//-------------------------------------------------
device_option_expansion_interface::device_option_expansion_interface(const machine_config &mconfig, device_t &device) :
	device_interface(device, "nabu_option_bus"),
	m_bus(nullptr),
	m_slot(-1)
{
}

//---------------------------------------------------
//  interface_pre_start - device-specific pre startup
//---------------------------------------------------
void device_option_expansion_interface::interface_pre_start()
{
	if (!m_bus || m_slot == -1)
		throw device_missing_dependencies();
}

//-------------------------------------------------
//  SLOT_INTERFACE( option_bus_devices )
//-------------------------------------------------

void option_bus_devices(device_slot_interface &device)
{
	device.option_add("fdc", NABU_OPTION_FDC);
	device.option_add("hdd", NABU_OPTION_HDD);
	device.option_add("rs232",  NABU_OPTION_RS232);
	device.option_add("romwbw", NABU_OPTION_ROMWBW);
}

}  // namespace bus::nabu
