// license:BSD-3-Clause
// copyright-holders:Brian Johnson

#include "emu.h"
#include "ctrl.h"

#include "joystick.h"
#include "paddle.h"


//**************************************************************************
//  DEVICE DEFINITION
//**************************************************************************

DEFINE_DEVICE_TYPE(NABU_CONTROLLER_PORT, bus::nabu::ctrl::nabu_controller_port_device, "nabu_controller_port", "NABU controller port")

namespace bus::nabu::ctrl {


//**************************************************************************
//  CARD INTERFACE
//**************************************************************************

//-----------------------------------------------------
//  device_nabu_controller_port_interface - constructor
//-----------------------------------------------------

device_nabu_controller_port_interface::device_nabu_controller_port_interface(const machine_config &mconfig, device_t &device) :
	device_interface(device, "nabuctrl")
{
	m_port = dynamic_cast<nabu_controller_port_device *>(device.owner());
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  nabu_controller_port_device - constructor
//-------------------------------------------------

nabu_controller_port_device::nabu_controller_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NABU_CONTROLLER_PORT, tag, owner, clock),
	device_slot_interface(mconfig, *this), m_device(nullptr)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void nabu_controller_port_device::device_start()
{
	m_device = dynamic_cast<device_nabu_controller_port_interface *>(get_card_device());
}

void nabu_controller_port_devices(device_slot_interface &device)
{
	device.option_add("joystick", NABU_JOYSTICK);
	device.option_add("paddle", NABU_PADDLE);
}

}
