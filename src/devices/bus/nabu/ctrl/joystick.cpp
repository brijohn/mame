// license:BSD-3-Clause
// copyright-holders:Brian Johnson

#include "emu.h"
#include "joystick.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(NABU_JOYSTICK, bus::nabu::ctrl::nabu_joystick_device, "nabu_joystick", "NABU Joystick")

namespace bus::nabu::ctrl {

static INPUT_PORTS_START( nabu_joystick )
	PORT_START("JOYSTICK")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )     PORT_CODE(INPUT_CODE_INVALID)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )     PORT_CODE(INPUT_CODE_INVALID)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )    PORT_CODE(INPUT_CODE_INVALID)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )       PORT_CODE(INPUT_CODE_INVALID)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON1 )           PORT_CODE(INPUT_CODE_INVALID)
INPUT_PORTS_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor nabu_joystick_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( nabu_joystick );
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  nabu_joystick_device - constructor
//-------------------------------------------------

nabu_joystick_device::nabu_joystick_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NABU_JOYSTICK, tag, owner, clock),
	device_nabu_controller_port_interface(mconfig, *this),
	m_joy(*this, "JOYSTICK")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void nabu_joystick_device::device_start()
{
}


//-------------------------------------------------
//  joy_r - joystick read
//-------------------------------------------------

uint8_t nabu_joystick_device::joy_r()
{
	return m_joy->read();
}

}
