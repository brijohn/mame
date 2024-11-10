// license:BSD-3-Clause
// copyright-holders:Brian Johnson

#include "emu.h"
#include "paddle.h"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(NABU_PADDLE, bus::nabu::ctrl::nabu_paddle_device, "nabu_paddle", "NABU paddles")

namespace bus::nabu::ctrl {

static INPUT_PORTS_START( nabu_paddles )
	PORT_START("JOYSTICK")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)         PORT_CODE(INPUT_CODE_INVALID)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)        PORT_CODE(INPUT_CODE_INVALID)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("POTX")
	PORT_BIT( 0xff, 0x80, IPT_PADDLE) PORT_PLAYER(1) PORT_SENSITIVITY(30) PORT_KEYDELTA(20) PORT_MINMAX(0, 255) PORT_CODE(INPUT_CODE_INVALID) PORT_CODE_DEC(INPUT_CODE_INVALID) PORT_CODE_INC(INPUT_CODE_INVALID) // pin 9

	PORT_START("POTY")
	PORT_BIT( 0xff, 0x80, IPT_PADDLE) PORT_PLAYER(2) PORT_SENSITIVITY(30) PORT_KEYDELTA(20) PORT_MINMAX(0, 255) PORT_CODE(INPUT_CODE_INVALID) PORT_CODE_DEC(INPUT_CODE_INVALID) PORT_CODE_INC(INPUT_CODE_INVALID) // pin 5
INPUT_PORTS_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor nabu_paddle_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( nabu_paddles );
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  nabu_paddle_device - constructor
//-------------------------------------------------

nabu_paddle_device::nabu_paddle_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NABU_PADDLE, tag, owner, clock),
	device_vcs_control_port_interface(mconfig, *this),
	m_joy(*this, "JOYSTICK"),
	m_potx(*this, "POTX"),
	m_poty(*this, "POTY")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void nabu_paddle_device::device_start()
{
}


//-------------------------------------------------
//  joy_r - joystick read
//-------------------------------------------------

uint8_t nabu_paddle_device::vcs_joy_r()
{
	return m_joy->read();
}


//-------------------------------------------------
//  potx_r - potentiometer X read
//-------------------------------------------------

uint8_t nabu_paddle_device::vcs_pot_x_r()
{
	return m_potx->read();
}


//-------------------------------------------------
//  poty_r - potentiometer Y read
//-------------------------------------------------

uint8_t nabu_paddle_device::vcs_pot_y_r()
{
	return m_poty->read();
}

} // bus::nabu::ctrl
