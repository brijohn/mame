// license:BSD-3-Clause
// copyright-holders:Brian Johnson

#ifndef MAME_BUS_NABU_CTRL_JOYSTICK_H
#define MAME_BUS_NABU_CTRL_JOYSTICK_H

#pragma once

#include "bus/vcs_ctrl/ctrl.h"


namespace bus::nabu::ctrl {

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> nabu_joystick_device

class nabu_joystick_device : public device_t, public device_vcs_control_port_interface
{
public:
	// construction/destruction
	nabu_joystick_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device_nabu_controller_port_interface overrides
	virtual uint8_t vcs_joy_r() override;

protected:
	nabu_joystick_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

private:
	required_ioport m_joy;
};

// ======================> nabu_ext_joystick_device

class nabu_ext_joystick_device : public nabu_joystick_device
{
public:
	// construction/destruction
	nabu_ext_joystick_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device_nabu_controller_port_interface overrides
	virtual uint8_t vcs_pot_x_r() override;
	virtual uint8_t vcs_pot_y_r() override;

	virtual bool has_pot_x() override { return true; }
	virtual bool has_pot_y() override { return true; }

protected:
	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

private:
	required_ioport m_potx;
	required_ioport m_poty;
};

}

// device type definition
DECLARE_DEVICE_TYPE_NS(NABU_JOYSTICK, bus::nabu::ctrl, nabu_joystick_device)
DECLARE_DEVICE_TYPE_NS(NABU_EXT_JOYSTICK, bus::nabu::ctrl, nabu_ext_joystick_device)


#endif // MAME_BUS_NABU_CTRL_JOYSTICK_H

