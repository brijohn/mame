// license:BSD-3-Clause
// copyright-holders:Brian Johnson

#ifndef MAME_BUS_NABU_CTRL_PADDLE_H
#define MAME_BUS_NABU_CTRL_PADDLE_H

#pragma once

#include "ctrl.h"


namespace bus::nabu::ctrl {

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> nabu_paddle_device

class nabu_paddle_device : public device_t, public device_nabu_controller_port_interface
{
public:
	// construction/destruction
	nabu_paddle_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device_nabu_controller_port_interface overrides
	virtual uint8_t joy_r() override;
	virtual uint8_t potx_r() override;
	virtual uint8_t poty_r() override;

	virtual bool has_potx() override { return true; }
	virtual bool has_poty() override { return true; }

protected:
	// device-level overrides
	virtual void device_start() override;

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

private:
	required_ioport m_joy;
	required_ioport m_potx;
	required_ioport m_poty;
};

} // namespace bus::nabu::ctrl

// device type definition
DECLARE_DEVICE_TYPE_NS(NABU_PADDLE, bus::nabu::ctrl, nabu_paddle_device)

#endif // MAME_BUS_NABU_CTRL_PADDLE_H
