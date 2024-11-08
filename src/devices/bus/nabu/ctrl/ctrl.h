// license:BSD-3-Clause
// copyright-holders:Brian Johnson

#ifndef MAME_BUS_NABU_CTRL_H
#define MAME_BUS_NABU_CTRL_H

#pragma once

namespace bus::nabu::ctrl {

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class nabu_controller_port_device;

class device_nabu_controller_port_interface : public device_interface
{
public:
	virtual ~device_nabu_controller_port_interface() { }

	virtual uint8_t joy_r() { return 0xff; }
	virtual uint8_t potx_r() { return 0xff; }
	virtual uint8_t poty_r() { return 0xff; }

	virtual bool has_potx() { return false; }
	virtual bool has_poty() { return false; }

protected:
	device_nabu_controller_port_interface(const machine_config &mconfig, device_t &device);

	nabu_controller_port_device *m_port;
};

class nabu_controller_port_device : public device_t, public device_slot_interface
{
public:
	// construction/destruction
	template <typename T>
	nabu_controller_port_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, char const* dflt)
		: nabu_controller_port_device(mconfig, tag, owner)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}
	nabu_controller_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// computer interface

	uint8_t read_joy() { return exists() ? m_device->joy_r() : 0xff; }
	uint8_t read_potx() { return exists() ? m_device->potx_r() : 0xff; }
	uint8_t read_poty() { return exists() ? m_device->poty_r() : 0xff; }

	bool exists() { return m_device != nullptr; }
	bool has_potx() { return exists() && m_device->has_potx(); }
	bool has_poty() { return exists() && m_device->has_poty(); }
protected:
	// device-level overrides
	virtual void device_start() override;

	device_nabu_controller_port_interface *m_device;
};

void nabu_controller_port_devices(device_slot_interface &device);

} // namespace bus::nabu::ctrl

// device type definition
DECLARE_DEVICE_TYPE_NS(NABU_CONTROLLER_PORT, bus::nabu::ctrl, nabu_controller_port_device)

#endif
