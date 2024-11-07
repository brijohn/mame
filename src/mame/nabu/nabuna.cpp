// license:BSD-3-Clause
// copyright-holders:Brian Johnson
/*******************************************************************
 *
 * NABU Network Adaptor (model NA-2)
 *
 *******************************************************************/

#include "emu.h"
#include "bus/nabu/na2.h"
#include "bus/rs232/null_modem.h"
#include "bus/rs232/pty.h"
#include "bus/rs232/rs232.h"

#include "nabuna.lh"

//**************************************************************************
//  DRIVER STATE
//**************************************************************************

class nabuna_state : public driver_device
{
public:
	nabuna_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_na2(*this, "na2")
	{ }

	void nabuna(machine_config &config);

private:
	required_device<bus::nabu::na2_device> m_na2;
};

static DEVICE_INPUT_DEFAULTS_START( npc_rs232_defaults )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_111900 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_111900 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
	DEVICE_INPUT_DEFAULTS( "FLOW_CONTROL", 0x07, 0x00 )
DEVICE_INPUT_DEFAULTS_END

static void npc_devices(device_slot_interface &device)
{
	device.option_add("pty",        PSEUDO_TERMINAL);
	device.option_add("null_modem", NULL_MODEM);
}

void nabuna_state::nabuna(machine_config &config)
{
	config.set_default_layout(layout_nabuna);

	NA2(config, m_na2);

	rs232_port_device &npc(RS232_PORT(config, "npc", npc_devices, "null_modem"));
	m_na2->txd_handler().set(npc, FUNC(rs232_port_device::write_txd));
	npc.rxd_handler().set(m_na2, FUNC(bus::nabu::na2_device::uart_rxd_w));
	npc.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(npc_rs232_defaults));
	npc.set_option_device_input_defaults("pty", DEVICE_INPUT_DEFAULTS_NAME(npc_rs232_defaults));
}

ROM_START(nabuna)
ROM_END

/*   YEAR  NAME      PARENT  COMPAT MACHINE   INPUT   CLASS         INIT        COMPANY  FULLNAME */
SYST(1983, nabuna,   0,      0,     nabuna,   0,      nabuna_state, empty_init, "NABU",  "NABU Network Adaptor NA-2", MACHINE_NO_SOUND_HW)
