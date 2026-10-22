// license:BSD-3-Clause
// copyright-holders:Brian Johnson


#ifndef MAME_NABU_NABUPC_H
#define MAME_NABU_NABUPC_H

#pragma once

#include "bus/centronics/ctronics.h"
#include "bus/nabu/option.h"
#include "bus/nabu/video/video.h"
#include "cpu/z80/z80.h"
#include "sound/ay8910.h"
#include "speaker.h"
#include "machine/ay31015.h"
#include "machine/i8251.h"
#include "machine/ram.h"


class nabupc_state : public driver_device
{
public:
	nabupc_state(const machine_config &mconfig, device_type type, const char *tag);

	void nabupc(machine_config &config);

	virtual void machine_start() override;
	virtual void machine_reset() override;

private:
	void memory_map(address_map &map);
	void io_map(address_map &map);

	image_init_result quickload_cb(device_image_interface &image);

	uint8_t read_mem(offs_t offset);

	uint8_t psg_portb_r();
	void psg_porta_w(uint8_t data);
	void control_w(uint8_t data);
	void centronics_busy_handler(uint8_t state);
	void update_irq();

	DECLARE_WRITE_LINE_MEMBER(rxrdy_w);
	DECLARE_WRITE_LINE_MEMBER(vdp_int_w);
	DECLARE_WRITE_LINE_MEMBER(hcca_dr_w);
	DECLARE_WRITE_LINE_MEMBER(hcca_tbre_w);
	DECLARE_WRITE_LINE_MEMBER(hcca_fe_w);
	DECLARE_WRITE_LINE_MEMBER(hcca_oe_w);

	DECLARE_WRITE_LINE_MEMBER(j9_int_w);
	DECLARE_WRITE_LINE_MEMBER(j10_int_w);
	DECLARE_WRITE_LINE_MEMBER(j11_int_w);
	DECLARE_WRITE_LINE_MEMBER(j12_int_w);

	IRQ_CALLBACK_MEMBER(int_ack_cb);

	required_device<z80_device> m_maincpu;
	required_device<bus::nabu::video_port_device> m_video_card;
	required_device<ay8910_device> m_ay8910;
	required_device<speaker_device> m_speaker;
	required_device<i8251_device> m_kbduart;
	required_device<ay31015_device> m_hccauart;
	required_device<ram_device> m_ram;
	required_device<centronics_device> m_centronics;
	required_device<bus::nabu::option_bus_device> m_bus;
	memory_view m_mem_view;

	output_finder<4> m_leds;

	uint16_t m_irq_in_prio;
	uint8_t m_int_lines;
	uint8_t m_porta;
	uint8_t m_portb;
	uint8_t m_control;
	uint16_t m_bios_size;
};

#endif
