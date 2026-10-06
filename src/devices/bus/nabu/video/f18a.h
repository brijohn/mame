// license:BSD-3-Clause
// copyright-holders:GTAMP, Troy Schrapel
/***************************************************************************

    NABU PC F18A / PICO9918 / PICO9918 PRO display cards

    The VDP is rendered by pico9918-core (3rdparty/pico9918-core, MIT
    licence, Troy Schrapel), which answers as an F18A, a PICO9918 or a
    PICO9918 PRO depending on the chip the card asks it to be. The card code
    is adapted from marduk's vdp_bridge.c (MIT licence, Troy Schrapel), see
    f18a.cpp.

***************************************************************************/

#ifndef MAME_BUS_NABU_VIDEO_F18A_H
#define MAME_BUS_NABU_VIDEO_F18A_H

#pragma once

#include "emu.h"
#include "screen.h"
#include "video.h"

#include "pico9918.h"
#include "pico9918_frame.h"

namespace bus::nabu {

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class pico9918_card_base : public device_t, public video_card_interface, public device_nvram_interface
{
public:
	virtual uint8_t read(offs_t offset) override;
	virtual void write(offs_t offset, uint8_t data) override;

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

protected:
	pico9918_card_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, int chip);

	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_pre_save() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;

	// device_nvram_interface overrides (the PICO9918 settings block)
	virtual void nvram_default() override;
	virtual bool nvram_read(util::read_stream &file) override;
	virtual bool nvram_write(util::write_stream &file) override;

private:
	static constexpr unsigned H_VIRTUAL = 640;
	static constexpr unsigned V_OUTPUT = 480;
	static constexpr unsigned SCANLINES = 262;
	static constexpr unsigned CONFIG_BYTES = 256;

	TIMER_CALLBACK_MEMBER(line_tick);

	void virtual_line();
	void recompute_cadence();
	void configure();
	void seed();
	void update_int();
	bool has_config() const;
	uint8_t hw_version() const;

	static void config_saved_cb(pico9918_t *inst, uint8_t *live, uint8_t key, void *userdata);
	static void config_reload_cb(pico9918_t *inst, void *userdata);

	required_device<screen_device> m_screen;
	emu_timer *m_line_timer;

	pico9918_t *m_vdp;
	const int m_chip;

	bitmap_rgb32 m_bitmap;
	std::unique_ptr<uint32_t []> m_argb;      // 4096-entry BGR12 -> ARGB map
	PICO9918_FRAME_LINE_BUFFER(m_pixels, H_VIRTUAL); // one virtual line
	pico9918_scanline_params_t m_params;
	pico9918_frame_display_t m_display;
	pico9918_frame_geometry_t m_geometry;

	// the library's instance, as a flat copy that save states can hold (see device_pre_save)
	size_t m_state_size;
	std::unique_ptr<uint8_t []> m_state;
	int m_splash_offset;
	uint8_t m_splash_hide;

	uint8_t m_config[CONFIG_BYTES];
	uint8_t *m_device_config;

	unsigned m_line;
	unsigned m_lines_per_call;
	unsigned m_field_lines;
	int m_int_state;
};

class f18a_card_device : public pico9918_card_base
{
public:
	f18a_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

class pico9918_card_device : public pico9918_card_base
{
public:
	pico9918_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

class pico9918pro_card_device : public pico9918_card_base
{
public:
	pico9918pro_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

} // namespace bus::nabu

DECLARE_DEVICE_TYPE_NS(NABU_F18A_CARD, bus::nabu, f18a_card_device)
DECLARE_DEVICE_TYPE_NS(NABU_PICO9918_CARD, bus::nabu, pico9918_card_device)
DECLARE_DEVICE_TYPE_NS(NABU_PICO9918PRO_CARD, bus::nabu, pico9918pro_card_device)

#endif // MAME_BUS_NABU_VIDEO_F18A_H
