// license:BSD-3-Clause
// copyright-holders:Brian Johnson
/***************************************************************************

    QX-16 APX-ICRT IBM CGA/MDA-compatible video card

***************************************************************************/

#ifndef MAME_BUS_EPSON_QX_VIDEO_QX16_IBM_CARDS_H
#define MAME_BUS_EPSON_QX_VIDEO_QX16_IBM_CARDS_H

#pragma once

#include "video.h"

#include "video/mc6845.h"
#include "emupal.h"

namespace bus::epson_qx::video {

class qx16_pcvideo_device : public device_t, public device_qx_video_interface
{
public:
	qx16_pcvideo_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

	// device_qx_video_interface
	virtual void install_io(address_space &space) override ATTR_COLD;
	virtual uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect) override;
	virtual bool enabled() const override { return BIT(m_mapr, 7); }

private:
	enum class unpack : uint8_t
	{
		TEXT,
		GFX_2BPP,
		GFX_1BPP
	};

	struct pipeline
	{
		unpack   kind;
		bool     enable;      // MR ENA
		bool     blink;       // MR BLEN
		bool     mono;        // CNTR B/W
		bool     underline;   // CNTR ENU
		bool     g400;        // CNTR G400 (R9=3, lines doubled)
		uint8_t  vmode;       // CNTR bits 7-5 (VM0-VM7)
		uint16_t font_base;   // CG ROM offset of selected font
		uint8_t  font_rows;   // 8 or 16
	};

	void mda_io_map(address_map &map) ATTR_COLD;
	void cga_io_map(address_map &map) ATTR_COLD;
	void common_io_map(address_map &map) ATTR_COLD;
	void cntr_mapr_map(address_map &map) ATTR_COLD;
	void palette_init(palette_device &palette) const;

	uint8_t mda_status_r();
	uint8_t cga_status_r();
	void lpen_set_w(uint8_t data) { m_lpen_latch = 1; }
	void lpen_clear_w(uint8_t data) { m_lpen_latch = 0; }
	void mode_control_w(uint8_t data);
	void color_select_w(uint8_t data);
	void cntr_w(uint8_t data);
	void mapr_w(uint8_t data);
	void update_vram_mapping();
	void update_pipeline();

	uint16_t vram_addr(uint16_t ma, uint8_t ra) const;
	uint16_t glyph_row(uint8_t ra) const;
	bool video_dot();

	void hsync_changed(int state);
	void vsync_changed(int state);
	void de_changed(int state);

	MC6845_UPDATE_ROW(crtc_update_row);

	required_device<hd6845s_device> m_crtc;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_region_ptr<uint8_t> m_chargen;
	memory_share_creator<uint8_t> m_vram;

	uint8_t  m_mode_control;
	uint8_t  m_color_select;
	uint8_t  m_cntr;
	uint8_t  m_mapr;
	uint8_t  m_vsync;
	uint8_t  m_hsync;
	uint8_t  m_de;
	uint8_t  m_lpen_latch;
	int      m_framecnt;
	pipeline m_pipe;
};


void qx16_ibm_video_cards(device_slot_interface &device);

} // namespace bus::epson_qx::video


DECLARE_DEVICE_TYPE_NS(QX16_PCVIDEO, bus::epson_qx::video, qx16_pcvideo_device)

#endif // MAME_BUS_EPSON_QX_VIDEO_QX16_IBM_CARDS_H
