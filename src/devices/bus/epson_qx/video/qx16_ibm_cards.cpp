// license:BSD-3-Clause
// copyright-holders:Brian Johnson
/***************************************************************************

    QX-16 APX-ICRT IBM CGA/MDA-compatible video card

    HD46505 CRTC + GAIBVA/GAIBVD gate arrays, 32K VRAM, 8K CG ROM.

***************************************************************************/

#include "emu.h"
#include "qx16_ibm_cards.h"

#include "screen.h"


#define SCREEN_TAG  ":screen"
#define VIDEO_CLOCK 16_MHz_XTAL


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(QX16_PCVIDEO, bus::epson_qx::video::qx16_pcvideo_device, "qx16_pcvideo", "QX-16 APX-ICRT Video Card (CGA/MDA)")


namespace bus::epson_qx::video {

namespace {

// CG ROM: 7x12 font rows 0-7 at 0000h, rows 8-15 at 0800h; 5x7 at 1000h; 7x7 at 1800h
ROM_START(qx16_pcvideo)
	ROM_REGION(0x2000, "chargen", 0)
	ROM_LOAD("icrt-chargen.bin", 0x0000, 0x2000, CRC(e2ef440c) SHA1(0e1663eedd50d4990dbc2a7c6497b27180b49582))
ROM_END

static const gfx_layout charlayout_8x16 =
{
	8, 16, 256, 1,
	{ 0 },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8, 2048*8, 2049*8, 2050*8, 2051*8, 2052*8, 2053*8, 2054*8, 2055*8 },
	8*8
};

static const gfx_layout charlayout_8x8 =
{
	8, 8, 256, 1,
	{ 0 },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8
};

static GFXDECODE_START(gfx_qx16_pcvideo)
	GFXDECODE_ENTRY("chargen", 0x0000, charlayout_8x16, 0, 1)
	GFXDECODE_ENTRY("chargen", 0x1000, charlayout_8x8, 0, 1)
	GFXDECODE_ENTRY("chargen", 0x1800, charlayout_8x8, 0, 1)
GFXDECODE_END

// pens 0-15 IRGB, 16-18 mono black / white / intense white
enum
{
	PEN_MONO_BLACK = 16,
	PEN_MONO_WHITE,
	PEN_MONO_INTENSE
};

} // anonymous namespace


//**************************************************************************
//  DEVICE
//**************************************************************************

qx16_pcvideo_device::qx16_pcvideo_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QX16_PCVIDEO, tag, owner, clock)
	, device_qx_video_interface(mconfig, *this)
	, m_crtc(*this, "mc6845")
	, m_screen(*this, SCREEN_TAG)
	, m_palette(*this, "palette")
	, m_chargen(*this, "chargen")
	, m_vram(*this, "vram", 0x8000, ENDIANNESS_LITTLE)
	, m_mode_control(0)
	, m_color_select(0)
	, m_cntr(0)
	, m_mapr(0)
	, m_vsync(0)
	, m_hsync(0)
	, m_de(0)
	, m_lpen_latch(0)
	, m_video_dot(0)
	, m_framecnt(0)
	, m_pipe{}
{
}

const tiny_rom_entry *qx16_pcvideo_device::device_rom_region() const
{
	return ROM_NAME(qx16_pcvideo);
}

void qx16_pcvideo_device::device_add_mconfig(machine_config &config)
{
	PALETTE(config, m_palette, FUNC(qx16_pcvideo_device::palette_init), 19);
	GFXDECODE(config, "gfxdecode", m_palette, gfx_qx16_pcvideo);

	HD6845S(config, m_crtc, VIDEO_CLOCK / 8);
	m_crtc->set_screen(SCREEN_TAG);
	m_crtc->set_show_border_area(false);
	m_crtc->set_char_width(8);
	m_crtc->set_update_row_callback(FUNC(qx16_pcvideo_device::crtc_update_row));
	m_crtc->out_hsync_callback().set(FUNC(qx16_pcvideo_device::hsync_changed));
	m_crtc->out_vsync_callback().set(FUNC(qx16_pcvideo_device::vsync_changed));
	m_crtc->out_de_callback().set(FUNC(qx16_pcvideo_device::de_changed));
}

void qx16_pcvideo_device::device_start()
{
	save_item(NAME(m_mode_control));
	save_item(NAME(m_color_select));
	save_item(NAME(m_cntr));
	save_item(NAME(m_mapr));
	save_item(NAME(m_vsync));
	save_item(NAME(m_hsync));
	save_item(NAME(m_de));
	save_item(NAME(m_lpen_latch));
	save_item(NAME(m_video_dot));
	save_item(NAME(m_framecnt));
}

void qx16_pcvideo_device::device_reset()
{
	m_mode_control = 0;
	m_color_select = 0;
	m_cntr = 0;
	m_mapr = 0;
	m_vsync = 0;
	m_hsync = 0;
	m_de = 0;
	m_lpen_latch = 0;
	m_video_dot = 0;
	m_framecnt = 0;
	update_pipeline();
	update_vram_mapping();
}

void qx16_pcvideo_device::device_post_load()
{
	update_pipeline();
	update_vram_mapping();
}

void qx16_pcvideo_device::palette_init(palette_device &palette) const
{
	for (int i = 0; i < 16; i++)
	{
		uint8_t const l = BIT(i, 3) ? 0xff : 0xaa;
		uint8_t const d = BIT(i, 3) ? 0x55 : 0x00;
		palette.set_pen_color(i, BIT(i, 2) ? l : d, BIT(i, 1) ? l : d, BIT(i, 0) ? l : d);
	}
	palette.set_pen_color(6, rgb_t(0xaa, 0x55, 0x00));
	palette.set_pen_color(PEN_MONO_BLACK, rgb_t::black());
	palette.set_pen_color(PEN_MONO_WHITE, rgb_t(0x00, 0xaa, 0x00));
	palette.set_pen_color(PEN_MONO_INTENSE, rgb_t(0x00, 0xff, 0x00));
}


//**************************************************************************
//  I/O
//**************************************************************************

void qx16_pcvideo_device::install_io(address_space &space)
{
	space.install_device(0x3b0, 0x3bf, *this, &qx16_pcvideo_device::mda_io_map);
	space.install_device(0x3d0, 0x3df, *this, &qx16_pcvideo_device::cga_io_map);
	space.install_device(0x3ce, 0x3cf, *this, &qx16_pcvideo_device::cntr_mapr_map);
}

void qx16_pcvideo_device::common_io_map(address_map &map)
{
	map(0x00, 0x00).mirror(0x06).w(m_crtc, FUNC(mc6845_device::address_w));
	map(0x01, 0x01).mirror(0x06).rw(m_crtc, FUNC(mc6845_device::register_r), FUNC(mc6845_device::register_w));
	map(0x08, 0x08).w(FUNC(qx16_pcvideo_device::mode_control_w));
	map(0x09, 0x09).w(FUNC(qx16_pcvideo_device::color_select_w));
}

void qx16_pcvideo_device::mda_io_map(address_map &map)
{
	common_io_map(map);
	map(0x0a, 0x0a).r(FUNC(qx16_pcvideo_device::mda_status_r));
	map(0x0b, 0x0b).w(FUNC(qx16_pcvideo_device::lpen_clear_w));
}

void qx16_pcvideo_device::cga_io_map(address_map &map)
{
	common_io_map(map);
	map(0x0a, 0x0a).r(FUNC(qx16_pcvideo_device::cga_status_r));
	map(0x0b, 0x0b).w(FUNC(qx16_pcvideo_device::lpen_clear_w));
	map(0x0c, 0x0c).w(FUNC(qx16_pcvideo_device::lpen_set_w));
}

void qx16_pcvideo_device::cntr_mapr_map(address_map &map)
{
	map(0x00, 0x00).w(FUNC(qx16_pcvideo_device::cntr_w));
	map(0x01, 0x01).w(FUNC(qx16_pcvideo_device::mapr_w));
}

// 3BAh: bit 0 = hsync, bit 3 = video dot (faked: toggles per read while
// display is active, so it reads as busy only inside the active area)
uint8_t qx16_pcvideo_device::mda_status_r()
{
	if (!machine().side_effects_disabled())
		m_video_dot ^= 1;
	return (m_de && m_video_dot ? 0x08 : 0x00) | m_hsync;
}

// 3DAh: bit 0 = display disabled, bit 1 = light pen latch, bit 3 = vsync
uint8_t qx16_pcvideo_device::cga_status_r()
{
	return (m_vsync ? 0x08 : 0x00) | (m_lpen_latch << 1) | (m_de ? 0x00 : 0x01);
}

void qx16_pcvideo_device::mode_control_w(uint8_t data)
{
	m_mode_control = data;
	update_pipeline();
}

void qx16_pcvideo_device::color_select_w(uint8_t data)
{
	m_color_select = data;
}

void qx16_pcvideo_device::cntr_w(uint8_t data)
{
	m_cntr = data;
	update_pipeline();
}

void qx16_pcvideo_device::mapr_w(uint8_t data)
{
	m_mapr = data;
	update_vram_mapping();
}

// MAPR: bit 7 = board enable, bit 6 = VRAM at B8000h, bit 5 = VRAM at B0000h
void qx16_pcvideo_device::update_vram_mapping()
{
	if (!m_slot->has_memspace())
		return;

	address_space &space = m_slot->memspace();
	space.unmap_readwrite(0xb0000, 0xbffff);

	if (!BIT(m_mapr, 7))
		return;

	if (BIT(m_mapr, 5))
		space.install_ram(0xb0000, 0xb7fff, &m_vram[0]);
	if (BIT(m_mapr, 6))
		space.install_ram(0xb8000, 0xbffff, &m_vram[0]);
}

void qx16_pcvideo_device::update_pipeline()
{
	pipeline &p = m_pipe;

	p.enable    = BIT(m_mode_control, 3);
	p.blink     = BIT(m_mode_control, 5);
	p.mono      = BIT(m_cntr, 1);
	p.underline = BIT(m_cntr, 2);
	p.g400      = !BIT(m_cntr, 5);
	p.vmode     = (m_cntr >> 5) & 0x07;

	if (BIT(m_mode_control, 1))
		p.kind = BIT(m_mode_control, 4) ? unpack::GFX_1BPP : unpack::GFX_2BPP;
	else
		p.kind = unpack::TEXT;

	if (BIT(m_cntr, 4))
	{
		p.font_base = BIT(m_cntr, 3) ? 0x1000 : 0x1800;   // 5x7 : 7x7
		p.font_rows = 8;
	}
	else
	{
		p.font_base = 0x0000;   // 7x12 in 8x16
		p.font_rows = 16;
	}

	m_crtc->set_unscaled_clock(VIDEO_CLOCK / (BIT(m_mode_control, 0) ? 8 : 16));
	m_crtc->set_hpixels_per_column(p.kind == unpack::GFX_1BPP ? 16 : 8);
}


//**************************************************************************
//  SYNC
//**************************************************************************

void qx16_pcvideo_device::hsync_changed(int state)
{
	m_hsync = state ? 1 : 0;
}

void qx16_pcvideo_device::vsync_changed(int state)
{
	m_vsync = state ? 1 : 0;
	if (state)
		m_framecnt++;
}

void qx16_pcvideo_device::de_changed(int state)
{
	m_de = state ? 1 : 0;
}


//**************************************************************************
//  RENDERING
//**************************************************************************

uint32_t qx16_pcvideo_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(rgb_t::black(), cliprect);
	m_crtc->screen_update(screen, bitmap, cliprect);
	return 0;
}

uint16_t qx16_pcvideo_device::vram_addr(uint16_t ma, uint8_t ra) const
{
	uint16_t const byte = ma << 1;

	if (m_pipe.kind == unpack::TEXT)
		return (byte & 0x3fff) | (BIT(m_cntr, 6) ? 0x4000 : 0x0000);

	uint16_t const bank = m_pipe.g400 ? (ra >> 1) & 1 : ra & 1;

	switch (m_pipe.vmode)
	{
	case 0: case 1: return (bank << 13) | (byte & 0x1fff);
	case 2: case 3: return 0x4000 | (bank << 13) | (byte & 0x1fff);
	case 6:         return ((ra & 3) << 13) | (byte & 0x1fff);
	default:        return (bank << 14) | (byte & 0x3fff);
	}
}

MC6845_UPDATE_ROW(qx16_pcvideo_device::crtc_update_row)
{
	rgb_t const *const palette = m_palette->palette()->entry_list_raw();
	uint32_t *p = &bitmap.pix(y);
	pipeline const &pipe = m_pipe;

	if (!pipe.enable)
	{
		rgb_t const border = palette[pipe.mono ? PEN_MONO_BLACK : (m_color_select & 0x0f)];
		std::fill_n(p, x_count * (pipe.kind == unpack::GFX_1BPP ? 16 : 8), border);
		return;
	}

	switch (pipe.kind)
	{
	case unpack::TEXT:
	{
		uint16_t const row = (pipe.font_rows == 16) ? ((ra & 0x08) ? 0x800 | (ra & 0x07) : ra) : (ra & 0x07);
		bool const last_row = (ra == pipe.font_rows - 1);

		for (int i = 0; i < x_count; i++)
		{
			uint16_t const addr = vram_addr(ma + i, ra);
			uint8_t const chr = m_vram[addr];
			uint8_t const attr = m_vram[addr + 1];
			uint8_t data = m_chargen[pipe.font_base + row + chr * 8];
			uint8_t fg, bg;

			if (pipe.mono)
			{
				bool const reverse = (attr & 0x77) == 0x70;
				fg = BIT(attr, 3) ? PEN_MONO_INTENSE : PEN_MONO_WHITE;
				bg = PEN_MONO_BLACK;
				if ((attr & 0x77) == 0x00)
					data = 0x00;
				if (pipe.underline && last_row && (attr & 0x07) == 0x01)
					data = 0xff;
				if (reverse)
					std::swap(fg, bg);
			}
			else
			{
				fg = attr & 0x0f;
				bg = pipe.blink ? (attr >> 4) & 0x07 : attr >> 4;
			}

			if (i == cursor_x)
			{
				if (m_framecnt & 0x08)
					data = 0xff;
			}
			else if (pipe.blink && BIT(attr, 7) && (m_framecnt & 0x10))
			{
				data = 0x00;
			}

			for (int b = 7; b >= 0; b--)
				*p++ = palette[BIT(data, b) ? fg : bg];
		}
		break;
	}

	case unpack::GFX_2BPP:
	{
		uint8_t pens[4];
		if (pipe.mono)
		{
			pens[0] = PEN_MONO_BLACK;
			pens[1] = pens[2] = pens[3] = BIT(m_color_select, 4) ? PEN_MONO_INTENSE : PEN_MONO_WHITE;
		}
		else
		{
			uint8_t const ib = (BIT(m_color_select, 4) << 3) | BIT(m_color_select, 5);
			pens[0] = m_color_select & 0x0f;
			pens[1] = ib | 0x2;
			pens[2] = ib | 0x4;
			pens[3] = ib | 0x6;
		}

		for (int i = 0; i < x_count; i++)
		{
			uint16_t const addr = vram_addr(ma + i, ra);
			uint16_t const data = (m_vram[addr] << 8) | m_vram[addr + 1];
			for (int s = 14; s >= 0; s -= 2)
				*p++ = palette[pens[(data >> s) & 0x03]];
		}
		break;
	}

	case unpack::GFX_1BPP:
	{
		uint8_t const fg = pipe.mono ? (BIT(m_color_select, 3) ? PEN_MONO_INTENSE : PEN_MONO_WHITE) : (m_color_select & 0x0f);
		uint8_t const bg = pipe.mono ? PEN_MONO_BLACK : 0;

		for (int i = 0; i < x_count; i++)
		{
			uint16_t const addr = vram_addr(ma + i, ra);
			uint16_t const data = (m_vram[addr] << 8) | m_vram[addr + 1];
			for (int b = 15; b >= 0; b--)
				*p++ = palette[BIT(data, b) ? fg : bg];
		}
		break;
	}
	}
}


//**************************************************************************
//  CARD OPTION LIST
//**************************************************************************

void qx16_ibm_video_cards(device_slot_interface &device)
{
	device.option_add("pcvideo", QX16_PCVIDEO);
}

} // namespace bus::epson_qx::video
