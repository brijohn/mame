// license:BSD-3-Clause
// copyright-holders:GTAMP, Troy Schrapel
/***************************************************************************

    NABU PC F18A / PICO9918 / PICO9918 PRO display cards

    The scanline cadence, frame handling and settings block are adapted from
    marduk's vdp_bridge.c (https://github.com/visrealm/marduk), and several
    of the comments below come from it. Each card is one pico9918-core
    instance, stepped by a timer once per NTSC line (262 a frame), rendering
    640x480.

    The parts adapted from vdp_bridge.c are covered by its license:

    Copyright (c) 2026 Troy Schrapel.

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following condition:  The
    above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

***************************************************************************/

#include "emu.h"
#include "f18a.h"

#include "pico9918_config.h"
#include "gpu/gpu.h"
#include "overlay/diag.h"
#include "overlay/splash.h"

#include <cstring>

// the line pointer the core caches between scanlines (pico9918.c); it points into host memory
extern "C" const uint8_t *pico9918_cached_line_source;

// The version of the vendored pico9918-core (3rdparty/pico9918-core, project() version).
#define PICO9918_CORE_VER_MAJOR 1
#define PICO9918_CORE_VER_MINOR 3
#define PICO9918_CORE_VER_PATCH 0

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(NABU_F18A_CARD, bus::nabu::f18a_card_device, "nabu_f18a_display", "NABU F18A Display")
DEFINE_DEVICE_TYPE(NABU_PICO9918_CARD, bus::nabu::pico9918_card_device, "nabu_pico9918_display", "NABU PICO9918 Display")
DEFINE_DEVICE_TYPE(NABU_PICO9918PRO_CARD, bus::nabu::pico9918pro_card_device, "nabu_pico9918pro_display", "NABU PICO9918 PRO Display")

namespace bus::nabu {

namespace {

// The NABU is NTSC.
constexpr float FRAME_RATE = 60.0f;

// A real board reports its die temperature; an emulator has nothing to read.
constexpr float TEMPERATURE = 40.0f;

} // anonymous namespace

//**************************************************************************
//  PICO9918 CARD BASE
//**************************************************************************

pico9918_card_base::pico9918_card_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, int chip) :
	device_t(mconfig, type, tag, owner, clock),
	video_card_interface(mconfig, *this),
	device_nvram_interface(mconfig, *this, chip >= PICO9918_CHIP_PICO9918),
	m_screen(*this, "screen"),
	m_line_timer(nullptr),
	m_vdp(nullptr),
	m_chip(chip),
	m_state_size(0),
	m_splash_offset(0),
	m_splash_hide(0),
	m_device_config(nullptr),
	m_line(0),
	m_lines_per_call(1),
	m_field_lines(SCANLINES),
	m_int_state(0)
{
	std::fill(std::begin(m_config), std::end(m_config), 0);
}

void pico9918_card_base::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_refresh_hz(FRAME_RATE);
	m_screen->set_size(H_VIRTUAL, V_OUTPUT);
	m_screen->set_visarea_full();
	m_screen->set_screen_update(FUNC(pico9918_card_base::screen_update));
}

bool pico9918_card_base::has_config() const
{
	// Only the PICO9918 boards carry the config port.
	return m_chip >= PICO9918_CHIP_PICO9918;
}

uint8_t pico9918_card_base::hw_version() const
{
	// Board revision, major nibble then minor. The PRO is the v2.x board.
	return (m_chip == PICO9918_CHIP_PICO9918_PRO) ? 0x20 : 0x10;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void pico9918_card_base::device_start()
{
	m_vdp = pico9918_new();
	if (!m_vdp)
		fatalerror("%s: could not allocate the pico9918-core instance\n", tag());

	pico9918_t *const tms9918 = m_vdp;

	pico9918_gpu_init(tms9918);
	pico9918_gpu_set_config_save_callback(tms9918, &pico9918_card_base::config_saved_cb, this);
	pico9918_frame_set_config_reload_callback(tms9918, &pico9918_card_base::config_reload_cb, this);
	pico9918_set_chip(tms9918, static_cast<pico9918_chip_t>(m_chip));

	m_bitmap.resize(H_VIRTUAL, V_OUTPUT);
	m_bitmap.fill(rgb_t::black());

	m_argb = std::make_unique<uint32_t []>(4096);
	for (unsigned v = 0; v < 4096; ++v)
		m_argb[v] = 0xff000000U | pico9918_pixel_rgb888(static_cast<PICO9918_PIXEL_T>(v));

	pico9918_config_defaults(m_config);

	configure();

	m_line_timer = timer_alloc(FUNC(pico9918_card_base::line_tick), this);

	// Save states. The instance is one flat block, but it holds host pointers (the callbacks and
	// the line buffer) that mean nothing in another process, so it is copied into m_state when
	// saving and copied back, with those pointers set up again, after loading.
	m_state_size = pico9918_instance_size();
	m_state = std::make_unique<uint8_t []>(m_state_size);
	std::fill_n(m_state.get(), m_state_size, 0);

	save_pointer(NAME(m_state), m_state_size);
	save_item(NAME(m_splash_offset));
	save_item(NAME(m_splash_hide));
	save_item(NAME(m_line));
	save_item(NAME(m_int_state));
	save_item(NAME(m_lines_per_call));
	save_item(NAME(m_field_lines));
	save_item(NAME(m_config));
	save_pointer(reinterpret_cast<uint8_t *>(&m_params), "m_params", sizeof(m_params));
	save_pointer(reinterpret_cast<uint8_t *>(&m_display), "m_display", sizeof(m_display));
	save_pointer(reinterpret_cast<uint8_t *>(&m_geometry), "m_geometry", sizeof(m_geometry));
	save_pointer(&m_bitmap.pix(0), "m_bitmap", size_t(m_bitmap.rowpixels()) * m_bitmap.height());
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void pico9918_card_base::device_reset()
{
	pico9918_t *const tms9918 = m_vdp;

	m_line = 0;

	// Clears VR56, so a program still running stops here.
	pico9918_reset(tms9918);
	pico9918_gpu_init(tms9918);

	// The diagnostics overlay is process-wide state, set up once.
	static bool diag_initialised = false;
	if (!diag_initialised)
	{
		pico9918_diag_init();
		diag_initialised = true;
	}

	const uint8_t hw = hw_version();
	const std::string firmware = util::string_format("%u.%u.%u", PICO9918_CORE_VER_MAJOR, PICO9918_CORE_VER_MINOR, PICO9918_CORE_VER_PATCH);
	const std::string hardware = util::string_format("%u.%u", hw >> 4, hw & 0x0f);
	pico9918_diag_set_version_info(hardware.c_str(), firmware.c_str());
	pico9918_diag_set_output_name("480P ", "@60");
	pico9918_diag_set_clock_hz(252000000.0f);

	seed();
	configure();

	pico9918_gpu_set_clock(tms9918, PICO9918_GPU_IPS_PRO);

	m_int_state = 0;
	m_slot->int_w(0);

	const attotime period = attotime::from_hz(FRAME_RATE * SCANLINES);
	m_line_timer->adjust(period, 0, period);
}

void pico9918_card_base::device_pre_save()
{
	if (m_vdp)
		std::memcpy(m_state.get(), m_vdp, m_state_size);

	// the splash animation is kept in the library's file statics, not in the instance
	bool hide = false;
	pico9918_splash_get_state(&m_splash_offset, &hide);
	m_splash_hide = hide ? 1 : 0;
}

void pico9918_card_base::device_post_load()
{
	pico9918_t *const tms9918 = m_vdp;

	std::memcpy(tms9918, m_state.get(), m_state_size);

	// the pointers that came back belong to the process that saved the state
	pico9918_gpu_set_config_save_callback(tms9918, &pico9918_card_base::config_saved_cb, this);
	pico9918_frame_set_config_reload_callback(tms9918, &pico9918_card_base::config_reload_cb, this);
	pico9918_cached_line_source = nullptr;
	m_device_config = pico9918_config(tms9918);
	pico9918_splash_set_state(m_splash_offset, m_splash_hide != 0);

	// the screen holds the last frame as it was saved; the next frame redraws it
}

void pico9918_card_base::device_stop()
{
	if (m_vdp)
	{
		pico9918_destroy(m_vdp);
		m_vdp = nullptr;
	}
	m_device_config = nullptr;
}

//-------------------------------------------------
//  settings block
//-------------------------------------------------

void pico9918_card_base::nvram_default()
{
	pico9918_config_defaults(m_config);
}

bool pico9918_card_base::nvram_read(util::read_stream &file)
{
	if (!has_config())
		return false;

	uint8_t block[CONFIG_BYTES];
	size_t actual;
	if (file.read(block, CONFIG_BYTES, actual) || actual != CONFIG_BYTES)
		return false;

	std::copy(std::begin(block), std::end(block), std::begin(m_config));
	if (pico9918_config_validate(m_config, hw_version()))
	{
		// Left set, it would re-save through the callback on every boot.
		m_config[PICO9918_CONF_SAVE_FORCED] = 0;
	}
	return true;
}

bool pico9918_card_base::nvram_write(util::write_stream &file)
{
	if (!has_config())
		return false;

	pico9918_config_prepare_save(m_config, hw_version());
	size_t actual;
	return !file.write(m_config, CONFIG_BYTES, actual) && actual == CONFIG_BYTES;
}

void pico9918_card_base::config_saved_cb(pico9918_t *inst, uint8_t *live, uint8_t key, void *userdata)
{
	pico9918_card_base &card = *static_cast<pico9918_card_base *>(userdata);
	constexpr unsigned first = PICO9918_CONFIG_FIRST_SETTABLE;
	constexpr unsigned count = PICO9918_CONFIG_BYTES - PICO9918_CONFIG_FIRST_SETTABLE;

	// Cancel means discard what the configurator staged, not persist it.
	if (key == PICO9918_CONF_PENDING_CANCEL)
		std::copy_n(&card.m_config[first], count, &live[first]);
	else
		std::copy_n(&live[first], count, &card.m_config[first]);
}

// Restores the settings the startup diagnostics screen takes away.
void pico9918_card_base::config_reload_cb(pico9918_t *inst, void *userdata)
{
	pico9918_card_base &card = *static_cast<pico9918_card_base *>(userdata);

	if (!card.m_device_config)
		return;

	std::copy_n(card.m_config, CONFIG_BYTES, card.m_device_config);
	pico9918_diag_config_updated(inst);
}

// The immediate apply consumes the mark set_chip leaves, so the first end of
// frame does not repeat it.
void pico9918_card_base::seed()
{
	pico9918_t *const tms9918 = m_vdp;

	m_device_config = pico9918_config(tms9918);
	std::copy_n(m_config, CONFIG_BYTES, m_device_config);

	pico9918_set_chip(tms9918, static_cast<pico9918_chip_t>(m_chip));
	pico9918_config_apply_now(tms9918, true);
	pico9918_diag_config_updated(tms9918);
}

//-------------------------------------------------
//  display
//-------------------------------------------------

// vPixelScale and vVirtualPixels are seeded here, then owned by the library.
void pico9918_card_base::configure()
{
	pico9918_t *const tms9918 = m_vdp;

	m_params.hVirtualPixels = H_VIRTUAL;
	m_params.interlaced = false;
	m_params.interlacedFieldOrder = 0;

	m_display.displayPixels = V_OUTPUT;
	m_display.interlaced = false;
	m_display.vPixelScale = 2;
	m_display.vVirtualPixels = V_OUTPUT / 2;

	m_geometry = pico9918_frame_geometry(tms9918, &m_display);
	recompute_cadence();
}

void pico9918_card_base::recompute_cadence()
{
	m_lines_per_call = (m_display.vPixelScale >= 2) ? 1 : 2;
	m_field_lines = SCANLINES * m_lines_per_call;
	m_params.vVirtualPixels = m_display.vVirtualPixels;
}

// One virtual (VGA) line, written out vPixelScale times.
void pico9918_card_base::virtual_line()
{
	pico9918_t *const tms9918 = m_vdp;

	if (m_line < m_params.vVirtualPixels)
	{
		const unsigned scale = m_display.vPixelScale ? m_display.vPixelScale : 1;

		for (unsigned rep = 0; rep < scale; ++rep)
		{
			const unsigned dy = m_line * scale + rep;
			if (dy >= V_OUTPUT)
				continue;

			uint32_t *const dest = &m_bitmap.pix(dy);
			// False means the buffer is untouched, so this row is still the last one's.
			if (pico9918_frame_output_line(tms9918, dy, &m_params, m_pixels))
			{
				for (unsigned x = 0; x < H_VIRTUAL; ++x)
					dest[x] = m_argb[m_pixels[x] & 0x0fff];
			}
			else if (dy > 0)
			{
				std::copy_n(&m_bitmap.pix(dy - 1), H_VIRTUAL, dest);
			}
		}

		// Inside the visible field: in row-30 modes the trigger sits at
		// vVirtualPixels, where end-of-frame raises it instead.
		if (m_line == m_geometry.triggerScanline)
			pico9918_frame_end_of_scanline(tms9918);
	}

	if (m_line == m_params.vVirtualPixels)
		pico9918_frame_porch(tms9918);

	if (++m_line >= m_field_lines)
	{
		m_line = 0;
		m_geometry = pico9918_frame_end(tms9918, TEMPERATURE, FRAME_RATE, &m_display);
		recompute_cadence();
	}
}

TIMER_CALLBACK_MEMBER(pico9918_card_base::line_tick)
{
	// Latched: a mode change inside the loop rewrites lines_per_call.
	const unsigned calls = m_lines_per_call;
	for (unsigned i = 0; i < calls; ++i)
		virtual_line();

	update_int();
}

// A locked device sends a write to R19 to R3 instead, so it can never arm the
// scanline source and R0 bit 4 stands for nothing there.
void pico9918_card_base::update_int()
{
	pico9918_t *const tms9918 = m_vdp;

	int state = pico9918_interrupt_status(tms9918) ? 1 : 0;
	if (state)
	{
		const bool enabled = (pico9918_reg_value(tms9918, TMS_REG_1) & TMS_R1_INT_ENABLE)
				|| (pico9918_unlocked(tms9918) && (pico9918_reg_value(tms9918, TMS_REG_0) & TMS_R0_INT_SCANLINE));
		if (!enabled)
			state = 0;
	}

	if (state != m_int_state)
	{
		m_int_state = state;
		m_slot->int_w(state);
	}
}

uint32_t pico9918_card_base::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	copybitmap(bitmap, m_bitmap, 0, 0, 0, 0, cliprect);
	return 0;
}

//-------------------------------------------------
//  bus access
//-------------------------------------------------

uint8_t pico9918_card_base::read(offs_t offset)
{
	pico9918_t *const tms9918 = m_vdp;
	uint8_t data;

	if (BIT(offset, 0))
	{
		data = pico9918_read_status(tms9918);
		update_int();
	}
	else
	{
		data = pico9918_read_data(tms9918);
	}
	return data;
}

void pico9918_card_base::write(offs_t offset, uint8_t data)
{
	pico9918_t *const tms9918 = m_vdp;

	if (BIT(offset, 0))
		pico9918_write_addr(tms9918, data);
	else
		pico9918_write_data(tms9918, data);
	update_int();
}

//**************************************************************************
//  CARDS
//**************************************************************************

f18a_card_device::f18a_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	pico9918_card_base(mconfig, NABU_F18A_CARD, tag, owner, clock, PICO9918_CHIP_F18A)
{
}

pico9918_card_device::pico9918_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	pico9918_card_base(mconfig, NABU_PICO9918_CARD, tag, owner, clock, PICO9918_CHIP_PICO9918)
{
}

pico9918pro_card_device::pico9918pro_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	pico9918_card_base(mconfig, NABU_PICO9918PRO_CARD, tag, owner, clock, PICO9918_CHIP_PICO9918_PRO)
{
}

} // namespace bus::nabu
