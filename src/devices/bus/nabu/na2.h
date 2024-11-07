// license:BSD-3-Clause
// copyright-holders:Brian Johnson
/*******************************************************************
 *
 * NABU Network Adaptor (model NA-2)
 *
 * Hardware-level emulation of the cable set-top box that feeds the
 * NABU PC.
 *
 *  - na2_device: the adaptor; 111,860 baud 8N1 serial to the NPC
 *  - palid_device: the addressing PAL's unit address
 *  - headend_device: simulated cable headend; carousels an npz pak
 *    archive or loops a cbs bitstream onto the line
 *  - na2_slot_device: RS-232 option for the NABU PC HCCA port
 *
 *******************************************************************/

#ifndef MAME_BUS_NABU_NA2_H
#define MAME_BUS_NABU_NA2_H

#pragma once

#include "bus/rs232/rs232.h"
#include "cpu/m6805/m68705.h"
#include "machine/ay31015.h"
#include "machine/gen_fifo.h"

namespace bus::nabu {

class palid_device : public device_t, public device_image_interface
{
public:
	palid_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	uint8_t address(unsigned index) const { return m_address[index & 3]; }

	// image-level overrides
	virtual bool is_readable()  const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return false; }
	virtual bool is_creatable() const noexcept override { return false; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual bool core_opens_image_file() const noexcept override { return false; }
	virtual const char *file_extensions() const noexcept override { return ""; }
	virtual const char *image_type_name() const noexcept override { return "pal_address"; }
	virtual const char *image_brief_type_name() const noexcept override { return "palid"; }
	virtual image_init_result call_load() override;
	virtual void call_unload() override;

protected:
	virtual void device_start() override;

private:
	static constexpr uint8_t DEFAULT_ADDRESS[4] = { 0x80, 0x01, 0x18, 0x87 };

	uint8_t m_address[4];
};

class headend_device : public device_t, public device_image_interface
{
public:
	headend_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <typename... T> void set_stream_callback(T &&... args) { m_stream_cb.set(std::forward<T>(args)...); }
	auto carrier_callback() { return m_carrier_cb.bind(); }

	// image-level overrides
	virtual bool is_readable()  const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return false; }
	virtual bool is_creatable() const noexcept override { return false; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual bool core_opens_image_file() const noexcept override { return true; }
	virtual const char *file_extensions() const noexcept override { return "npz,cbs"; }
	virtual const char *image_type_name() const noexcept override { return "headend_content"; }
	virtual const char *image_brief_type_name() const noexcept override { return "headend"; }
	virtual image_init_result call_load() override;
	virtual void call_unload() override;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	static constexpr uint32_t IDLE_GAP_BITS = 256 * 8;             // inter-frame flags
	static constexpr uint64_t MIN_STRIDE_BITS = 1'578'000;         // 250 ms; must exceed the NPC's ~150 ms re-arm
	static constexpr uint64_t GRANT_PERIOD_BITS = 4'734'000;       // 750 ms
	static constexpr uint32_t CONTROL_GAP_BITS = 131'072;          // ~20 ms after grant/time
	static constexpr uint64_t FILLER_SPAN_BITS = 140'000;          // one filler time frame + its gaps (~22ms)
	static constexpr size_t WHEEL_POOL = 192;                      // interleave width
	static constexpr int LEAD_FLAGS = 8;                           // flag bytes ahead of each frame
	static constexpr int TRAIL_FLAGS = 2;                          // flag bytes after each frame
	static constexpr uint32_t CHUNK_BITS = 128;                    // line bits per broadcast timer tick

	TIMER_CALLBACK_MEMBER(broadcast_tick);

	void build_wheel();
	void carousel_tick();
	std::vector<uint8_t> make_time_packet() const;
	void build_frame(const std::vector<uint8_t> &packet);
	uint8_t scramble_bit(int d);
	void emit_bits(const uint8_t *bits, uint32_t count);

	device_delegate<void (const uint8_t *, uint32_t)> m_stream_cb;
	devcb_write_line m_carrier_cb;

	emu_timer *m_broadcast_timer;

	std::vector<uint8_t> m_grant_packet;
	std::vector<uint8_t> m_cbs_data;
	uint32_t m_cbs_pos;

	std::vector<std::vector<uint8_t>> m_wheel_packets;
	std::vector<int32_t> m_wheel_order;
	uint32_t m_wheel_step;
	uint64_t m_bits_sent;
	uint64_t m_round_start_bits;
	uint64_t m_grant_sent_bits;
	bool m_time_pending;
	uint32_t m_idle_bits;
	uint64_t m_pad_until;
	uint8_t m_flag_phase;
	bool m_control_gap_pending;

	std::vector<uint8_t> m_frame;
	uint32_t m_frame_pos;
	uint32_t m_scram_sr;
};

class na2_device : public device_t
{
public:
	na2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	auto txd_handler() { return m_txd_cb.bind(); }
	void uart_rxd_w(int state) { m_uart->write_si(state); }

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	void uart_txd_w(int state) { m_txd_cb(state); }
	uint8_t porta_r();
	void porta_w(offs_t offset, uint8_t data, uint8_t mem_mask);
	void portb_w(offs_t offset, uint8_t data, uint8_t mem_mask);
	uint8_t portc_r();

	void cable_bits_w(const uint8_t *bits, uint32_t count);
	void cable_byte(uint8_t byte);
	uint8_t bus_r(uint8_t fallback);
	void irq_pulse();

	// devices
	required_device<m6805p2_device> m_maincpu;
	required_device<ay31015_device> m_uart;
	required_device<generic_fifo_u8_device> m_fifo;
	required_device<headend_device> m_headend;
	required_device<palid_device> m_palid;
	output_finder<4> m_leds;

	devcb_write_line m_txd_cb;

	uint8_t m_porta_pins;
	uint8_t m_porta_ddr;
	uint8_t m_portb_pins;
	uint8_t m_portb_ddr;
	uint8_t m_fifo_latch;
	uint8_t m_rx_latch;
	uint32_t m_pll_shift;
	bool m_rx_gate;

	uint32_t m_descram_sr;
	uint8_t m_flag_window;
	uint8_t m_rx_ones;
	uint8_t m_rx_byte;
	uint8_t m_rx_bit_count;
	bool m_in_frame;
	bool m_capture;             // current packet is being written to the FIFO
	uint8_t m_pal_byte_index;
	bool m_pal_unit;
	bool m_pal_bcast;
	uint8_t m_pal_status;
};

class na2_slot_device : public device_t, public device_rs232_port_interface
{
public:
	na2_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	virtual void input_txd(int state) override { m_na2->uart_rxd_w(state); }

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override { }
	virtual void device_reset() override { output_rxd(1); }

private:
	required_device<na2_device> m_na2;
};

} // namespace bus::nabu

DECLARE_DEVICE_TYPE_NS(NA2, bus::nabu, na2_device)
DECLARE_DEVICE_TYPE_NS(NABU_PALID, bus::nabu, palid_device)
DECLARE_DEVICE_TYPE_NS(NABU_HEADEND, bus::nabu, headend_device)
DECLARE_DEVICE_TYPE_NS(NABU_NETWORK_NA2_ADAPTER, bus::nabu, na2_slot_device)

#endif // MAME_BUS_NABU_NA2_H
