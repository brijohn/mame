// license:BSD-3-Clause
// copyright-holders:Brian Johnson
/*******************************************************************
 *
 * NABU Network Adaptor (model NA-2)
 *
 * The cable set-top box that receives the NABU software broadcast
 * and feeds it to the NABU PC over a 111,860 baud 8N1 RS-422 link.
 *
 * Digital board:
 *  - MC6805P2 control processor, 2KB mask ROM
 *  - TR1865 UART to the NABU PC, hard strapped 8N1
 *  - 1KB packet FIFO: N8X60 FIFO controller + 2x 2114
 *  - PAL16R6 address detector; each unit carries a unique
 *    address equal to its PAL serial label, broadcast
 *    address FF FF FF FF
 *  - TTL descrambler / HDLC flag detect / destuff receive chain
 *  - RF can: MC145155 PLL tuner, OQPSK demodulator, 6.312 Mbit/s
 *
 * The MCU shares its port A bus with the UART and the FIFO output
 * latch:
 *  - PB0: receive arm - sets the RX-armed flip-flop
 *  - PB1: FIFO read strobe, active low
 *  - PB2: FIFO->UART transmit gate, active low
 *  - PB3: UART /DRR+RRD
 *  - PB4: MESSAGE LED
 *  - PB5: LINK LED
 *  - PB6: UART THRL strobe
 *  - PB7: PLL ENB latch pulse
 *  - PC0: UART DR, PC1: UART THRE
 *  - PC2/PC3: PAL status (11 = idle, 10 = receiving address,
 *    00 = address matched)
 *  - PA6/PA7: PLL clock/data
 *
 * Front panel LEDs:
 *  - POWER
 *  - CABLE (carrier detect)
 *  - LINK
 *  - MESSAGE
 *
 * IRQ sources - the hardware has exactly two, both FIFO occupancy
 * transitions:
 *  - the FIFO becoming non-empty
 *  - the FIFO going empty on a read strobe
 *
 *******************************************************************/

#include "emu.h"
#include "na2.h"
#include "cpu/m6805/m68705.h"
#include "machine/ay31015.h"
#include "machine/clock.h"
#include "machine/gen_fifo.h"

#include "unzip.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <deque>

#define LOG_PLL     (1U << 1)
#define LOG_PACKET  (1U << 2)

#define VERBOSE 0
#include "logmacro.h"

//**************************************************************************
//  NABU PC NETWORK ADAPTER DEVICE
//**************************************************************************

DEFINE_DEVICE_TYPE(NA2, bus::nabu::na2_device, "na2", "NABU NA-2")
DEFINE_DEVICE_TYPE(NABU_PALID, bus::nabu::palid_device, "nabu_palid", "NABU Adaptor Addressing PAL")
DEFINE_DEVICE_TYPE(NABU_HEADEND, bus::nabu::headend_device, "nabu_headend", "NABU Cable Headend")
DEFINE_DEVICE_TYPE(NABU_NETWORK_NA2_ADAPTER, bus::nabu::na2_slot_device, "nabu_net_na2_adapter", "NABU Network Adapter - NA2")

namespace bus::nabu {

// nominal cable bit rate (2x the 3.156 Msym/s OQPSK symbol rate)
constexpr uint32_t CABLE_BIT_RATE = 6'312'000;

// PAL status on PC3:PC2 (11 = idle, 10 = receiving address, 00 = matched)
constexpr uint8_t PAL_IDLE      = 3;
constexpr uint8_t PAL_RECEIVING = 2;
constexpr uint8_t PAL_MATCHED   = 0;

// length of the unit/broadcast address the PAL matches at the start of a packet
constexpr uint8_t PAL_ADDRESS_BYTES = 4;

// appends the Genibus FCS (CRC-16 poly 0x1021)
static void append_fcs(std::vector<uint8_t> &packet)
{
	uint16_t crc = 0xffff;
	for (uint8_t b : packet) {
		crc ^= uint16_t(b) << 8;
		for (int i = 0; i < 8; i++)
			crc = BIT(crc, 15) ? (crc << 1) ^ 0x1021 : (crc << 1);
	}
	crc ^= 0xffff;
	packet.push_back(crc >> 8);
	packet.push_back(crc & 0xff);
}

//**************************************************************************
//  ADDRESSING PAL UNIT ADDRESS
//**************************************************************************

palid_device::palid_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NABU_PALID, tag, owner, clock)
	, device_image_interface(mconfig, *this)
{
	std::copy(std::begin(DEFAULT_ADDRESS), std::end(DEFAULT_ADDRESS), m_address);
}

void palid_device::device_start()
{
	save_item(NAME(m_address));
}

image_init_result palid_device::call_load()
{
	const char *const name = filename();

	if (name == nullptr || strlen(name) != 8) {
		seterror(image_error::INVALIDIMAGE);
		return image_init_result::FAIL;
	}

	uint32_t value = 0;
	for (int i = 0; i < 8; i++) {
		char const c = name[i];
		int digit;
		if (c >= '0' && c <= '9')
			digit = c - '0';
		else if (c >= 'A' && c <= 'F')
			digit = c - 'A' + 10;
		else if (c >= 'a' && c <= 'f')
			digit = c - 'a' + 10;
		else {
			seterror(image_error::INVALIDIMAGE);
			return image_init_result::FAIL;
		}
		value = (value << 4) | digit;
	}

	m_address[0] = (value >> 24) & 0xff;
	m_address[1] = (value >> 16) & 0xff;
	m_address[2] = (value >> 8) & 0xff;
	m_address[3] = value & 0xff;
	return image_init_result::PASS;
}

void palid_device::call_unload()
{
	std::copy(std::begin(DEFAULT_ADDRESS), std::end(DEFAULT_ADDRESS), m_address);
}


//**************************************************************************
//  CABLE HEADEND
//**************************************************************************



headend_device::headend_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NABU_HEADEND, tag, owner, clock)
	, device_image_interface(mconfig, *this)
	, m_stream_cb(*this)
	, m_carrier_cb(*this)
	, m_broadcast_timer(nullptr)
	, m_cbs_pos(0)
	, m_wheel_step(0)
	, m_bits_sent(0)
	, m_round_start_bits(0)
	, m_grant_sent_bits(0)
	, m_time_pending(true)
	, m_idle_bits(0)
	, m_pad_until(0)
	, m_flag_phase(0)
	, m_control_gap_pending(false)
	, m_frame_pos(0)
	, m_scram_sr(0)
{
}

void headend_device::device_start()
{
	m_stream_cb.resolve();
	m_carrier_cb.resolve_safe();

	m_broadcast_timer = timer_alloc(FUNC(headend_device::broadcast_tick), this);

	// broadcast-addressed Set Tiers packet granting all AND tiers
	m_grant_packet = { 0xff, 0xff, 0xff, 0xff, 0x81, 0x7f, 0xff, 0xff, 0xff };
	m_grant_packet.resize(17, 0x00);
	append_fcs(m_grant_packet);

	save_item(NAME(m_wheel_step));
	save_item(NAME(m_bits_sent));
	save_item(NAME(m_round_start_bits));
	save_item(NAME(m_pad_until));
	save_item(NAME(m_grant_sent_bits));
	save_item(NAME(m_time_pending));
	save_item(NAME(m_idle_bits));
	save_item(NAME(m_flag_phase));
	save_item(NAME(m_control_gap_pending));
	save_item(NAME(m_scram_sr));
	save_item(NAME(m_frame));
	save_item(NAME(m_frame_pos));
	save_item(NAME(m_cbs_pos));
}

void headend_device::device_reset()
{
	m_scram_sr = 0;
	m_frame.clear();
	m_frame_pos = 0;
	m_cbs_pos = 0;
	m_wheel_step = 0;
	m_bits_sent = 0;
	m_round_start_bits = 0;
	m_pad_until = 0;
	m_grant_sent_bits = 0;
	m_time_pending = true;
	m_idle_bits = 0;
	m_flag_phase = 0;
	m_control_gap_pending = false;
	if (exists())
		m_broadcast_timer->adjust(attotime::from_msec(500));
	else
		m_broadcast_timer->reset();
	m_carrier_cb(exists() ? 1 : 0);
}

image_init_result headend_device::call_load()
{
	if (is_filetype("npz")) {
		build_wheel();
		if (m_wheel_order.empty()) {
			seterror(image_error::INVALIDIMAGE);
			return image_init_result::FAIL;
		}
		m_broadcast_timer->adjust(attotime::from_msec(500));
		m_carrier_cb(1);
		return image_init_result::PASS;
	}
	if (is_filetype("cbs")) {
		uint64_t const size = length();
		if (size == 0 || size > 0x4000000) {
			seterror(image_error::INVALIDIMAGE);
			return image_init_result::FAIL;
		}
		m_cbs_data.resize(size);
		if (fread(&m_cbs_data[0], size) != size) {
			m_cbs_data.clear();
			seterror(image_error::INVALIDIMAGE);
			return image_init_result::FAIL;
		}
		m_cbs_pos = 0;
		m_broadcast_timer->adjust(attotime::from_msec(500));
		m_carrier_cb(1);
		return image_init_result::PASS;
	}
	seterror(image_error::INVALIDIMAGE);
	return image_init_result::FAIL;
}

void headend_device::call_unload()
{
	m_frame.clear();
	m_frame_pos = 0;
	m_cbs_data.clear();
	m_cbs_pos = 0;
	m_wheel_packets.clear();
	m_wheel_order.clear();
	m_wheel_step = 0;
	m_idle_bits = 0;
	m_broadcast_timer->reset();
	m_carrier_cb(0);
}

void headend_device::build_wheel()
{
	m_wheel_packets.clear();
	m_wheel_order.clear();
	m_wheel_step = 0;

	if (!exists() || !is_filetype("npz"))
		return;

	util::core_file::ptr proxy;
	util::archive_file::ptr zipfile;
	if (util::core_file::open_proxy(image_core_file(), proxy))
		return;
	if (util::archive_file::open_zip(std::move(proxy), zipfile))
		return;

	std::vector<std::string> members;
	for (int index = zipfile->first_file(); index >= 0; index = zipfile->next_file()) {
		std::string name = zipfile->current_name();
		if (name.size() > 4) {
			std::string ext = name.substr(name.size() - 4);
			std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
			if (ext == ".pak")
				members.push_back(std::move(name));
		}
	}
	std::sort(members.begin(), members.end());

	std::vector<std::vector<int32_t>> segments;
	std::vector<uint8_t> data;
	for (auto const &name : members) {
		if (zipfile->search(name, false) < 0)
			continue;
		uint64_t const length = zipfile->current_uncompressed_length();
		if (length == 0 || length > 0x100000) {
			logerror("wheel: skipping %s (%u bytes)\n", name, uint32_t(length));
			continue;
		}
		data.resize(length);
		if (zipfile->decompress(&data[0], uint32_t(length))) {
			logerror("wheel: skipping %s (decompress failed)\n", name);
			continue;
		}
		segments.emplace_back();
		size_t offset = 0;
		while (offset + 2 <= data.size()) {
			size_t const block = data[offset] | (data[offset + 1] << 8);
			if (block < 18 || offset + 2 + block > data.size()) {
				logerror("wheel: %s: malformed block at offset %u, %u trailing bytes dropped\n",
						name, uint32_t(offset), uint32_t(data.size() - offset));
				break;
			}
			segments.back().push_back(int32_t(m_wheel_packets.size()));
			m_wheel_packets.emplace_back(data.begin() + offset + 2, data.begin() + offset + 2 + block);
			offset += 2 + block;
		}
	}

	std::deque<std::vector<int32_t>> queue;
	for (auto &seg : segments)
		if (!seg.empty())
			queue.emplace_back(std::move(seg));
	std::vector<std::deque<int32_t>> pool;
	while (true) {
		for (auto &seg : pool) {
			if (seg.empty() && !queue.empty()) {
				seg.assign(queue.front().begin(), queue.front().end());
				queue.pop_front();
			}
		}
		pool.erase(std::remove_if(pool.begin(), pool.end(),
				[] (auto const &seg) { return seg.empty(); }), pool.end());
		while (pool.size() < WHEEL_POOL && !queue.empty()) {
			pool.emplace_back(queue.front().begin(), queue.front().end());
			queue.pop_front();
		}
		if (pool.empty())
			break;
		for (auto &seg : pool) {
			m_wheel_order.push_back(seg.front());
			seg.pop_front();
		}
		m_wheel_order.push_back(-1);
	}

	LOGMASKED(LOG_PACKET, "Carousel wheel: %u packets, %u steps\n",
			uint32_t(m_wheel_packets.size()), uint32_t(m_wheel_order.size()));
}

uint8_t headend_device::scramble_bit(int d)
{
	int const s = (~(d ^ (m_scram_sr >> 2) ^ (m_scram_sr >> 19))) & 1;
	m_scram_sr = (m_scram_sr << 1) | s;
	return s;
}

void headend_device::emit_bits(const uint8_t *bits, uint32_t count)
{
	if (!m_stream_cb.isnull())
		m_stream_cb(bits, count);
	m_bits_sent += count;
	m_broadcast_timer->adjust(attotime::from_ticks(count, CABLE_BIT_RATE));
}

void headend_device::carousel_tick()
{
	if (m_frame_pos < m_frame.size()) {
		uint32_t const count = std::min<uint32_t>(CHUNK_BITS, m_frame.size() - m_frame_pos);
		emit_bits(&m_frame[m_frame_pos], count);
		m_frame_pos += count;
		if (m_frame_pos >= m_frame.size()) {
			m_idle_bits += IDLE_GAP_BITS + (m_control_gap_pending ? CONTROL_GAP_BITS : 0);
			m_control_gap_pending = false;
			m_flag_phase = 0;
		}
		return;
	}

	if (m_idle_bits != 0) {
		uint8_t bits[CHUNK_BITS];
		uint32_t const count = std::min<uint32_t>(CHUNK_BITS, m_idle_bits);
		for (uint32_t i = 0; i < count; i++) {
			bits[i] = scramble_bit(BIT(0x7e, 7 - m_flag_phase));
			m_flag_phase = (m_flag_phase + 1) & 7;
		}
		emit_bits(bits, count);
		m_idle_bits -= count;
		return;
	}

	if (m_flag_phase != 0) {
		m_idle_bits = 8 - m_flag_phase;
		m_broadcast_timer->adjust(attotime::zero);
		return;
	}

	if (m_bits_sent - m_grant_sent_bits >= GRANT_PERIOD_BITS) {
		m_grant_sent_bits = m_bits_sent;
		m_control_gap_pending = true;
		build_frame(m_grant_packet);
	} else if (m_bits_sent < m_pad_until) {
		if (m_pad_until - m_bits_sent > FILLER_SPAN_BITS) {
			m_control_gap_pending = true;
			build_frame(make_time_packet());
		} else {
			m_idle_bits = uint32_t(std::min<uint64_t>(512, ((m_pad_until - m_bits_sent) + 7) & ~uint64_t(7)));
			m_broadcast_timer->adjust(attotime::zero);
			return;
		}
	} else if (m_time_pending) {
		m_time_pending = false;
		m_control_gap_pending = true;
		build_frame(make_time_packet());
	} else {
		int32_t const step = m_wheel_order[m_wheel_step];
		m_wheel_step = (m_wheel_step + 1) % m_wheel_order.size();
		if (step < 0) {
			m_pad_until = std::max(m_round_start_bits + MIN_STRIDE_BITS, m_bits_sent);
			m_round_start_bits = m_pad_until;
			m_time_pending = true;
		} else {
			// PAL-addressed packets get the control-frame gap
			if (m_wheel_packets[step][0] & 0x80)
				m_control_gap_pending = true;
			build_frame(m_wheel_packets[step]);
		}
	}
	m_broadcast_timer->adjust(attotime::zero);
}

void headend_device::build_frame(const std::vector<uint8_t> &packet)
{
	if (packet.size() >= 4)
		LOGMASKED(LOG_PACKET, "frame @%.4f addr %02x%02x%02x%02x len %u\n",
				double(m_bits_sent) / CABLE_BIT_RATE,
				packet[0], packet[1], packet[2], packet[3], uint32_t(packet.size()));
	m_frame.clear();
	m_frame.reserve((LEAD_FLAGS + TRAIL_FLAGS) * 8 + packet.size() * 8 * 6 / 5);
	m_frame_pos = 0;
	uint8_t tx_ones = 0;

	auto line_bit = [this](int d) {
		m_frame.push_back(scramble_bit(d));
	};
	auto data_bit = [&line_bit, &tx_ones](int d) {
		line_bit(d);
		if (d) {
			if (++tx_ones == 5) {
				line_bit(0);
				tx_ones = 0;
			}
		} else {
			tx_ones = 0;
		}
	};
	auto flag = [&line_bit, &tx_ones]() {
		tx_ones = 0;
		for (int b = 7; b >= 0; b--)
			line_bit(BIT(0x7e, b));
	};

	for (int i = 0; i < LEAD_FLAGS; i++)
		flag();
	for (uint8_t byte : packet)
		for (int b = 7; b >= 0; b--)
			data_bit(BIT(byte, b));
	for (int i = 0; i < TRAIL_FLAGS; i++)
		flag();
}

TIMER_CALLBACK_MEMBER(headend_device::broadcast_tick)
{
	// cbs playback: loop the recorded line bits continuously
	if (!m_cbs_data.empty()) {
		uint64_t const total = uint64_t(m_cbs_data.size()) * 8;
		uint8_t bits[CHUNK_BITS];
		uint32_t const count = uint32_t(std::min<uint64_t>(CHUNK_BITS, total - m_cbs_pos));
		for (uint32_t i = 0; i < count; i++) {
			uint32_t const pos = m_cbs_pos + i;
			bits[i] = BIT(m_cbs_data[pos >> 3], 7 - (pos & 7));
		}
		emit_bits(bits, count);
		m_cbs_pos += count;
		if (m_cbs_pos >= total)
			m_cbs_pos = 0;
		return;
	}

	if (!m_wheel_order.empty())
		carousel_tick();
}

std::vector<uint8_t> headend_device::make_time_packet() const
{
	auto now = std::chrono::system_clock::now();
	auto tim = std::chrono::system_clock::to_time_t(now);
	auto localtime = std::localtime(&tim);

	std::vector<uint8_t> packet = {
		0x7f, 0xff, 0xff,           // segment address
		0x00,                       // packet number
		0x01,                       // owner
		0x7f, 0xff, 0xff, 0xff,     // tier
		0x7f, 0x80,                 // mystery bytes
		0xb1,                       // Packet Type Flags: first + last packet
		0x00, 0x00,                 // packet number, 16-bit
		0x00, 0x00,                 // offset
		0x02, 0x02,
		uint8_t(localtime->tm_wday + 1),
		0x54,                       // 1984 forever
		uint8_t(localtime->tm_mon + 1),
		uint8_t(localtime->tm_mday),
		uint8_t(localtime->tm_hour),
		uint8_t(localtime->tm_min),
		uint8_t(localtime->tm_sec)
	};

	append_fcs(packet);
	return packet;
}

//**************************************************************************
//  DEVICE STATE
//**************************************************************************

ROM_START( na2 )
	ROM_REGION( 0x800, "maincpu", 0 )
	ROM_LOAD( "u31nabuna.rom", 0x000, 0x800, CRC(d71ce929) SHA1(c5fdb48ef815768ea16df8b12a1fc2ea6e526c4a) )
ROM_END

na2_device::na2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NA2, tag, owner, clock)
	, m_maincpu(*this, "maincpu")
	, m_uart(*this, "uart")
	, m_fifo(*this, "fifo")
	, m_headend(*this, "headend")
	, m_palid(*this, "palid")
	, m_leds(*this, "na2_led%u", 0U)
	, m_txd_cb(*this)
{
}

void na2_device::device_add_mconfig(machine_config &config)
{
	M6805P2(config, m_maincpu, 3.579545_MHz_XTAL);
	m_maincpu->set_timer_divisor(4);
	m_maincpu->set_timer_external_source(true);
	m_maincpu->porta_r().set(FUNC(na2_device::porta_r));
	m_maincpu->porta_w().set(FUNC(na2_device::porta_w));
	m_maincpu->portb_r().set_constant(0xff);
	m_maincpu->portb_w().set(FUNC(na2_device::portb_w));
	m_maincpu->portc_r().set(FUNC(na2_device::portc_r));

	GENERIC_FIFO_U8(config, m_fifo, 0);

	AY31015(config, m_uart);
	m_uart->write_so_callback().set(FUNC(na2_device::uart_txd_w));

	NABU_HEADEND(config, m_headend, 0);
	m_headend->set_stream_callback(FUNC(na2_device::cable_bits_w));
	m_headend->carrier_callback().set_output("na2_led1");

	NABU_PALID(config, m_palid, 0);

	clock_device &uclk(CLOCK(config, "uclk", 3.579545_MHz_XTAL / 2));
	uclk.signal_handler().set(m_uart, FUNC(ay31015_device::write_rcp));
	uclk.signal_handler().append(m_uart, FUNC(ay31015_device::write_tcp));

	// RC oscillator on the MCU TIMER pin
	clock_device &timer(CLOCK(config, "timer", 1066));
	timer.signal_handler().set(m_maincpu, FUNC(m6805p2_device::timer_w));
}

void na2_device::device_start()
{
	m_txd_cb.resolve_safe();
	m_leds.resolve();

	m_fifo->setup(1024,
				  []() { },
				  []() { },
				  []() { },
				  []() { },
				  []() { },
				  []() { },
				  []() { });

	// TR1865: 8 data bits, no parity, 1 stop bit
	m_uart->write_np(1);
	m_uart->write_nb2(1);
	m_uart->write_nb1(1);
	m_uart->write_eps(0);
	m_uart->write_tsb(0);
	m_uart->write_swe(0);
	m_uart->write_cs(1);

	save_item(NAME(m_porta_pins));
	save_item(NAME(m_porta_ddr));
	save_item(NAME(m_portb_pins));
	save_item(NAME(m_portb_ddr));
	save_item(NAME(m_fifo_latch));
	save_item(NAME(m_rx_latch));
	save_item(NAME(m_pll_shift));
	save_item(NAME(m_rx_gate));
	save_item(NAME(m_descram_sr));
	save_item(NAME(m_flag_window));
	save_item(NAME(m_rx_ones));
	save_item(NAME(m_rx_byte));
	save_item(NAME(m_rx_bit_count));
	save_item(NAME(m_in_frame));
	save_item(NAME(m_capture));
	save_item(NAME(m_pal_byte_index));
	save_item(NAME(m_pal_unit));
	save_item(NAME(m_pal_bcast));
	save_item(NAME(m_pal_status));
}

void na2_device::device_reset()
{
	m_porta_pins = 0xff;
	m_porta_ddr = 0;
	m_portb_pins = 0xff;
	m_portb_ddr = 0;
	m_fifo_latch = 0xff;
	m_rx_latch = 0xff;
	m_pll_shift = 0;
	m_rx_gate = false;

	m_descram_sr = 0;
	m_flag_window = 0;
	m_rx_ones = 0;
	m_rx_byte = 0;
	m_rx_bit_count = 0;
	m_in_frame = false;
	m_capture = false;
	m_pal_byte_index = PAL_ADDRESS_BYTES;
	m_pal_unit = false;
	m_pal_bcast = false;
	m_pal_status = PAL_IDLE;

	m_uart->write_rdav(1);

	m_leds[0] = 1;  // POWER
	m_leds[2] = 0;  // LINK
	m_leds[3] = 0;  // MESSAGE
}

uint8_t na2_device::bus_r(uint8_t fallback)
{
	if (!BIT(m_portb_pins, 1))
		return m_fifo_latch;
	if (!BIT(m_portb_pins, 3))
		return m_rx_latch;
	return fallback;
}

uint8_t na2_device::porta_r()
{
	return bus_r(0xff);
}

void na2_device::porta_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	uint8_t const pins = (data & mem_mask) | (m_porta_pins & ~mem_mask);
	uint8_t const driven = mem_mask & m_porta_ddr;

	if (BIT(driven, 6) && BIT(pins, 6) && !BIT(m_porta_pins, 6))
		m_pll_shift = (m_pll_shift << 1) | BIT(pins, 7);

	m_porta_pins = pins;
	m_porta_ddr = mem_mask;
}

void na2_device::portb_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	uint8_t const pins = (data & mem_mask) | (m_portb_pins & ~mem_mask);
	uint8_t const driven = mem_mask & m_portb_ddr;
	uint8_t const rising = pins & ~m_portb_pins & driven;
	uint8_t const falling = ~pins & m_portb_pins & driven;

	m_portb_pins = pins;
	m_portb_ddr = mem_mask;

	if (BIT(rising, 0)) {
		m_rx_gate = true;
		m_pal_status = PAL_IDLE;
	}

	if (BIT(falling, 1)) {
		bool const was_empty = m_fifo->is_empty();
		if (!was_empty)
			m_fifo_latch = m_fifo->pop();
		if (!BIT(pins, 2))
			m_uart->transmit(m_fifo_latch);
		if (!was_empty && m_fifo->is_empty())
			irq_pulse();
	}

	if (BIT(falling, 3)) {
		m_rx_latch = m_uart->receive();
		m_uart->write_rdav(0);
	}
	if (BIT(rising, 3))
		m_uart->write_rdav(1);

	m_leds[3] = BIT(~pins, 4);
	m_leds[2] = BIT(~pins, 5);

	if (BIT(falling, 6))
		m_uart->transmit(bus_r(m_porta_pins));

	if (BIT(falling, 7)) {
		uint16_t const word = m_pll_shift & 0xffff;
		LOGMASKED(LOG_PLL, "PLL tuning word 0x%04X: N = %u, LO = %u MHz\n",
				word, word & 0x3fff, word & 0x3fff);
	}
}

uint8_t na2_device::portc_r()
{
	return (m_pal_status << 2) | (m_uart->tbmt_r() << 1) | m_uart->dav_r();
}

void na2_device::cable_bits_w(const uint8_t *bits, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		int const s = bits[i] & 1;

		// self-synchronizing descrambler: d = /s ^ s[n-3] ^ s[n-20]
		int const d = (~(s ^ (m_descram_sr >> 2) ^ (m_descram_sr >> 19))) & 1;
		m_descram_sr = (m_descram_sr << 1) | s;

		m_flag_window = (m_flag_window << 1) | d;
		if (m_flag_window == 0x7e) {
			m_in_frame = true;
			m_rx_bit_count = 0;
			m_rx_ones = 0;
			m_pal_byte_index = 0;
			m_capture = m_rx_gate;
			m_pal_unit = m_rx_gate;
			m_pal_bcast = m_rx_gate;
			if (m_rx_gate) {
				m_fifo->clear();
				m_pal_status = PAL_IDLE;
			}
			continue;
		}

		if (d == 0) {
			if (m_rx_ones == 5) {
				m_rx_ones = 0;
				continue;
			}
			m_rx_ones = 0;
		} else {
			m_rx_ones++;
		}

		if (!m_in_frame)
			continue;

		m_rx_byte = (m_rx_byte << 1) | d;
		if (++m_rx_bit_count == 8) {
			m_rx_bit_count = 0;
			cable_byte(m_rx_byte);
		}
	}
}

void na2_device::cable_byte(uint8_t byte)
{
	if (m_pal_byte_index < PAL_ADDRESS_BYTES) {
		if (m_pal_byte_index == 0 && m_pal_unit && BIT(byte, 7))
			m_pal_status = PAL_RECEIVING;
		m_pal_unit = m_pal_unit && (byte == m_palid->address(m_pal_byte_index));
		m_pal_bcast = m_pal_bcast && (byte == 0xff);
		if (m_pal_byte_index == PAL_ADDRESS_BYTES - 1 && (m_pal_unit || m_pal_bcast))
			m_pal_status = PAL_MATCHED;
		m_pal_byte_index++;
	}

	if (m_capture && !m_fifo->is_full()) {
		bool const was_empty = m_fifo->is_empty();
		m_fifo->push(byte);
		if (was_empty)
			irq_pulse();
	}
}

void na2_device::irq_pulse()
{
	m_rx_gate = false;
	m_maincpu->set_input_line(M6805_IRQ_LINE, ASSERT_LINE);
	m_maincpu->set_input_line(M6805_IRQ_LINE, CLEAR_LINE);
}

const tiny_rom_entry *na2_device::device_rom_region() const
{
	return ROM_NAME( na2 );
}

na2_slot_device::na2_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NABU_NETWORK_NA2_ADAPTER, tag, owner, clock)
	, device_rs232_port_interface(mconfig, *this)
	, m_na2(*this, "na2")
{
}

void na2_slot_device::device_add_mconfig(machine_config &config)
{
	NA2(config, m_na2);
	m_na2->txd_handler().set(FUNC(na2_slot_device::output_rxd));
}

} // namespace bus::nabu

