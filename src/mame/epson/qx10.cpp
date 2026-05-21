// license:BSD-3-Clause
// copyright-holders:Mariusz Wojcieszek, Angelo Salese, Brian Johnson
/***************************************************************************

    QX-10

    Preliminary driver by Mariusz Wojcieszek

    Status:
    Driver boots and load CP/M from floppy image.

    Done:
    - preliminary memory map
    - sound
    - floppy (upd765)
    - DMA
    - Interrupts (pic8295)
    - Video via Q10GMS / Q10CMS card slot

    banking:
    * 0x1c = 0 0x20 = 0
      - 0x0000-0xdfff: ROM (8KB mirrored to 56KB)
      - 0xe000-0xffff: Resident RAM
    * 0x1c = 0 0x20 = 1
      - 0x0000-0x7fff: ROM (8KB mirrored to 32KB)
      - 0x8000-0xdfff: CMOS (2KB mirrored to 24KB)
      - 0xe000-0xffff: Resident RAM
    * 0x1c = 1 0x20 = 0
      - 0x0000-0xdfff: Bankable RAM (current bank via 0x18)
      - 0xe000-0xffff: Resident RAM
    * 0x1c = 1 0x20 = 1
      - 0x0000-0x7fff: Bankable RAM (current bank via 0x18)
      - 0x8000-0xdfff: CMOS (2KB mirrored to 24KB)
      - 0xe000-0xffff: Resident RAM
****************************************************************************/


#include "emu.h"

#include "bus/centronics/ctronics.h"
#include "bus/epson_qx/keyboard/keyboard.h"
#include "bus/epson_qx/option.h"
#include "bus/epson_qx/video/qx16_ibm_cards.h"
#include "bus/epson_qx/video/qx_gdc_cards.h"
#include "bus/epson_qx/video/video.h"
#include "bus/rs232/rs232.h"
#include "cpu/i86/i86.h"
#include "cpu/z80/z80.h"
#include "imagedev/floppy.h"
#include "imagedev/snapquik.h"
#include "machine/am9517a.h"
#include "machine/i8255.h"
#include "machine/mc146818.h"
#include "machine/nvram.h"
#include "machine/output_latch.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/ram.h"
#include "machine/upd765.h"
#include "machine/z80sio.h"
#include "sound/spkrdev.h"

#include "speaker.h"
#include "screen.h"
#include "softlist_dev.h"


namespace {

#define MAIN_CLK    15974400

#define RS232_TAG   "rs232"

/*
    Driver data
*/

class qx10_state : public driver_device
{
public:
	qx10_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_pit_1(*this, "pit8253_1"),
		m_pit_2(*this, "pit8253_2"),
		m_pic_m(*this, "pic8259_master"),
		m_pic_s(*this, "pic8259_slave"),
		m_scc(*this, "upd7201"),
		m_ppi(*this, "i8255"),
		m_dma_1(*this, "8237dma_1"),
		m_dma_2(*this, "8237dma_2"),
		m_fdc(*this, "upd765"),
		m_floppy(*this, "upd765:%u", 0U),
		m_rtc(*this, "rtc"),
		m_kbd(*this, "kbd"),
		m_centronics(*this, "centronics"),
		m_bus(*this, "bus"),
		m_video_slot(*this, "video"),
		m_speaker(*this, "speaker"),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_lower_view(*this, "lower_ram"),
		m_upper_view(*this, "upper_ram"),
		m_external_view(*this, "external_ram"),
		m_ram(*this, RAM_TAG),
		m_rambank(*this, "rambank"),
		m_cmosram(*this, "cmosram", 0x800, ENDIANNESS_LITTLE),
		m_nvram(*this, "cmosram")
	{
	}

	void qx10(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	virtual void add_video_slot(machine_config &config) ATTR_COLD;
	virtual void add_option_slots(machine_config &config) ATTR_COLD;
	virtual void add_floppies(machine_config &config) ATTR_COLD;

	virtual int ram_bank_select(int drambank) const { return drambank; }

	void update_memory_mapping();

	void update_speaker();

	void qx10_18_w(uint8_t data);
	void prom_sel_w(uint8_t data);
	void cmos_sel_w(uint8_t data);
	void qx10_upd765_interrupt(int state);
	void update_fdd_motor(uint8_t state);
	void fdd_motor_w(uint8_t data);
	uint8_t qx10_30_r();
	void tc_w(int state);
	void sqw_out(uint8_t state);
	IRQ_CALLBACK_MEMBER( inta_call );
	uint8_t get_slave_ack(offs_t offset);
	uint8_t memory_read_byte(offs_t offset);
	void memory_write_byte(offs_t offset, uint8_t data);

	uint8_t portb_r();
	void portc_w(uint8_t data);

	void centronics_busy_handler(uint8_t state);
	void centronics_fault_handler(uint8_t state);
	void centronics_perror_handler(uint8_t state);
	void centronics_select_handler(uint8_t state);
	void centronics_sense_handler(uint8_t state);

	void keyboard_clk(int state);
	void keyboard_irq(int state);
	void speaker_freq(int state);
	void speaker_duration(int state);

	DECLARE_QUICKLOAD_LOAD_MEMBER(quickload_cb);

	void dma_hrq_changed(int state);

	void qx10_io(address_map &map) ATTR_COLD;
	void qx10_mem(address_map &map) ATTR_COLD;

	required_device<pit8253_device> m_pit_1;
	required_device<pit8253_device> m_pit_2;
	required_device<pic8259_device> m_pic_m;
	required_device<pic8259_device> m_pic_s;
	required_device<upd7201_device> m_scc;
	required_device<i8255_device> m_ppi;
	required_device<am9517a_device> m_dma_1;
	required_device<am9517a_device> m_dma_2;
	required_device<upd765a_device> m_fdc;
	required_device_array<floppy_connector, 2> m_floppy;
	required_device<mc146818_device> m_rtc;
	required_device<bus::epson_qx::keyboard::keyboard_port_device> m_kbd;
	required_device<centronics_device> m_centronics;
	required_device<bus::epson_qx::option_bus_device> m_bus;
	required_device<bus::epson_qx::video::video_slot_device> m_video_slot;
	required_device<speaker_sound_device>   m_speaker;
	required_device<z80_device> m_maincpu;
	required_device<screen_device> m_screen;
	memory_view m_lower_view;
	memory_view m_upper_view;
	memory_view m_external_view;
	required_device<ram_device> m_ram;
	memory_bank_creator m_rambank;
	memory_share_creator<u8> m_cmosram;
	required_device<nvram_device> m_nvram;

	/* FDD */
	int     m_fdcint = 0;
	uint8_t  m_motor_clk = 0;
	uint16_t m_counter = 0;
	//int     m_fdcready = 0;

	int m_spkr_enable = 0;
	int m_spkr_freq = 0;
	int m_pit1_out0 = 0;

	/* centronics */
	int m_centronics_error = 0;
	int m_centronics_busy = 0;
	int m_centronics_paper = 0;
	int m_centronics_select = 0;
	int m_centronics_sense = 0;


	/* memory */
	int     m_external_bank = 0;
	int     m_membank = 0;
	int     m_memprom = 0;
	int     m_memcmos = 0;
};

/*
    Sound
*/
void qx10_state::update_speaker()
{

	/*
	 *                 freq -----
	 * pit1_out0 -----            NAND ---- level
	 *                 NAND -----
	 * !enable   -----
	 */

	uint8_t level = ((!m_spkr_enable && m_pit1_out0) || !m_spkr_freq) ? 1 : 0;
	m_speaker->level_w(level);
}

/*
    Memory
*/
void qx10_state::update_memory_mapping()
{
	int drambank = -1;

	if (m_membank & 1)      { drambank = 0; }
	else if (m_membank & 2) { drambank = 1; }
	else if (m_membank & 4) { drambank = 2; }
	else if (m_membank & 8) { drambank = 3; }

	if (drambank >= 0)
	{
		m_rambank->set_entry(ram_bank_select(drambank));
	}

	if (m_external_bank)
	{
		m_external_view.select(0);
	}
	else
	{
		m_external_view.disable();
	}

	if (!m_memprom)
	{
		m_lower_view.select(0);
	}
	else
	{
		m_lower_view.disable();
	}

	if (m_memcmos)
	{
		m_upper_view.select(1);
	}
	else if(!m_memprom)
	{
		m_upper_view.select(0);
	}
	else
	{
		m_upper_view.disable();
	}
}

void qx10_state::qx10_18_w(uint8_t data)
{
	m_membank = (data >> 4) & 0x0f;
	m_spkr_enable = (data >> 2) & 0x01;
	m_external_bank = (data >> 3) & 0x01;
	m_pit_1->write_gate2(BIT(data, 1));
	m_pit_1->write_gate0(data & 1);
	update_speaker();
	update_memory_mapping();
}

void qx10_state::prom_sel_w(uint8_t data)
{
	m_memprom = data & 1;
	update_memory_mapping();
}

void qx10_state::cmos_sel_w(uint8_t data)
{
	m_memcmos = data & 1;
	update_memory_mapping();
}

/***********************************************************

    Quickload

    This loads a .COM file to address 0x100 then jumps
    there. Sometimes .COM has been renamed to .CPM to
    prevent windows going ballistic. These can be loaded
    as well.

************************************************************/

QUICKLOAD_LOAD_MEMBER(qx10_state::quickload_cb)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);

	if (image.length() >= 0xfd00)
		return std::make_pair(image_error::INVALIDLENGTH, std::string());

	/* The right RAM bank must be active */
	m_membank = 0;
	update_memory_mapping();

	/* Avoid loading a program if CP/M-80 is not in memory */
	if ((prog_space.read_byte(0) != 0xc3) || (prog_space.read_byte(5) != 0xc3))
	{
		machine_reset();
		return std::make_pair(image_error::UNSUPPORTED, std::string());
	}

	/* Load image to the TPA (Transient Program Area) */
	uint16_t quickload_size = image.length();
	for (uint16_t i = 0; i < quickload_size; i++)
	{
		uint8_t data;
		if (image.fread( &data, 1) != 1)
			return std::make_pair(image_error::UNSPECIFIED, std::string());
		prog_space.write_byte(i+0x100, data);
	}

	/* clear out command tail */
	prog_space.write_byte(0x80, 0);   prog_space.write_byte(0x81, 0);

	/* Roughly set SP basing on the BDOS position */
	m_maincpu->set_state_int(Z80_SP, 256 * prog_space.read_byte(7) - 300);
	m_maincpu->set_pc(0x100);       // start program

	return std::make_pair(std::error_condition(), std::string());
}

/*
    FDD
*/

static void qx10_floppies(device_slot_interface &device)
{
	device.option_add("525dd", FLOPPY_525_DD);
}

static void qx16_floppies(device_slot_interface &device)
{
	device.option_add("525qd", EPSON_SD_543);
}

void qx10_state::qx10_upd765_interrupt(int state)
{
	m_fdcint = state;

	//logerror("Interrupt from upd765: %d\n", state);
	// signal interrupt
	m_pic_m->ir6_w(state);
}

void qx10_state::update_fdd_motor(uint8_t state)
{
	for (auto& fdd : m_floppy)
	{
		floppy_image_device *floppy = fdd->get_device();
		if (floppy)
		{
			floppy->mon_w(state);
//			floppy->set_ready(state);
		}
	}
}

void qx10_state::fdd_motor_w(uint8_t data)
{
	m_counter = 0;
	update_fdd_motor(0);
	// motor off controlled by clock
}

uint8_t qx10_state::qx10_30_r()
{
	floppy_image_device *floppy1,*floppy2;

	floppy1 = m_floppy[0]->get_device();
	floppy2 = m_floppy[1]->get_device();

	return m_fdcint |
			BIT(m_counter, 11) << 1 |
			((floppy1 != nullptr) || (floppy2 != nullptr) ? 1 : 0) << 3 |
			m_membank << 4;
}

void qx10_state::centronics_busy_handler(uint8_t state)
{
	m_centronics_busy = state;
}

void qx10_state::centronics_perror_handler(uint8_t state)
{
	m_centronics_paper = state;
}

void qx10_state::centronics_fault_handler(uint8_t state)
{
	m_centronics_error = state;
}

void qx10_state::centronics_select_handler(uint8_t state)
{
	m_centronics_select = state;
}

void qx10_state::centronics_sense_handler(uint8_t state)
{
	m_centronics_sense = state;
}

uint8_t qx10_state::portb_r()
{
	uint8_t status = 0;

	status |= m_centronics_error  << 3;
	status |= m_centronics_paper  << 4;
	status |= m_centronics_busy   << 5;
	status |= m_centronics_sense  << 6;
	status |= m_centronics_select << 7;

	return status;
}

void qx10_state::portc_w(uint8_t data)
{
	m_centronics->write_strobe(BIT(data, 0));
	m_centronics->write_autofd(BIT(data, 4));
	m_centronics->write_init(!BIT(data, 5));
	m_pic_s->ir0_w(BIT(data, 3));
}

/*
    DMA8237
*/
void qx10_state::dma_hrq_changed(int state)
{
	/* Assert HLDA */
	m_dma_1->hack_w(state);
}

void qx10_state::tc_w(int state)
{
	/* floppy terminal count */
	m_fdc->tc_w(!state);
	m_bus->slots_w<&bus::epson_qx::option_slot_device::eopf>(state);
}

/*
    8237 DMA (Master)
    Channel 1: Floppy disk
    Channel 2: GDC
    Channel 3: Option slots
*/
uint8_t qx10_state::memory_read_byte(offs_t offset)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	return prog_space.read_byte(offset);
}

void qx10_state::memory_write_byte(offs_t offset, uint8_t data)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	return prog_space.write_byte(offset, data);
}

/*
    8237 DMA (Slave)
    Channel 1: Option slots #1
    Channel 2: Option slots #2
    Channel 3: Option slots #3
    Channel 4: Option slots #4
*/

/*
    MC146818
*/
void qx10_state::sqw_out(uint8_t state)
{
	uint8_t clk = !(state || BIT(m_counter, 11));
	uint16_t cnt = m_counter;

	if (!clk && m_motor_clk)
	{
		cnt = (cnt + 1) & 0xfff;
	}
	if (BIT(cnt, 11) && !BIT(m_counter, 11))
	{
		update_fdd_motor(1);
	}

	m_motor_clk = clk;
	m_counter = cnt;
}

void qx10_state::keyboard_irq(int state)
{
	m_scc->m1_r(); // always set
	m_pic_m->ir4_w(state);
}

void qx10_state::keyboard_clk(int state)
{
	// clock keyboard too
	m_kbd->clk_w(state);
	m_scc->rxca_w(state);
	m_scc->txca_w(state);
}

void qx10_state::speaker_duration(int state)
{
	m_pit1_out0 = state;
	update_speaker();
}

void qx10_state::speaker_freq(int state)
{
	m_spkr_freq = state;
	update_speaker();
}

/*
    Master PIC8259
    IR0     Power down detection interrupt
    IR1     Software timer #1 interrupt
    IR2     External interrupt INTF1
    IR3     External interrupt INTF2
    IR4     Keyboard/RS232 interrupt
    IR5     CRT/lightpen interrupt
    IR6     Floppy controller interrupt
    IR7     Slave cascade
*/

IRQ_CALLBACK_MEMBER(qx10_state::inta_call)
{
	uint32_t vector = m_pic_m->acknowledge() << 16;
	vector |= m_pic_m->acknowledge();
	vector |= m_pic_m->acknowledge() << 8;
	return vector;
}

uint8_t qx10_state::get_slave_ack(offs_t offset)
{
	if (offset==7) { // IRQ = 7
		return m_pic_s->acknowledge();
	}
	return 0x00;
}


/*
    Slave PIC8259
    IR0     Printer interrupt
    IR1     External interrupt #1
    IR2     Calendar clock interrupt
    IR3     External interrupt #2
    IR4     External interrupt #3
    IR5     Software timer #2 interrupt
    IR6     External interrupt #4
    IR7     External interrupt #5

*/

void qx10_state::qx10_mem(address_map &map)
{
	map.unmap_value_high();

	// 56KB bankable RAM (4 banks)¬
	map(0x0000, 0xdfff).bankrw("rambank");

	// External RAM: unmapped, cards install handlers here
	map(0x0000, 0xdfff).view(m_external_view);
	m_external_view[0](0x0000, 0xdfff).unmaprw();

	// ROM: 8KB → 32K
	map(0x0000, 0x7fff).view(m_lower_view);
	m_lower_view[0](0x0000, 0x1fff).mirror(0x6000).rom().region("maincpu", 0).nopw();

	// ROM: 8KB → 24K
	// CMOS: 2KB → 24K
	map(0x8000, 0xdfff).view(m_upper_view);
	m_upper_view[0](0x8000, 0x9fff).mirror(0x2000).rom().region("maincpu", 0).nopw();
	m_upper_view[0](0xc000, 0xdfff).rom().region("maincpu", 0).nopw();
	m_upper_view[1](0x8000, 0x87ff).mirror(0x3800).ram().share("cmosram");
	m_upper_view[1](0xc000, 0xc7ff).mirror(0x1800).ram().share("cmosram");

	// Resident RAM
	map(0xe000, 0xffff).ram();
}

void qx10_state::qx10_io(address_map &map)
{
	map.unmap_value_high();
	map(0x00, 0x03).mirror(0xff00).rw(m_pit_1, FUNC(pit8253_device::read), FUNC(pit8253_device::write));
	map(0x04, 0x07).mirror(0xff00).rw(m_pit_2, FUNC(pit8253_device::read), FUNC(pit8253_device::write));
	map(0x08, 0x09).mirror(0xff00).rw(m_pic_m, FUNC(pic8259_device::read), FUNC(pic8259_device::write));
	map(0x0c, 0x0d).mirror(0xff00).rw(m_pic_s, FUNC(pic8259_device::read), FUNC(pic8259_device::write));
	map(0x10, 0x13).mirror(0xff00).rw(m_scc, FUNC(upd7201_device::cd_ba_r), FUNC(upd7201_device::cd_ba_w));
	map(0x14, 0x17).mirror(0xff00).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(0x18, 0x1b).mirror(0xff00).portr("DSW").w(FUNC(qx10_state::qx10_18_w));
	map(0x1c, 0x1f).mirror(0xff00).w(FUNC(qx10_state::prom_sel_w));
	map(0x20, 0x23).mirror(0xff00).w(FUNC(qx10_state::cmos_sel_w));
	map(0x30, 0x33).mirror(0xff00).rw(FUNC(qx10_state::qx10_30_r), FUNC(qx10_state::fdd_motor_w));
	map(0x34, 0x35).mirror(0xff00).m(m_fdc, FUNC(upd765a_device::map));
//  map(0x3b, 0x3b) GDC light pen req
	map(0x3c, 0x3c).mirror(0xff00).rw(m_rtc, FUNC(mc146818_device::data_r), FUNC(mc146818_device::data_w));
	map(0x3d, 0x3d).mirror(0xff00).w(m_rtc, FUNC(mc146818_device::address_w));
	map(0x40, 0x4f).mirror(0xff00).rw(m_dma_1, FUNC(am9517a_device::read), FUNC(am9517a_device::write));
	map(0x50, 0x5f).mirror(0xff00).rw(m_dma_2, FUNC(am9517a_device::read), FUNC(am9517a_device::write));
}

/* Input ports */
/* TODO: shift break */
/*INPUT_CHANGED_MEMBER(qx10_state::key_stroke)
{
    if(newval && !oldval)
    {
        m_keyb.rx = uint8_t(param & 0x7f);
        m_pic_m->ir4_w(1);
    }

    if(oldval && !newval)
        m_keyb.rx = 0;
}*/

static INPUT_PORTS_START( qx10 )
	/* TODO: All of those have unknown meaning */
	PORT_START("DSW")
	PORT_DIPNAME( 0x01, 0x00, "DSW" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, DEF_STR( Unknown ) ) //CMOS related
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END

static INPUT_PORTS_START( qx16 )
	PORT_START("DSW")
	PORT_DIPNAME( 0x01, 0x00, "Valdocs" )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPUNUSED( 0x02, IP_ACTIVE_HIGH )
	PORT_DIPUNUSED( 0x04, IP_ACTIVE_HIGH )
	PORT_DIPUNUSED( 0x08, IP_ACTIVE_HIGH )
	PORT_DIPNAME( 0x70, 0x00, "MS-DOS Video Mode" )                // switches 5-7
	PORT_DIPSETTING(    0x00, "QX-16 Native" )
	PORT_DIPSETTING(    0x60, "IBM Monochrome" )
	PORT_DIPSETTING(    0x40, "IBM Color 80x25 (mono monitor)" )
	PORT_DIPSETTING(    0x20, "IBM Color 40x25 (mono monitor)" )
	PORT_DIPSETTING(    0x50, "IBM Color 80x25 (color monitor)" )
	PORT_DIPSETTING(    0x30, "IBM Color 40x25 (color monitor)" )
	PORT_DIPNAME( 0x80, 0x00, "POST Self-Test" )
	PORT_DIPSETTING(    0x00, "Full (factory default)" )
	PORT_DIPSETTING(    0x80, "Minimal" )
INPUT_PORTS_END


void qx10_state::machine_start()
{
	m_rambank->configure_entries(0, m_ram->size() / 0x10000, m_ram->pointer(), 0x10000);
	m_bus->set_memview(m_external_view[0]);

	// FDD
	save_item(NAME(m_fdcint));
	save_item(NAME(m_motor_clk));
	save_item(NAME(m_counter));

	// Speaker
	save_item(NAME(m_spkr_enable));
	save_item(NAME(m_spkr_freq));
	save_item(NAME(m_pit1_out0));

	// Centronics
	save_item(NAME(m_centronics_error));
	save_item(NAME(m_centronics_busy));
	save_item(NAME(m_centronics_paper));
	save_item(NAME(m_centronics_select));
	save_item(NAME(m_centronics_sense));

	// Memory banking
	save_item(NAME(m_external_bank));
	save_item(NAME(m_membank));
	save_item(NAME(m_memprom));
	save_item(NAME(m_memcmos));
}

void qx10_state::machine_reset()
{
	m_dma_1->dreq0_w(1);
	m_dma_1->dreq1_w(1);

	m_spkr_enable = 0;
	m_pit1_out0 = 1;

	m_external_bank = 0;
	m_memprom = 0;
	m_memcmos = 0;
	m_membank = 0;
	update_memory_mapping();
}

void qx10_state::add_video_slot(machine_config &config)
{
	EPSON_QX_VIDEO_SLOT(config, m_video_slot, bus::epson_qx::video::qx10_video_cards, "q10gms");
	m_video_slot->set_iospace(m_maincpu, AS_IO);
	m_video_slot->drq_callback().set(m_dma_1, FUNC(am9517a_device::dreq1_w)).invert();
}

void qx10_state::add_option_slots(machine_config &config)
{
	EPSON_QX_OPTION_BUS_SLOT(config, "option1", m_bus, 0, bus::epson_qx::option_bus_devices, nullptr);
	EPSON_QX_OPTION_BUS_SLOT(config, "option2", m_bus, 1, bus::epson_qx::option_bus_devices, nullptr);
	EPSON_QX_OPTION_BUS_SLOT(config, "option3", m_bus, 2, bus::epson_qx::option_bus_devices, nullptr);
	EPSON_QX_OPTION_BUS_SLOT(config, "option4", m_bus, 3, bus::epson_qx::option_bus_devices, nullptr);
	EPSON_QX_OPTION_BUS_SLOT(config, "option5", m_bus, 4, bus::epson_qx::option_bus_devices, nullptr);
}

void qx10_state::add_floppies(machine_config &config)
{
	FLOPPY_CONNECTOR(config, m_floppy[0], qx10_floppies, "525dd", floppy_image_device::default_mfm_floppy_formats);
	FLOPPY_CONNECTOR(config, m_floppy[1], qx10_floppies, "525dd", floppy_image_device::default_mfm_floppy_formats);
}

void qx10_state::qx10(machine_config &config)
{
	/* basic machine hardware */
	Z80(config, m_maincpu, MAIN_CLK / 4);
	m_maincpu->z80_set_m1_cycles(4+1);
	m_maincpu->set_addrmap(AS_PROGRAM, &qx10_state::qx10_mem);
	m_maincpu->set_addrmap(AS_IO, &qx10_state::qx10_io);
	m_maincpu->set_irq_acknowledge_callback(FUNC(qx10_state::inta_call));

	/* video hardware */
	SCREEN(config, m_screen);
	m_screen->set_raw(16.67_MHz_XTAL, 872, 152, 792, 421, 4, 404);
	m_screen->set_screen_update(m_video_slot, FUNC(bus::epson_qx::video::video_slot_device::screen_update));

	add_video_slot(config);

	/* Devices */

/*
    Timer 0
    Counter CLK                         Gate                    OUT             Operation
    0       Keyboard clock (1200bps)    Memory register D0      Speaker timer   Speaker timer (100ms)
    1       Keyboard clock (1200bps)    +5V                     8259A (10E) IR5 Software timer
    2       Clock 1,9668MHz             Memory register D7      8259 (12E) IR1  Software timer
*/
	PIT8253(config, m_pit_1);
	m_pit_1->set_clk<0>(1200);
	m_pit_1->out_handler<0>().set(FUNC(qx10_state::speaker_duration));
	m_pit_1->set_clk<1>(1200);
	m_pit_1->out_handler<1>().set(m_pic_s, FUNC(pic8259_device::ir5_w));
	m_pit_1->set_clk<2>(MAIN_CLK / 8);
	m_pit_1->out_handler<2>().set(m_pic_m, FUNC(pic8259_device::ir1_w));

/*
    Timer 1
    Counter CLK                 Gate        OUT                 Operation
    0       Clock 1,9668MHz     +5V         Speaker frequency   1kHz
    1       Clock 1,9668MHz     +5V         Keyboard clock      1200bps (Clock / 1664)
    2       Clock 1,9668MHz     +5V         RS-232C baud rate   9600bps (Clock / 208)
*/
	PIT8253(config, m_pit_2);
	m_pit_2->set_clk<0>(MAIN_CLK / 8);
	m_pit_2->out_handler<0>().set(FUNC(qx10_state::speaker_freq));
	m_pit_2->set_clk<1>(MAIN_CLK / 8);
	m_pit_2->out_handler<1>().set(FUNC(qx10_state::keyboard_clk));
	m_pit_2->set_clk<2>(MAIN_CLK / 8);
	m_pit_2->out_handler<2>().set(m_scc, FUNC(upd7201_device::rxtxcb_w));

	PIC8259(config, m_pic_m);
	m_pic_m->out_int_callback().set_inputline(m_maincpu, 0);
	m_pic_m->in_sp_callback().set_constant(1);
	m_pic_m->read_slave_ack_callback().set(FUNC(qx10_state::get_slave_ack));

	PIC8259(config, m_pic_s);
	m_pic_s->out_int_callback().set(m_pic_m, FUNC(pic8259_device::ir7_w));
	m_pic_s->in_sp_callback().set_constant(0);

	UPD7201(config, m_scc, MAIN_CLK/4); // channel b clock set by pit2 channel 2
	// Channel A: Keyboard
	m_scc->out_txda_callback().set(m_kbd, FUNC(bus::epson_qx::keyboard::keyboard_port_device::rxd_w));
	// Channel B: RS232
	m_scc->out_txdb_callback().set(RS232_TAG, FUNC(rs232_port_device::write_txd));
	m_scc->out_dtrb_callback().set(RS232_TAG, FUNC(rs232_port_device::write_dtr));
	m_scc->out_rtsb_callback().set(RS232_TAG, FUNC(rs232_port_device::write_rts));
	m_scc->out_int_callback().set(FUNC(qx10_state::keyboard_irq));

	AM9517A(config, m_dma_1, MAIN_CLK/4);
	m_dma_1->dreq_active_low();
	m_dma_1->out_hreq_callback().set(FUNC(qx10_state::dma_hrq_changed));
	m_dma_1->out_eop_callback().set(FUNC(qx10_state::tc_w));
	m_dma_1->in_memr_callback().set(FUNC(qx10_state::memory_read_byte));
	m_dma_1->out_memw_callback().set(FUNC(qx10_state::memory_write_byte));
	m_dma_1->in_ior_callback<0>().set(m_fdc, FUNC(upd765a_device::dma_r));
	m_dma_1->in_ior_callback<1>().set(m_video_slot, FUNC(bus::epson_qx::video::video_slot_device::dack_r));
	m_dma_1->out_iow_callback<0>().set(m_fdc, FUNC(upd765a_device::dma_w));
	m_dma_1->out_iow_callback<1>().set(m_video_slot, FUNC(bus::epson_qx::video::video_slot_device::dack_w));

	AM9517A(config, m_dma_2, MAIN_CLK/4);
	m_dma_2->dreq_active_low();

	I8255(config, m_ppi);
	m_ppi->out_pa_callback().set("prndata", FUNC(output_latch_device::write));
	m_ppi->in_pb_callback().set(FUNC(qx10_state::portb_r));
	m_ppi->out_pc_callback().set(FUNC(qx10_state::portc_w));

	MC146818(config, m_rtc, 32.768_kHz_XTAL);
	m_rtc->irq().set(m_pic_s, FUNC(pic8259_device::ir2_w));
	m_rtc->sqw().set(FUNC(qx10_state::sqw_out));

	UPD765A(config, m_fdc, 8'000'000, true, true);
	m_fdc->intrq_wr_callback().set(FUNC(qx10_state::qx10_upd765_interrupt));
	m_fdc->drq_wr_callback().set(m_dma_1, FUNC(am9517a_device::dreq0_w)).invert();
	add_floppies(config);

	rs232_port_device &rs232(RS232_PORT(config, RS232_TAG, default_rs232_devices, nullptr));
	rs232.rxd_handler().set(m_scc, FUNC(upd7201_device::rxb_w));

	EPSON_QX_KEYBOARD_PORT(config, m_kbd, bus::epson_qx::keyboard::keyboard_devices, "qx10_hasci");
	m_kbd->txd_handler().set(m_scc, FUNC(upd7201_device::rxa_w));

	output_latch_device &prndata(OUTPUT_LATCH(config, "prndata"));
	CENTRONICS(config, m_centronics, centronics_devices, nullptr);
	m_centronics->set_output_latch(prndata);
	m_centronics->ack_handler().set(m_ppi, FUNC(i8255_device::pc6_w));
	m_centronics->busy_handler().set(FUNC(qx10_state::centronics_busy_handler));
	m_centronics->perror_handler().set(FUNC(qx10_state::centronics_perror_handler));
	m_centronics->fault_handler().set(FUNC(qx10_state::centronics_fault_handler));
	m_centronics->select_handler().set(FUNC(qx10_state::centronics_select_handler));
	m_centronics->sense_handler().set(FUNC(qx10_state::centronics_sense_handler));

	/* sound hardware */
	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 1.00);

	/* internal ram */
	RAM(config, m_ram).set_default_size("256K");
	NVRAM(config, m_nvram, nvram_device::DEFAULT_NONE);

	EPSON_QX_OPTION_BUS(config, m_bus, MAIN_CLK / 4);
	m_dma_1->out_iow_callback<2>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dackf_w));
	m_dma_1->in_ior_callback<2>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dackf_r));
	m_dma_2->out_iow_callback<0>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_w<0>));
	m_dma_2->in_ior_callback<0>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_r<0>));
	m_dma_2->out_iow_callback<1>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_w<1>));
	m_dma_2->in_ior_callback<1>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_r<1>));
	m_dma_2->out_iow_callback<2>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_w<2>));
	m_dma_2->in_ior_callback<2>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_r<2>));
	m_dma_2->out_iow_callback<3>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_w<3>));
	m_dma_2->in_ior_callback<3>().set(m_bus, FUNC(bus::epson_qx::option_bus_device::dacks_r<3>));
	m_dma_2->out_eop_callback().set(m_bus, FUNC(bus::epson_qx::option_bus_device::slots_w<&bus::epson_qx::option_slot_device::eops>));
	m_bus->out_drqf_callback().set(m_dma_1, FUNC(am9517a_device::dreq2_w));
	m_bus->out_rdyf_callback().set(m_dma_1, FUNC(am9517a_device::ready_w));
	m_bus->out_drqs_callback<0>().set(m_dma_2, FUNC(am9517a_device::dreq0_w));
	m_bus->out_drqs_callback<1>().set(m_dma_2, FUNC(am9517a_device::dreq1_w));
	m_bus->out_drqs_callback<2>().set(m_dma_2, FUNC(am9517a_device::dreq2_w));
	m_bus->out_drqs_callback<3>().set(m_dma_2, FUNC(am9517a_device::dreq3_w));
	m_bus->out_rdys_callback().set(m_dma_2, FUNC(am9517a_device::ready_w));
	m_bus->out_inth1_callback().set(m_pic_m, FUNC(pic8259_device::ir2_w));
	m_bus->out_inth2_callback().set(m_pic_m, FUNC(pic8259_device::ir3_w));
	m_bus->out_intl_callback<0>().set(m_pic_s, FUNC(pic8259_device::ir1_w));
	m_bus->out_intl_callback<1>().set(m_pic_s, FUNC(pic8259_device::ir3_w));
	m_bus->out_intl_callback<2>().set(m_pic_s, FUNC(pic8259_device::ir4_w));
	m_bus->out_intl_callback<3>().set(m_pic_s, FUNC(pic8259_device::ir6_w));
	m_bus->out_intl_callback<4>().set(m_pic_s, FUNC(pic8259_device::ir7_w));
	m_bus->set_iospace(m_maincpu, AS_IO);
	add_option_slots(config);

	// software lists
	SOFTWARE_LIST(config, "flop_list").set_original("qx10_flop");

	QUICKLOAD(config, "quickload", "com,cpm", attotime::from_seconds(3)).set_load_callback(FUNC(qx10_state::quickload_cb));
}


/*
    QX-16 — shares the QX-10 Z80 side, adds an 8088 for MS-DOS mode.
*/

class qx16_state : public qx10_state
{
public:
	qx16_state(const machine_config &mconfig, device_type type, const char *tag)
		: qx10_state(mconfig, type, tag),
		m_subcpu(*this, "subcpu"),
		m_ibm_video_slot(*this, "ibm_video")
	{
	}

	void qx16(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	virtual void add_video_slot(machine_config &config) override ATTR_COLD;
	virtual void add_option_slots(machine_config &config) override ATTR_COLD;
	virtual void add_floppies(machine_config &config) override ATTR_COLD;

	virtual int ram_bank_select(int drambank) const override { return m_memgroup * 4 + drambank; }

	void cpu_switch_w(uint8_t data);
	void bank_group_w(uint8_t data) { m_memgroup = data & 1; update_memory_mapping(); }
	void qx16_18_w(uint8_t data);
	uint8_t qx16_portb_r();
	void qx16_portc_w(uint8_t data);
	uint8_t option_io_r(offs_t offset);
	void option_io_w(offs_t offset, uint8_t data);
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	IRQ_CALLBACK_MEMBER(inta_call_subcpu);
	uint8_t dma_memory_read_byte(offs_t offset);
	void dma_memory_write_byte(offs_t offset, uint8_t data);
	void qx16_z80_io(address_map &map) ATTR_COLD;
	void qx16_8088_io(address_map &map) ATTR_COLD;
	void qx16_8088_mem(address_map &map) ATTR_COLD;

	required_device<cpu_device> m_subcpu;
	required_device<bus::epson_qx::video::video_slot_device> m_ibm_video_slot;

	uint8_t m_memgroup = 0;
	uint8_t m_cpu_switch = 0;
};

void qx16_state::machine_start()
{
	qx10_state::machine_start();
	save_item(NAME(m_memgroup));
	save_item(NAME(m_cpu_switch));

	m_subcpu->space(AS_PROGRAM).install_ram(0x00000, m_ram->size() - 1, m_ram->pointer());
}

void qx16_state::machine_reset()
{
	qx10_state::machine_reset();
	m_subcpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
	m_cpu_switch = 0;
}

void qx16_state::cpu_switch_w(uint8_t data)
{
	m_cpu_switch = data;
	if(data)
	{
		m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		m_subcpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	}
	else
	{
		m_subcpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	}
}

uint32_t qx16_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	if (m_video_slot->enabled())
		return m_video_slot->screen_update(screen, bitmap, cliprect);
	if (m_ibm_video_slot->enabled())
		return m_ibm_video_slot->screen_update(screen, bitmap, cliprect);
	bitmap.fill(rgb_t::black(), cliprect);
	return 0;
}

void qx16_state::qx16_18_w(uint8_t data)
{
	m_membank = (data >> 4) & 0x0f;
	m_spkr_enable = (data >> 2) & 0x01;
	m_pit_1->write_gate2(BIT(data, 1));
	m_pit_1->write_gate0(data & 1);
	update_speaker();
}

uint8_t qx16_state::qx16_portb_r()
{
	uint8_t status = portb_r();

	status |= (m_memgroup << 1);

	return status;
}

void qx16_state::qx16_portc_w(uint8_t data)
{
	portc_w(data);

	int const mode = BIT(data, 1);
	for (auto &fc : m_floppy)
		if (auto *sd = dynamic_cast<epson_sd_543 *>(fc->get_device()))
			sd->tpi_mode_w(mode);
}

IRQ_CALLBACK_MEMBER(qx16_state::inta_call_subcpu)
{
	return m_pic_m->acknowledge();
}

uint8_t qx16_state::dma_memory_read_byte(offs_t offset)
{
	if (!m_cpu_switch)
		return m_maincpu->space(AS_PROGRAM).read_byte(offset & 0xffff);
	int drambank = 0;
	if      (m_membank & 1) drambank = 0;
	else if (m_membank & 2) drambank = 1;
	else if (m_membank & 4) drambank = 2;
	else if (m_membank & 8) drambank = 3;
	return m_ram->pointer()[(m_memgroup * 0x40000) + (drambank * 0x10000) + (offset & 0xffff)];
}

void qx16_state::dma_memory_write_byte(offs_t offset, uint8_t data)
{
	if (!m_cpu_switch)
	{
		m_maincpu->space(AS_PROGRAM).write_byte(offset & 0xffff, data);
		return;
	}
	int drambank = 0;
	if      (m_membank & 1) drambank = 0;
	else if (m_membank & 2) drambank = 1;
	else if (m_membank & 4) drambank = 2;
	else if (m_membank & 8) drambank = 3;
	m_ram->pointer()[(m_memgroup * 0x40000) + (drambank * 0x10000) + (offset & 0xffff)] = data;
}

void qx16_state::qx16_z80_io(address_map &map)
{
	qx10_io(map);
	map(0x24, 0x24).mirror(0xff00).w(FUNC(qx16_state::cpu_switch_w));
	map(0x28, 0x28).mirror(0xff00).w(FUNC(qx16_state::bank_group_w));
}

void qx16_state::qx16_8088_io(address_map &map)
{
	map(0x00, 0x03).rw(m_pit_1, FUNC(pit8253_device::read), FUNC(pit8253_device::write));
	map(0x04, 0x07).rw(m_pit_2, FUNC(pit8253_device::read), FUNC(pit8253_device::write));
	map(0x08, 0x09).rw(m_pic_m, FUNC(pic8259_device::read), FUNC(pic8259_device::write));
	map(0x0c, 0x0d).rw(m_pic_s, FUNC(pic8259_device::read), FUNC(pic8259_device::write));
	map(0x10, 0x13).rw(m_scc, FUNC(upd7201_device::cd_ba_r), FUNC(upd7201_device::cd_ba_w));
	map(0x14, 0x17).rw(m_ppi, FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(0x18, 0x18).portr("DSW").w(FUNC(qx16_state::qx16_18_w));
	map(0x28, 0x28).w(FUNC(qx16_state::bank_group_w));
	map(0x30, 0x33).rw(FUNC(qx16_state::qx10_30_r), FUNC(qx16_state::fdd_motor_w));
	map(0x34, 0x35).m(m_fdc, FUNC(upd765a_device::map));
	map(0x3c, 0x3c).rw(m_rtc, FUNC(mc146818_device::data_r), FUNC(mc146818_device::data_w));
	map(0x3d, 0x3d).w(m_rtc, FUNC(mc146818_device::address_w));
	map(0x40, 0x4f).rw(m_dma_1, FUNC(am9517a_device::read), FUNC(am9517a_device::write));
	map(0x50, 0x5f).rw(m_dma_2, FUNC(am9517a_device::read), FUNC(am9517a_device::write));
	map(0x80, 0xff).rw(FUNC(qx16_state::option_io_r), FUNC(qx16_state::option_io_w));
}

uint8_t qx16_state::option_io_r(offs_t offset)
{
	return m_bus->iospace().read_byte(0x80 + offset);
}

void qx16_state::option_io_w(offs_t offset, uint8_t data)
{
	m_bus->iospace().write_byte(0x80 + offset, data);
}

void qx16_state::qx16_8088_mem(address_map &map)
{
	map.unmap_value_high();
	map(0xc0000, 0xc3fff).mirror(0x3c000).rom().region("subcpu", 0);
}

void qx16_state::add_video_slot(machine_config &config)
{
	EPSON_QX_VIDEO_SLOT(config, m_video_slot, bus::epson_qx::video::qx16_video_cards, "q16gms");
	m_video_slot->set_iospace(m_maincpu, AS_IO);
	m_video_slot->set_extra_iospace(m_subcpu, AS_IO);
	m_video_slot->drq_callback().set(m_dma_1, FUNC(am9517a_device::dreq1_w)).invert();

	EPSON_QX_VIDEO_SLOT(config, m_ibm_video_slot, bus::epson_qx::video::qx16_ibm_video_cards, "pcvideo");
	m_ibm_video_slot->set_iospace(m_subcpu, AS_IO);
	m_ibm_video_slot->set_memspace(m_subcpu, AS_PROGRAM);
}

void qx16_state::add_option_slots(machine_config &config)
{
	EPSON_QX_OPTION_BUS_SLOT(config, "option1", m_bus, 0, bus::epson_qx::option_bus_devices, nullptr);
	EPSON_QX_OPTION_BUS_SLOT(config, "option2", m_bus, 1, bus::epson_qx::option_bus_devices, nullptr);
	EPSON_QX_OPTION_BUS_SLOT(config, "option3", m_bus, 2, bus::epson_qx::option_bus_devices, nullptr);
}

void qx16_state::add_floppies(machine_config &config)
{
	FLOPPY_CONNECTOR(config, m_floppy[0], qx16_floppies, "525qd", floppy_image_device::default_mfm_floppy_formats);
	FLOPPY_CONNECTOR(config, m_floppy[1], qx16_floppies, "525qd", floppy_image_device::default_mfm_floppy_formats);
}

void qx16_state::qx16(machine_config &config)
{
	qx10(config);

	I8088(config, m_subcpu, MAIN_CLK / 3);
	m_subcpu->set_addrmap(AS_PROGRAM, &qx16_state::qx16_8088_mem);
	m_subcpu->set_addrmap(AS_IO, &qx16_state::qx16_8088_io);
	m_maincpu->set_addrmap(AS_IO, &qx16_state::qx16_z80_io);

	m_pic_m->out_int_callback().append_inputline(m_subcpu, 0);
	m_subcpu->set_irq_acknowledge_callback(FUNC(qx16_state::inta_call_subcpu));

	m_dma_1->in_memr_callback().set(FUNC(qx16_state::dma_memory_read_byte));
	m_dma_1->out_memw_callback().set(FUNC(qx16_state::dma_memory_write_byte));
	m_dma_2->in_memr_callback().set(FUNC(qx16_state::dma_memory_read_byte));
	m_dma_2->out_memw_callback().set(FUNC(qx16_state::dma_memory_write_byte));

	m_ppi->in_pb_callback().set(FUNC(qx16_state::qx16_portb_r));
	m_ppi->out_pc_callback().set(FUNC(qx16_state::qx16_portc_w));

	m_ram->set_default_size("512k");
	m_screen->set_screen_update(FUNC(qx16_state::screen_update));
}


/* ROM definition */
ROM_START( qx10 )
	ROM_REGION( 0x2000, "maincpu", ROMREGION_ERASEFF )
	ROM_SYSTEM_BIOS(0, "v006", "v0.06")
	ROMX_LOAD( "ipl006.bin", 0x0000, 0x0800, CRC(3155056a) SHA1(67cc0ae5055d472aa42eb40cddff6da69ffc6553), ROM_BIOS(0))
	ROM_RELOAD(0x800, 0x800)
	ROM_RELOAD(0x1000, 0x800)
	ROM_RELOAD(0x1800, 0x800)
	ROM_SYSTEM_BIOS(1, "v003", "v0.03")
	ROMX_LOAD( "ipl003.bin", 0x0000, 0x0800, CRC(3cbc4008) SHA1(cc8c7d1aa0cca8f9753d40698b2dc6802fd5f890), ROM_BIOS(1))
	ROM_RELOAD(0x800, 0x800)
	ROM_RELOAD(0x1000, 0x800)
	ROM_RELOAD(0x1800, 0x800)
ROM_END

ROM_START( qx16 )
	// Each BIOS index pairs a Z80 IPL with its matching 8088 BIOS.
	ROM_SYSTEM_BIOS(0, "v20a", "Z80 IPL v2.0a / 8088 BIOS v2.25")
	ROM_SYSTEM_BIOS(1, "v31h", "Z80 IPL v3.1h / 8088 BIOS v2.26h")
	ROM_SYSTEM_BIOS(2, "cfboot", "Z80 IPL v3.0A-CF / GlaBIOS v0.4.3")

	ROM_REGION( 0x2000, "maincpu", ROMREGION_ERASEFF )
	ROMX_LOAD( "ipl2.0a.bin", 0x0000, 0x2000, CRC(9bc5c643) SHA1(8970c3eb9bda3dd6bfc2a6f0172757a8d9ed32c9), ROM_BIOS(0))
	ROMX_LOAD( "ipl3.1h.bin", 0x0000, 0x2000, CRC(917da126) SHA1(a1439c0cd1b1c5a30ae08ed3d439acbfc1b0759b), ROM_BIOS(1))
	ROMX_LOAD( "ipl3.0a-cf.bin", 0x0000, 0x2000, CRC(c4ef78b3) SHA1(14b97469730b4c93caff29ff0c924569474d5150), ROM_BIOS(2))

	// 8088 BIOS: mapped to 0xc0000-0xfffff (mirrored across the full 256K window)
	ROM_REGION( 0x4000, "subcpu", ROMREGION_ERASEFF )
	ROMX_LOAD( "bios225.bin", 0x0000, 0x4000, CRC(bbc732fe) SHA1(833be3e636a9718a234af0d5c0a44f854593c543), ROM_BIOS(0))
	ROMX_LOAD( "bios226h.bin",  0x0000, 0x4000, CRC(3d5deb8e) SHA1(2999423882bd4b6e33fa2f40e1c2677bc103a79b), ROM_BIOS(1))
	ROMX_LOAD( "glabios-0.4.3.bin",  0x0000, 0x4000, CRC(7e082a0f) SHA1(cc28ceb9e4f62e2db7ca77df9ce5ff1d94d86b93), ROM_BIOS(2))
ROM_END

} // anonymous namespace


/* Driver */

/*    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT        COMPANY  FULLNAME  FLAGS */
COMP( 1983, qx10, 0,      0,      qx10,    qx10,  qx10_state, empty_init, "Epson", "QX-10",  MACHINE_NOT_WORKING )
COMP( 1985, qx16, 0,      0,      qx16,    qx16,  qx16_state, empty_init, "Epson", "QX-16",  0 )
