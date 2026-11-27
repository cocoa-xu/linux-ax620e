// SPDX-License-Identifier: GPL-2.0
/*
 * Lontium LT7911EXC USB-C/DisplayPort to MIPI CSI-2 bridge
 *
 * The bridge runs its own firmware from flash: it trains the DisplayPort
 * link, converts the video to CSI-2 YUV422 8-bit and reports the detected
 * timings and the audio sample rate through a register page. The host reads
 * that page on the interrupt the firmware raises when the video or the
 * audio appears or disappears. The firmware turns an RGB source into BT.709
 * at full range; the host may ask for BT.601 or the limited range instead
 * (V4L2_MBUS_FRAMEFMT_SET_CSC), which the driver sets again after every
 * new input.
 *
 * The EDID the bridge presents to the source is kept in the same SPI flash
 * as the firmware, in a 64 KiB block of its own, which the host reaches
 * through a flash controller on the same register page.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/v4l2-dv-timings.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

#include <uapi/linux/lt7911exc.h>

#define LT7911EXC_PAGE_SELECT		0xff

#define REG_FW_VERSION			CCI_REG24(0xe081)
#define REG_INT_TYPE			CCI_REG8(0xe084)
#define  INT_VIDEO_OFF			0x00
#define  INT_VIDEO_READY		0x01
#define  INT_AUDIO_OFF			0x02
#define  INT_AUDIO_READY		0x03
#define REG_PIXEL_CLOCK			CCI_REG24(0xe085)	/* kHz */
#define REG_H_TOTAL			CCI_REG16(0xe088)
#define REG_V_TOTAL			CCI_REG16(0xe08a)
#define REG_H_ACTIVE			CCI_REG16(0xe08c)
#define REG_V_ACTIVE			CCI_REG16(0xe08e)
#define REG_AUDIO_RATE			CCI_REG8(0xe0ad)	/* kHz */

/* SPI flash controller, programmed the way the vendor driver does. */
#define REG_FLASH_RUN			CCI_REG8(0xe051)
#define REG_FLASH_CMD			CCI_REG8(0xe054)
#define  FLASH_CMD_OPCODE		0x01
#define  FLASH_CMD_ADDRESS		0x05
#define  FLASH_CMD_READ			0x45
#define REG_FLASH_OPCODE		CCI_REG8(0xe055)
#define REG_FLASH_LENGTH		CCI_REG8(0xe056)
#define REG_FLASH_57			CCI_REG8(0xe057)
#define REG_FLASH_58			CCI_REG8(0xe058)
#define REG_FLASH_ADDR_H		CCI_REG8(0xe05a)
#define REG_FLASH_ADDR_M		CCI_REG8(0xe05b)
#define REG_FLASH_ADDR_L		CCI_REG8(0xe05c)
#define REG_FLASH_WDATA			0xe05d
#define REG_FLASH_RDATA			0xe05e
#define REG_FLASH_PROG			CCI_REG8(0xe05f)
#define  FLASH_PROG_IDLE		0x00
#define  FLASH_PROG_WRITE		0x01
#define  FLASH_PROG_FIFO_RESET		0x08

/*
 * While set, the host reaches the chip's own registers: the flash controller
 * above and the other pages, which read as zeros without it. The firmware's
 * registers on page 0xe0 answer either way.
 */
#define REG_HOST_ACCESS			CCI_REG8(0xe0ee)

/*
 * The converter from the source's RGB to YCbCr, which the firmware sets up
 * for every new input and leaves disabled when the source sends YCbCr.
 */
#define REG_CSC				CCI_REG8(0xc50c)
#define  CSC_MODE			GENMASK(7, 4)
#define  CSC_ENABLE			BIT(0)

#define SPI_NOR_WREN			0x06
#define SPI_NOR_READ			0x03
#define SPI_NOR_BLOCK_ERASE		0xd8	/* 64 KiB */

#define LT7911EXC_FLASH_PAGE		32
#define LT7911EXC_EDID_ADDR		0x60000
#define LT7911EXC_EDID_BLOCK_SIZE	128
#define LT7911EXC_EDID_MAX_BLOCKS	4
#define LT7911EXC_EDID_SIZE		(LT7911EXC_EDID_BLOCK_SIZE * \
					 LT7911EXC_EDID_MAX_BLOCKS)

#define LT7911EXC_LANES			4

static const struct v4l2_dv_timings_cap lt7911exc_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with CLANG */
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(160, 3840, 120, 2160, 25000000, 600000000,
			     V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			     V4L2_DV_BT_STD_CVT,
			     V4L2_DV_BT_CAP_PROGRESSIVE |
			     V4L2_DV_BT_CAP_CUSTOM |
			     V4L2_DV_BT_CAP_REDUCED_BLANKING)
};

struct lt7911exc_csc {
	enum v4l2_ycbcr_encoding ycbcr_enc;
	bool full_in;
	bool full_out;
};

/* The CSC modes the firmware knows, by REG_CSC's mode field. */
static const struct lt7911exc_csc lt7911exc_csc_modes[] = {
	[0x0] = { V4L2_YCBCR_ENC_601, false, false },
	[0x1] = { V4L2_YCBCR_ENC_601, false, true },
	[0x2] = { V4L2_YCBCR_ENC_709, false, false },
	[0x3] = { V4L2_YCBCR_ENC_709, false, true },
	[0x4] = { V4L2_YCBCR_ENC_BT2020, false, true },
	[0x6] = { V4L2_YCBCR_ENC_601, true, false },
	[0x7] = { V4L2_YCBCR_ENC_601, true, true },
	[0x8] = { V4L2_YCBCR_ENC_709, true, false },
	[0x9] = { V4L2_YCBCR_ENC_709, true, true },
	[0xa] = { V4L2_YCBCR_ENC_BT2020, true, true },
};

static const struct regmap_range_cfg lt7911exc_ranges[] = {
	{
		.name = "register_range",
		.range_min = 0,
		.range_max = 0xffff,
		.selector_reg = LT7911EXC_PAGE_SELECT,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt7911exc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = lt7911exc_ranges,
	.num_ranges = ARRAY_SIZE(lt7911exc_ranges),
};

static const struct v4l2_ctrl_config lt7911exc_ctrl_audio_rate = {
	.id = LT7911EXC_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt7911exc_ctrl_audio_present = {
	.id = LT7911EXC_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

struct lt7911exc {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *audio_rate;
	struct v4l2_ctrl *audio_present;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	int irq;
	/* Detected input timings; zero width when there is no signal. */
	struct v4l2_dv_timings detected;
	struct v4l2_dv_timings timings;
	/*
	 * The CSC mode found for the input and the one in effect, -1 when the
	 * bridge converts nothing; the encoding and the range asked for,
	 * *_DEFAULT for the firmware's. Protected by the active state's lock.
	 */
	int fw_csc;
	int csc;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;

	/* Serializes the host's register accesses and protects the EDID copy. */
	struct mutex access_lock;
	u8 edid[LT7911EXC_EDID_SIZE];
	/* Blocks in edid[], 0 until the flash has been read. */
	unsigned int edid_blocks;
};

static const struct v4l2_event lt7911exc_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static struct lt7911exc *to_lt7911exc(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt7911exc, sd);
}

static void lt7911exc_read_timings(struct lt7911exc *lt)
{
	struct v4l2_bt_timings *bt = &lt->detected.bt;
	u64 pclk, htotal, vtotal, width, height;
	int ret = 0;

	cci_read(lt->regmap, REG_PIXEL_CLOCK, &pclk, &ret);
	cci_read(lt->regmap, REG_H_TOTAL, &htotal, &ret);
	cci_read(lt->regmap, REG_V_TOTAL, &vtotal, &ret);
	cci_read(lt->regmap, REG_H_ACTIVE, &width, &ret);
	cci_read(lt->regmap, REG_V_ACTIVE, &height, &ret);

	memset(&lt->detected, 0, sizeof(lt->detected));
	lt->detected.type = V4L2_DV_BT_656_1120;
	if (ret || !pclk || !width || !height || htotal < width ||
	    vtotal < height)
		return;

	bt->width = width;
	bt->height = height;
	bt->hsync = htotal - width;
	bt->vsync = vtotal - height;
	bt->pixelclock = pclk * 1000;
	bt->interlaced = V4L2_DV_PROGRESSIVE;
}

static u32 lt7911exc_fps(const struct v4l2_bt_timings *bt)
{
	u32 total = V4L2_DV_BT_FRAME_WIDTH(bt) * V4L2_DV_BT_FRAME_HEIGHT(bt);

	return total ? div_u64(bt->pixelclock, total) : 0;
}

static s64 lt7911exc_pixel_rate(const struct v4l2_bt_timings *bt)
{
	/* 16 bits per pixel spread over the data lanes. */
	return div_u64((u64)bt->width * bt->height * lt7911exc_fps(bt) * 16,
		       LT7911EXC_LANES) ?: 1;
}

/* The firmware reports the rate in whole kHz. */
static u32 lt7911exc_audio_hz(u8 khz)
{
	switch (khz) {
	case 44:
		return 44100;
	case 88:
		return 88200;
	case 176:
		return 176400;
	default:
		return khz * 1000;
	}
}

static void lt7911exc_update_audio(struct lt7911exc *lt, bool present)
{
	u64 khz = 0;
	u32 rate = 0;

	if (present && !cci_read(lt->regmap, REG_AUDIO_RATE, &khz, NULL))
		rate = lt7911exc_audio_hz(khz);

	v4l2_ctrl_s_ctrl(lt->audio_rate, rate);
	v4l2_ctrl_s_ctrl(lt->audio_present, rate != 0);
}

static int lt7911exc_flash_cmd(struct lt7911exc *lt, u8 type, u8 opcode,
			       u32 addr, u8 len)
{
	int ret = 0;

	cci_write(lt->regmap, REG_FLASH_CMD, type, &ret);
	cci_write(lt->regmap, REG_FLASH_OPCODE, opcode, &ret);
	if (type == FLASH_CMD_READ) {
		cci_write(lt->regmap, REG_FLASH_LENGTH, len, &ret);
		cci_write(lt->regmap, REG_FLASH_57, 0, &ret);
		cci_write(lt->regmap, REG_FLASH_58, 0, &ret);
	}
	if (type != FLASH_CMD_OPCODE) {
		cci_write(lt->regmap, REG_FLASH_ADDR_H, (addr >> 16) & 0xff,
			  &ret);
		cci_write(lt->regmap, REG_FLASH_ADDR_M, (addr >> 8) & 0xff,
			  &ret);
		cci_write(lt->regmap, REG_FLASH_ADDR_L, addr & 0xff, &ret);
	}
	cci_write(lt->regmap, REG_FLASH_RUN, 1, &ret);
	cci_write(lt->regmap, REG_FLASH_RUN, 0, &ret);

	return ret;
}

static int lt7911exc_flash_read(struct lt7911exc *lt, u32 addr, u8 *buf,
				size_t len)
{
	size_t off;
	int ret = 0;

	cci_write(lt->regmap, REG_FLASH_PROG, FLASH_PROG_FIFO_RESET, &ret);
	cci_write(lt->regmap, REG_FLASH_PROG, FLASH_PROG_IDLE, &ret);
	if (ret)
		return ret;
	/* As long as the vendor driver waits after a FIFO reset. */
	msleep(1000);

	for (off = 0; off < len; off += LT7911EXC_FLASH_PAGE) {
		u8 n = min_t(size_t, len - off, LT7911EXC_FLASH_PAGE);

		ret = lt7911exc_flash_cmd(lt, FLASH_CMD_READ, SPI_NOR_READ,
					  addr + off, n);
		if (!ret)
			ret = regmap_bulk_read(lt->regmap, REG_FLASH_RDATA,
					       buf + off, n);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Erase the EDID block, and only that one: the firmware lives in block 0
 * of the same flash.
 */
static int lt7911exc_flash_erase_edid(struct lt7911exc *lt)
{
	int ret;

	ret = lt7911exc_flash_cmd(lt, FLASH_CMD_OPCODE, SPI_NOR_WREN, 0, 0);
	if (!ret)
		ret = lt7911exc_flash_cmd(lt, FLASH_CMD_ADDRESS,
					  SPI_NOR_BLOCK_ERASE,
					  LT7911EXC_EDID_ADDR, 0);
	if (ret)
		return ret;

	/*
	 * The controller offers no busy flag. The vendor driver waits 200 ms;
	 * a 64 KiB NOR block erase may take up to about 2 s.
	 */
	msleep(2000);

	return 0;
}

static int lt7911exc_flash_program(struct lt7911exc *lt, u32 addr,
				   const u8 *buf, size_t len)
{
	size_t off;
	int ret = 0;

	for (off = 0; off < len; off += LT7911EXC_FLASH_PAGE) {
		u32 page = addr + off;

		cci_write(lt->regmap, REG_HOST_ACCESS, 1, &ret);
		cci_write(lt->regmap, REG_FLASH_PROG, FLASH_PROG_WRITE, &ret);
		cci_write(lt->regmap, REG_FLASH_ADDR_H, (page >> 16) & 0xff,
			  &ret);
		cci_write(lt->regmap, REG_FLASH_ADDR_M, (page >> 8) & 0xff,
			  &ret);
		cci_write(lt->regmap, REG_FLASH_ADDR_L, page & 0xff, &ret);
		if (!ret)
			ret = regmap_raw_write(lt->regmap, REG_FLASH_WDATA,
					       buf + off, LT7911EXC_FLASH_PAGE);
		cci_write(lt->regmap, REG_FLASH_PROG, FLASH_PROG_IDLE, &ret);
		if (ret)
			return ret;
	}

	return 0;
}

/* Ends the host's register access, also after a failed one. */
static int lt7911exc_end_host_access(struct lt7911exc *lt, int ret)
{
	int err = cci_write(lt->regmap, REG_HOST_ACCESS, 0, NULL);

	return ret ?: err;
}

static unsigned int lt7911exc_edid_blocks(const u8 *edid)
{
	static const u8 header[] = { 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0 };

	if (memcmp(edid, header, sizeof(header)))
		return 0;

	return min_t(unsigned int, edid[126] + 1, LT7911EXC_EDID_MAX_BLOCKS);
}

static bool lt7911exc_edid_valid(const u8 *edid, unsigned int blocks)
{
	unsigned int i;
	u8 sum = 0;

	if (lt7911exc_edid_blocks(edid) == 0)
		return false;

	for (i = 0; i < blocks * LT7911EXC_EDID_BLOCK_SIZE; i++) {
		sum += edid[i];
		if (i % LT7911EXC_EDID_BLOCK_SIZE ==
		    LT7911EXC_EDID_BLOCK_SIZE - 1 && sum)
			return false;
	}

	return true;
}

/* Called with access_lock held and the interrupt disabled. */
static int lt7911exc_load_edid(struct lt7911exc *lt)
{
	int ret = 0;

	if (lt->edid_blocks)
		return 0;

	cci_write(lt->regmap, REG_HOST_ACCESS, 1, &ret);
	if (!ret)
		ret = lt7911exc_flash_read(lt, LT7911EXC_EDID_ADDR, lt->edid,
					   sizeof(lt->edid));
	ret = lt7911exc_end_host_access(lt, ret);
	if (ret)
		return ret;

	lt->edid_blocks = lt7911exc_edid_blocks(lt->edid);

	return lt->edid_blocks ? 0 : -ENODATA;
}

/* Called with access_lock held and the interrupt disabled. */
static int lt7911exc_store_edid(struct lt7911exc *lt, const u8 *edid,
				unsigned int blocks)
{
	size_t len = blocks * LT7911EXC_EDID_BLOCK_SIZE;
	int ret = 0;

	lt->edid_blocks = 0;

	cci_write(lt->regmap, REG_HOST_ACCESS, 1, &ret);
	if (!ret)
		ret = lt7911exc_flash_erase_edid(lt);
	if (!ret)
		ret = lt7911exc_flash_program(lt, LT7911EXC_EDID_ADDR, edid,
					      len);
	if (!ret)
		ret = lt7911exc_flash_read(lt, LT7911EXC_EDID_ADDR, lt->edid,
					   sizeof(lt->edid));
	ret = lt7911exc_end_host_access(lt, ret);
	if (ret)
		return ret;

	if (memcmp(lt->edid, edid, len)) {
		dev_err(lt->sd.dev, "EDID verification failed\n");
		return -EIO;
	}

	lt->edid_blocks = blocks;

	return 0;
}

static void lt7911exc_flash_begin(struct lt7911exc *lt)
{
	/* The firmware must not see the host in the middle of an access. */
	if (lt->irq > 0)
		disable_irq(lt->irq);
	mutex_lock(&lt->access_lock);
}

static void lt7911exc_flash_end(struct lt7911exc *lt)
{
	mutex_unlock(&lt->access_lock);
	if (lt->irq > 0)
		enable_irq(lt->irq);
}

static int lt7911exc_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	int ret;

	memset(edid->reserved, 0, sizeof(edid->reserved));
	if (edid->pad)
		return -EINVAL;

	lt7911exc_flash_begin(lt);
	ret = lt7911exc_load_edid(lt);
	if (ret)
		goto out;

	if (!edid->start_block && !edid->blocks) {
		edid->blocks = lt->edid_blocks;
		goto out;
	}
	if (edid->start_block >= lt->edid_blocks || !edid->blocks) {
		ret = -EINVAL;
		goto out;
	}

	edid->blocks = min(edid->blocks, lt->edid_blocks - edid->start_block);
	memcpy(edid->edid,
	       lt->edid + edid->start_block * LT7911EXC_EDID_BLOCK_SIZE,
	       edid->blocks * LT7911EXC_EDID_BLOCK_SIZE);
out:
	lt7911exc_flash_end(lt);

	return ret;
}

static int lt7911exc_set_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	int ret;

	memset(edid->reserved, 0, sizeof(edid->reserved));
	if (edid->pad || edid->start_block)
		return -EINVAL;
	if (edid->blocks > LT7911EXC_EDID_MAX_BLOCKS) {
		edid->blocks = LT7911EXC_EDID_MAX_BLOCKS;
		return -E2BIG;
	}
	/* Erasing the EDID would leave the source with nothing to read. */
	if (!edid->blocks || !lt7911exc_edid_valid(edid->edid, edid->blocks))
		return -EINVAL;

	lt7911exc_flash_begin(lt);
	ret = lt7911exc_store_edid(lt, edid->edid, edid->blocks);
	lt7911exc_flash_end(lt);

	if (!ret)
		dev_info(sd->dev,
			 "stored a %u block EDID; the source reads it when it reconnects\n",
			 edid->blocks);

	return ret;
}

/* The CSC mode the bridge uses, -1 when it converts nothing. */
static int lt7911exc_read_csc(struct lt7911exc *lt)
{
	u64 val = 0;
	int ret = 0;

	mutex_lock(&lt->access_lock);
	cci_write(lt->regmap, REG_HOST_ACCESS, 1, &ret);
	cci_read(lt->regmap, REG_CSC, &val, &ret);
	ret = lt7911exc_end_host_access(lt, ret);
	mutex_unlock(&lt->access_lock);

	if (ret || !(val & CSC_ENABLE) ||
	    FIELD_GET(CSC_MODE, val) >= ARRAY_SIZE(lt7911exc_csc_modes) ||
	    !lt7911exc_csc_modes[FIELD_GET(CSC_MODE, val)].ycbcr_enc)
		return -1;

	return FIELD_GET(CSC_MODE, val);
}

/*
 * The CSC mode for the encoding and the range out asked for, the range in
 * being the firmware's, which follows the source. The firmware's own choice
 * is BT.709 at full range; the bridge outlives the host, so its register may
 * hold what an earlier one asked for instead.
 */
static int lt7911exc_wanted_csc(struct lt7911exc *lt,
				enum v4l2_ycbcr_encoding ycbcr_enc,
				enum v4l2_quantization quantization)
{
	bool full_in, full_out = quantization != V4L2_QUANTIZATION_LIM_RANGE;
	unsigned int i;

	if (lt->fw_csc < 0)
		return -1;

	full_in = lt7911exc_csc_modes[lt->fw_csc].full_in;
	if (ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT)
		ycbcr_enc = V4L2_YCBCR_ENC_709;

	for (i = 0; i < ARRAY_SIZE(lt7911exc_csc_modes); i++)
		if (lt7911exc_csc_modes[i].ycbcr_enc == ycbcr_enc &&
		    lt7911exc_csc_modes[i].full_in == full_in &&
		    lt7911exc_csc_modes[i].full_out == full_out)
			return i;

	return lt->fw_csc;
}

static void lt7911exc_set_csc(struct lt7911exc *lt, int csc)
{
	int ret = 0;

	if (csc < 0 || csc == lt->csc)
		return;

	mutex_lock(&lt->access_lock);
	cci_write(lt->regmap, REG_HOST_ACCESS, 1, &ret);
	cci_update_bits(lt->regmap, REG_CSC, CSC_MODE,
			FIELD_PREP(CSC_MODE, csc), &ret);
	ret = lt7911exc_end_host_access(lt, ret);
	mutex_unlock(&lt->access_lock);

	if (ret)
		dev_err(lt->sd.dev, "failed to set the CSC: %d\n", ret);
	else
		lt->csc = csc;
}

static void lt7911exc_fill_format(struct lt7911exc *lt, int csc,
				  struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = lt->detected.bt.width;
	fmt->height = lt->detected.bt.height;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_REC709;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	if (csc < 0) {
		/* The source's own YCbCr, which the bridge does not describe. */
		fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	} else {
		fmt->ycbcr_enc = lt7911exc_csc_modes[csc].ycbcr_enc;
		fmt->quantization = lt7911exc_csc_modes[csc].full_out ?
				    V4L2_QUANTIZATION_FULL_RANGE :
				    V4L2_QUANTIZATION_LIM_RANGE;
	}
}

/* Stores the format of the input and the CSC mode in effect. */
static void lt7911exc_refresh(struct lt7911exc *lt,
			      struct v4l2_subdev_state *state)
{
	s64 rate = lt7911exc_pixel_rate(&lt->detected.bt);

	lt7911exc_fill_format(lt, lt->csc,
			      v4l2_subdev_state_get_format(state, 0));
	__v4l2_ctrl_modify_range(lt->pixel_rate, rate, rate, 1, rate);
}

static int lt7911exc_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *fmt)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	enum v4l2_ycbcr_encoding ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	enum v4l2_quantization quantization = V4L2_QUANTIZATION_DEFAULT;
	int csc;

	if (fmt->format.flags & V4L2_MBUS_FRAMEFMT_SET_CSC) {
		if (fmt->format.ycbcr_enc == V4L2_YCBCR_ENC_601 ||
		    fmt->format.ycbcr_enc == V4L2_YCBCR_ENC_709)
			ycbcr_enc = fmt->format.ycbcr_enc;
		if (fmt->format.quantization == V4L2_QUANTIZATION_FULL_RANGE ||
		    fmt->format.quantization == V4L2_QUANTIZATION_LIM_RANGE)
			quantization = fmt->format.quantization;
	}

	csc = lt7911exc_wanted_csc(lt, ycbcr_enc, quantization);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		lt7911exc_fill_format(lt, csc, &fmt->format);
		*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
		return 0;
	}

	lt->ycbcr_enc = ycbcr_enc;
	lt->quantization = quantization;
	lt7911exc_set_csc(lt, csc);
	lt7911exc_refresh(lt, state);
	lt7911exc_fill_format(lt, lt->csc, &fmt->format);

	return 0;
}

static int lt7911exc_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	code->flags = V4L2_SUBDEV_MBUS_CODE_CSC_YCBCR_ENC |
		      V4L2_SUBDEV_MBUS_CODE_CSC_QUANTIZATION;

	return 0;
}

static int lt7911exc_s_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (!v4l2_valid_dv_timings(timings, &lt7911exc_timings_cap, NULL, NULL))
		return -ERANGE;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	/* The firmware follows the source: only the detected mode is valid. */
	if (!v4l2_match_dv_timings(&lt->detected, timings, 0, false))
		ret = -EINVAL;
	else
		lt->timings = *timings;
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int lt7911exc_g_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	*timings = lt->timings;
	v4l2_subdev_unlock_state(state);

	return 0;
}

static int lt7911exc_query_dv_timings(struct v4l2_subdev *sd, unsigned int pad,
				      struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt = to_lt7911exc(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	*timings = lt->detected;
	if (!timings->bt.width)
		ret = -ENOLINK;
	else if (!v4l2_valid_dv_timings(timings, &lt7911exc_timings_cap,
					NULL, NULL))
		ret = -ERANGE;
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int lt7911exc_enum_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &lt7911exc_timings_cap,
					NULL, NULL);
}

static int lt7911exc_dv_timings_cap(struct v4l2_subdev *sd,
				    struct v4l2_dv_timings_cap *cap)
{
	*cap = lt7911exc_timings_cap;

	return 0;
}

static int lt7911exc_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				     struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = LT7911EXC_LANES;

	return 0;
}

static int lt7911exc_enable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state, u32 pad,
				    u64 streams_mask)
{
	return 0;
}

static int lt7911exc_disable_streams(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state, u32 pad,
				     u64 streams_mask)
{
	return 0;
}

static int lt7911exc_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = state ? V4L2_SUBDEV_FORMAT_TRY :
				 V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	return lt7911exc_set_format(sd, state, &fmt);
}

static const struct v4l2_subdev_core_ops lt7911exc_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops lt7911exc_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops lt7911exc_pad_ops = {
	.set_fmt = lt7911exc_set_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.enum_mbus_code = lt7911exc_enum_mbus_code,
	.enable_streams = lt7911exc_enable_streams,
	.disable_streams = lt7911exc_disable_streams,
	.get_edid = lt7911exc_get_edid,
	.set_edid = lt7911exc_set_edid,
	.s_dv_timings = lt7911exc_s_dv_timings,
	.g_dv_timings = lt7911exc_g_dv_timings,
	.query_dv_timings = lt7911exc_query_dv_timings,
	.enum_dv_timings = lt7911exc_enum_dv_timings,
	.dv_timings_cap = lt7911exc_dv_timings_cap,
	.get_mbus_config = lt7911exc_get_mbus_config,
};

static const struct v4l2_subdev_ops lt7911exc_subdev_ops = {
	.core = &lt7911exc_core_ops,
	.video = &lt7911exc_video_ops,
	.pad = &lt7911exc_pad_ops,
};

static const struct v4l2_subdev_internal_ops lt7911exc_internal_ops = {
	.init_state = lt7911exc_init_state,
};

static const struct media_entity_operations lt7911exc_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* Reads the CSC mode the firmware set up for a new input. */
static void lt7911exc_read_fw_csc(struct lt7911exc *lt)
{
	lt->fw_csc = lt->detected.bt.width ? lt7911exc_read_csc(lt) : -1;
	lt->csc = lt->fw_csc;
}

static void lt7911exc_update(struct lt7911exc *lt)
{
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(&lt->sd);
	lt7911exc_read_timings(lt);
	lt->timings = lt->detected;
	/* The firmware set the CSC up before it raised the interrupt. */
	lt7911exc_read_fw_csc(lt);
	lt7911exc_set_csc(lt, lt7911exc_wanted_csc(lt, lt->ycbcr_enc,
						   lt->quantization));
	lt7911exc_refresh(lt, state);
	v4l2_subdev_unlock_state(state);

	dev_info(lt->sd.dev, "input %ux%u@%u\n", lt->detected.bt.width,
		 lt->detected.bt.height, lt7911exc_fps(&lt->detected.bt));
	v4l2_subdev_notify_event(&lt->sd, &lt7911exc_ev_source_change);
}

static irqreturn_t lt7911exc_irq(int irq, void *data)
{
	struct lt7911exc *lt = data;
	u64 type;
	int ret;

	ret = cci_read(lt->regmap, REG_INT_TYPE, &type, NULL);
	if (ret)
		return IRQ_NONE;

	switch (type) {
	case INT_VIDEO_OFF:
	case INT_VIDEO_READY:
		lt7911exc_update(lt);
		break;
	case INT_AUDIO_OFF:
		lt7911exc_update_audio(lt, false);
		break;
	case INT_AUDIO_READY:
		lt7911exc_update_audio(lt, true);
		break;
	}

	return IRQ_HANDLED;
}

static int lt7911exc_check_endpoint(struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!fwnode)
		return dev_err_probe(dev, -EINVAL, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return dev_err_probe(dev, ret, "invalid endpoint\n");
	if (ep.bus.mipi_csi2.num_data_lanes != LT7911EXC_LANES)
		return dev_err_probe(dev, -EINVAL,
				     "only 4 data lanes are supported\n");

	return 0;
}

static int lt7911exc_init_controls(struct lt7911exc *lt)
{
	struct v4l2_ctrl_handler *hdl = &lt->ctrl_handler;
	s64 rate = lt7911exc_pixel_rate(&lt->detected.bt);

	v4l2_ctrl_handler_init(hdl, 3);
	lt->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, 1,
					   INT_MAX, 1, rate);
	lt->audio_rate = v4l2_ctrl_new_custom(hdl, &lt7911exc_ctrl_audio_rate,
					      NULL);
	lt->audio_present = v4l2_ctrl_new_custom(hdl,
						 &lt7911exc_ctrl_audio_present,
						 NULL);
	if (hdl->error) {
		int ret = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	lt->sd.ctrl_handler = hdl;

	/* The interrupt for audio that started before probe has been missed. */
	lt7911exc_update_audio(lt, true);

	return 0;
}

static void lt7911exc_cleanup(void *data)
{
	struct lt7911exc *lt = data;

	v4l2_subdev_cleanup(&lt->sd);
	media_entity_cleanup(&lt->sd.entity);
	v4l2_ctrl_handler_free(&lt->ctrl_handler);
}

static int lt7911exc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lt7911exc *lt;
	u64 version;
	int ret;

	lt = devm_kzalloc(dev, sizeof(*lt), GFP_KERNEL);
	if (!lt)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &lt->access_lock);
	if (ret)
		return ret;

	lt->regmap = devm_regmap_init_i2c(client, &lt7911exc_regmap_config);
	if (IS_ERR(lt->regmap))
		return dev_err_probe(dev, PTR_ERR(lt->regmap),
				     "failed to init regmap\n");

	ret = lt7911exc_check_endpoint(dev);
	if (ret)
		return ret;

	/*
	 * Leave the bridge running: resetting it drops the DisplayPort link,
	 * and the source does not always retrain it.
	 */
	lt->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lt->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(lt->reset_gpio),
				     "failed to get reset gpio\n");

	ret = cci_read(lt->regmap, REG_FW_VERSION, &version, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "no response from the bridge\n");

	v4l2_i2c_subdev_init(&lt->sd, client, &lt7911exc_subdev_ops);
	lt->sd.internal_ops = &lt7911exc_internal_ops;
	lt->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	lt->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	lt->sd.entity.ops = &lt7911exc_entity_ops;
	lt->pad.flags = MEDIA_PAD_FL_SOURCE;

	lt7911exc_read_timings(lt);
	lt->timings = lt->detected;
	lt7911exc_read_fw_csc(lt);
	lt7911exc_set_csc(lt, lt7911exc_wanted_csc(lt, lt->ycbcr_enc,
						   lt->quantization));

	ret = lt7911exc_init_controls(lt);
	if (ret)
		return ret;

	ret = media_entity_pads_init(&lt->sd.entity, 1, &lt->pad);
	if (ret) {
		v4l2_ctrl_handler_free(&lt->ctrl_handler);
		return ret;
	}

	ret = v4l2_subdev_init_finalize(&lt->sd);
	if (ret) {
		media_entity_cleanup(&lt->sd.entity);
		v4l2_ctrl_handler_free(&lt->ctrl_handler);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, lt7911exc_cleanup, lt);
	if (ret)
		return ret;

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						lt7911exc_irq, IRQF_ONESHOT,
						dev_name(dev), lt);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request irq\n");
		lt->irq = client->irq;
	}

	ret = v4l2_async_register_subdev(&lt->sd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register subdev\n");

	dev_info(dev, "firmware %06llx, input %ux%u@%u, audio %d Hz\n", version,
		 lt->detected.bt.width, lt->detected.bt.height,
		 lt7911exc_fps(&lt->detected.bt),
		 v4l2_ctrl_g_ctrl(lt->audio_rate));

	return 0;
}

static void lt7911exc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
}

static const struct of_device_id lt7911exc_of_match[] = {
	{ .compatible = "lontium,lt7911exc" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt7911exc_of_match);

static struct i2c_driver lt7911exc_driver = {
	.driver = {
		.name = "lt7911exc",
		.of_match_table = lt7911exc_of_match,
	},
	.probe = lt7911exc_probe,
	.remove = lt7911exc_remove,
};
module_i2c_driver(lt7911exc_driver);

MODULE_DESCRIPTION("Lontium LT7911EXC USB-C/DP to MIPI CSI-2 bridge driver");
MODULE_LICENSE("GPL");
