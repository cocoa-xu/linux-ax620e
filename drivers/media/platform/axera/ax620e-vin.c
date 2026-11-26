// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E video input
 *
 * MIPI D-PHY, CSI-2 receiver and the ISP front end (IFE) in YUV bypass mode:
 * the receiver takes CSI-2 YUV422 8-bit and the IFE write DMA stores it as
 * packed YUYV. None of the ISP processing blocks is used.
 *
 * The register sequences were recovered from the vendor driver at run time;
 * where the meaning of a value is unknown it is kept as the vendor wrote it.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define AX_VIN_NAME			"ax620e-vin"

/* ISP_SYS_GLB: clocks and resets of the ISP subsystem, SET/CLR pairs */
#define GLB_PIPE_CFG			0x090
#define GLB_PIPE_EN			0x094
#define GLB_CLK_SEL_SET			0x0c8
#define GLB_CLK_SEL_CLR			0x0cc
#define GLB_CLK_SEL_CSIRX_CFG		GENMASK(1, 0)
/*
 * ISP core and bus clock sources as the vendor ISP driver programs them. The
 * reset value selects 24 MHz: the pipeline then runs at a few frames per
 * second and drops most of each frame.
 */
#define GLB_CLK_SEL_ISP			GENMASK(12, 2)
#define GLB_CLK_SEL_ISP_RUN		0x5ac
#define GLB_EB0				0x004
#define GLB_EB0_ISP			GENMASK(5, 2)
#define GLB_EB0_SET			0x0d0
#define GLB_EB0_CLR			0x0d4
#define GLB_EB0_CFG_PHY			BIT(0)
#define GLB_EB0_DPHY_RX_REF		BIT(1)
#define GLB_EB0_ALL			GENMASK(5, 0)
#define GLB_EB1_SET			0x0d8
#define GLB_EB1_CLR			0x0dc
#define GLB_EB1_SYS_PIXEL		BIT(4)
#define GLB_EB1_CSIRX_PCLK		BIT(7)
#define GLB_EB1_ALL			GENMASK(9, 1)
#define GLB_RST0_ASSERT			0x0e0
#define GLB_RST0_DEASSERT		0x0e4
#define GLB_RST0_CSIRX_PIXEL		BIT(2)
#define GLB_RST0_CSIRX_PPI_BYTE		BIT(3)
#define GLB_RST0_CSIRX_PRST		BIT(4)
#define GLB_RST0_CSIRX_SYS		BIT(5)
#define GLB_RST0_DESKEW			(BIT(10) | BIT(11))
#define GLB_RST0_CSIRX			(GLB_RST0_CSIRX_PIXEL | \
					 GLB_RST0_CSIRX_PPI_BYTE | \
					 GLB_RST0_CSIRX_PRST | \
					 GLB_RST0_CSIRX_SYS | GLB_RST0_DESKEW)
/* What the vendor driver pulses to recover from receive errors. */
#define GLB_RST0_CSIRX_RECOVER		(GLB_RST0_CSIRX & ~GLB_RST0_CSIRX_PRST)
#define GLB_RST0_DPHYRX			BIT(12)
#define GLB_RST0_ALL			GENMASK(31, 0)
/* The vendor driver never pulses bit 27. */
#define GLB_RST0_PULSE			(GLB_RST0_ALL & ~BIT(27))
#define GLB_RST1_ASSERT			0x0e8
#define GLB_RST1_DEASSERT		0x0ec
#define GLB_RST1_ALL			(GENMASK(19, 0) | BIT(21) | BIT(22))
#define GLB_CSI_SEL_SET			0x218
#define GLB_CSI_SEL_CLR			0x21c

/* COMM_GLB (shared syscon): D-PHY RX power and its TLB clock */
#define COMM_CLK_EB_SET			0x028
#define COMM_CLK_EB_DPHYRX_TLB		BIT(9)
#define COMM_RST_DEASSERT		0x05c
#define COMM_RST_DPHYRX_TLB		BIT(7)
#define COMM_DPHY_PWR_OFF_SET		0x1ec
#define COMM_DPHY_PWR_OFF_CLR		0x1f0
#define COMM_DPHY_PWR_READY_SET		0x1f8
#define COMM_DPHY_PWR_READY_CLR		0x1fc

/* D-PHY RX */
#define DPHY_GLB_CFG_LAST		0x030
#define DPHY_LANE_SEL_SET		0x0c8
#define DPHY_LANE_SEL_CLR		0x0cc
#define DPHY_LANE_SEL_DPDN_SWAP		GENMASK(5, 0)
#define DPHY_LANE_SEL_DATABUS16		BIT(24)
#define DPHY_LANE_SEL_1D2C		BIT(25)
#define DPHY_HSRX_CFG0			0x0d0
#define DPHY_HSRX_CFG1			0x0d4
#define DPHY_HSRX_PRE_SET		0x0d8
#define DPHY_HSRX_PRE_CLR		0x0dc
#define DPHY_DESKEW_RST_SET		0x0e8
#define DPHY_DESKEW_RST			GENMASK(13, 10)
#define DPHY_LANE_EN_SET		0x110
#define DPHY_LANE_EN_CLR		0x114
#define DPHY_LANE_EN_ALL		GENMASK(5, 0)
#define DPHY_LANE_EN_DATA		GENMASK(5, 2)

/* CSI-2 controller */
#define CSI_RX_SOFT_RST			0x004
/* Error interrupts: status (write 1 to clear) and enable */
#define CSI_ERR_STATUS			0x028
#define CSI_ERR_EN			0x02c
#define CSI_ERR_ALL			0x00011ee1
#define CSI_ERR_FIFO_OVERFLOW		BIT(16)
#define CSI_ERR_TRUNC_HEADER		BIT(12)
#define CSI_ERR_TRUNC_PACKET		BIT(11)
#define CSI_ERR_RECOVER			(CSI_ERR_FIFO_OVERFLOW | \
					 CSI_ERR_TRUNC_HEADER | \
					 CSI_ERR_TRUNC_PACKET)
/* D-PHY errors (start of transmission per lane) */
#define CSI_PHY_ERR_STATUS		0x04c
#define CSI_PHY_ERR_EN			0x050
#define CSI_PHY_ERR_ALL			0x00111100
#define CSI_STREAM_CTRL			0x100
#define CSI_STREAM_START		BIT(0)
#define CSI_STREAM_STOP			BIT(1)
#define CSI_STREAM_SOFT_RST		BIT(4)

/* ISP top: ten interrupt groups of enable / W1C clear / raw / masked status */
#define ISP_IRQ_EN(g)			(0x10 * (g))
#define ISP_IRQ_CLR(g)			(0x10 * (g) + 0x4)
#define ISP_IRQ_STATUS(g)		(0x10 * (g) + 0xc)
#define ISP_IRQ_GROUPS			10
#define ISP_IRQ_SOF_GROUP		2
#define ISP_IRQ_SOF			BIT(0)
#define ISP_IRQ_EOF_GROUP		5
#define ISP_IRQ_EOF			BIT(9)
#define ISP_IRQ_MASK			0x150
/*
 * Pipeline blocks a frame goes through, latched at its start; the vendor's
 * last command list before a stop clears it.
 */
#define ISP_FRAME_EN			0x178
/* AXI port of the IFE (write DMA): writing all ones stops it. */
#define ISP_AXI_CTRL			0x184
#define ISP_AXI_STOP			GENMASK(31, 0)
/* Status after each AXI control register: nonzero while writes are pending. */
#define ISP_AXI_STATUS			0x188
#define ISP_START			0x1b8
/* Processing blocks, unused in bypass: only reset and interrupt masks. */
#define ISP_SUB1			0x80000
#define ISP_SUB2			0xc0000
#define ISP_SUB_IRQ_GROUPS		8

/* IFE input */
#define IFE				0x6400
#define IFE_START			(IFE + 0x004)
#define IFE_CTRL			(IFE + 0x008)
/* Stop request of a channel: the IFE drops the frame it is in. */
#define IFE_CTRL_STOP(ch)		BIT(ch)
#define IFE_IN_EN			(IFE + 0x00c)
#define IFE_UPDATE0			(IFE + 0x100)
#define IFE_UPDATE1			(IFE + 0x104)
#define IFE_IN_SIZE			(IFE + 0x118)
/* CSI-2 data type of the lines the IFE takes: it drops all others. */
#define IFE_IN_DT0			(IFE + 0x140)
#define IFE_IN_DT1			(IFE + 0x144)
#define IFE_IN_DT2			(IFE + 0x14c)
#define IFE_IN_DT			GENMASK(13, 8)

/* IFE write DMA */
#define WDMA				0x14000
#define WDMA_ARM			(WDMA + 0x0cc)
#define WDMA_FRAME_ID			(WDMA + 0x0d0)
#define WDMA_FRAME_ID_LATCHED		GENMASK(31, 16)
#define WDMA_ADDR			(WDMA + 0x0d4)
#define WDMA_SIZE			(WDMA + 0x2ec)
#define WDMA_STRIDE0			(WDMA + 0x2f0)
#define WDMA_STRIDE1			(WDMA + 0x2f4)
#define WDMA_START			(WDMA + 0x6dc)

#define AX_VIN_MIN_SIZE			64
#define AX_VIN_MAX_WIDTH		3840
#define AX_VIN_MAX_HEIGHT		2160
#define AX_VIN_STRIDE_ALIGN		64
/* After the IFE stop, as long as the slowest frame. */
#define AX_VIN_STOP_SETTLE_MS		50

struct ax_vin_reg {
	u32 offset;
	u32 value;
};

static const struct ax_vin_reg ax_vin_csi_cfg[] = {
	{ 0x008, 0x43210410 }, { 0x040, 0x0000001f }, { 0x01c, 0x00000007 },
	{ 0x024, 0x0000006f }, { 0x10c, 0x00000301 }, { 0x110, 0x00000010 },
	{ 0x118, 0x00000d1f },
};

/* IFE input settings besides size and data type. */
static const struct ax_vin_reg ax_vin_ife_cfg[] = {
	{ IFE + 0x110, 0x00000040 }, { IFE_IN_EN, 0x00000001 },
	{ IFE + 0x114, 0x00000000 }, { IFE + 0x124, 0x00000000 },
	{ IFE + 0x13c, 0x00000001 }, { IFE + 0x148, 0x3a3a3a00 },
	{ IFE + 0x11c, 0x00200020 },
};

/* Rewritten before every frame, as the vendor command lists do. */
static const struct ax_vin_reg ax_vin_frame_cfg[] = {
	{ 0x154, 0x00000000 }, { 0x158, 0x00008007 }, { 0x200, 0x00000000 },
	{ 0x170, 0x00000030 }, { 0x174, 0x00000000 }, { 0x1a4, 0x57fffbbf },
	{ ISP_FRAME_EN, 0x0000000c },
	{ WDMA + 0x2fc, 0x00000004 }, { WDMA + 0x2f8, 0x00000000 },
	{ WDMA + 0x0d8, 0x000f951f }, { WDMA + 0x6e0, 0xffffffff },
	{ WDMA + 0x638, 0x0003fe00 }, { WDMA + 0x528, 0x00000001 },
	{ WDMA + 0x508, 0x00000001 }, { WDMA + 0x588, 0x00000011 },
	{ WDMA + 0x578, 0x00000001 }, { WDMA + 0x0dc, 0x00000001 },
};

struct ax_vin_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	u16 frame_id;
};

struct ax_vin {
	struct device *dev;
	void __iomem *isp;
	void __iomem *glb;
	void __iomem *csi;
	void __iomem *dphy;
	struct regmap *comm;
	int irq;
	/* CSI-2 receive errors, as read by the interrupt for its thread */
	int csi_irq;
	u32 csi_err;
	u32 csi_phy_err;
	unsigned int csi_recoveries;

	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	/* The CSI-2 transmitter feeding the receiver, once bound. */
	struct v4l2_subdev *source;
	/* The source's controls, offered on the video node. */
	struct v4l2_ctrl_handler ctrl_handler;
	struct video_device vdev;
	struct vb2_queue queue;
	/* Serializes the ioctls and the vb2 queue. */
	struct mutex lock;

	/* Protects queued, armed, active, sequence and frame_id. */
	spinlock_t qlock;
	struct list_head queued;
	/* Programmed into the DMA, latched at the next frame start. */
	struct ax_vin_buffer *armed;
	/* Being written by the DMA. */
	struct ax_vin_buffer *active;
	u32 active_sequence;
	u32 sequence;
	u16 frame_id;
	/*
	 * Stopping: arm nothing new and stop the receiver once the DMA is
	 * idle. After a reset in the middle of a frame, every later frame
	 * comes out shifted by a fixed offset until the SoC is reset.
	 */
	bool stopping;
	bool idle;
	wait_queue_head_t idle_wq;
	unsigned int mid_frame_stops;

	struct v4l2_pix_format fmt;
};

static struct ax_vin_buffer *to_ax_vin_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct ax_vin_buffer, vb);
}

static void ax_vin_write_regs(void __iomem *base, const struct ax_vin_reg *regs,
			      unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		writel(regs[i].value, base + regs[i].offset);
}

static void ax_vin_field(void __iomem *base, u32 set, u32 clr, u32 mask,
			 u32 value)
{
	writel(mask, base + clr);
	writel(value, base + set);
}

static void ax_vin_pulse_resets(void __iomem *glb, unsigned int assert,
				unsigned int deassert, unsigned long mask)
{
	unsigned int bit;

	/* One at a time, in the vendor driver's order. */
	for_each_set_bit(bit, &mask, BITS_PER_LONG) {
		writel(BIT(bit), glb + assert);
		writel(BIT(bit), glb + deassert);
	}
}

static void ax_vin_isp_reset(struct ax_vin *vin)
{
	static const u32 axi_ctrl[] = {
		ISP_AXI_CTRL, ISP_SUB1 + 0x144, ISP_SUB2 + 0x148,
	};
	void __iomem *glb = vin->glb, *isp = vin->isp;
	unsigned int i;
	u32 status;

	ax_vin_field(glb, GLB_CLK_SEL_SET, GLB_CLK_SEL_CLR, GLB_CLK_SEL_ISP,
		     GLB_CLK_SEL_ISP_RUN);
	writel(GLB_EB0_ALL, glb + GLB_EB0_SET);
	writel(GLB_EB1_ALL, glb + GLB_EB1_SET);
	writel(GLB_RST0_ALL, glb + GLB_RST0_DEASSERT);
	writel(GLB_RST1_ALL, glb + GLB_RST1_DEASSERT);

	for (i = 0; i < ARRAY_SIZE(axi_ctrl); i++)
		writel(ISP_AXI_STOP, isp + axi_ctrl[i]);
	for (i = 0; i < ARRAY_SIZE(axi_ctrl); i++)
		if (readl_poll_timeout(isp + axi_ctrl[i] + 4, status, !status,
				       200, 10 * USEC_PER_MSEC))
			dev_warn(vin->dev, "AXI port %#x still busy (%#x)\n",
				 axi_ctrl[i], status);
	ax_vin_pulse_resets(glb, GLB_RST0_ASSERT, GLB_RST0_DEASSERT,
			    GLB_RST0_PULSE);
	ax_vin_pulse_resets(glb, GLB_RST1_ASSERT, GLB_RST1_DEASSERT,
			    GLB_RST1_ALL);
	for (i = 0; i < ARRAY_SIZE(axi_ctrl); i++)
		writel(0, isp + axi_ctrl[i]);

	for (i = 1; i <= ISP_IRQ_GROUPS; i++)
		writel(0, isp + ISP_IRQ_EN(i));
	for (i = 1; i <= ISP_SUB_IRQ_GROUPS; i++) {
		writel(0, isp + ISP_SUB1 + ISP_IRQ_EN(i));
		writel(0, isp + ISP_SUB2 + ISP_IRQ_EN(i));
	}
	writel(0xffffffff, isp + ISP_IRQ_MASK);
	writel(0xffffffff, isp + ISP_SUB1 + 0x110);
	writel(0xffffdfff, isp + ISP_SUB2 + 0x110);
	writel(0x00002000, isp + ISP_SUB2 + 0x11c);

	writel(0x00230001, glb + GLB_PIPE_CFG);
	writel(1, glb + GLB_PIPE_EN);
}

static void ax_vin_dphy_power(struct ax_vin *vin, bool on)
{
	if (on) {
		regmap_write(vin->comm, COMM_DPHY_PWR_OFF_CLR, 1);
		regmap_write(vin->comm, COMM_DPHY_PWR_READY_CLR, 1);
	} else {
		regmap_write(vin->comm, COMM_DPHY_PWR_READY_SET, 1);
		regmap_write(vin->comm, COMM_DPHY_PWR_OFF_SET, 1);
	}
}

static void ax_vin_mipi_start(struct ax_vin *vin)
{
	/* Physical lane of D0..D3 and C0, C1 for the 4-lane mode. */
	static const u8 data_lanes[] = { 0, 1, 3, 4 };
	static const u8 clock_lanes[] = { 2, 5 };
	void __iomem *glb = vin->glb, *dphy = vin->dphy, *csi = vin->csi;
	unsigned int i;

	writel(CSI_STREAM_STOP, csi + CSI_STREAM_CTRL);
	writel(DPHY_DESKEW_RST, dphy + DPHY_DESKEW_RST_SET);
	ax_vin_dphy_power(vin, false);
	writel(CSI_STREAM_SOFT_RST | CSI_STREAM_STOP, csi + CSI_STREAM_CTRL);
	writel(0x3, csi + CSI_RX_SOFT_RST);
	writel(GLB_RST0_DPHYRX, glb + GLB_RST0_ASSERT);
	writel(GLB_RST0_CSIRX, glb + GLB_RST0_DEASSERT);
	writel(DPHY_DESKEW_RST, dphy + DPHY_DESKEW_RST_SET);
	regmap_write(vin->comm, COMM_RST_DEASSERT, COMM_RST_DPHYRX_TLB);

	writel(GLB_EB1_CSIRX_PCLK | GLB_EB1_SYS_PIXEL, glb + GLB_EB1_SET);
	ax_vin_field(glb, GLB_CLK_SEL_SET, GLB_CLK_SEL_CLR,
		     GLB_CLK_SEL_CSIRX_CFG, GLB_CLK_SEL_CSIRX_CFG);
	writel(GLB_EB0_CFG_PHY | GLB_EB0_DPHY_RX_REF, glb + GLB_EB0_SET);
	regmap_write(vin->comm, COMM_CLK_EB_SET, COMM_CLK_EB_DPHYRX_TLB);
	ax_vin_dphy_power(vin, true);
	writel(DPHY_LANE_EN_ALL, dphy + DPHY_LANE_EN_CLR);

	/* CSI controller 0 takes D-PHY 0. */
	ax_vin_field(glb, GLB_CSI_SEL_SET, GLB_CSI_SEL_CLR, 0x03, 0x00);
	ax_vin_field(glb, GLB_CSI_SEL_SET, GLB_CSI_SEL_CLR, 0x30, 0x30);

	for (i = 0; i <= DPHY_GLB_CFG_LAST; i += 4)
		writel(0, dphy + i);
	for (i = 0; i < ARRAY_SIZE(data_lanes); i++)
		ax_vin_field(dphy, DPHY_LANE_SEL_SET, DPHY_LANE_SEL_CLR,
			     0x7 << (21 - 3 * i), data_lanes[i] << (21 - 3 * i));
	for (i = 0; i < ARRAY_SIZE(clock_lanes); i++)
		ax_vin_field(dphy, DPHY_LANE_SEL_SET, DPHY_LANE_SEL_CLR,
			     0x7 << (9 - 3 * i), clock_lanes[i] << (9 - 3 * i));
	writel(DPHY_LANE_SEL_1D2C, dphy + DPHY_LANE_SEL_CLR);
	ax_vin_field(dphy, DPHY_LANE_SEL_SET, DPHY_LANE_SEL_CLR,
		     DPHY_LANE_SEL_DPDN_SWAP, 0);
	writel(DPHY_LANE_SEL_DATABUS16, dphy + DPHY_LANE_SEL_CLR);
	/* HS-RX clock and data prepare times of both lane groups. */
	for (i = 0; i < 4; i++)
		ax_vin_field(dphy, DPHY_HSRX_PRE_SET, DPHY_HSRX_PRE_CLR,
			     0xff << (8 * i), 0x08 << (8 * i));
	writel(0xfff, dphy + DPHY_HSRX_CFG1);
	writel(0x820, dphy + DPHY_HSRX_CFG0);

	msleep(25);
	writel(GLB_RST0_DPHYRX, glb + GLB_RST0_DEASSERT);
	/*
	 * Keep lane deskew in reset, as the vendor driver does below 1.5 Gb/s:
	 * the LT7911EXC sends no deskew sequence.
	 */
	writel(DPHY_DESKEW_RST, dphy + DPHY_DESKEW_RST_SET);
	writel(DPHY_LANE_EN_ALL, dphy + DPHY_LANE_EN_SET);
	writel(DPHY_LANE_EN_DATA, dphy + DPHY_LANE_EN_CLR);
	writel(DPHY_LANE_EN_DATA, dphy + DPHY_LANE_EN_SET);

	ax_vin_write_regs(csi, ax_vin_csi_cfg, ARRAY_SIZE(ax_vin_csi_cfg));
	writel(CSI_STREAM_SOFT_RST | CSI_STREAM_STOP, csi + CSI_STREAM_CTRL);
	writel(0, csi + CSI_STREAM_CTRL);
	writel(CSI_STREAM_START, csi + CSI_STREAM_CTRL);
	writel(0xffffffff, csi + CSI_ERR_STATUS);
	writel(0xffffffff, csi + CSI_PHY_ERR_STATUS);
	writel(CSI_ERR_ALL, csi + CSI_ERR_EN);
	writel(CSI_PHY_ERR_ALL, csi + CSI_PHY_ERR_EN);
}

static void ax_vin_ife_start(struct ax_vin *vin)
{
	const struct v4l2_pix_format *fmt = &vin->fmt;
	u32 dt = FIELD_PREP(IFE_IN_DT, MIPI_CSI2_DT_YUV422_8B);
	void __iomem *isp = vin->isp;

	writel(0xffff7ffb, isp + ISP_IRQ_MASK);
	writel(0, isp + IFE_UPDATE0);
	writel(0, isp + IFE_UPDATE1);
	ax_vin_write_regs(isp, ax_vin_ife_cfg, ARRAY_SIZE(ax_vin_ife_cfg));
	writel(fmt->height << 16 | fmt->width, isp + IFE_IN_SIZE);
	writel(dt, isp + IFE_IN_DT0);
	writel(dt, isp + IFE_IN_DT1);
	writel(dt, isp + IFE_IN_DT2);
	writel(1, isp + IFE_UPDATE0);
	writel(1, isp + IFE_UPDATE1);

	writel(1, isp + WDMA_START);
	writel(ISP_IRQ_EOF, isp + ISP_IRQ_CLR(ISP_IRQ_EOF_GROUP));
	writel(ISP_IRQ_SOF, isp + ISP_IRQ_CLR(ISP_IRQ_SOF_GROUP));
	writel(1, isp + ISP_START);
	writel(0, isp + IFE_UPDATE0);
	writel(0, isp + IFE_UPDATE1);
	writel(0x10000, isp + IFE_CTRL);
	writel(1, isp + IFE_START);
	writel(1, isp + IFE_UPDATE0);
	writel(1, isp + IFE_UPDATE1);
}

/*
 * The vendor driver's stop: the IFE drops the frame it is in, then the MIPI
 * receiver and the clocks go off, with no reset. Resetting the ISP while the
 * source had cut a frame short left every later frame shifted until the SoC
 * was reset; the next start resets it behind a stopped AXI port instead.
 */
static void ax_vin_hw_stop(struct ax_vin *vin)
{
	void __iomem *isp = vin->isp, *glb = vin->glb, *dphy = vin->dphy;
	/* The ISP only answers while clocked: at probe after a cold boot not. */
	bool clocked = (readl(glb + GLB_EB0) & GLB_EB0_ISP) == GLB_EB0_ISP;
	u32 status;

	writel(0, isp + ISP_IRQ_EN(ISP_IRQ_SOF_GROUP));
	writel(0, isp + ISP_IRQ_EN(ISP_IRQ_EOF_GROUP));
	if (clocked) {
		/* The update makes the stop take effect now, mid-frame too. */
		writel(IFE_CTRL_STOP(0), isp + IFE_CTRL);
		writel(1, isp + IFE_UPDATE0);
		writel(1, isp + IFE_UPDATE1);
		msleep(AX_VIN_STOP_SETTLE_MS);
		writel(0, isp + ISP_FRAME_EN);
		/*
		 * Keep the write DMA off the buffers, which are freed next:
		 * the AXI port stays stopped until the next start.
		 */
		writel(ISP_AXI_STOP, isp + ISP_AXI_CTRL);
		if (readl_poll_timeout(isp + ISP_AXI_STATUS, status, !status,
				       200, 10 * USEC_PER_MSEC))
			dev_warn(vin->dev, "AXI writes still pending (%#x)\n",
				 status);
	}

	/* No error recovery from here on: it would turn the lanes back on. */
	writel(0, vin->csi + CSI_ERR_EN);
	writel(0, vin->csi + CSI_PHY_ERR_EN);
	if (vin->csi_irq > 0)
		synchronize_irq(vin->csi_irq);

	writel(GLB_RST0_DPHYRX, glb + GLB_RST0_ASSERT);
	writel(GLB_EB0_DPHY_RX_REF | GLB_EB0_CFG_PHY, glb + GLB_EB0_CLR);
	if (clocked) {
		writel(DPHY_LANE_EN_ALL, dphy + DPHY_LANE_EN_CLR);
		writel(DPHY_DESKEW_RST, dphy + DPHY_DESKEW_RST_SET);
	}
	ax_vin_dphy_power(vin, false);
	writel(GLB_RST0_DESKEW, glb + GLB_RST0_ASSERT);
	/*
	 * Unlike the vendor driver, leave the D-PHY TLB clock on: the D-PHY
	 * registers only respond with it, and the next start pulses the
	 * D-PHY resets before it would turn the clock back on.
	 */
	writel(0, vin->csi + CSI_STREAM_CTRL);
	writel(CSI_STREAM_STOP, vin->csi + CSI_STREAM_CTRL);
	writel(GLB_EB1_SYS_PIXEL | GLB_EB1_CSIRX_PCLK, glb + GLB_EB1_CLR);
	writel(GLB_EB0_ALL, glb + GLB_EB0_CLR);
	writel(GLB_EB1_ALL, glb + GLB_EB1_CLR);
}

/* Called with qlock held. */
static void ax_vin_arm_next(struct ax_vin *vin)
{
	const struct v4l2_pix_format *fmt = &vin->fmt;
	void __iomem *isp = vin->isp;
	struct ax_vin_buffer *buf;
	dma_addr_t addr;

	if (vin->stopping || vin->armed || list_empty(&vin->queued))
		return;

	buf = list_first_entry(&vin->queued, struct ax_vin_buffer, list);
	list_del(&buf->list);
	if (!++vin->frame_id)
		vin->frame_id = 1;
	buf->frame_id = vin->frame_id;
	addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);

	ax_vin_write_regs(isp, ax_vin_frame_cfg, ARRAY_SIZE(ax_vin_frame_cfg));
	writel(fmt->height << 16 | fmt->width, isp + WDMA_SIZE);
	writel(fmt->bytesperline / 8, isp + WDMA_STRIDE0);
	writel(fmt->bytesperline / 8, isp + WDMA_STRIDE1);
	writel(addr >> 3, isp + WDMA_ADDR);
	writel(buf->frame_id, isp + WDMA_FRAME_ID);
	/* Must be last: an arm that misses a frame start waits for the next. */
	writel(1, isp + WDMA_ARM);
	vin->armed = buf;
}

static irqreturn_t ax_vin_irq(int irq, void *data)
{
	struct ax_vin *vin = data;
	void __iomem *isp = vin->isp;
	u32 sof, eof, latched;

	sof = readl(isp + ISP_IRQ_STATUS(ISP_IRQ_SOF_GROUP)) & ISP_IRQ_SOF;
	eof = readl(isp + ISP_IRQ_STATUS(ISP_IRQ_EOF_GROUP)) & ISP_IRQ_EOF;
	if (!sof && !eof)
		return IRQ_NONE;
	if (sof)
		writel(sof, isp + ISP_IRQ_CLR(ISP_IRQ_SOF_GROUP));
	if (eof)
		writel(eof, isp + ISP_IRQ_CLR(ISP_IRQ_EOF_GROUP));

	latched = FIELD_GET(WDMA_FRAME_ID_LATCHED, readl(isp + WDMA_FRAME_ID));

	spin_lock(&vin->qlock);

	if (eof && vin->active) {
		struct vb2_v4l2_buffer *vb = &vin->active->vb;

		vb->vb2_buf.timestamp = ktime_get_ns();
		vb->sequence = vin->active_sequence;
		vb->field = V4L2_FIELD_NONE;
		vb2_buffer_done(&vb->vb2_buf, VB2_BUF_STATE_DONE);
		vin->active = NULL;
	}

	if (sof) {
		vin->sequence++;
		if (vin->armed && vin->armed->frame_id == latched) {
			/* The frame end of the previous buffer never came. */
			if (vin->active)
				vb2_buffer_done(&vin->active->vb.vb2_buf,
						VB2_BUF_STATE_ERROR);
			vin->active = vin->armed;
			vin->active_sequence = vin->sequence - 1;
			vin->armed = NULL;
		}
		ax_vin_arm_next(vin);
	}

	/* Right after a frame end: the receiver is in vertical blanking. */
	if (vin->stopping && !vin->idle && !vin->active && !vin->armed) {
		writel(CSI_STREAM_STOP, vin->csi + CSI_STREAM_CTRL);
		vin->idle = true;
		wake_up(&vin->idle_wq);
	}

	spin_unlock(&vin->qlock);

	return IRQ_HANDLED;
}

static irqreturn_t ax_vin_csi_irq(int irq, void *data)
{
	struct ax_vin *vin = data;
	u32 err = readl(vin->csi + CSI_ERR_STATUS);
	u32 phy = readl(vin->csi + CSI_PHY_ERR_STATUS);

	if (!err && !phy)
		return IRQ_NONE;
	writel(err, vin->csi + CSI_ERR_STATUS);
	writel(phy, vin->csi + CSI_PHY_ERR_STATUS);
	vin->csi_err = err;
	vin->csi_phy_err = phy;

	return IRQ_WAKE_THREAD;
}

/*
 * The vendor driver's recovery from a broken packet or lane: restart the
 * receiver and deskew with the data lanes off, so that it locks on the next
 * packet start again.
 *
 * The line stays masked until this returns (oneshot), and this returns no
 * sooner than a millisecond: a source that keeps failing would otherwise
 * keep the CPU in the hard handler, this thread would never run, and the
 * kernel would disable the interrupt as unhandled.
 */
static irqreturn_t ax_vin_csi_irq_thread(int irq, void *data)
{
	struct ax_vin *vin = data;

	if (vin->csi_phy_err || vin->csi_err & CSI_ERR_RECOVER) {
		writel(DPHY_LANE_EN_DATA, vin->dphy + DPHY_LANE_EN_CLR);
		writel(GLB_RST0_CSIRX_RECOVER, vin->glb + GLB_RST0_ASSERT);
		writel(GLB_RST0_CSIRX_RECOVER, vin->glb + GLB_RST0_DEASSERT);
		writel(DPHY_LANE_EN_DATA, vin->dphy + DPHY_LANE_EN_SET);
		WRITE_ONCE(vin->csi_recoveries, vin->csi_recoveries + 1);
	}
	usleep_range(1000, 2000);

	return IRQ_HANDLED;
}

static void ax_vin_return_buffers(struct ax_vin *vin,
				  enum vb2_buffer_state state)
{
	struct ax_vin_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&vin->qlock, flags);
	if (vin->active)
		vb2_buffer_done(&vin->active->vb.vb2_buf, state);
	if (vin->armed)
		vb2_buffer_done(&vin->armed->vb.vb2_buf, state);
	vin->active = NULL;
	vin->armed = NULL;
	list_for_each_entry_safe(buf, tmp, &vin->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&vin->qlock, flags);
}

static int ax_vin_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct ax_vin *vin = vb2_get_drv_priv(q);

	if (*nplanes)
		return sizes[0] < vin->fmt.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = vin->fmt.sizeimage;

	return 0;
}

static int ax_vin_buf_prepare(struct vb2_buffer *vb)
{
	struct ax_vin *vin = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < vin->fmt.sizeimage)
		return -EINVAL;
	if (vb2_dma_contig_plane_dma_addr(vb, 0) & 0x7)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, vin->fmt.sizeimage);

	return 0;
}

static void ax_vin_buf_queue(struct vb2_buffer *vb)
{
	struct ax_vin *vin = vb2_get_drv_priv(vb->vb2_queue);
	struct ax_vin_buffer *buf = to_ax_vin_buffer(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&vin->qlock, flags);
	list_add_tail(&buf->list, &vin->queued);
	if (vb2_start_streaming_called(vb->vb2_queue))
		ax_vin_arm_next(vin);
	spin_unlock_irqrestore(&vin->qlock, flags);
}

static int ax_vin_check_source(struct ax_vin *vin)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	if (!vin->source)
		return 0;

	ret = v4l2_subdev_call_state_active(vin->source, pad, get_fmt, &fmt);
	if (ret)
		return ret;
	if (!fmt.format.width || !fmt.format.height)
		return -ENOLINK;
	if (fmt.format.width != vin->fmt.width ||
	    fmt.format.height != vin->fmt.height)
		return -EPIPE;

	return 0;
}

static int ax_vin_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ax_vin *vin = vb2_get_drv_priv(q);
	void __iomem *isp = vin->isp;
	unsigned long flags;
	int ret;

	ret = ax_vin_check_source(vin);
	if (ret) {
		ax_vin_return_buffers(vin, VB2_BUF_STATE_QUEUED);
		return ret;
	}

	vin->sequence = 0;
	vin->stopping = false;
	vin->idle = false;

	ax_vin_isp_reset(vin);
	ax_vin_mipi_start(vin);
	ax_vin_ife_start(vin);

	spin_lock_irqsave(&vin->qlock, flags);
	ax_vin_arm_next(vin);
	spin_unlock_irqrestore(&vin->qlock, flags);

	writel(ISP_IRQ_SOF, isp + ISP_IRQ_CLR(ISP_IRQ_SOF_GROUP));
	writel(ISP_IRQ_EOF, isp + ISP_IRQ_CLR(ISP_IRQ_EOF_GROUP));
	writel(ISP_IRQ_SOF, isp + ISP_IRQ_EN(ISP_IRQ_SOF_GROUP));
	writel(ISP_IRQ_EOF, isp + ISP_IRQ_EN(ISP_IRQ_EOF_GROUP));

	if (vin->source) {
		ret = v4l2_subdev_call(vin->source, video, s_stream, 1);
		if (ret && ret != -ENOIOCTLCMD) {
			ax_vin_hw_stop(vin);
			synchronize_irq(vin->irq);
			ax_vin_return_buffers(vin, VB2_BUF_STATE_QUEUED);
			return ret;
		}
	}

	return 0;
}

static void ax_vin_stop_streaming(struct vb2_queue *q)
{
	struct ax_vin *vin = vb2_get_drv_priv(q);
	unsigned long flags;

	spin_lock_irqsave(&vin->qlock, flags);
	vin->stopping = true;
	if (!vin->active && !vin->armed) {
		writel(CSI_STREAM_STOP, vin->csi + CSI_STREAM_CTRL);
		vin->idle = true;
	}
	spin_unlock_irqrestore(&vin->qlock, flags);

	/* At most the armed frame and the one being written. */
	if (!wait_event_timeout(vin->idle_wq, READ_ONCE(vin->idle),
				msecs_to_jiffies(200))) {
		WRITE_ONCE(vin->mid_frame_stops, vin->mid_frame_stops + 1);
		dev_warn(vin->dev, "no frame end, stopping mid-frame\n");
	}

	if (vin->source)
		v4l2_subdev_call(vin->source, video, s_stream, 0);
	ax_vin_hw_stop(vin);
	synchronize_irq(vin->irq);
	ax_vin_return_buffers(vin, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops ax_vin_qops = {
	.queue_setup		= ax_vin_queue_setup,
	.buf_prepare		= ax_vin_buf_prepare,
	.buf_queue		= ax_vin_buf_queue,
	.start_streaming	= ax_vin_start_streaming,
	.stop_streaming		= ax_vin_stop_streaming,
};

static void ax_vin_fill_fmt(struct v4l2_pix_format *pix)
{
	pix->width = clamp_t(u32, ALIGN(pix->width, 2), AX_VIN_MIN_SIZE,
			     AX_VIN_MAX_WIDTH);
	pix->height = clamp_t(u32, pix->height, AX_VIN_MIN_SIZE,
			      AX_VIN_MAX_HEIGHT);
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = ALIGN(pix->width * 2, AX_VIN_STRIDE_ALIGN);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->flags &= V4L2_PIX_FMT_FLAG_SET_CSC;
}

static void ax_vin_copy_colour(struct v4l2_pix_format *pix,
			       const struct v4l2_mbus_framefmt *mbus)
{
	pix->colorspace = mbus->colorspace;
	pix->ycbcr_enc = mbus->ycbcr_enc;
	pix->quantization = mbus->quantization;
	pix->xfer_func = mbus->xfer_func;
}

/*
 * The colour of the picture is the source's. With V4L2_PIX_FMT_FLAG_SET_CSC
 * it converts to the encoding and the range asked for if it can; a try only
 * tells what it would do.
 */
static void ax_vin_colour(struct ax_vin *vin, struct v4l2_pix_format *pix,
			  enum v4l2_subdev_format_whence which)
{
	struct v4l2_subdev_format fmt = { .which = which };
	int ret = -ENODEV;

	if (pix->flags & V4L2_PIX_FMT_FLAG_SET_CSC) {
		fmt.format.flags = V4L2_MBUS_FRAMEFMT_SET_CSC;
		fmt.format.ycbcr_enc = pix->ycbcr_enc;
		fmt.format.quantization = pix->quantization;
	}

	if (vin->source && which == V4L2_SUBDEV_FORMAT_TRY)
		ret = v4l2_subdev_call_state_try(vin->source, pad, set_fmt,
						 &fmt);
	else if (vin->source)
		ret = v4l2_subdev_call_state_active(vin->source, pad, set_fmt,
						    &fmt);

	if (ret) {
		pix->colorspace = V4L2_COLORSPACE_REC709;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix->quantization = V4L2_QUANTIZATION_DEFAULT;
		pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
		return;
	}

	ax_vin_copy_colour(pix, &fmt.format);
}

static int ax_vin_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, AX_VIN_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Axera AX620E video input", sizeof(cap->card));

	return 0;
}

static int ax_vin_enum_fmt(struct file *file, void *priv,
			   struct v4l2_fmtdesc *f)
{
	struct ax_vin *vin = video_drvdata(file);
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_YUYV;
	if (vin->source &&
	    !v4l2_subdev_call_state_active(vin->source, pad, enum_mbus_code,
					   &code)) {
		if (code.flags & V4L2_SUBDEV_MBUS_CODE_CSC_YCBCR_ENC)
			f->flags |= V4L2_FMT_FLAG_CSC_YCBCR_ENC;
		if (code.flags & V4L2_SUBDEV_MBUS_CODE_CSC_QUANTIZATION)
			f->flags |= V4L2_FMT_FLAG_CSC_QUANTIZATION;
	}

	return 0;
}

static int ax_vin_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ax_vin *vin = video_drvdata(file);
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_ACTIVE };

	f->fmt.pix = vin->fmt;
	/* The source's colour follows its input. */
	if (vin->source &&
	    !v4l2_subdev_call_state_active(vin->source, pad, get_fmt, &fmt))
		ax_vin_copy_colour(&f->fmt.pix, &fmt.format);

	return 0;
}

static int ax_vin_try_fmt(struct file *file, void *priv,
			  struct v4l2_format *f)
{
	struct ax_vin *vin = video_drvdata(file);

	ax_vin_fill_fmt(&f->fmt.pix);
	ax_vin_colour(vin, &f->fmt.pix, V4L2_SUBDEV_FORMAT_TRY);

	return 0;
}

static int ax_vin_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ax_vin *vin = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	ax_vin_fill_fmt(pix);
	/* With the buffers allocated only the colour may change. */
	if (vb2_is_busy(&vin->queue) &&
	    (pix->width != vin->fmt.width || pix->height != vin->fmt.height))
		return -EBUSY;

	ax_vin_colour(vin, pix, V4L2_SUBDEV_FORMAT_ACTIVE);
	vin->fmt = *pix;

	return 0;
}

static int ax_vin_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_YUYV)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = AX_VIN_MIN_SIZE;
	fsize->stepwise.max_width = AX_VIN_MAX_WIDTH;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = AX_VIN_MIN_SIZE;
	fsize->stepwise.max_height = AX_VIN_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int ax_vin_enum_input(struct file *file, void *priv,
			     struct v4l2_input *inp)
{
	struct ax_vin *vin = video_drvdata(file);
	struct v4l2_dv_timings timings;

	if (inp->index)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(inp->name, "MIPI CSI-2", sizeof(inp->name));
	if (vin->source) {
		inp->capabilities = V4L2_IN_CAP_DV_TIMINGS;
		if (v4l2_subdev_call(vin->source, pad, query_dv_timings, 0,
				     &timings))
			inp->status = V4L2_IN_ST_NO_SIGNAL;
	}

	return 0;
}

static int ax_vin_query_dv_timings(struct file *file, void *priv,
				   struct v4l2_dv_timings *timings)
{
	struct ax_vin *vin = video_drvdata(file);

	if (!vin->source)
		return -ENOTTY;

	return v4l2_subdev_call(vin->source, pad, query_dv_timings, 0, timings);
}

static int ax_vin_g_dv_timings(struct file *file, void *priv,
			       struct v4l2_dv_timings *timings)
{
	struct ax_vin *vin = video_drvdata(file);

	if (!vin->source)
		return -ENOTTY;

	return v4l2_subdev_call(vin->source, pad, g_dv_timings, 0, timings);
}

static int ax_vin_s_dv_timings(struct file *file, void *priv,
			       struct v4l2_dv_timings *timings)
{
	struct ax_vin *vin = video_drvdata(file);
	int ret;

	if (!vin->source)
		return -ENOTTY;
	if (vb2_is_busy(&vin->queue))
		return -EBUSY;

	ret = v4l2_subdev_call(vin->source, pad, s_dv_timings, 0, timings);
	if (ret)
		return ret;

	vin->fmt.width = timings->bt.width;
	vin->fmt.height = timings->bt.height;
	ax_vin_fill_fmt(&vin->fmt);

	return 0;
}

static int ax_vin_enum_dv_timings(struct file *file, void *priv,
				  struct v4l2_enum_dv_timings *timings)
{
	struct ax_vin *vin = video_drvdata(file);

	if (!vin->source)
		return -ENOTTY;

	timings->pad = 0;
	return v4l2_subdev_call(vin->source, pad, enum_dv_timings, timings);
}

static int ax_vin_dv_timings_cap(struct file *file, void *priv,
				 struct v4l2_dv_timings_cap *cap)
{
	struct ax_vin *vin = video_drvdata(file);

	if (!vin->source)
		return -ENOTTY;

	cap->pad = 0;
	return v4l2_subdev_call(vin->source, pad, dv_timings_cap, cap);
}

/*
 * The bridge keeps its EDID in flash: reading it takes over a second and
 * writing it several. Let the buffer queue run meanwhile.
 */
static int ax_vin_edid(struct file *file, struct v4l2_edid *edid, bool set)
{
	struct ax_vin *vin = video_drvdata(file);
	struct v4l2_subdev *source = vin->source;
	int ret;

	if (!source)
		return -ENOTTY;
	if (edid->pad)
		return -EINVAL;

	mutex_unlock(&vin->lock);
	if (set)
		ret = v4l2_subdev_call(source, pad, set_edid, edid);
	else
		ret = v4l2_subdev_call(source, pad, get_edid, edid);
	mutex_lock(&vin->lock);

	return ret;
}

static int ax_vin_g_edid(struct file *file, void *priv, struct v4l2_edid *edid)
{
	return ax_vin_edid(file, edid, false);
}

static int ax_vin_s_edid(struct file *file, void *priv, struct v4l2_edid *edid)
{
	return ax_vin_edid(file, edid, true);
}

static int ax_vin_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	if (sub->type == V4L2_EVENT_SOURCE_CHANGE)
		return v4l2_src_change_event_subscribe(fh, sub);

	return v4l2_ctrl_subscribe_event(fh, sub);
}

static int ax_vin_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int ax_vin_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops ax_vin_ioctl_ops = {
	.vidioc_querycap		= ax_vin_querycap,
	.vidioc_enum_fmt_vid_cap	= ax_vin_enum_fmt,
	.vidioc_g_fmt_vid_cap		= ax_vin_g_fmt,
	.vidioc_try_fmt_vid_cap		= ax_vin_try_fmt,
	.vidioc_s_fmt_vid_cap		= ax_vin_s_fmt,
	.vidioc_enum_framesizes		= ax_vin_enum_framesizes,
	.vidioc_enum_input		= ax_vin_enum_input,
	.vidioc_g_input			= ax_vin_g_input,
	.vidioc_s_input			= ax_vin_s_input,
	.vidioc_query_dv_timings	= ax_vin_query_dv_timings,
	.vidioc_g_dv_timings		= ax_vin_g_dv_timings,
	.vidioc_s_dv_timings		= ax_vin_s_dv_timings,
	.vidioc_enum_dv_timings		= ax_vin_enum_dv_timings,
	.vidioc_dv_timings_cap		= ax_vin_dv_timings_cap,
	.vidioc_g_edid			= ax_vin_g_edid,
	.vidioc_s_edid			= ax_vin_s_edid,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_subscribe_event		= ax_vin_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

};

static const struct v4l2_file_operations ax_vin_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.unlocked_ioctl	= video_ioctl2,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

static void ax_vin_notify(struct v4l2_subdev *sd, unsigned int notification,
			  void *arg)
{
	struct ax_vin *vin = container_of(sd->v4l2_dev, struct ax_vin,
					  v4l2_dev);

	if (notification == V4L2_DEVICE_NOTIFY_EVENT)
		v4l2_event_queue(&vin->vdev, arg);
}

static int ax_vin_notify_bound(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *sd,
			       struct v4l2_async_connection *asc)
{
	struct ax_vin *vin = container_of(notifier, struct ax_vin, notifier);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	ret = v4l2_ctrl_add_handler(&vin->ctrl_handler, sd->ctrl_handler, NULL,
				    true);
	if (ret)
		return ret;

	vin->source = sd;
	if (!vb2_is_busy(&vin->queue) &&
	    !v4l2_subdev_call_state_active(sd, pad, get_fmt, &fmt) &&
	    fmt.format.width && fmt.format.height) {
		vin->fmt.width = fmt.format.width;
		vin->fmt.height = fmt.format.height;
		ax_vin_fill_fmt(&vin->fmt);
	}
	dev_info(vin->dev, "bound %s\n", sd->name);

	return 0;
}

static void ax_vin_notify_unbind(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_connection *asc)
{
	struct ax_vin *vin = container_of(notifier, struct ax_vin, notifier);

	mutex_lock(&vin->lock);
	vin->source = NULL;
	/* Drop the references to the controls the source is about to free. */
	v4l2_ctrl_handler_free(&vin->ctrl_handler);
	v4l2_ctrl_handler_init(&vin->ctrl_handler, 3);
	mutex_unlock(&vin->lock);
}

static const struct v4l2_async_notifier_operations ax_vin_notify_ops = {
	.bound = ax_vin_notify_bound,
	.unbind = ax_vin_notify_unbind,
};

static int ax_vin_register_source(struct ax_vin *vin)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct v4l2_async_connection *asc;
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(vin->dev), NULL);
	if (!fwnode)
		return 0;

	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep);
	if (!ret && ep.bus.mipi_csi2.num_data_lanes != 4)
		ret = dev_err_probe(vin->dev, -EINVAL,
				    "only 4 data lanes are supported\n");
	if (ret) {
		fwnode_handle_put(fwnode);
		return ret;
	}

	v4l2_async_nf_init(&vin->notifier, &vin->v4l2_dev);
	vin->notifier.ops = &ax_vin_notify_ops;
	asc = v4l2_async_nf_add_fwnode_remote(&vin->notifier, fwnode,
					      struct v4l2_async_connection);
	fwnode_handle_put(fwnode);
	if (IS_ERR(asc)) {
		v4l2_async_nf_cleanup(&vin->notifier);
		return PTR_ERR(asc);
	}

	ret = v4l2_async_nf_register(&vin->notifier);
	if (ret)
		v4l2_async_nf_cleanup(&vin->notifier);

	return ret;
}

static int ax_vin_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct ax_vin *vin;
	struct vb2_queue *q;
	int ret;

	vin = devm_kzalloc(dev, sizeof(*vin), GFP_KERNEL);
	if (!vin)
		return -ENOMEM;
	vin->dev = dev;
	mutex_init(&vin->lock);
	spin_lock_init(&vin->qlock);
	init_waitqueue_head(&vin->idle_wq);
	INIT_LIST_HEAD(&vin->queued);

	vin->isp = devm_platform_ioremap_resource_byname(pdev, "isp");
	if (IS_ERR(vin->isp))
		return PTR_ERR(vin->isp);
	vin->glb = devm_platform_ioremap_resource_byname(pdev, "glb");
	if (IS_ERR(vin->glb))
		return PTR_ERR(vin->glb);
	vin->csi = devm_platform_ioremap_resource_byname(pdev, "csi");
	if (IS_ERR(vin->csi))
		return PTR_ERR(vin->csi);
	vin->dphy = devm_platform_ioremap_resource_byname(pdev, "dphy");
	if (IS_ERR(vin->dphy))
		return PTR_ERR(vin->dphy);
	vin->comm = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "axera,comm-glb");
	if (IS_ERR(vin->comm))
		return dev_err_probe(dev, PTR_ERR(vin->comm),
				     "no COMM_GLB syscon\n");

	/* The boot firmware or a previous kernel may have left it running. */
	ax_vin_hw_stop(vin);

	vin->irq = platform_get_irq(pdev, 0);
	if (vin->irq < 0)
		return vin->irq;
	ret = devm_request_irq(dev, vin->irq, ax_vin_irq, 0, AX_VIN_NAME, vin);
	if (ret)
		return ret;

	vin->csi_irq = platform_get_irq_byname(pdev, "csi");
	if (vin->csi_irq < 0)
		return vin->csi_irq;
	ret = devm_request_threaded_irq(dev, vin->csi_irq, ax_vin_csi_irq,
					ax_vin_csi_irq_thread, IRQF_ONESHOT,
					AX_VIN_NAME "-csi", vin);
	if (ret)
		return ret;

	vin->fmt.width = AX_VIN_MAX_WIDTH;
	vin->fmt.height = AX_VIN_MAX_HEIGHT;
	vin->fmt.colorspace = V4L2_COLORSPACE_REC709;
	ax_vin_fill_fmt(&vin->fmt);

	vin->v4l2_dev.notify = ax_vin_notify;
	ret = v4l2_device_register(dev, &vin->v4l2_dev);
	if (ret)
		return ret;
	v4l2_ctrl_handler_init(&vin->ctrl_handler, 3);

	q = &vin->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->lock = &vin->lock;
	q->drv_priv = vin;
	q->buf_struct_size = sizeof(struct ax_vin_buffer);
	q->ops = &ax_vin_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
			     V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	q->min_queued_buffers = 1;
	q->dev = dev;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_v4l2;

	vdev = &vin->vdev;
	strscpy(vdev->name, AX_VIN_NAME, sizeof(vdev->name));
	vdev->v4l2_dev = &vin->v4l2_dev;
	vdev->fops = &ax_vin_fops;
	vdev->ioctl_ops = &ax_vin_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &vin->lock;
	vdev->queue = q;
	vdev->ctrl_handler = &vin->ctrl_handler;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, vin);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_v4l2;

	platform_set_drvdata(pdev, vin);

	ret = ax_vin_register_source(vin);
	if (ret)
		goto err_vdev;

	dev_info(dev, "registered %s\n", video_device_node_name(vdev));

	return 0;

err_vdev:
	vb2_video_unregister_device(vdev);
err_v4l2:
	v4l2_ctrl_handler_free(&vin->ctrl_handler);
	v4l2_device_unregister(&vin->v4l2_dev);
	return ret;
}

static void ax_vin_remove(struct platform_device *pdev)
{
	struct ax_vin *vin = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&vin->notifier);
	v4l2_async_nf_cleanup(&vin->notifier);
	vb2_video_unregister_device(&vin->vdev);
	v4l2_ctrl_handler_free(&vin->ctrl_handler);
	v4l2_device_unregister(&vin->v4l2_dev);
	mutex_destroy(&vin->lock);
}

/*
 * Stops in the middle of a frame since boot: after one, frames may come out
 * shifted until the SoC is reset.
 */
static ssize_t mid_frame_stops_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ax_vin *vin = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(vin->mid_frame_stops));
}
static DEVICE_ATTR_RO(mid_frame_stops);

static ssize_t csi_recoveries_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ax_vin *vin = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(vin->csi_recoveries));
}
static DEVICE_ATTR_RO(csi_recoveries);

static struct attribute *ax_vin_attrs[] = {
	&dev_attr_mid_frame_stops.attr,
	&dev_attr_csi_recoveries.attr,
	NULL
};
ATTRIBUTE_GROUPS(ax_vin);

static const struct of_device_id ax_vin_of_match[] = {
	{ .compatible = "axera,ax620e-vin" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax_vin_of_match);

static struct platform_driver ax_vin_driver = {
	.probe = ax_vin_probe,
	.remove = ax_vin_remove,
	.driver = {
		.name = AX_VIN_NAME,
		.of_match_table = ax_vin_of_match,
		.dev_groups = ax_vin_groups,
	},
};
module_platform_driver(ax_vin_driver);

MODULE_DESCRIPTION("Axera AX620E video input driver");
MODULE_LICENSE("GPL");
