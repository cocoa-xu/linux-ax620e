// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver: Axera AX620E JPEG, H.264 and HEVC encoders
 *
 * Two VeriSilicon VC8000E cores, one in JPEG-only configuration (JENC) and
 * one for H.264 and HEVC (VENC). Their registers sit behind a VCMD command
 * parser at +0x1000 and are also directly writable, which is how this
 * driver uses them.
 */

#include "hantro.h"
#include "hantro_jpeg.h"

static const struct hantro_fmt ax620e_jenc_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_YUYV422,
	},
	{
		.fourcc = V4L2_PIX_FMT_UYVY,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_UYVY422,
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.codec_mode = HANTRO_MODE_JPEG_ENC,
		.max_depth = 2,
		.header_size = JPEG_HEADER_SIZE,
		.frmsize = {
			.min_width = 96,
			.max_width = 4096,
			.step_width = MB_DIM,
			.min_height = 32,
			.max_height = 4096,
			.step_height = MB_DIM,
		},
	},
};

static const struct hantro_codec_ops ax620e_jenc_codec_ops[] = {
	[HANTRO_MODE_JPEG_ENC] = {
		.run = hantro_vc8000e_jpeg_enc_run,
		.reset = hantro_vc8000e_reset,
		.done = hantro_vc8000e_jpeg_enc_done,
	},
};

static const struct hantro_irq ax620e_jenc_irqs[] = {
	{ "jenc", hantro_vc8000e_jpeg_enc_irq },
};

static const char * const ax620e_jenc_clk_names[] = { "jenc" };

const struct hantro_variant ax620e_jenc_variant = {
	.enc_offset = 0x1000,
	.enc_fmts = ax620e_jenc_fmts,
	.num_enc_fmts = ARRAY_SIZE(ax620e_jenc_fmts),
	.codec = HANTRO_JPEG_ENCODER,
	.codec_ops = ax620e_jenc_codec_ops,
	.irqs = ax620e_jenc_irqs,
	.num_irqs = ARRAY_SIZE(ax620e_jenc_irqs),
	.clk_names = ax620e_jenc_clk_names,
	.num_clocks = ARRAY_SIZE(ax620e_jenc_clk_names),
};

#define AX620E_VENC_FRMSIZE {			\
	.min_width = FMT_MIN_WIDTH,		\
	.max_width = FMT_4K_WIDTH,		\
	.step_width = MB_DIM,			\
	.min_height = FMT_MIN_HEIGHT,		\
	.max_height = FMT_4K_HEIGHT,		\
	.step_height = MB_DIM,			\
}

static const struct hantro_fmt ax620e_venc_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_YUYV422,
		.frmsize = AX620E_VENC_FRMSIZE,
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_UYVY422,
		.frmsize = AX620E_VENC_FRMSIZE,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.codec_mode = HANTRO_MODE_NONE,
		.enc_fmt = ROCKCHIP_VPU_ENC_FMT_YUV420SP,
		.frmsize = AX620E_VENC_FRMSIZE,
	}, {
		.fourcc = V4L2_PIX_FMT_H264_SLICE,
		.codec_mode = HANTRO_MODE_H264_ENC,
		.max_depth = 2,
		.frmsize = AX620E_VENC_FRMSIZE,
	}, {
		.fourcc = V4L2_PIX_FMT_HEVC_SLICE,
		.codec_mode = HANTRO_MODE_HEVC_ENC,
		.max_depth = 2,
		.frmsize = AX620E_VENC_FRMSIZE,
	},
};

static const struct hantro_codec_ops ax620e_venc_codec_ops[] = {
	[HANTRO_MODE_H264_ENC] = {
		.run = hantro_vc8000e_h264_enc_run,
		.reset = hantro_vc8000e_reset,
		.done = hantro_vc8000e_h264_enc_done,
		.init = hantro_vc8000e_h264_enc_init,
		.exit = hantro_vc8000e_h264_enc_exit,
	},
	[HANTRO_MODE_HEVC_ENC] = {
		.run = hantro_vc8000e_hevc_enc_run,
		.reset = hantro_vc8000e_reset,
		.done = hantro_vc8000e_hevc_enc_done,
		.init = hantro_vc8000e_hevc_enc_init,
		.exit = hantro_vc8000e_hevc_enc_exit,
	},
};

static const struct hantro_irq ax620e_venc_irqs[] = {
	{ "venc", hantro_vc8000e_irq },
};

static const char * const ax620e_venc_clk_names[] = { "venc" };

const struct hantro_variant ax620e_venc_variant = {
	.enc_offset = 0x1000,
	.enc_fmts = ax620e_venc_fmts,
	.num_enc_fmts = ARRAY_SIZE(ax620e_venc_fmts),
	.codec = HANTRO_H264_ENCODER | HANTRO_HEVC_ENCODER,
	.codec_ops = ax620e_venc_codec_ops,
	.irqs = ax620e_venc_irqs,
	.num_irqs = ARRAY_SIZE(ax620e_venc_irqs),
	.clk_names = ax620e_venc_clk_names,
	.num_clocks = ARRAY_SIZE(ax620e_venc_clk_names),
	.h264_enc_fixed_num_lengths = 1,
};
