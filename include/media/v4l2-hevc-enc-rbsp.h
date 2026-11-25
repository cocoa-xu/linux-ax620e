/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * V4L2 HEVC Encode RBSP: the parameter sets of an encoded stream, as Annex B
 * NAL units. Encoders that write slice segment headers in hardware need no
 * more.
 */

#ifndef _MEDIA_V4L2_HEVC_ENC_RBSP_H
#define _MEDIA_V4L2_HEVC_ENC_RBSP_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>

#define V4L2_HEVC_NAL_TRAIL_R		1
#define V4L2_HEVC_NAL_IDR_W_RADL	19
#define V4L2_HEVC_NAL_VPS		32
#define V4L2_HEVC_NAL_SPS		33
#define V4L2_HEVC_NAL_PPS		34

#define V4L2_HEVC_PROFILE_IDC_MAIN	1

#define V4L2_HEVC_VUI_FLAG_VIDEO_SIGNAL_TYPE	0x1
#define V4L2_HEVC_VUI_FLAG_FULL_RANGE		0x2
#define V4L2_HEVC_VUI_FLAG_COLOUR_DESCRIPTION	0x4
#define V4L2_HEVC_VUI_FLAG_TIMING_INFO		0x8

/**
 * struct v4l2_hevc_enc_seq - What a sequence needs beyond the SPS control
 *
 * @profile_idc: general_profile_idc
 * @level_idc: general_level_idc, 30 times the level
 * @tier_flag: general_tier_flag
 * @conf_win_right_offset: conformance window, in chroma samples
 * @conf_win_bottom_offset: conformance window, in chroma samples
 * @num_units_in_tick: VUI timing, the frame duration in time_scale units
 * @time_scale: VUI timing
 * @video_format: VUI video_format
 * @colour_primaries: VUI colour description
 * @transfer_characteristics: VUI colour description
 * @matrix_coeffs: VUI colour description
 * @vui_flags: see V4L2_HEVC_VUI_FLAG_{}
 */
struct v4l2_hevc_enc_seq {
	u8 profile_idc;
	u8 level_idc;
	u8 tier_flag;
	u16 conf_win_right_offset;
	u16 conf_win_bottom_offset;
	u32 num_units_in_tick;
	u32 time_scale;
	u8 video_format;
	u8 colour_primaries;
	u8 transfer_characteristics;
	u8 matrix_coeffs;
	u32 vui_flags;
};

int v4l2_hevc_enc_rbsp_vps(u8 *dst, unsigned int size,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_enc_seq *seq);
int v4l2_hevc_enc_rbsp_sps(u8 *dst, unsigned int size,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_enc_seq *seq);
int v4l2_hevc_enc_rbsp_pps(u8 *dst, unsigned int size,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_ctrl_hevc_pps *pps);

#endif
