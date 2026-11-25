/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * V4L2 HEVC Encode Core: the state of a stateless HEVC encoder from one
 * picture to the next. The parameter sets come from the HEVC SPS and PPS
 * controls, used as templates, and go before each IDR picture; the slice QP
 * comes from the HEVC frame QP controls; each picture refers to the one before
 * it. The hardware writes the slice segment headers.
 */

#ifndef _MEDIA_V4L2_HEVC_ENC_H
#define _MEDIA_V4L2_HEVC_ENC_H

#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-hevc-enc-rbsp.h>
#include <media/videobuf2-v4l2.h>

#define V4L2_HEVC_ENC_FLAG_INTER_PRED	0x1

struct v4l2_hevc_enc;

struct v4l2_hevc_enc_rec_buffer {
	void *private_data;
	bool allocated;
};

/**
 * struct v4l2_hevc_enc_ref - The reference picture
 *
 * @buffer_current: reconstruction of the picture being encoded
 * @buffer: reconstruction of the reference picture
 * @valid: @buffer holds the reference picture
 * @poc: PicOrderCntVal of the reference picture
 * @pic_order_cnt: PicOrderCntVal of the picture being encoded
 */
struct v4l2_hevc_enc_ref {
	struct v4l2_hevc_enc_rec_buffer buffer_current;
	struct v4l2_hevc_enc_rec_buffer buffer;
	bool valid;
	s32 poc;
	s32 pic_order_cnt;
};

/**
 * struct v4l2_hevc_enc_state - The picture being encoded
 *
 * @sps: sequence parameter set, sized for the coded format
 * @pps: picture parameter set
 * @encode: encode parameters of the picture
 * @seq: what the parameter sets need beyond @sps
 * @width: visible width
 * @height: visible height
 * @qp: slice QP
 * @qp_min: lowest QP of a coding unit
 * @qp_max: highest QP of a coding unit
 */
struct v4l2_hevc_enc_state {
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct v4l2_ctrl_hevc_encode_params encode;
	struct v4l2_hevc_enc_seq seq;
	unsigned int width;
	unsigned int height;
	unsigned int qp;
	unsigned int qp_min;
	unsigned int qp_max;
};

/**
 * struct v4l2_hevc_enc_ops - Driver hooks
 *
 * @state_constrain: makes the state what the hardware can encode, before
 *		     the picture size is derived from the format
 * @rec_buffer_alloc: allocates a reconstruction buffer
 * @rec_buffer_free: frees a reconstruction buffer
 */
struct v4l2_hevc_enc_ops {
	int (*state_constrain)(struct v4l2_hevc_enc *enc,
			       struct v4l2_hevc_enc_state *state);
	int (*rec_buffer_alloc)(struct v4l2_hevc_enc *enc,
				struct v4l2_hevc_enc_rec_buffer *buffer);
	void (*rec_buffer_free)(struct v4l2_hevc_enc *enc,
				struct v4l2_hevc_enc_rec_buffer *buffer);
};

/**
 * struct v4l2_hevc_enc - HEVC encode core
 *
 * @ops: driver hooks
 * @private_data: for the driver
 * @format_mplane: coded format, for the picture size and colorimetry
 * @timeperframe: frame interval
 * @ctrl_handler: holds the HEVC SPS, PPS, encode parameters and QP controls
 * @flags: see V4L2_HEVC_ENC_FLAG_{}
 * @state: the picture being encoded
 * @ref: the reference picture
 * @header_size: bytes of parameter sets before the slice in the coded buffer
 * @pictures: pictures encoded since init
 */
struct v4l2_hevc_enc {
	const struct v4l2_hevc_enc_ops *ops;
	void *private_data;
	const struct v4l2_pix_format_mplane *format_mplane;
	const struct v4l2_fract *timeperframe;
	struct v4l2_ctrl_handler *ctrl_handler;
	unsigned int flags;

	struct v4l2_hevc_enc_state state;
	struct v4l2_hevc_enc_ref ref;
	unsigned int header_size;
	unsigned int pictures;
};

int v4l2_hevc_enc_init(struct v4l2_hevc_enc *enc);
void v4l2_hevc_enc_exit(struct v4l2_hevc_enc *enc);
int v4l2_hevc_enc_step(struct v4l2_hevc_enc *enc,
		       struct vb2_v4l2_buffer *buffer);
void v4l2_hevc_enc_complete(struct v4l2_hevc_enc *enc,
			    struct vb2_v4l2_buffer *buffer);

#endif
