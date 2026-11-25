// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 HEVC Encode Core
 */

#include <linux/align.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <media/v4l2-hevc-enc.h>

/* Table A.8, Main tier: general_level_idc, MaxLumaPs, MaxLumaSr. */
static const struct {
	u8 level_idc;
	u32 max_luma_ps;
	u64 max_luma_sr;
} levels[] = {
	{ 30, 36864, 552960 },
	{ 60, 122880, 3686400 },
	{ 63, 245760, 7372800 },
	{ 90, 552960, 16588800 },
	{ 93, 983040, 33177600 },
	{ 120, 2228224, 66846720 },
	{ 123, 2228224, 133693440 },
	{ 150, 8912896, 267386880 },
	{ 153, 8912896, 534773760 },
	{ 156, 8912896, 1069547520 },
	{ 180, 35651584, 1069547520 },
	{ 183, 35651584, 2139095040 },
	{ 186, 35651584, 4278190080ULL },
};

static u8 level_idc(unsigned int width, unsigned int height,
		    const struct v4l2_fract *timeperframe)
{
	u64 luma_ps = (u64)width * height;
	u64 luma_sr = 0;
	unsigned int i;

	if (timeperframe->numerator)
		luma_sr = div_u64(luma_ps * timeperframe->denominator +
				  timeperframe->numerator - 1,
				  timeperframe->numerator);

	for (i = 0; i < ARRAY_SIZE(levels); i++) {
		u64 max_side_squared = 8ULL * levels[i].max_luma_ps;

		if (luma_ps <= levels[i].max_luma_ps &&
		    luma_sr <= levels[i].max_luma_sr &&
		    (u64)width * width <= max_side_squared &&
		    (u64)height * height <= max_side_squared)
			return levels[i].level_idc;
	}

	return levels[ARRAY_SIZE(levels) - 1].level_idc;
}

/* The code points of ITU-T H.273, 2 for unspecified. */

static u8 colour_primaries(enum v4l2_colorspace colorspace)
{
	switch (colorspace) {
	case V4L2_COLORSPACE_REC709:
	case V4L2_COLORSPACE_JPEG:
	case V4L2_COLORSPACE_SRGB:
		return 1;
	case V4L2_COLORSPACE_470_SYSTEM_M:
		return 4;
	case V4L2_COLORSPACE_470_SYSTEM_BG:
		return 5;
	case V4L2_COLORSPACE_SMPTE170M:
	case V4L2_COLORSPACE_BT878:
		return 6;
	case V4L2_COLORSPACE_SMPTE240M:
		return 7;
	case V4L2_COLORSPACE_BT2020:
		return 9;
	case V4L2_COLORSPACE_DCI_P3:
		return 11;
	default:
		return 2;
	}
}

static u8 transfer_characteristics(enum v4l2_xfer_func xfer_func)
{
	switch (xfer_func) {
	case V4L2_XFER_FUNC_709:
		return 1;
	case V4L2_XFER_FUNC_SMPTE240M:
		return 7;
	case V4L2_XFER_FUNC_NONE:
		return 8;
	case V4L2_XFER_FUNC_SRGB:
		return 13;
	case V4L2_XFER_FUNC_SMPTE2084:
		return 16;
	case V4L2_XFER_FUNC_DCI_P3:
		return 17;
	default:
		return 2;
	}
}

static u8 matrix_coeffs(enum v4l2_ycbcr_encoding ycbcr_enc)
{
	switch (ycbcr_enc) {
	case V4L2_YCBCR_ENC_709:
	case V4L2_YCBCR_ENC_XV709:
		return 1;
	case V4L2_YCBCR_ENC_601:
	case V4L2_YCBCR_ENC_XV601:
		return 6;
	case V4L2_YCBCR_ENC_SMPTE240M:
		return 7;
	case V4L2_YCBCR_ENC_BT2020:
		return 9;
	case V4L2_YCBCR_ENC_BT2020_CONST_LUM:
		return 10;
	default:
		return 2;
	}
}

static void seq_colour(struct v4l2_hevc_enc_seq *seq,
		       const struct v4l2_pix_format_mplane *format)
{
	enum v4l2_colorspace colorspace = format->colorspace;
	enum v4l2_ycbcr_encoding ycbcr_enc = format->ycbcr_enc;
	enum v4l2_quantization quantization = format->quantization;
	enum v4l2_xfer_func xfer_func = format->xfer_func;

	if (colorspace == V4L2_COLORSPACE_DEFAULT)
		return;

	if (ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT)
		ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(colorspace);
	if (xfer_func == V4L2_XFER_FUNC_DEFAULT)
		xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(colorspace);
	if (quantization == V4L2_QUANTIZATION_DEFAULT)
		quantization = V4L2_MAP_QUANTIZATION_DEFAULT(false, colorspace,
							     ycbcr_enc);

	seq->vui_flags |= V4L2_HEVC_VUI_FLAG_VIDEO_SIGNAL_TYPE |
			  V4L2_HEVC_VUI_FLAG_COLOUR_DESCRIPTION;
	if (quantization == V4L2_QUANTIZATION_FULL_RANGE)
		seq->vui_flags |= V4L2_HEVC_VUI_FLAG_FULL_RANGE;

	/* video_format: unspecified */
	seq->video_format = 5;
	seq->colour_primaries = colour_primaries(colorspace);
	seq->transfer_characteristics = transfer_characteristics(xfer_func);
	seq->matrix_coeffs = matrix_coeffs(ycbcr_enc);
}

static s32 ctrl_value(struct v4l2_hevc_enc *enc, u32 id, s32 fallback)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(enc->ctrl_handler, id);

	return ctrl ? ctrl->cur.val : fallback;
}

static int state_prepare(struct v4l2_hevc_enc *enc)
{
	struct v4l2_hevc_enc_state *state = &enc->state;
	struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	struct v4l2_ctrl_hevc_pps *pps = &state->pps;
	struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	const struct v4l2_pix_format_mplane *format = enc->format_mplane;
	const struct v4l2_fract *timeperframe = enc->timeperframe;
	bool inter = enc->flags & V4L2_HEVC_ENC_FLAG_INTER_PRED;
	struct v4l2_ctrl *ctrl;
	unsigned int min_cb_size;
	int ret;

	memset(state, 0, sizeof(*state));

	ctrl = v4l2_ctrl_find(enc->ctrl_handler, V4L2_CID_STATELESS_HEVC_SPS);
	if (!ctrl)
		return -EINVAL;
	*sps = *ctrl->p_cur.p_hevc_sps;

	ctrl = v4l2_ctrl_find(enc->ctrl_handler, V4L2_CID_STATELESS_HEVC_PPS);
	if (!ctrl)
		return -EINVAL;
	*pps = *ctrl->p_cur.p_hevc_pps;

	ctrl = v4l2_ctrl_find(enc->ctrl_handler,
			      V4L2_CID_STATELESS_HEVC_ENCODE_PARAMS);
	if (!ctrl)
		return -EINVAL;
	*encode = *ctrl->p_cur.p_hevc_encode_params;

	/* A stream starts with an IDR picture; B pictures are not encoded. */
	if (!enc->pictures && !(encode->flags & V4L2_HEVC_ENCODE_FLAG_IDR_PIC))
		return -EINVAL;
	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_B ||
	    (encode->slice_type == V4L2_HEVC_SLICE_TYPE_P && !inter))
		return -EINVAL;

	/*
	 * One parameter set of each kind, no sub-layers or long-term
	 * references, and every picture the reference of the next.
	 */
	sps->sps_max_sub_layers_minus1 = 0;
	sps->sps_max_dec_pic_buffering_minus1 = inter;
	sps->sps_max_num_reorder_pics = 0;
	sps->sps_max_latency_increase_plus1 = 0;
	sps->num_long_term_ref_pics_sps = 0;
	sps->flags &= ~V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT;
	if (!inter)
		sps->num_short_term_ref_pic_sets = 0;
	else if (!sps->num_short_term_ref_pic_sets)
		sps->num_short_term_ref_pic_sets = 1;
	encode->pic_parameter_set_id = pps->pic_parameter_set_id;

	state->width = format->width;
	state->height = format->height;
	state->seq.profile_idc = V4L2_HEVC_PROFILE_IDC_MAIN;

	if (enc->ops->state_constrain) {
		ret = enc->ops->state_constrain(enc, state);
		if (ret)
			return ret;
	}

	/* The coded size is a whole number of minimum coding blocks. */
	min_cb_size = 1 << (sps->log2_min_luma_coding_block_size_minus3 + 3);
	sps->pic_width_in_luma_samples = ALIGN(state->width, min_cb_size);
	sps->pic_height_in_luma_samples = ALIGN(state->height, min_cb_size);

	/* Conformance window offsets count chroma samples, two luma ones. */
	state->seq.conf_win_right_offset =
		(sps->pic_width_in_luma_samples - state->width) / 2;
	state->seq.conf_win_bottom_offset =
		(sps->pic_height_in_luma_samples - state->height) / 2;

	state->seq.level_idc = level_idc(sps->pic_width_in_luma_samples,
					 sps->pic_height_in_luma_samples,
					 timeperframe);

	if (timeperframe->numerator && timeperframe->denominator) {
		state->seq.vui_flags |= V4L2_HEVC_VUI_FLAG_TIMING_INFO;
		state->seq.num_units_in_tick = timeperframe->numerator;
		state->seq.time_scale = timeperframe->denominator;
	}

	seq_colour(&state->seq, format);

	state->qp_min = ctrl_value(enc, V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP, 0);
	state->qp_max = ctrl_value(enc, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP, 51);
	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_I)
		state->qp = ctrl_value(enc, V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
				       26);
	else
		state->qp = ctrl_value(enc, V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
				       26);
	state->qp = clamp(state->qp, state->qp_min, state->qp_max);

	return 0;
}

/*
 * PicOrderCntVal as 8.3.1 derives it: every picture is a reference picture
 * of TemporalId 0, so prevTid0Pic is the reference picture.
 */
static int ref_step(struct v4l2_hevc_enc *enc)
{
	const struct v4l2_hevc_enc_state *state = &enc->state;
	const struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	struct v4l2_hevc_enc_ref *ref = &enc->ref;
	s32 max_lsb = 1 << (state->sps.log2_max_pic_order_cnt_lsb_minus4 + 4);
	s32 lsb = encode->slice_pic_order_cnt_lsb;
	s32 prev_lsb, prev_msb, msb;

	if (encode->flags & V4L2_HEVC_ENCODE_FLAG_IDR_PIC) {
		ref->pic_order_cnt = 0;
		return 0;
	}

	if (lsb >= max_lsb || !ref->valid)
		return -EINVAL;

	prev_lsb = ref->poc & (max_lsb - 1);
	prev_msb = ref->poc - prev_lsb;

	if (lsb < prev_lsb && prev_lsb - lsb >= max_lsb / 2)
		msb = prev_msb + max_lsb;
	else if (lsb > prev_lsb && lsb - prev_lsb > max_lsb / 2)
		msb = prev_msb - max_lsb;
	else
		msb = prev_msb;

	ref->pic_order_cnt = msb + lsb;

	/* The short-term RPS name the picture just before. */
	if (ref->pic_order_cnt != ref->poc + 1)
		return -EINVAL;

	return 0;
}

static int headers_step(struct v4l2_hevc_enc *enc,
			struct vb2_v4l2_buffer *buffer)
{
	const struct v4l2_hevc_enc_state *state = &enc->state;
	unsigned int size = vb2_plane_size(&buffer->vb2_buf, 0);
	u8 *dst = vb2_plane_vaddr(&buffer->vb2_buf, 0);
	unsigned int n = 0;
	int ret;

	enc->header_size = 0;

	if (!(state->encode.flags & V4L2_HEVC_ENCODE_FLAG_IDR_PIC))
		return 0;

	if (!dst)
		return -EINVAL;

	ret = v4l2_hevc_enc_rbsp_vps(dst, size, &state->sps, &state->seq);
	if (ret < 0)
		return ret;
	n += ret;

	ret = v4l2_hevc_enc_rbsp_sps(dst + n, size - n, &state->sps,
				     &state->seq);
	if (ret < 0)
		return ret;
	n += ret;

	ret = v4l2_hevc_enc_rbsp_pps(dst + n, size - n, &state->sps,
				     &state->pps);
	if (ret < 0)
		return ret;

	enc->header_size = n + ret;

	return 0;
}

static int rec_buffer_alloc(struct v4l2_hevc_enc *enc,
			    struct v4l2_hevc_enc_rec_buffer *buffer)
{
	int ret = enc->ops->rec_buffer_alloc(enc, buffer);

	if (!ret)
		buffer->allocated = true;

	return ret;
}

static void rec_buffer_free(struct v4l2_hevc_enc *enc,
			    struct v4l2_hevc_enc_rec_buffer *buffer)
{
	if (!buffer->allocated)
		return;

	enc->ops->rec_buffer_free(enc, buffer);
	buffer->allocated = false;
}

int v4l2_hevc_enc_init(struct v4l2_hevc_enc *enc)
{
	int ret;

	if (!enc->ops || !enc->ops->rec_buffer_alloc ||
	    !enc->ops->rec_buffer_free || !enc->format_mplane ||
	    !enc->timeperframe || !enc->ctrl_handler)
		return -EINVAL;

	memset(&enc->state, 0, sizeof(enc->state));
	memset(&enc->ref, 0, sizeof(enc->ref));
	enc->header_size = 0;
	enc->pictures = 0;

	ret = rec_buffer_alloc(enc, &enc->ref.buffer_current);
	if (ret)
		return ret;

	if (!(enc->flags & V4L2_HEVC_ENC_FLAG_INTER_PRED))
		return 0;

	ret = rec_buffer_alloc(enc, &enc->ref.buffer);
	if (ret)
		rec_buffer_free(enc, &enc->ref.buffer_current);

	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_init);

void v4l2_hevc_enc_exit(struct v4l2_hevc_enc *enc)
{
	rec_buffer_free(enc, &enc->ref.buffer);
	rec_buffer_free(enc, &enc->ref.buffer_current);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_exit);

/*
 * Prepares the picture of the encode parameters control: its state, its
 * reference and, for an IDR picture, the parameter sets at the start of the
 * coded buffer.
 */
int v4l2_hevc_enc_step(struct v4l2_hevc_enc *enc,
		       struct vb2_v4l2_buffer *buffer)
{
	int ret;

	ret = state_prepare(enc);
	if (ret)
		return ret;

	ret = ref_step(enc);
	if (ret)
		return ret;

	return headers_step(enc, buffer);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_step);

/* The picture is encoded: it becomes the reference of the next. */
void v4l2_hevc_enc_complete(struct v4l2_hevc_enc *enc,
			    struct vb2_v4l2_buffer *buffer)
{
	const struct v4l2_ctrl_hevc_encode_params *encode = &enc->state.encode;
	struct v4l2_hevc_enc_ref *ref = &enc->ref;

	if (encode->flags & V4L2_HEVC_ENCODE_FLAG_IDR_PIC)
		buffer->flags |= V4L2_BUF_FLAG_KEYFRAME;
	else if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_P)
		buffer->flags |= V4L2_BUF_FLAG_PFRAME;

	if (enc->flags & V4L2_HEVC_ENC_FLAG_INTER_PRED) {
		swap(ref->buffer_current, ref->buffer);
		ref->valid = true;
		ref->poc = ref->pic_order_cnt;
	}

	enc->pictures++;
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_complete);

MODULE_DESCRIPTION("V4L2 HEVC Encode Core");
MODULE_LICENSE("GPL");
