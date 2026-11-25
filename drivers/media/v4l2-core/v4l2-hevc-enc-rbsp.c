// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 HEVC Encode RBSP
 *
 * Each parameter set is written to a small RBSP buffer first, then copied
 * into the stream as an Annex B NAL unit with its emulation prevention bytes.
 * Only what an encoder chooses is written: no long-term references, sub-layers
 * or scaling list data, and short-term RPS that list the previous pictures.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <media/v4l2-hevc-enc-rbsp.h>

/* An SPS with timing and colour stays under 100 bytes. */
#define RBSP_SIZE	256

struct rbsp {
	u8 data[RBSP_SIZE];
	unsigned int bits;
};

static void put_bits(struct rbsp *rbsp, u32 value, unsigned int count)
{
	while (count--) {
		unsigned int byte = rbsp->bits / 8;

		if (byte < RBSP_SIZE && (value >> count) & 1)
			rbsp->data[byte] |= BIT(7 - rbsp->bits % 8);
		rbsp->bits++;
	}
}

static void put_flag(struct rbsp *rbsp, u64 flags, u64 flag)
{
	put_bits(rbsp, !!(flags & flag), 1);
}

static void put_ue(struct rbsp *rbsp, u32 value)
{
	u32 code = value + 1;
	unsigned int length = fls(code);

	put_bits(rbsp, 0, length - 1);
	put_bits(rbsp, code, length);
}

static void put_se(struct rbsp *rbsp, s32 value)
{
	put_ue(rbsp, value > 0 ? 2 * value - 1 : -2 * value);
}

static void put_trailing_bits(struct rbsp *rbsp)
{
	put_bits(rbsp, 1, 1);
	put_bits(rbsp, 0, (8 - rbsp->bits % 8) % 8);
}

static int nal_write(u8 *dst, unsigned int size, u8 nal_unit_type,
		     const struct rbsp *rbsp)
{
	unsigned int length = rbsp->bits / 8;
	unsigned int zeros = 0;
	unsigned int i, n = 0;

	if (rbsp->bits > RBSP_SIZE * 8)
		return -ENOSPC;

	/* Start code, header, at most one emulation prevention byte per two. */
	if (size < 6 + length + length / 2)
		return -ENOSPC;

	dst[n++] = 0;
	dst[n++] = 0;
	dst[n++] = 0;
	dst[n++] = 1;

	/* nuh_layer_id 0, nuh_temporal_id_plus1 1 */
	dst[n++] = nal_unit_type << 1;
	dst[n++] = 1;

	for (i = 0; i < length; i++) {
		if (zeros == 2 && rbsp->data[i] <= 3) {
			dst[n++] = 3;
			zeros = 0;
		}

		dst[n++] = rbsp->data[i];
		zeros = rbsp->data[i] ? 0 : zeros + 1;
	}

	return n;
}

static void put_profile_tier_level(struct rbsp *rbsp,
				   const struct v4l2_hevc_enc_seq *seq)
{
	/* general_profile_space */
	put_bits(rbsp, 0, 2);
	put_bits(rbsp, seq->tier_flag, 1);
	put_bits(rbsp, seq->profile_idc, 5);
	/* general_profile_compatibility_flag[], from j = 0 */
	put_bits(rbsp, BIT(31 - seq->profile_idc), 32);
	/* progressive, not interlaced, packed, frames only */
	put_bits(rbsp, 0x9, 4);
	/* general_reserved_zero_43bits, general_inbld_flag */
	put_bits(rbsp, 0, 32);
	put_bits(rbsp, 0, 12);
	put_bits(rbsp, seq->level_idc, 8);
}

int v4l2_hevc_enc_rbsp_vps(u8 *dst, unsigned int size,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_enc_seq *seq)
{
	struct rbsp rbsp = {};

	put_bits(&rbsp, sps->video_parameter_set_id, 4);
	/* vps_base_layer_internal_flag, vps_base_layer_available_flag */
	put_bits(&rbsp, 0x3, 2);
	/* vps_max_layers_minus1, vps_max_sub_layers_minus1 */
	put_bits(&rbsp, 0, 6);
	put_bits(&rbsp, 0, 3);
	/* vps_temporal_id_nesting_flag, vps_reserved_0xffff_16bits */
	put_bits(&rbsp, 1, 1);
	put_bits(&rbsp, 0xffff, 16);
	put_profile_tier_level(&rbsp, seq);
	/* vps_sub_layer_ordering_info_present_flag */
	put_bits(&rbsp, 1, 1);
	put_ue(&rbsp, sps->sps_max_dec_pic_buffering_minus1);
	put_ue(&rbsp, sps->sps_max_num_reorder_pics);
	put_ue(&rbsp, sps->sps_max_latency_increase_plus1);
	/* vps_max_layer_id, vps_num_layer_sets_minus1 */
	put_bits(&rbsp, 0, 6);
	put_ue(&rbsp, 0);
	/* vps_timing_info_present_flag, vps_extension_flag */
	put_bits(&rbsp, 0, 2);
	put_trailing_bits(&rbsp);

	return nal_write(dst, size, V4L2_HEVC_NAL_VPS, &rbsp);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_vps);

static void put_st_ref_pic_set(struct rbsp *rbsp, unsigned int index,
			       unsigned int num_negative_pics)
{
	unsigned int i;

	/* inter_ref_pic_set_prediction_flag */
	if (index)
		put_bits(rbsp, 0, 1);

	put_ue(rbsp, num_negative_pics);
	/* num_positive_pics */
	put_ue(rbsp, 0);

	for (i = 0; i < num_negative_pics; i++) {
		/* delta_poc_s0_minus1: each the picture before the last */
		put_ue(rbsp, 0);
		/* used_by_curr_pic_s0_flag */
		put_bits(rbsp, 1, 1);
	}
}

static void put_vui(struct rbsp *rbsp, const struct v4l2_hevc_enc_seq *seq)
{
	u32 flags = seq->vui_flags;

	/* aspect_ratio_info_present_flag, overscan_info_present_flag */
	put_bits(rbsp, 0, 2);

	put_flag(rbsp, flags, V4L2_HEVC_VUI_FLAG_VIDEO_SIGNAL_TYPE);
	if (flags & V4L2_HEVC_VUI_FLAG_VIDEO_SIGNAL_TYPE) {
		put_bits(rbsp, seq->video_format, 3);
		put_flag(rbsp, flags, V4L2_HEVC_VUI_FLAG_FULL_RANGE);
		put_flag(rbsp, flags, V4L2_HEVC_VUI_FLAG_COLOUR_DESCRIPTION);

		if (flags & V4L2_HEVC_VUI_FLAG_COLOUR_DESCRIPTION) {
			put_bits(rbsp, seq->colour_primaries, 8);
			put_bits(rbsp, seq->transfer_characteristics, 8);
			put_bits(rbsp, seq->matrix_coeffs, 8);
		}
	}

	/*
	 * chroma_loc_info_present_flag, neutral_chroma_indication_flag,
	 * field_seq_flag, frame_field_info_present_flag,
	 * default_display_window_flag
	 */
	put_bits(rbsp, 0, 5);

	put_flag(rbsp, flags, V4L2_HEVC_VUI_FLAG_TIMING_INFO);
	if (flags & V4L2_HEVC_VUI_FLAG_TIMING_INFO) {
		put_bits(rbsp, seq->num_units_in_tick, 32);
		put_bits(rbsp, seq->time_scale, 32);
		/*
		 * vui_poc_proportional_to_timing_flag,
		 * vui_hrd_parameters_present_flag
		 */
		put_bits(rbsp, 0, 2);
	}

	/* bitstream_restriction_flag */
	put_bits(rbsp, 0, 1);
}

int v4l2_hevc_enc_rbsp_sps(u8 *dst, unsigned int size,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_hevc_enc_seq *seq)
{
	bool conformance_window = seq->conf_win_right_offset ||
				  seq->conf_win_bottom_offset;
	struct rbsp rbsp = {};
	u64 flags = sps->flags;
	unsigned int i;

	put_bits(&rbsp, sps->video_parameter_set_id, 4);
	/* sps_max_sub_layers_minus1, sps_temporal_id_nesting_flag */
	put_bits(&rbsp, 0, 3);
	put_bits(&rbsp, 1, 1);
	put_profile_tier_level(&rbsp, seq);
	put_ue(&rbsp, sps->seq_parameter_set_id);

	put_ue(&rbsp, sps->chroma_format_idc);
	if (sps->chroma_format_idc == 3)
		put_flag(&rbsp, flags, V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE);

	put_ue(&rbsp, sps->pic_width_in_luma_samples);
	put_ue(&rbsp, sps->pic_height_in_luma_samples);

	put_bits(&rbsp, conformance_window, 1);
	if (conformance_window) {
		put_ue(&rbsp, 0);
		put_ue(&rbsp, seq->conf_win_right_offset);
		put_ue(&rbsp, 0);
		put_ue(&rbsp, seq->conf_win_bottom_offset);
	}

	put_ue(&rbsp, sps->bit_depth_luma_minus8);
	put_ue(&rbsp, sps->bit_depth_chroma_minus8);
	put_ue(&rbsp, sps->log2_max_pic_order_cnt_lsb_minus4);

	/* sps_sub_layer_ordering_info_present_flag */
	put_bits(&rbsp, 1, 1);
	put_ue(&rbsp, sps->sps_max_dec_pic_buffering_minus1);
	put_ue(&rbsp, sps->sps_max_num_reorder_pics);
	put_ue(&rbsp, sps->sps_max_latency_increase_plus1);

	put_ue(&rbsp, sps->log2_min_luma_coding_block_size_minus3);
	put_ue(&rbsp, sps->log2_diff_max_min_luma_coding_block_size);
	put_ue(&rbsp, sps->log2_min_luma_transform_block_size_minus2);
	put_ue(&rbsp, sps->log2_diff_max_min_luma_transform_block_size);
	put_ue(&rbsp, sps->max_transform_hierarchy_depth_inter);
	put_ue(&rbsp, sps->max_transform_hierarchy_depth_intra);

	put_flag(&rbsp, flags, V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED);
	/* sps_scaling_list_data_present_flag: the default lists */
	if (flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED)
		put_bits(&rbsp, 0, 1);

	put_flag(&rbsp, flags, V4L2_HEVC_SPS_FLAG_AMP_ENABLED);
	put_flag(&rbsp, flags, V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET);

	put_flag(&rbsp, flags, V4L2_HEVC_SPS_FLAG_PCM_ENABLED);
	if (flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) {
		put_bits(&rbsp, sps->pcm_sample_bit_depth_luma_minus1, 4);
		put_bits(&rbsp, sps->pcm_sample_bit_depth_chroma_minus1, 4);
		put_ue(&rbsp, sps->log2_min_pcm_luma_coding_block_size_minus3);
		put_ue(&rbsp, sps->log2_diff_max_min_pcm_luma_coding_block_size);
		put_flag(&rbsp, flags,
			 V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED);
	}

	put_ue(&rbsp, sps->num_short_term_ref_pic_sets);
	for (i = 0; i < sps->num_short_term_ref_pic_sets; i++)
		put_st_ref_pic_set(&rbsp, i,
				   sps->sps_max_dec_pic_buffering_minus1);

	/* long_term_ref_pics_present_flag */
	put_bits(&rbsp, 0, 1);
	put_flag(&rbsp, flags, V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED);
	put_flag(&rbsp, flags,
		 V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED);

	/* vui_parameters_present_flag */
	put_bits(&rbsp, 1, 1);
	put_vui(&rbsp, seq);

	/* sps_extension_present_flag */
	put_bits(&rbsp, 0, 1);
	put_trailing_bits(&rbsp);

	return nal_write(dst, size, V4L2_HEVC_NAL_SPS, &rbsp);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_sps);

int v4l2_hevc_enc_rbsp_pps(u8 *dst, unsigned int size,
			   const struct v4l2_ctrl_hevc_sps *sps,
			   const struct v4l2_ctrl_hevc_pps *pps)
{
	struct rbsp rbsp = {};
	u64 flags = pps->flags;
	unsigned int i;

	put_ue(&rbsp, pps->pic_parameter_set_id);
	put_ue(&rbsp, sps->seq_parameter_set_id);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT);
	put_bits(&rbsp, pps->num_extra_slice_header_bits, 3);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT);
	put_ue(&rbsp, pps->num_ref_idx_l0_default_active_minus1);
	put_ue(&rbsp, pps->num_ref_idx_l1_default_active_minus1);
	put_se(&rbsp, pps->init_qp_minus26);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED);

	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED);
	if (flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED)
		put_ue(&rbsp, pps->diff_cu_qp_delta_depth);

	put_se(&rbsp, pps->pps_cb_qp_offset);
	put_se(&rbsp, pps->pps_cr_qp_offset);
	put_flag(&rbsp, flags,
		 V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_TILES_ENABLED);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED);

	if (flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) {
		put_ue(&rbsp, pps->num_tile_columns_minus1);
		put_ue(&rbsp, pps->num_tile_rows_minus1);
		put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING);

		if (!(flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)) {
			for (i = 0; i < pps->num_tile_columns_minus1; i++)
				put_ue(&rbsp, pps->column_width_minus1[i]);
			for (i = 0; i < pps->num_tile_rows_minus1; i++)
				put_ue(&rbsp, pps->row_height_minus1[i]);
		}

		put_flag(&rbsp, flags,
			 V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED);
	}

	put_flag(&rbsp, flags,
		 V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED);

	put_flag(&rbsp, flags,
		 V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT);
	if (flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		put_flag(&rbsp, flags,
			 V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED);
		put_flag(&rbsp, flags,
			 V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER);

		if (!(flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER)) {
			put_se(&rbsp, pps->pps_beta_offset_div2);
			put_se(&rbsp, pps->pps_tc_offset_div2);
		}
	}

	/* pps_scaling_list_data_present_flag */
	put_bits(&rbsp, 0, 1);
	put_flag(&rbsp, flags, V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT);
	put_ue(&rbsp, pps->log2_parallel_merge_level_minus2);
	put_flag(&rbsp, flags,
		 V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT);

	/* pps_extension_present_flag */
	put_bits(&rbsp, 0, 1);
	put_trailing_bits(&rbsp);

	return nal_write(dst, size, V4L2_HEVC_NAL_PPS, &rbsp);
}
EXPORT_SYMBOL_GPL(v4l2_hevc_enc_rbsp_pps);

MODULE_DESCRIPTION("V4L2 HEVC Encode RBSP");
MODULE_LICENSE("GPL");
