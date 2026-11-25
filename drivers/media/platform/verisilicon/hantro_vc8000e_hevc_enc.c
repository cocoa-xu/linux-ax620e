// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver: HEVC encoding on the VeriSilicon VC8000E
 *
 * Programmed the way the Axera AX620E's own firmware programs this core:
 * coding blocks of 8x8 to 64x64, transforms of 4x4 to 16x16, AMP and SAO,
 * CABAC, and one reference picture named by the first of two short-term RPS
 * in the SPS. The core writes each slice segment header from its registers,
 * so the parameter sets must say what the registers say.
 */

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-hevc-enc.h>

#include "hantro.h"
#include "hantro_hw.h"
#include "hantro_vc8000e_regs.h"

/* The slice header picks one of two SPS short-term RPS with a 1-bit index. */
#define ST_RPS_COUNT	2

/*
 * Lambdas by QP: the square root of 0.2 * 2^((QP - 12) / 3) in 1/128 for
 * the SATD costs, and 64 or 48 times 2^((QP - 31) / 3) for SAO.
 */
static const u16 satd_lambda[52] = {
	14, 16, 18, 20, 23, 25, 29, 32,
	36, 40, 45, 51, 57, 64, 72, 81,
	91, 102, 114, 129, 144, 162, 182, 204,
	229, 257, 288, 324, 363, 408, 458, 514,
	577, 648, 727, 816, 916, 1028, 1154, 1295,
	1454, 1632, 1832, 2056, 2308, 2591, 2908, 3264,
	3664, 4112, 4616, 5181,
};

static const u16 sao_lambda_luma[52] = {
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	2, 2, 3, 4, 5, 6, 8, 10,
	12, 16, 20, 25, 32, 40, 50, 64,
	80, 101, 128, 161, 203, 256, 322, 406,
	512, 645, 812, 1024, 1290, 1625, 2048, 2580,
	3250, 4096, 5160, 6501,
};

static const u16 sao_lambda_chroma[52] = {
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 2, 3, 3, 4, 6, 7,
	9, 12, 15, 19, 24, 30, 38, 48,
	60, 76, 96, 120, 152, 192, 241, 304,
	384, 483, 609, 768, 967, 1219, 1536, 1935,
	2438, 3072, 3870, 4876,
};

static int
hantro_vc8000e_hevc_enc_state_constrain(struct v4l2_hevc_enc *enc,
					struct v4l2_hevc_enc_state *state)
{
	struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	struct v4l2_ctrl_hevc_pps *pps = &state->pps;
	struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;

	/* SPS */

	sps->chroma_format_idc = 1;
	sps->bit_depth_luma_minus8 = 0;
	sps->bit_depth_chroma_minus8 = 0;

	/* The core writes slice_pic_order_cnt_lsb with 16 bits. */
	sps->log2_max_pic_order_cnt_lsb_minus4 = 12;

	sps->log2_min_luma_coding_block_size_minus3 = 0;
	sps->log2_diff_max_min_luma_coding_block_size = 3;
	sps->log2_min_luma_transform_block_size_minus2 = 0;
	sps->log2_diff_max_min_luma_transform_block_size = 2;
	sps->max_transform_hierarchy_depth_inter = 4;
	sps->max_transform_hierarchy_depth_intra = 2;

	sps->flags = V4L2_HEVC_SPS_FLAG_AMP_ENABLED |
		     V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET;

	if (sps->num_short_term_ref_pic_sets)
		sps->num_short_term_ref_pic_sets = ST_RPS_COUNT;

	/* PPS */

	pps->flags &= V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;
	pps->flags |= V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT |
		      V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED |
		      V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED |
		      V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;

	pps->num_extra_slice_header_bits = 0;
	pps->num_ref_idx_l0_default_active_minus1 = 0;
	pps->num_ref_idx_l1_default_active_minus1 = 0;
	pps->init_qp_minus26 = clamp_t(s8, pps->init_qp_minus26, -26, 25);
	pps->diff_cu_qp_delta_depth = 0;

	/* One chroma QP offset register for both. */
	pps->pps_cb_qp_offset = clamp_t(s8, pps->pps_cb_qp_offset, -12, 12);
	pps->pps_cr_qp_offset = pps->pps_cb_qp_offset;

	pps->num_tile_columns_minus1 = 0;
	pps->num_tile_rows_minus1 = 0;
	pps->log2_parallel_merge_level_minus2 = 0;

	/* Encode */

	encode->flags &= V4L2_HEVC_ENCODE_FLAG_IDR_PIC |
			 V4L2_HEVC_ENCODE_FLAG_CABAC_INIT;
	encode->flags |= V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_LUMA |
			 V4L2_HEVC_ENCODE_FLAG_SLICE_SAO_CHROMA;

	return 0;
}

static int
hantro_vc8000e_hevc_enc_rec_buffer_alloc(struct v4l2_hevc_enc *enc,
					 struct v4l2_hevc_enc_rec_buffer *buffer)
{
	struct hantro_ctx *ctx = enc->private_data;
	struct device *dev = ctx->dev->dev;
	struct hantro_vc8000e_rec_buf *rec_buf;
	unsigned int width = ALIGN(ctx->src_fmt.width, 64);
	unsigned int height = ALIGN(ctx->src_fmt.height, 64);
	struct hantro_aux_buf *aux[3];
	unsigned int i;

	rec_buf = kzalloc(sizeof(*rec_buf), GFP_KERNEL);
	if (!rec_buf)
		return -ENOMEM;

	/* Whole coding tree blocks, a 4x downscaled luma for the search. */
	rec_buf->luma.size = width * height;
	rec_buf->chroma.size = width * height / 2;
	rec_buf->luma_4n.size = width * height / 16;

	aux[0] = &rec_buf->luma;
	aux[1] = &rec_buf->chroma;
	aux[2] = &rec_buf->luma_4n;

	for (i = 0; i < ARRAY_SIZE(aux); i++) {
		aux[i]->cpu = dma_alloc_coherent(dev, aux[i]->size,
						 &aux[i]->dma, GFP_KERNEL);
		if (!aux[i]->cpu)
			goto error;
	}

	buffer->private_data = rec_buf;

	return 0;

error:
	while (i--)
		dma_free_coherent(dev, aux[i]->size, aux[i]->cpu, aux[i]->dma);
	kfree(rec_buf);

	return -ENOMEM;
}

static void
hantro_vc8000e_hevc_enc_rec_buffer_free(struct v4l2_hevc_enc *enc,
					struct v4l2_hevc_enc_rec_buffer *buffer)
{
	struct hantro_ctx *ctx = enc->private_data;
	struct device *dev = ctx->dev->dev;
	struct hantro_vc8000e_rec_buf *rec_buf = buffer->private_data;

	dma_free_coherent(dev, rec_buf->luma.size, rec_buf->luma.cpu,
			  rec_buf->luma.dma);
	dma_free_coherent(dev, rec_buf->chroma.size, rec_buf->chroma.cpu,
			  rec_buf->chroma.dma);
	dma_free_coherent(dev, rec_buf->luma_4n.size, rec_buf->luma_4n.cpu,
			  rec_buf->luma_4n.dma);

	kfree(rec_buf);
	buffer->private_data = NULL;
}

static const struct v4l2_hevc_enc_ops hantro_vc8000e_hevc_enc_ops = {
	.state_constrain	= hantro_vc8000e_hevc_enc_state_constrain,
	.rec_buffer_alloc	= hantro_vc8000e_hevc_enc_rec_buffer_alloc,
	.rec_buffer_free	= hantro_vc8000e_hevc_enc_rec_buffer_free,
};

int hantro_vc8000e_hevc_enc_init(struct hantro_ctx *ctx)
{
	struct hantro_hevc_enc_hw_ctx *hevc_ctx = &ctx->hevc_enc;
	struct hantro_aux_buf *nal_tbl = &hevc_ctx->nal_tbl;
	struct v4l2_hevc_enc *enc = &hevc_ctx->enc;
	struct device *dev = ctx->dev->dev;
	int ret;

	/* The sizes of the NAL units of a picture: one slice. */
	nal_tbl->size = PAGE_SIZE;
	nal_tbl->cpu = dma_alloc_coherent(dev, nal_tbl->size, &nal_tbl->dma,
					  GFP_KERNEL);
	if (!nal_tbl->cpu)
		return -ENOMEM;

	enc->ops = &hantro_vc8000e_hevc_enc_ops;
	enc->private_data = ctx;
	enc->format_mplane = &ctx->dst_fmt;
	enc->timeperframe = &ctx->dst_timeperframe;
	enc->ctrl_handler = &ctx->ctrl_handler;
	enc->flags = V4L2_HEVC_ENC_FLAG_INTER_PRED;

	ret = v4l2_hevc_enc_init(enc);
	if (ret)
		dma_free_coherent(dev, nal_tbl->size, nal_tbl->cpu,
				  nal_tbl->dma);

	return ret;
}

void hantro_vc8000e_hevc_enc_exit(struct hantro_ctx *ctx)
{
	struct hantro_hevc_enc_hw_ctx *hevc_ctx = &ctx->hevc_enc;
	struct hantro_aux_buf *nal_tbl = &hevc_ctx->nal_tbl;

	v4l2_hevc_enc_exit(&hevc_ctx->enc);
	dma_free_coherent(ctx->dev->dev, nal_tbl->size, nal_tbl->cpu,
			  nal_tbl->dma);
}

static void lambda_setup(struct hantro_vc8000e_regs *regs,
			 const struct v4l2_hevc_enc_state *state)
{
	bool intra = state->encode.slice_type == V4L2_HEVC_SLICE_TYPE_I;
	unsigned int qp = state->qp;
	/* Sixteen QPs down from here, for coding units below the slice QP. */
	unsigned int top = max(qp, 15U);

	regs->swreg37.lambda_sao_luma = sao_lambda_luma[qp];
	regs->swreg37.lambda_sao_chroma = sao_lambda_chroma[qp];

	regs->swreg125.intra_satd_lambda_0 = satd_lambda[top];
	regs->swreg125.intra_satd_lambda_1 = satd_lambda[top - 1];
	regs->swreg126.intra_satd_lambda_2 = satd_lambda[top - 2];
	regs->swreg126.intra_satd_lambda_3 = satd_lambda[top - 3];
	regs->swreg127.intra_satd_lambda_4 = satd_lambda[top - 4];
	regs->swreg127.intra_satd_lambda_5 = satd_lambda[top - 5];
	regs->swreg128.intra_satd_lambda_6 = satd_lambda[top - 6];
	regs->swreg128.intra_satd_lambda_7 = satd_lambda[top - 7];
	regs->swreg129.intra_satd_lambda_8 = satd_lambda[top - 8];
	regs->swreg129.intra_satd_lambda_9 = satd_lambda[top - 9];
	regs->swreg130.intra_satd_lambda_10 = satd_lambda[top - 10];
	regs->swreg130.intra_satd_lambda_11 = satd_lambda[top - 11];
	regs->swreg131.intra_satd_lambda_12 = satd_lambda[top - 12];
	regs->swreg131.intra_satd_lambda_13 = satd_lambda[top - 13];
	regs->swreg132.intra_satd_lambda_14 = satd_lambda[top - 14];
	regs->swreg132.intra_satd_lambda_15 = satd_lambda[top - 15];

	/* The core derives the motion lambdas from the QP with these. */
	if (regs->swreg214.hwabsqpsupport) {
		regs->swreg170_qp_absolute.sse_qp_factor = intra ? 13095 : 13107;
		regs->swreg171_qp_absolute.sad_qp_factor = intra ? 14647 : 14654;
	}
}

static void mode_decision_setup(struct hantro_vc8000e_regs *regs)
{
	regs->swreg26.intra_size_factor_0 = 337;
	regs->swreg26.intra_size_factor_1 = 337;
	regs->swreg26.intra_size_factor_2 = 472;
	regs->swreg27.intra_size_factor_3 = 472;

	regs->swreg27.intra_mode_factor_0 = 24;
	regs->swreg27.intra_mode_factor_1 = 37;
	regs->swreg27.intra_mode_factor_2 = 78;

	regs->swreg35.bits_est_tu_split_penalty = 3;
	regs->swreg35.bits_est_bias_intra_cu_8 = 14;
	regs->swreg35.bits_est_bias_intra_cu_16 = 26;
	regs->swreg36.bits_est_bias_intra_cu_32 = 57;
	regs->swreg36.bits_est_bias_intra_cu_64 = 202;
	regs->swreg36.bits_est_1n_cu_penalty = 5;
	regs->swreg36.inter_skip_bias = 124;

	regs->swreg201.mean_thr0 = 5;
	regs->swreg201.mean_thr1 = 5;
	regs->swreg201.mean_thr2 = 5;
	regs->swreg201.mean_thr3 = 5;

	regs->swreg202.thr_dc_lum_8x8 = 2;
	regs->swreg202.thr_dc_chroma_8x8 = 2;
	regs->swreg203_hevc.thr_dc_lum_16x16 = 2;
	regs->swreg203_hevc.thr_dc_chroma_16x16 = 2;
	regs->swreg204.thr_dc_lum_32x32 = 2;
	regs->swreg204.thr_dc_chroma_32x32 = 2;

	regs->swreg205.thr_ac_num_lum_8x8 = 12;
	regs->swreg205.thr_ac_num_chroma_8x8 = 3;
	regs->swreg206.thr_ac_num_lum_16x16 = 51;
	regs->swreg206.thr_ac_num_chroma_16x16 = 12;
	regs->swreg207.thr_ac_num_lum_32x32 = 204;
	regs->swreg207.thr_ac_num_chroma_32x32 = 51;

	regs->swreg208_hevc.mdqpy = 30;
	regs->swreg208_hevc.mdqpc = 30;

	regs->swreg182.qp_delta_gain = 313;

	regs->swreg224.cb_const_pixel = 0x80;
	regs->swreg224.cr_const_pixel = 0x80;

	regs->swreg225.roimap_qpdelta_ver = 1;
	regs->swreg225.num_tile_rows = 1;
	regs->swreg225.num_tile_columns = 1;
	regs->swreg225.loop_filter_across_tiles_enabled_flag = 1;

	regs->swreg226.me_vert_searchrange_hevc = 5;
	regs->swreg288.cuinfoversion = 1;

	regs->swreg307.extlinebuffer_linecnt_lum_fwd = 16;
	regs->swreg307.extlinebuffer_linecnt_lum_bwd = 16;
	regs->swreg307.extlinebuffer_linecnt_chr_fwd = 8;
	regs->swreg307.extlinebuffer_linecnt_chr_bwd = 8;
}

int hantro_vc8000e_hevc_enc_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct v4l2_pix_format_mplane *src_fmt = &ctx->src_fmt;
	struct hantro_hevc_enc_hw_ctx *hevc_ctx = &ctx->hevc_enc;
	struct hantro_vc8000e_regs *regs = &hevc_ctx->vc8000e_regs;
	struct v4l2_hevc_enc *enc = &hevc_ctx->enc;
	const struct v4l2_hevc_enc_state *state = &enc->state;
	const struct v4l2_ctrl_hevc_sps *sps = &state->sps;
	const struct v4l2_ctrl_hevc_pps *pps = &state->pps;
	const struct v4l2_ctrl_hevc_encode_params *encode = &state->encode;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct hantro_vc8000e_rec_buf *rec_buf;
	const struct v4l2_format_info *info;
	unsigned int luma_stride, chroma_stride;
	unsigned int coded_width;
	dma_addr_t src_dma;
	bool idr;
	int ret;

	hantro_start_prepare_run(ctx);

	info = v4l2_format_info(src_fmt->pixelformat);
	if (!info)
		return -EINVAL;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	ret = v4l2_hevc_enc_step(enc, dst_buf);
	if (ret)
		return ret;

	idr = encode->flags & V4L2_HEVC_ENCODE_FLAG_IDR_PIC;
	coded_width = sps->pic_width_in_luma_samples;

	memset(regs, 0, sizeof(*regs));

	/* Read relevant read-only registers. */
	hantro_vc8000e_swreg_read(vpu, regs, swreg0);
	hantro_vc8000e_swreg_read(vpu, regs, swreg80);
	hantro_vc8000e_swreg_read(vpu, regs, swreg214);
	hantro_vc8000e_swreg_read(vpu, regs, swreg226);
	hantro_vc8000e_swreg_read(vpu, regs, swreg287);

	if (!regs->swreg80.hwhevcsupport)
		return -ENODEV;

	regs->swreg4.mode = HANTRO_VC8000E_SWREG4_MODE_HEVC;

	/* Input */

	regs->swreg38.input_format = ctx->vpu_src_fmt->enc_fmt;
	regs->swreg38.input_rotation = HANTRO_VC8000E_SWREG38_INPUT_ROTATION_0;
	regs->swreg38.rowlength = src_fmt->width;

	/* Pixels per line for packed formats, bytes per line otherwise. */
	luma_stride = src_fmt->plane_fmt[0].bytesperline;
	if (info->comp_planes == 1)
		luma_stride /= info->bpp[0];
	regs->swreg210.input_lu_stride = luma_stride;

	src_dma = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	regs->swreg12.input_y_base = src_dma;

	if (info->comp_planes > 1) {
		chroma_stride = src_fmt->num_planes > 1 ?
				src_fmt->plane_fmt[1].bytesperline :
				luma_stride;
		regs->swreg211.input_ch_stride = chroma_stride;
		regs->swreg13.input_cb_base = src_fmt->num_planes > 1 ?
			vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 1) :
			src_dma + luma_stride * src_fmt->height;
	}

	/* Output: the slice after the parameter sets the core wrote. */

	regs->swreg4.output_strm_mode =
		HANTRO_VC8000E_SWREG4_OUTPUT_STRM_MODE_BYTE_STREAM;
	regs->swreg8.output_strm_base =
		vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0) +
		enc->header_size;
	regs->swreg9.output_strm_buffer_limit =
		vb2_plane_size(&dst_buf->vb2_buf, 0) - enc->header_size;

	regs->swreg10.size_tbl_base = hevc_ctx->nal_tbl.dma;
	regs->swreg6.nal_size_write = 1;
	regs->swreg196.num_ctb_rows_per_sync = 1;
	regs->swreg199.hash_type = HANTRO_VC8000E_SWREG199_HASH_TYPE_NONE;

	/* Picture */

	regs->swreg5.pic_width = coded_width / 8;
	regs->swreg5.pic_height = sps->pic_height_in_luma_samples / 8;
	regs->swreg281.chroma_format_idc = sps->chroma_format_idc;
	regs->swreg38.output_bitwidth_lum =
		HANTRO_VC8000E_SWREG38_OUTPUT_BITWIDTH_LUM_8_BIT;

	regs->swreg11.poc = enc->ref.pic_order_cnt;
	regs->swreg277.log2_max_pic_order_cnt_lsb =
		sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
	regs->swreg277.log2_max_frame_num = 12;

	/* Coding tree and transforms */

	regs->swreg4.min_cb_size = sps->log2_min_luma_coding_block_size_minus3;
	regs->swreg4.max_cb_size = sps->log2_min_luma_coding_block_size_minus3 +
				   sps->log2_diff_max_min_luma_coding_block_size;
	regs->swreg4.min_trb_size =
		sps->log2_min_luma_transform_block_size_minus2;
	regs->swreg4.max_trb_size =
		sps->log2_min_luma_transform_block_size_minus2 +
		sps->log2_diff_max_min_luma_transform_block_size;
	regs->swreg4.max_trans_hierarchy_depth_inter =
		sps->max_transform_hierarchy_depth_inter;
	regs->swreg4.max_trans_hierarchy_depth_intra =
		sps->max_transform_hierarchy_depth_intra;
	regs->swreg4.sao_enable =
		!!(sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET);

	/* Reconstruction, in 4x4 tiles: a stride covers four lines. */

	regs->swreg212.ref_lu_stride = coded_width * 4;
	regs->swreg237.ref_ch_stride = coded_width * 4;
	regs->swreg213.ref_ds_lu_stride = coded_width;

	rec_buf = enc->ref.buffer_current.private_data;
	regs->swreg15.recon_y_base = rec_buf->luma.dma;
	regs->swreg16.recon_chroma_base = rec_buf->chroma.dma;
	regs->swreg72.recon_luma_4n_base = rec_buf->luma_4n.dma;

	/* Reference: the picture before, as the first SPS short-term RPS says. */

	regs->swreg6.num_short_term_ref_pic_sets =
		sps->num_short_term_ref_pic_sets;
	regs->swreg6.rps_id = idr ? sps->num_short_term_ref_pic_sets : 0;
	regs->swreg236.short_term_ref_pic_set_sps_flag = 1;

	regs->swreg17.l0_used_by_curr_pic0 = 1;
	regs->swreg17.l0_delta_poc0 = 0x3ff;
	regs->swreg197.l0_delta_poc0_msb = 0x3ff;
	regs->swreg193.l0_used_by_next_pic0 = 1;
	regs->swreg193.l0_used_by_next_pic1 = 1;
	regs->swreg194.l1_used_by_next_pic0 = 1;
	regs->swreg194.l1_used_by_next_pic1 = 1;

	if (encode->slice_type == V4L2_HEVC_SLICE_TYPE_P) {
		struct hantro_vc8000e_rec_buf *ref_buf =
			enc->ref.buffer.private_data;

		regs->swreg17.active_l0_cnt = 1;
		regs->swreg18.refpic_recon_l0_y0 = ref_buf->luma.dma;
		regs->swreg19.refpic_recon_l0_chroma0 = ref_buf->chroma.dma;
		regs->swreg74.refpic_recon_l0_4n0_base = ref_buf->luma_4n.dma;
	}

	/* Slice */

	regs->swreg5.frame_coding_type =
		encode->slice_type == V4L2_HEVC_SLICE_TYPE_I ?
		HANTRO_VC8000E_SWREG5_FRAME_CODING_TYPE_I :
		HANTRO_VC8000E_SWREG5_FRAME_CODING_TYPE_P;
	regs->swreg191.nal_unit_type = idr ? V4L2_HEVC_NAL_IDR_W_RADL :
					     V4L2_HEVC_NAL_TRAIL_R;
	regs->swreg191.pps_id = encode->pic_parameter_set_id;
	regs->swreg193.entropy_coding_mode = 1;
	regs->swreg7.cabac_init_flag =
		!!(encode->flags & V4L2_HEVC_ENCODE_FLAG_CABAC_INIT);

	/* Quantization */

	regs->swreg7.pic_init_qp = pps->init_qp_minus26 + 26;
	regs->swreg7.pic_qp = state->qp;
	regs->swreg172.qp_min = state->qp_min;
	regs->swreg173.qp_max = state->qp_max;
	regs->swreg4.chroma_qp_offset = pps->pps_cb_qp_offset;
	regs->swreg6.cu_qp_delta_enabled =
		!!(pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED);

	/* Deblocking, as the PPS says: the slices do not override it. */

	regs->swreg6.deblocking_filter_dis =
		!!(pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER);
	regs->swreg6.deblocking_tc_offset = pps->pps_tc_offset_div2;
	regs->swreg6.deblocking_beta_offset = pps->pps_beta_offset_div2;

	lambda_setup(regs, state);
	mode_decision_setup(regs);
	hantro_vc8000e_areas_disable(regs);

	/* Bus */

	regs->swreg272.wr_urgent_disable_threshold =
		HANTRO_VC8000E_SWREG272_URGENT_THRESHOLD_DISABLE;
	regs->swreg272.wr_urgent_enable_threshold =
		HANTRO_VC8000E_SWREG272_URGENT_THRESHOLD_DISABLE;
	regs->swreg272.rd_urgent_disable_threshold =
		HANTRO_VC8000E_SWREG272_URGENT_THRESHOLD_DISABLE;
	regs->swreg272.rd_urgent_enable_threshold =
		HANTRO_VC8000E_SWREG272_URGENT_THRESHOLD_DISABLE;

	regs->swreg81.max_burst = 16;
	regs->swreg261.axi_read_outstanding_num = 64;
	regs->swreg246.axi_write_outstanding_num = 64;
	regs->swreg320.axi_burst_align_rd_lu_ref_prefetch = 1;

	regs->swreg3.clock_gate_inter_h264_e = 1;
	regs->swreg3.clock_gate_inter_h265_e = 1;
	regs->swreg3.clock_gate_inter_e = 1;
	regs->swreg3.clock_gate_encoder_h264_e = 1;
	regs->swreg3.clock_gate_encoder_h265_e = 1;
	regs->swreg3.clock_gate_encoder_e = 1;

	/* Interrupts: clear every status, raise one per picture. */

	regs->swreg1.irq = 1;
	regs->swreg1.frame_rdy_status = 1;
	regs->swreg1.bus_error_status = 1;
	regs->swreg1.sw_reset = 1;
	regs->swreg1.buffer_full = 1;
	regs->swreg1.timeout = 1;
	regs->swreg1.irq_line_buffer = 1;
	regs->swreg1.slice_rdy_status = 1;
	regs->swreg1.irq_dis = 0;
	regs->swreg1.timeout_int = 1;

	hantro_io_copy(vpu->enc_base, regs, sizeof(*regs));

	hantro_end_prepare_run(ctx);

	regs->swreg5.enable = 1;
	hantro_vc8000e_swreg_write(vpu, regs, swreg5);

	return 0;
}

void hantro_vc8000e_hevc_enc_done(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct hantro_hevc_enc_hw_ctx *hevc_ctx = &ctx->hevc_enc;
	struct hantro_vc8000e_regs *regs = &hevc_ctx->vc8000e_regs;
	struct v4l2_hevc_enc *enc = &hevc_ctx->enc;
	struct vb2_v4l2_buffer *dst_buf = hantro_get_dst_buf(ctx);

	/* The core leaves the bytes it wrote in the buffer limit register. */
	hantro_vc8000e_swreg_read(vpu, regs, swreg9);

	vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
			      regs->swreg9.output_strm_buffer_limit +
			      enc->header_size);

	v4l2_hevc_enc_complete(enc, dst_buf);
}
