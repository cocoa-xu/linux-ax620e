// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver
 *
 * Copyright (C) 2025-2026 Paul Kocialkowski <paulk@sys-base.io>
 */

#include "hantro.h"
#include "hantro_vc8000e_regs.h"

irqreturn_t hantro_vc8000e_irq(int irq, void *dev_id)
{
	struct hantro_dev *vpu = dev_id;
	u32 regs_buffer[HANTRO_VC8000E_SWREG_OFFSET(swreg6) / 4];
	struct hantro_vc8000e_regs *regs =
		(struct hantro_vc8000e_regs *)regs_buffer;
	enum vb2_buffer_state state;

	hantro_vc8000e_swreg_read(vpu, regs, swreg1);

	/* Nothing pending (irq, status bits 2-9, segment ready): not ours. */
	if (!(regs_buffer[HANTRO_VC8000E_SWREG_OFFSET(swreg1) / 4] &
	      (BIT(0) | GENMASK(9, 2) | BIT(12))))
		return IRQ_NONE;

	pr_debug("+ hantro-vc8000e-irq: %#x\n",
		 regs_buffer[HANTRO_VC8000E_SWREG_OFFSET(swreg1) / 4]);

	if (regs->swreg1.irq)
		pr_debug("  - irq\n");
	if (regs->swreg1.frame_rdy_status)
		pr_debug("  - frame ready\n");
	if (regs->swreg1.bus_error_status)
		pr_debug("  - bus error\n");
	if (regs->swreg1.sw_reset)
		pr_debug("  - sw reset\n");
	if (regs->swreg1.buffer_full)
		pr_debug("  - buffer full\n");
	if (regs->swreg1.timeout)
		pr_debug("  - timeout\n");
	if (regs->swreg1.slice_rdy_status)
		pr_debug("  - slice ready\n");
	if (regs->swreg1.irq_fuse_error)
		pr_debug("  - fuse error\n");
	if (regs->swreg1.strm_segment_rdy_int)
		pr_debug("  - segment ready\n");

	hantro_vc8000e_swreg_read(vpu, regs, swreg4);
	hantro_vc8000e_swreg_read(vpu, regs, swreg5);

	/* Make sure to disble the encoder on error for safety. */
	if ((regs->swreg1.bus_error_status ||
	     regs->swreg1.buffer_full ||
	     regs->swreg1.timeout ||
	     regs->swreg1.irq_fuse_error) &&
	    (regs->swreg4.mode == HANTRO_VC8000E_SWREG4_MODE_H264 ||
	     regs->swreg4.mode == HANTRO_VC8000E_SWREG4_MODE_HEVC)) {
		regs->swreg5.enable = 0;
		hantro_vc8000e_swreg_write(vpu, regs, swreg5);
	}

	state = regs->swreg1.frame_rdy_status ? VB2_BUF_STATE_DONE :
						VB2_BUF_STATE_ERROR;

	regs->swreg1.irq_dis = 1;
	regs->swreg1.timeout_int = 0;

	hantro_vc8000e_swreg_write(vpu, regs, swreg1);

	hantro_irq_done(vpu, state);

	return IRQ_HANDLED;
}

/* Empty intra, IPCM and ROI areas: none of them applies. */
void hantro_vc8000e_areas_disable(struct hantro_vc8000e_regs *regs)
{
	/* Intra */

	regs->swreg23.intra_area_left = 0xff;
	regs->swreg195.intra_area_left_msb = 1;
	regs->swreg249.intra_area_left_msb2 = 1;

	regs->swreg23.intra_area_right = 0xff;
	regs->swreg195.intra_area_right_msb = 1;
	regs->swreg249.intra_area_right_msb2 = 1;

	regs->swreg23.intra_area_top = 0xff;
	regs->swreg195.intra_area_top_msb = 1;
	regs->swreg249.intra_area_top_msb2 = 1;

	regs->swreg23.intra_area_bottom = 0xff;
	regs->swreg195.intra_area_bottom_msb = 1;
	regs->swreg249.intra_area_bottom_msb2 = 1;

	/* IPCM1 */

	regs->swreg208_h264.ipcm1_left = 0x1ff;
	regs->swreg249.ipcm1_left_msb = 1;

	regs->swreg209.ipcm1_right = 0x1ff;
	regs->swreg249.ipcm1_right_msb = 1;

	regs->swreg209.ipcm1_top = 0x1ff;
	regs->swreg209.ipcm1_bottom = 0x1ff;

	regs->swreg249.ipcm1_top_msb = 1;
	regs->swreg249.ipcm1_bottom_msb = 1;

	/* IPCM2 */

	regs->swreg210.ipcm2_left = 0x1ff;
	regs->swreg249.ipcm2_left_msb = 1;

	regs->swreg211.ipcm2_right = 0x1ff;
	regs->swreg249.ipcm2_right_msb = 1;

	regs->swreg212.ipcm2_top = 0x1ff;
	regs->swreg249.ipcm2_top_msb = 1;

	regs->swreg213.ipcm2_bottom = 0x1ff;
	regs->swreg249.ipcm2_bottom_msb = 1;

	/* ROI1 */

	regs->swreg24.roi1_left = 0xff;
	regs->swreg195.roi1_left_msb = 1;
	regs->swreg249.roi1_left_msb2 = 1;

	regs->swreg24.roi1_right = 0xff;
	regs->swreg195.roi1_right_msb = 1;
	regs->swreg249.roi1_right_msb2 = 1;

	regs->swreg24.roi1_top = 0xff;
	regs->swreg195.roi1_top_msb = 1;
	regs->swreg249.roi1_top_msb2 = 1;

	regs->swreg24.roi1_bottom = 0xff;
	regs->swreg195.roi1_bottom_msb = 1;
	regs->swreg249.roi1_bottom_msb2 = 1;

	/* ROI2 */

	regs->swreg25.roi2_left = 0xff;
	regs->swreg195.roi2_left_msb = 1;
	regs->swreg249.roi2_left_msb2 = 1;

	regs->swreg25.roi2_right = 0xff;
	regs->swreg195.roi2_right_msb = 1;
	regs->swreg249.roi2_right_msb2 = 1;

	regs->swreg25.roi2_top = 0xff;
	regs->swreg195.roi2_top_msb = 1;
	regs->swreg249.roi2_top_msb2 = 1;

	regs->swreg25.roi2_bottom = 0xff;
	regs->swreg195.roi2_bottom_msb = 1;
	regs->swreg249.roi2_bottom_msb2 = 1;

	if (regs->swreg226.hwroi8support) {
		/* ROI3 */

		regs->swreg252.roi3_left = 0x3ff;
		regs->swreg252.roi3_right = 0x3ff;
		regs->swreg252.roi3_top = 0x3ff;
		regs->swreg253.roi3_bottom = 0x3ff;

		/* ROI4 */

		regs->swreg253.roi4_left = 0x3ff;
		regs->swreg254.roi4_right = 0x3ff;
		regs->swreg253.roi4_top = 0x3ff;
		regs->swreg254.roi4_bottom = 0x3ff;

		/* ROI5 */

		regs->swreg254.roi5_left = 0x3ff;
		regs->swreg255.roi5_right = 0x3ff;
		regs->swreg255.roi5_top = 0x3ff;
		regs->swreg255.roi5_bottom = 0x3ff;

		/* ROI6 */

		regs->swreg256.roi6_left = 0x3ff;
		regs->swreg256.roi6_right = 0x3ff;
		regs->swreg256.roi6_top = 0x3ff;
		regs->swreg257.roi6_bottom = 0x3ff;

		/* ROI7 */

		regs->swreg257.roi7_left = 0x3ff;
		regs->swreg258.roi7_right = 0x3ff;
		regs->swreg257.roi7_top = 0x3ff;
		regs->swreg258.roi7_bottom = 0x3ff;

		/* ROI8 */

		regs->swreg258.roi8_left = 0x3ff;
		regs->swreg259.roi8_right = 0x3ff;
		regs->swreg259.roi8_top = 0x3ff;
		regs->swreg259.roi8_bottom = 0x3ff;
	}
}
