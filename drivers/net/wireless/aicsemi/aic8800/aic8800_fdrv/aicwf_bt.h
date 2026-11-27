/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _AICWF_BT_H_
#define _AICWF_BT_H_

#include <linux/types.h>

struct rwnx_hw;

int aicwf_bt_init(struct rwnx_hw *rwnx_hw);
void aicwf_bt_deinit(struct rwnx_hw *rwnx_hw);
void aicwf_bt_recv(struct rwnx_hw *rwnx_hw, const u8 *data, u32 len);

#endif /* _AICWF_BT_H_ */
