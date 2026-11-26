// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E clock and reset controllers (COMM_GLB, FLASH_SYS, VPU_SYS,
 * PERIPH_SYS)
 *
 * Each subsystem has a "GLB" register block. Every state register R has a
 * SET and a CLR alias: writing a 1 there sets or clears that bit, so gates,
 * resets and mux fields can be changed without a read-modify-write. VPU_SYS
 * and PERIPH_SYS place them at set_base + 2 * R and set_base + 2 * R + 4,
 * FLASH_SYS at 0x4000 + R and 0x8000 + R, COMM_GLB right after R.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/axera,ax620e-comm-glb.h>
#include <dt-bindings/clock/axera,ax620e-cpu-sys.h>
#include <dt-bindings/clock/axera,ax620e-flash-sys.h>
#include <dt-bindings/clock/axera,ax620e-periph-sys.h>
#include <dt-bindings/clock/axera,ax620e-vpu-sys.h>
#include <dt-bindings/reset/axera,ax620e-comm-glb.h>
#include <dt-bindings/reset/axera,ax620e-flash-sys.h>
#include <dt-bindings/reset/axera,ax620e-periph-sys.h>
#include <dt-bindings/reset/axera,ax620e-vpu-sys.h>

#define AX_GLB_NO_PARENT		(-1)

struct ax620e_glb_mux {
	unsigned int id;
	const char *name;
	/* NULL for a source that is not known. */
	const char * const *parents;
	u8 num_parents;
	u16 reg;
	u8 shift;
	u8 width;
	/* clk_set_rate() picks the parent, as cpufreq needs for the CPU's */
	bool reparent;
};

/* Divides by field + 1; a new value takes effect on a pulse of update_bit. */
struct ax620e_glb_div {
	unsigned int id;
	const char *name;
	unsigned int parent;
	u16 reg;
	u8 shift;
	u8 width;
	u8 update_bit;
};

struct ax620e_glb_gate {
	unsigned int id;
	const char *name;
	/* A mux of this block, or AX_GLB_NO_PARENT to use parent_fw. */
	int parent;
	const char *parent_fw;
	u16 reg;
	u8 bit;
};

struct ax620e_glb_reset {
	u16 reg;
	u32 mask;
};

struct ax620e_glb_data {
	u16 set_base;
	u8 set_stride;
	u16 clr_offset;
	const struct ax620e_glb_mux *muxes;
	unsigned int num_muxes;
	const struct ax620e_glb_div *divs;
	unsigned int num_divs;
	const struct ax620e_glb_gate *gates;
	unsigned int num_gates;
	const struct ax620e_glb_reset *resets;
	unsigned int num_resets;
	unsigned int num_clks;
	/* Offset of the I2S routing register, 0 for none. */
	u16 audio_routing;
	/* No set and clear registers: muxes are read, changed and written. */
	bool rmw;
};

struct ax620e_glb {
	void __iomem *base;
	spinlock_t lock;
	const struct ax620e_glb_data *data;
	struct reset_controller_dev rcdev;
};

struct ax620e_glb_clk {
	struct clk_hw hw;
	struct ax620e_glb *glb;
	u16 reg;
	u8 shift;
	u8 width;
	u32 mask;
	u8 update_bit;
};

static void __iomem *ax620e_glb_set(struct ax620e_glb *glb, u16 reg)
{
	return glb->base + glb->data->set_base + glb->data->set_stride * reg;
}

static void __iomem *ax620e_glb_clr(struct ax620e_glb *glb, u16 reg)
{
	return ax620e_glb_set(glb, reg) + glb->data->clr_offset;
}

static struct ax620e_glb_clk *to_ax620e_glb_clk(struct clk_hw *hw)
{
	return container_of(hw, struct ax620e_glb_clk, hw);
}

static int ax620e_glb_gate_enable(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	writel(clk->mask, ax620e_glb_set(clk->glb, clk->reg));

	return 0;
}

static void ax620e_glb_gate_disable(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	writel(clk->mask, ax620e_glb_clr(clk->glb, clk->reg));
}

static int ax620e_glb_gate_is_enabled(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	return !!(readl(clk->glb->base + clk->reg) & clk->mask);
}

static const struct clk_ops ax620e_glb_gate_ops = {
	.enable = ax620e_glb_gate_enable,
	.disable = ax620e_glb_gate_disable,
	.is_enabled = ax620e_glb_gate_is_enabled,
};

static u8 ax620e_glb_mux_get_parent(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	return (readl(clk->glb->base + clk->reg) & clk->mask) >> clk->shift;
}

static int ax620e_glb_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);
	unsigned long flags;
	u32 val;

	/* One write: a clear first would pass through parent 0. */
	if (clk->glb->data->rmw) {
		spin_lock_irqsave(&clk->glb->lock, flags);
		val = readl(clk->glb->base + clk->reg) & ~clk->mask;
		writel(val | index << clk->shift, clk->glb->base + clk->reg);
		spin_unlock_irqrestore(&clk->glb->lock, flags);
		return 0;
	}

	writel(clk->mask, ax620e_glb_clr(clk->glb, clk->reg));
	writel(index << clk->shift, ax620e_glb_set(clk->glb, clk->reg));

	return 0;
}

static const struct clk_ops ax620e_glb_mux_ops = {
	.get_parent = ax620e_glb_mux_get_parent,
	.set_parent = ax620e_glb_mux_set_parent,
	.determine_rate = __clk_mux_determine_rate,
};

static unsigned long ax620e_glb_div_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);
	u32 val = (readl(clk->glb->base + clk->reg) & clk->mask) >> clk->shift;

	return divider_recalc_rate(hw, parent_rate, val, NULL, 0, clk->width);
}

static int ax620e_glb_div_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	return divider_determine_rate(hw, req, NULL, clk->width, 0);
}

static int ax620e_glb_div_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);
	int val;

	val = divider_get_val(rate, parent_rate, NULL, clk->width, 0);
	if (val < 0)
		return val;

	writel(clk->mask, ax620e_glb_clr(clk->glb, clk->reg));
	writel(val << clk->shift, ax620e_glb_set(clk->glb, clk->reg));
	writel(BIT(clk->update_bit), ax620e_glb_set(clk->glb, clk->reg));
	udelay(2);
	writel(BIT(clk->update_bit), ax620e_glb_clr(clk->glb, clk->reg));

	return 0;
}

static const struct clk_ops ax620e_glb_div_ops = {
	.recalc_rate = ax620e_glb_div_recalc_rate,
	.determine_rate = ax620e_glb_div_determine_rate,
	.set_rate = ax620e_glb_div_set_rate,
};

static struct ax620e_glb *to_ax620e_glb(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct ax620e_glb, rcdev);
}

static int ax620e_glb_reset_assert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct ax620e_glb *glb = to_ax620e_glb(rcdev);
	const struct ax620e_glb_reset *rst = &glb->data->resets[id];

	writel(rst->mask, ax620e_glb_set(glb, rst->reg));

	return 0;
}

static int ax620e_glb_reset_deassert(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct ax620e_glb *glb = to_ax620e_glb(rcdev);
	const struct ax620e_glb_reset *rst = &glb->data->resets[id];

	writel(rst->mask, ax620e_glb_clr(glb, rst->reg));

	return 0;
}

static int ax620e_glb_reset_reset(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	ax620e_glb_reset_assert(rcdev, id);
	udelay(1);
	return ax620e_glb_reset_deassert(rcdev, id);
}

static int ax620e_glb_reset_status(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct ax620e_glb *glb = to_ax620e_glb(rcdev);
	const struct ax620e_glb_reset *rst = &glb->data->resets[id];

	return !!(readl(glb->base + rst->reg) & rst->mask);
}

static const struct reset_control_ops ax620e_glb_reset_ops = {
	.assert = ax620e_glb_reset_assert,
	.deassert = ax620e_glb_reset_deassert,
	.reset = ax620e_glb_reset_reset,
	.status = ax620e_glb_reset_status,
};

static struct clk_hw *ax620e_glb_register(struct device *dev,
					  const struct ax620e_glb_clk *tmpl,
					  struct clk_init_data *init)
{
	struct ax620e_glb_clk *clk;
	int ret;

	clk = devm_kmemdup(dev, tmpl, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return ERR_PTR(-ENOMEM);
	clk->hw.init = init;

	ret = devm_clk_hw_register(dev, &clk->hw);

	return ret ? ERR_PTR(ret) : &clk->hw;
}

static int ax620e_glb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct ax620e_glb_data *data;
	struct clk_hw_onecell_data *clk_data;
	struct ax620e_glb *glb;
	unsigned int i, j;
	struct clk_hw *hw;
	u32 routing;
	int ret;

	data = device_get_match_data(dev);
	glb = devm_kzalloc(dev, sizeof(*glb), GFP_KERNEL);
	if (!glb)
		return -ENOMEM;
	glb->data = data;
	spin_lock_init(&glb->lock);

	glb->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(glb->base))
		return PTR_ERR(glb->base);

	if (data->audio_routing &&
	    !device_property_read_u32(dev, "axera,audio-routing", &routing))
		writel(routing, glb->base + data->audio_routing);

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, data->num_clks),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;
	clk_data->num = data->num_clks;
	for (i = 0; i < data->num_clks; i++)
		clk_data->hws[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < data->num_muxes; i++) {
		const struct ax620e_glb_mux *mux = &data->muxes[i];
		struct ax620e_glb_clk tmpl = {
			.glb = glb,
			.reg = mux->reg,
			.shift = mux->shift,
			.width = mux->width,
			.mask = GENMASK(mux->shift + mux->width - 1, mux->shift),
		};
		struct clk_parent_data *parents;
		struct clk_init_data init = { };

		parents = devm_kcalloc(dev, mux->num_parents, sizeof(*parents),
				       GFP_KERNEL);
		if (!parents)
			return -ENOMEM;
		for (j = 0; j < mux->num_parents; j++) {
			if (mux->parents[j])
				parents[j].fw_name = mux->parents[j];
			else
				parents[j].index = -1;
		}

		init.name = mux->name;
		init.ops = &ax620e_glb_mux_ops;
		init.parent_data = parents;
		init.num_parents = mux->num_parents;
		init.flags = mux->reparent ? 0 : CLK_SET_RATE_NO_REPARENT;
		hw = ax620e_glb_register(dev, &tmpl, &init);
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		if (WARN_ON(mux->id >= data->num_clks))
			return -EINVAL;
		clk_data->hws[mux->id] = hw;
	}

	for (i = 0; i < data->num_divs; i++) {
		const struct ax620e_glb_div *div = &data->divs[i];
		struct ax620e_glb_clk tmpl = {
			.glb = glb,
			.reg = div->reg,
			.shift = div->shift,
			.width = div->width,
			.mask = GENMASK(div->shift + div->width - 1, div->shift),
			.update_bit = div->update_bit,
		};
		struct clk_parent_data parent = { .hw = clk_data->hws[div->parent] };
		struct clk_init_data init = { };

		init.name = div->name;
		init.ops = &ax620e_glb_div_ops;
		init.parent_data = &parent;
		init.num_parents = 1;
		hw = ax620e_glb_register(dev, &tmpl, &init);
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		if (WARN_ON(div->id >= data->num_clks))
			return -EINVAL;
		clk_data->hws[div->id] = hw;
	}

	for (i = 0; i < data->num_gates; i++) {
		const struct ax620e_glb_gate *gate = &data->gates[i];
		struct ax620e_glb_clk tmpl = {
			.glb = glb,
			.reg = gate->reg,
			.mask = BIT(gate->bit),
		};
		struct clk_parent_data parent = { };
		struct clk_init_data init = { };

		if (gate->parent == AX_GLB_NO_PARENT)
			parent.fw_name = gate->parent_fw;
		else
			parent.hw = clk_data->hws[gate->parent];

		init.name = gate->name;
		init.ops = &ax620e_glb_gate_ops;
		init.parent_data = &parent;
		init.num_parents = 1;
		init.flags = CLK_SET_RATE_PARENT;
		hw = ax620e_glb_register(dev, &tmpl, &init);
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		if (WARN_ON(gate->id >= data->num_clks))
			return -EINVAL;
		clk_data->hws[gate->id] = hw;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
	if (ret)
		return ret;

	glb->rcdev.owner = THIS_MODULE;
	glb->rcdev.ops = &ax620e_glb_reset_ops;
	glb->rcdev.of_node = dev->of_node;
	glb->rcdev.nr_resets = data->num_resets;

	return devm_reset_controller_register(dev, &glb->rcdev);
}

static const char * const vpu_src_parents[] = {
	"cpll_208m", "cpll_312m", "epll_375m", "cpll_416m", "epll_500m",
	"npll_533m",
};

static const struct ax620e_glb_mux vpu_muxes[] = {
	{ AX620E_VPU_CLK_JENC_SEL, "vpu_jenc_sel", vpu_src_parents, 6, 0x0, 0, 3 },
	{ AX620E_VPU_CLK_VDEC_SEL, "vpu_vdec_sel", vpu_src_parents, 6, 0x0, 3, 3 },
	{ AX620E_VPU_CLK_GLB_SEL, "vpu_glb_sel", vpu_src_parents, 6, 0x0, 6, 3 },
};

static const struct ax620e_glb_gate vpu_gates[] = {
	{ AX620E_VPU_CLK_JENC, "vpu_jenc", AX620E_VPU_CLK_JENC_SEL, NULL, 0x8, 4 },
	{ AX620E_VPU_CLK_VDEC, "vpu_vdec", AX620E_VPU_CLK_VDEC_SEL, NULL, 0x8, 6 },
	{ AX620E_VPU_CLK_VENC, "vpu_venc", AX620E_VPU_CLK_GLB_SEL, NULL, 0x8, 7 },
};

static const struct ax620e_glb_reset vpu_resets[] = {
	[AX620E_VPU_RST_JENC] = { 0xc, BIT(4) },
	[AX620E_VPU_RST_VDEC] = { 0xc, BIT(6) },
	[AX620E_VPU_RST_VENC] = { 0xc, BIT(7) },
};

static const struct ax620e_glb_data ax620e_vpu_sys_data = {
	.set_base = 0x104,
	.set_stride = 2,
	.clr_offset = 4,
	.muxes = vpu_muxes,
	.num_muxes = ARRAY_SIZE(vpu_muxes),
	.gates = vpu_gates,
	.num_gates = ARRAY_SIZE(vpu_gates),
	.resets = vpu_resets,
	.num_resets = ARRAY_SIZE(vpu_resets),
	.num_clks = AX620E_VPU_CLK_VENC + 1,
};

/* GPIO debounce and watchdog sources. */
static const char * const periph_gpio_parents[] = { "rtc_32k", "cpll_24m" };
static const char * const periph_i2c_parents[] = {
	"cpll_24m", "epll_50m", "cpll_156m", "cpll_208m",
};

static const struct ax620e_glb_mux periph_muxes[] = {
	{ AX620E_PERIPH_CLK_GPIO_SEL, "periph_gpio_sel", periph_gpio_parents, 2, 0x0, 2, 1 },
	{ AX620E_PERIPH_CLK_I2C_SEL, "periph_i2c_sel", periph_i2c_parents, 4, 0x0, 3, 2 },
	{ AX620E_PERIPH_CLK_WDT0_SEL, "periph_wdt0_sel", periph_gpio_parents, 2, 0x0, 19, 1 },
	{ AX620E_PERIPH_CLK_WDT2_SEL, "periph_wdt2_sel", periph_gpio_parents, 2, 0x0, 20, 1 },
	{ AX620E_PERIPH_CLK_TIMER_SEL, "periph_timer_sel", periph_gpio_parents, 2, 0x0, 13, 1 },
};

#define PERIPH_GATE(_id, _name, _parent, _reg, _bit) \
	{ _id, _name, _parent, "pclk_top", _reg, _bit }

static const struct ax620e_glb_gate periph_gates[] = {
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO0, "gpio0", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 4),
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO1, "gpio1", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 5),
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO2, "gpio2", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 6),
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO3, "gpio3", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 7),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C0, "i2c0", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 8),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C1, "i2c1", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 9),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C2, "i2c2", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 10),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C3, "i2c3", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 11),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C4, "i2c4", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 12),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C5, "i2c5", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 13),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C6, "i2c6", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 14),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C7, "i2c7", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 15),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO0, "pclk_gpio0", AX_GLB_NO_PARENT, 0xc, 13),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO1, "pclk_gpio1", AX_GLB_NO_PARENT, 0xc, 14),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO2, "pclk_gpio2", AX_GLB_NO_PARENT, 0xc, 15),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO3, "pclk_gpio3", AX_GLB_NO_PARENT, 0xc, 16),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C0, "pclk_i2c0", AX_GLB_NO_PARENT, 0xc, 17),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C1, "pclk_i2c1", AX_GLB_NO_PARENT, 0xc, 18),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C2, "pclk_i2c2", AX_GLB_NO_PARENT, 0xc, 19),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C3, "pclk_i2c3", AX_GLB_NO_PARENT, 0xc, 20),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C4, "pclk_i2c4", AX_GLB_NO_PARENT, 0xc, 21),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C5, "pclk_i2c5", AX_GLB_NO_PARENT, 0xc, 22),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C6, "pclk_i2c6", AX_GLB_NO_PARENT, 0xc, 23),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C7, "pclk_i2c7", AX_GLB_NO_PARENT, 0xc, 24),
	PERIPH_GATE(AX620E_PERIPH_CLK_WDT0, "wdt0", AX620E_PERIPH_CLK_WDT0_SEL, 0x4, 14),
	PERIPH_GATE(AX620E_PERIPH_CLK_WDT2, "wdt2", AX620E_PERIPH_CLK_WDT2_SEL, 0x4, 15),
	PERIPH_GATE(AX620E_PERIPH_PCLK_WDT0, "pclk_wdt0", AX_GLB_NO_PARENT, 0x10, 19),
	PERIPH_GATE(AX620E_PERIPH_PCLK_WDT2, "pclk_wdt2", AX_GLB_NO_PARENT, 0x10, 20),
	{ AX620E_PERIPH_CLK_I2S_REF, "i2s_ref", AX_GLB_NO_PARENT, "hpll_12m288", 0x4, 3 },
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2S_S, "pclk_i2s_s", AX_GLB_NO_PARENT, 0xc, 28),
	PERIPH_GATE(AX620E_PERIPH_ACLK_DMA, "aclk_dma", AX_GLB_NO_PARENT, 0x8, 0),
	PERIPH_GATE(AX620E_PERIPH_PCLK_DMA, "pclk_dma", AX_GLB_NO_PARENT, 0xc, 11),
	/* The first boot stage sets the SPI master mux ([12:11]) to cpll_208m. */
	{ AX620E_PERIPH_CLK_SPI2, "spi2", AX_GLB_NO_PARENT, "cpll_208m", 0x4, 8 },
	PERIPH_GATE(AX620E_PERIPH_PCLK_SPI2, "pclk_spi2", AX_GLB_NO_PARENT, 0x10, 4),
	/* The DesignWare timers of the PWM blocks; each channel has a gate. */
	PERIPH_GATE(AX620E_PERIPH_CLK_TIMER, "periph_timer", AX620E_PERIPH_CLK_TIMER_SEL, 0x4, 9),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM00, "pwm00", AX620E_PERIPH_CLK_TIMER, 0x8, 19),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM01, "pwm01", AX620E_PERIPH_CLK_TIMER, 0x8, 20),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM02, "pwm02", AX620E_PERIPH_CLK_TIMER, 0x8, 21),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM03, "pwm03", AX620E_PERIPH_CLK_TIMER, 0x8, 22),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM10, "pwm10", AX620E_PERIPH_CLK_TIMER, 0x8, 23),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM11, "pwm11", AX620E_PERIPH_CLK_TIMER, 0x8, 24),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM12, "pwm12", AX620E_PERIPH_CLK_TIMER, 0x8, 25),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM13, "pwm13", AX620E_PERIPH_CLK_TIMER, 0x8, 26),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM20, "pwm20", AX620E_PERIPH_CLK_TIMER, 0x8, 27),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM21, "pwm21", AX620E_PERIPH_CLK_TIMER, 0x8, 28),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM22, "pwm22", AX620E_PERIPH_CLK_TIMER, 0x8, 29),
	PERIPH_GATE(AX620E_PERIPH_CLK_PWM23, "pwm23", AX620E_PERIPH_CLK_TIMER, 0x8, 30),
	PERIPH_GATE(AX620E_PERIPH_PCLK_PWM0, "pclk_pwm0", AX_GLB_NO_PARENT, 0xc, 31),
	PERIPH_GATE(AX620E_PERIPH_PCLK_PWM1, "pclk_pwm1", AX_GLB_NO_PARENT, 0x10, 0),
	PERIPH_GATE(AX620E_PERIPH_PCLK_PWM2, "pclk_pwm2", AX_GLB_NO_PARENT, 0x10, 1),
};

/* Each block has an APB reset and a core reset; they are used together. */
static const struct ax620e_glb_reset periph_resets[] = {
	[AX620E_PERIPH_RST_GPIO0] = { 0x18, BIT(10) | BIT(11) },
	[AX620E_PERIPH_RST_GPIO1] = { 0x18, BIT(12) | BIT(13) },
	[AX620E_PERIPH_RST_GPIO2] = { 0x18, BIT(14) | BIT(15) },
	[AX620E_PERIPH_RST_GPIO3] = { 0x18, BIT(16) | BIT(17) },
	[AX620E_PERIPH_RST_I2C0] = { 0x18, BIT(18) | BIT(19) },
	[AX620E_PERIPH_RST_I2C1] = { 0x18, BIT(20) | BIT(21) },
	[AX620E_PERIPH_RST_I2C2] = { 0x18, BIT(22) | BIT(23) },
	[AX620E_PERIPH_RST_I2C3] = { 0x18, BIT(24) | BIT(25) },
	[AX620E_PERIPH_RST_I2C4] = { 0x18, BIT(26) | BIT(27) },
	[AX620E_PERIPH_RST_I2C5] = { 0x18, BIT(28) | BIT(29) },
	[AX620E_PERIPH_RST_WDT0] = { 0x24, BIT(0) | BIT(1) },
	[AX620E_PERIPH_RST_WDT2] = { 0x24, BIT(2) | BIT(3) },
	[AX620E_PERIPH_RST_I2S_S_APB] = { 0x1c, BIT(8) },
	[AX620E_PERIPH_RST_I2S_S] = { 0x1c, BIT(9) },
	[AX620E_PERIPH_RST_DMA_AXI] = { 0x18, BIT(1) },
	[AX620E_PERIPH_RST_DMA_APB] = { 0x18, BIT(2) },
	[AX620E_PERIPH_RST_SPI2] = { 0x20, BIT(2) | BIT(3) },
	/* The block's reset and those of its four channels. */
	[AX620E_PERIPH_RST_PWM0] = { 0x1c, GENMASK(19, 15) },
	[AX620E_PERIPH_RST_PWM1] = { 0x1c, GENMASK(24, 20) },
	[AX620E_PERIPH_RST_PWM2] = { 0x1c, GENMASK(29, 25) },
};

static const struct ax620e_glb_data ax620e_periph_sys_data = {
	.set_base = 0xa8,
	.set_stride = 2,
	.clr_offset = 4,
	.muxes = periph_muxes,
	.num_muxes = ARRAY_SIZE(periph_muxes),
	.gates = periph_gates,
	.num_gates = ARRAY_SIZE(periph_gates),
	.resets = periph_resets,
	.num_resets = ARRAY_SIZE(periph_resets),
	.num_clks = AX620E_PERIPH_PCLK_PWM2 + 1,
	.audio_routing = 0x3c,
};

static const char * const flash_bus_parents[] = {
	"cpll_24m", "epll_100m", "cpll_156m", "cpll_208m", "epll_250m",
	"cpll_312m",
};

/* Only the 400 MHz NPLL output of the card clock sources is known. */
static const char * const flash_card_parents[] = {
	NULL, NULL, NULL, "npll_400m",
};

static const struct ax620e_glb_mux flash_muxes[] = {
	{ AX620E_FLASH_CLK_BUS_SEL, "flash_bus_sel", flash_bus_parents, 6, 0x0, 6, 3 },
	{ AX620E_FLASH_CLK_SD_CARD_SEL, "flash_sd_card_sel", flash_card_parents, 4, 0x0, 16, 2 },
	{ AX620E_FLASH_CLK_SDIO_CARD_SEL, "flash_sdio_card_sel", flash_card_parents, 4, 0x0, 18, 2 },
};

static const struct ax620e_glb_div flash_divs[] = {
	{ AX620E_FLASH_CLK_SD_CARD_DIV, "flash_sd_card_div", AX620E_FLASH_CLK_SD_CARD_SEL, 0xc, 20, 6, 26 },
	{ AX620E_FLASH_CLK_SDIO_CARD_DIV, "flash_sdio_card_div", AX620E_FLASH_CLK_SDIO_CARD_SEL, 0x10, 0, 6, 6 },
};

static const struct ax620e_glb_gate flash_gates[] = {
	{ AX620E_FLASH_CLK_SD_CARD, "flash_sd_card", AX620E_FLASH_CLK_SD_CARD_DIV, NULL, 0x4, 9 },
	{ AX620E_FLASH_CLK_SDIO_CARD, "flash_sdio_card", AX620E_FLASH_CLK_SDIO_CARD_DIV, NULL, 0x4, 10 },
	{ AX620E_FLASH_CLK_USB2_REF, "flash_usb2_ref", AX_GLB_NO_PARENT, "cpll_24m", 0x4, 12 },
	{ AX620E_FLASH_CLK_USB2_REF_ALT, "flash_usb2_ref_alt", AX_GLB_NO_PARENT, "cpll_24m", 0x4, 14 },
	{ AX620E_FLASH_ACLK_SD, "flash_aclk_sd", AX620E_FLASH_CLK_BUS_SEL, NULL, 0x8, 3 },
	{ AX620E_FLASH_ACLK_SDIO, "flash_aclk_sdio", AX620E_FLASH_CLK_BUS_SEL, NULL, 0x8, 4 },
	{ AX620E_FLASH_CLK_USB2_BUS, "flash_usb2_bus", AX620E_FLASH_CLK_BUS_SEL, NULL, 0x8, 5 },
	{ AX620E_FLASH_PCLK_SD, "flash_pclk_sd", AX_GLB_NO_PARENT, "pclk_top", 0x8, 17 },
	{ AX620E_FLASH_PCLK_SDIO, "flash_pclk_sdio", AX_GLB_NO_PARENT, "pclk_top", 0x8, 18 },
};

static const struct ax620e_glb_reset flash_resets[] = {
	[AX620E_FLASH_RST_SD_CARD] = { 0x14, BIT(15) },
	[AX620E_FLASH_RST_SD_AXI] = { 0x14, BIT(16) },
	[AX620E_FLASH_RST_SD_APB] = { 0x14, BIT(17) },
	[AX620E_FLASH_RST_SDIO_CARD] = { 0x14, BIT(18) },
	[AX620E_FLASH_RST_SDIO_AXI] = { 0x14, BIT(19) },
	[AX620E_FLASH_RST_SDIO_APB] = { 0x14, BIT(20) },
	[AX620E_FLASH_RST_USB2_PHY] = { 0x14, BIT(24) },
	[AX620E_FLASH_RST_USB2_VCC] = { 0x14, BIT(25) },
};

static const struct ax620e_glb_data ax620e_flash_sys_data = {
	.set_base = 0x4000,
	.set_stride = 1,
	.clr_offset = 0x4000,
	.muxes = flash_muxes,
	.num_muxes = ARRAY_SIZE(flash_muxes),
	.divs = flash_divs,
	.num_divs = ARRAY_SIZE(flash_divs),
	.gates = flash_gates,
	.num_gates = ARRAY_SIZE(flash_gates),
	.resets = flash_resets,
	.num_resets = ARRAY_SIZE(flash_resets),
	.num_clks = AX620E_FLASH_PCLK_SDIO + 1,
};

/*
 * COMM_GLB is also a syscon for the D-PHY power control of the video input;
 * the clocks and resets here are the ones no other driver writes.
 */
static const struct ax620e_glb_gate comm_gates[] = {
	{ AX620E_COMM_PCLK_THM, "comm_pclk_thm", AX_GLB_NO_PARENT, "pclk_top", 0x30, 10 },
	{ AX620E_COMM_CLK_THM, "comm_clk_thm", AX_GLB_NO_PARENT, "cpll_24m", 0x30, 14 },
};

static const struct ax620e_glb_reset comm_resets[] = {
	[AX620E_COMM_RST_THM_APB] = { 0x54, BIT(19) },
	[AX620E_COMM_RST_THM] = { 0x54, BIT(20) },
};

static const struct ax620e_glb_data ax620e_comm_glb_data = {
	.set_base = 0x4,
	.set_stride = 1,
	.clr_offset = 4,
	.gates = comm_gates,
	.num_gates = ARRAY_SIZE(comm_gates),
	.resets = comm_resets,
	.num_resets = ARRAY_SIZE(comm_resets),
	.num_clks = AX620E_COMM_CLK_THM + 1,
};

/* The CPUs' clock: fixed PLL outputs, which cpufreq switches between. */
static const char * const cpu_parents[] = {
	"cpll_24m", "cpll_208m", "epll_500m", "npll_800m", "cpupll_1200m",
};

static const struct ax620e_glb_mux cpu_muxes[] = {
	{ AX620E_CPU_CLK_SEL, "cpu_sel", cpu_parents, 5, 0x0, 2, 3, true },
};

static const struct ax620e_glb_data ax620e_cpu_sys_data = {
	.muxes = cpu_muxes,
	.num_muxes = ARRAY_SIZE(cpu_muxes),
	.num_clks = AX620E_CPU_CLK_SEL + 1,
	.rmw = true,
};

static const struct of_device_id ax620e_glb_of_match[] = {
	{ .compatible = "axera,ax620e-comm-glb", .data = &ax620e_comm_glb_data },
	{ .compatible = "axera,ax620e-cpu-sys", .data = &ax620e_cpu_sys_data },
	{ .compatible = "axera,ax620e-flash-sys", .data = &ax620e_flash_sys_data },
	{ .compatible = "axera,ax620e-periph-sys", .data = &ax620e_periph_sys_data },
	{ .compatible = "axera,ax620e-vpu-sys", .data = &ax620e_vpu_sys_data },
	{ }
};
MODULE_DEVICE_TABLE(of, ax620e_glb_of_match);

static struct platform_driver ax620e_glb_driver = {
	.probe = ax620e_glb_probe,
	.driver = {
		.name = "ax620e-glb",
		.of_match_table = ax620e_glb_of_match,
	},
};
module_platform_driver(ax620e_glb_driver);

MODULE_DESCRIPTION("Axera AX620E clock and reset controllers");
MODULE_LICENSE("GPL");
