// SPDX-License-Identifier: GPL-2.0
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright 2020 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <dt-bindings/clock/ad9545.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/rational.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define AD9545_PRODUCT_ID_LOW		0x0004
#define AD9545_PRODUCT_ID_HIGH		0x0005

#define AD9545_IO_UPDATE		0x000F
#define AD9545_CHIP_ID			0x0121
#define AD9545_SYS_CLK_FB_DIV		0x0200
#define AD9545_SYS_CLK_INPUT		0x0201
#define AD9545_SYS_CLK_REF_FREQ		0x0202
#define AD9545_SYS_STABILITY_T		0x0207
#define AD9545_REF_A_CTRL		0x0300
#define AD9545_REF_B_CTRL		0x0304
#define AD9545_REF_A_RDIV		0x0400
#define AD9545_REF_A_PERIOD		0x0404
#define AD9545_DPLL0_FTW		0x1000
#define AD9545_DRIVER_0A_CONF		0x10D7
#define AD9545_SYNC_CTRL0		0x10DB
#define AD9545_APLL0_M_DIV		0x1081
#define AD9545_Q0A_DIV			0x1100
#define AD9545_DPLL0_EN			0x1220
#define AD9545_DPLL0_SOURCE		0x1201
#define AD9545_DPLL0_LOOP_BW		0x1204
#define AD9545_DPLL0_N_DIV		0x120C
#define AD9545_DPLL0_FRAC		0x1210
#define AD9545_DPLL0_MOD		0x1213
#define AD9545_DRIVER_1A_CONF		0x14D7
#define AD9545_Q1A_DIV			0x1500
#define AD9545_CALIB_CLK		0x2000
#define AD9545_POWER_DOWN_REF		0x2001
#define AD9545_PWR_CALIB_CH0		0x2100
#define AD9545_CTRL_CH0			0x2101
#define AD9545_DPLL0_MODE		0x2105
#define AD9545_NCO0_FREQ		0x2805
#define AD9545_PLL_STATUS		0x3001
#define AD9545_PLL0_STATUS		0x3100

#define AD9545_REF_CTRL_DIF_MSK			GENMASK(3,2)
#define AD9545_REF_CTRL_REFA_MSK		GENMASK(5,4)
#define AD9545_REF_CTRL_REFAA_MSK		GENMASK(7,6)

#define AD9545_UPDATE_REGS			0x1

#define AD9545_SYNC_CTRLX(x)			(AD9545_SYNC_CTRL0 + ((x) * 0x400))
#define AD9545_REF_X_RDIV(x)			(AD9545_REF_A_RDIV + ((x) * 0x20))
#define AD9545_REF_X_PERIOD(x)			(AD9545_REF_A_PERIOD + ((x) * 0x20))

#define AD9545_APLLX_M_DIV(x)			(AD9545_APLL0_M_DIV + ((x) * 0x400))

#define AD9545_Q0_DIV(x)			(AD9545_Q0A_DIV + ((x) * 0x9))
#define AD9545_Q1_DIV(x)			(AD9545_Q1A_DIV + ((x) * 0x9))
#define AD545_QX_DIV(x)				((x) > 5 ? AD9545_Q1_DIV(x - 5) : AD9545_Q0_DIV(x))

#define AD9545_DPLLX_FTW(x)			(AD9545_DPLL0_FTW + ((x) * 0x400))
#define AD9545_DPLLX_EN(x)			(AD9545_DPLL0_EN + ((x) * 0x400))
#define AD9545_DPLLX_SOURCE(x)			(AD9545_DPLL0_SOURCE + ((x) * 0x400))
#define AD9545_DPLLX_LOOP_BW(x)			(AD9545_DPLL0_LOOP_BW + ((x) * 0x400))
#define AD9545_DPLLX_N_DIV(x)			(AD9545_DPLL0_N_DIV + ((x) * 0x400))
#define AD9545_DPLLX_FRAC_DIV(x)		(AD9545_DPLL0_FRAC + ((x) * 0x400))
#define AD9545_DPLLX_MOD_DIV(x)			(AD9545_DPLL0_MOD + ((x) * 0x400))
#define AD9545_DPLLX_MODE(x)			(AD9545_DPLL0_MODE + ((x) * 0x400))

#define AD9545_PWR_CALIB_CHX(x)			(AD9545_PWR_CALIB_CH0 + ((x) * 0x100))
#define AD9545_PLLX_STATUS(x)			(AD9545_PLL0_STATUS + ((x) * 0x100))

#define AD9545_PROFILE_SEL_MODE_MSK		GENMASK(3,2)
#define AD9545_PROFILE_SEL_MODE(x)		FIELD_PREP(AD9545_PROFILE_SEL_MODE_MSK, x)

#define AD9545_NCOX_FREQ(x)			(AD9545_NCO0_FREQ + ((x) * 0x40))

/* AD9545_PWR_CALIB_CHX bitfields */
#define AD9545_PWR_DOWN_CH			BIT(0)
#define AD9545_CALIB_APLL			BIT(1)

/* AD9545_SYNC_CTRLX bitfields */
#define AD9545_SYNC_CTRL_DPLL_REF_MSK		BIT(2)
#define AD9545_SYNC_CTRL_MODE_MSK		GENMASK(1, 0)

#define AD9545_SYS_PLL_STABLE_MSK		GENMASK(1, 0)
#define AD9545_SYS_PLL_STABLE(x)		(((x) & AD9545_SYS_PLL_STABLE_MSK) == 0x3)

#define AD9545_APLL_LOCKED(x)			((x) & BIT(3))

#define AD9545_SYS_CLK_STABILITY_MS	50

#define AD9545_R_DIV_MAX		0x40000000
#define AD9545_IN_MAX_TDC_FREQ_HZ	200000

#define AD9545_APLL_M_DIV_MIN		1
#define AD9545_APLL_M_DIV_MAX		255

#define AD9545_DPLL_MAX_N		1073741823
#define AD9545_DPLL_MAX_FRAC		116777215
#define AD9545_DPLL_MAX_MOD		116777215

#define AD9545_NCO_MAX_FREQ		65535

static const unsigned short AD9545_regs[][2] = {
//	{0x0200, 0x18},	/* System Clock Registers Details */
//	{0x0201, 0x09},
//	{0x0203, 0x00},
//	{0x0204, 0xB0},
//	{0x0205, 0x71},
//	{0x0206, 0x0B},
	{0x0280, 0x05}, /* SYSCLK Compensation Register */
	{0x0282, 0x05},
	{0x0285, 0xF4},
	{0x0286, 0x01}, /* todo in the future ^^^ */
//	{0x0300, 0x10}, /* Reference General A Registers */
//	{0x0304, 0x20}, /* Reference General B Register */
//	{0x0400, 0x31}, /* Reference Input A Registers */
//	{0x0405, 0xE8}, /* REFA nominal period [15:8] */
//	{0x0406, 0x64},
//	{0x0407, 0xA7},
//	{0x0408, 0xB3},
//	{0x0409, 0xB6},
//	{0x040A, 0xE0},
//	{0x040B, 0x0D}, /* REFA nominal period [59:56] */
//	{0x040C, 0x40}, /* REFA offset limit [7:0] */
//	{0x040D, 0x42}, /* REFA offset limit [15:8] */
//	{0x040E, 0x0F}, /* REFA offset limit [23:16] */
//	{0x0420, 0x31}, /* Reference Input AA Registers */
//	{0x0425, 0xE8},
//	{0x0426, 0x76},
//	{0x0427, 0x48},
//	{0x0428, 0x17},
//	{0x0446, 0x64}, /* Reference Input B Registers */
//	{0x0447, 0xA7},
//	{0x0448, 0xB3},
//	{0x0449, 0xB6},
//	{0x044A, 0xE0},
//	{0x044B, 0x0D},
//	{0x0460, 0x00}, /* Reference Input BB Registers */
//	{0x0465, 0x00},
//	{0x0466, 0x64},
//	{0x0467, 0xA7},
//	{0x0468, 0xB3},
//	{0x0469, 0xB6},
//	{0x046A, 0xE0},
//	{0x046B, 0x0D},
	{0x0800, 0x58}, /* Source Profile 0 A Registers */ /* Phase lock threshold */
	{0x0801, 0x1B}, /* Phase lock threshold */
	{0x0803, 0xC8}, /* Phase lock fill rate */
	{0x0804, 0xC8}, /* Phase lock drain rate */
	{0x0805, 0x88}, /* Frequency lock threshold */
	{0x0806, 0x13}, /* Frequency lock threshold */
	{0x0808, 0xC8}, /* Frequency lock fill rate */
	{0x0809, 0xC8}, /* Frequency lock drain rate */
	{0x0820, 0x88}, /* Source Profile 1 AA Registers */
	{0x0821, 0x13},
	{0x0823, 0xC8},
	{0x0824, 0xC8},
	{0x0825, 0x88},
	{0x0826, 0x13},
	{0x0828, 0xC8},
	{0x0829, 0xC8},
	{0x0883, 0x64}, /* Source Profile 4 NCO 0 Registers */
	{0x0884, 0x64},
	{0x0888, 0x64},
	{0x0889, 0x64},
	{0x08A3, 0xC8}, /* Source Profile 5 NCO 1 Registers */
	{0x08A4, 0xC8},
	{0x08A8, 0xC8},
	{0x08A9, 0xC8},
	{0x1000, 0x55}, /* DPLL Channel 0 Registers */
	{0x1001, 0x55},
	{0x1002, 0x55},
	{0x1003, 0x55},
	{0x1004, 0x55},
	{0x10C4, 0x53}, /* Distribution General 0 Registers */
	{0x10C5, 0x07},
	{0x10C6, 0x90},
	{0x10C7, 0x2F},
	{0x10C8, 0x50},
	{0x10C9, 0x09},
	{0x10CA, 0x90},
	{0x10CB, 0x2F},
	{0x10CC, 0x50},
	{0x10CD, 0x09},
//	{0x10D7, 0x02}, /* OUT0A */
//	{0x10D8, 0x0C}, /* OUT0B */
//	{0x10DB, 0x05}, /* OUT0C */
//	{0x1100, 0x0A}, /* Distribution Divider Q0A Registers */ /* Q0A div ratio: 0xA */
	{0x1201, 0x04}, /* DPLL Translation Profile 0.0 Registers prev" [0x05]*/
	{0x1204, 0x50},
	{0x1205, 0xC3},
	{0x1208, 0xFF},
	{0x1209, 0xFF},
	{0x120A, 0x52},
	{0x120B, 0x07},
	{0x120C, 0xFF},
	{0x120D, 0x05},
	{0x1210, 0x01},
	{0x1213, 0x02},
	{0x1216, 0x01},
	{0x1217, 0x76},
	{0x1400, 0x8E}, /* DPLL Channel 1 Registers */
	{0x1401, 0xE3},
	{0x1402, 0x38},
	{0x1403, 0x8E},
	{0x1404, 0xE8},
	{0x1405, 0x21},
	{0x1481, 0x0C}, /* APLL Channel 1 Registers */
	{0x14C2, 0x90}, /* Distribution General 1 Register */
	{0x14C3, 0x2F},
	{0x14C4, 0x50},
	{0x14C5, 0x09},
//	{0x14D7, 0x02}, /* OUT1A */
//	{0x14D8, 0x02}, /* OUT1B */
	{0x14DB, 0x05},
	{0x1500, 0x0C}, /* Distribution Divider 1 A Registers */
	{0x1508, 0x07},
	{0x1512, 0x06}, /* Distribution Divider 1 B Registers */
	{0x151A, 0x07},
	{0x1601, 0x02}, /* DPLL Translation Profile 1.0 Registers [0x03]*/
	{0x1604, 0x50},
	{0x1605, 0xC3},
	{0x1608, 0x8F},
	{0x1609, 0x2F},
	{0x160A, 0x50},
	{0x160B, 0x09},
	{0x160C, 0x1F},
	{0x160D, 0x5F},
	{0x160E, 0xA0},
	{0x160F, 0x12},
	{0x2001, 0x03}, /* IRQ Map DPLL0 Clear Registers - power down references prev: [0x0F] */
	{0x2101, 0x08}, /* Operational Control Channel 0 Registers */
	{0x2103, 0x10},
	{0x2104, 0x10},
	{0x2201, 0x08}, /* Operational Control Channel 1 Registers */
//	{0x2804, 0x80}, /* Auxiliary NCO 0 Registers */
//	{0x2809, 0x80},
	{0x3001, 0x03}, /* Status Readback Register */
	{0x3002, 0x05},
	{0x3003, 0xB8},
	{0x3004, 0x11},
	{0x3005, 0x0B},
	{0x3006, 0x0B},
	{0x3007, 0x0B},
	{0x3008, 0x0B},
	{0x3100, 0x28}, /* STATUS_READBACK_PLL_0 Registers */
	{0x3101, 0x01},
	{0x3200, 0x28}, /* STATUS_READBACK_PLL_1 Register */
	{0x3201, 0x01},
	{0x3A00, 0x3B}, /* TDC_AUXILIARY_READ Register */
	{0x3A01, 0x44},
	{0x3A02, 0x44},
	{0x3A03, 0xEA},
	{0x3A04, 0xAB},
	{0x3A05, 0x4F},
	{0x3A0A, 0xFF},
	{0x3A0B, 0xFF},
	{0x3A0C, 0xFF},
	{0x3A0D, 0xFF},
	{0x3A0E, 0xFF},
	{0x3A0F, 0x01},
	{0x3A1E, 0xFF},
	{0x3A2A, 0xFF},
};

static const unsigned int ad9545_apll_rate_ranges_hz[2][2] = {
	{2400000000, 3200000000}, {3200000000, 4000000000}
};

static const unsigned short ad9545_vco_calibration_op[][2] = {
	{AD9545_CALIB_CLK, 0},
	{AD9545_IO_UPDATE, AD9545_UPDATE_REGS},
	{AD9545_CALIB_CLK, BIT(2)},
	{AD9545_IO_UPDATE, AD9545_UPDATE_REGS},
};

static const u8 ad9545_tdc_source_mapping[] = {
	0, 1, 2, 3, 8, 9,
};

static const u32 ad9545_hyst_scales_bp[] = {
	0, 3125, 6250, 12500, 25000, 50000, 75000, 87500
};

static const u32 ad9545_out_source_ua[] = {
	7500, 12500, 15000
};

static const char *ad9545_ref_clk_names[] = {
	"Ref-A", "Ref-AA", "Ref-B", "Ref-BB",
};

static const char *ad9545_in_clk_names[] = {
	"Ref-A-Div", "Ref-AA-Div", "Ref-B-Div", "Ref-BB-Div",
};

static const char *ad9545_out_clk_names[] = {
	"Q0A-div", "Q0AA-div", "Q0B-div", "Q0BB-div", "Q0C-div", "Q0CC-div", "Q1A-div", "Q1AA-div",
	"Q1B-div", "Q1BB-div",
};

static const char *ad9545_pll_clk_names[] = {
	"PLL0", "PLL1",
};

static const char * ad9545_aux_nco_clk_names[] = {
	"AUX_NCO0", "AUX_NCO1",
};

enum ad9545_ref_mode {
	AD9545_SINGLE_ENDED = 0,
	AD9545_DIFFERENTIAL,
};

enum ad9545_single_ended_config {
	AD9545_AC_COUPLED_IF = 0,
	AD9545_DC_COUPLED_1V2,
	AD9545_DC_COUPLED_1V8,
	AD9545_IN_PULL_UP,
};

enum ad9545_diferential_config {
	AD9545_AC_COUPLED = 0,
	AD9545_DC_COUPLED,
	AD9545_DC_COUPLED_LVDS,
};

enum ad9545_output_mode {
	AD9545_SINGLE_DIV_DIF = 0,
	AD9545_SINGLE_DIV,
	AD9545_DUAL_DIV,
};

struct ad9545_out_clk {
	struct ad9545_state		*st;
	bool				output_used;
	bool				source_current;
	enum ad9545_output_mode 	output_mode;
	u32 				source_ua;
	struct clk_hw			hw;
	unsigned int			address;
};

struct ad9545_ppl_clk {
	struct ad9545_state		*st;
	bool				pll_used;
	unsigned int			address;
	unsigned int			loop_bw;
	struct clk_hw			hw;
	u8				tdc_source;
};

struct ad9545_ref_in_clk {
	struct clk_hw			hw;
	struct ad9545_state		*st;
	u32				r_div_ratio;
	bool				ref_used;
	u32				d_tol_ppb;
	u8				monitor_hyst_scale;
	u32				valid_t_ms;
	struct clk			*parent_clk;
	unsigned int			address;
	enum ad9545_ref_mode		mode;
	union {
		enum ad9545_single_ended_config		s_conf;
		enum ad9545_diferential_config		d_conf;
	};
};

struct ad9545_aux_nco_clk {
	struct clk_hw			hw;
	bool				nco_used;
	struct ad9545_state		*st;
	unsigned int			address;
	u32				nco_freq_hz;
};

struct ad9545_sys_clk {
	bool				sys_clk_freq_doubler;
	bool				sys_clk_crystal;
	u32				ref_freq_hz;
	u32				sys_freq_hz;
};

struct ad9545_state {
	struct i2c_client		*client;
	struct regmap 			*regmap;
	struct ad9545_sys_clk		sys_clk;
	struct ad9545_ppl_clk		pll_clks[2];
	struct ad9545_ref_in_clk	ref_in_clks[4];
	struct ad9545_out_clk		out_clks[10];
	struct ad9545_aux_nco_clk	aux_nco_clks[2];
};

#define to_ref_in_clk(_hw) 	container_of(_hw, struct ad9545_ref_in_clk, hw)
#define to_pll_clk(_hw) 	container_of(_hw, struct ad9545_ppl_clk, hw)
#define to_out_clk(_hw) 	container_of(_hw, struct ad9545_out_clk, hw)
#define to_nco_clk(_hw) 	container_of(_hw, struct ad9545_aux_nco_clk, hw)

static const struct regmap_range ad9545_reg_ranges[] = {
	{
		.range_min = 0,
		.range_max = 0x3A3B,
	},
};

static const struct regmap_access_table ad9545_reg_access_t = {
	.yes_ranges = ad9545_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad9545_reg_ranges),
};

static const struct regmap_config ad9545_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
    .max_register = 0x3A3B,
    .rd_table = &ad9545_reg_access_t,
};

static int ad9545_parse_dt_inputs(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	struct clk *clk;
	bool prop_found;
	int ref_ind;
	u32 val;
	int ret;
	int i;

	fwnode = dev_fwnode(&st->client->dev);

	prop_found = false;
	fwnode_for_each_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,r-divider-ratio"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &ref_ind);
		if (ret < 0)
			return ret;

		if (ref_ind > 3)
			return -EINVAL;

		st->ref_in_clks[ref_ind].ref_used = true;
		st->ref_in_clks[ref_ind].address = ref_ind;
		st->ref_in_clks[ref_ind].st = st;

		if ((prop_found = fwnode_property_present(child, "adi,single-ended-mode"))) {
			st->ref_in_clks[ref_ind].mode = AD9545_SINGLE_ENDED;
			ret = fwnode_property_read_u32(child, "adi,single-ended-mode", &val);
			if (ret < 0)
				return ret;

			st->ref_in_clks[ref_ind].s_conf = val;
		} else if ((prop_found = fwnode_property_present(child, "adi,differential-mode"))) {
			st->ref_in_clks[ref_ind].mode = AD9545_DIFFERENTIAL;
			ret = fwnode_property_read_u32(child, "adi,differential-mode", &val);
			if (ret < 0)
				return ret;

			st->ref_in_clks[ref_ind].d_conf = val;
		}

		if (!prop_found) {
			dev_err(&st->client->dev, "No mode specified for input reference.\n");
			return -EINVAL;
		}

		ret = fwnode_property_read_u32(child, "adi,r-divider-ratio", &val);
		if (!ret)
			st->ref_in_clks[ref_ind].r_div_ratio = val;

		ret = fwnode_property_read_u32(child, "adi,ref-dtol-pbb", &val);
		if (ret < 0)
			return ret;
		st->ref_in_clks[ref_ind].d_tol_ppb = val;

		ret = fwnode_property_read_u32(child, "adi,ref-monitor-hysteresis-pbb", &val);
		if (ret < 0)
			return ret;

		for (i = 0; i < ARRAY_SIZE(ad9545_hyst_scales_bp); i++) {
			if (ad9545_hyst_scales_bp[i] == val) {
				st->ref_in_clks[ref_ind].monitor_hyst_scale = i;
				break;
			}
		}
		if (i == ARRAY_SIZE(ad9545_hyst_scales_bp))
			return -EINVAL;

		ret = fwnode_property_read_u32(child, "adi,ref-validation-timer-ms", &val);
		if (ret < 0)
			return ret;

		st->ref_in_clks[ref_ind].valid_t_ms = val;

		clk = devm_clk_get(&st->client->dev, ad9545_ref_clk_names[ref_ind]);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		st->ref_in_clks[ref_ind].parent_clk = clk;
	}

	return 0;
}

static int ad9545_parse_dt_plls(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	u32 val;
	u32 addr;
	int ret;

	fwnode = dev_fwnode(&st->client->dev);

	prop_found = false;
	fwnode_for_each_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,pll-source"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &addr);
		if (ret < 0)
			return ret;

		if (addr > 1)
			return -EINVAL;

		st->pll_clks[addr].pll_used = true;
		st->pll_clks[addr].address = addr;

		ret = fwnode_property_read_u32(child, "adi,pll-source", &val);
		if (ret < 0)
			return ret;

		if (val > 5)
			return -EINVAL;

		st->pll_clks[addr].tdc_source = val;

		ret = fwnode_property_read_u32(child, "adi,pll-loop-bandwidth-hz", &val);
		if (ret < 0)
			return ret;

		st->pll_clks[addr].loop_bw = val;
	}

	return 0;
}

static int ad9545_parse_dt_outputs(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	int ref_ind;
	u32 val;
	int ret;

	fwnode = dev_fwnode(&st->client->dev);

	prop_found = false;
	fwnode_for_each_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,output-mode"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &ref_ind);
		if (ret < 0)
			return ret;

		if (ref_ind > 9)
			return -EINVAL;

		st->out_clks[ref_ind].output_used = true;
		st->out_clks[ref_ind].address = ref_ind;

		if (fwnode_property_present(child, "adi,current-source"))
			st->out_clks[ref_ind].source_current = true;

		ret = fwnode_property_read_u32(child, "adi,current-source-microamp", &val);
		if (ret < 0)
			return ret;

		st->out_clks[ref_ind].source_ua = val;

		ret = fwnode_property_read_u32(child, "adi,output-mode", &val);
		if (ret < 0)
			return ret;

		st->out_clks[ref_ind].output_mode = val;
	}

	return 0;
}

static int ad9545_parse_dt_ncos(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	u32 val;
	u32 addr;
	int ret;

	fwnode = dev_fwnode(&st->client->dev);

	prop_found = false;
	fwnode_for_each_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,nco-freq-hz"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &addr);
		if (ret < 0)
			return ret;

		if (addr > 1)
			return -EINVAL;

		st->aux_nco_clks[addr].nco_used = true;
		st->aux_nco_clks[addr].address = addr;
		st->aux_nco_clks[addr].st = st;

		ret = fwnode_property_read_u32(child, "adi,nco-freq-hz", &val);
		if (ret < 0)
			return ret;

		st->aux_nco_clks[addr].nco_freq_hz = val;
	}

	return 0;
}

static int ad9545_parse_dt(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = dev_fwnode(&st->client->dev);

	ret = fwnode_property_read_u32(fwnode, "adi,ref-frequency-hz", &st->sys_clk.ref_freq_hz);
	if (ret < 0)
		return ret;

	st->sys_clk.sys_clk_crystal = fwnode_property_present(fwnode, "adi,ref-crystal");

	ret = ad9545_parse_dt_inputs(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_plls(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_outputs(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_ncos(st);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9545_check_id(struct ad9545_state *st)
{
	u32 chip_id;
	u32 val;
	int ret;

	ret = regmap_read(st->regmap, AD9545_PRODUCT_ID_LOW, &val);
	if (ret < 0)
		return ret;

	chip_id = val;
	ret = regmap_read(st->regmap, AD9545_PRODUCT_ID_HIGH, &val);
	if (ret < 0)
		return ret;

	chip_id += val << 8;
	if (chip_id != AD9545_CHIP_ID) {
		dev_err(&st->client->dev, "Unrecognized CHIP_ID 0x%X\n", chip_id);
		return -ENODEV;
	}

	return 0;
}

static int ad9545_io_update(struct ad9545_state *st)
{
	return regmap_write(st->regmap, AD9545_IO_UPDATE, AD9545_UPDATE_REGS);
}

static int ad9545_sys_clk_setup(struct ad9545_state *st)
{
	u8 div_ratio;
	u8 buf[5];
	u32 fosc;
	int ret;
	u8 val;
	u32 fs;
	int i;

	/*
	 * System frequency must be between 2250 MHz and 2415 MHz.
	 * fs = fosc * K / j
	 * K - feedback divider ratio [4, 255]
	 * j = 1/2 if frequency doubler is enabled
	 */
	fosc = DIV_ROUND_UP(st->sys_clk.ref_freq_hz, 1000000);

	if (st->sys_clk.sys_clk_freq_doubler)
		fosc *= 2;

	div_ratio = 0;
	for (i = 4; i < 256; i++) {
		fs = i * fosc;

		if (fs > 2250 && fs < 2415) {
			div_ratio = i;
			break;
		}
	}

	st->sys_clk.sys_freq_hz = st->sys_clk.ref_freq_hz * div_ratio;
	if (st->sys_clk.sys_clk_freq_doubler)
		st->sys_clk.sys_freq_hz *= 2;

	pr_info("DEBUG: sys clock frequenc hz=%u\n", st->sys_clk.sys_freq_hz);

	if (!div_ratio) {
		dev_err(&st->client->dev, "No feedback divider ratio for sys clk PLL found.\n");
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, AD9545_SYS_CLK_FB_DIV, div_ratio);
	if (ret < 0)
		return ret;

	/* enable crystal maintaining amplifier */
	val = 0;
	if (st->sys_clk.sys_clk_crystal)
		val |= BIT(3);

	if (st->sys_clk.sys_clk_freq_doubler)
		val |= BIT(0);

	ret = regmap_write(st->regmap, AD9545_SYS_CLK_INPUT, val);
	if (ret < 0)
		return ret;

	/* write reference frequency provided at XOA, XOB */
	for (i = 0; i < 5; i++)
		buf[i] = (st->sys_clk.ref_freq_hz >> (i * 8)) & 0xFF;

	ret = regmap_bulk_write(st->regmap, AD9545_SYS_CLK_REF_FREQ, buf, 5);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, AD9545_SYS_STABILITY_T, AD9545_SYS_CLK_STABILITY_MS);
}

static int ad9545_get_q_div(struct ad9545_state *st, int addr, u32 *q_div)
{
	__le32 regval;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD545_QX_DIV(addr), &regval, 4);
	if (ret < 0)
		return ret;

	*q_div = le32_to_cpu(regval);

	return 0;
}

static int ad9545_set_q_div(struct ad9545_state *st, int addr, u32 q_div)
{
	__le32 regval;
	int ret;

	regval = cpu_to_le32(q_div);
	ret = regmap_bulk_write(st->regmap, AD545_QX_DIV(addr), &regval, 4);
	if (ret < 0)
		return ret;

	return ad9545_io_update(st);
}

static unsigned long ad95452_out_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	u32 div;
	int ret;

	ret = ad9545_get_q_div(clk->st, clk->address, &div);
	if (ret < 0) {
		dev_err(&clk->st->client->dev, "Could not read Q div value.");
		return 0;
	}

	return DIV_ROUND_CLOSEST(parent_rate, div);
}

static long ad9545_out_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	unsigned int div;

	div = DIV_ROUND_CLOSEST(*parent_rate, rate);
	if (!div)
		return *parent_rate;

	return DIV_ROUND_CLOSEST(*parent_rate, div);
}

static int ad9545_out_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	u32 div = DIV_ROUND_CLOSEST(parent_rate, rate);

	return ad9545_set_q_div(clk->st, clk->address, div);
}

static const struct clk_ops ad9545_out_clk_ops = {
	.recalc_rate = ad95452_out_clk_recalc_rate,
	.round_rate = ad9545_out_clk_round_rate,
	.set_rate = ad9545_out_clk_set_rate,
};

static int ad9545_outputs_setup(struct ad9545_state *st)
{
	struct clk_init_data init[10];
	int out_i;
	u16 addr;
	int ret;
	u8 reg;
	int i;
	int j;

	/* set autosync mode */
	for (i = 0; i < 2; i++) {
		reg = FIELD_PREP(AD9545_SYNC_CTRL_MODE_MSK, 1);
		ret = regmap_write(st->regmap, AD9545_SYNC_CTRLX(i), reg);
		if (ret < 0)
			return ret;
	}

	/* configure current sources */
	for (i = 0; i < 5; i++) {
		st->out_clks[i * 2].st = st;
		st->out_clks[i * 2 + 1].st = st;

		if (st->out_clks[i * 2].output_used)
			out_i = i * 2;
		else if (st->out_clks[i * 2].output_used)
			out_i = i * 2 + 1;
		else
			continue;

		reg = 0;
		if (st->out_clks[out_i].source_current)
			reg = 1;

		for (j = 0; j < ARRAY_SIZE(ad9545_out_source_ua); j++)
			if (ad9545_out_source_ua[j] == st->out_clks[out_i].source_ua)
				reg |= FIELD_PREP(GENMASK(2,1), i);

		reg |= FIELD_PREP(GENMASK(4, 3), st->out_clks[out_i].output_mode);

		if (i < 3)
			addr = AD9545_DRIVER_0A_CONF + i;
		else
			addr = AD9545_DRIVER_1A_CONF + (i - 3);

		ret = regmap_write(st->regmap, addr, reg);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < 10; i++) {
		if (!st->out_clks[i].output_used)
			continue;

		init[i].name = ad9545_out_clk_names[i];
		init[i].ops = &ad9545_out_clk_ops;

		if (i > 5) {
			init[i].parent_names = &ad9545_pll_clk_names[1];
		} else {
			init[i].parent_names = &ad9545_pll_clk_names[0];
		}

		init[i].num_parents = 1;

		/* at startup set a higher than 0 div ratio */
		ret = ad9545_set_q_div(st, i, 1);
		if (ret < 0)
			return ret;

		st->out_clks[i].hw.init = &init[i];
		ret = devm_clk_hw_register(&st->client->dev, &st->out_clks[i].hw);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9545_set_r_div(struct ad9545_state *st, u32 div, int addr)
{
	int ret;
	u8 reg;
	int i;

	if (div > AD9545_R_DIV_MAX)
		return -EINVAL;

	/* r-div ratios are mapped from 0 onward */
	div -= 1;
	for (i = 0; i < 4; i++) {
		reg = (div >> (i * 8)) && 0xFF;

		ret = regmap_write(st->regmap, AD9545_REF_X_RDIV(addr) + i, reg);
		if (ret < 0)
			return ret;
	}

	return ad9545_io_update(st);
}

static int ad9545_get_r_div(struct ad9545_state *st, int addr, u32 *r_div)
{
	int ret;
	u32 div;
	u32 reg;
	int i;

	div = 0;
	for (i = 0; i < 4; i++) {
		ret = regmap_read(st->regmap, AD9545_REF_X_RDIV(addr) + i, &reg);
		if (ret < 0)
			return ret;

		div += (reg << (i * 8));
	}

	/* r-div ratios are mapped from 0 onward */
	*r_div = ++div;

	return 0;
}

static unsigned long ad95452_in_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_ref_in_clk *clk = to_ref_in_clk(hw);
	u32 div;
	int ret;

	ret = ad9545_get_r_div(clk->st, clk->address, &div);
	if (ret < 0) {
		dev_err(&clk->st->client->dev, "Could not read r div value.");
		return 1;
	}

	return DIV_ROUND_CLOSEST(parent_rate, div);
}

static long ad9545_in_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	unsigned int div;

	div = DIV_ROUND_CLOSEST(*parent_rate, rate);
	div = clamp_t(unsigned int, div, 1, AD9545_R_DIV_MAX);

	return DIV_ROUND_CLOSEST(*parent_rate, div);
}

static int ad9545_in_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_ref_in_clk *clk = to_ref_in_clk(hw);
	u32 div = DIV_ROUND_UP(parent_rate, rate);

	return ad9545_set_r_div(clk->st, div, clk->address);
}

static const struct clk_ops ad9545_in_clk_ops = {
	.recalc_rate = ad95452_in_clk_recalc_rate,
	.round_rate = ad9545_in_clk_round_rate,
	.set_rate = ad9545_in_clk_set_rate,
};

static int ad9545_input_refs_setup(struct ad9545_state *st)
{
	struct clk_init_data init[4];
	u64 period_es;
	int ret;
	u32 val;
	u8 reg;
	int i;
	int j;

	/* configure input references */
	for (i = 0; i < 4; i += 2) {
		if (st->ref_in_clks[i].mode == AD9545_DIFFERENTIAL) {
			reg = BIT(0);
			reg |= FIELD_PREP(AD9545_REF_CTRL_DIF_MSK, st->ref_in_clks[i].d_conf);
		} else {
			reg = 0;
			reg |= FIELD_PREP(AD9545_REF_CTRL_REFA_MSK, st->ref_in_clks[i].s_conf);
			reg |= FIELD_PREP(AD9545_REF_CTRL_REFAA_MSK, st->ref_in_clks[i + 1].s_conf);
		}

		ret = regmap_write(st->regmap, AD9545_REF_A_CTRL + i * 2, reg);
		if (ret < 0)
			return ret;
	}

	/* configure refs r dividers */
	for (i = 0; i < 4; i++) {
		ret = ad9545_set_r_div(st, st->ref_in_clks[i].r_div_ratio, i);
		if (ret < 0)
			return ret;
	}

	/* specify nominal period on each ref */
	for (i = 0; i < 4; i++) {
		if (!st->ref_in_clks[i].ref_used) {
			/* write nominal period in attoseconds */
			period_es = 1000000000000000000;
			val = clk_get_rate(st->ref_in_clks[i].parent_clk);

			DIV_ROUND_UP_ULL(period_es, val);

			for (j = 0; j < 8; j++) {
				reg = (period_es >> (j * 8)) && 0xFF;

				ret = regmap_write(st->regmap, AD9545_REF_X_PERIOD(i) + j, reg);
				if (ret < 0)
					return ret;
			}
		}
	}

	for (i = 0; i < 4; i++) {
		if (st->ref_in_clks[i].ref_used) {
			init[i].name = ad9545_in_clk_names[i];
			init[i].ops = &ad9545_in_clk_ops;
			init[i].parent_names = &ad9545_ref_clk_names[i];
			init[i].num_parents = 1;

			st->ref_in_clks[i].hw.init = &init[i];
			ret = devm_clk_hw_register(&st->client->dev, &st->ref_in_clks[i].hw);
			if (ret < 0)
				return ret;
		}
	}

	/* disable unused references */
	reg = 0;
	for (i = 0; i < 4; i++) {
		if (!st->ref_in_clks[i].ref_used)
			reg |= (1 << i);
	}

	ret = regmap_write(st->regmap, AD9545_POWER_DOWN_REF, reg);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9545_set_freerun_freq(struct ad9545_ppl_clk *clk, u32 freq)
{
	__le64 regval;
	u64 ftw = 1;

	/*
	 * In case of unlock event the DPLL will go in open-loop mode and output
	 * the freq given by the freerun tuning word.
	 * DPLLx Freerun TW = 2 48 × (f NCO /f S )
	*/

	ftw <<= 48;
	ftw = DIV_ROUND_CLOSEST(ftw, clk->st->sys_clk.sys_freq_hz);
	ftw *= freq;
	regval = cpu_to_le64(ftw);

	return regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_FTW(clk->address), &regval, 6);
}

static u64 ad9545_calc_pll_params(struct ad9545_ppl_clk *clk, unsigned long rate,
				  unsigned long parent_rate, u32 *m, u32 *n,
				  unsigned long *frac, unsigned long *mod)
{
	u32 min_dpll_n_div;
	u64 output_rate;
	u32 dpll_n_div;
	u32 fnco_max;
	u32 m_div;
	u64 den;
	u64 num;

	/*
	 * DPLL uses a NCO limited by the system clock: fnco < fs / 4
	 * When requested freq exceeds this limit use APLL to multiply freq.
	 */
	fnco_max = clk->st->sys_clk.sys_freq_hz / 4;
	m_div = DIV_ROUND_UP(rate, fnco_max);

	if (m_div > AD9545_APLL_M_DIV_MAX)
		m_div = AD9545_APLL_M_DIV_MAX;

	/*
	 * If N + FRAC / MOD = rate / (m_div * parent_rate)
	 * and N = [rate / (m_div * past_rate)]:
	 * We get: FRAC/MOD = (rate / (m_div * parent_rate)) - N
	 */
	dpll_n_div = rate / (parent_rate * m_div);

	/*
	 * APLL has to be able to satisfy output freq bounds
	 * thus output of DPLL has a lower bound
	 */
	min_dpll_n_div = DIV_ROUND_UP(ad9545_apll_rate_ranges_hz[clk->address][0],
				      AD9545_APLL_M_DIV_MAX * parent_rate);
	dpll_n_div = clamp_t(u32, dpll_n_div, min_dpll_n_div, AD9545_DPLL_MAX_N);

	den = rate - (dpll_n_div * m_div * parent_rate);
	num = m_div * parent_rate;

	rational_best_approximation(num, den, AD9545_DPLL_MAX_FRAC, AD9545_DPLL_MAX_MOD, frac, mod);
	*m = m_div;
	*n = dpll_n_div;

	output_rate = mul_u64_u32_div(*frac * parent_rate, m_div, *mod);
	output_rate += parent_rate * dpll_n_div * m_div;

	return output_rate;
}

static unsigned long ad95452_pll_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_ppl_clk *clk = to_pll_clk(hw);
	unsigned long output_rate;
	__le32 regval;
	u32 frac;
	u32 mod;
	int ret;
	u32 m;
	u32 n;

	ret = regmap_bulk_read(clk->st->regmap, AD9545_DPLLX_N_DIV(clk->address), &regval, 4);
	if (ret < 0)
		return ret;
	n = le32_to_cpu(regval);

	m = 0;
	ret = regmap_read(clk->st->regmap, AD9545_APLLX_M_DIV(clk->address), &m);
	if (ret < 0)
		return ret;
	if (!m)
		m = 1;

	regval = 0;
	ret = regmap_bulk_read(clk->st->regmap, AD9545_DPLLX_FRAC_DIV(clk->address), &regval, 3);
	if (ret < 0)
		return ret;
	frac = le32_to_cpu(regval);

	regval = 0;
	ret = regmap_bulk_read(clk->st->regmap, AD9545_DPLLX_MOD_DIV(clk->address), &regval, 3);
	if (ret < 0)
		return ret;
	mod = le32_to_cpu(regval);
	mod += 1;

	/* Output rate of APLL = parent_rate * (N + (Frac / Mod)) * M */
	output_rate = mul_u64_u32_div(frac * parent_rate, m, mod);
	output_rate += parent_rate * n * m;

	pr_info("DEBUG: recalc rate: %lu, frac: %u, mod: %u, m=%u, n=%u\n", output_rate, frac, mod, m, n);

	return output_rate;
}

static long ad9545_pll_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	struct ad9545_ppl_clk *clk = to_pll_clk(hw);
	unsigned long frac;
	unsigned long mod;
	unsigned long freq;
	u32 m;
	u32 n;

	freq = ad9545_calc_pll_params(clk, rate, *parent_rate, &m, &n, &frac, &mod);
	pr_info("DEBUG: round rate: %lu, frac: %u, mod: %u, m=%u, n=%u\n", freq, frac, mod, m, n);
	return freq;
}

static int ad9545_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_ppl_clk *clk = to_pll_clk(hw);
	unsigned long out_rate;
	unsigned long frac;
	unsigned long mod;
	__le32 regval;
	int ret;
	u32 m;
	u32 n;

	out_rate = ad9545_calc_pll_params(clk, rate, parent_rate, &m, &n, &frac, &mod);
	if (out_rate != rate)
		return -EINVAL;

	regval = cpu_to_le32(n);
	ret = regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_N_DIV(clk->address), &regval, 4);
	if (ret < 0)
		return ret;

	ret = regmap_write(clk->st->regmap, AD9545_APLLX_M_DIV(clk->address), m);
	if (ret < 0)
		return ret;

	regval = cpu_to_le32(frac);
	ret = regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_FRAC_DIV(clk->address), &regval, 3);
	if (ret < 0)
		return ret;

	regval = cpu_to_le32(mod - 1);
	ret = regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_MOD_DIV(clk->address), &regval, 3);
	if (ret < 0)
		return ret;

	pr_info("DEBUG: set rate: %lu, frac: %u, mod: %u, m=%u, n=%u\n", out_rate, frac, mod, m, n);
	return ad9545_set_freerun_freq(clk, rate);
}

static const struct clk_ops ad9545_pll_clk_ops = {
	.recalc_rate = ad95452_pll_clk_recalc_rate,
	.round_rate = ad9545_pll_clk_round_rate,
	.set_rate = ad9545_pll_set_rate,
};

static int ad9545_TODO_init_100hz_freq(struct ad9545_ppl_clk *pll)
{
	unsigned long parent_rate;
	u64 mid;
	u64 rate;
	int ret;
	u8 m;

	/* set DPLL in free run freq */
	/* force free-run mode */
	ret = regmap_write(pll->st->regmap, AD9545_DPLLX_MODE(pll->address), 1);
	if (ret < 0)
		return ret;

	/* TODO: set a debug init sequence should be replaced by a simple set rate !*/
	ret = ad9545_set_freerun_freq(pll, 245760000);
	if (ret < 0)
		return ret;

	/* set APLL M div*/
	mid = ad9545_apll_rate_ranges_hz[pll->address][0];
	mid += ad9545_apll_rate_ranges_hz[pll->address][1];
	mid /= 2;
	m = mid / 245760000;
	pr_info("DEBUG: Setting m=%d", m);
	ret = regmap_write(pll->st->regmap, AD9545_APLLX_M_DIV(pll->address), m);
	if (ret < 0)
		return ret;

	/* set OUT 0 A q-div */
	ret = ad9545_set_q_div(pll->st, pll->address * 7, mid / 44000);
	if (ret < 0)
		return ret;

	parent_rate = 245760000;
	rate = ad95452_pll_clk_recalc_rate(&pll->hw, parent_rate);

	return 0;
}

static int ad9545_TODO_set_fixed_rate(struct ad9545_ppl_clk *pll) {
	__le32 regval;
	int ret;
	u8 m;

	regval = cpu_to_le32(1000000);
	ret = regmap_bulk_write(pll->st->regmap, AD9545_DPLLX_N_DIV(pll->address), &regval, 4);
	if (ret < 0)
		return ret;

	if (pll->address == 0) {
		m = 14;
	} else {
		m = 18;
	}
	ret = regmap_write(pll->st->regmap, AD9545_APLLX_M_DIV(pll->address), m);
	if (ret < 0)
		return ret;

	/* set OUT 0 A q-div */
	ret = ad9545_set_q_div(pll->st, pll->address * 7, 300000);
	if (ret < 0)
		return ret;
}

static int ad9545_plls_setup(struct ad9545_state *st)
{
	struct clk_init_data init[2];
	struct ad9545_ppl_clk *pll;
	struct clk * clk;
	__le32 regval;
	u8 reg;
	int ret;
	int i;

	for (i = 0; i < 2; i++) {
		pll = &st->pll_clks[i];
		if (!pll->pll_used)
			continue;

		/* enable pll profile */
		ret = regmap_write(st->regmap, AD9545_DPLLX_EN(i), 1);
		if (ret < 0)
			return ret;

		/* set TDC source */
		reg = ad9545_tdc_source_mapping[pll->tdc_source];
		ret = regmap_write(st->regmap, AD9545_DPLLX_SOURCE(i), reg);
		if (ret < 0)
			return ret;

		reg = AD9545_PROFILE_SEL_MODE(3);
		ret = regmap_write(st->regmap, AD9545_DPLLX_MODE(i), reg);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(pll->loop_bw);
		ret = regmap_bulk_write(st->regmap, AD9545_DPLLX_LOOP_BW(i), &regval, 4);
		if (ret < 0)
			return ret;

		pll->st = st;
		pll->address = i;

		init[i].name = ad9545_pll_clk_names[i];
		init[i].ops = &ad9545_pll_clk_ops;
		if (pll->tdc_source > 3) {
			init[i].parent_names = &ad9545_aux_nco_clk_names[pll->tdc_source - 4];
		} else {
			init[i].parent_names = &ad9545_in_clk_names[pll->tdc_source];
		}
		init[i].num_parents = 1;

		pll->hw.init = &init[i];
		ret = devm_clk_hw_register(&st->client->dev, &pll->hw);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9545_get_nco_freq(struct ad9545_state *st, int addr, u32 *freq)
{
	__le16 regval;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD9545_NCOX_FREQ(addr), &regval, 2);
	if (ret < 0)
		return ret;

	*freq = le16_to_cpu(regval);
	pr_info("DEBUG: ad9545: fnco freq: %d\n", *freq);
	return 0;
}

static int ad9545_set_nco_freq(struct ad9545_state *st, int addr, u32 freq)
{
	__le32 regval;
	int ret;

	regval = cpu_to_le32(freq);
	ret = regmap_bulk_write(st->regmap, AD9545_NCOX_FREQ(addr), &regval, 2);
	if (ret < 0)
		return ret;

	return ad9545_io_update(st);
}

static unsigned long ad95452_nco_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_aux_nco_clk *clk = to_nco_clk(hw);
	u32 rate;
	int ret;

	ret = ad9545_get_nco_freq(clk->st, clk->address, &rate);
	if (ret < 0) {
		dev_err(&clk->st->client->dev, "Could not read NCO freq.");
		return 0;
	}

	return rate;
}

static long ad9545_nco_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	return clamp_t(u16, rate, 1, AD9545_NCO_MAX_FREQ);
}

static int ad9545_nco_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_aux_nco_clk *clk = to_nco_clk(hw);

	return ad9545_set_nco_freq(clk->st, clk->address, rate);
}

static const struct clk_ops ad9545_nco_clk_ops = {
	.recalc_rate = ad95452_nco_clk_recalc_rate,
	.round_rate = ad9545_nco_clk_round_rate,
	.set_rate = ad9545_nco_clk_set_rate,
};

static int ad9545_aux_ncos_setup(struct ad9545_state *st)
{
	struct clk_init_data init[2];
	struct ad9545_aux_nco_clk *nco;
	int ret;
	int i;

	for (i = 0; i < 2; i++) {
		nco = &st->aux_nco_clks[i];
		if (!nco->nco_used)
			continue;

		ret = ad9545_set_nco_freq(st, nco->address, nco->nco_freq_hz);
		if (ret < 0)
			return ret;

		init[i].name = ad9545_aux_nco_clk_names[i];
		init[i].ops = &ad9545_nco_clk_ops;

		nco->hw.init = &init[i];
		ret = devm_clk_hw_register(&st->client->dev, &nco->hw);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9545_calib_system_clock(struct ad9545_state *st)
{
	int ret;
	u32 reg;
	int i;
	int j;

	for (i = 0; i < 2; i++) {
		for (j = 0; j < ARRAY_SIZE(ad9545_vco_calibration_op); j++) {
			ret = regmap_write(st->regmap, ad9545_vco_calibration_op[j][0],
					   ad9545_vco_calibration_op[j][1]);
			if (ret < 0)
				return ret;
		}

		/* wait for sys pll to lock and become stable */
		msleep(50 + AD9545_SYS_CLK_STABILITY_MS);

		ret = regmap_read(st->regmap, AD9545_PLL_STATUS, &reg);
		if (ret < 0)
			return ret;

		if (AD9545_SYS_PLL_STABLE(reg)) {
			ret = regmap_write(st->regmap, AD9545_CALIB_CLK, 0);
			if ( ret < 0)
				return ret;

			return ad9545_io_update(st);
		}
	}

	pr_info("DEBUG: ad9545: status pll: %d\n", reg);
	dev_err(&st->client->dev, "System PLL unlocked.\n");
	return -EIO;
}

static int ad9545_calib_aplls(struct ad9545_state *st)
{
	int cal_count;
	int ret;
	u32 reg;
	int i;

	for (i = 0; i < 2; i++) {

		if (!st->pll_clks[i].pll_used)
			continue;

		/* APLL VCO callibration operation */
		cal_count = 0;
		while(cal_count < 2) {
			ret = regmap_write(st->regmap, AD9545_PWR_CALIB_CHX(i), 0);
			if (ret < 0)
				return ret;

			ret = ad9545_io_update(st);
			if (ret < 0)
				return ret;

			ret = regmap_write(st->regmap, AD9545_PWR_CALIB_CHX(i), AD9545_CALIB_APLL);
			if (ret < 0)
				return ret;

			ret = ad9545_io_update(st);
			if (ret < 0)
				return ret;

			cal_count += 1;
			mdelay(100);

			ret = regmap_read(st->regmap, AD9545_PLLX_STATUS(i), &reg);
			if (ret < 0)
				return ret;

			if (AD9545_APLL_LOCKED(reg)) {
				pr_info("ad9545: PLL locked: status apll: %x\n", reg);
				ret = regmap_write(st->regmap, AD9545_PWR_CALIB_CHX(i), 0);
				if (ret < 0)
					return ret;

				ret = ad9545_io_update(st);
				if (ret < 0)
					return ret;

				cal_count = 2;
				break;
			} else {
				pr_info("DEBUG: ad9545: status apll: %x\n", reg);
				dev_err(&st->client->dev, "APLL unlocked.\n");
			}
		}
	}

	return 0;
}

static int ad9545_setup(struct ad9545_state *st)
{
	int ret;
	u32 val;

	ret = ad9545_sys_clk_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_input_refs_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_aux_ncos_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_plls_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_outputs_setup(st);
	if (ret < 0)
		return ret;

	/* TODO remove this after finishing set_rates: */
	ad9545_TODO_init_100hz_freq(&st->pll_clks[0]);
	ad9545_TODO_init_100hz_freq(&st->pll_clks[1]);

	ret = ad9545_io_update(st);
	if (ret < 0)
		return ret;

	ret = ad9545_calib_system_clock(st);
	if (ret < 0)
		return ret;

	ret = ad9545_calib_aplls(st);
	if (ret < 0)
		return ret;

	ret = ad9545_io_update(st);
	if (ret < 0)
		return ret;

	/* check locks */
	/* TODO: err on unlocked pll */
	regmap_read(st->regmap, 0x3001, &val);
	pr_info("DEBUG: ad9545: PLL status reg: %x", val);

	/* check locks */
	/* TODO: err on unlocked pll */
	regmap_read(st->regmap, 0x3100, &val);
	pr_info("DEBUG: ad9545: DPLL0 lock status reg: %x", val);

	/* check locks */
	/* TODO: err on unlocked pll */
	regmap_read(st->regmap, 0x3200, &val);
	pr_info("DEBUG: ad9545: DPLL1 lock status reg: %x", val);

	return 0;
}

static int ad9545_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ad9545_state *st;
	int ret;

	pr_info("DEBUG: AD9545 driver entered probe.\n");

	st = devm_kzalloc(&client->dev, sizeof(struct ad9545_state), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;
	st->regmap = devm_regmap_init_i2c(client, &ad9545_regmap_config);
	if (IS_ERR(st->regmap)) {
		dev_err(&client->dev, "devm_regmap_init_i2c failed!\n");
		return PTR_ERR(st->regmap);
	}

	ret = ad9545_check_id(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt(st);
	if (ret < 0)
		return ret;

	ret = ad9545_setup(st);
	if (ret < 0)
		return ret;

	pr_info("DEBUG: AD9545 driver probed.\n");

	return 0;
}

static const struct of_device_id ad9545_of_match[] = {
	{ .compatible = "adi,ad9545" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9545_of_match);

static const struct i2c_device_id ad9545_id[] = {
	{"adi,ad9545", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad9545_id);

static struct i2c_driver ad9545_driver = {
	.driver = {
		.name	= "ad9545",
		.of_match_table = ad9545_of_match,
	},
	.probe		= ad9545_probe,
	.id_table	= ad9545_id,
};
module_i2c_driver(ad9545_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9545");
MODULE_LICENSE("GPL v2");
