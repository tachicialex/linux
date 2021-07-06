// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/**
 *  Driver for Analog Devices Industrial Ethernet T1L PHYs
 *
 * Copyright 2020 Analog Devices Inc.
 */
#include <linux/kernel.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/property.h>

#define PHY_ID_ADIN1100				0x0283bc81

static const int phy_10_features_array[] = {
	ETHTOOL_LINK_MODE_10baseT1L_Full_BIT,
	ETHTOOL_LINK_MODE_2400mv_BIT,
	ETHTOOL_LINK_MODE_1000mv_BIT,
};

#define ADIN_B10L_PCS_CNTRL			0x08e6
#define   ADIN_PCS_CNTRL_B10L_LB_PCS_EN		BIT(14)

#define ADIN_B10L_PMA_CNTRL			0x08f6
#define   ADIN_PMA_CNTRL_B10L_LB_PMA_LOC_EN	BIT(0)

#define ADIN_B10L_PMA_STAT			0x08f7
#define   ADIN_PMA_STAT_B10L_LB_PMA_LOC_ABLE	BIT(13)
#define   ADIN_PMA_STAT_B10L_TX_LVL_HI_ABLE	BIT(12)

#define ADIN_AN_CONTROL				0x0200
#define   ADIN_AN_RESTART			BIT(9)
#define   ADIN_AN_EN				BIT(12)

#define ADIN_AN_STATUS				0x0201
#define ADIN_AN_ADV_ABILITY_L			0x0202
#define   ADIN_AN_ADV_FORCE_MS			BIT(12)

#define ADIN_AN_ADV_ABILITY_M			0x0203
#define   ADIN_AN_ADV_MST			BIT(4)

#define ADIN_AN_ADV_ABILITY_H			0x0204U
#define   ADIN_AN_ADV_B10L_TX_LVL_HI_ABL	BIT(13)
#define   ADIN_AN_ADV_B10L_TX_LVL_HI_REQ	BIT(12)

#define ADIN_AN_LP_ADV_ABILITY_L		0x0205

#define ADIN_AN_LP_ADV_ABILITY_M		0x0206
#define   ADIN_AN_LP_ADV_B10L			BIT(14)
#define   ADIN_AN_LP_ADV_B1000			BIT(7)
#define   ADIN_AN_LP_ADV_B10S_FD		BIT(6)
#define   ADIN_AN_LP_ADV_B100			BIT(5)
#define   ADIN_AN_LP_ADV_MST			BIT(4)

#define ADIN_AN_LP_ADV_ABILITY_H		0x0207
#define   ADIN_AN_LP_ADV_B10L_EEE		BIT(14)
#define   ADIN_AN_LP_ADV_B10L_TX_LVL_HI_ABL	BIT(13)
#define   ADIN_AN_LP_ADV_B10L_TX_LVL_HI_REQ	BIT(12)
#define   ADIN_AN_LP_ADV_B10S_HD		BIT(11)

#define ADIN_FC_EN				0x8001

#define ADIN_MSE_VAL				0x830B

#define ADIN_CRSM_SFT_RST			0x8810
#define   ADIN_CRSM_SFT_RST_EN			BIT(0)

#define ADIN_CRSM_SFT_PD_CNTRL			0x8812
#define   ADIN_CRSM_SFT_PD_CNTRL_EN		BIT(0)

#define ADIN_CRSM_STAT				0x8818
#define   ADIN_CRSM_SFT_PD_RDY			BIT(1)
#define   ADIN_CRSM_SYS_RDY			BIT(0)

#define AN_PHY_INST_STATUS			0x8030
#define   ADIN_IS_CFG_SLV			BIT(2)
#define   ADIN_IS_CFG_MST			BIT(3)

#define ADIN_MAC_IF_LOOPBACK			0x803d
#define   ADIN_MAC_IF_LOOPBACK_EN		BIT(0)
#define   ADIN_MAC_IF_REMOTE_LOOPBACK_EN	BIT(2)

#define ADIN_SQI_MAX	7

struct adin_hw_stat {
	const char *string;
	u16 reg1;
	u16 reg2;
};

static const struct adin_hw_stat adin_hw_stats[] = {
	{ "total_frames_error_count",		0x8008 },
	{ "total_frames_count",			0x8009, 0x800A }, /* hi, lo */
	{ "length_error_frames_count",		0x800B },
	{ "alignment_error_frames_count",	0x800C },
	{ "symbol_error_count",			0x800D },
	{ "oversized_frames_count",		0x800E },
	{ "undersized_frames_count",		0x800F },
	{ "odd_nibble_frames_count",		0x8010 },
	{ "odd_preamble_packet_count",		0x8011 },
	{ "false_carrier_events_count",		0x8013 },
};

struct adin_mse_sqi_range {
	u16 start;
	u16 end;
};

static const struct adin_mse_sqi_range adin_mse_sqi_map[] = {
	{ 0x0A74, 0xFFFF },
	{ 0x084E, 0x0A74 },
	{ 0x0698, 0x084E },
	{ 0x053D, 0x0698 },
	{ 0x0429, 0x053D },
	{ 0x034E, 0x0429 },
	{ 0x02A0, 0x034E },
	{ 0x0000, 0x02A0 },
};

/**
 * struct adin_priv - ADIN PHY driver private data
 * tx_level_24v			set if the PHY supports 2.4V TX levels (10BASE-T1L)
 * stats:			statistic counters for the PHY
 */
struct adin_priv {
	u64			stats[ARRAY_SIZE(adin_hw_stats)];
	unsigned int		tx_level_24v:1;
};

static void adin_mii_adv_m_to_ethtool_adv_t(unsigned long *advertising, u32 adv)
{
	if (adv & ADIN_AN_LP_ADV_B10L)
		linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT1L_Full_BIT, advertising);
	if (adv & ADIN_AN_LP_ADV_B100)
		linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT1_Full_BIT, advertising);
}

static void adin_mii_adv_h_to_ethtool_adv_t(unsigned long *advertising, u32 adv)
{
	if (adv & ADIN_AN_LP_ADV_B10S_HD)
		linkmode_set_bit(ETHTOOL_LINK_MODE_10baseT1L_Half_BIT, advertising);
	if (adv & ADIN_AN_LP_ADV_B10L_TX_LVL_HI_ABL)
		linkmode_set_bit(ETHTOOL_LINK_MODE_2400mv_BIT, advertising);
	if (!(adv & ADIN_AN_LP_ADV_B10L_TX_LVL_HI_REQ))
		linkmode_set_bit(ETHTOOL_LINK_MODE_1000mv_BIT, advertising);
}

static int adin_read_lpa(struct phy_device *phydev)
{
	int val;

	linkmode_zero(phydev->lp_advertising);

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_STATUS);
	if (val < 0)
		return val;

	if (!(val & MDIO_AN_STAT1_COMPLETE)) {
		phydev->pause = 0;
		phydev->asym_pause = 0;

		return 0;
	}

	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			 phydev->lp_advertising);

	/* Read the link partner's base page advertisement */
	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_L);
	if (val < 0)
		return val;

	phydev->pause = val & LPA_PAUSE_CAP ? 1 : 0;
	phydev->asym_pause = val & LPA_PAUSE_ASYM ? 1 : 0;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_M);
	if (val < 0)
		return val;

	adin_mii_adv_m_to_ethtool_adv_t(phydev->lp_advertising, val);

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_H);
	if (val < 0)
		return val;

	adin_mii_adv_h_to_ethtool_adv_t(phydev->lp_advertising, val);

	return 0;
}

static int adin_read_status(struct phy_device *phydev)
{
	int ret;
	int cfg;

	ret = genphy_c45_read_link(phydev);
	if (ret)
		return ret;

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->pause = 0;
	phydev->asym_pause = 0;
	phydev->master_slave_get = MASTER_SLAVE_CFG_UNKNOWN;
	phydev->master_slave_state = MASTER_SLAVE_STATE_UNKNOWN;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		ret = adin_read_lpa(phydev);
		if (ret)
			return ret;

		phy_resolve_aneg_linkmode(phydev);
	} else {
		/* Only one mode & duplex supported */
		linkmode_zero(phydev->lp_advertising);
		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_FULL;
	}

	ret = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_L);
	if (ret < 0)
		return ret;

	cfg = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_M);
	if (cfg < 0)
		return cfg;

	if (ret & ADIN_AN_ADV_FORCE_MS) {
		if (cfg & ADIN_AN_ADV_MST)
			phydev->master_slave_get = MASTER_SLAVE_CFG_MASTER_FORCE;
		else
			phydev->master_slave_get = MASTER_SLAVE_CFG_SLAVE_FORCE;
	} else {
		if (cfg & ADIN_AN_ADV_MST)
			phydev->master_slave_get = MASTER_SLAVE_CFG_MASTER_PREFERRED;
		else
			phydev->master_slave_get = MASTER_SLAVE_CFG_SLAVE_PREFERRED;
	}

	ret = phy_read_mmd(phydev, MDIO_MMD_AN, AN_PHY_INST_STATUS);
	if (ret < 0)
		return ret;

	if (ret & ADIN_IS_CFG_SLV)
		phydev->master_slave_state = MASTER_SLAVE_STATE_SLAVE;

	if (ret & ADIN_IS_CFG_MST)
		phydev->master_slave_state = MASTER_SLAVE_STATE_MASTER;

	return 0;
}

static int adin_config_aneg(struct phy_device *phydev)
{
	struct adin_priv *priv = phydev->priv;
	int ret;

	/* No sense to continue if auto-neg is disabled,
	 * only one link-mode supported.
	 */
	if (phydev->autoneg == AUTONEG_DISABLE)
		return 0;

	switch (phydev->master_slave_set) {
	case MASTER_SLAVE_CFG_MASTER_FORCE:
	case MASTER_SLAVE_CFG_SLAVE_FORCE:
		ret = phy_set_bits_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_L,
				       ADIN_AN_ADV_FORCE_MS);
		if (ret < 0)
			return ret;
		break;
	case MASTER_SLAVE_CFG_MASTER_PREFERRED:
	case MASTER_SLAVE_CFG_SLAVE_PREFERRED:
		ret = phy_clear_bits_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_L,
					 ADIN_AN_ADV_FORCE_MS);
		break;
	default:
		break;
	}

	switch (phydev->master_slave_set) {
	case MASTER_SLAVE_CFG_MASTER_FORCE:
	case MASTER_SLAVE_CFG_MASTER_PREFERRED:
		ret = phy_set_bits_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_M, ADIN_AN_ADV_MST);
		if (ret < 0)
			return ret;
		break;
	case MASTER_SLAVE_CFG_SLAVE_FORCE:
	case MASTER_SLAVE_CFG_SLAVE_PREFERRED:
		ret = phy_clear_bits_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_M,
					 ADIN_AN_ADV_MST);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}

	if (priv->tx_level_24v)
		ret = phy_set_bits_mmd(phydev, MDIO_MMD_AN,
				       ADIN_AN_ADV_ABILITY_H,
				       ADIN_AN_ADV_B10L_TX_LVL_HI_ABL |
				       ADIN_AN_ADV_B10L_TX_LVL_HI_REQ);
	else
		ret = phy_clear_bits_mmd(phydev, MDIO_MMD_AN,
					 ADIN_AN_ADV_ABILITY_H,
					 ADIN_AN_ADV_B10L_TX_LVL_HI_ABL |
					 ADIN_AN_ADV_B10L_TX_LVL_HI_REQ);

	if (ret < 0)
		return ret;

	return phy_set_bits_mmd(phydev, MDIO_MMD_AN, ADIN_AN_CONTROL, ADIN_AN_RESTART);
}

static void adin_link_change_notify(struct phy_device *phydev)
{
	bool tx_level_24v;
	bool lp_tx_level_24v;
	int val, mask;

	if (phydev->state != PHY_RUNNING || phydev->autoneg == AUTONEG_DISABLE)
		return;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_H);
	if (val < 0)
		return;

	mask = ADIN_AN_LP_ADV_B10L_TX_LVL_HI_ABL | ADIN_AN_LP_ADV_B10L_TX_LVL_HI_REQ;
	lp_tx_level_24v = (val & mask) == mask;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_H);
	if (val < 0)
		return;

	mask = ADIN_AN_ADV_B10L_TX_LVL_HI_ABL | ADIN_AN_ADV_B10L_TX_LVL_HI_REQ;
	tx_level_24v = (val & mask) == mask;

	if (tx_level_24v && lp_tx_level_24v)
		phydev_info(phydev, "Negotiated 2.4V TX level\n");
	else
		phydev_info(phydev, "Negotiated 1.0V TX level\n");
}

static int adin_set_powerdown_mode(struct phy_device *phydev, bool en)
{
	int ret;
	int val;

	if (en)
		val = ADIN_CRSM_SFT_PD_CNTRL_EN;
	else
		val = 0;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1,
			    ADIN_CRSM_SFT_PD_CNTRL, val);
	if (ret < 0)
		return ret;

	return phy_read_mmd_poll_timeout(phydev, MDIO_MMD_VEND1, ADIN_CRSM_STAT, ret,
					 (ret & ADIN_CRSM_SFT_PD_RDY) == val,
					 1000, 30000, true);
}

static int adin_suspend(struct phy_device *phydev)
{
	return adin_set_powerdown_mode(phydev, true);
}

static int adin_resume(struct phy_device *phydev)
{
	return adin_set_powerdown_mode(phydev, false);
}

static int adin_set_loopback(struct phy_device *phydev, bool enable)
{
	if (enable)
		return phy_set_bits_mmd(phydev, MDIO_MMD_PCS, ADIN_B10L_PCS_CNTRL,
					ADIN_PCS_CNTRL_B10L_LB_PCS_EN);

	/* PCS loopback (according to 10BASE-T1L spec) */
	return phy_clear_bits_mmd(phydev, MDIO_MMD_PCS, ADIN_B10L_PCS_CNTRL,
				 ADIN_PCS_CNTRL_B10L_LB_PCS_EN);
}

static int adin_config_init(struct phy_device *phydev)
{
	struct adin_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	int ret;

	ret = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, ADIN_B10L_PMA_STAT);
	if (ret < 0)
		return ret;

	/* This depends on the voltage level from the power source */
	priv->tx_level_24v = !!(ret & ADIN_PMA_STAT_B10L_TX_LVL_HI_ABLE);

	phydev_dbg(phydev, "PHY supports 2.4V TX level: %s\n",
		   priv->tx_level_24v ? "yes" : "no");

	if (priv->tx_level_24v &&
	    device_property_present(dev, "adi,disable-2-4-v-tx-level")) {
		phydev_info(phydev,
			    "PHY supports 2.4V TX level, but disabled via config\n");
		priv->tx_level_24v = 0;
	}

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND2, ADIN_FC_EN, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int adin_soft_reset(struct phy_device *phydev)
{
	int ret;

	ret = phy_set_bits_mmd(phydev, MDIO_MMD_VEND1, ADIN_CRSM_SFT_RST, ADIN_CRSM_SFT_RST_EN);
	if (ret < 0)
		return ret;

	return phy_read_mmd_poll_timeout(phydev, MDIO_MMD_VEND1, ADIN_CRSM_STAT, ret,
					 (ret & ADIN_CRSM_SYS_RDY),
					 10000, 30000, true);
}

static int adin_get_features(struct phy_device *phydev)
{
	linkmode_set_bit_array(phy_basic_ports_array, ARRAY_SIZE(phy_basic_ports_array),
			       phydev->supported);

	linkmode_set_bit_array(phy_10_features_array, ARRAY_SIZE(phy_10_features_array),
			       phydev->supported);

	return 0;
}

static int adin_get_sset_count(struct phy_device *phydev)
{
	return ARRAY_SIZE(adin_hw_stats);
}

static void adin_get_strings(struct phy_device *phydev, u8 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adin_hw_stats); i++)
		strlcpy(&data[i * ETH_GSTRING_LEN], adin_hw_stats[i].string, ETH_GSTRING_LEN);
}

static u64 adin_get_stat(struct phy_device *phydev, int i)
{
	const struct adin_hw_stat *stat = &adin_hw_stats[i];
	struct adin_priv *priv = phydev->priv;
	u64 val;
	int ret;

	ret = phy_read_mmd(phydev, MDIO_MMD_VEND2, stat->reg1);
	if (ret < 0)
		return (u64)(~0);

	val = (0xffff & ret);

	if (stat->reg2 != 0) {
		ret = phy_read_mmd(phydev, MDIO_MMD_VEND2, stat->reg2);
		if (ret < 0)
			return (u64)(~0);

		val = (val << 16) + (0xffff & ret);
	}

	priv->stats[i] += val;

	return priv->stats[i];
}

static void adin_get_stats(struct phy_device *phydev, struct ethtool_stats *stats, u64 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adin_hw_stats); i++)
		data[i] = adin_get_stat(phydev, i);
}

static int adin_get_sqi(struct phy_device *phydev)
{
	u16 mse_val;
	int sqi;
	int ret;

	ret = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_STAT1);
	if (ret < 0)
		return ret;
	else if (!(ret & MDIO_STAT1_LSTATUS))
		return 0;

	ret = phy_read_mmd(phydev, MDIO_STAT1, ADIN_MSE_VAL);
	if (ret < 0)
		return ret;

	mse_val = 0xFFFF & ret;
	for (sqi = 0; sqi < ARRAY_SIZE(adin_mse_sqi_map); sqi++) {
		if (mse_val >= adin_mse_sqi_map[sqi].start && mse_val <= adin_mse_sqi_map[sqi].end)
			return sqi;
	}

	return -EINVAL;
}

static int adin_get_sqi_max(struct phy_device *phydev)
{
	return ADIN_SQI_MAX;
}

static int adin_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct adin_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	return 0;
}

static struct phy_driver adin_driver[] = {
	{
		PHY_ID_MATCH_MODEL(PHY_ID_ADIN1100),
		.name			= "ADIN1100",
		.get_features		= adin_get_features,
		.soft_reset		= adin_soft_reset,
		.probe			= adin_probe,
		.config_init		= adin_config_init,
		.config_aneg		= adin_config_aneg,
		.link_change_notify	= adin_link_change_notify,
		.read_status		= adin_read_status,
		.set_loopback		= adin_set_loopback,
		.suspend		= adin_suspend,
		.resume			= adin_resume,
		.get_sset_count		= adin_get_sset_count,
		.get_strings		= adin_get_strings,
		.get_stats		= adin_get_stats,
		.get_sqi		= adin_get_sqi,
		.get_sqi_max		= adin_get_sqi_max,
	},
};

module_phy_driver(adin_driver);

static struct mdio_device_id __maybe_unused adin_tbl[] = {
	{ PHY_ID_MATCH_MODEL(PHY_ID_ADIN1100) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, adin_tbl);
MODULE_DESCRIPTION("Analog Devices Industrial Ethernet T1L PHY driver");
MODULE_LICENSE("Dual BSD/GPL");
