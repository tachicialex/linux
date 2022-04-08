// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* ADIN1110 Low Power 10BASE-T1L Ethernet MAC-PH
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include "adin_mac.h"

#define ADIN1110_PHY_ID_VAL	0x0283BC91

static void adin1110_read_frames(struct adin_mac *priv)
{
	u32 status1;
	int ret;

	while (1) {
		ret = adin_mac_read_reg(priv, ADIN_MAC_STATUS1, &status1);
		if (ret < 0)
			return;

		if (!(status1 & ADIN_MAC_RX_RDY))
			break;

		ret = adin_mac_read_fifo(priv, ADIN_MAC_RX);
		if (ret < 0)
			return;
	}
}

static irqreturn_t adin1110_irq(int irq, void *p)
{
	struct adin_mac *priv = p;
	u32 status1;
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	adin_mac_read_reg(priv, ADIN_MAC_STATUS1, &status1);

	if (priv->append_crc && (status1 & ADIN_MAC_SPI_ERR))
		netdev_warn(priv->netdev, "SPI CRC error on write.\n");

	ret = adin_mac_read_reg(priv, ADIN_MAC_TX_SPACE, &val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return IRQ_HANDLED;
	}

	priv->tx_space = val;

	if (status1 & ADIN_MAC_RX_RDY)
		adin1110_read_frames(priv);

	/* clear IRQ sources */
	adin_mac_write_reg(priv, ADIN_MAC_STATUS0, ADIN_MAC_CLEAR_STATUS0);
	adin_mac_write_reg(priv, ADIN_MAC_STATUS1, ADIN_MAC_CLEAR_STATUS1);

	mutex_unlock(&priv->lock);

	if (priv->tx_space > 0)
		netif_wake_queue(priv->netdev);

	return IRQ_HANDLED;
}

static int adin1110_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	if (!netif_running(netdev))
		return -EINVAL;

	if (!netdev->phydev)
		return -ENODEV;

	return phy_mii_ioctl(netdev->phydev, rq, cmd);
}

static int adin1110_net_open(struct net_device *net_dev)
{
	struct adin_mac *priv = netdev_priv(net_dev);
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	val = ADIN_MAC_CRC_APPEND;
	ret = adin_mac_set_bits(priv, ADIN_MAC_CONFIG2, val, val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	val = ADIN_MAC_TX_RDY_IRQ | ADIN_MAC_RX_RDY_IRQ | ADIN_MAC_LINK_CHANGE_IRQ |
	      ADIN_MAC_SPI_ERR_IRQ;
	ret = adin_mac_set_bits(priv, ADIN_MAC_IMASK1, val, 0);
	if (ret < 0) {
		netdev_err(net_dev, "Failed to enable chip IRQs: %d\n", ret);
		mutex_unlock(&priv->lock);
		return ret;
	}

	ret = adin_mac_read_reg(priv, ADIN_MAC_TX_SPACE, &val);
	if (ret < 0) {
		netdev_err(net_dev, "Failed to read TX FIFO space: %d\n", ret);
		mutex_unlock(&priv->lock);
		return ret;
	}

	priv->tx_space = val;

	ret = adin_mac_init_mac(priv);
	if (ret < 0)
		return ret;

	mutex_unlock(&priv->lock);

	phy_start(priv->phydev);

	/* ADIN1110 INT_N pin will be used to signal the host */
	ret = request_threaded_irq(net_dev->irq, NULL, adin1110_irq,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   net_dev->name, priv);

	if (ret < 0) {
		netdev_err(net_dev, "Failed to get IRQ: %d\n", ret);
		return ret;
	}

	netif_start_queue(priv->netdev);

	return ret;
}

static int adin1110_net_stop(struct net_device *net_dev)
{
	struct adin_mac *priv = netdev_priv(net_dev);

	netif_stop_queue(priv->netdev);
	flush_work(&priv->tx_work);
	phy_stop(priv->phydev);
	free_irq(net_dev->irq, priv);

	return 0;
}

static void adin1110_tx_work(struct work_struct *work)
{
	struct adin_mac *priv = container_of(work, struct adin_mac, tx_work);
	struct sk_buff *txb;
	bool last;
	int ret;

	mutex_lock(&priv->lock);

	last = skb_queue_empty(&priv->txq);

	while (!last) {
		txb = skb_dequeue(&priv->txq);
		last = skb_queue_empty(&priv->txq);

		if (txb) {
			ret = adin_mac_write_fifo(priv, txb, ADIN_MAC_TX);
			if (ret < 0)
				netdev_err(priv->netdev, "Frame write error: %d\n", ret);

			dev_kfree_skb(txb);
		}
	}

	mutex_unlock(&priv->lock);
}

static netdev_tx_t adin1110_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct adin_mac *priv = netdev_priv(dev);
	netdev_tx_t netdev_ret = NETDEV_TX_OK;
	u32 tx_space_needed;

	spin_lock(&priv->state_lock);

	tx_space_needed = skb->len + ADIN_MAC_FRAME_HEADER_LEN + ADIN_MAC_INTERNAL_SIZE_HEADER_LEN;
	if (tx_space_needed > priv->tx_space) {
		netif_stop_queue(dev);
		netdev_ret = NETDEV_TX_BUSY;
	} else {
		priv->tx_space -= tx_space_needed;
		skb_queue_tail(&priv->txq, skb);
	}

	spin_unlock(&priv->state_lock);

	schedule_work(&priv->tx_work);

	return netdev_ret;
}

void adin1110_ndo_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *storage)
{
	struct adin_mac *priv = netdev_priv(dev);
	u32 val;

	mutex_lock(&priv->lock);

	adin_mac_read_reg(priv, ADIN_MAC_RX_FRM_CNT, &val);
	storage->rx_packets = val;

	adin_mac_read_reg(priv, ADIN_MAC_TX_FRM_CNT, &val);
	storage->tx_packets = val;

	storage->rx_bytes = priv->rx_bytes;
	storage->tx_bytes = priv->tx_bytes;

	adin_mac_read_reg(priv, ADIN_MAC_RX_CRC_ERR_CNT, &val);
	storage->rx_errors += val;

	adin_mac_read_reg(priv, ADIN_MAC_RX_ALGN_ERR_CNT, &val);
	storage->rx_errors += val;

	adin_mac_read_reg(priv, ADIN_MAC_RX_LS_ERR_CNT, &val);
	storage->rx_errors += val;

	adin_mac_read_reg(priv, ADIN_MAC_RX_PHY_ERR_CNT, &val);
	storage->rx_errors += val;

	adin_mac_read_reg(priv, ADIN_MAC_RX_DROP_FULL_CNT, &val);
	storage->rx_dropped = val;

	adin_mac_read_reg(priv, ADIN_MAC_RX_MCAST_CNT, &val);
	storage->multicast = val;

	mutex_unlock(&priv->lock);
}

static const struct net_device_ops adin1110_netdev_ops = {
	.ndo_open		= adin1110_net_open,
	.ndo_stop		= adin1110_net_stop,
	.ndo_do_ioctl		= adin1110_ioctl,
	.ndo_start_xmit		= adin1110_start_xmit,
	.ndo_set_mac_address	= adin_mac_set_mac_address,
	.ndo_set_rx_mode	= adin_mac_set_rx_mode,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats64	= adin1110_ndo_get_stats64,
};

static void adin1110_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *di)
{
	strscpy(di->driver, "ADIN1110", sizeof(di->driver));
	strscpy(di->version, "1.00", sizeof(di->version));
	strscpy(di->bus_info, dev_name(dev->dev.parent), sizeof(di->bus_info));
}

static const struct ethtool_ops adin1110_ethtool_ops = {
	.get_drvinfo		= adin1110_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
};

static void adin1110_adjust_link(struct net_device *dev)
{
	struct phy_device *phydev = dev->phydev;

	if (!phydev->link)
		phy_print_status(phydev);
}

static void adin1110_disconnect_phy(void *data)
{
	phy_disconnect(data);
}

static const struct adin_mac_config adin1110_mac_cfg = {
	.phy_id_check	= ADIN1110_PHY_ID_VAL,
	.name		= "adin1110",
	.phy_mask	= BIT(0),
	.netdev_ops	= &adin1110_netdev_ops,
	.ethtool_ops	= &adin1110_ethtool_ops,
	.tx_work	= adin1110_tx_work,
};

static int adin1110_probe(struct spi_device *spi)
{
	struct net_device *netdev;
	struct adin_mac *priv;

	priv = adin_mac_init(spi, &adin1110_mac_cfg);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	netdev = priv->netdev;

	/* there is only one PHY connected to our registered MDIO bus */
	priv->phydev = phy_find_first(priv->mii_bus);
	if (!priv->phydev)
		return -ENODEV;

	priv->phydev = phy_connect(netdev, phydev_name(priv->phydev),
				   adin1110_adjust_link, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(priv->phydev))
		return PTR_ERR(priv->phydev);

	return devm_add_action_or_reset(&spi->dev, adin1110_disconnect_phy, priv->phydev);
}

static const struct of_device_id adin1110_match_table[] = {
	{ .compatible = "adi,adin1110" },
	{ }
};
MODULE_DEVICE_TABLE(of, adin1110_match_table);

static struct spi_driver adin1110_driver = {
	.driver = {
		.name = "adin1110",
		.of_match_table = adin1110_match_table,
	},
	.probe = adin1110_probe,
};
module_spi_driver(adin1110_driver);

MODULE_DESCRIPTION("ADIN1110 Network driver");
MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_LICENSE("Dual BSD/GPL");
