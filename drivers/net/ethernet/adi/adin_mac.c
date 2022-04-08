// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* Analog Devices 10BASE-T1L MAC common functions.
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include "adin_mac.h"

DECLARE_CRC8_TABLE(adin_mac_crc_table);

static u8 adin_mac_crc_data(u8 *data, u32 len)
{
	return crc8(adin_mac_crc_table, data, len, 0);
}

int adin_mac_read_reg(struct adin_mac *priv, u16 reg, u32 *val)
{
	struct spi_transfer t[2] = {0};
	__le16 __reg = cpu_to_le16(reg);
	u32 header_len = ADIN_MAC_RD_HEADER_LEN;
	u32 read_len = ADIN_MAC_REG_LEN;
	int ret;

	priv->data[0] = ADIN_MAC_CD | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);
	priv->data[2] = 0x00;

	if (priv->append_crc) {
		priv->data[2] = adin_mac_crc_data(&priv->data[0], 2);
		priv->data[3] = 0x00;
		header_len++;
	}

	t[0].tx_buf = &priv->data[0];
	t[0].len = header_len;

	if (priv->append_crc)
		read_len++;

	memset(&priv->data[header_len], 0, read_len);
	t[1].rx_buf = &priv->data[header_len];
	t[1].len = read_len;

	ret = spi_sync_transfer(priv->spidev, t, 2);
	if (ret)
		return ret;

	if (priv->append_crc) {
		u8 recv_crc;
		u8 crc;

		crc = adin_mac_crc_data(&priv->data[header_len], ADIN_MAC_REG_LEN);
		recv_crc = priv->data[header_len + ADIN_MAC_REG_LEN];

		if (crc != recv_crc) {
			netdev_err(priv->netdev, "CRC error.");
			return -EBADMSG;
		}
	}

	*val = get_unaligned_be32(&priv->data[header_len]);

	return ret;
}
EXPORT_SYMBOL_GPL(adin_mac_read_reg);

int adin_mac_write_reg(struct adin_mac *priv, u16 reg, u32 val)
{
	u32 header_len = ADIN_MAC_WR_HEADER_LEN;
	u32 write_len = ADIN_MAC_REG_LEN;
	__le16 __reg = cpu_to_le16(reg);

	priv->data[0] = ADIN_MAC_CD | ADIN_MAC_WRITE | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);

	if (priv->append_crc) {
		priv->data[2] = adin_mac_crc_data(&priv->data[0], header_len);
		header_len++;
	}

	put_unaligned_be32(val, &priv->data[header_len]);
	if (priv->append_crc) {
		priv->data[header_len + write_len] = adin_mac_crc_data(&priv->data[header_len],
								       write_len);
		write_len++;
	}

	return spi_write(priv->spidev, &priv->data[0], header_len + write_len);
}
EXPORT_SYMBOL_GPL(adin_mac_write_reg);


int adin_mac_set_bits(struct adin_mac *priv, u16 reg, unsigned long mask, unsigned long val)
{
	u32 write_val;
	int ret;

	ret = adin_mac_read_reg(priv, reg, &write_val);
	if (ret < 0)
		return ret;

	set_mask_bits(&write_val, mask, val);

	return adin_mac_write_reg(priv, reg, write_val);
}
EXPORT_SYMBOL_GPL(adin_mac_set_bits);

static int adin_mac_round_len(int len)
{
	/* can read/write only mutiples of 4 bytes of payload */
	len = ALIGN(len, 4);

	/* NOTE: ADIN_MAC_WR_HEADER_LEN should be used for write ops. */
	if (len + ADIN_MAC_RD_HEADER_LEN > ADIN_MAC_MAX_BUFF)
		return -EINVAL;

	return len;
}

int adin_mac_read_fifo(struct adin_mac *priv, u32 reg)
{
	u32 header_len = ADIN_MAC_RD_HEADER_LEN;
	__le16 __reg = cpu_to_le16(reg);
	struct spi_transfer t[2] = {0};
	struct sk_buff *rxb;
	u32 frame_size;
	u32 frame_size_no_fcs;
	int round_len;
	int ret;

	ret = adin_mac_read_reg(priv, ADIN_MAC_RX_FSIZE, &frame_size);
	if (ret < 0)
		return ret;

	/* the read frame size includes the extra 2 bytes from the ADIN MAC frame header */
	if (frame_size < ADIN_MAC_FRAME_HEADER_LEN + ADIN_MAC_FEC_LEN)
		return ret;

	round_len = adin_mac_round_len(frame_size);
	if (round_len < 0)
		return ret;

	frame_size_no_fcs = frame_size - ADIN_MAC_FRAME_HEADER_LEN - ADIN_MAC_FEC_LEN;

	rxb = netdev_alloc_skb(priv->netdev, frame_size_no_fcs);
	if (!rxb)
		return -ENOMEM;

	memset(priv->data, 0, round_len + ADIN_MAC_RD_HEADER_LEN);

	priv->data[0] = ADIN_MAC_CD | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);
	priv->data[2] = 0x00;

	if (priv->append_crc) {
		priv->data[2] = adin_mac_crc_data(&priv->data[0], 2);
		priv->data[3] = 0x00;
		header_len++;
	}

	t[0].tx_buf = &priv->data[0];
	t[0].len = header_len;

	t[1].rx_buf = &priv->data[header_len];
	t[1].len = round_len;

	ret = spi_sync_transfer(priv->spidev, t, 2);
	if (ret) {
		kfree_skb(rxb);
		return ret;
	}

	skb_put(rxb, frame_size_no_fcs);
	skb_copy_to_linear_data(rxb, &priv->data[header_len + ADIN_MAC_FRAME_HEADER_LEN],
				frame_size_no_fcs);

	rxb->protocol = eth_type_trans(rxb, priv->netdev);

	netif_rx_ni(rxb);

	priv->rx_bytes += frame_size - ADIN_MAC_FRAME_HEADER_LEN;

	return 0;
}
EXPORT_SYMBOL_GPL(adin_mac_read_fifo);

int adin_mac_write_fifo(struct adin_mac *priv, struct sk_buff *txb, u32 reg)
{
	u32 header_len = ADIN_MAC_WR_HEADER_LEN;
	__le16 __reg = cpu_to_le16(reg);
	int padding = 0;
	int round_len;
	int padded_len;
	int ret;

	/* Pad frame to 64 byte length,
	 * MAC nor PHY will otherwise add the
	 * required padding.
	 * The FEC will be added by the MAC internally.
	 */
	if (txb->len + ADIN_MAC_FEC_LEN < 64)
		padding = 64 - (txb->len + ADIN_MAC_FEC_LEN);

	padded_len = txb->len + padding + ADIN_MAC_FRAME_HEADER_LEN;

	round_len = adin_mac_round_len(padded_len);
	if (round_len < 0)
		return round_len;

	ret = adin_mac_write_reg(priv, ADIN_MAC_TX_FSIZE, padded_len);
	if (ret < 0)
		return ret;

	memset(priv->data, 0, round_len + ADIN_MAC_WR_HEADER_LEN);

	priv->data[0] = ADIN_MAC_CD | ADIN_MAC_WRITE | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);
	if (priv->append_crc) {
		priv->data[2] = adin_mac_crc_data(&priv->data[0], 2);
		header_len++;
	}

	memcpy(&priv->data[header_len + ADIN_MAC_FRAME_HEADER_LEN], txb->data, txb->len);

	ret = spi_write(priv->spidev, &priv->data[0], round_len + header_len);
	if (ret < 0)
		return ret;

	priv->tx_bytes += txb->len;

	return 0;
}
EXPORT_SYMBOL_GPL(adin_mac_write_fifo);

static int adin_mac_read_mdio_acc(struct adin_mac *priv)
{
	u32 val;
	int ret;

	ret = adin_mac_read_reg(priv, ADIN_MAC_MDIOACC, &val);
	if (ret < 0)
		return 0;

	return val;
}

int adin_mac_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct adin_mac *priv = bus->priv;
	u32 val = 0;
	int ret;

	mutex_lock(&priv->lock);

	val |= FIELD_PREP(ADIN_MAC_MDIO_OP, ADIN_MAC_MDIO_OP_RD);
	val |= FIELD_PREP(ADIN_MAC_MDIO_ST, 0x1);
	val |= FIELD_PREP(ADIN_MAC_MDIO_PRTAD, 0x1);
	val |= FIELD_PREP(ADIN_MAC_MDIO_DEVAD, reg);

	/* write the clause 22 read command to the chip */
	ret = adin_mac_write_reg(priv, ADIN_MAC_MDIOACC, val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	/* ADIN_MAC_MDIO_TRDONE BIT of the ADIN_MAC_MDIOACC
	 * register is set when the read is done.
	 * After the transaction is done, ADIN_MAC_MDIO_DATA
	 * bitfield of ADIN_MAC_MDIOACC register will contain
	 * the requested register value.
	 */
	ret = readx_poll_timeout(adin_mac_read_mdio_acc, priv, val, (val & ADIN_MAC_MDIO_TRDONE),
				 10000, 30000);
	mutex_unlock(&priv->lock);

	if (ret < 0)
		return ret;

	return (val & ADIN_MAC_MDIO_DATA);
}
EXPORT_SYMBOL_GPL(adin_mac_mdio_read);

int adin_mac_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 reg_val)
{
	struct adin_mac *priv = bus->priv;
	u32 val = 0;
	int ret;

	mutex_lock(&priv->lock);

	val |= FIELD_PREP(ADIN_MAC_MDIO_OP, ADIN_MAC_MDIO_OP_WR);
	val |= FIELD_PREP(ADIN_MAC_MDIO_ST, 0x1);
	val |= FIELD_PREP(ADIN_MAC_MDIO_PRTAD, 0x1);
	val |= FIELD_PREP(ADIN_MAC_MDIO_DEVAD, reg);
	val |= FIELD_PREP(ADIN_MAC_MDIO_DATA, reg_val);

	/* write the clause 22 write command to the chip */
	ret = adin_mac_write_reg(priv, ADIN_MAC_MDIOACC, val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	ret = readx_poll_timeout(adin_mac_read_mdio_acc, priv, val, (val & ADIN_MAC_MDIO_TRDONE),
				 10000, 30000);
	mutex_unlock(&priv->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(adin_mac_mdio_write);

/* ADIN MAC-PHY contains one or more ADIN1100 PHYs.
 * By registering a new MDIO bus we allow the PAL to discover
 * the encapsulated PHYs and probe the ADIN1100 driver.
 */
int adin_mac_register_mdiobus(struct adin_mac *priv, struct device *dev,
			      const char* name, unsigned int mask)
{
	struct mii_bus *mii_bus;
	int ret;

	mii_bus = devm_mdiobus_alloc(dev);
	if (!mii_bus)
		return -ENOMEM;

	mii_bus->name = name;
	mii_bus->read = adin_mac_mdio_read;
	mii_bus->write = adin_mac_mdio_write;
	mii_bus->priv = priv;
	mii_bus->parent = dev;
	mii_bus->phy_mask = ~((u32)mask);
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	ret = devm_mdiobus_register(dev, mii_bus);
	if (ret)
		return ret;

	priv->mii_bus = mii_bus;

	return 0;
}
EXPORT_SYMBOL_GPL(adin_mac_register_mdiobus);

/* ADIN MAC can filter up to 16 MAC addresses, mac_nr here is the slot used */
static int adin_mac_write_mac_address(struct adin_mac *priv, int mac_nr, u8 *addr, u8 *mask)
{
	u32 offset = mac_nr * 2;
	int ret;
	u32 val;

	/* tell MAC to forward this DA to host */
	val = ADIN_MAC_MAC_ADDR_APPLY2PORT | ADIN_MAC_MAC_ADDR_TO_HOST;
	val |= get_unaligned_be16(&addr[0]);
	ret = adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_FILTER_UPR + offset, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be32(&addr[2]);
	ret =  adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_FILTER_LWR + offset, val);
	if (ret < 0)
		return ret;

	val = ADIN_MAC_MAC_ADDR_APPLY2PORT | ADIN_MAC_MAC_ADDR_TO_HOST;
	val |= get_unaligned_be16(&mask[0]);
	ret = adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_MASK_UPR + offset, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be32(&mask[2]);
	return adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_MASK_LWR + offset, val);
}

static int adin_mac_clear_mac_address(struct adin_mac *priv, int mac_nr)
{
	u32 offset = mac_nr * 2;
	int ret;

	ret = adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_FILTER_UPR + offset, 0);
	if (ret < 0)
		return ret;

	ret =  adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_FILTER_LWR + offset, 0);
	if (ret < 0)
		return ret;

	ret = adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_MASK_UPR + offset, 0);
	if (ret < 0)
		return ret;

	return adin_mac_write_reg(priv, ADIN_MAC_MAC_ADDR_MASK_LWR + offset, 0);
}

int adin_mac_multicast_filter(struct adin_mac *priv, int mac_nr, bool accept_multicast)
{
	u8 mask[ETH_ALEN] = {0};
	u8 mac[ETH_ALEN] = {0};

	if (accept_multicast) {
		mask[0] = BIT(1);
		mac[0] = BIT(1);

		return adin_mac_write_mac_address(priv, mac_nr, mac, mask);
	}

	return adin_mac_clear_mac_address(priv, mac_nr);
}
EXPORT_SYMBOL_GPL(adin_mac_multicast_filter);

int adin_mac_set_mac_address(struct net_device *netdev, void *addr)
{
	struct adin_mac *priv = netdev_priv(netdev);
	struct sockaddr *sa = addr;
	u8 mask[ETH_ALEN];

	if (netif_running(netdev))
		return -EBUSY;

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;

	ether_addr_copy(netdev->dev_addr, sa->sa_data);
	memset(mask, 0xFF, ETH_ALEN);

	return adin_mac_write_mac_address(priv, 0, netdev->dev_addr, mask);
}
EXPORT_SYMBOL_GPL(adin_mac_set_mac_address);

int adin_mac_init_mac(struct adin_mac *priv)
{
	struct net_device *netdev = priv->netdev;
	u8 mask[ETH_ALEN];
	u8 mac[ETH_ALEN];
	int ret;

	memset(mask, 0xFF, ETH_ALEN);
	ret = adin_mac_write_mac_address(priv, 0, netdev->dev_addr, mask);
	if (ret < 0) {
		netdev_err(netdev, "Could not set MAC address: %pM, %d\n", mac, ret);
		return ret;
	}

	memset(mac, 0xFF, ETH_ALEN);
	ret = adin_mac_write_mac_address(priv, 1, mac, mask);
	if (ret < 0) {
		netdev_err(netdev, "Could not set Broadcast MAC address: %d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(adin_mac_init_mac);

void adin_mac_rx_mode_work(struct work_struct *work)
{
	struct adin_mac *priv = container_of(work, struct adin_mac, rx_mode_work);

	mutex_lock(&priv->lock);

	adin_mac_set_bits(priv, ADIN_MAC_CONFIG2, ADIN_MAC_FWD_UNK2HOST,
			  (priv->flags & IFF_PROMISC) ? ADIN_MAC_FWD_UNK2HOST : 0);

	adin_mac_multicast_filter(priv, 2, !!(priv->flags & IFF_ALLMULTI));

	mutex_unlock(&priv->lock);
}
EXPORT_SYMBOL_GPL(adin_mac_rx_mode_work);

void adin_mac_set_rx_mode(struct net_device *dev)
{
	struct adin_mac *priv = netdev_priv(dev);

	spin_lock(&priv->state_lock);

	priv->flags = dev->flags;
	schedule_work(&priv->rx_mode_work);

	spin_unlock(&priv->state_lock);
}
EXPORT_SYMBOL_GPL(adin_mac_set_rx_mode);

/* PHY ID is stored in the MAC registers too, check spi connection by reading it */
static int adin_mac_check_spi(struct adin_mac *priv)
{
	int ret;
	u32 val;

	ret = adin_mac_read_reg(priv, ADIN_MAC_PHY_ID, &val);
	if (ret < 0)
		return ret;

	if (val != priv->cfg->phy_id_check) {
		netdev_err(priv->netdev, "PHY ID read: %x\n", val);
		return -EIO;
	}

	snprintf(priv->netdev->name, IFNAMSIZ, "%s-%u", priv->cfg->name,
		 priv->spidev->chip_select);

	return 0;
}

struct adin_mac *adin_mac_init(struct spi_device *spi, const struct adin_mac_config *cfg)
{
	struct device *dev = &spi->dev;
	struct net_device *netdev;
	struct adin_mac *priv;
	const u8 *mac_addr;
	u8 mac[ETH_ALEN];
	int ret;

	netdev = devm_alloc_etherdev(dev, sizeof(struct adin_mac));
	if (!netdev)
		return ERR_PTR(-ENOMEM);

	priv = netdev_priv(netdev);
	priv->cfg = cfg;
	priv->spidev = spi;
	priv->netdev = netdev;
	spi->bits_per_word = 8;
	SET_NETDEV_DEV(netdev, dev);

	mutex_init(&priv->lock);
	spin_lock_init(&priv->state_lock);

	/* use of CRC on control and data transactions is pin dependent */
	priv->append_crc = device_property_read_bool(dev, "adi,spi-crc");
	if (priv->append_crc)
		crc8_populate_msb(adin_mac_crc_table, 0x7);

	mac_addr = device_get_mac_address(dev, mac, ETH_ALEN);
	if (!mac_addr) {
		netdev_err(netdev, "MAC address invalid: %pM, %d\n", mac, ret);
		return ERR_PTR(-EINVAL);
	}

	ether_addr_copy(netdev->dev_addr, mac);

	INIT_WORK(&priv->tx_work, cfg->tx_work);
	INIT_WORK(&priv->rx_mode_work, adin_mac_rx_mode_work);

	ret = adin_mac_check_spi(priv);
	if (ret < 0) {
		netdev_err(netdev, "SPI read failed: %d\n", ret);
		return ERR_PTR(ret);
	}

	snprintf(priv->mii_bus_name, MII_BUS_ID_SIZE, "%s_%u_eth_mii",
		 priv->cfg->name, priv->spidev->chip_select);
	ret = adin_mac_register_mdiobus(priv, dev, priv->mii_bus_name, priv->cfg->phy_mask);
	if (ret < 0) {
		netdev_err(netdev, "Could not register MDIO bus %d\n", ret);
		return ERR_PTR(ret);
	}

	skb_queue_head_init(&priv->txq);

	netdev->irq = spi->irq;

	netif_carrier_off(priv->netdev);

	/* FIXME: This should be changed to 10BASET1L when introduced to PAL */
	netdev->if_port = IF_PORT_10BASET;
	netdev->netdev_ops = cfg->netdev_ops;
	netdev->ethtool_ops = cfg->ethtool_ops;

	ret = devm_register_netdev(dev, netdev);
	if (ret) {
		dev_err(dev, "failed to register network device\n");
		return ERR_PTR(ret);
	}

	return priv;
}
EXPORT_SYMBOL_GPL(adin_mac_init);
