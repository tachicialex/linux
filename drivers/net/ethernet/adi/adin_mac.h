// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* Analog Devices 10BASE-T1L MAC common functions.
 *
 * Copyright 2021 Analog Devices Inc.
 */

 #ifndef _ADIN_MAC_H
 #define _ADIN_MAC_H

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cache.h>
#include <linux/crc8.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

#define ADIN_MAC_PHY_ID				0x1

#define ADIN_MAC_CONFIG2			0x06
#define   ADIN_MAC_CRC_APPEND			BIT(5)
#define   ADIN_MAC_FWD_UNK2HOST			BIT(2)

#define ADIN_MAC_STATUS0			0x08

#define ADIN_MAC_STATUS1			0x09
#define   ADIN_MAC_SPI_ERR			BIT(10)
#define   ADIN_MAC_RX_RDY			BIT(4)
#define   ADIN_MAC_TX_RDY			BIT(3)

#define ADIN_MAC_IMASK1				0x0D
#define   ADIN_MAC_SPI_ERR_IRQ			BIT(10)
#define   ADIN_MAC_RX_RDY_IRQ			BIT(4)
#define   ADIN_MAC_TX_RDY_IRQ			BIT(3)
#define   ADIN_MAC_LINK_CHANGE_IRQ		BIT(1)

#define ADIN_MAC_MDIOACC			0x20
#define   ADIN_MAC_MDIO_TRDONE			BIT(31)
#define   ADIN_MAC_MDIO_TAERR			BIT(30)
#define   ADIN_MAC_MDIO_ST			GENMASK(29, 28)
#define   ADIN_MAC_MDIO_OP			GENMASK(27, 26)
#define   ADIN_MAC_MDIO_PRTAD			GENMASK(25, 21)
#define   ADIN_MAC_MDIO_DEVAD			GENMASK(20, 16)
#define   ADIN_MAC_MDIO_DATA			GENMASK(15, 0)

#define ADIN_MAC_TX_FSIZE			0x30
#define ADIN_MAC_TX				0x31
#define ADIN_MAC_TX_SPACE			0x32
#define ADIN_MAC_RX_THRESH			0x33
#define ADIN_MAC_TX_THRESH			0x34

#define ADIN_MAC_FIFO_CLR			0x36
#define ADIN_MAC_FIFO_SIZE			0x3E
#define ADIN_MAC_TFC				0x3F

#define ADIN_MAC_MAC_ADDR_FILTER_UPR		0x50
#define   ADIN_MAC_MAC_ADDR_APPLY2PORT		GENMASK(31, 30)
#define   ADIN_MAC_MAC_ADDR_HOST_PRI		BIT(19)
#define   ADIN_MAC_MAC_ADDR_TO_HOST		BIT(16)
#define   ADIN_MAC_MAC_ADDR			GENMASK(15, 0)

#define ADIN_MAC_MAC_ADDR_FILTER_LWR		0x51

#define ADIN_MAC_MAC_ADDR_MASK_UPR		0x70
#define ADIN_MAC_MAC_ADDR_MASK_LWR		0x71

#define ADIN_MAC_RX_FSIZE			0x90
#define ADIN_MAC_RX				0x91

#define ADIN_MAC_RX_FRM_CNT			0xA0
#define ADIN_MAC_RX_MCAST_CNT			0xA2
#define ADIN_MAC_RX_CRC_ERR_CNT			0xA4
#define ADIN_MAC_RX_ALGN_ERR_CNT		0xA5
#define ADIN_MAC_RX_LS_ERR_CNT			0xA6
#define ADIN_MAC_RX_PHY_ERR_CNT			0xA7
#define ADIN_MAC_RX_DROP_FULL_CNT		0xAC

#define ADIN_MAC_TX_FRM_CNT			0xA8
#define ADIN_MAC_TX_MCAST_CNT			0xAA

#define ADIN_MAC_CLEAR_STATUS0			0x1F7F
#define ADIN_MAC_CLEAR_STATUS1			0x1F01D0A

/* MDIO_OP codes */
#define ADIN_MAC_MDIO_OP_WR			0x1
#define ADIN_MAC_MDIO_OP_RD			0x3

#define ADIN_MAC_CD				BIT(7)
#define ADIN_MAC_WRITE				BIT(5)

#define ADIN_MAC_MAX_BUFF			2048
#define ADIN_MAC_WR_HEADER_LEN			2
#define ADIN_MAC_FRAME_HEADER_LEN		2
#define ADIN_MAC_INTERNAL_SIZE_HEADER_LEN	2
#define ADIN_MAC_RD_HEADER_LEN			3
#define ADIN_MAC_REG_LEN			4
#define ADIN_MAC_FEC_LEN			4

#define ADIN_MAC_TX_SPACE_MAX			0x0FFF

struct adin_mac {
	struct mutex			lock; /* protect spi */
	spinlock_t			state_lock; /* protect RX mode */
	struct work_struct		tx_work;
	u32				tx_space;
	u64				rx_bytes;
	u64				tx_bytes;
	struct work_struct		rx_mode_work;
	u32				flags;
	struct sk_buff_head		txq;
	struct mii_bus			*mii_bus;
	struct phy_device		*phydev;
	struct net_device		*netdev;
	struct spi_device		*spidev;
	bool				append_crc;
	int				irq;
	char				mii_bus_name[MII_BUS_ID_SIZE];
	const struct adin_mac_config	*cfg;
	u8				data[ADIN_MAC_MAX_BUFF] ____cacheline_aligned;
};

struct adin_mac_config {
	u32				phy_id_check;
	char				name[MII_BUS_ID_SIZE];
	unsigned int			phy_mask;
	const struct net_device_ops	*netdev_ops;
	const struct ethtool_ops 	*ethtool_ops;
	void				(*tx_work)(struct work_struct *);
};

int adin_mac_read_reg(struct adin_mac *priv, u16 reg, u32 *val);
int adin_mac_write_reg(struct adin_mac *priv, u16 reg, u32 val);
int adin_mac_set_bits(struct adin_mac *priv, u16 reg, unsigned long mask, unsigned long val);
int adin_mac_read_fifo(struct adin_mac *priv, u32 reg);
int adin_mac_write_fifo(struct adin_mac *priv, struct sk_buff *txb, u32 reg);
int adin_mac_mdio_read(struct mii_bus *bus, int phy_id, int reg);
int adin_mac_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 reg_val);
int adin_mac_multicast_filter(struct adin_mac *priv, int mac_nr, bool accept_multicast);
int adin_mac_set_mac_address(struct net_device *netdev, void *addr);
void adin_mac_set_rx_mode(struct net_device *dev);
int adin_mac_init_mac(struct adin_mac *priv);
int adin_mac_register_mdiobus(struct adin_mac *priv, struct device *dev,
			      const char* name, unsigned int mask);
struct adin_mac *adin_mac_init(struct spi_device *spi, const struct adin_mac_config *cfg);

#endif	/* _ADIN_MAC_H */
