/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PTP clock of a Marvell 88E6xxx switch (88E6352 style TAI, as on the
 * 88E6341). The TAI is a free running 32 bit counter; like Linux
 * (drivers/net/dsa/mv88e6xxx/ptp.c) the clock is a software time counter
 * on top of it, so setting, stepping and rate changes never touch the
 * hardware and port time stamps are converted with the same counter.
 *
 * Also does the global PTP setup: L2 PTP EtherType, time stamp all event
 * messages, Pdelay_Resp into the second arrival register, and on the
 * 88E6341 time stamps at the MAC instead of the PHY (unreliable there).
 */

#define DT_DRV_COMPAT marvell_mv88e6xxx_ptp

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/net/ptp_time.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ptp_clock_mv88e6xxx, CONFIG_ETHERNET_LOG_LEVEL);

#include "../ethernet/phy/phy_marvell_mv88e6xxx.h"
#include "ptp_clock_mv88e6xxx.h"
#include "ptp_clock_mv88e6xxx_tc.h"

/* TAI global registers */
#define TAI_CLOCK_PERIOD 0x01
#define TAI_TIME_LO      0x0e

/* PTP global registers */
#define PTP_ETHERTYPE        0x00
#define PTP_MSGTYPE          0x01
#define PTP_MSGTYPE_ALL      0x000f
#define PTP_TS_ARRIVAL_PTR   0x02
#define PTP_MSGTYPE_PDLY_RES 0x0008
#define PTP_6341_CFG         0x07
#define PTP_6341_CFG_TS_MAC  0x8080 /* update, index 0, time stamp at MAC */

#define ETH_P_1588 0x88f7

/* A 32 bit counter at 8 ns wraps every 34 s */
#define OVERFLOW_PERIOD K_SECONDS(4)

/* The PI loop never needs more than this */
#define MAX_RATIO_PPM 1000

struct ptp_mv88e6xxx_config {
	const struct device *sw;
};

struct ptp_mv88e6xxx_data {
	const struct device *dev;
	struct k_mutex lock;
	struct mv_tc tc;
	struct k_work_delayable overflow;
};

static int read_cyc(const struct device *dev, uint32_t *cyc)
{
	const struct ptp_mv88e6xxx_config *cfg = dev->config;
	uint16_t t[2];
	int ret;

	ret = mv88e6xxx_avb_read(cfg->sw, MV88E6XXX_AVB_PORT_TAI, TAI_TIME_LO, t, 2);
	if (ret == 0) {
		*cyc = ((uint32_t)t[1] << 16) | t[0];
	}

	return ret;
}

static int ptp_mv88e6xxx_get(const struct device *dev, struct net_ptp_time *tm)
{
	struct ptp_mv88e6xxx_data *data = dev->data;
	uint32_t cyc;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = read_cyc(dev, &cyc);
	if (ret == 0) {
		mv_tc_update(&data->tc, cyc);
		tm->second = data->tc.ns / NSEC_PER_SEC;
		tm->nanosecond = data->tc.ns % NSEC_PER_SEC;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int ptp_mv88e6xxx_set(const struct device *dev, struct net_ptp_time *tm)
{
	struct ptp_mv88e6xxx_data *data = dev->data;
	uint32_t cyc;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = read_cyc(dev, &cyc);
	if (ret == 0) {
		mv_tc_set(&data->tc, cyc, tm->second * NSEC_PER_SEC + tm->nanosecond);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int ptp_mv88e6xxx_adjust(const struct device *dev, int increment)
{
	struct ptp_mv88e6xxx_data *data = dev->data;
	uint32_t cyc;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = read_cyc(dev, &cyc);
	if (ret == 0) {
		mv_tc_adjust(&data->tc, cyc, increment);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int ptp_mv88e6xxx_rate_adjust(const struct device *dev, double ratio)
{
	struct ptp_mv88e6xxx_data *data = dev->data;
	uint32_t cyc;
	int ret;

	if (ratio < 1.0 - MAX_RATIO_PPM * 1e-6 || ratio > 1.0 + MAX_RATIO_PPM * 1e-6) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = read_cyc(dev, &cyc);
	if (ret == 0) {
		mv_tc_set_ratio(&data->tc, cyc, ratio);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

int ptp_clock_mv88e6xxx_cyc2ns(const struct device *dev, uint32_t cyc, uint64_t *ns)
{
	struct ptp_mv88e6xxx_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	*ns = mv_tc_cyc2ns(&data->tc, cyc);
	k_mutex_unlock(&data->lock);

	return 0;
}

static void ptp_mv88e6xxx_overflow(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ptp_mv88e6xxx_data *data = CONTAINER_OF(dwork, struct ptp_mv88e6xxx_data, overflow);
	uint32_t cyc;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (read_cyc(data->dev, &cyc) == 0) {
		mv_tc_update(&data->tc, cyc);
	}
	k_mutex_unlock(&data->lock);

	k_work_reschedule(dwork, OVERFLOW_PERIOD);
}

static int ptp_mv88e6xxx_init(const struct device *dev)
{
	const struct ptp_mv88e6xxx_config *cfg = dev->config;
	struct ptp_mv88e6xxx_data *data = dev->data;
	uint16_t period_ps;
	uint16_t product;
	uint32_t cyc;
	int ret;

	data->dev = dev;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->overflow, ptp_mv88e6xxx_overflow);

	if (!device_is_ready(cfg->sw)) {
		return -ENODEV;
	}
	product = mv88e6xxx_product(cfg->sw);

	ret = mv88e6xxx_avb_read(cfg->sw, MV88E6XXX_AVB_PORT_TAI, TAI_CLOCK_PERIOD, &period_ps, 1);
	if (ret < 0 || period_ps == 0 || period_ps > 20000) {
		LOG_ERR("unexpected TAI clock period %u ps (%d)", period_ps, ret);
		return ret < 0 ? ret : -ENODEV;
	}

	ret = mv88e6xxx_avb_write(cfg->sw, MV88E6XXX_AVB_PORT_PTP, PTP_ETHERTYPE, ETH_P_1588);
	if (ret == 0) {
		ret = mv88e6xxx_avb_write(cfg->sw, MV88E6XXX_AVB_PORT_PTP, PTP_MSGTYPE,
					  PTP_MSGTYPE_ALL);
	}
	if (ret == 0) {
		ret = mv88e6xxx_avb_write(cfg->sw, MV88E6XXX_AVB_PORT_PTP, PTP_TS_ARRIVAL_PTR,
					  PTP_MSGTYPE_PDLY_RES);
	}
	if (ret == 0 && (product == 0x341 || product == 0x141)) {
		ret = mv88e6xxx_avb_write(cfg->sw, MV88E6XXX_AVB_PORT_PTP, PTP_6341_CFG,
					  PTP_6341_CFG_TS_MAC);
	}
	if (ret == 0) {
		ret = read_cyc(dev, &cyc);
	}
	if (ret < 0) {
		LOG_ERR("PTP setup failed (%d)", ret);
		return ret;
	}

	mv_tc_init(&data->tc, period_ps, cyc, 0);
	k_work_reschedule(&data->overflow, OVERFLOW_PERIOD);

	LOG_INF("%s: TAI %u ps per cycle", dev->name, period_ps);

	return 0;
}

static DEVICE_API(ptp_clock, ptp_mv88e6xxx_api) = {
	.set = ptp_mv88e6xxx_set,
	.get = ptp_mv88e6xxx_get,
	.adjust = ptp_mv88e6xxx_adjust,
	.rate_adjust = ptp_mv88e6xxx_rate_adjust,
};

#define PTP_MV88E6XXX_INIT(n)                                                                      \
	static const struct ptp_mv88e6xxx_config ptp_mv88e6xxx_config_##n = {                      \
		.sw = DEVICE_DT_GET(DT_INST_PARENT(n)),                                            \
	};                                                                                         \
	static struct ptp_mv88e6xxx_data ptp_mv88e6xxx_data_##n;                                   \
	DEVICE_DT_INST_DEFINE(n, ptp_mv88e6xxx_init, NULL, &ptp_mv88e6xxx_data_##n,                \
			      &ptp_mv88e6xxx_config_##n, POST_KERNEL,                              \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &ptp_mv88e6xxx_api);

DT_INST_FOREACH_STATUS_OKAY(PTP_MV88E6XXX_INIT)
