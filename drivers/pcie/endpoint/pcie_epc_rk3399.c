/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 PCIe endpoint controller on the EPC API (pcie_epc.h):
 * Cadence core behind the Rockchip client wrapper, PHY in the GRF. The
 * same controller is the root complex in the other role (pcie_rk3399.c,
 * role from zephyr/drivers/pcie/role.h); bring-up shared in
 * pcie_rk3399_port.c.
 *
 * Outbound: the 32 MiB "mem" window is 32 Cadence regions of 1 MiB;
 * region 0 is kept for MSI writes, the rest is the EPC core's window.
 * MSI-X: the core only holds the capability; the table is in BAR memory
 * and the message goes out through the outbound window like MSI. Linux
 * hides it on RK3399 (the TRM lists only MSI and INTx for endpoints);
 * PCIE_EPC_RK3399_MSIX offers it.
 */

#define DT_DRV_COMPAT rockchip_rk3399_pcie_ep

#include <errno.h>
#include <stdint.h>

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#if defined(CONFIG_PCIE_EPC_RK3399_DOORBELL)
#include <zephyr/drivers/interrupt_controller/gicv3_its.h>
#include <zephyr/irq.h>
#endif
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_ep_cdns.h>
#if defined(CONFIG_PCIE_ROLE)
#include <zephyr/drivers/pcie/role.h>
#endif
#if defined(CONFIG_PCIE_CTRL_SHELL)
#include <zephyr/drivers/pcie/pcie_shell.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/cache.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "pcie_ep_cdns.h"
#include "../pcie_rk3399_port.h"

LOG_MODULE_REGISTER(pcie_epc_rk3399, CONFIG_PCIE_EP_LOG_LEVEL);

#define PCI_COMMAND_REG          0x04U
#define  PCI_COMMAND_MASTER      BIT(2)
#define  PCI_COMMAND_INTX_DIS    BIT(10)
#define PCI_EXP_LNKCTL           0x10U
#define  PCI_EXP_LNKCTL_RL       BIT(5)
#define CLIENT_INT_HOT_RST       BIT(13)
/*
 * Client interrupts (status RK_CLIENT_INT_STATUS, write 1 to clear; mask
 * RK_CLIENT_INT_MASK, 1 = masked, high half write enable), on the "sys"
 * and "client" lines. INTA..D (8:5) are root complex only.
 */
#define CLIENT_INT_PHY           BIT(2)  /* link state change */
#define CLIENT_INT_UDMA          BIT(3)  /* uDMA done or error, detail in UDMA_INT */
#define CLIENT_INT_LOCAL         BIT(4)  /* core error, detail in LM_INT_STATUS */
#define CLIENT_INT_EVENTS        (BIT(0) | BIT(1) | CLIENT_INT_PHY | CLIENT_INT_LOCAL | BIT(9) | \
				  BIT(10) | BIT(11) | BIT(12) | CLIENT_INT_HOT_RST)
/* core error interrupts (LM): status write 1 to clear, mask 1 = masked */
#define LM_INT_STATUS            (CDNS_LM_BASE + 0x20cU)
#define LM_INT_MASK              (CDNS_LM_BASE + 0x210U)
#define LM_INT_ALL               (GENMASK(11, 0) | BIT(18) | BIT(19))

/*
 * uDMA (TRM part 2, 17.5.6 and 17.6.9): at apb + 0xe00000, channels 0x14
 * apart. A channel walks a linked list of descriptors in local memory;
 * each one moves up to 16 MiB between a local (AXI) and a PCIe address
 * in one direction, set per channel by CTRL ob_not_ib. The PCIe side
 * goes to the controller directly, not through an outbound region.
 */
#define UDMA_BASE                0xe00000U
#define UDMA_CH(c)               (UDMA_BASE + 0x14U * (c))
#define UDMA_CTRL(c)             (UDMA_CH(c) + 0x00U)
#define  UDMA_CTRL_GO            BIT(0)
#define  UDMA_CTRL_OB            BIT(1)  /* outbound: local -> PCIe */
#define UDMA_SP_L(c)             (UDMA_CH(c) + 0x04U)
#define UDMA_SP_U(c)             (UDMA_CH(c) + 0x08U)
#define UDMA_ATTR_L(c)           (UDMA_CH(c) + 0x0cU)
#define UDMA_ATTR_U(c)           (UDMA_CH(c) + 0x10U)
#define UDMA_INT                 (UDMA_BASE + 0xa0U) /* write the value back to clear */
#define UDMA_INT_ENA             (UDMA_BASE + 0xa4U)
#define UDMA_INT_DIS             (UDMA_BASE + 0xa8U)
#define  UDMA_INT_DONE(c)        BIT(c)
#define  UDMA_INT_ERR(c)         BIT(8 + (c))
#define UDMA_CAP                 (UDMA_BASE + 0xf8U)
#define UDMA_CONF                (UDMA_BASE + 0xfcU)
#define UDMA_CHAN                0U
#define UDMA_MAX_LEN             MB(8) /* per descriptor; the field holds up to 16 MiB - 1 */
#define UDMA_TIMEOUT_MS          1000
#define UDMA_ALIGN               8U /* PCIe address and length; local address */

/* descriptor control byte */
#define UDMA_DC_IRQ              BIT(0)
#define UDMA_DC_CONTINUE         BIT(5)
/* channel status byte */
#define UDMA_DS_DONE             BIT(0)
#define UDMA_DS_ERRORS           GENMASK(7, 1)

/* 32-bit AXI address and attributes, 64-bit PCIe address and attributes (CONF) */
struct udma_desc {
	uint32_t axi_addr;
	uint32_t axi_attr;
	uint32_t pci_addr_lo;
	uint32_t pci_addr_hi;
	uint32_t tlp_attr_lo;
	uint32_t tlp_attr_hi;
	uint32_t len_ctrl;  /* 23:0 length, 31:24 control */
	uint32_t status;    /* 7:0 AXI, 15:8 PCIe, 23:16 channel */
	uint32_t next;
} __packed;

#define UDMA_DESCS               2U

/* doorbell DeviceID candidates: 0-7 and bus 1-15 (device 0, function 0) */
#define DB_SLOTS (IS_ENABLED(CONFIG_PCIE_EPC_RK3399_DOORBELL) &&                                  \
		  CONFIG_PCIE_EPC_RK3399_DOORBELL_DEVID < 0 ? 23 : 1)
#define UDMA_DESC_ALIGN          64U

static const char *const client_int_name[16] = {
	[0] = "power-state", [1] = "hot-plug", [2] = "phy/link", [3] = "udma", [4] = "local",
	[9] = "correctable", [10] = "nonfatal", [11] = "fatal", [12] = "dpa",
	[13] = "hot-reset",
};

static const char *const lm_int_name[20] = {
	[0] = "pnp-fifo-parity", [1] = "cpl-fifo-parity", [2] = "replay-ram-parity",
	[3] = "pnp-fifo-overflow", [4] = "cpl-fifo-overflow", [5] = "replay-timeout",
	[6] = "replay-rollover", [7] = "phy-rx-error", [8] = "malformed-tlp",
	[9] = "unexpected-cpl", [10] = "fc-error", [11] = "cpl-timeout",
	[18] = "unmapped-tc", [19] = "msi-mask-change",
};

#define REBAR_FUNCS   8U /* physical functions tracked for the Resizable BAR capability */
#define OB_REGIONS    32U
#define OB_REGION     MB(1)
#define OB_IRQ_REGION 0U
#define POLL_DOWN_MS  5
#define POLL_UP_MS    20

struct epc_rk3399_config {
	DEVICE_MMIO_NAMED_ROM(apb);
	uintptr_t mem_base;
	size_t mem_size;
	struct rk3399_pcie_port_cfg port;
	struct gpio_dt_spec perst;
	struct gpio_dt_spec slot_power;
	struct pcie_epc_header hdr;
	bool has_ids;
	uint8_t max_functions;
	uint8_t max_vfs; /* SR-IOV VFs of function 0, routing functions 1..max_vfs */
	uint8_t msi_vectors;
	bool autostart;
	void (*irq_config)(void);
#if defined(CONFIG_PCIE_EPC_RK3399_DOORBELL)
	const struct device *its;
	uint32_t db_prio;
#endif
};

struct epc_rk3399_data {
	struct pcie_epc_data epc; /* must be first */
	DEVICE_MMIO_NAMED_RAM(apb);
	struct cdns_ep_core core;
	const struct device *dev;
	struct k_spinlock lock;
	uint32_t ob_map;
	uint8_t ob_count[OB_REGIONS];
	mm_reg_t irq_va;
	uint64_t irq_pci_addr;
	uint8_t irq_pci_fn;
	struct k_work_delayable link_work;
	struct gpio_callback perst_cb;
	struct pcie_epc_features features;
	struct pcie_epc_features vf_features; /* no INTx */
	bool active;
	/* between start and stop; set before the link work is first queued */
	bool running;
	bool perst_asserted;
	bool link_up;
	uint16_t bme; /* per routing function, PF and VFs */
	uint16_t vf_bar_users[6]; /* VFs holding the shared layout of each VF BAR */
	struct k_timer sriov_timer; /* ARI Capable Hierarchy for the VFs, see link_work */
	atomic_t arie_set;
	uint32_t ups, downs, hot_resets;
	/* interrupt events: counted in the ISR, logged and acted on in the link work */
	atomic_t int_pending; /* client status bits not yet seen by the link work */
	uint32_t client_cnt[16];
	uint32_t lm_cnt[20];
	uint32_t client_logged[16];
	uint32_t lm_logged[20];
	/* uDMA channel UDMA_CHAN, one transfer at a time */
	struct k_mutex udma_lock;
	struct k_sem udma_done;
	atomic_t udma_int; /* UDMA_INT bits seen by the ISR */
	struct {
		uint32_t n_to, n_from, n_err, n_timeout, n_polled;
		uint64_t bytes_to, bytes_from, cyc_to, cyc_from;
		uint32_t last_err; /* descriptor status word */
	} udma_stat;
#if defined(CONFIG_PCIE_EPC_RK3399_DOORBELL)
	/* doorbell: one LPI per candidate ITS DeviceID, mapped at the first allocation */
	struct db_slot {
		const struct device *dev;
		uint32_t devid;
		uint32_t intid;
		uint32_t hits;
	} db_slot[DB_SLOTS];
	uint8_t db_nslots;
	int32_t db_seen; /* DeviceID of the first doorbell, -1 none yet */
	pcie_epc_doorbell_cb_t db_cb;
	void *db_user;
	uint8_t db_func;
	int8_t db_cpu; /* CPU the LPIs are delivered to, -1 where mapped */
#endif
	/*
	 * Resizable BAR, per physical function: the memory BARs as set (all
	 * are listed in the capability once one is resizable) and the sizes
	 * last seen, to report a change by the host.
	 */
	struct {
		size_t size[6];     /* 0: not set or I/O */
		size_t max_size[6]; /* non-zero: resizable */
		uint8_t idx[6];     /* entry in the capability */
		uint8_t seen[6];    /* last size of a resizable BAR, as n (1 MiB << n) */
		uint8_t n;          /* entries programmed, 0 = capability off */
		uint8_t listed;     /* BARs in the programmed capability */
		void *virt[6];      /* CPU view of the BAR memory, for the MSI-X table */
	} rebar[REBAR_FUNCS];
	/* MSI-X per physical function: table in BAR memory, written by the host */
	struct {
		volatile uint32_t *tbl;
		uint16_t nr;
		uint32_t sent, masked;
	} msix[REBAR_FUNCS];
	int64_t last_up_log; /* uptime of the last "link up" INF line */
};

#define DEV_CFG(_dev)  ((const struct epc_rk3399_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct epc_rk3399_data *)(_dev)->data)

/* routing functions: the PF(s), then the SR-IOV VFs of function 0 */
static inline uint8_t nfuncs(const struct device *dev)
{
	return DEV_CFG(dev)->max_functions + DEV_CFG(dev)->max_vfs;
}

static inline bool is_vf(const struct device *dev, uint8_t func)
{
	return func >= DEV_CFG(dev)->max_functions && func < nfuncs(dev);
}

static const struct cdns_ep_variant rk3399_cdns_variant = {
	.ob_region_offset = 0x20U,
	.ib_bar_offset = 0x828U,
	.ob_max_nbits = 20U,
	.ob_has_cpu_addr = false,
	.did_via_rp_bank = true,
};

static inline mm_reg_t apb(const struct device *dev)
{
	return DEVICE_MMIO_NAMED_GET(dev, apb);
}

/* uDMA registers, at UDMA_BASE in the apb window (outside the core) */
static inline uint32_t udma_rd(const struct device *dev, uint32_t reg)
{
	return sys_read32(apb(dev) + reg);
}

static inline void udma_wr(const struct device *dev, uint32_t reg, uint32_t val)
{
	sys_write32(val, apb(dev) + reg);
}

static bool active_role(void)
{
#if defined(CONFIG_PCIE_ROLE)
	return pcie_role_get() == PCIE_ROLE_EP;
#else
	return true;
#endif
}

/*
 * Rockchip tuning of the Cadence core that Linux applies on the RK3399 (as
 * root complex): the FTS count to leave L0s at its maximum (no common
 * reference clock on a crossover link), and the Tx credit maximum update
 * interval (Linux also rewrites it on every PHY link interrupt, "need this
 * to enter L1 again"). Both once, before the link trains: rewriting them
 * with the link up left the endpoint without memory read completions.
 */
#define LM_PLC1                 (CDNS_LM_BASE + 0x004U)
#define  LM_PLC1_FTS            GENMASK(23, 8)
#define LM_TXCREDIT_CFG1        (CDNS_LM_BASE + 0x020U)
#define  LM_TXCREDIT_CFG1_MUI   GENMASK(31, 16)
#define TXCREDIT_MUI_NS         24000U

static void core_tune(struct epc_rk3399_data *d)
{
	uint32_t v = cdns_ep_read(&d->core, LM_PLC1);

	cdns_ep_write(&d->core, LM_PLC1, (v & ~LM_PLC1_FTS) | FIELD_PREP(LM_PLC1_FTS, 0xffffU));
	v = cdns_ep_read(&d->core, LM_TXCREDIT_CFG1);
	cdns_ep_write(&d->core, LM_TXCREDIT_CFG1, (v & ~LM_TXCREDIT_CFG1_MUI) |
		      FIELD_PREP(LM_TXCREDIT_CFG1_MUI, TXCREDIT_MUI_NS >> 3));
}

/* --- link supervision ------------------------------------------------------ */

/*
 * The configuration a hot reset or link down clears may include the
 * outbound translation (the TRM does not say): program the MSI region
 * again on the next interrupt instead of trusting the cached address.
 */
static void irq_region_forget(struct epc_rk3399_data *d)
{
	k_spinlock_key_t key = k_spin_lock(&d->lock);

	d->irq_pci_addr = UINT64_MAX;
	k_spin_unlock(&d->lock, key);
}

static void link_down(const struct device *dev)
{
	struct epc_rk3399_data *d = dev->data;

	irq_region_forget(d);
	if (d->link_up) {
		d->link_up = false;
		d->bme = 0;
		d->downs++;
		pcie_epc_notify(dev, PCIE_EPC_EVT_LINK_DOWN, 0);
	}
}

/* 1 ms, ISR context: VF Enable without ARI Capable Hierarchy -> set it */
static void sriov_timer_fn(struct k_timer *t)
{
	struct epc_rk3399_data *d = CONTAINER_OF(t, struct epc_rk3399_data, sriov_timer);
	uint32_t reg = CDNS_EP_FUNC_SRIOV_CAP + CDNS_SRIOV_CTRL;
	uint32_t ctl = cdns_ep_read(&d->core, reg);

	if ((ctl & CDNS_SRIOV_CTRL_VFE) != 0U && (ctl & CDNS_SRIOV_CTRL_ARIE) == 0U) {
		cdns_ep_write(&d->core, reg, ctl | CDNS_SRIOV_CTRL_ARIE);
		atomic_set(&d->arie_set, 1);
	}
}

static void epc_rk3399_isr(const struct device *dev)
{
	struct epc_rk3399_data *d = dev->data;
	uint32_t ist = rk_client_read(apb(dev), RK_CLIENT_INT_STATUS) &
		       (CLIENT_INT_EVENTS | CLIENT_INT_UDMA);

	if ((ist & CLIENT_INT_UDMA) != 0U) {
		uint32_t ust = udma_rd(dev, UDMA_INT);

		udma_wr(dev, UDMA_INT, ust);
		if (ust != 0U) {
			(void)atomic_or(&d->udma_int, (atomic_val_t)ust);
			k_sem_give(&d->udma_done);
		}
	}

	if ((ist & CLIENT_INT_LOCAL) != 0U) {
		uint32_t lst = cdns_ep_read(&d->core, LM_INT_STATUS) & LM_INT_ALL;

		cdns_ep_write(&d->core, LM_INT_STATUS, lst);
		for (unsigned int i = 0; i < ARRAY_SIZE(d->lm_cnt); i++) {
			if ((lst & BIT(i)) != 0U) {
				d->lm_cnt[i]++;
			}
		}
	}
	rk_client_write(apb(dev), RK_CLIENT_INT_STATUS, ist);
	for (unsigned int i = 0; i < ARRAY_SIZE(d->client_cnt); i++) {
		if ((ist & BIT(i)) != 0U) {
			d->client_cnt[i]++;
		}
	}
	ist &= CLIENT_INT_EVENTS;
	if (ist == 0U) {
		return;
	}
	(void)atomic_or(&d->int_pending, (atomic_val_t)ist);
	/* link change, hot reset or error: let the link work look now, not in 20 ms */
	if (d->running) {
		(void)k_work_reschedule(&d->link_work, K_NO_WAIT);
	}
}

/*
 * One line for the interrupt events since the last one. uDMA completion is
 * the normal end of a transfer and the local interrupt is itemized in the
 * LM sources below, so neither is listed; both stay counted in the info.
 */
#define INT_LOG_SKIP (CLIENT_INT_UDMA | CLIENT_INT_LOCAL)

static void int_log(struct epc_rk3399_data *d)
{
	char buf[160];
	size_t o = 0;

	for (unsigned int i = 0; i < ARRAY_SIZE(d->client_cnt); i++) {
		uint32_t n = d->client_cnt[i] - d->client_logged[i];

		if (n != 0U && (INT_LOG_SKIP & BIT(i)) == 0U && client_int_name[i] != NULL &&
		    o < sizeof(buf)) {
			o += snprintf(buf + o, sizeof(buf) - o, " %s %u", client_int_name[i], n);
		}
		d->client_logged[i] += n;
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(d->lm_cnt); i++) {
		uint32_t n = d->lm_cnt[i] - d->lm_logged[i];

		if (n != 0U && lm_int_name[i] != NULL && o < sizeof(buf)) {
			o += snprintf(buf + o, sizeof(buf) - o, " %s %u", lm_int_name[i], n);
		}
		d->lm_logged[i] += n;
	}
	if (o != 0U) {
		LOG_WRN("%s: irq:%s", d->dev->name, buf);
	}
}

static int rebar_update(const struct device *dev, uint8_t func);
static void rebar_poll(const struct device *dev);

static void link_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct epc_rk3399_data *d = CONTAINER_OF(dwork, struct epc_rk3399_data, link_work);
	const struct device *dev = d->dev;
	uint32_t ist;

	if (d->perst_asserted || !d->running) {
		return;
	}

	ist = (uint32_t)atomic_clear(&d->int_pending) |
	      rk_client_read(apb(dev), RK_CLIENT_INT_STATUS);
	int_log(d);
	rebar_poll(dev);
	if (ist & CLIENT_INT_HOT_RST) {
		rk_client_write(apb(dev), RK_CLIENT_INT_STATUS, CLIENT_INT_HOT_RST);
		d->hot_resets++;
		cdns_ep_write(&d->core, CDNS_AT_LINKDOWN, 0U);
		irq_region_forget(d);
		pcie_epc_notify(dev, PCIE_EPC_EVT_HOT_RESET, 0);
	}

	if (!rk3399_pcie_link_up(apb(dev)) ||
	    (cdns_ep_read(&d->core, CDNS_LM_PL_CONFIG0) & CDNS_LM_PL_CONFIG0_LS_READY) == 0U) {
		/*
		 * A data link down resets the function's configuration space
		 * (BARs, Command, SR-IOV Control): always a real link down.
		 */
		link_down(dev);
		(void)k_work_reschedule(&d->link_work, K_MSEC(POLL_DOWN_MS));
		return;
	}

	if (!d->link_up) {
		unsigned int gen, width;

		rk3399_pcie_link_info(apb(dev), &gen, &width);
		/* a flapping link would flood the console: one line per 10 s at most */
		if (d->last_up_log == 0 || k_uptime_get() - d->last_up_log > 10000) {
			LOG_INF("%s: link up, Gen%u x%u (ups %u downs %u)", dev->name, gen,
				width, d->ups + 1U, d->downs);
			d->last_up_log = k_uptime_get();
		} else {
			LOG_DBG("%s: link up, Gen%u x%u", dev->name, gen, width);
		}
		/* the outbound window answers SLVERR until this is cleared */
		cdns_ep_write(&d->core, CDNS_AT_LINKDOWN, 0U);
		d->link_up = true;
		d->ups++;
		pcie_epc_notify(dev, PCIE_EPC_EVT_LINK_UP, 0);
	}

	/*
	 * The core needs ARI Capable Hierarchy whenever VF Enable is set (TRM,
	 * SR-IOV Control), but a host whose root port lacks ARI forwarding
	 * never sets it: the VFs then answer config requests with garbage.
	 * For VFs 1..7 the ARI and legacy routing IDs are the same (device 0),
	 * so sriov_timer sets it from the local side, within the 100 ms Linux
	 * waits after VF Enable before it scans the VFs; this only reports it.
	 */
	if (atomic_cas(&d->arie_set, 1, 0)) {
		LOG_INF("%s: VFs enabled (%u), ARI Capable Hierarchy set locally", dev->name,
				cdns_ep_read(&d->core, CDNS_EP_FUNC_SRIOV_CAP + CDNS_SRIOV_NUM_VFS) &
					0xffffU);
	}

	/* Bus Master Enable of every enabled function */
	for (uint8_t fn = 0; fn < nfuncs(dev); fn++) {
		bool on;

		if ((d->epc.func_map & BIT(fn)) == 0U && fn != 0U) {
			continue;
		}
		on = (cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn) + PCI_COMMAND_REG) &
		      PCI_COMMAND_MASTER) != 0U;
		if (on != ((d->bme & BIT(fn)) != 0U)) {
			WRITE_BIT(d->bme, fn, on);
			pcie_epc_notify(dev, on ? PCIE_EPC_EVT_BME : PCIE_EPC_EVT_BME_CLEARED, fn);
		}
	}
	(void)k_work_reschedule(&d->link_work, K_MSEC(POLL_UP_MS));
}

static void retrain(struct epc_rk3399_data *d)
{
	uint32_t reg = CDNS_RP_BASE + CDNS_EP_FUNC_PCIE_CAP + PCI_EXP_LNKCTL;

	cdns_ep_write(&d->core, reg, cdns_ep_read(&d->core, reg) | PCI_EXP_LNKCTL_RL);
}

/* Rockchip GPIO v1 has no both-edge trigger: arm the opposite edge each time. */
static void perst_arm(const struct epc_rk3399_config *cfg, bool asserted)
{
	(void)gpio_pin_interrupt_configure_dt(&cfg->perst, asserted ? GPIO_INT_EDGE_TO_INACTIVE
								  : GPIO_INT_EDGE_TO_ACTIVE);
}

static void perst_handler(const struct device *port, struct gpio_callback *cb,
			  gpio_port_pins_t pins)
{
	struct epc_rk3399_data *d = CONTAINER_OF(cb, struct epc_rk3399_data, perst_cb);
	const struct device *dev = d->dev;
	const struct epc_rk3399_config *cfg = dev->config;
	int asserted = gpio_pin_get_dt(&cfg->perst);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	if (asserted < 0) {
		return;
	}
	perst_arm(cfg, asserted != 0);

	if (asserted != 0) {
		if (!d->perst_asserted) {
			d->perst_asserted = true;
			(void)k_work_cancel_delayable(&d->link_work);
			link_down(dev);
			pcie_epc_notify(dev, PCIE_EPC_EVT_PERST_ASSERT, 0);
		}
		return;
	}
	if (d->perst_asserted) {
		d->perst_asserted = false;
		pcie_epc_notify(dev, PCIE_EPC_EVT_PERST_DEASSERT, 0);
		if (d->running) {
			retrain(d);
			(void)k_work_reschedule(&d->link_work, K_NO_WAIT);
		}
	}
}

/* --- EPC operations -------------------------------------------------------- */

static int epc_start(const struct device *dev)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	int ret;

	if (!d->active) {
		return -EBUSY;
	}
	cdns_ep_write(&d->core, CDNS_LM_EP_FUNC_CFG,
		      (d->epc.func_map | BIT(0)) & BIT_MASK(cfg->max_functions));
	rk_client_write(apb(dev), RK_CLIENT_CONFIG,
			RK_WM16(RK_CLIENT_LINK_TRAIN, 1U) | RK_WM16(RK_CLIENT_CONF_ENABLE, 1U));
	/*
	 * Not epc.started: the core sets that after start() returns, and the
	 * link work queued below can run first (system work queue preempts the
	 * caller), return on it and never be queued again.
	 */
	d->running = true;

	if (cfg->max_vfs != 0U) {
		k_timer_start(&d->sriov_timer, K_MSEC(1), K_MSEC(1));
	}
	/* events as interrupts; the link work's polling stays as the fallback */
	cdns_ep_write(&d->core, LM_INT_STATUS, LM_INT_ALL);
	cdns_ep_write(&d->core, LM_INT_MASK, (uint32_t)~LM_INT_ALL);
	rk_client_write(apb(dev), RK_CLIENT_INT_STATUS, CLIENT_INT_EVENTS);
	udma_wr(dev, UDMA_INT_ENA, UDMA_INT_DONE(UDMA_CHAN) | UDMA_INT_ERR(UDMA_CHAN));
	rk_client_write(apb(dev), RK_CLIENT_INT_MASK, (CLIENT_INT_EVENTS | CLIENT_INT_UDMA) << 16);
	if (cfg->perst.port != NULL) {
		ret = gpio_pin_get_dt(&cfg->perst);
		if (ret < 0) {
			return ret;
		}
		d->perst_asserted = (ret != 0);
		perst_arm(cfg, d->perst_asserted);
		if (d->perst_asserted) {
			LOG_INF("%s: waiting for PERST# deassertion", dev->name);
			return 0;
		}
	}
	(void)k_work_reschedule(&d->link_work, K_NO_WAIT);
	return 0;
}

static void epc_stop(const struct device *dev)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;

	if (!d->active) {
		return;
	}
	if (cfg->perst.port != NULL) {
		(void)gpio_pin_interrupt_configure_dt(&cfg->perst, GPIO_INT_DISABLE);
	}
	d->running = false;
	rk_client_write(apb(dev), RK_CLIENT_INT_MASK,
			((CLIENT_INT_EVENTS | CLIENT_INT_UDMA) << 16) | CLIENT_INT_EVENTS |
			CLIENT_INT_UDMA);
	cdns_ep_write(&d->core, LM_INT_MASK, LM_INT_ALL);
	k_timer_stop(&d->sriov_timer);
	(void)k_work_cancel_delayable(&d->link_work);
	link_down(dev);
	rk_client_write(apb(dev), RK_CLIENT_CONFIG,
			RK_WM16(RK_CLIENT_LINK_TRAIN, 0U) | RK_WM16(RK_CLIENT_CONF_ENABLE, 0U));
}

static int epc_write_header(const struct device *dev, uint8_t func,
			    const struct pcie_epc_header *hdr)
{
	struct epc_rk3399_data *d = dev->data;
	struct pcie_ep_cdns_header h = {
		.vendor_id = hdr->vendor_id,
		.device_id = hdr->device_id,
		.revision_id = hdr->revision_id,
		.class_code = ((uint32_t)hdr->baseclass << 16) | ((uint32_t)hdr->subclass << 8) |
			      hdr->prog_if,
		.subsys_vendor_id = hdr->subsys_vendor_id,
		.subsys_id = hdr->subsys_id,
		.interrupt_pin = hdr->interrupt_pin,
	};

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	if (is_vf(dev, func)) {
		uint32_t base = CDNS_EP_FUNC_BASE(func);

		/*
		 * A VF reads Vendor ID 0xffff and the VF Device ID of its PF
		 * (one for all VFs); the rest of the header is per VF.
		 */
		cdns_ep_sriov_set_vf_did(&d->core, 0, hdr->device_id);
		cdns_ep_write(&d->core, base + 0x08U,
			      (uint32_t)h.revision_id | ((h.class_code & 0xffffffU) << 8));
		cdns_ep_write(&d->core, base + 0x2cU,
			      (uint32_t)h.subsys_vendor_id | ((uint32_t)h.subsys_id << 16));
		return 0;
	}
	return cdns_ep_write_header(&d->core, func, &h);
}

static int epc_set_bar(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar)
{
	struct epc_rk3399_data *d = dev->data;
	uint32_t flags = 0;
	int ret;

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	if (bar->flags & PCIE_EPC_BAR_IO) {
		flags |= PCIE_EP_CDNS_BAR_IO;
	}
	if (bar->flags & PCIE_EPC_BAR_MEM64) {
		flags |= PCIE_EP_CDNS_BAR_64BIT;
	}
	if (bar->flags & PCIE_EPC_BAR_PREFETCH) {
		flags |= PCIE_EP_CDNS_BAR_PREFETCH;
	}
	if (is_vf(dev, func)) {
		ret = cdns_ep_set_vf_bar(&d->core, 0, func, bar->barno, (uintptr_t)bar->phys,
					     bar->size, flags);

		if (ret == 0) {
			d->vf_bar_users[bar->barno] |= BIT(func);
		}
		return ret;
	}
	if ((bar->flags & PCIE_EPC_BAR_RESIZABLE) != 0U &&
	    (func >= REBAR_FUNCS || (bar->flags & PCIE_EPC_BAR_IO) != 0U ||
	     (bar->phys & (MAX(bar->max_size, bar->size) - 1U)) != 0U)) {
		return -EINVAL;
	}
	ret = cdns_ep_set_bar(&d->core, func, bar->barno, (uintptr_t)bar->phys, bar->size, flags);
	if (ret != 0 || func >= REBAR_FUNCS || bar->barno >= 6U) {
		return ret;
	}
	if (d->rebar[func].size[bar->barno] != bar->size) {
		d->rebar[func].listed &= ~BIT(bar->barno); /* new layout: initial size again */
	}
	d->rebar[func].size[bar->barno] = (bar->flags & PCIE_EPC_BAR_IO) != 0U ? 0U : bar->size;
	d->rebar[func].virt[bar->barno] = bar->virt;
	d->rebar[func].max_size[bar->barno] =
		(bar->flags & PCIE_EPC_BAR_RESIZABLE) != 0U ? bar->max_size : 0U;
	if ((bar->flags & PCIE_EPC_BAR_RESIZABLE) == 0U && d->rebar[func].n == 0U) {
		return 0; /* no Resizable BAR capability involved */
	}
	ret = rebar_update(dev, func);
	if (ret != 0) {
		d->rebar[func].size[bar->barno] = 0U;
		d->rebar[func].max_size[bar->barno] = 0U;
		(void)rebar_update(dev, func);
	}
	return ret;
}

static uint8_t log2_mib(size_t size)
{
	return (uint8_t)(LOG2(size) - 20U);
}

/*
 * (Re)write the Resizable BAR capability of a physical function: off
 * without a resizable BAR, else every memory BAR of the function listed,
 * a fixed one with its own size as the only choice.
 */
static int rebar_update(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;
	struct cdns_ep_rebar ent[CDNS_EP_REBAR_MAX];
	bool any = false;
	uint8_t n = 0;

	/* keep what the host chose: rewriting the capability must not undo a resize */
	for (uint8_t b = 0; b < 6U && d->rebar[func].n != 0U; b++) {
		if (d->rebar[func].max_size[b] != 0U && (d->rebar[func].listed & BIT(b)) != 0U) {
			d->rebar[func].seen[b] =
				cdns_ep_rebar_size(&d->core, func, d->rebar[func].idx[b]);
		}
	}
	for (uint8_t b = 0; b < 6U; b++) {
		any = any || d->rebar[func].max_size[b] != 0U;
	}
	if (any) {
		for (uint8_t b = 0; b < 6U; b++) {
			size_t sz = d->rebar[func].size[b];
			size_t max = MAX(d->rebar[func].max_size[b], sz);

			if (sz == 0U) {
				continue;
			}
			if (sz < d->features.rebar_min_size || !IS_POWER_OF_TWO(sz) ||
			    !IS_POWER_OF_TWO(max)) {
				LOG_ERR("fn%u BAR%u: 0x%zx, a function with a resizable BAR needs "
					"powers of two from 1 MiB", func, b, sz);
				return -EINVAL;
			}
			if (d->rebar[func].max_size[b] == 0U ||
			    (d->rebar[func].listed & BIT(b)) == 0U) {
				d->rebar[func].seen[b] = log2_mib(sz);
			}
			ent[n] = (struct cdns_ep_rebar){
				.bar = b,
				.sizes = (uint32_t)GENMASK(log2_mib(max), log2_mib(sz)),
				.size = d->rebar[func].seen[b],
			};
			d->rebar[func].idx[b] = n;
			n++;
		}
	}
	d->rebar[func].listed = 0U;
	for (uint8_t i = 0; i < n; i++) {
		d->rebar[func].listed |= BIT(ent[i].bar);
	}
	d->rebar[func].n = n;
	return cdns_ep_rebar_program(&d->core, func, ent, n);
}

/* link work: report a resizable BAR the host gave another size */
static void rebar_poll(const struct device *dev)
{
	struct epc_rk3399_data *d = dev->data;

	for (uint8_t fn = 0; fn < MIN(DEV_CFG(dev)->max_functions, REBAR_FUNCS); fn++) {
		if (d->rebar[fn].n == 0U) {
			continue;
		}
		for (uint8_t b = 0; b < 6U; b++) {
			uint8_t now;

			if (d->rebar[fn].max_size[b] == 0U) {
				continue;
			}
			now = cdns_ep_rebar_size(&d->core, fn, d->rebar[fn].idx[b]);
			if (now != d->rebar[fn].seen[b]) {
				LOG_INF("fn%u BAR%u resized by the host: %u -> %u MiB", fn, b,
					1U << d->rebar[fn].seen[b], 1U << now);
				d->rebar[fn].seen[b] = now;
				pcie_epc_notify(dev, PCIE_EPC_EVT_BAR_RESIZED, fn);
			}
		}
	}
}

static int epc_get_bar_size(const struct device *dev, uint8_t func, uint8_t barno, size_t *size)
{
	struct epc_rk3399_data *d = dev->data;

	if (is_vf(dev, func) || func >= REBAR_FUNCS || barno >= 6U ||
	    d->rebar[func].size[barno] == 0U) {
		return -EINVAL;
	}
	*size = d->rebar[func].max_size[barno] != 0U
			? MB(1) << cdns_ep_rebar_size(&d->core, func, d->rebar[func].idx[barno])
			: d->rebar[func].size[barno];
	return 0;
}

static void epc_clear_bar(const struct device *dev, uint8_t func, uint8_t barno)
{
	struct epc_rk3399_data *d = dev->data;

	if (is_vf(dev, func) && barno < 6U) {
		d->vf_bar_users[barno] &= ~BIT(func);
		(void)cdns_ep_clear_vf_bar(&d->core, 0, func, barno, d->vf_bar_users[barno] == 0U);
		return;
	}
	if (func < nfuncs(dev)) {
		(void)cdns_ep_clear_bar(&d->core, func, barno);
	}
	if (func < REBAR_FUNCS && barno < 6U && d->rebar[func].size[barno] != 0U) {
		d->rebar[func].size[barno] = 0U;
		d->rebar[func].max_size[barno] = 0U;
		(void)rebar_update(dev, func);
	}
}

/* cpu_addr: physical address in the window; up to 1 MiB per region */
/*
 * The link work clears the AXI link-down status within a poll period of
 * the link coming back; an access that comes sooner (host restores the
 * config and sends a command right after a hot reset) clears it here.
 */
static void ob_unblock(struct epc_rk3399_data *d)
{
	if (cdns_ep_read(&d->core, CDNS_AT_LINKDOWN) != 0U) {
		cdns_ep_write(&d->core, CDNS_AT_LINKDOWN, 0U);
	}
}

static int epc_map_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr,
			uint64_t pci_addr, size_t size)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	uint32_t first, n;
	k_spinlock_key_t key;

	if (cpu_addr < cfg->mem_base + OB_REGION || size == 0 ||
	    cpu_addr + size > cfg->mem_base + cfg->mem_size) {
		return -EINVAL;
	}
	first = (cpu_addr - cfg->mem_base) / OB_REGION;
	n = DIV_ROUND_UP((cpu_addr - cfg->mem_base) % OB_REGION + size, OB_REGION);
	if (size < OB_REGION && ((cpu_addr | pci_addr) & (size - 1)) != 0) {
		return -EINVAL; /* translation replaces the low bits */
	}

	ob_unblock(d);
	key = k_spin_lock(&d->lock);
	for (uint32_t i = 0; i < n; i++) {
		uint64_t chunk = MIN((uint64_t)size, (uint64_t)OB_REGION);
		uint8_t nbits = cdns_ep_ob_nbits(&d->core, pci_addr + i * OB_REGION, chunk);

		cdns_ep_ob_program(&d->core, func, first + i,
				   cfg->mem_base + (first + i) * OB_REGION,
				   pci_addr + (uint64_t)i * OB_REGION, nbits);
		d->ob_map |= BIT(first + i);
	}
	d->ob_count[first] = (uint8_t)n;
	k_spin_unlock(&d->lock, key);
	return 0;
}

static void epc_unmap_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	k_spinlock_key_t key;
	uint32_t first;

	ARG_UNUSED(func);
	if (cpu_addr < cfg->mem_base + OB_REGION || cpu_addr >= cfg->mem_base + cfg->mem_size) {
		return;
	}
	/*
	 * The window is Normal-NC: writes through it may still sit in the CPU
	 * write buffer. Drain them before the region goes, or they reach the
	 * core with no translation.
	 */
	barrier_dsync_fence_full();
	first = (cpu_addr - cfg->mem_base) / OB_REGION;
	key = k_spin_lock(&d->lock);
	for (uint32_t i = 0; i < MAX(d->ob_count[first], 1U) && first + i < OB_REGIONS; i++) {
		cdns_ep_ob_clear(&d->core, first + i);
		d->ob_map &= ~BIT(first + i);
	}
	d->ob_count[first] = 0;
	k_spin_unlock(&d->lock, key);
}

static int epc_set_msi(const struct device *dev, uint8_t func, uint8_t nr_irqs)
{
	struct epc_rk3399_data *d = dev->data;

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	return cdns_ep_set_msi(&d->core, func, nr_irqs);
}

static int epc_get_msi(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;
	uint32_t ctrl;

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	ctrl = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + CDNS_EP_FUNC_MSI_CAP);
	if ((ctrl & CDNS_MSI_CTRL_ME) == 0U) {
		return -EINVAL;
	}
	return 1 << ((ctrl & CDNS_MSI_CTRL_MME_MASK) >> CDNS_MSI_CTRL_MME_SHIFT);
}

static int raise_intx(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;

	if (is_vf(dev, func)) {
		return -ENOTSUP; /* VFs have no INTx (SR-IOV) */
	}
	if (cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + PCI_COMMAND_REG) &
	    PCI_COMMAND_INTX_DIS) {
		return -EINVAL;
	}
	/* The TRM asks for some AHB clock cycles between edges; 1 ms is ample. */
	rk_client_write(apb(dev), RK_CLIENT_LEGACY_INT_CTRL,
			RK_WM16(RK_CLIENT_INT_IN, 1U) | RK_WM16(RK_CLIENT_INT_PEND_ST, 1U));
	k_busy_wait(1000);
	rk_client_write(apb(dev), RK_CLIENT_LEGACY_INT_CTRL,
			RK_WM16(RK_CLIENT_INT_IN, 0U) | RK_WM16(RK_CLIENT_INT_PEND_ST, 0U));
	return 0;
}

/* one interrupt message: a DWORD write through outbound region 0 */
static int irq_send(const struct device *dev, uint8_t func, uint16_t irq_num, uint64_t pci_addr,
		    uint32_t data)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	k_spinlock_key_t key;
	uint64_t base = pci_addr & ~(uint64_t)0xffU;

	barrier_dsync_fence_full(); /* data written through the window goes first */
	ob_unblock(d);
	key = k_spin_lock(&d->lock);
	if (d->irq_pci_addr != base || d->irq_pci_fn != func) {
		cdns_ep_ob_program(&d->core, func, OB_IRQ_REGION, cfg->mem_base, base, 8U);
		d->irq_pci_addr = base;
		d->irq_pci_fn = func;
	}
	if (IS_ENABLED(CONFIG_PCIE_EPC_RK3399_MSI_TRACE)) {
		/* the state an MSI write meets, to compare one that faults with one that works */
		LOG_INF("msi fn%u #%u -> 0x%llx/0x%04x: ltssm 0x%02x st0 %08x st1 %08x pl0 %08x "
			"cmd %08x ob %08x/%08x int %08x linkdown %u", func, irq_num,
			(unsigned long long)pci_addr, data, rk3399_pcie_ltssm(apb(dev)),
			rk_client_read(apb(dev), RK_CLIENT_BASIC_STATUS0),
			rk_client_read(apb(dev), RK_CLIENT_BASIC_STATUS1),
			cdns_ep_read(&d->core, CDNS_LM_PL_CONFIG0),
			cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + PCI_COMMAND_REG),
			cdns_ep_read(&d->core, CDNS_AT_OB_PCI_ADDR0(0x20U, OB_IRQ_REGION)),
			cdns_ep_read(&d->core, CDNS_AT_OB_DESC0(0x20U, OB_IRQ_REGION)),
			rk_client_read(apb(dev), RK_CLIENT_INT_STATUS),
			cdns_ep_read(&d->core, CDNS_AT_LINKDOWN));
		LOG_PANIC();
	}
	/*
	 * One DWORD with the message data in the low half, as an MSI write is
	 * on the link: the outbound AXI slave answers a 16-bit write to the
	 * Device mapped window with SLVERR (SError).
	 */
	sys_write32(data, d->irq_va + (mm_reg_t)(pci_addr & 0xfcU));
	barrier_dsync_fence_full(); /* push the MSI write out */
	k_spin_unlock(&d->lock, key);
	return 0;
}

static int raise_msi(const struct device *dev, uint8_t func, uint16_t irq_num)
{
	struct epc_rk3399_data *d = dev->data;
	uint64_t pci_addr;
	uint16_t data;
	int ret;

	ret = cdns_ep_msi_target(&d->core, func, irq_num, &pci_addr, &data);
	if (ret != 0) {
		return ret;
	}
	return irq_send(dev, func, irq_num, pci_addr, data);
}

/*
 * MSI-X vector irq_num (from 1): address and data from the table the host
 * wrote into BAR memory. A masked vector (function mask, or bit 0 of the
 * vector control) only sets its pending bit; nothing resends it on unmask.
 */
static int raise_msix(const struct device *dev, uint8_t func, uint16_t irq_num)
{
	struct epc_rk3399_data *d = dev->data;
	volatile uint32_t *e;
	uint32_t ctrl;
	uint64_t addr;
	uint16_t v = irq_num - 1U;

	if (func >= REBAR_FUNCS || d->msix[func].tbl == NULL || irq_num == 0U ||
	    irq_num > d->msix[func].nr) {
		return -EINVAL;
	}
	ctrl = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + CDNS_EP_FUNC_MSIX_CAP);
	if ((ctrl & CDNS_MSIX_CTRL_EN) == 0U) {
		return -EINVAL;
	}
	e = &d->msix[func].tbl[v * 4U];
	if ((ctrl & CDNS_MSIX_CTRL_FMASK) != 0U || (e[3] & BIT(0)) != 0U) {
		d->msix[func].tbl[d->msix[func].nr * 4U + v / 32U] |= BIT(v % 32U);
		d->msix[func].masked++;
		return 0;
	}
	addr = ((uint64_t)e[1] << 32) | e[0];
	if (addr == 0U || (addr & 3U) != 0U) {
		return -EINVAL;
	}
	if (d->msix[func].sent++ == 0U) {
		LOG_INF("fn%u MSI-X %u: 0x%08x -> 0x%llx (first one)", func, irq_num, e[2],
			(unsigned long long)addr);
	}
	return irq_send(dev, func, irq_num, addr, e[2]);
}

static int epc_set_msix(const struct device *dev, uint8_t func, uint16_t nr_irqs, uint8_t bir,
			uint32_t offset)
{
	struct epc_rk3399_data *d = dev->data;
	size_t len = nr_irqs * 16U + DIV_ROUND_UP(nr_irqs, 64U) * 8U; /* table, QWORD PBA */
	volatile uint32_t *tbl;
	int ret;

	if (!IS_ENABLED(CONFIG_PCIE_EPC_RK3399_MSIX) || is_vf(dev, func)) {
		return -ENOTSUP;
	}
	if (func >= REBAR_FUNCS || bir >= 6U || d->rebar[func].virt[bir] == NULL ||
	    offset + len > d->rebar[func].size[bir]) {
		return -EINVAL;
	}
	ret = cdns_ep_set_msix(&d->core, func, nr_irqs, bir, offset);
	if (ret != 0) {
		return ret;
	}
	/* reset state: every vector masked, nothing pending */
	tbl = (volatile uint32_t *)((uint8_t *)d->rebar[func].virt[bir] + offset);
	for (uint16_t v = 0; v < nr_irqs; v++) {
		tbl[v * 4U + 0U] = 0U;
		tbl[v * 4U + 1U] = 0U;
		tbl[v * 4U + 2U] = 0U;
		tbl[v * 4U + 3U] = 1U;
	}
	for (uint16_t w = 0; w < DIV_ROUND_UP(nr_irqs, 64U) * 2U; w++) {
		tbl[nr_irqs * 4U + w] = 0U;
	}
	d->msix[func].tbl = tbl;
	d->msix[func].nr = nr_irqs;
	return 0;
}

static int epc_get_msix(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;
	uint32_t ctrl;

	if (func >= REBAR_FUNCS || d->msix[func].tbl == NULL) {
		return -EINVAL;
	}
	ctrl = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + CDNS_EP_FUNC_MSIX_CAP);
	if ((ctrl & CDNS_MSIX_CTRL_EN) == 0U) {
		return -EINVAL;
	}
	return (int)FIELD_GET(CDNS_MSIX_CTRL_TSIZE, ctrl) + 1;
}

static int epc_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
			 uint16_t irq_num)
{
	struct epc_rk3399_data *d = dev->data;

	if (!d->link_up) {
		return -ENOTCONN;
	}
	if ((type == PCIE_EPC_IRQ_MSI || type == PCIE_EPC_IRQ_MSIX) && !(d->bme & BIT(func)) &&
	    (cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + PCI_COMMAND_REG) &
	     PCI_COMMAND_MASTER) == 0U) {
		/*
		 * The cached bit lags the link work by up to a poll period; a
		 * function that saw BME in its own Command register raises
		 * right away. Only a clear register refuses.
		 */
		return -EACCES; /* no Bus Master Enable: the write would be dropped */
	}
	switch (type) {
	case PCIE_EPC_IRQ_INTX:
		return raise_intx(dev, func);
	case PCIE_EPC_IRQ_MSI:
		return raise_msi(dev, func, irq_num);
	case PCIE_EPC_IRQ_MSIX:
		return raise_msix(dev, func, irq_num);
	default:
		return -ENOTSUP;
	}
}

static int epc_cfg_read(const struct device *dev, uint8_t func, uint32_t offset, uint32_t *val)
{
	struct epc_rk3399_data *d = dev->data;

	if ((offset & 3U) != 0U || offset >= 0x1000U || func >= nfuncs(dev)) {
		return -EINVAL;
	}
	*val = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + offset);
	return 0;
}

static int epc_cfg_write(const struct device *dev, uint8_t func, uint32_t offset, uint32_t val)
{
	struct epc_rk3399_data *d = dev->data;

	if ((offset & 3U) != 0U || offset >= 0x1000U || func >= nfuncs(dev)) {
		return -EINVAL;
	}
	cdns_ep_write(&d->core, CDNS_EP_FUNC_BASE(func) + offset, val);
	return 0;
}

static const struct pcie_epc_features *epc_get_features(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;

	if (is_vf(dev, func)) {
		return &d->vf_features;
	}
	return func < nfuncs(dev) ? &d->features : NULL;
}

static bool epc_link_up(const struct device *dev)
{
	return DEV_DATA(dev)->link_up;
}

static struct udma_desc udma_desc[UDMA_DESCS] __aligned(UDMA_DESC_ALIGN);
static uint8_t udma_flush_buf[UDMA_DESC_ALIGN] __aligned(UDMA_DESC_ALIGN);
static uint8_t udma_bounce[KB(16)] __aligned(CONFIG_DCACHE_LINE_SIZE);

static void udma_desc_fill(struct udma_desc *dsc, uint32_t axi, uint64_t pci, uint32_t len,
			   uint8_t ctrl, uint32_t next)
{
	dsc->axi_addr = axi;
	dsc->axi_attr = 0U;
	dsc->pci_addr_lo = (uint32_t)pci;
	dsc->pci_addr_hi = (uint32_t)(pci >> 32);
	dsc->tlp_attr_lo = 0U; /* requester ID of function 0, TC0, strict ordering */
	dsc->tlp_attr_hi = 0U;
	dsc->len_ctrl = (len & GENMASK(23, 0)) | ((uint32_t)ctrl << 24);
	dsc->status = 0U;
	dsc->next = next;
}

/*
 * Run the list at udma_desc[0..n-1] on the channel and wait for it. The
 * done interrupt comes from the last descriptor; with the interrupt lost
 * the status register is read after the timeout. Each descriptor's
 * channel status byte then says how far the list got.
 */
static int udma_run(const struct device *dev, unsigned int n, bool outbound)
{
	struct epc_rk3399_data *d = dev->data;
	uint32_t ust;
	int ret = 0;

	sys_cache_data_flush_range(udma_desc, sizeof(udma_desc));
	(void)atomic_clear(&d->udma_int);
	k_sem_reset(&d->udma_done);
	udma_wr(dev, UDMA_ATTR_L(UDMA_CHAN), 0U);
	udma_wr(dev, UDMA_SP_L(UDMA_CHAN), (uint32_t)k_mem_phys_addr(&udma_desc[0]));
	udma_wr(dev, UDMA_SP_U(UDMA_CHAN), 0U);
	udma_wr(dev, UDMA_CTRL(UDMA_CHAN),
		      UDMA_CTRL_GO | (outbound ? UDMA_CTRL_OB : 0U));

	if (k_sem_take(&d->udma_done, K_MSEC(UDMA_TIMEOUT_MS)) != 0) {
		ust = udma_rd(dev, UDMA_INT) &
		      (UDMA_INT_DONE(UDMA_CHAN) | UDMA_INT_ERR(UDMA_CHAN));
		if (ust != 0U) {
			udma_wr(dev, UDMA_INT, ust);
			d->udma_stat.n_polled++;
		} else {
			d->udma_stat.n_timeout++;
			ret = -ETIMEDOUT;
		}
	} else {
		ust = (uint32_t)atomic_get(&d->udma_int);
	}

	sys_cache_data_invd_range(udma_desc, sizeof(udma_desc));
	for (unsigned int i = 0; i < n && ret == 0; i++) {
		uint8_t cs = (uint8_t)(udma_desc[i].status >> 16);

		if ((cs & UDMA_DS_ERRORS) != 0U || (cs & UDMA_DS_DONE) == 0U) {
			ret = -EIO;
		}
	}
	if (ret == 0 && (ust & UDMA_INT_ERR(UDMA_CHAN)) != 0U) {
		ret = -EIO;
	}
	if (ret != 0) {
		d->udma_stat.n_err++;
		for (unsigned int i = 0; i < n; i++) {
			if (udma_desc[i].status != 0U || i == n - 1U) {
				d->udma_stat.last_err = udma_desc[i].status;
			}
		}
		if (d->udma_stat.n_err <= 5U) {
			LOG_WRN("udma %s: %d, int %08x, status %08x/%08x", outbound ? "out" : "in",
				ret, ust, udma_desc[0].status, udma_desc[n - 1U].status);
		}
	}
	return ret;
}

/* aligned run: phys and size multiples of UDMA_ALIGN, mutex held */
static int udma_xfer(const struct device *dev, uint64_t pci_addr, uintptr_t phys, size_t size,
		     bool to_host)
{
	int ret = 0;

	for (size_t off = 0; off < size && ret == 0; off += UDMA_MAX_LEN) {
		uint32_t len = (uint32_t)MIN(size - off, (size_t)UDMA_MAX_LEN);

		udma_desc_fill(&udma_desc[0], (uint32_t)(phys + off), pci_addr + off, len,
			       UDMA_DC_IRQ, 0U);
		ret = udma_run(dev, 1U, to_host);
		if (ret == 0 && to_host) {
			/*
			 * The done interrupt says the writes left the channel,
			 * not that they reached the host. A read of the last
			 * bytes on the same path is ordered behind them (reads
			 * do not pass posted writes), so the data is visible
			 * before the caller's MSI.
			 */
			udma_desc_fill(&udma_desc[0], (uint32_t)k_mem_phys_addr(udma_flush_buf),
				       pci_addr + off + len - 4U, 4U, UDMA_DC_IRQ, 0U);
			sys_cache_data_flush_and_invd_range(udma_flush_buf,
							    sizeof(udma_flush_buf));
			ret = udma_run(dev, 1U, false);
		}
	}
	return ret;
}

/*
 * The middle part, PCIe address aligned. Local memory goes through the
 * bounce buffer when it is not aligned for the channel or, toward local
 * memory, when its ends share cache lines with other data: invalidating
 * such a line drops the neighbour's writes, cleaning it overwrites DMA
 * data.
 */
static int udma_mid(const struct device *dev, uint64_t pci_addr, uint8_t *local, size_t size,
		    bool to_host)
{
	uintptr_t phys = k_mem_phys_addr(local);
	bool bounce = to_host ? (phys & (UDMA_ALIGN - 1U)) != 0U
			      : ((phys | size) & (CONFIG_DCACHE_LINE_SIZE - 1U)) != 0U;
	int ret = 0;

	if (!bounce) {
		if (to_host) {
			sys_cache_data_flush_range(local, size);
		} else {
			sys_cache_data_flush_and_invd_range(local, size);
		}
		ret = udma_xfer(dev, pci_addr, phys, size, to_host);
		if (!to_host) {
			sys_cache_data_invd_range(local, size);
		}
		return ret;
	}
	for (size_t off = 0; off < size && ret == 0; off += sizeof(udma_bounce)) {
		size_t n = MIN(size - off, sizeof(udma_bounce));

		if (to_host) {
			memcpy(udma_bounce, local + off, n);
			sys_cache_data_flush_range(udma_bounce, n);
		} else {
			sys_cache_data_flush_and_invd_range(udma_bounce, sizeof(udma_bounce));
		}
		ret = udma_xfer(dev, pci_addr + off, k_mem_phys_addr(udma_bounce), n, to_host);
		if (ret == 0 && !to_host) {
			sys_cache_data_invd_range(udma_bounce, sizeof(udma_bounce));
			memcpy(local + off, udma_bounce, n);
		}
	}
	return ret;
}

/*
 * A transfer to a PCIe address 3 bytes off alignment ends early (channel
 * status 0x02, PCIe transfer incomplete); the TRM names no alignment
 * rule. The unaligned head and tail (up to 7 bytes each) go through the
 * outbound window instead.
 */
static int epc_xfer_dma(const struct device *dev, uint8_t func, uint64_t pci_addr, void *local,
			size_t size, bool to_host)
{
	struct epc_rk3399_data *d = dev->data;
	uint8_t *buf = local;
	size_t head, tail, mid;
	uint64_t t0;
	int ret = 0;

	if (!d->link_up) {
		return -ENOTCONN;
	}
	if (func != 0U) {
		return -ENOTSUP; /* descriptors carry function 0's requester ID */
	}
	if (!(d->bme & BIT(func)) &&
	    (cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + PCI_COMMAND_REG) &
	     PCI_COMMAND_MASTER) == 0U) {
		return -EACCES;
	}
	if (size == 0U) {
		return 0;
	}
	if ((uint64_t)k_mem_phys_addr(local) + size > BIT64(32)) {
		return -EINVAL; /* 32-bit AXI addresses */
	}
	head = MIN(size, (size_t)((UDMA_ALIGN - (pci_addr & (UDMA_ALIGN - 1U))) &
				  (UDMA_ALIGN - 1U)));
	tail = (size - head) & (UDMA_ALIGN - 1U);
	mid = size - head - tail;

	k_mutex_lock(&d->udma_lock, K_FOREVER);
	ob_unblock(d);
	t0 = k_cycle_get_64();
	if (head != 0U) {
		ret = pcie_epc_xfer_memcpy(dev, func, pci_addr, buf, head, to_host);
	}
	if (ret == 0 && mid != 0U) {
		ret = udma_mid(dev, pci_addr + head, buf + head, mid, to_host);
	}
	if (ret == 0 && tail != 0U) {
		ret = pcie_epc_xfer_memcpy(dev, func, pci_addr + head + mid, buf + head + mid,
					   tail, to_host);
	}
	if (ret == 0) {
		if (to_host) {
			d->udma_stat.n_to++;
			d->udma_stat.bytes_to += size;
			d->udma_stat.cyc_to += k_cycle_get_64() - t0;
		} else {
			d->udma_stat.n_from++;
			d->udma_stat.bytes_from += size;
			d->udma_stat.cyc_from += k_cycle_get_64() - t0;
		}
	}
	k_mutex_unlock(&d->udma_lock);
	return ret;
}

#if defined(CONFIG_PCIE_EPC_RK3399_DOORBELL)
static void db_isr(const void *arg)
{
	struct db_slot *slot = (struct db_slot *)arg;
	const struct device *dev = slot->dev;
	struct epc_rk3399_data *d = dev->data;
	pcie_epc_doorbell_cb_t cb = d->db_cb;

	slot->hits++;
	if (d->db_seen < 0) {
		d->db_seen = (int32_t)slot->devid;
	}
	if (cb != NULL) {
		cb(dev, d->db_func, d->db_user);
	}
}

static uint32_t db_candidate(unsigned int i)
{
	if (CONFIG_PCIE_EPC_RK3399_DOORBELL_DEVID >= 0) {
		return (uint32_t)CONFIG_PCIE_EPC_RK3399_DOORBELL_DEVID;
	}
	return i < 8U ? i : (i - 7U) << 8;
}

/* EventID 0 of each candidate DeviceID to its own LPI */
static int db_map(const struct device *dev)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;

	for (unsigned int i = d->db_nslots; i < DB_SLOTS; i++) {
		struct db_slot *slot = &d->db_slot[i];
		int ret;

		slot->dev = dev;
		slot->devid = db_candidate(i);
		ret = its_setup_deviceid(cfg->its, slot->devid, 1);
		if (ret != 0) {
			LOG_ERR("doorbell: DeviceID 0x%x: %d", slot->devid, ret);
			return ret;
		}
		slot->intid = its_alloc_intid(cfg->its);
		ret = its_map_intid(cfg->its, slot->devid, 0, slot->intid);
		if (ret != 0) {
			LOG_ERR("doorbell: map DeviceID 0x%x to %u: %d", slot->devid, slot->intid,
				ret);
			return ret;
		}
		if (irq_connect_dynamic(slot->intid, cfg->db_prio, db_isr, slot, 0) < 0) {
			return -ENOMEM;
		}
		irq_enable(slot->intid);
		d->db_nslots = (uint8_t)(i + 1U);
	}
	return 0;
}

static int epc_alloc_doorbell(const struct device *dev, uint8_t func, pcie_epc_doorbell_cb_t cb,
			      void *user_data, struct pcie_epc_doorbell *db)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	int ret;

	if (func >= nfuncs(dev) || cb == NULL || db == NULL) {
		return -EINVAL;
	}
	if (d->db_cb != NULL) {
		return -EBUSY;
	}
	ret = db_map(dev);
	if (ret != 0) {
		return ret;
	}
	/* deliver the doorbell to the CPU of its user (the thread allocating it) */
	if (arch_num_cpus() > 1 && d->db_cpu != (int8_t)arch_curr_cpu()->id) {
		unsigned int cpu = arch_curr_cpu()->id;

		for (unsigned int i = 0; i < d->db_nslots; i++) {
			ret = its_move_intid(cfg->its, d->db_slot[i].devid, 0, cpu);
			if (ret != 0) {
				LOG_WRN("doorbell: LPI %u to CPU%u: %d", d->db_slot[i].intid, cpu,
					ret);
				break;
			}
		}
		d->db_cpu = ret == 0 ? (int8_t)cpu : -1;
	}
	d->db_func = func;
	d->db_user = user_data;
	d->db_cb = cb;
	/* the ITS sits on the same AXI interconnect: CPU and inbound addresses agree */
	db->phys = its_get_msi_addr(cfg->its);
	db->data = 0U;
	return 0;
}

static int epc_free_doorbell(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;

	if (d->db_cb == NULL || d->db_func != func) {
		return -EINVAL;
	}
	/* the LPIs stay mapped and enabled for the next allocation; db_isr ignores them */
	d->db_cb = NULL;
	return 0;
}
#endif /* CONFIG_PCIE_EPC_RK3399_DOORBELL */

static DEVICE_API(pcie_epc, epc_rk3399_api) = {
	.start = epc_start,
	.stop = epc_stop,
	.write_header = epc_write_header,
	.set_bar = epc_set_bar,
	.clear_bar = epc_clear_bar,
	.map_addr = epc_map_addr,
	.unmap_addr = epc_unmap_addr,
	.set_msi = epc_set_msi,
	.get_msi = epc_get_msi,
	.raise_irq = epc_raise_irq,
	.xfer_dma = epc_xfer_dma,
	.set_msix = epc_set_msix,
	.get_msix = epc_get_msix,
	.get_bar_size = epc_get_bar_size,
#if defined(CONFIG_PCIE_EPC_RK3399_DOORBELL)
	.alloc_doorbell = epc_alloc_doorbell,
	.free_doorbell = epc_free_doorbell,
#endif
	.cfg_read = epc_cfg_read,
	.cfg_write = epc_cfg_write,
	.get_features = epc_get_features,
	.link_up = epc_link_up,
};

/* --- pcie shell backend ---------------------------------------------------- */

#if defined(CONFIG_PCIE_CTRL_SHELL)
static bool sh_active(const struct device *dev)
{
	ARG_UNUSED(dev);
	return active_role();
}

static void sh_info(const struct device *dev, const struct shell *sh)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	uint8_t s;

	if (!d->active) {
		shell_print(sh, "endpoint idle (root complex role this boot)");
		return;
	}
	s = rk3399_pcie_ltssm(apb(dev));
	shell_print(sh, "ltssm      0x%02x %s, link %s, started %d, perst %s", s,
		    rk3399_pcie_ltssm_name(s), d->link_up ? "up" : "down", d->epc.started,
		    cfg->perst.port == NULL ? "-" : (d->perst_asserted ? "asserted" : "released"));
	if (d->link_up) {
		unsigned int gen, width;

		rk3399_pcie_link_info(apb(dev), &gen, &width);
		shell_print(sh, "link       Gen%u x%u (of x%u, max Gen%u)", gen, width,
			    cfg->port.lanes, cfg->port.link_gen);
	}
	shell_print(sh, "id         %08x  cmd/sts %08x  class %08x", 
		    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0)),
		    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0) + 4),
		    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0) + 8));
	shell_print(sh, "functions  %02x enabled, bme %02x, ob regions %08x", d->epc.func_map,
		    d->bme, d->ob_map);
	for (int b = 0; b < 6; b++) {
		shell_print(sh, "BAR%d       %08x", b,
			    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0) + 0x10 + 4 * b));
	}
	shell_print(sh, "events     up %u down %u hot-reset %u", d->ups, d->downs, d->hot_resets);
	for (unsigned int i = 0; i < ARRAY_SIZE(d->client_cnt); i++) {
		if (d->client_cnt[i] != 0U) {
			shell_print(sh, "irq        %-18s %u",
				    client_int_name[i] != NULL ? client_int_name[i] : "?",
				    d->client_cnt[i]);
		}
	}
	shell_print(sh, "udma       cap %08x conf %08x int %08x, client int %08x mask %08x",
		    udma_rd(dev, UDMA_CAP), udma_rd(dev, UDMA_CONF),
		    udma_rd(dev, UDMA_INT), rk_client_read(apb(dev), RK_CLIENT_INT_STATUS),
		    rk_client_read(apb(dev), RK_CLIENT_INT_MASK));
	shell_print(sh, "udma       to host %u (%llu B, %llu us), from host %u (%llu B, %llu us)",
		    d->udma_stat.n_to, d->udma_stat.bytes_to,
		    k_cyc_to_us_floor64(d->udma_stat.cyc_to), d->udma_stat.n_from,
		    d->udma_stat.bytes_from, k_cyc_to_us_floor64(d->udma_stat.cyc_from));
	shell_print(sh, "udma       errors %u (last status %08x), timeouts %u, irq lost %u",
		    d->udma_stat.n_err, d->udma_stat.last_err, d->udma_stat.n_timeout,
		    d->udma_stat.n_polled);
	for (uint8_t fn = 0; fn < MIN(DEV_CFG(dev)->max_functions, REBAR_FUNCS); fn++) {
		for (uint8_t b = 0; b < 6U && d->rebar[fn].n != 0U; b++) {
			size_t now;

			if ((d->rebar[fn].listed & BIT(b)) == 0U ||
			    epc_get_bar_size(dev, fn, b, &now) != 0) {
				continue;
			}
			shell_print(sh, "rebar      fn%u BAR%u %zu MiB%s (max %zu MiB)", fn, b,
				    now >> 20, d->rebar[fn].max_size[b] != 0U ? "" : ", fixed",
				    MAX(d->rebar[fn].max_size[b], d->rebar[fn].size[b]) >> 20);
		}
	}
#if defined(CONFIG_PCIE_EPC_RK3399_DOORBELL)
	shell_fprintf(sh, SHELL_NORMAL, "doorbell   %s, fn %u, CPU %d, %u DeviceIDs mapped, seen ",
		      d->db_cb != NULL ? "on" : "off", d->db_func, d->db_cpu, d->db_nslots);
	if (d->db_seen < 0) {
		shell_fprintf(sh, SHELL_NORMAL, "none\n");
	} else {
		shell_fprintf(sh, SHELL_NORMAL, "0x%x\n", (unsigned int)d->db_seen);
	}
	for (unsigned int i = 0; i < d->db_nslots; i++) {
		if (d->db_slot[i].hits != 0U) {
			shell_print(sh, "doorbell   DeviceID 0x%04x LPI %u: %u",
				    d->db_slot[i].devid, d->db_slot[i].intid, d->db_slot[i].hits);
		}
	}
#endif
	for (unsigned int i = 0; i < ARRAY_SIZE(d->lm_cnt); i++) {
		if (d->lm_cnt[i] != 0U) {
			shell_print(sh, "core error %-18s %u",
				    lm_int_name[i] != NULL ? lm_int_name[i] : "?", d->lm_cnt[i]);
		}
	}
}

#if defined(CONFIG_PCIE_ROLE)
static int sh_mode_set(const struct device *dev, enum pcie_shell_role role)
{
	ARG_UNUSED(dev);
	return pcie_role_save(role == PCIE_SHELL_ROLE_RC ? PCIE_ROLE_RC : PCIE_ROLE_EP);
}

static bool sh_mode_saved(const struct device *dev, enum pcie_shell_role *role)
{
	enum pcie_role r;

	ARG_UNUSED(dev);
	if (!pcie_role_saved(&r)) {
		return false;
	}
	*role = r == PCIE_ROLE_RC ? PCIE_SHELL_ROLE_RC : PCIE_SHELL_ROLE_EP;
	return true;
}
#endif

static int sh_link(const struct device *dev, const struct shell *sh, bool do_retrain)
{
	struct epc_rk3399_data *d = dev->data;

	if (!d->active) {
		return -EBUSY;
	}
	if (do_retrain) {
		retrain(d);
	}
	shell_print(sh, "link %s, ltssm %s", d->link_up ? "up" : "down",
		    rk3399_pcie_ltssm_name(rk3399_pcie_ltssm(apb(dev))));
	return 0;
}

/*
 * Busy-poll LTSSM, the client's data link status and LM link ready for ms
 * milliseconds and print every change with its time: recovery passes and
 * short drops the link work's polling never sees. Interrupts stay on.
 */
static int sh_link_trace(const struct device *dev, const struct shell *sh, uint32_t ms)
{
	struct epc_rk3399_data *d = dev->data;
	static struct {
		uint32_t t_us;
		uint8_t ltssm;
		uint8_t dl;
		uint8_t ls;
	} ev[256];
	unsigned int n = 0, lost = 0;
	uint64_t t0 = k_cycle_get_64(), end = t0 + k_us_to_cyc_ceil64((uint64_t)ms * 1000U);
	uint32_t prev = UINT32_MAX;

	if (!d->active) {
		return -EBUSY;
	}
	for (uint64_t now = t0; now < end; now = k_cycle_get_64()) {
		uint8_t s = rk3399_pcie_ltssm(apb(dev));
		uint8_t dl = (rk_client_read(apb(dev), RK_CLIENT_BASIC_STATUS1) >> 20) & 3U;
		uint8_t ls = cdns_ep_read(&d->core, CDNS_LM_PL_CONFIG0) & 1U;
		uint32_t cur = ((uint32_t)s << 16) | ((uint32_t)dl << 8) | ls;

		if (cur == prev) {
			continue;
		}
		prev = cur;
		if (n < ARRAY_SIZE(ev)) {
			ev[n].t_us = (uint32_t)k_cyc_to_us_floor64(now - t0);
			ev[n].ltssm = s;
			ev[n].dl = dl;
			ev[n].ls = ls;
			n++;
		} else {
			lost++;
		}
	}
	for (unsigned int i = 0; i < n; i++) {
		shell_print(sh, "%10u us  ltssm 0x%02x %-24s dl %u ls %u", ev[i].t_us, ev[i].ltssm,
			    rk3399_pcie_ltssm_name(ev[i].ltssm), ev[i].dl, ev[i].ls);
	}
	shell_print(sh, "%u changes in %u ms%s", n + lost, ms,
		    lost ? " (later ones not kept)" : "");
	return 0;
}

static int sh_id_get(const struct device *dev, pcie_bdf_t bdf, struct pcie_shell_id *id)
{
	struct epc_rk3399_data *d = dev->data;
	uint8_t fn = PCIE_BDF_TO_FUNC(bdf);
	uint32_t v;

	if (!d->active || fn >= nfuncs(dev)) {
		return -EINVAL;
	}
	v = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn));
	id->vid = v & 0xffff;
	id->did = v >> 16;
	v = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn) + 0x2c);
	id->svid = v & 0xffff;
	id->ssid = v >> 16;
	id->class_code = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn) + 8) >> 8;
	return 0;
}

/* applied now; nothing kept (no settings storage on this board yet) */
static int sh_id_set(const struct device *dev, pcie_bdf_t bdf, const struct pcie_shell_id *id)
{
	struct pcie_epc_header h = {
		.vendor_id = id->vid,
		.device_id = id->did,
		.subsys_vendor_id = id->svid,
		.subsys_id = id->ssid,
		.interrupt_pin = 1,
	};
	struct pcie_shell_id cur;
	uint32_t cc;

	if (sh_id_get(dev, bdf, &cur) != 0) {
		return -EINVAL;
	}
	cc = id->class_code == UINT32_MAX ? cur.class_code : id->class_code;
	h.baseclass = cc >> 16;
	h.subclass = cc >> 8;
	h.prog_if = cc;
	return epc_write_header(dev, PCIE_BDF_TO_FUNC(bdf), &h);
}

static uint32_t sh_cfg_read(const struct device *dev, pcie_bdf_t bdf, unsigned int reg)
{
	uint32_t v;

	if (!DEV_DATA(dev)->active ||
	    epc_cfg_read(dev, PCIE_BDF_TO_FUNC(bdf), reg * 4U, &v) != 0) {
		return UINT32_MAX;
	}
	return v;
}

static void sh_cfg_write(const struct device *dev, pcie_bdf_t bdf, unsigned int reg, uint32_t v)
{
	if (DEV_DATA(dev)->active) {
		(void)epc_cfg_write(dev, PCIE_BDF_TO_FUNC(bdf), reg * 4U, v);
	}
}
#endif

/* --- init ------------------------------------------------------------------ */

static int epc_rk3399_init(const struct device *dev)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	struct pcie_epc_ob_window ob = {
		.cpu_base = cfg->mem_base + OB_REGION,
		.axi_base = cfg->mem_base + OB_REGION,
		.size = cfg->mem_size - OB_REGION,
		.page_size = OB_REGION,
	};
	int ret;

	d->dev = dev;
#if defined(CONFIG_PCIE_CTRL_SHELL)
	static const struct pcie_shell_ctrl shell_ctrl = {
		.name = "rk3399",
		.dev = DEVICE_DT_INST_GET(0),
		.role = PCIE_SHELL_ROLE_EP,
		.active = sh_active,
		.info = sh_info,
#if defined(CONFIG_PCIE_ROLE)
		.mode_set = sh_mode_set,
		.mode_saved = sh_mode_saved,
#endif
		.link = sh_link,
		.link_trace = sh_link_trace,
		.id_get = sh_id_get,
		.id_set = sh_id_set,
		.cfg_read = sh_cfg_read,
		.cfg_write = sh_cfg_write,
	};

	pcie_shell_register(&shell_ctrl);
#endif

	if (!active_role()) {
		LOG_INF("%s: root complex role this boot, endpoint off", dev->name);
		return -EBUSY;
	}

	DEVICE_MMIO_NAMED_MAP(dev, apb, K_MEM_CACHE_NONE);
	d->core.base = apb(dev) + RK_CORE_OFFSET;
	d->core.variant = &rk3399_cdns_variant;
	d->irq_pci_addr = UINT64_MAX;
	d->ob_map = BIT(OB_IRQ_REGION);
	k_work_init_delayable(&d->link_work, link_work);
	k_mutex_init(&d->udma_lock);
	k_sem_init(&d->udma_done, 0, 1);
#if defined(CONFIG_PCIE_EPC_RK3399_DOORBELL)
	d->db_seen = -1;
	d->db_cpu = -1;
#endif
	k_timer_init(&d->sriov_timer, sriov_timer_fn, NULL);

	/* the slot's 3.3 V belongs to the host now */
	if (cfg->slot_power.port != NULL) {
		(void)gpio_pin_configure_dt(&cfg->slot_power, GPIO_OUTPUT_INACTIVE);
	}

	ret = rk3399_pcie_port_prepare(&cfg->port);
	if (ret == 0) {
		ret = rk3399_pcie_port_init(&cfg->port, apb(dev), false);
	}
	if (ret != 0) {
		LOG_ERR("%s: port init failed: %d", dev->name, ret);
		return ret;
	}
	core_tune(d);
	cfg->irq_config();
	LOG_INF("%s: core up, reset VID/SSVID 0x%08x", dev->name,
		cdns_ep_read(&d->core, CDNS_LM_ID));

	/*
	 * The core comes out of reset with BARs of its own enabled (BAR2/3
	 * 64-bit 4 MiB, BAR5 4 MiB on RK3399): disable all, the functions
	 * set the ones they use.
	 */
	for (uint8_t fn = 0; fn < cfg->max_functions; fn++) {
		for (uint8_t bar = 0; bar < 6U; bar++) {
			(void)cdns_ep_clear_bar(&d->core, fn, bar);
		}
	}
	/* the VF BARs too (reset: 3 x 64-bit 4 MiB), and their translations */
	for (uint8_t fn = 1; fn <= 8U; fn++) {
		for (uint8_t bar = 0; bar < 6U; bar++) {
			(void)cdns_ep_clear_vf_bar(&d->core, 0, fn, bar, true);
		}
	}
	if (IS_ENABLED(CONFIG_PCIE_EPC_RK3399_SRIOV)) {
		cdns_ep_sriov_setup(&d->core, 0, cfg->max_vfs);
	}

	if (!IS_ENABLED(CONFIG_PCIE_EPC_RK3399_MSIX)) {
		cdns_ep_hide_msix_cap(&d->core);
	}
	for (uint8_t fn = 0; fn <= cfg->max_vfs; fn++) {
		if (!cdns_ep_hide_flr(&d->core, fn)) {
			LOG_WRN("%s: fn%u still advertises FLR, a host reset will hang it",
				dev->name, fn);
		}
	}
	cdns_ep_write(&d->core, CDNS_LM_EP_FUNC_CFG, BIT(0));
	if (cfg->has_ids) {
		(void)epc_write_header(dev, 0, &cfg->hdr);
	}
	ret = cdns_ep_set_msi(&d->core, 0, cfg->msi_vectors);
	if (ret != 0) {
		return ret;
	}
	/*
	 * The whole outbound window is identity mapped (normal non-cacheable)
	 * by soc/rockchip/rk3399/mmu_regions.c, as pcie_epc_core hands out
	 * cpu addresses as pointers; mapping region 0 again would collide.
	 */
	d->irq_va = (mm_reg_t)(cfg->mem_base + OB_IRQ_REGION * OB_REGION);

	if (cfg->perst.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->perst)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->perst, GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}
		gpio_init_callback(&d->perst_cb, perst_handler, BIT(cfg->perst.pin));
		ret = gpio_add_callback_dt(&cfg->perst, &d->perst_cb);
		if (ret != 0) {
			return ret;
		}
	}

	d->features = (struct pcie_epc_features){
		.linkup_notifier = true,
		.msi_capable = true,
		.msix_capable = IS_ENABLED(CONFIG_PCIE_EPC_RK3399_MSIX),
		.intx_capable = true,
		.align = 256,
		.max_functions = cfg->max_functions,
		.max_vfs = cfg->max_vfs,
		.ob_mmio_only = true, /* Normal-NC bursts into the window raise an SError */
		.rebar_min_size = MB(1),
	};
	d->vf_features = d->features;
	d->vf_features.intx_capable = false;
	d->vf_features.msix_capable = false; /* the VF MSI-X capability is not driven */
	d->vf_features.rebar_min_size = 0; /* VF Resizable BAR: not in this core */

	ret = pcie_epc_init_common(dev, &ob, cfg->autostart);
	if (ret != 0) {
		return ret;
	}
	d->active = true;
	return 0;
}

#define RK_CLK(n, name)                                                                            \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, name)),                        \
		.id = DT_INST_CLOCKS_CELL_BY_NAME(n, name, clkid),                                 \
	}

#define RK_RST(n, name)                                                                            \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR_BY_NAME(n, name)),                         \
		.id = DT_INST_RESET_CELL_BY_NAME(n, name, id),                                     \
	}

#define EPC_RK3399_INIT(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP(n, max_functions) >= 1 &&                                        \
		     DT_INST_PROP(n, max_functions) <= CDNS_EP_MAX_FUNCS);                         \
	BUILD_ASSERT(DT_INST_PROP(n, max_virtual_functions) == 0 ||                               \
		     (IS_ENABLED(CONFIG_PCIE_EPC_RK3399_SRIOV) &&                                    \
		      DT_INST_PROP(n, max_functions) == 1 &&                                         \
		      DT_INST_PROP(n, max_virtual_functions) <= 8),                                \
		     "max-virtual-functions: 1..8 VFs of the one PF, needs PCIE_EPC_RK3399_SRIOV"); \
	static void epc_rk3399_irq_config_##n(void);                                               \
	static const struct epc_rk3399_config epc_rk3399_config_##n = {                            \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(apb, DT_DRV_INST(n)),                           \
		.mem_base = DT_INST_REG_ADDR_BY_NAME(n, mem),                                      \
		.mem_size = DT_INST_REG_SIZE_BY_NAME(n, mem),                                      \
		.port = {                                                                          \
			.clks = {                                                                  \
				[RK_CLK_ACLK] = RK_CLK(n, aclk),                                   \
				[RK_CLK_ACLK_PERF] = RK_CLK(n, aclk_perf),                         \
				[RK_CLK_HCLK] = RK_CLK(n, hclk),                                   \
				[RK_CLK_PM] = RK_CLK(n, pm),                                       \
			},                                                                         \
			.rsts = {                                                                  \
				[RK_RST_CORE] = RK_RST(n, core),                                   \
				[RK_RST_MGMT] = RK_RST(n, mgmt),                                   \
				[RK_RST_MGMT_STICKY] = RK_RST(n, mgmt_sticky),                     \
				[RK_RST_PIPE] = RK_RST(n, pipe),                                   \
				[RK_RST_PM] = RK_RST(n, pm),                                       \
				[RK_RST_PCLK] = RK_RST(n, pclk),                                   \
				[RK_RST_ACLK] = RK_RST(n, aclk),                                   \
				[RK_RST_PHY] = RK_RST(n, phy),                                     \
			},                                                                         \
			.grf = DEVICE_DT_GET(DT_INST_PHANDLE(n, rockchip_grf)),                    \
			.lanes = DT_INST_PROP(n, num_lanes),                                       \
			.link_gen = DT_INST_PROP(n, max_link_speed),                               \
		},                                                                                 \
		.perst = GPIO_DT_SPEC_INST_GET_OR(n, perst_gpios, {0}),                            \
		.slot_power = GPIO_DT_SPEC_INST_GET_OR(n, slot_power_gpios, {0}),                  \
		.has_ids = DT_INST_NODE_HAS_PROP(n, vendor_id) &&                                  \
			   DT_INST_NODE_HAS_PROP(n, device_id),                                    \
		.hdr = {                                                                           \
			.vendor_id = DT_INST_PROP_OR(n, vendor_id, 0),                             \
			.device_id = DT_INST_PROP_OR(n, device_id, 0),                             \
			.baseclass = (DT_INST_PROP(n, class_code) >> 16) & 0xff,                   \
			.subclass = (DT_INST_PROP(n, class_code) >> 8) & 0xff,                     \
			.prog_if = DT_INST_PROP(n, class_code) & 0xff,                             \
			.subsys_vendor_id = DT_INST_PROP(n, subsystem_vendor_id),                  \
			.subsys_id = DT_INST_PROP(n, subsystem_id),                                \
			.interrupt_pin = 1,                                                        \
		},                                                                                 \
		.max_functions = DT_INST_PROP(n, max_functions),                                   \
		.max_vfs = DT_INST_PROP(n, max_virtual_functions),                                 \
		.msi_vectors = DT_INST_PROP(n, msi_vectors),                                       \
		.autostart = !DT_INST_PROP(n, zephyr_no_autostart),                                \
		.irq_config = epc_rk3399_irq_config_##n,                                           \
		IF_ENABLED(CONFIG_PCIE_EPC_RK3399_DOORBELL,                                        \
			   (.its = DEVICE_DT_GET(DT_INST_PHANDLE(n, msi_parent)),                  \
			    .db_prio = DT_INST_IRQ_BY_NAME(n, client, priority),))                 \
	};                                                                                         \
	static struct epc_rk3399_data epc_rk3399_data_##n;                                         \
	static void epc_rk3399_irq_config_##n(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, sys, irq),                                      \
			    DT_INST_IRQ_BY_NAME(n, sys, priority), epc_rk3399_isr,                 \
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ_BY_NAME(n, sys, flags));            \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, client, irq),                                   \
			    DT_INST_IRQ_BY_NAME(n, client, priority), epc_rk3399_isr,              \
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ_BY_NAME(n, client, flags));         \
		irq_enable(DT_INST_IRQ_BY_NAME(n, sys, irq));                                      \
		irq_enable(DT_INST_IRQ_BY_NAME(n, client, irq));                                   \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, epc_rk3399_init, NULL, &epc_rk3399_data_##n,                      \
			      &epc_rk3399_config_##n, POST_KERNEL, CONFIG_PCIE_EPC_INIT_PRIORITY,  \
			      &epc_rk3399_api);

DT_INST_FOREACH_STATUS_OKAY(EPC_RK3399_INIT)
