/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NVMe endpoint function.
 *
 * The endpoint shows up on the host as an NVM Express controller (class
 * 01:08:02) with one namespace per Zephyr disk (disk_access: SATA disks on
 * a port and behind a port multiplier, SD, RAM disk, ...): NSID n is the
 * n-th entry of disk-names. A stock host NVMe driver binds to it.
 *
 * The model follows the Linux NVMe PCI endpoint target
 * (drivers/nvme/target/pci-epf.c):
 *  - BAR0 holds the controller registers, the doorbells (stride 4 B) and
 *    the MSI-X table; the host writes them, the function polls them.
 *  - CC is polled; CC.EN 0 -> 1 creates the admin queues from AQA/ASQ/ACQ,
 *    CC.EN 1 -> 0 and CC.SHN tear everything down.
 *  - SQ doorbells are polled round robin with an arbitration burst;
 *    commands are fetched from host memory, their PRPs walked into PCI
 *    address segments, data moved with the controller DMA (CPU copy
 *    through an outbound window otherwise), the completion entry written
 *    to the CQ and an MSI-X / MSI / INTx raised once per batch.
 * Unlike Linux there is no nvmet core behind it: the admin command set
 * (identify, features, logs, queue management) and the NVM read, write,
 * flush and write zeroes commands are implemented here, executed one at a
 * time by one thread.
 *
 * Only pcie_epc.h is used: the function knows nothing about the
 * controller.
 */

#define DT_DRV_COMPAT zephyr_pcie_epf_nvme

#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/version.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epf.h>
#ifdef CONFIG_PCIE_AARDVARK_MODE
#include <zephyr/drivers/pcie/aardvark.h>
#endif
#ifdef CONFIG_MMU
#include <zephyr/kernel/internal/mm.h>
#endif

#include "pcie_epf_nvme.h"

LOG_MODULE_REGISTER(pcie_epf_nvme, CONFIG_PCIE_EP_LOG_LEVEL);

#define MPS        4096U
#define MPS_MASK   (MPS - 1U)
#define MDTS_BYTES (CONFIG_PCIE_EPF_NVME_MDTS_KB * 1024U)
#define MAX_SEGS   (MDTS_BYTES / MPS + 1U)
#define NR_QUEUES  (CONFIG_PCIE_EPF_NVME_MAX_IO_QUEUES + 1)
#define AERL       3U
#define SQ_BURST   8U
#define MAX_NS     CONFIG_PCIE_EPF_NVME_MAX_NAMESPACES

BUILD_ASSERT(IS_POWER_OF_TWO(MDTS_BYTES) && MDTS_BYTES >= MPS,
	     "MDTS must be a power of two of at least 4 KiB");

/* class code of an NVM Express I/O controller */
#define NVME_CLASS 0x010802U

/* Configuration space */
#define CFG_COMMAND     0x04
#define CFG_COMMAND_BME BIT(2)
#define CFG_CLASS       0x08
#define CFG_EXT_CAP     0x100
#define EXT_CAP_ID_DSN  0x0003

struct nvme_queue {
	uint64_t addr;
	uint32_t db;
	uint16_t depth;
	uint16_t head;
	uint16_t tail;
	/* SQ: completion queue; CQ: interrupt vector */
	uint16_t link;
	uint8_t phase;
	bool live;
	bool ien;
	bool irq_pending;
};

struct nvme_seg {
	uint64_t pci;
	uint32_t len;
};

struct nvme_ns {
	bool ready;
	uint64_t nsze;
	uint32_t lba_size;
	uint8_t lbads;
};

struct epf_nvme_config {
	const struct device *epc;
	uint8_t func;
	struct pcie_epc_header hdr;
	uintptr_t mem_base;
	size_t mem_size;
	const char *const *disks;
	uint8_t n_disks;
	const char *serial;
	const char *model;
	uint16_t io_queues;
	uint16_t mqes;
	uint16_t msix_irqs;
	uint8_t msi_irqs;
	k_thread_entry_t thread_fn;
};

struct epf_nvme_stats {
	uint64_t cmds;
	uint64_t admin_cmds;
	uint64_t reads;
	uint64_t writes;
	uint64_t bytes_read;
	uint64_t bytes_written;
	uint64_t errors;
	uint64_t irqs;
	uint64_t irq_errors;
	uint64_t xfer_errors;
	uint32_t enables;
	uint32_t resets;
};

struct epf_nvme_data {
	const struct device *dev;
	struct pcie_epc_listener listener;
	struct pcie_epc_bar bar;
	volatile uint8_t *regs;
	uintptr_t mem_virt;

	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_PCIE_EPF_NVME_STACK_SIZE);
	/* controller reset asked for by a link event */
	atomic_t reset_req;
	bool link;

	uint32_t cc;
	uint32_t csts;
	bool enabled;
	bool bme;
	uint16_t max_vectors;
	uint32_t msix_offset;

	struct nvme_queue sq[NR_QUEUES];
	struct nvme_queue cq[NR_QUEUES];
	/* granted by Set Features Number of Queues, 0-based */
	uint16_t nsqa;
	uint16_t ncqa;

	uint16_t aer_cid[AERL + 1];
	uint8_t aer_count;

	struct nvme_ns ns[MAX_NS];

	char sn[21];
	uint64_t eui64;

	/* feature values, Get Features returns what Set Features stored */
	uint32_t feat_arb;
	uint32_t feat_temp;
	uint32_t feat_err;
	uint32_t feat_vwc;
	uint32_t feat_coalesce;
	uint32_t feat_atomic;
	uint32_t feat_aec;
	uint32_t irq_cd; /* coalescing disabled, per vector (first 32) */

	struct epf_nvme_stats st;
	int64_t boot_ms;

	struct nvme_seg seg[MAX_SEGS];
	uint32_t nseg;
	struct nvme_sqe sqe[SQ_BURST];
	uint64_t prp_list[MPS / 8] __aligned(64);
	uint8_t scratch[NVME_IDENTIFY_SIZE] __aligned(64);
	uint8_t buf[MDTS_BYTES] __aligned(64);
};

static inline uint32_t reg_rd(struct epf_nvme_data *d, uint32_t off)
{
	return sys_le32_to_cpu(sys_read32((mem_addr_t)(d->regs + off)));
}

static inline void reg_wr(struct epf_nvme_data *d, uint32_t off, uint32_t v)
{
	sys_write32(sys_cpu_to_le32(v), (mem_addr_t)(d->regs + off));
}

static inline uint64_t reg_rd64(struct epf_nvme_data *d, uint32_t off)
{
	return (uint64_t)reg_rd(d, off) | ((uint64_t)reg_rd(d, off + 4) << 32);
}

static inline void reg_wr64(struct epf_nvme_data *d, uint32_t off, uint64_t v)
{
	reg_wr(d, off, (uint32_t)v);
	reg_wr(d, off + 4, (uint32_t)(v >> 32));
}

static uint64_t nvme_cap(const struct epf_nvme_config *cfg)
{
	/*
	 * Contiguous queues required, 4 KiB pages only, doorbell stride 4 B,
	 * NVM command set, no NSSR, boot partitions, CMB or PMR. TO covers a
	 * backend that spins a disk up on the first identify.
	 */
	return NVME_CAP_MQES(cfg->mqes - 1U) | NVME_CAP_CQR |
	       NVME_CAP_TO(CONFIG_PCIE_EPF_NVME_READY_TIMEOUT_MS / 500U) | NVME_CAP_CSS_NVM |
	       NVME_CAP_MPSMIN(0) | NVME_CAP_MPSMAX(0);
}

/* Host memory access: refused without bus mastering (outbound faults) */
static int host_xfer(struct epf_nvme_data *d, uint64_t pci, void *local, size_t len,
		     bool to_host)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	int ret = -ENOTSUP;

	if (!d->bme) {
		return -EACCES;
	}
	if (IS_ENABLED(CONFIG_PCIE_EPF_NVME_DMA)) {
		ret = pcie_epc_xfer_dma(cfg->epc, cfg->func, pci, local, len, to_host);
	}
	if (ret == -ENOTSUP) {
		ret = pcie_epc_xfer_memcpy(cfg->epc, cfg->func, pci, local, len, to_host);
	}
	if (ret != 0) {
		d->st.xfer_errors++;
		LOG_WRN("%s host 0x%llx, %zu B: %d", to_host ? "to" : "from",
			(unsigned long long)pci, len, ret);
	}
	return ret;
}

static void bme_update(struct epf_nvme_data *d)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint32_t v = 0;

	(void)pcie_epc_cfg_read(cfg->epc, cfg->func, CFG_COMMAND, &v);
	d->bme = (v & CFG_COMMAND_BME) != 0;
}

/*
 * Interrupts
 */
static void raise_irq(struct epf_nvme_data *d, struct nvme_queue *cq)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint16_t iv = cq->link;
	int n, ret;

	cq->irq_pending = false;
	if (!cq->ien) {
		return;
	}

	n = pcie_epc_get_msix(cfg->epc, cfg->func);
	if (n > 0) {
		ret = pcie_epc_raise_irq(cfg->epc, cfg->func, PCIE_EPC_IRQ_MSIX,
					 (uint16_t)(iv % (uint16_t)n) + 1U);
	} else {
		n = pcie_epc_get_msi(cfg->epc, cfg->func);
		if (n > 0) {
			/* fewer messages than asked for: they alias */
			ret = pcie_epc_raise_irq(cfg->epc, cfg->func, PCIE_EPC_IRQ_MSI,
						 (uint16_t)(iv & (uint16_t)(n - 1)) + 1U);
		} else {
			ret = pcie_epc_raise_irq(cfg->epc, cfg->func, PCIE_EPC_IRQ_INTX, 0);
		}
	}
	if (ret != 0) {
		d->st.irq_errors++;
		LOG_WRN("CQ vector %u: raise irq failed (%d)", iv, ret);
	} else {
		d->st.irqs++;
	}
}

/*
 * Completion queues
 */

/* Room for one entry, waiting for the host to move the head */
static int cq_wait_room(struct epf_nvme_data *d, struct nvme_queue *cq)
{
	int64_t t0 = k_uptime_get();

	for (;;) {
		uint32_t head = reg_rd(d, cq->db);

		if (head < cq->depth) {
			cq->head = (uint16_t)head;
		}
		if ((uint16_t)((cq->tail + 1U) % cq->depth) != cq->head) {
			return 0;
		}
		/* full: let the host run its handler, give up if it resets us */
		if (cq->irq_pending) {
			raise_irq(d, cq);
		}
		if ((reg_rd(d, NVME_REG_CC) & NVME_CC_EN) == 0 || atomic_get(&d->reset_req) != 0) {
			return -ECANCELED;
		}
		if (t0 != 0 && k_uptime_get() - t0 > 1000) {
			LOG_DBG("CQ %u full for 1 s (head %u tail %u)", (unsigned int)(cq - d->cq),
				cq->head, cq->tail);
			t0 = 0;
		}
		k_usleep(CONFIG_PCIE_EPF_NVME_IDLE_POLL_US);
	}
}

static int cq_post(struct epf_nvme_data *d, struct nvme_queue *sq, uint16_t cid, uint16_t status,
		   uint32_t dw0)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	struct nvme_queue *cq = &d->cq[sq->link];
	struct pcie_epc_host_map m;
	volatile uint32_t *p;
	int ret;

	if (!cq->live) {
		return -ENODEV;
	}
	ret = cq_wait_room(d, cq);
	if (ret != 0) {
		return ret;
	}
	if (!d->bme) {
		return -EACCES;
	}

	ret = pcie_epc_map_host(cfg->epc, cfg->func, cq->addr + (uint64_t)cq->tail * 16U, 16,
				&m);
	if (ret != 0) {
		d->st.xfer_errors++;
		return ret;
	}
	p = m.ptr;
	/*
	 * Dword by dword, the one with the phase tag last: the host polls on
	 * the phase and must never see a new phase with stale contents.
	 */
	sys_write32(sys_cpu_to_le32(dw0), (mem_addr_t)&p[0]);
	sys_write32(0, (mem_addr_t)&p[1]);
	sys_write32(sys_cpu_to_le32((uint32_t)sq->head | ((uint32_t)(sq - d->sq) << 16)),
		    (mem_addr_t)&p[2]);
	barrier_dmem_fence_full();
	sys_write32(sys_cpu_to_le32((uint32_t)cid |
				    ((uint32_t)((status << 1) | cq->phase) << 16)),
		    (mem_addr_t)&p[3]);
	barrier_dmem_fence_full();
	/* non-posted read: the entry is in host memory before the interrupt */
	(void)sys_read32((mem_addr_t)&p[3]);
	pcie_epc_unmap_host(cfg->epc, &m);

	cq->tail++;
	if (cq->tail == cq->depth) {
		cq->tail = 0;
		cq->phase ^= 1U;
	}
	cq->irq_pending = true;
	if (status != NVME_SC_SUCCESS) {
		d->st.errors++;
	}
	return 0;
}

/*
 * PRPs -> PCI address segments (contiguous pages merged)
 */
static void seg_add(struct epf_nvme_data *d, uint64_t pci, uint32_t len)
{
	if (d->nseg != 0U) {
		struct nvme_seg *s = &d->seg[d->nseg - 1U];

		if (s->pci + s->len == pci) {
			s->len += len;
			return;
		}
	}
	d->seg[d->nseg].pci = pci;
	d->seg[d->nseg].len = len;
	d->nseg++;
}

static uint16_t prp_parse(struct epf_nvme_data *d, const struct nvme_sqe *e, uint32_t len)
{
	uint64_t prp1 = NVME_SQE_PRP1(e);
	uint64_t prp2 = NVME_SQE_PRP2(e);
	uint32_t first, rem;
	uint64_t list;

	d->nseg = 0;
	if (NVME_SQE_PSDT(e) != 0U) {
		/* SGLS = 0 in identify controller: PRPs only */
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	if ((prp1 & 3U) != 0U) {
		return NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
	}

	first = MIN(len, MPS - (uint32_t)(prp1 & MPS_MASK));
	seg_add(d, prp1, first);
	rem = len - first;
	if (rem == 0U) {
		return NVME_SC_SUCCESS;
	}
	if (prp2 == 0U) {
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	if (rem <= MPS) {
		if ((prp2 & MPS_MASK) != 0U) {
			return NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
		}
		seg_add(d, prp2, rem);
		return NVME_SC_SUCCESS;
	}

	/* PRP2 points to a list; its last entry chains when more is needed */
	list = prp2;
	while (rem != 0U) {
		uint32_t avail = (MPS - (uint32_t)(list & MPS_MASK)) / 8U;
		uint32_t need = DIV_ROUND_UP(rem, MPS);
		uint32_t n = MIN(need, avail);
		bool chain = need > avail;

		if ((list & 7U) != 0U) {
			return NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
		}
		if (host_xfer(d, list, d->prp_list, n * 8U, false) != 0) {
			return NVME_SC_DATA_XFER_ERROR;
		}
		for (uint32_t i = 0; i < n - (chain ? 1U : 0U); i++) {
			uint64_t prp = sys_le64_to_cpu(d->prp_list[i]);
			uint32_t l = MIN(rem, MPS);

			if (prp == 0U) {
				return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
			}
			if ((prp & MPS_MASK) != 0U) {
				return NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR;
			}
			if (d->nseg == MAX_SEGS && d->seg[d->nseg - 1U].pci +
							   d->seg[d->nseg - 1U].len != prp) {
				return NVME_SC_INTERNAL | NVME_STATUS_DNR;
			}
			seg_add(d, prp, l);
			rem -= l;
		}
		if (chain) {
			list = sys_le64_to_cpu(d->prp_list[n - 1U]);
			if (list == 0U) {
				return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
			}
		}
	}
	return NVME_SC_SUCCESS;
}

/* Move @len bytes of the command data between d->buf and the host */
static uint16_t data_xfer(struct epf_nvme_data *d, const struct nvme_sqe *e, uint32_t len,
			  bool to_host)
{
	uint16_t status = prp_parse(d, e, len);
	uint32_t off = 0;

	if (status != NVME_SC_SUCCESS) {
		return status;
	}
	for (uint32_t i = 0; i < d->nseg; i++) {
		if (host_xfer(d, d->seg[i].pci, &d->buf[off], d->seg[i].len, to_host) != 0) {
			return NVME_SC_DATA_XFER_ERROR;
		}
		off += d->seg[i].len;
	}
	return NVME_SC_SUCCESS;
}

/*
 * Namespace backend
 */
/*
 * Namespace @nsid (1-based) of disk-names[nsid - 1]. A disk that is not
 * there (empty port multiplier port, no card) stays inactive and is asked
 * again on the next identify, so a disk that comes up late still appears
 * after a host rescan.
 */
static struct nvme_ns *ns_get(struct epf_nvme_data *d, uint32_t nsid)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	struct nvme_ns *ns;
	const char *name;
	uint32_t count = 0, size = 0;
	int ret;

	if (nsid == 0U || nsid > cfg->n_disks) {
		return NULL;
	}
	ns = &d->ns[nsid - 1U];
	if (ns->ready) {
		return ns;
	}
	name = cfg->disks[nsid - 1U];
	ret = disk_access_init(name);
	if (ret == 0) {
		ret = disk_access_ioctl(name, DISK_IOCTL_GET_SECTOR_COUNT, &count);
	}
	if (ret == 0) {
		ret = disk_access_ioctl(name, DISK_IOCTL_GET_SECTOR_SIZE, &size);
	}
	if (ret != 0 || count == 0U || size < 512U || size > MPS || !IS_POWER_OF_TWO(size)) {
		LOG_DBG("disk %s not usable (%d, %u x %u B)", name, ret, count, size);
		return ns;
	}
	ns->nsze = count;
	ns->lba_size = size;
	ns->lbads = (uint8_t)(find_lsb_set(size) - 1);
	ns->ready = true;
	LOG_INF("namespace %u: disk %s, %llu x %u B (%llu MiB)", nsid, name,
		(unsigned long long)ns->nsze, size,
		(unsigned long long)((ns->nsze * size) >> 20));
	if (count == UINT32_MAX) {
		/* disk_access has 32-bit sector numbers */
		LOG_WRN("namespace %u: disk %s truncated to 2^32 sectors", nsid, name);
	}
	return ns;
}

static inline bool ns_active(struct epf_nvme_data *d, uint32_t nsid)
{
	struct nvme_ns *ns = ns_get(d, nsid);

	return ns != NULL && ns->ready;
}

/* Serial number and EUI-64 from the Device Serial Number capability */
static void ids_init(struct epf_nvme_data *d)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint32_t off = CFG_EXT_CAP;
	uint32_t hdr, lo = 0, hi = 0;

	d->eui64 = 0;
	for (int i = 0; i < 32 && off >= CFG_EXT_CAP; i++) {
		if (pcie_epc_cfg_read(cfg->epc, cfg->func, off, &hdr) != 0 || hdr == 0U ||
		    hdr == UINT32_MAX) {
			break;
		}
		if ((hdr & 0xffffU) == EXT_CAP_ID_DSN) {
			(void)pcie_epc_cfg_read(cfg->epc, cfg->func, off + 4, &lo);
			(void)pcie_epc_cfg_read(cfg->epc, cfg->func, off + 8, &hi);
			d->eui64 = ((uint64_t)hi << 32) | lo;
			break;
		}
		off = hdr >> 20;
	}

	if (cfg->serial != NULL && cfg->serial[0] != '\0') {
		snprintf(d->sn, sizeof(d->sn), "%s", cfg->serial);
	} else if (d->eui64 != 0U) {
		snprintf(d->sn, sizeof(d->sn), "%016llx", (unsigned long long)d->eui64);
	} else {
		snprintf(d->sn, sizeof(d->sn), "ZEPHYR%04x%04x%02x", cfg->hdr.vendor_id,
			 cfg->hdr.device_id, cfg->func);
	}
}

/* ASCII, space padded, not terminated */
static void put_str(uint8_t *dst, const char *s, size_t len)
{
	size_t n = MIN(strlen(s), len);

	memset(dst, ' ', len);
	memcpy(dst, s, n);
}

static void build_id_ctrl(struct epf_nvme_data *d, uint8_t *p)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	char fr[9];

	memset(p, 0, NVME_IDENTIFY_SIZE);
	sys_put_le16(cfg->hdr.vendor_id, &p[NVME_IDC_VID]);
	sys_put_le16(cfg->hdr.subsys_vendor_id, &p[NVME_IDC_SSVID]);
	put_str(&p[NVME_IDC_SN], d->sn, 20);
	put_str(&p[NVME_IDC_MN], cfg->model, 40);
	snprintf(fr, sizeof(fr), "%u.%u.%u", KERNEL_VERSION_MAJOR, KERNEL_VERSION_MINOR,
		 KERNEL_PATCHLEVEL);
	put_str(&p[NVME_IDC_FR], fr, 8);
	p[NVME_IDC_RAB] = 6;
	p[NVME_IDC_MDTS] = (uint8_t)(find_lsb_set(MDTS_BYTES / MPS) - 1);
	sys_put_le16(1, &p[NVME_IDC_CNTLID]);
	sys_put_le32(NVME_VS(1, 4, 0), &p[NVME_IDC_VER]);
	p[NVME_IDC_CNTRLTYPE] = 1; /* I/O controller */
	p[NVME_IDC_ACL] = 3;
	p[NVME_IDC_AERL] = AERL;
	p[NVME_IDC_FRMW] = BIT(0) | (1U << 1); /* slot 1 read only, one slot */
	sys_put_le16(343, &p[NVME_IDC_WCTEMP]);
	sys_put_le16(373, &p[NVME_IDC_CCTEMP]);
	p[NVME_IDC_SQES] = (NVME_SQES << 4) | NVME_SQES;
	p[NVME_IDC_CQES] = (NVME_CQES << 4) | NVME_CQES;
	sys_put_le32(cfg->n_disks, &p[NVME_IDC_NN]);
	sys_put_le16(NVME_ONCS_WRITE_ZEROES, &p[NVME_IDC_ONCS]);
	p[NVME_IDC_VWC] = IS_ENABLED(CONFIG_PCIE_EPF_NVME_VWC) ? 1 : 0;
	snprintf((char *)&p[NVME_IDC_SUBNQN], 256, "nqn.2014-08.org.zephyrproject:nvme:%s",
		 d->sn);
	/* power state 0: 5 W, operational */
	sys_put_le16(500, &p[NVME_IDC_PSD0]);
}

/*
 * Unique namespace ID from the function's EUI-64 (the Device Serial
 * Number): NGUID = EUI-64, 0, NSID. No namespace EUI-64, it would have to
 * be made up.
 */
static void put_nguid(struct epf_nvme_data *d, uint32_t nsid, uint8_t *p)
{
	sys_put_be64(d->eui64, p);
	sys_put_be32(0, p + 8);
	sys_put_be32(nsid, p + 12);
}

static void build_id_ns(struct epf_nvme_data *d, uint32_t nsid, uint8_t *p)
{
	struct nvme_ns *ns = ns_get(d, nsid);

	memset(p, 0, NVME_IDENTIFY_SIZE);
	if (ns == NULL || !ns->ready) {
		return; /* NSZE 0: inactive, the host skips it */
	}
	sys_put_le64(ns->nsze, &p[NVME_IDN_NSZE]);
	sys_put_le64(ns->nsze, &p[NVME_IDN_NCAP]);
	sys_put_le64(ns->nsze, &p[NVME_IDN_NUSE]);
	p[NVME_IDN_NLBAF] = 0;
	p[NVME_IDN_FLBAS] = 0;
	if (d->eui64 != 0U) {
		put_nguid(d, nsid, &p[NVME_IDN_NGUID]);
	}
	/* LBA format 0: LBADS, no metadata, best performance */
	p[NVME_IDN_LBAF0 + 2] = ns->lbads;
}

static void build_ns_descs(struct epf_nvme_data *d, uint32_t nsid, uint8_t *p)
{
	size_t i = 0;

	memset(p, 0, NVME_IDENTIFY_SIZE);
	if (d->eui64 != 0U) {
		p[i] = NVME_NIDT_NGUID;
		p[i + 1] = 16;
		put_nguid(d, nsid, &p[i + 4]);
		i += 20;
	}
	p[i] = NVME_NIDT_CSI;
	p[i + 1] = 1;
	p[i + 4] = NVME_CSI_NVM;
}

static uint16_t adm_identify(struct epf_nvme_data *d, const struct nvme_sqe *e)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint8_t cns = NVME_SQE_CDW(e, 10) & 0xffU;
	uint8_t csi = (NVME_SQE_CDW(e, 11) >> 24) & 0xffU;
	uint32_t nsid = NVME_SQE_NSID(e);
	uint8_t *p = d->buf;

	switch (cns) {
	case NVME_CNS_NS:
		/* allocated but inactive: zeros */
		if (nsid == 0U || nsid > cfg->n_disks) {
			return NVME_SC_INVALID_NS | NVME_STATUS_DNR;
		}
		build_id_ns(d, nsid, p);
		break;
	case NVME_CNS_CTRL:
		build_id_ctrl(d, p);
		break;
	case NVME_CNS_NS_ACTIVE_LIST:
		if (nsid >= 0xfffffffeU) {
			return NVME_SC_INVALID_NS | NVME_STATUS_DNR;
		}
		memset(p, 0, NVME_IDENTIFY_SIZE);
		for (uint32_t n = nsid + 1U, i = 0; n <= cfg->n_disks; n++) {
			if (ns_active(d, n)) {
				sys_put_le32(n, &p[4U * i++]);
			}
		}
		break;
	case NVME_CNS_NS_DESC_LIST:
		if (!ns_active(d, nsid)) {
			return NVME_SC_INVALID_NS | NVME_STATUS_DNR;
		}
		build_ns_descs(d, nsid, p);
		break;
	case NVME_CNS_CS_NS:
		if (!ns_active(d, nsid)) {
			return NVME_SC_INVALID_NS | NVME_STATUS_DNR;
		}
		__fallthrough;
	case NVME_CNS_CS_CTRL:
		/* NVM command set specific structures: nothing beyond the base */
		if (csi != NVME_CSI_NVM) {
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		memset(p, 0, NVME_IDENTIFY_SIZE);
		break;
	default:
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	return data_xfer(d, e, NVME_IDENTIFY_SIZE, true);
}

/* 128-bit little endian counter from a 64-bit value */
static void put_le128(uint64_t v, uint8_t *p)
{
	sys_put_le64(v, p);
	sys_put_le64(0, p + 8);
}

static uint16_t adm_get_log(struct epf_nvme_data *d, const struct nvme_sqe *e)
{
	uint32_t cdw10 = NVME_SQE_CDW(e, 10);
	uint8_t lid = cdw10 & 0xffU;
	uint32_t numd = (cdw10 >> 16) | ((NVME_SQE_CDW(e, 11) & 0xffffU) << 16);
	uint64_t lpo = NVME_SQE_CDW(e, 12) | ((uint64_t)NVME_SQE_CDW(e, 13) << 32);
	uint64_t len64 = ((uint64_t)numd + 1U) * 4U;
	uint8_t *log = d->scratch;
	uint32_t len;
	size_t size;

	if (len64 > MDTS_BYTES || (lpo & 3U) != 0U) {
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	len = (uint32_t)len64;
	memset(log, 0, sizeof(d->scratch));

	switch (lid) {
	case NVME_LOG_ERROR:
		/* no error entries */
		size = sizeof(d->scratch);
		break;
	case NVME_LOG_SMART: {
		int64_t up_s = (k_uptime_get() - d->boot_ms) / 1000;

		size = NVME_LOG_SMART_SIZE;
		sys_put_le16(CONFIG_PCIE_EPF_NVME_TEMPERATURE_K, &log[NVME_SMART_TEMP]);
		log[NVME_SMART_AVAIL_SPARE] = 100;
		log[NVME_SMART_SPARE_THRESH] = 10;
		/* data units: thousands of 512 B, rounded up */
		put_le128(DIV_ROUND_UP(d->st.bytes_read / 512U, 1000U),
			  &log[NVME_SMART_DATA_READ]);
		put_le128(DIV_ROUND_UP(d->st.bytes_written / 512U, 1000U),
			  &log[NVME_SMART_DATA_WRITTEN]);
		put_le128(d->st.reads, &log[NVME_SMART_HOST_READS]);
		put_le128(d->st.writes, &log[NVME_SMART_HOST_WRITES]);
		put_le128(1, &log[NVME_SMART_POWER_CYCLES]);
		put_le128((uint64_t)up_s / 3600U, &log[NVME_SMART_POWER_ON_HOURS]);
		break;
	}
	case NVME_LOG_FW_SLOT: {
		char fr[9];

		size = 512;
		log[0] = 1; /* active: slot 1 */
		snprintf(fr, sizeof(fr), "%u.%u.%u", KERNEL_VERSION_MAJOR, KERNEL_VERSION_MINOR,
			 KERNEL_PATCHLEVEL);
		put_str(&log[8], fr, 8);
		break;
	}
	default:
		return NVME_SC_INVALID_LOG_PAGE | NVME_STATUS_DNR;
	}

	if (lpo >= size) {
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	/* beyond the structure: zeros */
	memset(d->buf, 0, len);
	memcpy(d->buf, &log[lpo], MIN(len, size - (size_t)lpo));
	return data_xfer(d, e, len, true);
}

static uint16_t adm_create_cq(struct epf_nvme_data *d, const struct nvme_sqe *e)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint16_t qid = NVME_SQE_CDW(e, 10) & 0xffffU;
	uint16_t qsize = (NVME_SQE_CDW(e, 10) >> 16) + 1U;
	uint32_t cdw11 = NVME_SQE_CDW(e, 11);
	uint16_t iv = cdw11 >> 16;
	uint64_t addr = NVME_SQE_PRP1(e);
	struct nvme_queue *cq;

	if (qid == 0U || qid > MIN(cfg->io_queues, d->ncqa + 1U) || d->cq[qid].live) {
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;
	}
	if (qsize < 2U || qsize > cfg->mqes) {
		return NVME_SC_QUEUE_SIZE | NVME_STATUS_DNR;
	}
	if ((cdw11 & BIT(0)) == 0U || (addr & MPS_MASK) != 0U) {
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	if ((cdw11 & BIT(1)) != 0U && iv >= d->max_vectors) {
		return NVME_SC_INVALID_VECTOR | NVME_STATUS_DNR;
	}

	cq = &d->cq[qid];
	*cq = (struct nvme_queue){
		.addr = addr,
		.db = NVME_CQ_DB(qid),
		.depth = qsize,
		.link = iv,
		.phase = 1,
		.ien = (cdw11 & BIT(1)) != 0U,
		.live = true,
	};
	reg_wr(d, cq->db, 0);
	LOG_DBG("CQ %u: %u entries @0x%llx, vector %u%s", qid, qsize, (unsigned long long)addr, iv,
		cq->ien ? "" : " (no irq)");
	return NVME_SC_SUCCESS;
}

static uint16_t adm_create_sq(struct epf_nvme_data *d, const struct nvme_sqe *e)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint16_t qid = NVME_SQE_CDW(e, 10) & 0xffffU;
	uint16_t qsize = (NVME_SQE_CDW(e, 10) >> 16) + 1U;
	uint32_t cdw11 = NVME_SQE_CDW(e, 11);
	uint16_t cqid = cdw11 >> 16;
	uint64_t addr = NVME_SQE_PRP1(e);
	struct nvme_queue *sq;

	if (qid == 0U || qid > MIN(cfg->io_queues, d->nsqa + 1U) || d->sq[qid].live) {
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;
	}
	if (qsize < 2U || qsize > cfg->mqes) {
		return NVME_SC_QUEUE_SIZE | NVME_STATUS_DNR;
	}
	if (cqid == 0U || cqid >= NR_QUEUES || !d->cq[cqid].live) {
		return NVME_SC_CQ_INVALID | NVME_STATUS_DNR;
	}
	if ((cdw11 & BIT(0)) == 0U || (addr & MPS_MASK) != 0U) {
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}

	sq = &d->sq[qid];
	*sq = (struct nvme_queue){
		.addr = addr,
		.db = NVME_SQ_DB(qid),
		.depth = qsize,
		.link = cqid,
		.live = true,
	};
	reg_wr(d, sq->db, 0);
	LOG_DBG("SQ %u: %u entries @0x%llx, CQ %u", qid, qsize, (unsigned long long)addr, cqid);
	return NVME_SC_SUCCESS;
}

static uint16_t adm_delete_sq(struct epf_nvme_data *d, const struct nvme_sqe *e)
{
	uint16_t qid = NVME_SQE_CDW(e, 10) & 0xffffU;

	if (qid == 0U || qid >= NR_QUEUES || !d->sq[qid].live) {
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;
	}
	/* commands run to completion before the next is fetched: none in flight */
	d->sq[qid].live = false;
	return NVME_SC_SUCCESS;
}

static uint16_t adm_delete_cq(struct epf_nvme_data *d, const struct nvme_sqe *e)
{
	uint16_t qid = NVME_SQE_CDW(e, 10) & 0xffffU;

	if (qid == 0U || qid >= NR_QUEUES || !d->cq[qid].live) {
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;
	}
	for (int i = 1; i < NR_QUEUES; i++) {
		if (d->sq[i].live && d->sq[i].link == qid) {
			return NVME_SC_QUEUE_DELETION | NVME_STATUS_DNR;
		}
	}
	d->cq[qid].live = false;
	return NVME_SC_SUCCESS;
}

static uint16_t adm_get_feat(struct epf_nvme_data *d, const struct nvme_sqe *e, uint32_t *dw0)
{
	uint8_t fid = NVME_SQE_CDW(e, 10) & 0xffU;
	uint8_t sel = (NVME_SQE_CDW(e, 10) >> 8) & 0x7U;
	uint32_t cdw11 = NVME_SQE_CDW(e, 11);

	if (sel == 3U) {
		/* supported capabilities: changeable, not saveable */
		*dw0 = BIT(2);
		return NVME_SC_SUCCESS;
	}
	if (sel == 2U) {
		/* nothing is saved: saved = default */
		sel = 1U;
	}

	switch (fid) {
	case NVME_FEAT_ARBITRATION:
		*dw0 = sel == 1U ? 0x7U : d->feat_arb;
		break;
	case NVME_FEAT_POWER_MGMT:
		*dw0 = 0;
		break;
	case NVME_FEAT_TEMP_THRESH:
		*dw0 = sel == 1U ? 343U : d->feat_temp;
		break;
	case NVME_FEAT_ERR_RECOVERY:
		*dw0 = sel == 1U ? 0U : d->feat_err;
		break;
	case NVME_FEAT_VOLATILE_WC:
		if (!IS_ENABLED(CONFIG_PCIE_EPF_NVME_VWC)) {
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		*dw0 = sel == 1U ? 1U : d->feat_vwc;
		break;
	case NVME_FEAT_NUM_QUEUES:
		*dw0 = ((uint32_t)d->ncqa << 16) | d->nsqa;
		break;
	case NVME_FEAT_IRQ_COALESCE:
		*dw0 = sel == 1U ? 0U : d->feat_coalesce;
		break;
	case NVME_FEAT_IRQ_CONFIG: {
		uint16_t iv = cdw11 & 0xffffU;

		if (iv >= d->max_vectors) {
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		*dw0 = iv | ((iv < 32U && sel == 0U && (d->irq_cd & BIT(iv)) != 0U) ? BIT(16)
										     : 0U);
		break;
	}
	case NVME_FEAT_WRITE_ATOMIC:
		*dw0 = sel == 1U ? 0U : d->feat_atomic;
		break;
	case NVME_FEAT_ASYNC_EVENT:
		*dw0 = sel == 1U ? 0U : d->feat_aec;
		break;
	default:
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	return NVME_SC_SUCCESS;
}

static uint16_t adm_set_feat(struct epf_nvme_data *d, const struct nvme_sqe *e, uint32_t *dw0)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint32_t cdw10 = NVME_SQE_CDW(e, 10);
	uint8_t fid = cdw10 & 0xffU;
	uint32_t cdw11 = NVME_SQE_CDW(e, 11);

	if ((cdw10 & BIT(31)) != 0U) {
		return NVME_SC_FEAT_NOT_SAVEABLE | NVME_STATUS_DNR;
	}

	switch (fid) {
	case NVME_FEAT_ARBITRATION:
		d->feat_arb = cdw11;
		break;
	case NVME_FEAT_POWER_MGMT:
		/* one power state */
		if ((cdw11 & 0x1fU) != 0U) {
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		break;
	case NVME_FEAT_TEMP_THRESH:
		d->feat_temp = cdw11 & 0xffffU;
		break;
	case NVME_FEAT_ERR_RECOVERY:
		d->feat_err = cdw11;
		break;
	case NVME_FEAT_VOLATILE_WC:
		if (!IS_ENABLED(CONFIG_PCIE_EPF_NVME_VWC)) {
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		d->feat_vwc = cdw11 & BIT(0);
		break;
	case NVME_FEAT_NUM_QUEUES: {
		uint16_t nsqr = cdw11 & 0xffffU;
		uint16_t ncqr = cdw11 >> 16;

		if (nsqr == 0xffffU || ncqr == 0xffffU) {
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		/* only while no I/O queue exists */
		for (int i = 1; i < NR_QUEUES; i++) {
			if (d->sq[i].live || d->cq[i].live) {
				return NVME_SC_FEAT_NOT_CHANGEABLE | NVME_STATUS_DNR;
			}
		}
		d->nsqa = MIN(nsqr, cfg->io_queues - 1U);
		d->ncqa = MIN(ncqr, cfg->io_queues - 1U);
		*dw0 = ((uint32_t)d->ncqa << 16) | d->nsqa;
		break;
	}
	case NVME_FEAT_IRQ_COALESCE:
		d->feat_coalesce = cdw11 & 0xffffU;
		break;
	case NVME_FEAT_IRQ_CONFIG: {
		uint16_t iv = cdw11 & 0xffffU;

		if (iv >= d->max_vectors) {
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		}
		if (iv < 32U) {
			WRITE_BIT(d->irq_cd, iv, (cdw11 & BIT(16)) != 0U);
		}
		break;
	}
	case NVME_FEAT_WRITE_ATOMIC:
		d->feat_atomic = cdw11 & BIT(0);
		break;
	case NVME_FEAT_ASYNC_EVENT:
		d->feat_aec = cdw11;
		break;
	default:
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	return NVME_SC_SUCCESS;
}

/*
 * Execute one admin command. Returns false when no completion is to be
 * posted now (Asynchronous Event Request, kept until there is an event).
 */
static bool admin_exec(struct epf_nvme_data *d, const struct nvme_sqe *e, uint16_t *status,
		       uint32_t *dw0)
{
	uint8_t opc = NVME_SQE_OPC(e);

	d->st.admin_cmds++;
	*dw0 = 0;
	if (NVME_SQE_FUSE(e) != 0U) {
		*status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		return true;
	}

	switch (opc) {
	case NVME_ADM_DELETE_SQ:
		*status = adm_delete_sq(d, e);
		break;
	case NVME_ADM_CREATE_SQ:
		*status = adm_create_sq(d, e);
		break;
	case NVME_ADM_GET_LOG:
		*status = adm_get_log(d, e);
		break;
	case NVME_ADM_DELETE_CQ:
		*status = adm_delete_cq(d, e);
		break;
	case NVME_ADM_CREATE_CQ:
		*status = adm_create_cq(d, e);
		break;
	case NVME_ADM_IDENTIFY:
		*status = adm_identify(d, e);
		break;
	case NVME_ADM_ABORT:
		/* commands are not queued here: nothing to abort */
		*dw0 = BIT(0);
		*status = NVME_SC_SUCCESS;
		break;
	case NVME_ADM_SET_FEAT:
		*status = adm_set_feat(d, e, dw0);
		break;
	case NVME_ADM_GET_FEAT:
		*status = adm_get_feat(d, e, dw0);
		break;
	case NVME_ADM_AER:
		if (d->aer_count > AERL) {
			*status = NVME_SC_AER_LIMIT | NVME_STATUS_DNR;
			break;
		}
		d->aer_cid[d->aer_count++] = NVME_SQE_CID(e);
		return false;
	case NVME_ADM_KEEP_ALIVE:
		*status = NVME_SC_SUCCESS;
		break;
	default:
		LOG_DBG("admin opcode 0x%02x not supported", opc);
		*status = NVME_SC_INVALID_OPCODE | NVME_STATUS_DNR;
		break;
	}
	if (*status != NVME_SC_SUCCESS) {
		LOG_DBG("admin opcode 0x%02x cdw10 %08x: status 0x%x", opc, NVME_SQE_CDW(e, 10),
			*status);
	}
	return true;
}

static uint16_t io_rw(struct epf_nvme_data *d, const struct nvme_ns *ns, const char *disk,
		     const struct nvme_sqe *e, bool write)
{
	uint64_t slba = NVME_SQE_CDW(e, 10) | ((uint64_t)NVME_SQE_CDW(e, 11) << 32);
	uint32_t nlb = (NVME_SQE_CDW(e, 12) & 0xffffU) + 1U;
	uint64_t len = (uint64_t)nlb << ns->lbads;
	uint16_t status;
	int ret;

	if (slba + nlb > ns->nsze || slba + nlb > (uint64_t)UINT32_MAX + 1U) {
		return NVME_SC_LBA_RANGE | NVME_STATUS_DNR;
	}
	if (len > MDTS_BYTES) {
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}

	if (write) {
		status = data_xfer(d, e, (uint32_t)len, false);
		if (status != NVME_SC_SUCCESS) {
			return status;
		}
		ret = disk_access_write(disk, d->buf, (uint32_t)slba, nlb);
		if (ret != 0) {
			LOG_WRN("%s: write lba %llu +%u: %d", disk, (unsigned long long)slba, nlb,
				ret);
			return NVME_SC_WRITE_FAULT;
		}
		d->st.writes++;
		d->st.bytes_written += len;
		return NVME_SC_SUCCESS;
	}

	ret = disk_access_read(disk, d->buf, (uint32_t)slba, nlb);
	if (ret != 0) {
		LOG_WRN("%s: read lba %llu +%u: %d", disk, (unsigned long long)slba, nlb, ret);
		return NVME_SC_READ_ERROR;
	}
	status = data_xfer(d, e, (uint32_t)len, true);
	if (status == NVME_SC_SUCCESS) {
		d->st.reads++;
		d->st.bytes_read += len;
	}
	return status;
}

static uint16_t io_write_zeroes(struct epf_nvme_data *d, const struct nvme_ns *ns,
			       const char *disk, const struct nvme_sqe *e)
{
	uint64_t slba = NVME_SQE_CDW(e, 10) | ((uint64_t)NVME_SQE_CDW(e, 11) << 32);
	uint32_t nlb = (NVME_SQE_CDW(e, 12) & 0xffffU) + 1U;
	uint32_t chunk = MDTS_BYTES >> ns->lbads;

	if (slba + nlb > ns->nsze || slba + nlb > (uint64_t)UINT32_MAX + 1U) {
		return NVME_SC_LBA_RANGE | NVME_STATUS_DNR;
	}
	memset(d->buf, 0, MIN((uint64_t)nlb << ns->lbads, MDTS_BYTES));
	while (nlb != 0U) {
		uint32_t n = MIN(nlb, chunk);
		int ret = disk_access_write(disk, d->buf, (uint32_t)slba, n);

		if (ret != 0) {
			LOG_WRN("%s: write zeroes lba %llu +%u: %d", disk, (unsigned long long)slba,
				n, ret);
			return NVME_SC_WRITE_FAULT;
		}
		slba += n;
		nlb -= n;
	}
	return NVME_SC_SUCCESS;
}

static uint16_t io_exec(struct epf_nvme_data *d, const struct nvme_sqe *e)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint8_t opc = NVME_SQE_OPC(e);
	uint32_t nsid = NVME_SQE_NSID(e);
	struct nvme_ns *ns;
	const char *disk;

	d->st.cmds++;
	if (NVME_SQE_FUSE(e) != 0U) {
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
	if (opc == NVME_CMD_FLUSH && nsid == UINT32_MAX) {
		/* every namespace */
		for (uint32_t i = 0; i < cfg->n_disks; i++) {
			if (d->ns[i].ready &&
			    disk_access_ioctl(cfg->disks[i], DISK_IOCTL_CTRL_SYNC, NULL) != 0) {
				return NVME_SC_WRITE_FAULT;
			}
		}
		return NVME_SC_SUCCESS;
	}
	/* a host normally identifies first; probe here too for one that did not */
	ns = ns_get(d, nsid);
	if (ns == NULL || !ns->ready) {
		return NVME_SC_INVALID_NS | NVME_STATUS_DNR;
	}
	disk = cfg->disks[nsid - 1U];

	switch (opc) {
	case NVME_CMD_FLUSH:
		if (disk_access_ioctl(disk, DISK_IOCTL_CTRL_SYNC, NULL) != 0) {
			return NVME_SC_WRITE_FAULT;
		}
		return NVME_SC_SUCCESS;
	case NVME_CMD_WRITE:
		return io_rw(d, ns, disk, e, true);
	case NVME_CMD_READ:
		return io_rw(d, ns, disk, e, false);
	case NVME_CMD_WRITE_ZEROES:
		return io_write_zeroes(d, ns, disk, e);
	default:
		return NVME_SC_INVALID_OPCODE | NVME_STATUS_DNR;
	}
}

/*
 * Controller state
 */
static void queues_reset(struct epf_nvme_data *d)
{
	memset(d->sq, 0, sizeof(d->sq));
	memset(d->cq, 0, sizeof(d->cq));
	d->aer_count = 0;
}

static void ctrl_disable(struct epf_nvme_data *d, bool shutdown)
{
	const struct epf_nvme_config *cfg = d->dev->config;

	if (d->enabled) {
		LOG_INF("%s controller", shutdown ? "shutting down" : "disabling");
		for (uint32_t i = 0; shutdown && i < cfg->n_disks; i++) {
			if (d->ns[i].ready) {
				(void)disk_access_ioctl(cfg->disks[i], DISK_IOCTL_CTRL_SYNC, NULL);
			}
		}
	}
	d->enabled = false;
	queues_reset(d);
	d->csts &= ~NVME_CSTS_RDY;
	if (shutdown) {
		d->csts |= NVME_CSTS_SHST_CMPLT;
		/*
		 * Clear CC.EN as the Linux target does: the host's next disable
		 * finds RDY already 0 and may set EN again before the next poll,
		 * which must still look like an enable.
		 */
		d->cc &= ~NVME_CC_EN;
		reg_wr(d, NVME_REG_CC, d->cc);
	}
}

static void ctrl_enable(struct epf_nvme_data *d)
{
	const struct epf_nvme_config *cfg = d->dev->config;
	uint32_t aqa = reg_rd(d, NVME_REG_AQA);
	uint64_t asq = reg_rd64(d, NVME_REG_ASQ);
	uint64_t acq = reg_rd64(d, NVME_REG_ACQ);
	uint16_t sqs = NVME_AQA_ASQS(aqa) + 1U;
	uint16_t cqs = NVME_AQA_ACQS(aqa) + 1U;

	if (NVME_CC_MPS(d->cc) != 0U || NVME_CC_CSS(d->cc) != 0U || NVME_CC_AMS(d->cc) != 0U ||
	    NVME_CC_IOSQES(d->cc) != NVME_SQES || NVME_CC_IOCQES(d->cc) != NVME_CQES ||
	    sqs < 2U || cqs < 2U || ((asq | acq) & MPS_MASK) != 0U) {
		LOG_ERR("bad configuration: CC %08x AQA %08x ASQ %llx ACQ %llx", d->cc, aqa,
			(unsigned long long)asq, (unsigned long long)acq);
		d->csts |= NVME_CSTS_CFS;
		/* as after a shutdown: the next CC.EN write is an enable again */
		d->cc &= ~NVME_CC_EN;
		reg_wr(d, NVME_REG_CC, d->cc);
		return;
	}

	queues_reset(d);
	d->cq[0] = (struct nvme_queue){
		.addr = acq,
		.db = NVME_CQ_DB(0),
		.depth = cqs,
		.link = 0,
		.phase = 1,
		.ien = true,
		.live = true,
	};
	d->sq[0] = (struct nvme_queue){
		.addr = asq,
		.db = NVME_SQ_DB(0),
		.depth = sqs,
		.link = 0,
		.live = true,
	};
	reg_wr(d, NVME_SQ_DB(0), 0);
	reg_wr(d, NVME_CQ_DB(0), 0);

	d->nsqa = cfg->io_queues - 1U;
	d->ncqa = cfg->io_queues - 1U;
	d->feat_arb = 0x7U;
	d->feat_temp = 343U;
	d->feat_err = 0;
	d->feat_vwc = 1U;
	d->feat_coalesce = 0;
	d->feat_atomic = 0;
	d->feat_aec = 0;
	d->irq_cd = 0;

	if (d->sn[0] == '\0') {
		ids_init(d);
	}

	d->enabled = true;
	d->csts = NVME_CSTS_RDY;
	d->st.enables++;
	LOG_INF("controller enabled: admin SQ %u @0x%llx, CQ %u @0x%llx", sqs,
		(unsigned long long)asq, cqs, (unsigned long long)acq);
}

/* Back to the power-on state after a link reset, the host is gone */
static void ctrl_reset(struct epf_nvme_data *d)
{
	const struct epf_nvme_config *cfg = d->dev->config;

	ctrl_disable(d, false);
	d->cc = 0;
	d->csts = 0;
	reg_wr64(d, NVME_REG_CAP, nvme_cap(cfg));
	reg_wr(d, NVME_REG_VS, NVME_VS(1, 4, 0));
	reg_wr(d, NVME_REG_CC, 0);
	reg_wr(d, NVME_REG_CSTS, 0);
	d->st.resets++;
}

static void poll_cc(struct epf_nvme_data *d)
{
	uint32_t cc = reg_rd(d, NVME_REG_CC);
	uint32_t old = d->cc;

	if (cc == old) {
		return;
	}
	d->cc = cc;

	if ((cc & NVME_CC_EN) != 0U && (old & NVME_CC_EN) == 0U) {
		d->csts &= ~(NVME_CSTS_CFS | NVME_CSTS_SHST_MASK);
		ctrl_enable(d);
	}
	if ((cc & NVME_CC_EN) == 0U && (old & NVME_CC_EN) != 0U) {
		ctrl_disable(d, false);
		d->csts &= ~(NVME_CSTS_CFS | NVME_CSTS_SHST_MASK);
	}
	if (NVME_CC_SHN(cc) != 0U && NVME_CC_SHN(old) == 0U) {
		ctrl_disable(d, true);
	}
	if (NVME_CC_SHN(cc) == 0U && NVME_CC_SHN(old) != 0U) {
		d->csts &= ~NVME_CSTS_SHST_MASK;
	}
	reg_wr(d, NVME_REG_CSTS, d->csts);
}

/* Fetch and execute up to one arbitration burst; returns commands done */
static int process_sq(struct epf_nvme_data *d, uint16_t qid)
{
	struct nvme_queue *sq = &d->sq[qid];
	uint32_t tail = reg_rd(d, sq->db);
	uint32_t burst = d->feat_arb & 0x7U;
	int done = 0;

	burst = burst == 7U ? sq->depth : MIN(BIT(burst), sq->depth);

	if (tail >= sq->depth) {
		/* invalid doorbell write, the host is misbehaving: ignore it */
		return 0;
	}
	sq->tail = (uint16_t)tail;

	while (sq->live && sq->head != sq->tail && (uint32_t)done < burst) {
		uint32_t n = (sq->tail > sq->head) ? sq->tail - sq->head : sq->depth - sq->head;

		n = MIN(n, MIN(burst - (uint32_t)done, (uint32_t)SQ_BURST));
		if (host_xfer(d, sq->addr + (uint64_t)sq->head * 64U, d->sqe, n * 64U, false) !=
		    0) {
			return done;
		}
		for (uint32_t i = 0; i < n; i++) {
			struct nvme_sqe *e = &d->sqe[i];
			uint16_t status;
			uint32_t dw0 = 0;
			bool post = true;

			for (int w = 0; w < 16; w++) {
				e->dw[w] = sys_le32_to_cpu(e->dw[w]);
			}
			sq->head = (sq->head + 1U) % sq->depth;

			if (qid == 0U) {
				post = admin_exec(d, e, &status, &dw0);
			} else {
				status = io_exec(d, e);
			}
			if (!d->enabled) {
				/* a command that touched the controller state */
				return done;
			}
			if (post && cq_post(d, sq, NVME_SQE_CID(e), status, dw0) != 0) {
				return done;
			}
			done++;
			/* the admin queue never coalesces */
			if (qid == 0U && d->cq[0].irq_pending) {
				raise_irq(d, &d->cq[0]);
			}
			if (!sq->live) {
				break;
			}
		}
	}
	return done;
}

static void nvme_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct epf_nvme_data *d = dev->data;
	int64_t last = 0;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		int busy = 0;

		if (atomic_cas(&d->reset_req, 1, 0)) {
			ctrl_reset(d);
		}
		bme_update(d);
		poll_cc(d);
		/* without bus mastering the queues are out of reach */
		if (!d->enabled || !d->bme) {
			k_sleep(K_MSEC(CONFIG_PCIE_EPF_NVME_CC_POLL_MS));
			continue;
		}

		for (uint16_t q = 0; q < NR_QUEUES && d->enabled; q++) {
			if (d->sq[q].live) {
				busy += process_sq(d, q);
			}
		}
		for (uint16_t q = 1; q < NR_QUEUES && d->enabled; q++) {
			if (d->cq[q].live && d->cq[q].irq_pending) {
				raise_irq(d, &d->cq[q]);
			}
		}

		/*
		 * Busy poll while commands come, then back off; the same idea as
		 * the Linux target's SQ polling work.
		 */
		if (busy != 0) {
			last = k_uptime_get();
			k_yield();
		} else if (k_uptime_get() - last < CONFIG_PCIE_EPF_NVME_BUSY_POLL_MS) {
			k_yield();
		} else {
			k_usleep(CONFIG_PCIE_EPF_NVME_IDLE_POLL_US);
		}
	}
}

static void epf_nvme_event(const struct device *epc, struct pcie_epc_listener *l,
			   enum pcie_epc_event evt, uint8_t func)
{
	struct epf_nvme_data *d = CONTAINER_OF(l, struct epf_nvme_data, listener);

	ARG_UNUSED(epc);
	ARG_UNUSED(func);

	switch (evt) {
	case PCIE_EPC_EVT_LINK_UP:
		LOG_INF("link up");
		d->link = true;
		break;
	case PCIE_EPC_EVT_LINK_DOWN:
	case PCIE_EPC_EVT_HOT_RESET:
	case PCIE_EPC_EVT_FLR:
	case PCIE_EPC_EVT_PERST_ASSERT:
		/* host reboot: CC.EN still set in the BAR must not survive it */
		if (evt == PCIE_EPC_EVT_LINK_DOWN) {
			LOG_INF("link down");
			d->link = false;
		}
		atomic_set(&d->reset_req, 1);
		break;
	default:
		break;
	}
}

static int epf_nvme_setup_bar(const struct device *dev)
{
	const struct epf_nvme_config *cfg = dev->config;
	struct epf_nvme_data *d = dev->data;
	const struct pcie_epc_features *feat = pcie_epc_get_features(cfg->epc, cfg->func);
	size_t regs = ROUND_UP(NVME_REG_DBS + (size_t)NR_QUEUES * 8U, 8);
	size_t want = regs;
	size_t sz;
	uintptr_t base;
	int ret;

	if (feat != NULL && feat->msix_capable && cfg->msix_irqs != 0U) {
		/* MSI-X table and PBA after the doorbells */
		d->msix_offset = (uint32_t)ROUND_UP(regs, 0x1000);
		want = d->msix_offset + cfg->msix_irqs * 16U +
		       DIV_ROUND_UP(cfg->msix_irqs, 64) * 8U;
	}
	sz = pcie_epf_bar_size(feat, 0, MAX(want, 0x4000));
	if (sz == 0) {
		LOG_ERR("BAR0 not available");
		return -EINVAL;
	}

	base = ROUND_UP(cfg->mem_base, MAX(sz, feat != NULL ? feat->align : 256));
	if (base + sz > cfg->mem_base + cfg->mem_size) {
		LOG_ERR("memory-region too small for BAR0 (0x%zx)", sz);
		return -ENOMEM;
	}
	d->regs = (volatile uint8_t *)(d->mem_virt + (base - cfg->mem_base));
	memset((void *)d->regs, 0, sz);

	/* NVMe 1.1 and later: BAR0 is 64-bit */
	d->bar = (struct pcie_epc_bar){
		.barno = 0,
		.flags = PCIE_EPC_BAR_MEM64,
		.phys = pcie_epc_cpu_to_axi(cfg->epc, base),
		.size = sz,
		.virt = (void *)d->regs,
	};
	ret = pcie_epc_set_bar(cfg->epc, cfg->func, &d->bar);
	if (ret != 0) {
		LOG_ERR("set_bar 0 failed (%d)", ret);
		return ret;
	}
	/* BAR0 only: whatever the controller enables at reset goes away */
	for (uint8_t b = 1; b < PCIE_EPC_NUM_BARS; b++) {
		if (feat == NULL || (feat->reserved_bar & BIT(b)) == 0U) {
			pcie_epc_clear_bar(cfg->epc, cfg->func, b);
		}
	}

	/*
	 * Interrupt vectors a CQ may name: the MSI-X table size, else the MSI
	 * vectors. MSI stays configured with MSI-X for hosts that use it
	 * (fewer messages alias), INTx is the last resort.
	 */
	d->max_vectors = 1;
	if (cfg->msi_irqs != 0U && pcie_epc_set_msi(cfg->epc, cfg->func, cfg->msi_irqs) == 0) {
		d->max_vectors = cfg->msi_irqs;
	}
	if (d->msix_offset != 0U &&
	    pcie_epc_set_msix(cfg->epc, cfg->func, cfg->msix_irqs, 0, d->msix_offset) == 0) {
		d->max_vectors = cfg->msix_irqs;
	} else {
		d->msix_offset = 0;
	}
	return 0;
}

#if defined(CONFIG_PCIE_EPF_NVME_SHELL)
#include <zephyr/shell/shell.h>

static const struct device *shell_inst[2];
static int n_shell_inst;

static void shell_add(const struct device *dev)
{
	if (n_shell_inst < ARRAY_SIZE(shell_inst)) {
		shell_inst[n_shell_inst++] = dev;
	}
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (int i = 0; i < n_shell_inst; i++) {
		const struct device *dev = shell_inst[i];
		const struct epf_nvme_config *cfg = dev->config;
		struct epf_nvme_data *d = dev->data;
		const struct epf_nvme_stats *s = &d->st;

		shell_print(sh, "%s: fn%u on %s, BAR0 %zu KiB, %u vectors%s, SN %s", dev->name,
			    cfg->func, cfg->epc->name, d->bar.size / 1024U,
			    d->max_vectors, d->msix_offset != 0U ? " (MSI-X)" : "", d->sn);
		shell_print(sh, "  link %s, bus master %s, CC %08x CSTS %08x AQA %08x, %s",
			    d->link ? "up" : "down", d->bme ? "on" : "off",
			    reg_rd(d, NVME_REG_CC), reg_rd(d, NVME_REG_CSTS),
			    reg_rd(d, NVME_REG_AQA), d->enabled ? "enabled" : "disabled");
		for (uint32_t n = 0; n < cfg->n_disks; n++) {
			const struct nvme_ns *ns = &d->ns[n];

			if (ns->ready) {
				shell_print(sh, "  ns %u: %s, %llu x %u B (%llu MiB)", n + 1U,
					    cfg->disks[n], (unsigned long long)ns->nsze,
					    ns->lba_size,
					    (unsigned long long)((ns->nsze * ns->lba_size) >> 20));
			} else {
				shell_print(sh, "  ns %u: %s, inactive", n + 1U, cfg->disks[n]);
			}
		}
		for (int q = 0; q < NR_QUEUES; q++) {
			const struct nvme_queue *sq = &d->sq[q], *cq = &d->cq[q];

			if (cq->live) {
				shell_print(sh, "  CQ %d: %u @%llx head %u tail %u ph %u iv %u%s",
					    q, cq->depth, (unsigned long long)cq->addr, cq->head,
					    cq->tail, cq->phase, cq->link, cq->ien ? "" : " noirq");
			}
			if (sq->live) {
				shell_print(sh, "  SQ %d: %u @%llx head %u tail %u db %u -> CQ %u",
					    q, sq->depth, (unsigned long long)sq->addr, sq->head,
					    sq->tail, reg_rd(d, sq->db), sq->link);
			}
		}
		shell_print(sh, "  admin %llu, io %llu (r %llu / %llu B, w %llu / %llu B), "
			    "errors %llu", s->admin_cmds, s->cmds, s->reads, s->bytes_read,
			    s->writes, s->bytes_written, s->errors);
		shell_print(sh, "  irqs %llu (failed %llu), transfer errors %llu, enables %u, "
			    "link resets %u, AERs held %u", s->irqs, s->irq_errors,
			    s->xfer_errors, s->enables, s->resets, d->aer_count);
	}
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (int i = 0; i < n_shell_inst; i++) {
		struct epf_nvme_data *d = shell_inst[i]->data;

		memset(&d->st, 0, sizeof(d->st));
	}
	shell_print(sh, "statistics cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_nvme_epf,
	SHELL_CMD(status, NULL, "Controller, queues and statistics", cmd_status),
	SHELL_CMD(clear, NULL, "Clear the statistics", cmd_reset),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(nvme_epf, &sub_nvme_epf, "NVMe endpoint function", cmd_status);
#else
static inline void shell_add(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif

static int epf_nvme_init(const struct device *dev)
{
	const struct epf_nvme_config *cfg = dev->config;
	struct epf_nvme_data *d = dev->data;
	struct pcie_epc_header hdr;
	uint32_t cls = 0;
	int ret;

	d->dev = dev;
	d->boot_ms = k_uptime_get();

	/* another function may be the one selected for this boot */
	if (!pcie_epf_selected("nvme")) {
		return -ENODEV;
	}

	if (!device_is_ready(cfg->epc)) {
#if defined(CONFIG_PCIE_AARDVARK_MODE)
		if (advk_mode_get() == ADVK_MODE_RC) {
			LOG_INF("%s: root complex mode, function off", dev->name);
			return -ENODEV;
		}
#endif
		LOG_ERR("controller %s not ready", cfg->epc->name);
		return -ENODEV;
	}
	if (cfg->io_queues == 0U || cfg->io_queues > CONFIG_PCIE_EPF_NVME_MAX_IO_QUEUES ||
	    cfg->mqes < 2U) {
		LOG_ERR("io-queues 1..%d, max-queue-entries >= 2",
			CONFIG_PCIE_EPF_NVME_MAX_IO_QUEUES);
		return -EINVAL;
	}


	ret = pcie_epc_claim_function(cfg->epc, cfg->func);
	if (ret != 0) {
		LOG_ERR("function %u: claim failed (%d)", cfg->func, ret);
		return ret;
	}

	/* the class code is what the host driver binds to, whatever DT says */
	hdr = cfg->hdr;
	hdr.baseclass = (uint8_t)(NVME_CLASS >> 16);
	hdr.subclass = (uint8_t)(NVME_CLASS >> 8);
	hdr.prog_if = (uint8_t)NVME_CLASS;
	ret = pcie_epc_write_header(cfg->epc, cfg->func, &hdr);
	if (ret != 0) {
		return ret;
	}
	/* a saved identity override of the controller may replace the class */
	if (pcie_epc_cfg_read(cfg->epc, cfg->func, CFG_CLASS, &cls) == 0 &&
	    (cls >> 8) != NVME_CLASS) {
		LOG_WRN("class code is %06x, not %06x: the host NVMe driver will not bind",
			cls >> 8, NVME_CLASS);
	}

	if (cfg->mem_size != 0U) {
#ifdef CONFIG_MMU
		/*
		 * Same as the test function: the carveout is outside the kernel
		 * image and may sit at physical 0, so no 1:1 device_map().
		 */
		k_mem_map_phys_bare((uint8_t **)&d->mem_virt, cfg->mem_base, cfg->mem_size,
				    K_MEM_PERM_RW |
#ifdef K_MEM_ARM_NORMAL_NC
				    K_MEM_ARM_NORMAL_NC
#else
				    K_MEM_CACHE_NONE
#endif
		);
#else
		d->mem_virt = cfg->mem_base;
#endif
	}

	ret = epf_nvme_setup_bar(dev);
	if (ret != 0) {
		return ret;
	}
	reg_wr64(d, NVME_REG_CAP, nvme_cap(cfg));
	reg_wr(d, NVME_REG_VS, NVME_VS(1, 4, 0));

	d->listener.func = cfg->func;
	d->listener.event = epf_nvme_event;
	pcie_epc_add_listener(cfg->epc, &d->listener);
	shell_add(dev);

	k_thread_create(&d->thread, d->stack, K_KERNEL_STACK_SIZEOF(d->stack), cfg->thread_fn,
			(void *)dev, NULL, NULL, CONFIG_PCIE_EPF_NVME_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&d->thread, dev->name);

	LOG_INF("%s: fn%u %04x:%04x on %s, %u namespaces, %u I/O queues x %u, MDTS %u KiB",
		dev->name, cfg->func, cfg->hdr.vendor_id, cfg->hdr.device_id, cfg->epc->name,
		cfg->n_disks, cfg->io_queues, cfg->mqes, CONFIG_PCIE_EPF_NVME_MDTS_KB);
	return 0;
}

/*
 * Without memory-region (targets where any RAM is reachable from the
 * controller), the BAR lives in a static buffer.
 */
#define EPF_NVME_MEM(n)                                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region),                                       \
		    (.mem_base = DT_REG_ADDR(DT_INST_PHANDLE(n, memory_region)),                   \
		     .mem_size = DT_REG_SIZE(DT_INST_PHANDLE(n, memory_region)),),                 \
		    (.mem_base = (uintptr_t)epf_nvme_bar_##n,                                      \
		     .mem_size = sizeof(epf_nvme_bar_##n),))

#define EPF_NVME_BAR_BUF(n)                                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region), (),                                   \
		    (static uint8_t epf_nvme_bar_##n[0x8000] __aligned(0x8000);))

#define EPF_NVME_DEFINE(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, disk_names) <= MAX_NS,                                    \
		     "more disk-names than CONFIG_PCIE_EPF_NVME_MAX_NAMESPACES");                  \
	EPF_NVME_BAR_BUF(n)                                                                        \
	static const char *const epf_nvme_disks_##n[] = DT_INST_PROP(n, disk_names);              \
	static void epf_nvme_thread_##n(void *p1, void *p2, void *p3)                              \
	{                                                                                          \
		nvme_thread(p1, p2, p3);                                                           \
	}                                                                                          \
	static const struct epf_nvme_config epf_nvme_cfg_##n = {                                   \
		.epc = PCIE_EPF_DT_INST_EPC(n),                                                    \
		.func = PCIE_EPF_DT_INST_FUNC(n),                                                  \
		.hdr = PCIE_EPF_DT_HEADER(DT_DRV_INST(n)),                                         \
		EPF_NVME_MEM(n)                                                                    \
		.disks = epf_nvme_disks_##n,                                                       \
		.n_disks = DT_INST_PROP_LEN(n, disk_names),                                        \
		.serial = DT_INST_PROP_OR(n, serial_number, NULL),                                 \
		.model = DT_INST_PROP(n, model_number),                                            \
		.io_queues = DT_INST_PROP(n, io_queues),                                           \
		.mqes = DT_INST_PROP(n, max_queue_entries),                                        \
		.msix_irqs = DT_INST_PROP(n, msix_interrupts),                                     \
		.msi_irqs = DT_INST_PROP(n, msi_interrupts),                                       \
		.thread_fn = epf_nvme_thread_##n,                                                  \
	};                                                                                         \
	static struct epf_nvme_data epf_nvme_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, epf_nvme_init, NULL, &epf_nvme_data_##n, &epf_nvme_cfg_##n,       \
			      POST_KERNEL, CONFIG_PCIE_EPF_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(EPF_NVME_DEFINE)
