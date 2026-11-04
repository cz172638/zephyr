/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NVMe endpoint function against a fake controller: the test is the host
 * driver. It programs BAR0, places queues and data in the simulated host
 * memory, rings doorbells and checks completions, data on the RAM disks
 * and the interrupts raised.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>

#include "fake_epc.h"
#include "pcie_epf_nvme.h"

/* host memory layout */
#define ASQ    (HOST_BASE + 0x0000)
#define ACQ    (HOST_BASE + 0x1000)
#define IOSQ1  (HOST_BASE + 0x2000)
#define IOCQ1  (HOST_BASE + 0x3000)
#define IDBUF  (HOST_BASE + 0x4000)
#define PRPL   (HOST_BASE + 0x5000)
#define PRPL2  (HOST_BASE + 0x6000)
#define DATA   (HOST_BASE + 0x10000)
#define DATA2  (HOST_BASE + 0x30000)

#define ADMIN_DEPTH 8
#define IO_DEPTH    4

struct hq {
	uint64_t base;
	uint16_t qid;
	uint16_t depth;
	uint16_t idx;
	uint8_t phase;
};

static struct hq asq, acq, sq1, cq1;
static uint16_t next_cid;

static volatile uint32_t *bar(uint32_t off)
{
	return (volatile uint32_t *)((uint8_t *)fake_bar0.virt + off);
}

static uint32_t rd(uint32_t off)
{
	return *bar(off);
}

static void wr(uint32_t off, uint32_t v)
{
	*bar(off) = v;
}

static bool wait_csts(uint32_t mask, uint32_t val)
{
	for (int i = 0; i < 1000; i++) {
		if ((rd(NVME_REG_CSTS) & mask) == val) {
			return true;
		}
		k_sleep(K_MSEC(1));
	}
	return false;
}

/* submit, wait for the completion; returns the status field (no phase) */
static int submit(struct hq *sq, struct hq *cq, struct nvme_sqe *e, uint32_t *dw0)
{
	struct nvme_cqe *c;
	uint16_t cid = next_cid++;

	e->dw[0] = (e->dw[0] & 0xffffU) | ((uint32_t)cid << 16);
	memcpy(host_ptr(sq->base + sq->idx * 64U), e, 64);
	sq->idx = (sq->idx + 1U) % sq->depth;
	wr(NVME_SQ_DB(sq->qid), sq->idx);

	c = host_ptr(cq->base + cq->idx * 16U);
	for (int i = 0; i < 2000; i++) {
		if ((c->status & 1U) == cq->phase) {
			int status = c->status >> 1;

			zassert_equal(c->cid, cid, "cid %u, want %u", c->cid, cid);
			zassert_equal(c->sqid, sq->qid);
			zassert_equal(c->sqhd, sq->idx, "sqhd %u, want %u", c->sqhd, sq->idx);
			if (dw0 != NULL) {
				*dw0 = c->dw0;
			}
			cq->idx++;
			if (cq->idx == cq->depth) {
				cq->idx = 0;
				cq->phase ^= 1U;
			}
			wr(NVME_CQ_DB(cq->qid), cq->idx);
			return status;
		}
		k_sleep(K_MSEC(1));
	}
	return -ETIMEDOUT;
}

static int admin(struct nvme_sqe *e, uint32_t *dw0)
{
	return submit(&asq, &acq, e, dw0);
}

static int io(struct nvme_sqe *e)
{
	return submit(&sq1, &cq1, e, NULL);
}

static void ctrl_off(void)
{
	wr(NVME_REG_CC, 0);
	zassert_true(wait_csts(NVME_CSTS_RDY, 0), "RDY stays set");
}

static void ctrl_on(void)
{
	ctrl_off();
	memset(fake_host_mem, 0, 0x10000);
	asq = (struct hq){.base = ASQ, .qid = 0, .depth = ADMIN_DEPTH};
	acq = (struct hq){.base = ACQ, .qid = 0, .depth = ADMIN_DEPTH, .phase = 1};
	wr(NVME_REG_AQA, ((ADMIN_DEPTH - 1U) << 16) | (ADMIN_DEPTH - 1U));
	wr(NVME_REG_ASQ, (uint32_t)ASQ);
	wr(NVME_REG_ASQ + 4, (uint32_t)(ASQ >> 32));
	wr(NVME_REG_ACQ, (uint32_t)ACQ);
	wr(NVME_REG_ACQ + 4, (uint32_t)(ACQ >> 32));
	wr(NVME_REG_CC, NVME_CC_EN | (NVME_SQES << 16) | (NVME_CQES << 20));
	zassert_true(wait_csts(NVME_CSTS_RDY, NVME_CSTS_RDY), "no RDY: CC %08x CSTS %08x",
		     rd(NVME_REG_CC), rd(NVME_REG_CSTS));
}

static struct nvme_sqe cmd(uint8_t opc, uint32_t nsid)
{
	struct nvme_sqe e = {0};

	e.dw[0] = opc;
	e.dw[1] = nsid;
	return e;
}

static void set_prp(struct nvme_sqe *e, uint64_t prp1, uint64_t prp2)
{
	e->dw[6] = (uint32_t)prp1;
	e->dw[7] = (uint32_t)(prp1 >> 32);
	e->dw[8] = (uint32_t)prp2;
	e->dw[9] = (uint32_t)(prp2 >> 32);
}

static void io_queues_up(void)
{
	struct nvme_sqe e;
	uint32_t dw0;

	e = cmd(NVME_ADM_SET_FEAT, 0);
	e.dw[10] = NVME_FEAT_NUM_QUEUES;
	e.dw[11] = (7U << 16) | 7U;
	zassert_equal(admin(&e, &dw0), 0);
	/* io-queues = 3 */
	zassert_equal(dw0, (2U << 16) | 2U, "granted %08x", dw0);

	e = cmd(NVME_ADM_CREATE_CQ, 0);
	set_prp(&e, IOCQ1, 0);
	e.dw[10] = ((IO_DEPTH - 1U) << 16) | 1U;
	e.dw[11] = (1U << 16) | BIT(1) | BIT(0); /* vector 1 */
	zassert_equal(admin(&e, NULL), 0);

	e = cmd(NVME_ADM_CREATE_SQ, 0);
	set_prp(&e, IOSQ1, 0);
	e.dw[10] = ((IO_DEPTH - 1U) << 16) | 1U;
	e.dw[11] = (1U << 16) | BIT(0);
	zassert_equal(admin(&e, NULL), 0);

	sq1 = (struct hq){.base = IOSQ1, .qid = 1, .depth = IO_DEPTH};
	cq1 = (struct hq){.base = IOCQ1, .qid = 1, .depth = IO_DEPTH, .phase = 1};
}

static void *setup(void)
{
	/* BAR0 is set while the function initialises */
	zassert_not_null(fake_bar0.virt);
	return NULL;
}

static void before(void *f)
{
	ARG_UNUSED(f);
	fake_bme = true;
	ctrl_on();
}

ZTEST(epf_nvme, test_header_and_registers)
{
	uint64_t cap = rd(NVME_REG_CAP) | ((uint64_t)rd(NVME_REG_CAP + 4) << 32);

	zassert_equal(fake_hdr.baseclass, 0x01);
	zassert_equal(fake_hdr.subclass, 0x08);
	zassert_equal(fake_hdr.prog_if, 0x02);
	zassert_equal(fake_hdr.vendor_id, 0x11ab);
	zassert_true(fake_bar0.flags & PCIE_EPC_BAR_MEM64);

	zassert_equal(cap & 0xffff, 63, "MQES");
	zassert_true(cap & NVME_CAP_CQR);
	zassert_true(cap & NVME_CAP_CSS_NVM);
	zassert_equal((cap >> 32) & 0xf, 0, "DSTRD");
	zassert_equal(rd(NVME_REG_VS), NVME_VS(1, 4, 0));
	zassert_equal(rd(NVME_REG_CSTS), NVME_CSTS_RDY);
}

ZTEST(epf_nvme, test_identify)
{
	struct nvme_sqe e = cmd(NVME_ADM_IDENTIFY, 0);
	uint8_t *p = host_ptr(IDBUF);
	uint32_t *list = host_ptr(IDBUF);
	char sn[21] = {0};

	/* identify controller, buffer across a page: PRP1 offset + PRP2 */
	set_prp(&e, IDBUF + 0x800, IDBUF + 0x1000);
	e.dw[10] = NVME_CNS_CTRL;
	zassert_equal(admin(&e, NULL), 0);
	p += 0x800;
	zassert_equal(sys_get_le16(&p[NVME_IDC_VID]), 0x11ab);
	memcpy(sn, &p[NVME_IDC_SN], 20);
	zassert_mem_equal(sn, "0123456789abcdef    ", 20, "SN '%s'", sn);
	zassert_equal(p[NVME_IDC_MDTS], 3, "32 KiB / 4 KiB");
	zassert_equal(sys_get_le32(&p[NVME_IDC_NN]), 3);
	zassert_equal(p[NVME_IDC_SQES], 0x66);
	zassert_equal(p[NVME_IDC_CQES], 0x44);
	zassert_equal(sys_get_le32(&p[NVME_IDC_VER]), NVME_VS(1, 4, 0));
	zassert_true(strncmp((char *)&p[NVME_IDC_SUBNQN], "nqn.2014-08.org.zephyrproject:", 30) ==
		     0);

	/* active namespaces: 1 and 2, the third disk does not exist */
	e = cmd(NVME_ADM_IDENTIFY, 0);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = NVME_CNS_NS_ACTIVE_LIST;
	zassert_equal(admin(&e, NULL), 0);
	zassert_equal(list[0], 1);
	zassert_equal(list[1], 2);
	zassert_equal(list[2], 0);

	p = host_ptr(IDBUF);
	e = cmd(NVME_ADM_IDENTIFY, 1);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = NVME_CNS_NS;
	zassert_equal(admin(&e, NULL), 0);
	zassert_equal(sys_get_le64(&p[NVME_IDN_NSZE]), 512);
	zassert_equal(p[NVME_IDN_LBAF0 + 2], 9);
	zassert_equal(sys_get_be64(&p[NVME_IDN_NGUID]), FAKE_DSN);
	zassert_equal(sys_get_be32(&p[NVME_IDN_NGUID + 12]), 1);

	e = cmd(NVME_ADM_IDENTIFY, 2);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = NVME_CNS_NS;
	zassert_equal(admin(&e, NULL), 0);
	zassert_equal(sys_get_le64(&p[NVME_IDN_NSZE]), 32);
	zassert_equal(p[NVME_IDN_LBAF0 + 2], 12);

	/* allocated, inactive: zeros */
	e = cmd(NVME_ADM_IDENTIFY, 3);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = NVME_CNS_NS;
	zassert_equal(admin(&e, NULL), 0);
	zassert_equal(sys_get_le64(&p[NVME_IDN_NSZE]), 0);

	e = cmd(NVME_ADM_IDENTIFY, 4);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = NVME_CNS_NS;
	zassert_equal(admin(&e, NULL), NVME_SC_INVALID_NS | NVME_STATUS_DNR);

	/* descriptors: NGUID, CSI */
	e = cmd(NVME_ADM_IDENTIFY, 2);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = NVME_CNS_NS_DESC_LIST;
	zassert_equal(admin(&e, NULL), 0);
	zassert_equal(p[0], NVME_NIDT_NGUID);
	zassert_equal(p[1], 16);
	zassert_equal(sys_get_be32(&p[4 + 12]), 2);
	zassert_equal(p[20], NVME_NIDT_CSI);
	zassert_equal(p[24], NVME_CSI_NVM);

	e = cmd(NVME_ADM_IDENTIFY, 0);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = 0x10;
	zassert_equal(admin(&e, NULL), NVME_SC_INVALID_FIELD | NVME_STATUS_DNR);
}

ZTEST(epf_nvme, test_admin_errors)
{
	struct nvme_sqe e = cmd(0x7f, 0);
	uint32_t dw0;

	zassert_equal(admin(&e, NULL), NVME_SC_INVALID_OPCODE | NVME_STATUS_DNR);

	/* SQ on a CQ that does not exist */
	e = cmd(NVME_ADM_CREATE_SQ, 0);
	set_prp(&e, IOSQ1, 0);
	e.dw[10] = (3U << 16) | 1U;
	e.dw[11] = (2U << 16) | BIT(0);
	zassert_equal(admin(&e, NULL), NVME_SC_CQ_INVALID | NVME_STATUS_DNR);

	/* queue beyond io-queues */
	e = cmd(NVME_ADM_CREATE_CQ, 0);
	set_prp(&e, IOCQ1, 0);
	e.dw[10] = (3U << 16) | 4U;
	e.dw[11] = BIT(0);
	zassert_equal(admin(&e, NULL), NVME_SC_QID_INVALID | NVME_STATUS_DNR);

	/* too deep */
	e.dw[10] = (64U << 16) | 1U;
	zassert_equal(admin(&e, NULL), NVME_SC_QUEUE_SIZE | NVME_STATUS_DNR);

	/* vector beyond the MSI-X table */
	e.dw[10] = (3U << 16) | 1U;
	e.dw[11] = (8U << 16) | BIT(1) | BIT(0);
	zassert_equal(admin(&e, NULL), NVME_SC_INVALID_VECTOR | NVME_STATUS_DNR);

	/* CQ in use by an SQ */
	io_queues_up();
	e = cmd(NVME_ADM_DELETE_CQ, 0);
	e.dw[10] = 1;
	zassert_equal(admin(&e, NULL), NVME_SC_QUEUE_DELETION | NVME_STATUS_DNR);
	e = cmd(NVME_ADM_DELETE_SQ, 0);
	e.dw[10] = 1;
	zassert_equal(admin(&e, NULL), 0);
	e = cmd(NVME_ADM_DELETE_CQ, 0);
	e.dw[10] = 1;
	zassert_equal(admin(&e, NULL), 0);

	e = cmd(NVME_ADM_GET_LOG, 0);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = (127U << 16) | 0x7e;
	zassert_equal(admin(&e, NULL), NVME_SC_INVALID_LOG_PAGE | NVME_STATUS_DNR);

	e = cmd(NVME_ADM_GET_FEAT, 0);
	e.dw[10] = NVME_FEAT_NUM_QUEUES;
	zassert_equal(admin(&e, &dw0), 0);
	zassert_equal(dw0, (2U << 16) | 2U);

	e = cmd(NVME_ADM_SET_FEAT, 0);
	e.dw[10] = BIT(31) | NVME_FEAT_ARBITRATION;
	zassert_equal(admin(&e, NULL), NVME_SC_FEAT_NOT_SAVEABLE | NVME_STATUS_DNR);
}

static void fill(uint8_t *p, size_t len, uint8_t seed)
{
	for (size_t i = 0; i < len; i++) {
		p[i] = (uint8_t)(seed + i * 7U + (i >> 9));
	}
}

ZTEST(epf_nvme, test_read_write_prp_list)
{
	static uint8_t disk[16384];
	uint32_t irq0 = fake_irq_count[1];
	struct nvme_sqe e;
	uint64_t *prpl = host_ptr(PRPL);
	/* 12 KiB starting 0x200 into a page: 4 pages, scattered, through a list */
	uint64_t pages[4] = {DATA + 0x5200, DATA + 0x2000, DATA + 0x9000, DATA + 0xa000};
	uint8_t *src[4] = {host_ptr(pages[0]), host_ptr(pages[1]), host_ptr(pages[2]),
			   host_ptr(pages[3])};
	size_t len[4] = {0xe00, 0x1000, 0x1000, 0x200};

	io_queues_up();

	for (int i = 0; i < 4; i++) {
		fill(src[i], len[i], (uint8_t)(i * 31));
	}
	prpl[0] = pages[1];
	prpl[1] = pages[2];
	prpl[2] = pages[3];

	e = cmd(NVME_CMD_WRITE, 1);
	set_prp(&e, pages[0], PRPL);
	e.dw[10] = 100; /* SLBA */
	e.dw[12] = 24 - 1; /* 24 x 512 B */
	zassert_equal(io(&e), 0);

	zassert_ok(disk_access_read("RAM0", disk, 100, 24));
	for (int i = 0, off = 0; i < 4; off += len[i], i++) {
		zassert_mem_equal(&disk[off], src[i], len[i], "page %d", i);
	}

	/* and back, one contiguous buffer (pages merge into one segment) */
	memset(host_ptr(DATA2), 0, 0x4000);
	prpl[0] = DATA2 + 0x1000;
	prpl[1] = DATA2 + 0x2000;
	e = cmd(NVME_CMD_READ, 1);
	set_prp(&e, DATA2, PRPL);
	e.dw[10] = 100;
	e.dw[12] = 24 - 1;
	zassert_equal(io(&e), 0);
	zassert_mem_equal(host_ptr(DATA2), disk, 12288);

	zassert_true(fake_irq_count[1] > irq0, "no MSI-X on vector 1");
}

ZTEST(epf_nvme, test_chained_prp_list)
{
	static uint8_t disk[32768];
	uint64_t *l1 = host_ptr(PRPL + 0xff0);	/* two entries left in the page */
	uint64_t *l2 = host_ptr(PRPL2);
	struct nvme_sqe e;

	io_queues_up();

	/* 32 KiB = MDTS: 8 pages, every other one, list chains after one entry */
	for (int i = 0; i < 8; i++) {
		fill(host_ptr(DATA + i * 0x2000), 0x1000, (uint8_t)(i + 1));
	}
	l1[0] = DATA + 0x2000;
	l1[1] = PRPL2; /* chain */
	for (int i = 2; i < 8; i++) {
		l2[i - 2] = DATA + i * 0x2000;
	}

	e = cmd(NVME_CMD_WRITE, 2);
	set_prp(&e, DATA, PRPL + 0xff0);
	e.dw[10] = 4;
	e.dw[12] = 8 - 1; /* 8 x 4 KiB */
	zassert_equal(io(&e), 0);

	zassert_ok(disk_access_read("RAM1", disk, 4, 8));
	for (int i = 0; i < 8; i++) {
		zassert_mem_equal(&disk[i * 0x1000], host_ptr(DATA + i * 0x2000), 0x1000,
				  "page %d", i);
	}

	/* larger than MDTS */
	e.dw[12] = 9 - 1;
	zassert_equal(io(&e), NVME_SC_INVALID_FIELD | NVME_STATUS_DNR);
}

ZTEST(epf_nvme, test_io_errors_flush_zeroes)
{
	static uint8_t disk[4096];
	struct nvme_sqe e;

	io_queues_up();

	e = cmd(NVME_CMD_READ, 1);
	set_prp(&e, DATA, 0);
	e.dw[10] = 510;
	e.dw[12] = 3 - 1;
	zassert_equal(io(&e), NVME_SC_LBA_RANGE | NVME_STATUS_DNR);

	e = cmd(NVME_CMD_READ, 3);
	set_prp(&e, DATA, 0);
	zassert_equal(io(&e), NVME_SC_INVALID_NS | NVME_STATUS_DNR);

	e = cmd(0x7f, 1);
	zassert_equal(io(&e), NVME_SC_INVALID_OPCODE | NVME_STATUS_DNR);

	/* PRP1 not dword aligned */
	e = cmd(NVME_CMD_READ, 1);
	set_prp(&e, DATA + 2, 0);
	zassert_equal(io(&e), NVME_SC_PRP_INVALID_OFFSET | NVME_STATUS_DNR);

	memset(disk, 0x5a, sizeof(disk));
	zassert_ok(disk_access_write("RAM0", disk, 200, 8));
	e = cmd(NVME_CMD_WRITE_ZEROES, 1);
	e.dw[10] = 201;
	e.dw[12] = 6 - 1;
	zassert_equal(io(&e), 0);
	zassert_ok(disk_access_read("RAM0", disk, 200, 8));
	zassert_equal(disk[511], 0x5a);
	zassert_equal(disk[512], 0);
	zassert_equal(disk[7 * 512 - 1], 0);
	zassert_equal(disk[7 * 512], 0x5a);

	e = cmd(NVME_CMD_FLUSH, 1);
	zassert_equal(io(&e), 0);
	e = cmd(NVME_CMD_FLUSH, UINT32_MAX);
	zassert_equal(io(&e), 0);
}

ZTEST(epf_nvme, test_cq_wrap_and_smart)
{
	uint8_t *log = host_ptr(IDBUF);
	struct nvme_sqe e;
	uint64_t writes;

	io_queues_up();

	/* 3 x the queue depth: SQ and CQ wrap, phase flips */
	for (int i = 0; i < 3 * IO_DEPTH; i++) {
		e = cmd(NVME_CMD_WRITE, 1);
		set_prp(&e, DATA, 0);
		e.dw[10] = i;
		zassert_equal(io(&e), 0, "write %d", i);
	}
	zassert_equal(cq1.phase, 0);

	e = cmd(NVME_ADM_GET_LOG, 0);
	set_prp(&e, IDBUF, 0);
	e.dw[10] = ((NVME_LOG_SMART_SIZE / 4U - 1U) << 16) | NVME_LOG_SMART;
	zassert_equal(admin(&e, NULL), 0);
	writes = sys_get_le64(&log[NVME_SMART_HOST_WRITES]);
	zassert_true(writes >= 3 * IO_DEPTH, "host writes %llu", writes);
	zassert_equal(log[NVME_SMART_AVAIL_SPARE], 100);
}

ZTEST(epf_nvme, test_aer_held_and_shutdown)
{
	struct nvme_sqe e = cmd(NVME_ADM_AER, 0);
	uint16_t cid = next_cid++;
	struct nvme_cqe *c;

	/* no completion for an AER */
	e.dw[0] |= (uint32_t)cid << 16;
	memcpy(host_ptr(asq.base + asq.idx * 64U), &e, 64);
	asq.idx = (asq.idx + 1U) % asq.depth;
	wr(NVME_SQ_DB(0), asq.idx);
	k_sleep(K_MSEC(20));
	c = host_ptr(acq.base + acq.idx * 16U);
	zassert_not_equal(c->status & 1U, acq.phase, "AER completed");

	/* commands behind it still complete */
	e = cmd(NVME_ADM_KEEP_ALIVE, 0);
	zassert_equal(admin(&e, NULL), 0);

	/* normal shutdown */
	wr(NVME_REG_CC, rd(NVME_REG_CC) | (1U << 14));
	zassert_true(wait_csts(NVME_CSTS_SHST_MASK, NVME_CSTS_SHST_CMPLT), "no SHST complete");
}

ZTEST(epf_nvme, test_bad_config_and_bme)
{
	struct nvme_sqe e;

	/* 8 KiB pages are not supported: fatal status */
	ctrl_off();
	wr(NVME_REG_CC, NVME_CC_EN | (1U << 7) | (NVME_SQES << 16) | (NVME_CQES << 20));
	zassert_true(wait_csts(NVME_CSTS_CFS, NVME_CSTS_CFS), "no CFS");
	zassert_false(rd(NVME_REG_CSTS) & NVME_CSTS_RDY);
	ctrl_on();
	zassert_false(rd(NVME_REG_CSTS) & NVME_CSTS_CFS);

	/* no bus mastering: nothing is fetched until it is back */
	fake_bme = false;
	e = cmd(NVME_ADM_KEEP_ALIVE, 0);
	e.dw[0] |= (uint32_t)next_cid << 16;
	memcpy(host_ptr(asq.base + asq.idx * 64U), &e, 64);
	wr(NVME_SQ_DB(0), (asq.idx + 1U) % asq.depth);
	k_sleep(K_MSEC(20));
	zassert_not_equal(((struct nvme_cqe *)host_ptr(acq.base))->status & 1U, acq.phase);
	zassert_equal(fake_maps, 0, "outbound access without BME");
	fake_bme = true;
	/* resubmit through the helper: same slot, same doorbell value */
	zassert_equal(admin(&e, NULL), 0);
}

ZTEST(epf_nvme, test_link_down_resets)
{
	const struct device *epc = DEVICE_DT_GET(DT_NODELABEL(fake_epc));

	io_queues_up();
	/* host reboot: CC.EN left in the BAR must not survive */
	pcie_epc_notify(epc, PCIE_EPC_EVT_LINK_DOWN, 0xff);
	zassert_true(wait_csts(NVME_CSTS_RDY, 0), "still ready");
	zassert_equal(rd(NVME_REG_CC), 0);
	zassert_equal(rd(NVME_REG_VS), NVME_VS(1, 4, 0));

	/* and it comes up again from scratch */
	ctrl_on();
	io_queues_up();
}

ZTEST_SUITE(epf_nvme, NULL, setup, before, NULL, NULL);
