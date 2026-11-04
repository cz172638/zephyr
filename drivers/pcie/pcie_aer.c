/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * PCIe Advanced Error Reporting below a root port, controller neutral.
 *
 * Error messages (ERR_COR, ERR_NONFATAL, ERR_FATAL) travel upstream to
 * the root port. A function sends one when its Device Control reporting
 * enable for the severity is set (UR has its own enable), or for
 * non-fatal and fatal also with Command.SERR#. A bridge (every switch
 * port, the root port) forwards non-fatal and fatal messages from its
 * secondary side only with Bridge Control.SERR# Enable. The root port
 * latches the first source in Error Source Identification and raises
 * its interrupt when Root Error Command enables it.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/pcie/aer.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* Configuration space, byte offsets */
#define CMDSTAT    0x04
#define HDRTYPE    0x0c
#define BUSNUM     0x18
#define SECSTAT    0x1c
#define CAPPTR     0x34
#define BRIDGE_CTL 0x3c

#define CMD_SERR        BIT(8)
#define CMD_PARITY      BIT(6)
#define STA_ERR_BITS    (BIT(24) | BIT(27) | BIT(28) | BIT(29) | BIT(30) | BIT(31))
#define BCTL_SERR       BIT(17) /* in the dword at 0x3c */
#define BCTL_PARITY     BIT(16)

#define CAP_ID_EXP      0x10
#define EXT_ID_AER      0x0001

/* PCI Express capability */
#define EXP_CAP     0x00
#define EXP_DEVCTL  0x08
#define DEVCTL_COR  BIT(0)
#define DEVCTL_NFE  BIT(1)
#define DEVCTL_FE   BIT(2)
#define DEVCTL_UR   BIT(3)
#define DEVSTA_ALL  GENMASK(19, 16)

/* AER capability */
#define AER_UE     0x04
#define AER_UEM    0x08
#define AER_UES    0x0c
#define AER_CE     0x10
#define AER_CEM    0x14
#define AER_CAPCTL 0x18
#define AER_HDR    0x1c
#define AER_RCMD   0x2c
#define AER_RSTA   0x30
#define AER_RSRC   0x34

#define CAPCTL_ECRC_GEN_CAP BIT(5)
#define CAPCTL_ECRC_GEN_EN  BIT(6)
#define CAPCTL_ECRC_CHK_CAP BIT(7)
#define CAPCTL_ECRC_CHK_EN  BIT(8)

#define RCMD_ALL    GENMASK(2, 0)
#define RSTA_COR    BIT(0)
#define RSTA_MCOR   BIT(1)
#define RSTA_UNC    BIT(2)
#define RSTA_MUNC   BIT(3)
#define RSTA_FIRST_FATAL BIT(4)
#define RSTA_NFAT   BIT(5)
#define RSTA_FAT    BIT(6)
#define RSTA_W1C    GENMASK(6, 0)

#define WALK_DEPTH 8

const char *const pcie_aer_ue_name[32] = {
	[0] = "TrainErr", [4] = "DLP", [5] = "SDES", [12] = "TLP", [13] = "FCP", [14] = "CmpltTO",
	[15] = "CmpltAbrt", [16] = "UnxCmplt", [17] = "RxOF", [18] = "MalfTLP", [19] = "ECRC",
	[20] = "UnsupReq", [21] = "ACSViol", [22] = "UncorrIntErr", [23] = "BlockedTLP",
	[24] = "AtomicOpBlocked", [25] = "TLPBlockedErr", [26] = "PoisonTLPBlocked",
};

const char *const pcie_aer_ce_name[32] = {
	[0] = "RxErr", [6] = "BadTLP", [7] = "BadDLLP", [8] = "Rollover", [12] = "Timeout",
	[13] = "AdvNonFatalErr", [14] = "CorrIntErr", [15] = "HeaderOF",
};

const char *pcie_aer_type_name(uint8_t type)
{
	static const char *const n[] = {"Endpoint", "Legacy EP", "?", "?", "Root Port",
					"Upstream", "Downstream", "PCIe-PCI", "PCI-PCIe",
					"RCiEP", "RCEC"};

	if (type == PCIE_AER_TYPE_NONE) {
		return "PCI";
	}
	return type < ARRAY_SIZE(n) ? n[type] : "?";
}

int pcie_aer_bits_str(char *buf, size_t len, uint32_t v, const char *const *names)
{
	int p = 0;

	buf[0] = '\0';
	for (int b = 0; b < 32 && p < (int)len - 1; b++) {
		if ((v & BIT(b)) != 0) {
			p += snprintf(&buf[p], len - p, "%s%s", p ? " " : "",
				      names[b] != NULL ? names[b] : "?");
		}
	}
	return MIN(p, (int)len - 1);
}

static inline uint32_t rd(const struct device *dev, pcie_bdf_t bdf, unsigned int off)
{
	return pcie_ctrl_conf_read(dev, bdf, off / 4);
}

static inline void wr(const struct device *dev, pcie_bdf_t bdf, unsigned int off, uint32_t v)
{
	pcie_ctrl_conf_write(dev, bdf, off / 4, v);
}

static bool present(const struct device *dev, pcie_bdf_t bdf)
{
	uint32_t id = rd(dev, bdf, 0);

	return id != 0xffffffff && id != 0 && (id & 0xffff) != 0xffff;
}

static uint16_t find_cap(const struct device *dev, pcie_bdf_t bdf, uint8_t id)
{
	uint32_t p;

	if ((rd(dev, bdf, CMDSTAT) & BIT(20)) == 0) {
		return 0;
	}
	p = rd(dev, bdf, CAPPTR) & 0xfc;
	for (int n = 0; n < 48 && p >= 0x40; n++) {
		uint32_t v = rd(dev, bdf, p);

		if ((v & 0xff) == id) {
			return p;
		}
		p = (v >> 8) & 0xfc;
	}
	return 0;
}

static uint16_t find_ext(const struct device *dev, pcie_bdf_t bdf, uint16_t id)
{
	uint32_t p = 0x100;

	for (int n = 0; n < 96 && p >= 0x100 && p < 0x1000; n++) {
		uint32_t v = rd(dev, bdf, p);

		if (v == 0 || v == 0xffffffff) {
			return 0;
		}
		if ((v & 0xffff) == id) {
			return p;
		}
		p = (v >> 20) & 0xffc;
	}
	return 0;
}

int pcie_aer_info(const struct device *dev, pcie_bdf_t bdf, struct pcie_aer_info *in)
{
	uint32_t v;

	memset(in, 0, sizeof(*in));
	if (!present(dev, bdf)) {
		return -ENODEV;
	}
	v = rd(dev, bdf, CMDSTAT);
	in->cmd = v & 0xffff;
	in->sta = v >> 16;
	in->bridge = ((rd(dev, bdf, HDRTYPE) >> 16) & 0x7f) == 1;
	if (in->bridge) {
		in->bctl = rd(dev, bdf, BRIDGE_CTL) >> 16;
	}
	in->type = PCIE_AER_TYPE_NONE;
	in->exp = find_cap(dev, bdf, CAP_ID_EXP);
	if (in->exp == 0) {
		return 0;
	}
	in->type = (rd(dev, bdf, in->exp + EXP_CAP) >> 20) & 0xf;
	v = rd(dev, bdf, in->exp + EXP_DEVCTL);
	in->devctl = v & 0xffff;
	in->devsta = v >> 16;
	in->aer = find_ext(dev, bdf, EXT_ID_AER);
	if (in->aer == 0) {
		return 0;
	}
	in->ue = rd(dev, bdf, in->aer + AER_UE);
	in->uem = rd(dev, bdf, in->aer + AER_UEM);
	in->ues = rd(dev, bdf, in->aer + AER_UES);
	in->ce = rd(dev, bdf, in->aer + AER_CE);
	in->cem = rd(dev, bdf, in->aer + AER_CEM);
	in->capctl = rd(dev, bdf, in->aer + AER_CAPCTL);
	for (int i = 0; i < 4; i++) {
		in->hdr[i] = rd(dev, bdf, in->aer + AER_HDR + 4 * i);
	}
	if (in->type == PCIE_AER_TYPE_RP || in->type == PCIE_AER_TYPE_RCEC) {
		in->root_cmd = rd(dev, bdf, in->aer + AER_RCMD);
		in->root_sta = rd(dev, bdf, in->aer + AER_RSTA);
		in->root_src = rd(dev, bdf, in->aer + AER_RSRC);
	}
	return 0;
}

static void clear_in(const struct device *dev, pcie_bdf_t bdf, const struct pcie_aer_info *in)
{
	wr(dev, bdf, CMDSTAT, in->cmd | STA_ERR_BITS);
	if (in->bridge) {
		uint32_t v = rd(dev, bdf, SECSTAT);

		wr(dev, bdf, SECSTAT, (v & 0xffff) | STA_ERR_BITS);
	}
	if (in->exp != 0) {
		wr(dev, bdf, in->exp + EXP_DEVCTL, in->devctl | DEVSTA_ALL);
	}
	if (in->aer != 0) {
		wr(dev, bdf, in->aer + AER_UE, 0xffffffff);
		wr(dev, bdf, in->aer + AER_CE, 0xffffffff);
		if (in->type == PCIE_AER_TYPE_RP || in->type == PCIE_AER_TYPE_RCEC) {
			wr(dev, bdf, in->aer + AER_RSTA, RSTA_W1C);
		}
	}
}

void pcie_aer_clear(const struct device *dev, pcie_bdf_t bdf)
{
	struct pcie_aer_info in;

	if (pcie_aer_info(dev, bdf, &in) == 0) {
		clear_in(dev, bdf, &in);
	}
}

/* ------------------------------------------------------------------------ */
/* Topology                                                                  */

static void walk_fn(const struct device *dev, pcie_bdf_t bdf, int depth, pcie_aer_visit_t visit,
		    void *user, int *n);

static void walk_bus(const struct device *dev, unsigned int bus, bool dev0_only, int depth,
		     pcie_aer_visit_t visit, void *user, int *n)
{
	for (unsigned int d = 0; d < (dev0_only ? 1U : 32U); d++) {
		for (unsigned int f = 0; f < 8; f++) {
			pcie_bdf_t bdf = PCIE_BDF(bus, d, f);

			if (!present(dev, bdf)) {
				if (f == 0) {
					break;
				}
				continue;
			}
			walk_fn(dev, bdf, depth, visit, user, n);
			if (f == 0 && (rd(dev, bdf, HDRTYPE) & BIT(23)) == 0) {
				break; /* single function */
			}
		}
	}
}

static void walk_fn(const struct device *dev, pcie_bdf_t bdf, int depth, pcie_aer_visit_t visit,
		    void *user, int *n)
{
	uint32_t busnum;
	unsigned int sec, sub;
	uint16_t exp;
	uint8_t type = PCIE_AER_TYPE_NONE;

	visit(dev, bdf, user);
	(*n)++;
	if (((rd(dev, bdf, HDRTYPE) >> 16) & 0x7f) != 1 || depth >= WALK_DEPTH) {
		return;
	}
	busnum = rd(dev, bdf, BUSNUM);
	sec = (busnum >> 8) & 0xff;
	sub = (busnum >> 16) & 0xff;
	if (sec == 0 || sec <= PCIE_BDF_TO_BUS(bdf) || sub < sec) {
		return; /* not enumerated */
	}
	exp = find_cap(dev, bdf, CAP_ID_EXP);
	if (exp != 0) {
		type = (rd(dev, bdf, exp + EXP_CAP) >> 20) & 0xf;
	}
	/* a link below root and downstream ports: device 0 only (no ARI here) */
	walk_bus(dev, sec, type == PCIE_AER_TYPE_RP || type == PCIE_AER_TYPE_DP, depth + 1, visit,
		 user, n);
}

int pcie_aer_walk(const struct device *dev, pcie_bdf_t root, pcie_aer_visit_t visit, void *user)
{
	int n = 0;

	if (!present(dev, root)) {
		return -ENODEV;
	}
	walk_fn(dev, root, 0, visit, user, &n);
	return n;
}

/* ------------------------------------------------------------------------ */
/* Setup                                                                     */

struct setup_ctx {
	uint32_t flags;
};

static void setup_fn(const struct device *dev, pcie_bdf_t bdf, void *user)
{
	struct setup_ctx *c = user;
	struct pcie_aer_info in;
	uint16_t devctl;
	bool port;

	if (pcie_aer_info(dev, bdf, &in) != 0) {
		return;
	}
	/* own non-fatal / fatal errors signalled as SERR# (also without the cap) */
	in.cmd |= CMD_SERR;
	if (in.bridge) {
		uint32_t v = rd(dev, bdf, BRIDGE_CTL);

		/* forward ERR_NONFATAL / ERR_FATAL from the secondary side */
		wr(dev, bdf, BRIDGE_CTL, v | BCTL_SERR);
	}
	if (in.exp != 0) {
		port = in.bridge || in.type == PCIE_AER_TYPE_RP || in.type == PCIE_AER_TYPE_UP ||
		       in.type == PCIE_AER_TYPE_DP;
		devctl = in.devctl | DEVCTL_COR | DEVCTL_NFE | DEVCTL_FE;
		if ((port && (c->flags & PCIE_AER_UR_BRIDGE) != 0) ||
		    (!port && (c->flags & PCIE_AER_UR_EP) != 0)) {
			devctl |= DEVCTL_UR;
		} else {
			devctl &= ~DEVCTL_UR;
		}
		in.devctl = devctl;
	}
	if (in.aer != 0) {
		uint32_t cc = in.capctl;

		if ((c->flags & PCIE_AER_ECRC) != 0) {
			if ((cc & CAPCTL_ECRC_GEN_CAP) != 0) {
				cc |= CAPCTL_ECRC_GEN_EN;
			}
			if ((cc & CAPCTL_ECRC_CHK_CAP) != 0) {
				cc |= CAPCTL_ECRC_CHK_EN;
			}
		} else {
			cc &= ~(CAPCTL_ECRC_GEN_EN | CAPCTL_ECRC_CHK_EN);
		}
		wr(dev, bdf, in.aer + AER_CAPCTL, cc);
		if (in.type == PCIE_AER_TYPE_RP || in.type == PCIE_AER_TYPE_RCEC) {
			wr(dev, bdf, in.aer + AER_RCMD, in.root_cmd | RCMD_ALL);
		}
	}
	/* the enumeration left UR (absent functions) latched: clear, then enable */
	clear_in(dev, bdf, &in);
}

int pcie_aer_setup(const struct device *dev, pcie_bdf_t root, uint32_t flags)
{
	struct setup_ctx c = {.flags = flags};

	return pcie_aer_walk(dev, root, setup_fn, &c);
}

/* ------------------------------------------------------------------------ */
/* Root port service and counters                                            */

static struct pcie_aer_stat stats[CONFIG_PCIE_AER_SOURCES];

static struct pcie_aer_stat *stat_for(pcie_bdf_t bdf)
{
	for (int i = 0; i < ARRAY_SIZE(stats); i++) {
		if (stats[i].last_ms != 0 && stats[i].bdf == bdf) {
			return &stats[i];
		}
	}
	for (int i = 0; i < ARRAY_SIZE(stats); i++) {
		if (stats[i].last_ms == 0) {
			stats[i].bdf = bdf;
			return &stats[i];
		}
	}
	return &stats[ARRAY_SIZE(stats) - 1]; /* full: the last slot takes the rest */
}

const struct pcie_aer_stat *pcie_aer_stat_get(int i)
{
	if (i < 0 || i >= ARRAY_SIZE(stats) || stats[i].last_ms == 0) {
		return NULL;
	}
	return &stats[i];
}

void pcie_aer_stat_clear(void)
{
	memset(stats, 0, sizeof(stats));
}

static void account(const struct device *dev, struct pcie_aer_event *ev,
		    pcie_aer_report_t report)
{
	struct pcie_aer_stat *s = stat_for(ev->bdf);

	s->cnt[ev->sev]++;
	s->ue_acc |= ev->ue;
	s->ce_acc |= ev->ce;
	s->last_ms = MAX(k_uptime_get(), 1);
	if (report != NULL) {
		report(dev, ev);
	}
}

/* Status of the source into ev, then cleared */
static void take_source(const struct device *dev, struct pcie_aer_event *ev)
{
	struct pcie_aer_info in;

	if (ev->bdf == PCIE_BDF_NONE || pcie_aer_info(dev, ev->bdf, &in) != 0) {
		return;
	}
	if (ev->sev == PCIE_AER_COR) {
		ev->ce = in.ce & ~in.cem;
		if (in.aer != 0) {
			wr(dev, ev->bdf, in.aer + AER_CE, ev->ce);
		}
	} else {
		ev->ue = in.ue & ~in.uem;
		memcpy(ev->hdr, in.hdr, sizeof(ev->hdr));
		ev->hdr_valid = ev->ue != 0;
		if (in.aer != 0) {
			wr(dev, ev->bdf, in.aer + AER_UE, ev->ue);
		}
	}
	if (in.exp != 0) {
		wr(dev, ev->bdf, in.exp + EXP_DEVCTL, in.devctl | DEVSTA_ALL);
	}
}

static pcie_bdf_t rid_to_bdf(uint32_t rid)
{
	return PCIE_BDF((rid >> 8) & 0xff, (rid >> 3) & 0x1f, rid & 7);
}

struct sweep_ctx {
	pcie_aer_report_t report;
	int n;
};

/* No latched source: whoever has status set */
static void sweep_fn(const struct device *dev, pcie_bdf_t bdf, void *user)
{
	struct sweep_ctx *c = user;
	struct pcie_aer_info in;
	struct pcie_aer_event ev = {.bdf = bdf};

	if (pcie_aer_info(dev, bdf, &in) != 0 || in.exp == 0) {
		return;
	}
	if ((in.ce & ~in.cem) != 0 || (in.aer == 0 && (in.devsta & BIT(0)) != 0)) {
		ev.sev = PCIE_AER_COR;
		take_source(dev, &ev);
		account(dev, &ev, c->report);
		c->n++;
	}
	if ((in.ue & ~in.uem) != 0 || (in.aer == 0 && (in.devsta & (BIT(1) | BIT(2))) != 0)) {
		ev.sev = ((in.ue & ~in.uem & in.ues) != 0 || (in.devsta & BIT(2)) != 0)
				 ? PCIE_AER_FATAL
				 : PCIE_AER_NONFATAL;
		ev.ce = 0;
		take_source(dev, &ev);
		account(dev, &ev, c->report);
		c->n++;
	}
}

int pcie_aer_root_service(const struct device *dev, pcie_bdf_t root, pcie_aer_report_t report)
{
	struct pcie_aer_info rp;
	int n = 0;

	if (pcie_aer_info(dev, root, &rp) != 0) {
		return -ENODEV;
	}
	if (rp.aer != 0 && (rp.root_sta & (RSTA_COR | RSTA_UNC)) != 0) {
		if ((rp.root_sta & RSTA_COR) != 0) {
			struct pcie_aer_event ev = {
				.bdf = rid_to_bdf(rp.root_src & 0xffff),
				.sev = PCIE_AER_COR,
				.multiple = (rp.root_sta & RSTA_MCOR) != 0,
			};

			take_source(dev, &ev);
			account(dev, &ev, report);
			n++;
		}
		if ((rp.root_sta & RSTA_UNC) != 0) {
			struct pcie_aer_event ev = {
				.bdf = rid_to_bdf(rp.root_src >> 16),
				.sev = (rp.root_sta & RSTA_FAT) != 0 ? PCIE_AER_FATAL
								     : PCIE_AER_NONFATAL,
				.multiple = (rp.root_sta & RSTA_MUNC) != 0,
			};

			take_source(dev, &ev);
			account(dev, &ev, report);
			n++;
		}
		wr(dev, root, rp.aer + AER_RSTA, rp.root_sta & RSTA_W1C);
		/* "multiple": the others are only in their own status, sweep for them */
		if ((rp.root_sta & (RSTA_MCOR | RSTA_MUNC)) == 0) {
			return n;
		}
	}

	struct sweep_ctx c = {.report = report};

	(void)pcie_aer_walk(dev, root, sweep_fn, &c);
	return n + c.n;
}
