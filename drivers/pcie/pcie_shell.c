/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Controller-neutral "pcie" shell, see <zephyr/drivers/pcie/pcie_shell.h>.
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/pcie/pcie_shell.h>
#if defined(CONFIG_LAB_EXT)
#include <zephyr/lab/mod.h>
#endif
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#define MAX_CTRL 4

static const struct pcie_shell_ctrl *ctrls[MAX_CTRL];
static int n_ctrls;

void pcie_shell_register(const struct pcie_shell_ctrl *ctrl)
{
	for (int i = 0; i < n_ctrls; i++) {
		if (ctrls[i] == ctrl) {
			return;
		}
	}
	if (n_ctrls < MAX_CTRL) {
		ctrls[n_ctrls++] = ctrl;
	}
}

static bool is_active(const struct pcie_shell_ctrl *c)
{
	return c->active == NULL || c->active(c->dev);
}

static const struct pcie_shell_ctrl *active_ctrl(const struct shell *sh)
{
	for (int i = 0; i < n_ctrls; i++) {
		if (is_active(ctrls[i])) {
			return ctrls[i];
		}
	}
	shell_error(sh, "no active PCIe controller");
	return NULL;
}

static const char *role_name(enum pcie_shell_role r)
{
	return r == PCIE_SHELL_ROLE_RC ? "rc" : "ep";
}

int pcie_shell_parse_bdf(const char *s, pcie_bdf_t *bdf)
{
	unsigned long b, d, f;
	char *e;

	b = strtoul(s, &e, 16);
	if (*e != ':') {
		return -EINVAL;
	}
	d = strtoul(e + 1, &e, 16);
	if (*e != '.') {
		return -EINVAL;
	}
	f = strtoul(e + 1, &e, 16);
	if (*e != '\0' || b > 255 || d > 31 || f > 7) {
		return -EINVAL;
	}
	*bdf = PCIE_BDF(b, d, f);
	return 0;
}

static int bdf_arg(const struct shell *sh, const char *s, pcie_bdf_t *bdf)
{
	if (pcie_shell_parse_bdf(s, bdf) != 0) {
		shell_error(sh, "%s: bus:dev.fn in hex, e.g. 1:0.0", s);
		return -EINVAL;
	}
	return 0;
}

static uint32_t cfg_rd(const struct pcie_shell_ctrl *c, pcie_bdf_t bdf, unsigned int off)
{
	return c->cfg_read(c->dev, bdf, off >> 2);
}

/* ------------------------------------------------------------------------ */
/* Capabilities                                                              */

static const struct {
	uint16_t id;
	bool ext;
	const char *name;
	const char *desc;
} cap_names[] = {
	{0x01, false, "pm", "Power Management"},
	{0x05, false, "msi", "MSI"},
	{0x09, false, "vndr", "Vendor Specific"},
	{0x0d, false, "ssvid", "Subsystem ID"},
	{0x10, false, "exp", "PCI Express"},
	{0x11, false, "msix", "MSI-X"},
	{0x13, false, "af", "Advanced Features"},
	{0x0001, true, "aer", "Advanced Error Reporting"},
	{0x0002, true, "vc", "Virtual Channel"},
	{0x0003, true, "dsn", "Device Serial Number"},
	{0x000b, true, "vsec", "Vendor Specific Extended"},
	{0x000d, true, "acs", "Access Control Services"},
	{0x000e, true, "ari", "Alternative Routing ID"},
	{0x0010, true, "sriov", "SR-IOV"},
	{0x0012, true, "mcast", "Multicast"},
	{0x0015, true, "rebar", "Resizable BAR"},
	{0x0018, true, "ltr", "Latency Tolerance Reporting"},
	{0x0019, true, "secpci", "Secondary PCI Express"},
	{0x001e, true, "l1ss", "L1 PM Substates"},
	{0x001f, true, "ptm", "Precision Time Measurement"},
	{0x0025, true, "dlf", "Data Link Feature"},
	{0x0026, true, "pl16", "Physical Layer 16 GT/s"},
};

static const char *cap_name(uint16_t id, bool ext, bool desc)
{
	for (size_t i = 0; i < ARRAY_SIZE(cap_names); i++) {
		if (cap_names[i].id == id && cap_names[i].ext == ext) {
			return desc ? cap_names[i].desc : cap_names[i].name;
		}
	}
	return "?";
}

#define MAX_CAPS 32

struct cap {
	uint16_t off;
	uint16_t id;
	uint8_t ver;
	bool ext;
};

/* Both chains, in order; returns the count */
static int caps_walk(const struct pcie_shell_ctrl *c, pcie_bdf_t bdf, struct cap *caps)
{
	int n = 0;
	uint32_t off;

	if ((cfg_rd(c, bdf, 0x04) & BIT(20)) != 0) {
		off = cfg_rd(c, bdf, 0x34) & 0xfc;
		for (int ttl = 48; off >= 0x40 && off < 0x100 && ttl > 0 && n < MAX_CAPS; ttl--) {
			uint32_t w = cfg_rd(c, bdf, off);

			caps[n++] = (struct cap){.off = off, .id = w & 0xff, .ext = false};
			off = (w >> 8) & 0xfc;
		}
	}
	off = 0x100;
	for (int ttl = 64; off >= 0x100 && off < 0x1000 && ttl > 0 && n < MAX_CAPS; ttl--) {
		uint32_t w = cfg_rd(c, bdf, off);

		if (w == 0 || w == 0xffffffff) {
			break;
		}
		caps[n++] = (struct cap){.off = off, .id = w & 0xffff, .ver = (w >> 16) & 0xf,
					 .ext = true};
		off = (w >> 20) & 0xffc;
	}
	return n;
}

/* Capability by name, by number ("0x10", ext: "e0x1") or by offset ("@0x100") */
static int cap_find(const struct shell *sh, const struct cap *caps, int n, const char *arg)
{
	uint32_t id = UINT32_MAX;
	bool ext = false;

	if (arg[0] == '@') {
		unsigned long off = strtoul(&arg[1], NULL, 0);

		for (int i = 0; i < n; i++) {
			if (caps[i].off == off) {
				return i;
			}
		}
	} else {
		for (size_t i = 0; i < ARRAY_SIZE(cap_names); i++) {
			if (strcmp(arg, cap_names[i].name) == 0) {
				id = cap_names[i].id;
				ext = cap_names[i].ext;
			}
		}
		if (id == UINT32_MAX) {
			ext = arg[0] == 'e';
			id = strtoul(ext ? &arg[1] : arg, NULL, 0);
		}
		for (int i = 0; i < n; i++) {
			if (caps[i].id == id && caps[i].ext == ext) {
				return i;
			}
		}
	}
	shell_error(sh, "capability %s not present", arg);
	return -ENOENT;
}

/* Length: up to the next capability above it, at most 0x40 */
static uint32_t cap_len(const struct cap *caps, int n, int i)
{
	uint32_t end = caps[i].ext ? 0x1000 : 0x100;

	for (int k = 0; k < n; k++) {
		if (caps[k].ext == caps[i].ext && caps[k].off > caps[i].off && caps[k].off < end) {
			end = caps[k].off;
		}
	}
	return MIN(end - caps[i].off, 0x40U);
}

static const char *const aer_ue[32] = {
	[0] = "TrainErr", [4] = "DLP", [5] = "SDES", [12] = "TLP", [13] = "FCP", [14] = "CmpltTO",
	[15] = "CmpltAbrt", [16] = "UnxCmplt", [17] = "RxOF", [18] = "MalfTLP", [19] = "ECRC",
	[20] = "UnsupReq", [21] = "ACSViol", [22] = "UncorrIntErr", [23] = "BlockedTLP",
	[24] = "AtomicOpBlocked", [25] = "TLPBlockedErr", [26] = "PoisonTLPBlocked",
};
static const char *const aer_ce[32] = {
	[0] = "RxErr", [6] = "BadTLP", [7] = "BadDLLP", [8] = "Rollover", [12] = "Timeout",
	[13] = "AdvNonFatalErr", [14] = "CorrIntErr", [15] = "HeaderOF",
};

static void bits(const struct shell *sh, const char *what, uint32_t v, const char *const *n)
{
	char buf[160];
	int p = snprintf(buf, sizeof(buf), "  %-8s %08x", what, v);

	for (int b = 0; b < 32 && p < (int)sizeof(buf) - 1; b++) {
		if ((v & BIT(b)) != 0) {
			p += snprintf(&buf[p], sizeof(buf) - p, " %s",
				      n[b] != NULL ? n[b] : "?");
		}
	}
	shell_print(sh, "%s", buf);
}

static void decode_exp(const struct shell *sh, const struct pcie_shell_ctrl *c, pcie_bdf_t bdf,
		       uint32_t off)
{
	static const char *const type[] = {"Endpoint", "Legacy Endpoint", "?", "?",
					   "Root Port", "Upstream Port", "Downstream Port",
					   "PCIe-PCI bridge", "PCI-PCIe bridge", "RC Integrated EP",
					   "RC Event Collector"};
	uint32_t cap = cfg_rd(c, bdf, off) >> 16;
	uint32_t devcap = cfg_rd(c, bdf, off + 0x04);
	uint32_t devctl = cfg_rd(c, bdf, off + 0x08);
	uint32_t lnkcap = cfg_rd(c, bdf, off + 0x0c);
	uint32_t lnk = cfg_rd(c, bdf, off + 0x10);
	unsigned int t = (cap >> 4) & 0xf;

	shell_print(sh, "  v%u %s", cap & 0xf, t < ARRAY_SIZE(type) ? type[t] : "?");
	shell_print(sh, "  DevCap  MPS %u, ext tag %c, FLR %c", 128U << (devcap & 7),
		    (devcap & BIT(5)) ? '+' : '-', (devcap & BIT(28)) ? '+' : '-');
	shell_print(sh, "  DevCtl  MPS %u, MRRS %u, err report cor%c nfat%c fat%c ur%c, RO%c NS%c",
		    128U << ((devctl >> 5) & 7), 128U << ((devctl >> 12) & 7),
		    (devctl & BIT(0)) ? '+' : '-', (devctl & BIT(1)) ? '+' : '-',
		    (devctl & BIT(2)) ? '+' : '-', (devctl & BIT(3)) ? '+' : '-',
		    (devctl & BIT(4)) ? '+' : '-', (devctl & BIT(11)) ? '+' : '-');
	shell_print(sh, "  DevSta  %s%s%s%s", (devctl & BIT(16)) ? "CorrErr " : "",
		    (devctl & BIT(17)) ? "NonFatalErr " : "", (devctl & BIT(18)) ? "FatalErr " : "",
		    (devctl & BIT(19)) ? "UnsupReq" : "");
	shell_print(sh, "  LnkCap  Gen%u x%u, ASPM %s, port %u", lnkcap & 0xf, (lnkcap >> 4) & 0x3f,
		    ((lnkcap >> 10) & 3) == 3 ? "L0s L1" : ((lnkcap >> 10) & 3) == 2 ? "L1"
			    : ((lnkcap >> 10) & 3) == 1 ? "L0s" : "-",
		    lnkcap >> 24);
	shell_print(sh, "  LnkSta  Gen%u x%u%s%s", (lnk >> 16) & 0xf, (lnk >> 20) & 0x3f,
		    (lnk & BIT(27)) ? ", training" : "", (lnk & BIT(29)) ? ", DL active" : "");
}

static void decode_aer(const struct shell *sh, const struct pcie_shell_ctrl *c, pcie_bdf_t bdf,
		       uint32_t off)
{
	bits(sh, "UESta", cfg_rd(c, bdf, off + 0x04), aer_ue);
	bits(sh, "UEMsk", cfg_rd(c, bdf, off + 0x08), aer_ue);
	bits(sh, "UESvrt", cfg_rd(c, bdf, off + 0x0c), aer_ue);
	bits(sh, "CESta", cfg_rd(c, bdf, off + 0x10), aer_ce);
	bits(sh, "CEMsk", cfg_rd(c, bdf, off + 0x14), aer_ce);
	shell_print(sh, "  CapCtl   %08x first error %u", cfg_rd(c, bdf, off + 0x18),
		    cfg_rd(c, bdf, off + 0x18) & 0x1f);
	shell_print(sh, "  HdrLog   %08x %08x %08x %08x", cfg_rd(c, bdf, off + 0x1c),
		    cfg_rd(c, bdf, off + 0x20), cfg_rd(c, bdf, off + 0x24),
		    cfg_rd(c, bdf, off + 0x28));
}

/* ------------------------------------------------------------------------ */
/* pcie ls                                                                   */

static void print_bars(const struct shell *sh, const struct pcie_shell_ctrl *c, pcie_bdf_t bdf,
		       int nbars)
{
	for (int b = 0; b < nbars; b++) {
		uint32_t lo = cfg_rd(c, bdf, 0x10 + 4 * b);
		uint64_t a;

		if (lo == 0 || lo == 0xffffffff) {
			continue;
		}
		if ((lo & 1) != 0) {
			shell_print(sh, "    BAR%d io  0x%x", b, lo & ~3U);
			continue;
		}
		a = lo & ~0xfULL;
		if ((lo & 6) == 4 && b + 1 < nbars) {
			a |= (uint64_t)cfg_rd(c, bdf, 0x14 + 4 * b) << 32;
		}
		shell_print(sh, "    BAR%d mem 0x%llx%s%s", b, (unsigned long long)a,
			    (lo & 6) == 4 ? " 64-bit" : "", (lo & 8) ? " prefetchable" : "");
		if ((lo & 6) == 4) {
			b++;
		}
	}
}

static void ls_dev(const struct shell *sh, const struct pcie_shell_ctrl *c, pcie_bdf_t bdf,
		   uint32_t id, bool verbose)
{
	uint32_t cls = cfg_rd(c, bdf, 0x08);
	uint32_t hdr = (cfg_rd(c, bdf, 0x0c) >> 16) & 0x7f;
	char extra[48] = "";

	if (hdr == 1) {
		uint32_t bus = cfg_rd(c, bdf, 0x18);

		snprintf(extra, sizeof(extra), " bridge, bus %02x..%02x", (bus >> 8) & 0xff,
			 (bus >> 16) & 0xff);
	}
	shell_print(sh, "%02x:%02x.%x %04x:%04x class %02x%02x%02x rev %02x%s",
		    PCIE_BDF_TO_BUS(bdf), PCIE_BDF_TO_DEV(bdf), PCIE_BDF_TO_FUNC(bdf), id & 0xffff,
		    id >> 16, cls >> 24, (cls >> 16) & 0xff, (cls >> 8) & 0xff, cls & 0xff, extra);
	if (!verbose) {
		return;
	}

	struct cap caps[MAX_CAPS];
	int n = caps_walk(c, bdf, caps);
	char buf[128];
	int p = 0;

	print_bars(sh, c, bdf, hdr == 1 ? 2 : 6);
	for (int i = 0; i < n && p < (int)sizeof(buf) - 12; i++) {
		p += snprintf(&buf[p], sizeof(buf) - p, " %s@%x",
			      cap_name(caps[i].id, caps[i].ext, false), caps[i].off);
	}
	if (n > 0) {
		shell_print(sh, "    caps%s", buf);
	}
	for (int i = 0; i < n; i++) {
		if (!caps[i].ext && caps[i].id == 0x10) {
			uint32_t lnk = cfg_rd(c, bdf, caps[i].off + 0x10);

			shell_print(sh, "    link Gen%u x%u%s", (lnk >> 16) & 0xf,
				    (lnk >> 20) & 0x3f, (lnk & BIT(29)) ? " DL active" : "");
		}
	}
}

static void ls_bus(const struct shell *sh, const struct pcie_shell_ctrl *c, unsigned int bus,
		   bool verbose, int depth)
{
	for (unsigned int d = 0; d < 32; d++) {
		for (unsigned int f = 0; f < 8; f++) {
			pcie_bdf_t bdf = PCIE_BDF(bus, d, f);
			uint32_t id = cfg_rd(c, bdf, 0);
			uint32_t hdr;

			if (id == 0xffffffff || id == 0) {
				if (f == 0) {
					break;
				}
				continue;
			}
			ls_dev(sh, c, bdf, id, verbose);
			hdr = cfg_rd(c, bdf, 0x0c) >> 16;
			if ((hdr & 0x7f) == 1 && depth < 16) {
				unsigned int sec = (cfg_rd(c, bdf, 0x18) >> 8) & 0xff;

				if (sec > bus) {
					ls_bus(sh, c, sec, verbose, depth + 1);
				}
			}
			if (f == 0 && (hdr & 0x80) == 0) {
				break;
			}
		}
	}
}

static int cmd_ls(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);

	if (c == NULL) {
		return -ENODEV;
	}
	ls_bus(sh, c, 0, argc > 1 && strcmp(argv[1], "-v") == 0, 0);
	return 0;
}

/* ------------------------------------------------------------------------ */
/* pcie cfg                                                                  */

static int cmd_cfg(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);
	unsigned long off = 0, len = 0x100;
	pcie_bdf_t bdf;

	if (c == NULL || bdf_arg(sh, argv[1], &bdf) != 0) {
		return -EINVAL;
	}
	if (argc > 2) {
		off = strtoul(argv[2], NULL, 0) & ~3UL;
	}
	if (argc > 3) {
		len = strtoul(argv[3], NULL, 0);
	}
	for (unsigned long o = off; o < MIN(off + len, 0x1000UL); o += 16) {
		shell_print(sh, "%03lx: %08x %08x %08x %08x", o, cfg_rd(c, bdf, o),
			    cfg_rd(c, bdf, o + 4), cfg_rd(c, bdf, o + 8), cfg_rd(c, bdf, o + 12));
	}
	return 0;
}

static int cmd_cfg_set(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);
	unsigned long off;
	pcie_bdf_t bdf;

	ARG_UNUSED(argc);
	if (c == NULL || bdf_arg(sh, argv[1], &bdf) != 0) {
		return -EINVAL;
	}
	if (c->cfg_write == NULL) {
		shell_error(sh, "read only");
		return -ENOTSUP;
	}
	off = strtoul(argv[2], NULL, 0) & 0xffc;
	c->cfg_write(c->dev, bdf, off >> 2, strtoul(argv[3], NULL, 0));
	shell_print(sh, "%03lx: %08x", off, cfg_rd(c, bdf, off));
	return 0;
}

/* ------------------------------------------------------------------------ */
/* pcie cap                                                                  */

static int cap_prep(const struct shell *sh, char *bdf_s, const struct pcie_shell_ctrl **c,
		    pcie_bdf_t *bdf, struct cap *caps, int *n)
{
	*c = active_ctrl(sh);
	if (*c == NULL || bdf_arg(sh, bdf_s, bdf) != 0) {
		return -EINVAL;
	}
	if (cfg_rd(*c, *bdf, 0) == 0xffffffff) {
		shell_error(sh, "%s: no device", bdf_s);
		return -ENODEV;
	}
	*n = caps_walk(*c, *bdf, caps);
	return 0;
}

static int cmd_cap_ls(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c;
	struct cap caps[MAX_CAPS];
	pcie_bdf_t bdf;
	int n;

	ARG_UNUSED(argc);
	if (cap_prep(sh, argv[1], &c, &bdf, caps, &n) != 0) {
		return -EINVAL;
	}
	for (int i = 0; i < n; i++) {
		if (caps[i].ext) {
			shell_print(sh, "[%03x] %-6s %04x v%u %s", caps[i].off,
				    cap_name(caps[i].id, true, false), caps[i].id, caps[i].ver,
				    cap_name(caps[i].id, true, true));
		} else {
			shell_print(sh, "[%03x] %-6s   %02x    %s", caps[i].off,
				    cap_name(caps[i].id, false, false), caps[i].id,
				    cap_name(caps[i].id, false, true));
		}
	}
	return 0;
}

/* get <bdf> <cap> [<off>]: dwords of the capability (decoded for exp and aer) */
static int cmd_cap_get(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c;
	struct cap caps[MAX_CAPS];
	pcie_bdf_t bdf;
	uint32_t base, len;
	int n, i;

	if (cap_prep(sh, argv[1], &c, &bdf, caps, &n) != 0) {
		return -EINVAL;
	}
	i = cap_find(sh, caps, n, argv[2]);
	if (i < 0) {
		return i;
	}
	base = caps[i].off;
	if (argc > 3) {
		uint32_t off = strtoul(argv[3], NULL, 0) & ~3U;

		shell_print(sh, "%s+%02x (%03x): %08x", argv[2], off, base + off,
			    cfg_rd(c, bdf, base + off));
		return 0;
	}
	len = cap_len(caps, n, i);
	shell_print(sh, "%s @%03x, %u bytes:", cap_name(caps[i].id, caps[i].ext, true), base,
		    len);
	for (uint32_t o = 0; o < len; o += 16) {
		char buf[64];
		int p = snprintf(buf, sizeof(buf), "  +%02x:", o);

		for (uint32_t k = o; k < MIN(o + 16, len); k += 4) {
			p += snprintf(&buf[p], sizeof(buf) - p, " %08x", cfg_rd(c, bdf, base + k));
		}
		shell_print(sh, "%s", buf);
	}
	if (!caps[i].ext && caps[i].id == 0x10) {
		decode_exp(sh, c, bdf, base);
	} else if (caps[i].ext && caps[i].id == 0x1) {
		decode_aer(sh, c, bdf, base);
	}
	return 0;
}

/* set <bdf> <cap> <off> <val>: write one dword of a capability */
static int cmd_cap_set(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c;
	struct cap caps[MAX_CAPS];
	pcie_bdf_t bdf;
	uint32_t a;
	int n, i;

	ARG_UNUSED(argc);
	if (cap_prep(sh, argv[1], &c, &bdf, caps, &n) != 0) {
		return -EINVAL;
	}
	if (c->cfg_write == NULL) {
		shell_error(sh, "read only");
		return -ENOTSUP;
	}
	i = cap_find(sh, caps, n, argv[2]);
	if (i < 0) {
		return i;
	}
	a = caps[i].off + (strtoul(argv[3], NULL, 0) & ~3U);
	c->cfg_write(c->dev, bdf, a >> 2, strtoul(argv[4], NULL, 0));
	shell_print(sh, "%s+%02lx (%03x): %08x", argv[2], strtoul(argv[3], NULL, 0) & ~3UL, a,
		    cfg_rd(c, bdf, a));
	return 0;
}

/* ------------------------------------------------------------------------ */
/* pcie controller                                                           */

static int cmd_ctrl_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (int i = 0; i < n_ctrls; i++) {
		const struct pcie_shell_ctrl *c = ctrls[i];
		enum pcie_shell_role saved;
		bool has_saved = c->mode_saved != NULL && c->mode_saved(c->dev, &saved);

		shell_print(sh, "%s %s: %s %s%s%s", is_active(c) ? "*" : " ", c->dev->name,
			    c->name, role_name(c->role), has_saved ? ", saved mode " : "",
			    has_saved ? role_name(saved) : "");
	}
	const struct pcie_shell_ctrl *c = active_ctrl(sh);

	if (c != NULL && c->info != NULL) {
		c->info(c->dev, sh);
	}
	return 0;
}

static int cmd_ctrl_mode(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);
	enum pcie_shell_role r;
	int ret;

	if (c == NULL) {
		return -ENODEV;
	}
	if (argc < 2) {
		enum pcie_shell_role saved;

		shell_print(sh, "running as %s%s%s", role_name(c->role),
			    c->mode_saved != NULL && c->mode_saved(c->dev, &saved) ? ", saved " : "",
			    c->mode_saved != NULL && c->mode_saved(c->dev, &saved)
				    ? role_name(saved)
				    : "");
		return 0;
	}
	if (strcmp(argv[1], "rc") == 0) {
		r = PCIE_SHELL_ROLE_RC;
	} else if (strcmp(argv[1], "ep") == 0) {
		r = PCIE_SHELL_ROLE_EP;
	} else {
		shell_error(sh, "rc or ep");
		return -EINVAL;
	}
	if (c->mode_set == NULL) {
		shell_error(sh, "%s: role fixed", c->dev->name);
		return -ENOTSUP;
	}
	ret = c->mode_set(c->dev, r);
	if (ret != 0) {
		shell_error(sh, "saving failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "saved %s; %s", argv[1],
		    r == c->role ? "already running so" : "from the next boot (kernel reboot cold)");
	return 0;
}

static int cmd_ctrl_link(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);

	if (c == NULL || c->link == NULL) {
		return -ENOTSUP;
	}
	return c->link(c->dev, sh, argc > 1 && strcmp(argv[1], "retrain") == 0);
}

static int cmd_ctrl_scan(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (c == NULL || c->scan == NULL) {
		shell_error(sh, "not a root complex");
		return -ENOTSUP;
	}
	return c->scan(c->dev, sh);
}

static int cmd_ctrl_irq(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (c == NULL || c->irq == NULL) {
		return -ENOTSUP;
	}
	c->irq(c->dev, sh);
	return 0;
}

static int parse_pair(const char *s, uint16_t *a, uint16_t *b)
{
	char *e;
	unsigned long x = strtoul(s, &e, 16), y;

	if (*e != ':') {
		return -EINVAL;
	}
	y = strtoul(e + 1, &e, 16);
	if (*e != '\0' || x > 0xffff || y > 0xffff) {
		return -EINVAL;
	}
	*a = x;
	*b = y;
	return 0;
}

/* id [<b:d.f>] [<vid:did> [<svid:ssid> [<class>]]] | id [<b:d.f>] reset */
static int cmd_ctrl_id(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);
	struct pcie_shell_id id;
	pcie_bdf_t bdf = PCIE_BDF(0, 0, 0);
	size_t a = 1;
	int ret;

	if (c == NULL || c->id_get == NULL) {
		return -ENOTSUP;
	}
	if (argc > a && strchr(argv[a], '.') != NULL) {
		if (bdf_arg(sh, argv[a], &bdf) != 0) {
			return -EINVAL;
		}
		a++;
	}
	if (argc > a && strcmp(argv[a], "reset") == 0) {
		ret = c->id_reset != NULL ? c->id_reset(c->dev, bdf) : -ENOTSUP;
		shell_print(sh, ret == 0 ? "saved identity dropped, defaults from the next boot"
					 : "cannot reset (%d)", ret);
		return ret;
	}
	ret = c->id_get(c->dev, bdf, &id);
	if (ret != 0) {
		shell_error(sh, "no identity for this function (%d)", ret);
		return ret;
	}
	if (argc > a) {
		id.class_code = UINT32_MAX;
		if (parse_pair(argv[a], &id.vid, &id.did) != 0 ||
		    (argc > a + 1 && parse_pair(argv[a + 1], &id.svid, &id.ssid) != 0)) {
			shell_error(sh, "<vid:did> [<svid:ssid> [<class>]] in hex");
			return -EINVAL;
		}
		if (argc > a + 2) {
			id.class_code = strtoul(argv[a + 2], NULL, 16) & 0xffffff;
		}
		if (c->id_set == NULL) {
			return -ENOTSUP;
		}
		ret = c->id_set(c->dev, bdf, &id);
		if (ret != 0) {
			shell_error(sh, "failed (%d)", ret);
			return ret;
		}
		(void)c->id_get(c->dev, bdf, &id);
	}
	shell_print(sh, "%02x:%02x.%x %04x:%04x subsystem %04x:%04x class %06x%s",
		    PCIE_BDF_TO_BUS(bdf), PCIE_BDF_TO_DEV(bdf), PCIE_BDF_TO_FUNC(bdf), id.vid,
		    id.did, id.svid, id.ssid, id.class_code, argc > a ? " (saved)" : "");
	return 0;
}


/* dsn [<b:d.f>] [<hex> | uid | new | reset] */
static int cmd_ctrl_dsn(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);
	pcie_bdf_t bdf = PCIE_BDF(0, 0, 0);
	const char *src = NULL;
	uint64_t v = 0;
	size_t a = 1;
	int ret;

	if (c == NULL || c->dsn == NULL) {
		shell_error(sh, "no Device Serial Number support");
		return -ENOTSUP;
	}
	if (argc > a && strchr(argv[a], '.') != NULL) {
		if (bdf_arg(sh, argv[a], &bdf) != 0) {
			return -EINVAL;
		}
		a++;
	}
	ret = c->dsn(c->dev, bdf, argc > a ? argv[a] : NULL, &v, &src);
	if (ret != 0) {
		shell_error(sh, "failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "%02x:%02x.%x DSN %016llx (lspci %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x)%s%s",
		    PCIE_BDF_TO_BUS(bdf), PCIE_BDF_TO_DEV(bdf), PCIE_BDF_TO_FUNC(bdf),
		    (unsigned long long)v, (uint8_t)(v >> 56), (uint8_t)(v >> 48),
		    (uint8_t)(v >> 40), (uint8_t)(v >> 32), (uint8_t)(v >> 24), (uint8_t)(v >> 16),
		    (uint8_t)(v >> 8), (uint8_t)v, src != NULL ? ", from " : "",
		    src != NULL ? src : "");
	return 0;
}

/* ------------------------------------------------------------------------ */

SHELL_SUBCMD_SET_CREATE(pcie_debug_cmds, (pcie_debug));

#if defined(CONFIG_LAB_EXT)
/*
 * pcie debug = the built-in subcommands (section set) followed by the
 * debug modules loaded with mount "pcie" (lab mod load).
 */
static void pcie_debug_get(size_t idx, struct shell_static_entry *entry)
{
	/* a section set starts with an empty element and ends at the next one */
	const struct shell_static_entry *st = (const struct shell_static_entry *)&pcie_debug_cmds;
	size_t n = 0;

	while (st[n + 1].syntax != NULL) {
		n++;
	}
	if (idx < n) {
		*entry = st[idx + 1];
	} else if (!lab_mod_get("pcie", idx - n, entry)) {
		entry->syntax = NULL;
	}
}

SHELL_DYNAMIC_CMD_CREATE(pcie_debug_dyn, pcie_debug_get);
#define PCIE_DEBUG_SUBCMDS (&pcie_debug_dyn)
#else
#define PCIE_DEBUG_SUBCMDS (&pcie_debug_cmds)
#endif
#if defined(CONFIG_PCIE_FABRIC_SHELL)
SHELL_SUBCMD_SET_CREATE(pcie_fabric_cmds, (pcie_fabric));
#endif

#if defined(CONFIG_PCIE_AER)
#include <zephyr/drivers/pcie/aer.h>

/* ------------------------------------------------------------------------ */
/* pcie aer                                                                  */

static const char *const rsta_name[32] = {
	[0] = "ErrCorRcvd", [1] = "MultiErrCor", [2] = "ErrUncRcvd", [3] = "MultiErrUnc",
	[4] = "FirstUncFatal", [5] = "NonFatalRcvd", [6] = "FatalRcvd",
};

static const struct pcie_aer_stat *aer_stat(pcie_bdf_t bdf)
{
	const struct pcie_aer_stat *st;

	for (int i = 0; (st = pcie_aer_stat_get(i)) != NULL; i++) {
		if (st->bdf == bdf) {
			return st;
		}
	}
	return NULL;
}

static void aer_devsta(char *b, size_t len, uint16_t sta)
{
	snprintf(b, len, "%s%s%s%s%s", (sta & BIT(0)) ? "C" : "", (sta & BIT(1)) ? "N" : "",
		 (sta & BIT(2)) ? "F" : "", (sta & BIT(3)) ? "U" : "", (sta & 0xf) ? "" : "-");
}

static void aer_line(const struct device *dev, pcie_bdf_t bdf, void *user)
{
	const struct shell *sh = user;
	const struct pcie_aer_stat *st = aer_stat(bdf);
	struct pcie_aer_info in;
	char ue[64], ce[48], ds[8], dc[8];

	if (pcie_aer_info(dev, bdf, &in) != 0) {
		return;
	}
	pcie_aer_bits_str(ue, sizeof(ue), in.ue, pcie_aer_ue_name);
	pcie_aer_bits_str(ce, sizeof(ce), in.ce, pcie_aer_ce_name);
	aer_devsta(ds, sizeof(ds), in.devsta);
	aer_devsta(dc, sizeof(dc), in.devctl);
	shell_print(sh, "%02x:%02x.%x %-10s %-3s rep %-4s%s sta %-4s UE %s CE %s | msgs c%u n%u f%u",
		    PCIE_BDF_TO_BUS(bdf), PCIE_BDF_TO_DEV(bdf), PCIE_BDF_TO_FUNC(bdf),
		    pcie_aer_type_name(in.type), in.aer ? "AER" : "-", dc,
		    in.bridge ? ((in.bctl & BIT(1)) ? " fwd" : " NOFWD") : "", ds,
		    ue[0] ? ue : "-", ce[0] ? ce : "-", st ? st->cnt[0] : 0, st ? st->cnt[1] : 0,
		    st ? st->cnt[2] : 0);
}

static void aer_detail(const struct shell *sh, const struct device *dev, pcie_bdf_t bdf)
{
	const struct pcie_aer_stat *st = aer_stat(bdf);
	struct pcie_aer_info in;
	char b[160];

	if (pcie_aer_info(dev, bdf, &in) != 0) {
		shell_error(sh, "%02x:%02x.%x absent", PCIE_BDF_TO_BUS(bdf), PCIE_BDF_TO_DEV(bdf),
			    PCIE_BDF_TO_FUNC(bdf));
		return;
	}
	shell_print(sh, "%02x:%02x.%x %s, PCIe cap @%03x, AER @%03x", PCIE_BDF_TO_BUS(bdf),
		    PCIE_BDF_TO_DEV(bdf), PCIE_BDF_TO_FUNC(bdf), pcie_aer_type_name(in.type),
		    in.exp, in.aer);
	shell_print(sh, "  Command  SERR# %c, parity %c; Status %04x%s%s%s", (in.cmd & BIT(8)) ? '+' : '-',
		    (in.cmd & BIT(6)) ? '+' : '-', in.sta, (in.sta & BIT(14)) ? " SigSysErr" : "",
		    (in.sta & BIT(13)) ? " RcvMAbort" : "", (in.sta & BIT(15)) ? " DetParErr" : "");
	if (in.bridge) {
		shell_print(sh, "  BridgeCtl %04x  SERR# forward %c, parity %c", in.bctl,
			    (in.bctl & BIT(1)) ? '+' : '-', (in.bctl & BIT(0)) ? '+' : '-');
	}
	if (in.exp != 0) {
		shell_print(sh, "  DevCtl   report cor%c nfat%c fat%c ur%c; DevSta %s%s%s%s",
			    (in.devctl & BIT(0)) ? '+' : '-', (in.devctl & BIT(1)) ? '+' : '-',
			    (in.devctl & BIT(2)) ? '+' : '-', (in.devctl & BIT(3)) ? '+' : '-',
			    (in.devsta & BIT(0)) ? "CorrErr " : "",
			    (in.devsta & BIT(1)) ? "NonFatalErr " : "",
			    (in.devsta & BIT(2)) ? "FatalErr " : "",
			    (in.devsta & BIT(3)) ? "UnsupReq" : "");
	}
	if (in.aer == 0) {
		shell_print(sh, "  no AER capability: Device Status only");
	} else {
		static const struct {
			const char *n;
			size_t o;
			bool ue;
		} r[] = {
			{"UESta", offsetof(struct pcie_aer_info, ue), true},
			{"UEMsk", offsetof(struct pcie_aer_info, uem), true},
			{"UESvrt", offsetof(struct pcie_aer_info, ues), true},
			{"CESta", offsetof(struct pcie_aer_info, ce), false},
			{"CEMsk", offsetof(struct pcie_aer_info, cem), false},
		};

		for (int i = 0; i < ARRAY_SIZE(r); i++) {
			uint32_t v = *(const uint32_t *)((const uint8_t *)&in + r[i].o);

			pcie_aer_bits_str(b, sizeof(b), v, r[i].ue ? pcie_aer_ue_name
								   : pcie_aer_ce_name);
			shell_print(sh, "  %-8s %08x %s", r[i].n, v, b);
		}
		shell_print(sh, "  CapCtl   %08x first error %u, ECRC gen %s%s, check %s%s",
			    in.capctl, in.capctl & 0x1f, (in.capctl & BIT(5)) ? "cap" : "-",
			    (in.capctl & BIT(6)) ? "+en" : "", (in.capctl & BIT(7)) ? "cap" : "-",
			    (in.capctl & BIT(8)) ? "+en" : "");
		shell_print(sh, "  HdrLog   %08x %08x %08x %08x", in.hdr[0], in.hdr[1], in.hdr[2],
			    in.hdr[3]);
		if (in.type == PCIE_AER_TYPE_RP || in.type == PCIE_AER_TYPE_RCEC) {
			pcie_aer_bits_str(b, sizeof(b), in.root_sta, rsta_name);
			shell_print(sh, "  RootCmd  %08x interrupt cor%c nfat%c fat%c",
				    in.root_cmd, (in.root_cmd & BIT(0)) ? '+' : '-',
				    (in.root_cmd & BIT(1)) ? '+' : '-',
				    (in.root_cmd & BIT(2)) ? '+' : '-');
			shell_print(sh, "  RootSta  %08x %s", in.root_sta, b[0] ? b : "-");
			shell_print(sh, "  SourceID cor %02x:%02x.%x, unc %02x:%02x.%x",
				    (in.root_src >> 8) & 0xff, (in.root_src >> 3) & 0x1f,
				    in.root_src & 7, (in.root_src >> 24) & 0xff,
				    (in.root_src >> 19) & 0x1f, (in.root_src >> 16) & 7);
		}
	}
	if (st != NULL) {
		char ue[96], ce[64];

		pcie_aer_bits_str(ue, sizeof(ue), st->ue_acc, pcie_aer_ue_name);
		pcie_aer_bits_str(ce, sizeof(ce), st->ce_acc, pcie_aer_ce_name);
		shell_print(sh, "  messages cor %u nonfatal %u fatal %u, last at %lld ms",
			    st->cnt[0], st->cnt[1], st->cnt[2], st->last_ms);
		shell_print(sh, "  seen UE %s, CE %s", ue[0] ? ue : "-", ce[0] ? ce : "-");
	} else {
		shell_print(sh, "  no messages from this function");
	}
}

static void aer_clear_fn(const struct device *dev, pcie_bdf_t bdf, void *user)
{
	ARG_UNUSED(user);
	pcie_aer_clear(dev, bdf);
}

/* pcie aer [<b:d.f> | setup [ur] [urbridge] [ecrc] | clear | poll] */
static int cmd_aer(const struct shell *sh, size_t argc, char **argv)
{
	const struct pcie_shell_ctrl *c = active_ctrl(sh);
	const pcie_bdf_t root = PCIE_BDF(0, 0, 0);
	const struct pcie_aer_stat *st;
	pcie_bdf_t bdf;
	int n;

	if (c == NULL) {
		return -ENODEV;
	}
	if (c->role != PCIE_SHELL_ROLE_RC) {
		shell_error(sh, "endpoint mode: the function's own AER is in pcie debug aer");
		return -ENOTSUP;
	}
	if (argc > 1 && strcmp(argv[1], "setup") == 0) {
		uint32_t flags = 0;

		for (size_t i = 2; i < argc; i++) {
			if (strcmp(argv[i], "ur") == 0) {
				flags |= PCIE_AER_UR_EP;
			} else if (strcmp(argv[i], "urbridge") == 0) {
				flags |= PCIE_AER_UR_BRIDGE;
			} else if (strcmp(argv[i], "ecrc") == 0) {
				flags |= PCIE_AER_ECRC;
			} else {
				shell_error(sh, "ur | urbridge | ecrc");
				return -EINVAL;
			}
		}
		n = pcie_aer_setup(c->dev, root, flags);
		shell_print(sh, "AER set up on %d function(s)%s%s%s", n,
			    (flags & PCIE_AER_UR_EP) ? ", UR on endpoints" : "",
			    (flags & PCIE_AER_UR_BRIDGE) ? ", UR on ports" : "",
			    (flags & PCIE_AER_ECRC) ? ", ECRC" : "");
		return n < 0 ? n : 0;
	}
	if (argc > 1 && strcmp(argv[1], "clear") == 0) {
		(void)pcie_aer_walk(c->dev, root, aer_clear_fn, NULL);
		pcie_aer_stat_clear();
		shell_print(sh, "status and counters cleared");
		return 0;
	}
	if (argc > 1 && strcmp(argv[1], "poll") == 0) {
		n = pcie_aer_root_service(c->dev, root, NULL);
		shell_print(sh, "%d event(s) taken (status cleared, counted)", n);
		return 0;
	}
	if (argc > 1) {
		if (pcie_shell_parse_bdf(argv[1], &bdf) != 0) {
			shell_error(sh, "<b:d.f> | setup | clear | poll");
			return -EINVAL;
		}
		aer_detail(sh, c->dev, bdf);
		return 0;
	}
	shell_print(sh, "rep: reporting enabled C(or) N(onfatal) F(atal) U(R); fwd: bridge "
			"forwards ERR_(NON)FATAL; sta: Device Status");
	n = pcie_aer_walk(c->dev, root, aer_line, (void *)sh);
	if (n < 0) {
		shell_error(sh, "no root port");
		return n;
	}
	for (int i = 0; (st = pcie_aer_stat_get(i)) != NULL; i++) {
		if (st->bdf == PCIE_BDF_NONE) {
			shell_print(sh, "unidentified source: c%u n%u f%u", st->cnt[0],
				    st->cnt[1], st->cnt[2]);
		}
	}
	return 0;
}
#endif /* CONFIG_PCIE_AER */

#if defined(CONFIG_PCIE_EPC)
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>

/* pcie controller ep function [<name>|reset]: the endpoint function for the next boot */
static int cmd_ep_function(const struct shell *sh, size_t argc, char **argv)
{
	const char *names[8];
	size_t n = pcie_epf_names(names, ARRAY_SIZE(names));
	const char *sel;
	int ret;

	if (argc > 1) {
		if (strcmp(argv[1], "reset") == 0) {
			ret = pcie_epf_select_reset();
		} else {
			ret = pcie_epf_select(argv[1]);
			if (ret == -ENOENT) {
				shell_error(sh, "no function \"%s\" in this build", argv[1]);
				return ret;
			}
		}
		if (ret != 0) {
			shell_error(sh, "not saved (%d)", ret);
			return ret;
		}
		shell_print(sh, "saved, takes effect after a reboot (kernel reboot cold)");
	}
	sel = pcie_epf_selection();
	shell_print(sh, "this boot: %s, selected: %s", pcie_epf_bound() ? pcie_epf_bound() : "none",
		    sel != NULL ? sel : "(first in devicetree)");
	for (size_t i = 0; i < MIN(n, ARRAY_SIZE(names)); i++) {
		bool b = pcie_epf_bound() != NULL && strcmp(names[i], pcie_epf_bound()) == 0;
		bool s2 = sel != NULL && strcmp(names[i], sel) == 0;

		shell_print(sh, "  %c %s%s", b ? '*' : ' ', names[i], s2 ? "  (selected)" : "");
	}
	if (n == 0) {
		shell_print(sh, "  no endpoint functions (root complex mode, or none in this build)");
	}
	return 0;
}

/* the started endpoint controller */
static const struct device *ep_active(void)
{
	STRUCT_SECTION_FOREACH(device, dev) {
		if (DEVICE_API_IS(pcie_epc, dev) && device_is_ready(dev) &&
		    pcie_epc_get_features(dev, 0) != NULL) {
			return dev;
		}
	}
	return NULL;
}

/* raise an interrupt toward the host by hand, e.g. to release a host waiting on one */
static int cmd_ep_irq(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *epc = ep_active();
	uint8_t fn = (uint8_t)strtoul(argv[1], NULL, 0);
	enum pcie_epc_irq_type type;
	uint16_t num = 0;
	int ret;

	if (epc == NULL) {
		shell_error(sh, "no endpoint controller");
		return -ENODEV;
	}
	if (strcmp(argv[2], "intx") == 0) {
		type = PCIE_EPC_IRQ_INTX;
	} else if (strcmp(argv[2], "msi") == 0 && argc > 3) {
		type = PCIE_EPC_IRQ_MSI;
		num = (uint16_t)strtoul(argv[3], NULL, 0);
	} else if (strcmp(argv[2], "msix") == 0 && argc > 3) {
		type = PCIE_EPC_IRQ_MSIX;
		num = (uint16_t)strtoul(argv[3], NULL, 0);
	} else {
		shell_error(sh, "intx | msi <n> | msix <n> (n from 1)");
		return -EINVAL;
	}
	ret = pcie_epc_raise_irq(epc, fn, type, num);
	shell_print(sh, "%s fn%u %s %u: %d", epc->name, fn, argv[2], num, ret);
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(pcie_ep_cmds,
	SHELL_CMD_ARG(irq, NULL, "<fn> intx | msi <n> | msix <n>  Raise an interrupt to the host",
		      cmd_ep_irq, 3, 1),
	SHELL_CMD_ARG(function, NULL,
		      "[<name>|reset] Endpoint function: list, or save the one for the next boot",
		      cmd_ep_function, 1, 1),
	SHELL_SUBCMD_SET_END);
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(pcie_ctrl_cmds,
	SHELL_CMD_ARG(info, NULL, "Controllers, role, link and state of the active one",
		      cmd_ctrl_info, 1, 0),
	SHELL_CMD_ARG(mode, NULL, "[rc|ep] Role, saved for the next boot", cmd_ctrl_mode, 1, 1),
	SHELL_CMD_ARG(link, NULL, "[retrain] Link state; retrain it (RC: PERST#, rescan)",
		      cmd_ctrl_link, 1, 1),
	SHELL_CMD_ARG(scan, NULL, "Enumerate again (root complex)", cmd_ctrl_scan, 1, 0),
	SHELL_CMD_ARG(irq, NULL, "Interrupt counters", cmd_ctrl_irq, 1, 0),
	SHELL_CMD_ARG(id, NULL,
		      "[<b:d.f>] [<vid:did> [<svid:ssid> [<class>]]] | [<b:d.f>] reset  "
		      "Identity (RC: root port, EP: function), saved",
		      cmd_ctrl_id, 1, 4),
#if defined(CONFIG_PCIE_EPC)
	SHELL_CMD(ep, &pcie_ep_cmds, "Endpoint: function, irq", NULL),
#endif
	SHELL_CMD_ARG(dsn, NULL, "[<b:d.f>] [<hex> | uid | new | reset]  Device Serial Number, saved",
		      cmd_ctrl_dsn, 1, 2),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(pcie_cfg_cmds,
	SHELL_CMD_ARG(set, NULL, "<b:d.f> <off> <val> Write a config dword", cmd_cfg_set, 4, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(pcie_cap_cmds,
	SHELL_CMD_ARG(ls, NULL, "<b:d.f> Capabilities", cmd_cap_ls, 2, 0),
	SHELL_CMD_ARG(get, NULL,
		      "<b:d.f> <cap> [<off>] Dump (exp, aer decoded); cap: name, id, e<id>, @off",
		      cmd_cap_get, 3, 1),
	SHELL_CMD_ARG(set, NULL, "<b:d.f> <cap> <off> <val> Write a dword of a capability",
		      cmd_cap_set, 5, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(pcie_cmds,
	SHELL_CMD_ARG(controller, &pcie_ctrl_cmds, "Controller: info, mode, link, scan, irq, id, ep, dsn",
		      cmd_ctrl_info, 1, 0),
	SHELL_CMD_ARG(ls, NULL, "[-v] Devices (EP: the local functions)", cmd_ls, 1, 1),
	SHELL_CMD_ARG(cfg, &pcie_cfg_cmds, "<b:d.f> [<off> [<len>]] Dump config space", cmd_cfg,
		      2, 2),
	SHELL_CMD(cap, &pcie_cap_cmds, "Capabilities: ls, get, set", NULL),
#if defined(CONFIG_PCIE_AER)
	SHELL_CMD_ARG(aer, NULL,
		      "[<b:d.f> | setup [ur] [urbridge] [ecrc] | clear | poll] Error reporting: "
		      "tree summary, one function, set up again, clear, service by hand",
		      cmd_aer, 1, 4),
#endif
	SHELL_CMD(debug, PCIE_DEBUG_SUBCMDS, "Controller specific bring-up and test commands",
		  NULL),
#if defined(CONFIG_PCIE_FABRIC_SHELL)
	SHELL_CMD(fabric, &pcie_fabric_cmds,
		  "Fabric: node, ping, send, recv, put, rma, poll; manager: ls, scan, up, down, "
		  "role", NULL),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pcie, &pcie_cmds, "PCI Express", NULL);
