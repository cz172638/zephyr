/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Poke PCI functions and links from a Linux host: config space through
 * libpci, link state and training on root and switch ports, AER, BAR memory
 * through sysfs resourceN, the pci-epf-test register block, and the sysfs
 * knobs (remove, rescan, reset, bind).
 *
 *   cc -O2 -Wall -o pcictl pcictl.c $(pkg-config --cflags --libs libpci)
 *
 *   pcictl ls                                  functions, BARs, drivers
 *   pcictl info   <bdf>                        ids, command, status, BARs, link, MSI
 *   pcictl cfg    <bdf> <off> [val]            config read / write
 *   pcictl cmd    <bdf> [+-mem +-bme +-io +-serr +-intx]   Command register
 *   pcictl rd     <bdf> <bar> <off> [count]    BAR read
 *   pcictl wr     <bdf> <bar> <off> <val>...   BAR write
 *   pcictl dump   <bdf> <bar> [off [len]]      BAR hex dump
 *   pcictl epf    <bdf> [bar]                  pci-epf-test registers
 *   pcictl link   <bdf> [status]               link speed, width, DLLLA, training, ASPM
 *   pcictl link   <port> retrain | disable | enable | speed <1..6> | aspm off|l0s|l1|l0s+l1
 *   pcictl hotreset <bridge> [ms]              Secondary Bus Reset below a bridge
 *   pcictl ports  <bridge>                     links of a switch (or root port) and below
 *   pcictl watch  <bdf> [ms]                   link, device, slot status as they change
 *   pcictl aer    <bdf> [clear]                AER and Device Status errors
 *   pcictl remove <bdf> | rescan | reset <bdf> | unbind <bdf> | bind <bdf> <drv>
 *
 * Options before the command: -w 8|16|32|64 access width (default 32; cfg
 * takes 8, 16 or 32), -q bare values from rd, -F <file> config space from an
 * "lspci -xxxx" dump instead of the hardware (no BARs, no sysfs). <bdf> as
 * lspci prints it.
 *
 * BAR accesses are single volatile loads and stores of the requested width
 * on an mmap of resourceN, in this process: a BAR that does not answer
 * raises SIGBUS here (reported, exit 3), not an oops in the host kernel.
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <pci/pci.h>

#define SYSFS "/sys/bus/pci/devices"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static struct pci_access *pacc;
static unsigned int width = 32;
static bool quiet;
static sigjmp_buf bus_jmp;
static volatile sig_atomic_t in_access;

static void __attribute__((noreturn, format(printf, 1, 2))) die(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
	exit(1);
}

static void __attribute__((noreturn)) usage(void)
{
	fputs("usage: pcictl [-w 8|16|32|64] [-q] [-F dump] ls | info <bdf> | cfg <bdf> <off> [val] |\n"
	      "       cmd <bdf> [+-mem +-bme +-io +-serr +-intx] | rd <bdf> <bar> <off> [count] |\n"
	      "       wr <bdf> <bar> <off> <val>... | dump <bdf> <bar> [off [len]] |\n"
	      "       epf <bdf> [bar] | link <bdf> [status | retrain | disable | enable |\n"
	      "       speed <1..6> | aspm off|l0s|l1|l0s+l1] | hotreset <bridge> [ms] |\n"
	      "       ports <bridge> | watch <bdf> [ms] | aer <bdf> [clear] |\n"
	      "       remove <bdf> | rescan | reset <bdf> |\n"
	      "       unbind <bdf> | bind <bdf> <driver>\n",
	      stderr);
	exit(2);
}

static uint64_t num(const char *s)
{
	char *end;
	uint64_t v;

	if (s == NULL) {
		usage();
	}
	errno = 0;
	v = strtoull(s, &end, 0);
	if (errno != 0 || *s == '\0' || *end != '\0') {
		die("%s: not a number", s);
	}
	return v;
}

static struct pci_dev *dev_get(const char *bdf)
{
	struct pci_filter f;
	char *err;

	if (bdf == NULL) {
		usage();
	}
	pci_filter_init(pacc, &f);
	err = pci_filter_parse_slot(&f, (char *)bdf);
	if (err != NULL || f.slot < 0 || f.func < 0 || f.bus < 0) {
		die("%s: not a bus:dev.fn", bdf);
	}
	for (struct pci_dev *d = pacc->devices; d != NULL; d = d->next) {
		if (pci_filter_match(&f, d)) {
			pci_fill_info(d, PCI_FILL_IDENT | PCI_FILL_CLASS | PCI_FILL_BASES |
						 PCI_FILL_SIZES | PCI_FILL_CAPS);
			return d;
		}
	}
	die("%s: no such function", bdf);
}

static void dev_path(const struct pci_dev *d, const char *file, char *buf, size_t len)
{
	/* PCICTL_SYSFS: another tree with resourceN files, for testing */
	const char *sys = getenv("PCICTL_SYSFS");

	snprintf(buf, len, "%s/%04x:%02x:%02x.%u/%s", sys != NULL ? sys : SYSFS, d->domain, d->bus,
		 d->dev, d->func, file);
}

static void sysfs_write(const char *path, const char *s)
{
	int fd = open(path, O_WRONLY);

	if (fd < 0) {
		die("%s: %s", path, strerror(errno));
	}
	if (write(fd, s, strlen(s)) != (ssize_t)strlen(s)) {
		die("%s: %s", path, strerror(errno));
	}
	close(fd);
}

static const char *driver(const struct pci_dev *d, char *buf, size_t len)
{
	char path[PATH_MAX];
	ssize_t n;
	char *p;

	dev_path(d, "driver", path, sizeof(path));
	n = readlink(path, buf, len - 1);
	if (n < 0) {
		return "-";
	}
	buf[n] = '\0';
	p = strrchr(buf, '/');
	return p != NULL ? p + 1 : buf;
}

static const char *size_str(uint64_t s, char *buf, size_t len)
{
	if (s >= (1ULL << 30)) {
		snprintf(buf, len, "%" PRIu64 "G", s >> 30);
	} else if (s >= (1ULL << 20)) {
		snprintf(buf, len, "%" PRIu64 "M", s >> 20);
	} else if (s >= (1ULL << 10)) {
		snprintf(buf, len, "%" PRIu64 "K", s >> 10);
	} else {
		snprintf(buf, len, "%" PRIu64, s);
	}
	return buf;
}

static bool print_bars(const struct pci_dev *d, const char *sep, const char *end)
{
	bool first = true;

	for (int i = 0; i < 6; i++) {
		char sz[16];

		if (d->size[i] == 0) {
			continue;
		}
		printf("%sBAR%d %s 0x%" PRIx64 " %s", first ? "" : sep, i,
		       (d->base_addr[i] & PCI_BASE_ADDRESS_SPACE_IO) ? "io" : "mem",
		       (uint64_t)(d->base_addr[i] & PCI_ADDR_MEM_MASK),
		       size_str(d->size[i], sz, sizeof(sz)));
		first = false;
	}
	if (!first) {
		fputs(end, stdout);
	}
	return !first;
}

static int dev_cmp(const void *a, const void *b)
{
	const struct pci_dev *x = *(struct pci_dev *const *)a;
	const struct pci_dev *y = *(struct pci_dev *const *)b;
	uint32_t kx = ((uint32_t)x->domain << 16) | ((uint32_t)x->bus << 8) | (x->dev << 3) |
		      x->func;
	uint32_t ky = ((uint32_t)y->domain << 16) | ((uint32_t)y->bus << 8) | (y->dev << 3) |
		      y->func;

	return (kx > ky) - (kx < ky);
}

/* libpci lists functions in no particular order: sort them as lspci does */
static void sort_devices(void)
{
	struct pci_dev **v;
	size_t n = 0, i = 0;

	for (struct pci_dev *d = pacc->devices; d != NULL; d = d->next) {
		n++;
	}
	if (n < 2U) {
		return;
	}
	v = calloc(n, sizeof(*v));
	if (v == NULL) {
		die("out of memory");
	}
	for (struct pci_dev *d = pacc->devices; d != NULL; d = d->next) {
		v[i++] = d;
	}
	qsort(v, n, sizeof(*v), dev_cmp);
	for (i = 0; i + 1U < n; i++) {
		v[i]->next = v[i + 1U];
	}
	v[n - 1U]->next = NULL;
	pacc->devices = v[0];
	free(v);
}

/* --- BAR memory ----------------------------------------------------------- */

static void on_sigbus(int sig)
{
	(void)sig;
	if (in_access) {
		siglongjmp(bus_jmp, 1);
	}
	_exit(3);
}

struct bar_map {
	volatile uint8_t *va;
	uint64_t size;
};

static struct bar_map bar_map(const struct pci_dev *d, unsigned int bar)
{
	char path[PATH_MAX], name[16];
	struct bar_map m;
	struct stat st;
	int fd;

	if (bar > 5U) {
		die("BAR%u: 0..5", bar);
	}
	snprintf(name, sizeof(name), "resource%u", bar);
	dev_path(d, name, path, sizeof(path));
	fd = open(path, O_RDWR | O_SYNC);
	if (fd < 0) {
		die("%s: %s", path, strerror(errno));
	}
	if (fstat(fd, &st) != 0 || st.st_size == 0) {
		die("%s: BAR has no size", path);
	}
	m.size = (uint64_t)st.st_size;
	m.va = mmap(NULL, m.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (m.va == MAP_FAILED) {
		die("%s: mmap: %s", path, strerror(errno));
	}
	close(fd);
	return m;
}

static void check_off(const struct bar_map *m, unsigned int bar, uint64_t off)
{
	unsigned int n = width / 8U;

	if (off + n > m->size) {
		die("BAR%u+0x%" PRIx64 ": outside the %" PRIu64 "-byte BAR", bar, off, m->size);
	}
	if ((off % n) != 0U) {
		die("BAR%u+0x%" PRIx64 ": not aligned to %u bytes", bar, off, n);
	}
}

static void __attribute__((noreturn)) bus_error(unsigned int bar, uint64_t off, const char *op)
{
	fprintf(stderr, "BAR%u+0x%" PRIx64 ": %s aborted (SIGBUS): the function did not answer\n",
		bar, off, op);
	exit(3);
}

static uint64_t bar_rd(const struct bar_map *m, unsigned int bar, uint64_t off)
{
	volatile uint8_t *p = m->va + off;
	uint64_t v = 0;

	check_off(m, bar, off);
	if (sigsetjmp(bus_jmp, 1) != 0) {
		in_access = 0;
		bus_error(bar, off, "read");
	}
	in_access = 1;
	switch (width) {
	case 8:
		v = *p;
		break;
	case 16:
		v = *(volatile uint16_t *)p;
		break;
	case 32:
		v = *(volatile uint32_t *)p;
		break;
	default:
		v = *(volatile uint64_t *)p;
		break;
	}
	in_access = 0;
	return v;
}

static void bar_wr(const struct bar_map *m, unsigned int bar, uint64_t off, uint64_t v)
{
	volatile uint8_t *p = m->va + off;

	check_off(m, bar, off);
	if (sigsetjmp(bus_jmp, 1) != 0) {
		in_access = 0;
		bus_error(bar, off, "write");
	}
	in_access = 1;
	switch (width) {
	case 8:
		*p = (uint8_t)v;
		break;
	case 16:
		*(volatile uint16_t *)p = (uint16_t)v;
		break;
	case 32:
		*(volatile uint32_t *)p = (uint32_t)v;
		break;
	default:
		*(volatile uint64_t *)p = v;
		break;
	}
	__sync_synchronize();
	in_access = 0;
}

/* --- commands ------------------------------------------------------------- */

static void cmd_ls(void)
{
	for (struct pci_dev *d = pacc->devices; d != NULL; d = d->next) {
		char name[128], drv[PATH_MAX];

		pci_fill_info(d, PCI_FILL_IDENT | PCI_FILL_CLASS | PCI_FILL_BASES | PCI_FILL_SIZES);
		printf("%02x:%02x.%u %04x:%04x %-40.40s %-18s ", d->bus, d->dev, d->func,
		       d->vendor_id, d->device_id,
		       pci_lookup_name(pacc, name, sizeof(name),
				       PCI_LOOKUP_VENDOR | PCI_LOOKUP_DEVICE, d->vendor_id,
				       d->device_id),
		       driver(d, drv, sizeof(drv)));
		print_bars(d, ", ", "");
		putchar('\n');
	}
}

static const char *flags(unsigned int v, const unsigned int *bit, const char *const *name,
			 size_t n, char *buf, size_t len)
{
	size_t o = 0;

	buf[0] = '\0';
	for (size_t i = 0; i < n; i++) {
		if ((v & bit[i]) != 0U && o < len) {
			o += snprintf(buf + o, len - o, "%s%s", o ? " " : "", name[i]);
		}
	}
	return o ? buf : "-";
}

static void cmd_info(struct pci_dev *d)
{
	static const unsigned int st_bit[] = {0x0100, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000};
	static const char *const st_name[] = {"master-data-parity", "sig-target-abort",
					      "rcv-target-abort", "rcv-master-abort",
					      "sig-system-error", "parity-error"};
	static const char *const speed[] = {"?", "2.5", "5", "8", "16", "32", "64"};
	uint16_t cmd = pci_read_word(d, PCI_COMMAND);
	uint16_t st = pci_read_word(d, PCI_STATUS);
	struct pci_cap *cap;
	char buf[256], drv[PATH_MAX];

	printf("%04x:%02x:%02x.%u %04x:%04x class %06x driver %s\n", d->domain, d->bus, d->dev,
	       d->func, d->vendor_id, d->device_id, pci_read_long(d, PCI_CLASS_REVISION) >> 8,
	       driver(d, drv, sizeof(drv)));
	printf("command %04x: io %u mem %u bme %u serr %u intx-disable %u\n", cmd, cmd & 1U,
	       (cmd >> 1) & 1U, (cmd >> 2) & 1U, (cmd >> 8) & 1U, (cmd >> 10) & 1U);
	printf("status  %04x: %s\n", st,
	       flags(st, st_bit, st_name, ARRAY_SIZE(st_bit), buf, sizeof(buf)));
	print_bars(d, "\n", "\n");

	cap = pci_find_cap(d, PCI_CAP_ID_EXP, PCI_CAP_NORMAL);
	if (cap != NULL) {
		uint16_t ds = pci_read_word(d, cap->addr + PCI_EXP_DEVSTA);
		uint16_t ls = pci_read_word(d, cap->addr + PCI_EXP_LNKSTA);
		unsigned int s = ls & PCI_EXP_LNKSTA_SPEED;

		printf("devsta %04x, link %s GT/s x%u\n", ds,
		       s < ARRAY_SIZE(speed) ? speed[s] : "?", (ls & PCI_EXP_LNKSTA_WIDTH) >> 4);
	}
	cap = pci_find_cap(d, PCI_CAP_ID_MSI, PCI_CAP_NORMAL);
	if (cap != NULL) {
		uint16_t c = pci_read_word(d, cap->addr + PCI_MSI_FLAGS);
		bool is64 = (c & PCI_MSI_FLAGS_64BIT) != 0U;
		uint64_t addr = pci_read_long(d, cap->addr + PCI_MSI_ADDRESS_LO);

		if (is64) {
			addr |= (uint64_t)pci_read_long(d, cap->addr + PCI_MSI_ADDRESS_HI) << 32;
		}
		printf("msi %s %u/%u vectors, addr 0x%" PRIx64 " data 0x%04x\n",
		       (c & PCI_MSI_FLAGS_ENABLE) ? "on" : "off", 1U << ((c >> 4) & 7U),
		       1U << ((c >> 1) & 7U), addr,
		       pci_read_word(d, cap->addr + (is64 ? PCI_MSI_DATA_64 : PCI_MSI_DATA_32)));
	}
}

static void cmd_cfg(struct pci_dev *d, int argc, char **argv)
{
	unsigned int off = (unsigned int)num(argv[0]);
	int ok = 1;

	if (argc > 1) {
		uint64_t v = num(argv[1]);

		switch (width) {
		case 8:
			ok = pci_write_byte(d, off, (uint8_t)v);
			break;
		case 16:
			ok = pci_write_word(d, off, (uint16_t)v);
			break;
		case 32:
			ok = pci_write_long(d, off, (uint32_t)v);
			break;
		default:
			die("cfg: -w 8, 16 or 32");
		}
		if (!ok) {
			die("config write 0x%x failed (not root?)", off);
		}
		return;
	}
	switch (width) {
	case 8:
		printf("%02x\n", pci_read_byte(d, off));
		break;
	case 16:
		printf("%04x\n", pci_read_word(d, off));
		break;
	case 32:
		printf("%08x\n", pci_read_long(d, off));
		break;
	default:
		die("cfg: -w 8, 16 or 32");
	}
}

static void cmd_cmd(struct pci_dev *d, int argc, char **argv)
{
	static const struct {
		const char *name;
		uint16_t bit;
	} bits[] = {
		{"io", PCI_COMMAND_IO},     {"mem", PCI_COMMAND_MEMORY},
		{"bme", PCI_COMMAND_MASTER}, {"serr", PCI_COMMAND_SERR},
		{"intx", 0x400},
	};
	uint16_t v = pci_read_word(d, PCI_COMMAND);

	for (int i = 0; i < argc; i++) {
		const char *a = argv[i];
		size_t j;

		if (a[0] != '+' && a[0] != '-') {
			die("%s: +|-io, mem, bme, serr or intx", a);
		}
		for (j = 0; j < ARRAY_SIZE(bits); j++) {
			if (strcmp(a + 1, bits[j].name) == 0) {
				break;
			}
		}
		if (j == ARRAY_SIZE(bits)) {
			die("%s: +|-io, mem, bme, serr or intx", a);
		}
		/* +intx enables INTx: clears Interrupt Disable */
		if ((a[0] == '+') != (bits[j].bit == 0x400)) {
			v |= bits[j].bit;
		} else {
			v &= ~bits[j].bit;
		}
	}
	if (argc > 0 && !pci_write_word(d, PCI_COMMAND, v)) {
		die("Command write failed (not root?)");
	}
	printf("command %04x\n", pci_read_word(d, PCI_COMMAND));
}

static void print_val(unsigned int bar, uint64_t off, uint64_t v)
{
	if (quiet) {
		printf("%0*" PRIx64 "\n", (int)(width / 4U), v);
	} else {
		printf("BAR%u+0x%" PRIx64 ": %0*" PRIx64 "\n", bar, off, (int)(width / 4U), v);
	}
}

static void cmd_rd(struct pci_dev *d, int argc, char **argv)
{
	unsigned int bar = (unsigned int)num(argv[0]);
	uint64_t off = num(argc > 1 ? argv[1] : NULL);
	uint64_t count = argc > 2 ? num(argv[2]) : 1;
	struct bar_map m = bar_map(d, bar);

	for (uint64_t i = 0; i < count; i++) {
		uint64_t o = off + i * (width / 8U);

		print_val(bar, o, bar_rd(&m, bar, o));
	}
}

static void cmd_wr(struct pci_dev *d, int argc, char **argv)
{
	unsigned int bar = (unsigned int)num(argv[0]);
	uint64_t off = num(argc > 1 ? argv[1] : NULL);
	struct bar_map m = bar_map(d, bar);

	if (argc < 3) {
		usage();
	}
	for (int i = 2; i < argc; i++) {
		bar_wr(&m, bar, off, num(argv[i]));
		off += width / 8U;
	}
}

static void cmd_dump(struct pci_dev *d, int argc, char **argv)
{
	unsigned int bar = (unsigned int)num(argv[0]);
	uint64_t off = argc > 1 ? num(argv[1]) : 0;
	struct bar_map m = bar_map(d, bar);
	uint64_t len = argc > 2 ? num(argv[2]) : (m.size - off < 256U ? m.size - off : 256U);
	unsigned int n = width / 8U;

	for (uint64_t o = off; o < off + len; o += 16U) {
		printf("%06" PRIx64 ":", o);
		for (uint64_t p = o; p < o + 16U && p < off + len; p += n) {
			printf(" %0*" PRIx64, (int)(width / 4U), bar_rd(&m, bar, p));
		}
		putchar('\n');
	}
}

/* --- link, switch ports, errors ------------------------------------------- */

#define EXP_TYPE_ROOT_PORT 0x4
#define EXP_TYPE_UPSTREAM  0x5
#define EXP_TYPE_DOWNSTREAM 0x6
#define LNKCTL_ASPM        0x0003
#define LNKCTL_DISABLE     0x0010
#define LNKCTL_RETRAIN     0x0020
#define LNKSTA_TRAIN       0x0800
#define LNKSTA_SLOT_CLK    0x1000
#define LNKSTA_DLLLA       0x2000
#define LNKSTA_BW_MGMT     0x4000
#define LNKSTA_AUTO_BW     0x8000
#define SLTSTA_PRESENT     0x0040
#define BCTL_BUS_RESET     0x0040
#define EXT_CAP_AER        0x0001

static const char *const speed_str[] = {"?", "2.5", "5", "8", "16", "32", "64"};

static const char *spd(unsigned int s)
{
	return s < ARRAY_SIZE(speed_str) ? speed_str[s] : "?";
}

static int exp_cap(struct pci_dev *d)
{
	struct pci_cap *cap = pci_find_cap(d, PCI_CAP_ID_EXP, PCI_CAP_NORMAL);

	return cap != NULL ? (int)cap->addr : -1;
}

static unsigned int port_type(struct pci_dev *d, int e)
{
	return (pci_read_word(d, e + PCI_EXP_FLAGS) & PCI_EXP_FLAGS_TYPE) >> 4;
}

static const char *type_str(unsigned int t)
{
	static const char *const n[] = {"endpoint", "legacy-ep", "?", "?", "root-port",
					"upstream", "downstream", "pcie-pci", "pci-pcie",
					"rc-ep", "rc-ec"};

	return t < ARRAY_SIZE(n) ? n[t] : "?";
}

static int exp_cap_port(struct pci_dev *d, const char *what)
{
	int e = exp_cap(d);
	unsigned int t;

	if (e < 0) {
		die("%02x:%02x.%u: no PCI Express capability", d->bus, d->dev, d->func);
	}
	t = port_type(d, e);
	if (t != EXP_TYPE_ROOT_PORT && t != EXP_TYPE_DOWNSTREAM) {
		die("%02x:%02x.%u: %s, link %s needs the root or downstream port above the link",
		    d->bus, d->dev, d->func, type_str(t), what);
	}
	return e;
}

static void link_line(struct pci_dev *d, int e)
{
	uint32_t cap = pci_read_long(d, e + PCI_EXP_LNKCAP);
	uint16_t ctl = pci_read_word(d, e + PCI_EXP_LNKCTL);
	uint16_t sta = pci_read_word(d, e + PCI_EXP_LNKSTA);
	uint16_t ctl2 = pci_read_word(d, e + PCI_EXP_LNKCTL2);
	static const char *const aspm[] = {"off", "L0s", "L1", "L0s+L1"};

	printf("%02x:%02x.%u %-10s link %s GT/s x%u (cap %s x%u, target %s)%s%s%s%s%s, aspm %s%s\n",
	       d->bus, d->dev, d->func, type_str(port_type(d, e)), spd(sta & 0xfU),
	       (sta >> 4) & 0x3fU, spd(cap & 0xfU), (cap >> 4) & 0x3fU, spd(ctl2 & 0xfU),
	       (sta & LNKSTA_DLLLA) ? " dl-active" : "", (sta & LNKSTA_TRAIN) ? " training" : "",
	       (sta & LNKSTA_SLOT_CLK) ? " common-clk" : "",
	       (sta & LNKSTA_BW_MGMT) ? " bw-mgmt" : "", (sta & LNKSTA_AUTO_BW) ? " auto-bw" : "",
	       aspm[ctl & LNKCTL_ASPM], (ctl & LNKCTL_DISABLE) ? ", DISABLED" : "");
}

static void wait_train(struct pci_dev *d, int e, unsigned int ms)
{
	/* without DLL Link Active Reporting, only the end of training is visible */
	bool dllla = (pci_read_long(d, e + PCI_EXP_LNKCAP) & (1U << 20)) != 0U;

	usleep(1000);
	for (unsigned int t = 1; t < ms; t++) {
		uint16_t sta = pci_read_word(d, e + PCI_EXP_LNKSTA);

		if ((sta & LNKSTA_TRAIN) == 0U && (!dllla || (sta & LNKSTA_DLLLA) != 0U)) {
			printf("trained after %u ms\n", t);
			return;
		}
		usleep(1000);
	}
	printf("not up after %u ms\n", ms);
}

/*
 * link <port> [retrain | disable | enable | speed <1..6> | aspm off|l0s|l1|l0s+l1]
 * on the root or downstream port above the link.
 */
static void cmd_link(struct pci_dev *d, int argc, char **argv)
{
	int e;
	uint16_t ctl;

	if (argc == 0 || strcmp(argv[0], "status") == 0) {
		e = exp_cap(d);
		if (e < 0) {
			die("no PCI Express capability");
		}
		link_line(d, e);
		return;
	}
	e = exp_cap_port(d, argv[0]);
	ctl = pci_read_word(d, e + PCI_EXP_LNKCTL);
	if (strcmp(argv[0], "retrain") == 0) {
		pci_write_word(d, e + PCI_EXP_LNKCTL, ctl | LNKCTL_RETRAIN);
		wait_train(d, e, 1000);
	} else if (strcmp(argv[0], "disable") == 0) {
		pci_write_word(d, e + PCI_EXP_LNKCTL, ctl | LNKCTL_DISABLE);
	} else if (strcmp(argv[0], "enable") == 0) {
		pci_write_word(d, e + PCI_EXP_LNKCTL, ctl & ~LNKCTL_DISABLE);
		wait_train(d, e, 1000);
	} else if (strcmp(argv[0], "speed") == 0 && argc > 1) {
		uint64_t s = num(argv[1]);
		uint16_t c2 = pci_read_word(d, e + PCI_EXP_LNKCTL2);

		if (s < 1U || s > 6U) {
			die("speed: 1 (2.5) .. 6 (64 GT/s)");
		}
		pci_write_word(d, e + PCI_EXP_LNKCTL2, (c2 & ~0xfU) | (uint16_t)s);
		pci_write_word(d, e + PCI_EXP_LNKCTL, ctl | LNKCTL_RETRAIN);
		wait_train(d, e, 1000);
	} else if (strcmp(argv[0], "aspm") == 0 && argc > 1) {
		static const char *const n[] = {"off", "l0s", "l1", "l0s+l1"};
		size_t i;

		for (i = 0; i < ARRAY_SIZE(n); i++) {
			if (strcmp(argv[1], n[i]) == 0) {
				break;
			}
		}
		if (i == ARRAY_SIZE(n)) {
			die("aspm: off, l0s, l1 or l0s+l1");
		}
		pci_write_word(d, e + PCI_EXP_LNKCTL, (ctl & ~LNKCTL_ASPM) | (uint16_t)i);
	} else {
		usage();
	}
	link_line(d, e);
}

/* Secondary Bus Reset on a bridge: hot reset of everything below it */
static void cmd_hotreset(struct pci_dev *d, int argc, char **argv)
{
	uint64_t ms = argc > 0 ? num(argv[0]) : 2;
	uint16_t bctl = pci_read_word(d, PCI_BRIDGE_CONTROL);
	int e;

	if ((pci_read_byte(d, PCI_HEADER_TYPE) & 0x7fU) != PCI_HEADER_TYPE_BRIDGE) {
		die("%02x:%02x.%u: not a bridge", d->bus, d->dev, d->func);
	}
	pci_write_word(d, PCI_BRIDGE_CONTROL, bctl | BCTL_BUS_RESET);
	usleep((useconds_t)ms * 1000U);
	pci_write_word(d, PCI_BRIDGE_CONTROL, bctl & ~BCTL_BUS_RESET);
	printf("secondary bus reset %" PRIu64 " ms\n", ms);
	e = exp_cap(d);
	if (e >= 0) {
		wait_train(d, e, 1000);
		link_line(d, e);
	}
	puts("functions below lost their config: remove them and rescan");
}

/* ASMedia ASM1184e per-port training flags, guessed (see pcivsec.pl ports) */
static void asm1184_ports(struct pci_dev *d)
{
	static const unsigned int port[] = {1, 3, 5, 7};

	for (unsigned int n = 0; n < ARRAY_SIZE(port); n++) {
		unsigned int blk = 0x400U + 0x80U * n;
		uint32_t f = pci_read_long(d, blk + 0x4cU);
		uint32_t s = pci_read_long(d, blk + 0x50U);
		unsigned int fl = f >> 24;
		const char *what = fl == 0x7fU ? "L0" : fl == 0x21U ? "empty" :
				   fl == 0x6dU ? "detected, no lock" :
				   fl == 0x13U ? "was up, now down" : "?";

		printf("    asm1184 port %u: flags %02x +50 %08x %s\n", port[n], fl, s, what);
	}
}

/* ports <bridge>: the bridge's own link and the links of the ports below it */
static void cmd_ports(struct pci_dev *d)
{
	unsigned int sec;
	int e = exp_cap(d);

	if ((pci_read_byte(d, PCI_HEADER_TYPE) & 0x7fU) != PCI_HEADER_TYPE_BRIDGE) {
		die("%02x:%02x.%u: not a bridge", d->bus, d->dev, d->func);
	}
	if (e >= 0) {
		link_line(d, e);
	}
	if (d->vendor_id == 0x1b21 && d->device_id == 0x1184) {
		asm1184_ports(d);
	}
	sec = pci_read_byte(d, PCI_SECONDARY_BUS);
	for (struct pci_dev *c = pacc->devices; c != NULL; c = c->next) {
		int ce;

		if (c->domain != d->domain || c->bus != sec) {
			continue;
		}
		pci_fill_info(c, PCI_FILL_IDENT | PCI_FILL_CAPS);
		ce = exp_cap(c);
		if (ce < 0) {
			printf("  %02x:%02x.%u %04x:%04x (not PCIe)\n", c->bus, c->dev, c->func,
			       c->vendor_id, c->device_id);
			continue;
		}
		printf("  ");
		link_line(c, ce);
		if (port_type(c, ce) == EXP_TYPE_DOWNSTREAM ||
		    port_type(c, ce) == EXP_TYPE_ROOT_PORT) {
			uint16_t ss = pci_read_word(c, ce + PCI_EXP_SLTSTA);
			unsigned int csec = pci_read_byte(c, PCI_SECONDARY_BUS);

			printf("    slot sta %04x%s, secondary bus %02x:", ss,
			       (ss & SLTSTA_PRESENT) ? " present" : "", csec);
			for (struct pci_dev *g = pacc->devices; g != NULL; g = g->next) {
				if (g->domain == c->domain && g->bus == csec) {
					pci_fill_info(g, PCI_FILL_IDENT);
					printf(" %02x.%u %04x:%04x", g->dev, g->func, g->vendor_id,
					       g->device_id);
				}
			}
			putchar('\n');
		}
	}
}

/* watch <bdf> [ms]: link, device and slot status as they change */
static void cmd_watch(struct pci_dev *d, int argc, char **argv)
{
	uint64_t ms = argc > 0 ? num(argv[0]) : 10;
	int e = exp_cap(d);
	uint32_t last = 0xffffffffU, last2 = 0xffffffffU;
	struct timespec t0, t;

	if (e < 0) {
		die("no PCI Express capability");
	}
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (;;) {
		uint32_t a = ((uint32_t)pci_read_word(d, e + PCI_EXP_LNKSTA) << 16) |
			     pci_read_word(d, e + PCI_EXP_DEVSTA);
		uint32_t b = ((uint32_t)pci_read_word(d, e + PCI_EXP_SLTSTA) << 16) |
			     pci_read_word(d, PCI_STATUS);

		if (a != last || b != last2) {
			clock_gettime(CLOCK_MONOTONIC, &t);
			printf("%10.3f lnksta %04x (%s GT/s x%u%s%s) devsta %04x sltsta %04x "
			       "status %04x\n",
			       (double)(t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9,
			       a >> 16, spd((a >> 16) & 0xfU), (a >> 20) & 0x3fU,
			       ((a >> 16) & LNKSTA_DLLLA) ? " dl-active" : "",
			       ((a >> 16) & LNKSTA_TRAIN) ? " training" : "", a & 0xffffU, b >> 16,
			       b & 0xffffU);
			fflush(stdout);
			last = a;
			last2 = b;
		}
		usleep((useconds_t)ms * 1000U);
	}
}

/* aer <bdf> [clear]: AER status, decoded; clear writes the set bits back */
static void cmd_aer(struct pci_dev *d, int argc, char **argv)
{
	static const char *const unc[32] = {
		[4] = "DLP", [5] = "SDES", [12] = "TLP-poisoned", [13] = "FCP",
		[14] = "cpl-timeout", [15] = "cpl-abort", [16] = "unexp-cpl", [17] = "rx-overflow",
		[18] = "malformed", [19] = "ECRC", [20] = "UR", [21] = "ACS", [22] = "internal",
	};
	static const char *const cor[32] = {
		[0] = "rx-err", [6] = "bad-TLP", [7] = "bad-DLLP", [8] = "replay-rollover",
		[12] = "replay-timeout", [13] = "advisory-nonfatal", [14] = "internal",
		[15] = "header-log-overflow",
	};
	struct pci_cap *cap;
	uint32_t u, c, ds;
	int e = exp_cap(d);

	pci_fill_info(d, PCI_FILL_EXT_CAPS);
	cap = pci_find_cap(d, EXT_CAP_AER, PCI_CAP_EXTENDED);
	if (e >= 0) {
		ds = pci_read_word(d, e + PCI_EXP_DEVSTA);
		printf("devsta %04x:%s%s%s%s\n", ds, (ds & 1U) ? " correctable" : "",
		       (ds & 2U) ? " non-fatal" : "", (ds & 4U) ? " fatal" : "",
		       (ds & 8U) ? " unsupported-request" : "");
	}
	if (cap == NULL) {
		puts("no AER capability");
	} else {
		u = pci_read_long(d, cap->addr + PCI_ERR_UNCOR_STATUS);
		c = pci_read_long(d, cap->addr + PCI_ERR_COR_STATUS);
		printf("uncorrectable %08x:", u);
		for (unsigned int i = 0; i < 32U; i++) {
			if (u & (1U << i)) {
				printf(" %s", unc[i] != NULL ? unc[i] : "?");
			}
		}
		printf("\ncorrectable   %08x:", c);
		for (unsigned int i = 0; i < 32U; i++) {
			if (c & (1U << i)) {
				printf(" %s", cor[i] != NULL ? cor[i] : "?");
			}
		}
		printf("\nheader log    %08x %08x %08x %08x\n",
		       pci_read_long(d, cap->addr + PCI_ERR_HEADER_LOG),
		       pci_read_long(d, cap->addr + PCI_ERR_HEADER_LOG + 4),
		       pci_read_long(d, cap->addr + PCI_ERR_HEADER_LOG + 8),
		       pci_read_long(d, cap->addr + PCI_ERR_HEADER_LOG + 12));
		if (argc > 0 && strcmp(argv[0], "clear") == 0) {
			pci_write_long(d, cap->addr + PCI_ERR_UNCOR_STATUS, u);
			pci_write_long(d, cap->addr + PCI_ERR_COR_STATUS, c);
		}
	}
	if (argc > 0 && strcmp(argv[0], "clear") == 0) {
		if (e >= 0) {
			pci_write_word(d, e + PCI_EXP_DEVSTA, 0x000f);
		}
		/* Status: the error bits are RW1C */
		pci_write_word(d, PCI_STATUS, 0xf900);
		puts("cleared");
	}
}

/* pci-epf-test register block (Linux drivers/misc/pci_endpoint_test.c) */
static void cmd_epf(struct pci_dev *d, int argc, char **argv)
{
	static const unsigned int cmd_bit[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
	static const char *const cmd_name[] = {"raise-intx", "raise-msi",       "raise-msix",
					       "read",       "write",           "copy",
					       "enable-doorbell", "disable-doorbell"};
	static const unsigned int st_bit[] = {0x001, 0x002, 0x004, 0x008, 0x010,
					      0x020, 0x040, 0x080, 0x100};
	static const char *const st_name[] = {"read-ok",    "read-fail",  "write-ok",
					      "write-fail", "copy-ok",    "copy-fail",
					      "irq-raised", "src-invalid", "dst-invalid"};
	static const unsigned int cap_bit[] = {0x01, 0x02, 0x04, 0x08, 0x10,
					       0x20, 0x40, 0x80, 0x100, 0x200};
	static const char *const cap_name[] = {"unaligned", "intx", "msi", "msix",
					       "bar0-reserved", "bar1-reserved", "bar2-reserved",
					       "bar3-reserved", "bar4-reserved", "bar5-reserved"};
	static const char *const irq[] = {"intx", "msi", "msix"};
	unsigned int bar = argc > 0 ? (unsigned int)num(argv[0]) : 0U;
	struct bar_map m = bar_map(d, bar);
	uint32_t r[13];
	char buf[256];

	width = 32;
	for (unsigned int i = 0; i < ARRAY_SIZE(r); i++) {
		r[i] = (uint32_t)bar_rd(&m, bar, i * 4U);
	}
	printf("magic    %08x\n", r[0]);
	printf("command  %08x %s\n", r[1],
	       flags(r[1], cmd_bit, cmd_name, ARRAY_SIZE(cmd_bit), buf, sizeof(buf)));
	printf("status   %08x %s\n", r[2],
	       flags(r[2], st_bit, st_name, ARRAY_SIZE(st_bit), buf, sizeof(buf)));
	printf("src      0x%" PRIx64 "\n", ((uint64_t)r[4] << 32) | r[3]);
	printf("dst      0x%" PRIx64 "\n", ((uint64_t)r[6] << 32) | r[5]);
	printf("size     %u\n", r[7]);
	printf("checksum %08x\n", r[8]);
	printf("irq      type %s number %u\n", r[9] < ARRAY_SIZE(irq) ? irq[r[9]] : "?", r[10]);
	printf("flags    %08x%s\n", r[11], (r[11] & 1U) ? " use-dma" : "");
	printf("caps     %08x %s\n", r[12],
	       flags(r[12], cap_bit, cap_name, ARRAY_SIZE(cap_bit), buf, sizeof(buf)));
}

int main(int argc, char **argv)
{
	struct sigaction sa = {.sa_handler = on_sigbus};
	char path[PATH_MAX];
	const char *c;
	int opt;
	const char *dump_file = NULL;

	while ((opt = getopt(argc, argv, "+w:qF:")) != -1) {
		switch (opt) {
		case 'w':
			width = (unsigned int)num(optarg);
			if (width != 8U && width != 16U && width != 32U && width != 64U) {
				die("-w: 8, 16, 32 or 64");
			}
			break;
		case 'q':
			quiet = true;
			break;
		case 'F':
			dump_file = optarg;
			break;
		default:
			usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc < 1) {
		usage();
	}
	c = argv[0];

	if (strcmp(c, "rescan") == 0) {
		sysfs_write("/sys/bus/pci/rescan", "1");
		return 0;
	}

	sigaction(SIGBUS, &sa, NULL);
	pacc = pci_alloc();
	if (dump_file != NULL) {
		pacc->method = PCI_ACCESS_DUMP;
		pci_set_param(pacc, "dump.name", (char *)dump_file);
	}
	pci_init(pacc);
	pci_scan_bus(pacc);
	sort_devices();

	if (strcmp(c, "ls") == 0) {
		cmd_ls();
	} else if (argc < 2) {
		usage();
	} else {
		struct pci_dev *d = dev_get(argv[1]);
		int n = argc - 2;
		char **a = argv + 2;

		if (strcmp(c, "info") == 0) {
			cmd_info(d);
		} else if (strcmp(c, "cfg") == 0 && n >= 1) {
			cmd_cfg(d, n, a);
		} else if (strcmp(c, "cmd") == 0) {
			cmd_cmd(d, n, a);
		} else if (strcmp(c, "rd") == 0 && n >= 2) {
			cmd_rd(d, n, a);
		} else if (strcmp(c, "wr") == 0 && n >= 3) {
			cmd_wr(d, n, a);
		} else if (strcmp(c, "dump") == 0 && n >= 1) {
			cmd_dump(d, n, a);
		} else if (strcmp(c, "epf") == 0) {
			cmd_epf(d, n, a);
		} else if (strcmp(c, "link") == 0) {
			cmd_link(d, n, a);
		} else if (strcmp(c, "hotreset") == 0) {
			cmd_hotreset(d, n, a);
		} else if (strcmp(c, "ports") == 0) {
			cmd_ports(d);
		} else if (strcmp(c, "watch") == 0) {
			cmd_watch(d, n, a);
		} else if (strcmp(c, "aer") == 0) {
			cmd_aer(d, n, a);
		} else if (strcmp(c, "remove") == 0) {
			dev_path(d, "remove", path, sizeof(path));
			sysfs_write(path, "1");
		} else if (strcmp(c, "reset") == 0) {
			dev_path(d, "reset", path, sizeof(path));
			sysfs_write(path, "1");
		} else if (strcmp(c, "unbind") == 0) {
			char bdf[16];

			dev_path(d, "driver/unbind", path, sizeof(path));
			snprintf(bdf, sizeof(bdf), "%04x:%02x:%02x.%u", d->domain, d->bus, d->dev,
				 d->func);
			if (access(path, W_OK) == 0) {
				sysfs_write(path, bdf);
			}
		} else if (strcmp(c, "bind") == 0 && n >= 1) {
			char bdf[16];

			snprintf(path, sizeof(path), "/sys/bus/pci/drivers/%s/bind", a[0]);
			snprintf(bdf, sizeof(bdf), "%04x:%02x:%02x.%u", d->domain, d->bus, d->dev,
				 d->func);
			sysfs_write(path, bdf);
		} else {
			usage();
		}
	}
	pci_cleanup(pacc);
	return 0;
}
