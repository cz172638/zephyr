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
 *   pcictl test   <bdf> [-b bar] [-t ms] [-d] [-k] [-l limit] <item>...
 *                 pci-epf-test over vfio-pci (No-IOMMU): bars, bar<n>, msi, msi<n>,
 *                 msix[<n>], read:<size>, write:<size>, copy:<size> (u: unaligned),
 *                 doorbell, all; -k rings the endpoint's doorbell after each
 *                 command (no waiting for its poll)
 *   pcictl buf    [-l limit] [ms]              DMA buffer for the endpoint's requests
 *   pcictl remove <bdf> | rescan | reset <bdf> | unbind <bdf> | bind <bdf> <drv>
 *   pcictl resize <bdf> [<bar> <size>]
 *                 Resizable BAR: the sizes on offer and the current ones;
 *                 with bar and size (1M, 4M, ...) unbind the driver, let the
 *                 kernel resize and reassign the windows, rebind
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
#include <linux/vfio.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/random.h>
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
	fputs("usage: pcictl [-w 8|16|32|64] [-q] [-F dump] ls | info <bdf> |\n"
	      "       cfg <bdf> <off> [val] |\n"
	      "       cmd <bdf> [+-mem +-bme +-io +-serr +-intx] | rd <bdf> <bar> <off> [count] |\n"
	      "       wr <bdf> <bar> <off> <val>... | dump <bdf> <bar> [off [len]] |\n"
	      "       epf <bdf> [bar] | link <bdf> [status | retrain | disable | enable |\n"
	      "       speed <1..6> | aspm off|l0s|l1|l0s+l1] | hotreset <bridge> [ms] |\n"
	      "       ports <bridge> | watch <bdf> [ms] | aer <bdf> [clear] |\n"
	      "       test <bdf> [-b bar] [-t ms] [-d] [-k] [-l limit]\n"
	      "            bars|bar<n>|msi[<n>]|msix[<n>]|doorbell|\n"
	      "            read:<size>[u]|write:<size>[u]|copy:<size>[u]|all |\n"
	      "       buf [-l limit] [ms] | remove <bdf> | rescan | reset <bdf> |\n"
	      "       unbind <bdf> | bind <bdf> <driver> | resize <bdf> [<bar> <size>] |\n"
	      "       sriov <pf> [<numvfs> [vfio]]\n",
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

#define EXT_CAP_REBAR 0x15

/* size with an optional K/M/G suffix */
static uint64_t num_size(const char *s)
{
	char *end;
	uint64_t v;

	errno = 0;
	v = strtoull(s, &end, 0);
	if (errno != 0 || end == s) {
		die("%s: not a size", s);
	}
	switch (*end) {
	case 'K':
	case 'k':
		v <<= 10;
		end++;
		break;
	case 'M':
	case 'm':
		v <<= 20;
		end++;
		break;
	case 'G':
	case 'g':
		v <<= 30;
		end++;
		break;
	default:
		break;
	}
	if (*end != '\0') {
		die("%s: not a size", s);
	}
	return v;
}

/*
 * Resizable BAR (PCIe ext. capability 0x15): list the entries, or resize
 * one through the kernel (resourceN_resize), which also moves the bridge
 * windows above. Only a 64-bit prefetchable BAR usually finds room: the
 * kernel grows prefetchable windows at run time, non-prefetchable ones
 * rarely.
 */
/* bind bdf to drv through driver_override, whatever it was bound to */
static void bind_override(const char *bdf, const char *drv)
{
	char path[PATH_MAX], link[PATH_MAX];

	snprintf(path, sizeof(path), "%s/%s/driver_override", SYSFS, bdf);
	sysfs_write(path, drv);
	snprintf(path, sizeof(path), "%s/%s/driver", SYSFS, bdf);
	if (readlink(path, link, sizeof(link) - 1) > 0) {
		snprintf(path, sizeof(path), "%s/%s/driver/unbind", SYSFS, bdf);
		sysfs_write(path, bdf);
	}
	sysfs_write("/sys/bus/pci/drivers_probe", bdf);
}

static void vf_token_set(const char *bdf);
static const char *vf_token(void);

/*
 * sriov <pf>: TotalVFs, NumVFs and the VFs with their drivers.
 * sriov <pf> <n>: n VFs ready for pcictl test. A PF without a driver, or
 * with vfio-pci, goes to pci-pf-stub (it enables SR-IOV without touching
 * the function); the PF gets Memory Space (the VF BARs decode only with
 * it) and Bus Master (an endpoint DMA engine shared by the functions may
 * need it, RK3399's does), and the VFs go to vfio-pci.
 * sriov <pf> <n> vfio: the PF on vfio-pci as well (vfio-pci enable_sriov),
 * so that the PF and its VFs can be tested at the same time. A VF of a
 * vfio-pci PF opens only with the PF's VF token: pcictl sets its own
 * (PCICTL_VF_TOKEN overrides it) and passes it when it opens a VF. The
 * PF's reset method is cleared, as a bus reset at its open would reset
 * the VFs under their users too.
 */
static void cmd_sriov(struct pci_dev *d, int n, char **a)
{
	char bdf[16], path[PATH_MAX], link[128], drv[PATH_MAX], val[16];
	unsigned int total = 0, cur = 0;
	FILE *f;

	snprintf(bdf, sizeof(bdf), "%04x:%02x:%02x.%u", d->domain, d->bus, d->dev, d->func);
	dev_path(d, "sriov_totalvfs", path, sizeof(path));
	f = fopen(path, "r");
	if (f == NULL || fscanf(f, "%u", &total) != 1) {
		die("%s: no SR-IOV (%s)", bdf, path);
	}
	fclose(f);
	dev_path(d, "sriov_numvfs", path, sizeof(path));
	f = fopen(path, "r");
	if (f != NULL) {
		if (fscanf(f, "%u", &cur) != 1) {
			cur = 0;
		}
		fclose(f);
	}

	if (n >= 1) {
		unsigned int want = (unsigned int)num(a[0]);
		const char *pfdrv = driver(d, drv, sizeof(drv));
		bool vfio = n >= 2 && strcmp(a[1], "vfio") == 0;

		if (want > total) {
			die("%s: %u VFs at most", bdf, total);
		}
		if (cur != 0U) {
			sysfs_write(path, "0"); /* the kernel takes a new count only from 0 */
		}
		if (want != 0U) {
			if (vfio) {
				char en[4] = "";

				f = fopen("/sys/module/vfio_pci/parameters/enable_sriov", "r");
				if (f == NULL || fgets(en, sizeof(en), f) == NULL || en[0] != 'Y') {
					if (f != NULL) {
						fclose(f);
					}
					die("vfio-pci without SR-IOV: echo Y > "
					    "/sys/module/vfio_pci/parameters/enable_sriov "
					    "(or modprobe vfio-pci enable_sriov=1)");
				}
				fclose(f);
				if (strcmp(pfdrv, "vfio-pci") != 0) {
					bind_override(bdf, "vfio-pci");
				}
				dev_path(d, "reset_method", path, sizeof(path));
				if (access(path, W_OK) == 0) {
					sysfs_write(path, "\n");
				}
				vf_token_set(bdf);
				dev_path(d, "sriov_numvfs", path, sizeof(path));
			} else if (strcmp(pfdrv, "-") == 0 || strcmp(pfdrv, "vfio-pci") == 0) {
				if (access("/sys/bus/pci/drivers/pci-pf-stub", F_OK) != 0) {
					die("pci-pf-stub not loaded: modprobe pci-pf-stub");
				}
				bind_override(bdf, "pci-pf-stub");
			}
			pci_write_word(d, PCI_COMMAND,
				       pci_read_word(d, PCI_COMMAND) | PCI_COMMAND_MEMORY |
					       PCI_COMMAND_MASTER);
			snprintf(val, sizeof(val), "%u", want);
			sysfs_write(path, val);
			for (unsigned int i = 0; i < want; i++) {
				ssize_t k;
				const char *b;

				snprintf(path, sizeof(path), "%s/%s/virtfn%u", SYSFS, bdf, i);
				k = readlink(path, link, sizeof(link) - 1);
				if (k <= 0) {
					continue;
				}
				link[k] = '\0';
				b = strrchr(link, '/');
				bind_override(b != NULL ? b + 1 : link, "vfio-pci");
			}
		}
		cur = want;
	}

	printf("%s: %u of %u VFs, PF driver %s, command %04x\n", bdf, cur, total,
	       driver(d, drv, sizeof(drv)), pci_read_word(d, PCI_COMMAND));
	if (strcmp(driver(d, drv, sizeof(drv)), "vfio-pci") == 0 && cur != 0U) {
		printf("  VF token %s\n", vf_token());
	}
	for (unsigned int i = 0; i < cur; i++) {
		char vfdrv[PATH_MAX];
		ssize_t k;
		const char *b;

		snprintf(path, sizeof(path), "%s/%s/virtfn%u", SYSFS, bdf, i);
		k = readlink(path, link, sizeof(link) - 1);
		if (k <= 0) {
			continue;
		}
		link[k] = '\0';
		b = strrchr(link, '/');
		snprintf(path, sizeof(path), "%s/%s/driver", SYSFS, b != NULL ? b + 1 : link);
		k = readlink(path, vfdrv, sizeof(vfdrv) - 1);
		vfdrv[k > 0 ? k : 0] = '\0';
		printf("  VF%u %s %s\n", i + 1, b != NULL ? b + 1 : link,
		       k > 0 ? strrchr(vfdrv, '/') + 1 : "-");
	}
}

static void cmd_resize(struct pci_dev *d, int n, char **a)
{
	char path[PATH_MAX], drv[256], bdf[16], val[16];
	struct pci_cap *cap;
	const char *bound;
	bool had;

	pci_fill_info(d, PCI_FILL_EXT_CAPS | PCI_FILL_BASES | PCI_FILL_SIZES);
	cap = pci_find_cap(d, EXT_CAP_REBAR, PCI_CAP_EXTENDED);
	if (cap == NULL) {
		die("%02x:%02x.%u: no Resizable BAR capability", d->bus, d->dev, d->func);
	}
	if (n == 0) {
		uint32_t ctrl0 = pci_read_long(d, cap->addr + 8);
		unsigned int nbar = (ctrl0 >> 5) & 7U;

		for (unsigned int i = 0; i < nbar; i++) {
			uint32_t c = pci_read_long(d, cap->addr + 4 + 8 * i);
			uint32_t ctl = pci_read_long(d, cap->addr + 8 + 8 * i);
			unsigned int bar = ctl & 7U;

			printf("BAR%u: %" PRIu64 "M of", bar, (uint64_t)1 << ((ctl >> 8) & 0x1fU));
			for (unsigned int b = 4; b < 24; b++) {
				if (c & (1U << b)) {
					printf(" %" PRIu64 "M", (uint64_t)1 << (b - 4));
				}
			}
			printf(", window %" PRIx64 "\n", (uint64_t)(d->base_addr[bar] & ~0xfULL));
		}
		return;
	}
	if (n != 2) {
		usage();
	}
	unsigned int bar = (unsigned int)num(a[0]);
	uint64_t size = num_size(a[1]);
	unsigned int log = 0;

	if (bar > 5U || size < (1ULL << 20) || (size & (size - 1)) != 0) {
		die("BAR 0..5 and a power of two from 1M");
	}
	while ((1ULL << (log + 20)) < size) {
		log++;
	}
	snprintf(bdf, sizeof(bdf), "%04x:%02x:%02x.%u", d->domain, d->bus, d->dev, d->func);
	bound = driver(d, drv, sizeof(drv));
	had = strcmp(bound, "-") != 0;
	if (had) {
		memmove(drv, bound, strlen(bound) + 1);
		dev_path(d, "driver/unbind", path, sizeof(path));
		sysfs_write(path, bdf);
	}
	snprintf(path, sizeof(path), "%s/%s/resource%u_resize", SYSFS, bdf, bar);
	snprintf(val, sizeof(val), "%u", log);
	{
		int fd = open(path, O_WRONLY);
		int err = 0;

		if (fd < 0 || write(fd, val, strlen(val)) != (ssize_t)strlen(val)) {
			err = errno;
		}
		if (fd >= 0) {
			close(fd);
		}
		if (had) {
			snprintf(path, sizeof(path), "/sys/bus/pci/drivers/%s/bind", drv);
			sysfs_write(path, bdf);
		}
		if (err != 0) {
			die("%s: %s (dmesg: window too small?)", bdf, strerror(err));
		}
	}
	printf("%s BAR%u: %" PRIu64 "M%s%s\n", bdf, bar, size >> 20, had ? ", rebound to " : "",
	       had ? drv : "");
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

/* --- test: pci-epf-test over vfio-pci (No-IOMMU) --------------------------- */

/*
 * The function is bound to vfio-pci; with no IOMMU the group is
 * /dev/vfio/noiommu-N and DMA addresses are physical: buffers are hugepages
 * (physically contiguous), their address from /proc/self/pagemap. MSI and
 * MSI-X arrive on eventfds, so every wait has a timeout, and a BAR that
 * does not answer is a SIGBUS in this process.
 */

#define EPT_MAGIC	0x00
#define EPT_COMMAND	0x04
#define EPT_STATUS	0x08
#define EPT_SRC_LO	0x0c
#define EPT_SRC_HI	0x10
#define EPT_DST_LO	0x14
#define EPT_DST_HI	0x18
#define EPT_SIZE	0x1c
#define EPT_CHECKSUM	0x20
#define EPT_IRQ_TYPE	0x24
#define EPT_IRQ_NUMBER	0x28
#define EPT_FLAGS	0x2c
#define EPT_CAPS	0x30
#define EPT_DB_BAR	0x34
#define EPT_DB_OFFSET	0x38
#define EPT_DB_DATA	0x3c

#define EPT_CMD_RAISE_MSI  0x02
#define EPT_CMD_RAISE_MSIX 0x04
#define EPT_CMD_READ	   0x08
#define EPT_CMD_WRITE	   0x10
#define EPT_CMD_COPY	   0x20
#define EPT_CMD_DB_ON	   0x40
#define EPT_CMD_DB_OFF	   0x80
#define EPT_ST_READ_OK	   0x01
#define EPT_ST_WRITE_OK	   0x04
#define EPT_ST_COPY_OK	   0x10
#define EPT_ST_IRQ	   0x40
#define EPT_ST_DB	   0x200
#define EPT_ST_DB_ON	   0x400
#define EPT_ST_DB_OFF	   0x1000
#define EPT_IRQ_MSI	   1
#define EPT_IRQ_MSIX	   2
#define EPT_FLAG_DMA	   0x1
#define EPT_CAP_UNALIGNED  0x1
#define EPT_CAP_DOORBELL   0x20	/* CAP_DYNAMIC_INBOUND_MAPPING */

#define HUGE_SIZE	(2UL << 20)

struct vt {
	struct pci_dev *d;
	char bdf[16];
	int container, group, dev;
	struct bar_map bar[6];
	uint64_t cfg_off;
	unsigned int reg_bar;
	int irq_type;			/* EPT_IRQ_MSI / EPT_IRQ_MSIX, 0 = none */
	int nirq;
	int efd[64];
	unsigned int timeout_ms;
	bool dma;
	uint64_t limit;			/* buffers must end below this bus address */
	uint32_t caps;
	bool kick;			/* ring the doorbell after each command */
	int db_bar;			/* doorbell BAR while enabled, -1 off */
	uint32_t db_off, db_data;
};

static struct vt *vt_cur;	/* for the exit handler: bus mastering off */

static uint32_t crc32_le(uint32_t crc, const uint8_t *p, size_t n)
{
	/* the kernel's crc32_le(~0, ...): reflected 0xedb88320, no final xor */
	while (n--) {
		crc ^= *p++;
		for (int k = 0; k < 8; k++) {
			crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
		}
	}
	return crc;
}

static void cache_flush(const void *p, size_t n)
{
#if defined(__aarch64__)
	/* DC CIVAC at EL0 (Linux sets SCTLR_EL1.UCI): buffers may not be coherent */
	uint64_t ctr;
	uintptr_t line, a, end = (uintptr_t)p + n;

	__asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
	line = 4U << ((ctr >> 16) & 0xfU);
	for (a = (uintptr_t)p & ~(line - 1U); a < end; a += line) {
		__asm__ volatile("dc civac, %0" : : "r"(a) : "memory");
	}
	__asm__ volatile("dsb sy" : : : "memory");
#else
	(void)p;
	(void)n;
	__sync_synchronize();
#endif
}

static uint64_t virt_to_phys(const void *v)
{
	uint64_t ent;
	long ps = sysconf(_SC_PAGESIZE);
	int fd = open("/proc/self/pagemap", O_RDONLY);

	if (fd < 0) {
		die("/proc/self/pagemap: %s", strerror(errno));
	}
	if (pread(fd, &ent, sizeof(ent), ((uintptr_t)v / (uint64_t)ps) * sizeof(ent)) !=
	    (ssize_t)sizeof(ent)) {
		die("/proc/self/pagemap: %s", strerror(errno));
	}
	close(fd);
	if (!(ent & (1ULL << 63)) || (ent & ((1ULL << 55) - 1U)) == 0U) {
		die("no physical address for %p (not root?)", v);
	}
	return (ent & ((1ULL << 55) - 1U)) * (uint64_t)ps + ((uintptr_t)v % (uint64_t)ps);
}

struct dbuf {
	uint8_t *va;
	uint64_t pa;
	int fd;		/* dma-buf of a heap, -1 for a hugepage */
};

static void dbuf_free(struct dbuf *b)
{
	if (b->va != NULL) {
		munmap(b->va, HUGE_SIZE);
	}
	if (b->fd >= 0) {
		close(b->fd);
	}
	b->va = NULL;
	b->fd = -1;
}

/* a 2 MiB hugepage below vt->limit (tries a few, gives the others back) */
static struct dbuf hugebuf(const struct vt *t)
{
	static void *tried[4096];
	int n = 0;
	struct dbuf b = {.fd = -1};

	while (n < (int)ARRAY_SIZE(tried)) {
		void *p = mmap(NULL, HUGE_SIZE, PROT_READ | PROT_WRITE,
			       MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB | MAP_POPULATE, -1, 0);

		if (p == MAP_FAILED) {
			break;	/* none left, or only pages above the limit */
		}
		memset(p, 0, HUGE_SIZE);
		if (mlock(p, HUGE_SIZE) != 0) {
			die("mlock: %s", strerror(errno));
		}
		b.va = p;
		b.pa = virt_to_phys(p);
		if (b.pa + HUGE_SIZE <= t->limit) {
			break;
		}
		tried[n++] = p;
		b.va = NULL;
	}
	for (int i = 0; i < n; i++) {
		munmap(tried[i], HUGE_SIZE);
	}
	/*
	 * The pool fills from the top of memory: on a 4 GiB board the first
	 * ~1000 pages lie above 4 GiB. A dma-heap buffer is no way out, its
	 * mapping is VM_PFNMAP, which pagemap does not show.
	 */
	if (b.va == NULL) {
		die("none of %d free hugepages below 0x%" PRIx64 " (DMA window of the RC): "
		    "allocate enough to reach low memory, e.g. echo 1200 > "
		    "/sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages", n, t->limit);
	}
	return b;
}

static uint32_t cfg_rd(struct vt *t, unsigned int off, unsigned int n)
{
	uint32_t v = 0;

	if (pread(t->dev, &v, n, (off_t)(t->cfg_off + off)) != (ssize_t)n) {
		die("vfio config read 0x%x: %s", off, strerror(errno));
	}
	return v;
}

static void cfg_wr(struct vt *t, unsigned int off, unsigned int n, uint32_t v)
{
	if (pwrite(t->dev, &v, n, (off_t)(t->cfg_off + off)) != (ssize_t)n) {
		die("vfio config write 0x%x: %s", off, strerror(errno));
	}
}

static void vt_master(struct vt *t, bool on)
{
	uint16_t cmd = (uint16_t)cfg_rd(t, PCI_COMMAND, 2);

	cmd = on ? (cmd | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER) : (cmd & ~PCI_COMMAND_MASTER);
	cfg_wr(t, PCI_COMMAND, 2, cmd);
}

static void vt_exit(void)
{
	if (vt_cur != NULL && vt_cur->dev >= 0) {
		uint16_t cmd = 0;

		/* no late DMA into pages this process gives back */
		if (pread(vt_cur->dev, &cmd, 2, (off_t)(vt_cur->cfg_off + PCI_COMMAND)) == 2) {
			cmd &= ~PCI_COMMAND_MASTER;
			off_t off = (off_t)(vt_cur->cfg_off + PCI_COMMAND);

			if (pwrite(vt_cur->dev, &cmd, 2, off) != 2) {
				fputs("bus mastering not turned off\n", stderr);
			}
		}
	}
}

static uint32_t reg_rd(struct vt *t, unsigned int off)
{
	unsigned int w = width;
	uint32_t v;

	width = 32;
	v = (uint32_t)bar_rd(&t->bar[t->reg_bar], t->reg_bar, off);
	width = w;
	return v;
}

static void reg_wr(struct vt *t, unsigned int off, uint32_t v)
{
	unsigned int w = width;

	width = 32;
	bar_wr(&t->bar[t->reg_bar], t->reg_bar, off, v);
	width = w;
}

/*
 * A VF reads Vendor and Device ID 0xffff, Memory Space 0 and zero BARs in
 * its own config space (SR-IOV): those live in the PF, VF Memory Space in
 * its SR-IOV Control. Look there instead.
 */
/*
 * An endpoint that rebooted under the host lost the BAR addresses the
 * host assigned: the PF's, and the VF BARs in its SR-IOV capability, which
 * the kernel writes only at enumeration. True when the kernel has a BAR of
 * the PF assigned that reads back zero.
 */
static bool pf_bars_lost(const char *pf, const uint8_t *c)
{
	char path[PATH_MAX];
	bool assigned = false, lost = false;
	FILE *f;

	if (snprintf(path, sizeof(path), "%s/%s/resource", SYSFS, pf) >= (int)sizeof(path)) {
		return false;
	}
	f = fopen(path, "r");
	if (f == NULL) {
		return false;
	}
	for (unsigned int i = 0; i < 6U; i++) {
		unsigned long long start, end, flags;
		uint32_t raw = c[0x10 + 4 * i] | c[0x11 + 4 * i] << 8 | c[0x12 + 4 * i] << 16 |
			       (uint32_t)c[0x13 + 4 * i] << 24;

		if (fscanf(f, "%llx %llx %llx", &start, &end, &flags) != 3) {
			break;
		}
		if (start == 0U || (raw & PCI_BASE_ADDRESS_SPACE_IO) != 0U) {
			continue;
		}
		assigned = true;
		/* a 64-bit BAR may sit above 4 GiB on the bus: check the upper half too */
		if ((raw & PCI_BASE_ADDRESS_MEM_MASK) != 0U) {
			fclose(f);
			return false;
		}
		if ((raw & PCI_BASE_ADDRESS_MEM_TYPE_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64 &&
		    i < 5U && (c[0x14 + 4 * i] | c[0x15 + 4 * i] | c[0x16 + 4 * i] |
			       c[0x17 + 4 * i]) != 0U) {
			fclose(f);
			return false;
		}
		lost = true;
	}
	fclose(f);
	return assigned && lost;
}

static int vf_alive(struct vt *t, const char *pf)
{
	char path[PATH_MAX];
	uint8_t c[0x28];
	int fd, n;

	if (snprintf(path, sizeof(path), "%s/%s/config", SYSFS, pf) >= (int)sizeof(path)) {
		return 0;
	}
	fd = open(path, O_RDONLY);
	if (fd < 0) {
		return 0; /* nothing to compare with: let the test find out */
	}
	n = (int)pread(fd, c, sizeof(c), 0);
	close(fd);
	if (n != (int)sizeof(c)) {
		return 0;
	}
	if ((c[0] | c[1] << 8) != t->d->vendor_id) {
		fprintf(stderr, "PF %s does not answer config requests (vendor %04x)\n", pf,
			c[0] | c[1] << 8);
		return -1;
	}
	if (!(c[4] & PCI_COMMAND_MEMORY)) {
		fprintf(stderr, "PF %s has Memory Space off: pcictl cmd %s +mem +bme\n", pf, pf);
		return -1;
	}
	if (pf_bars_lost(pf, c)) {
		fprintf(stderr,
			"PF %s lost its BARs (endpoint rebooted?): pcictl sriov %s 0; "
			"pcictl remove %s; pcictl rescan; pcictl sriov %s <n>\n",
			pf, pf, pf, pf);
		return -1;
	}
	if (t->dma && !(c[4] & PCI_COMMAND_MASTER)) {
		fprintf(stderr, "note: PF %s without Bus Master: an endpoint DMA engine shared "
			"by the functions may stay idle (pcictl cmd %s +bme)\n", pf, pf);
	}
	return 0;
}

/*
 * The real config space (sysfs, not vfio's virtualized copy): an endpoint
 * that rebooted shows Memory Space off and BARs zero; a BAR access would
 * then fault. Checked before every test.
 */
static int vt_alive(struct vt *t)
{
	char path[PATH_MAX], pf[PATH_MAX];
	uint16_t vendor, cmd;
	uint32_t bar, want;
	ssize_t n;

	snprintf(path, sizeof(path), "%s/%s/physfn", SYSFS, t->bdf);
	n = readlink(path, pf, sizeof(pf) - 1);
	if (n > 0) {
		const char *b;

		pf[n] = '\0';
		b = strrchr(pf, '/');
		return vf_alive(t, b != NULL ? b + 1 : pf);
	}

	vendor = pci_read_word(t->d, PCI_VENDOR_ID);
	cmd = pci_read_word(t->d, PCI_COMMAND);
	bar = pci_read_long(t->d, PCI_BASE_ADDRESS_0 + 4 * t->reg_bar);
	want = (uint32_t)(t->d->base_addr[t->reg_bar] & PCI_ADDR_MEM_MASK);
	if (vendor != t->d->vendor_id) {
		fprintf(stderr, "endpoint does not answer config requests (vendor %04x)\n",
			vendor);
		return -1;
	}
	if (!(cmd & PCI_COMMAND_MEMORY) || (bar & PCI_BASE_ADDRESS_MEM_MASK) != want) {
		fprintf(stderr,
			"endpoint lost its config space (command %04x, BAR%u %08x, was %08x): "
			"rebooted? pcictl remove %s; pcictl rescan\n",
			cmd, t->reg_bar, bar, want, t->bdf);
		return -1;
	}
	return 0;
}

static void vt_irq_off(struct vt *t)
{
	struct vfio_irq_set s = {
		.argsz = sizeof(s),
		.flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER,
		.index = t->irq_type == EPT_IRQ_MSIX ? VFIO_PCI_MSIX_IRQ_INDEX
						     : VFIO_PCI_MSI_IRQ_INDEX,
	};

	if (t->nirq == 0) {
		return;
	}
	ioctl(t->dev, VFIO_DEVICE_SET_IRQS, &s);
	for (int i = 0; i < t->nirq; i++) {
		close(t->efd[i]);
	}
	t->nirq = 0;
	t->irq_type = 0;
}

/* vectors the function offers (the capability's size), at most 16 */
static int vt_irq_max(struct vt *t, int type)
{
	struct vfio_irq_info ii = {
		.argsz = sizeof(ii),
		.index = type == EPT_IRQ_MSIX ? VFIO_PCI_MSIX_IRQ_INDEX : VFIO_PCI_MSI_IRQ_INDEX,
	};

	if (ioctl(t->dev, VFIO_DEVICE_GET_IRQ_INFO, &ii) != 0) {
		return 0;
	}
	return ii.count < 16U ? (int)ii.count : 16;
}

/* MSI or MSI-X with n vectors on eventfds; the kernel programs the capability */
static void vt_irq_on(struct vt *t, int type, int n)
{
	unsigned int index = type == EPT_IRQ_MSIX ? VFIO_PCI_MSIX_IRQ_INDEX
						  : VFIO_PCI_MSI_IRQ_INDEX;
	struct vfio_irq_info ii = {.argsz = sizeof(ii), .index = index};
	struct vfio_irq_set *s;

	if (t->irq_type == type && t->nirq >= n) {
		return;
	}
	vt_irq_off(t);
	if (ioctl(t->dev, VFIO_DEVICE_GET_IRQ_INFO, &ii) != 0 || ii.count == 0U) {
		die("%s not available", type == EPT_IRQ_MSIX ? "MSI-X" : "MSI");
	}
	/*
	 * MSI: all the vectors the function offers, once (as pci_endpoint_test
	 * does); re-enabling with another count fragments the MSI domain. A
	 * positive return is the number the platform can give.
	 */
	int want = (type == EPT_IRQ_MSI) ? (int)ii.count : n;

	if (want > (int)ARRAY_SIZE(t->efd)) {
		want = ARRAY_SIZE(t->efd);
	}
	for (;;) {
		int r;

		s = calloc(1, sizeof(struct vfio_irq_set) + sizeof(int32_t) * (size_t)want);
		if (s == NULL) {
			die("out of memory");
		}
		s->argsz = (uint32_t)(sizeof(struct vfio_irq_set) + sizeof(int32_t) * (size_t)want);
		s->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
		s->index = index;
		s->count = (uint32_t)want;
		for (int i = 0; i < want; i++) {
			t->efd[i] = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
			if (t->efd[i] < 0) {
				die("eventfd: %s", strerror(errno));
			}
			((int32_t *)s->data)[i] = t->efd[i];
		}
		r = ioctl(t->dev, VFIO_DEVICE_SET_IRQS, s);
		free(s);
		if (r == 0) {
			break;
		}
		for (int i = 0; i < want; i++) {
			close(t->efd[i]);
		}
		if (r < 0 || r >= want) {
			die("enable %d %s vectors: %s", want,
			    type == EPT_IRQ_MSIX ? "MSI-X" : "MSI", r < 0 ? strerror(errno) : "?");
		}
		/* multi-message MSI: a power of two */
		want = (type == EPT_IRQ_MSI) ? (1 << (31 - __builtin_clz((unsigned int)r))) : r;
	}
	if (want < n) {
		die("%s%d: only %d vectors", type == EPT_IRQ_MSIX ? "MSI-X" : "MSI", n, want);
	}
	t->nirq = want;
	t->irq_type = type;
}

/* drain stale events, so a late interrupt is not taken for the next one */
static void vt_irq_drain(struct vt *t)
{
	uint64_t c;

	for (int i = 0; i < t->nirq; i++) {
		while (read(t->efd[i], &c, sizeof(c)) == (ssize_t)sizeof(c)) {
		}
	}
}

/* wait for vector v (0-based); returns microseconds, or -1 on timeout */
static long vt_wait(struct vt *t, int v, const struct timespec *t0)
{
	struct pollfd p = {.fd = t->efd[v], .events = POLLIN};
	struct timespec t1;
	uint64_t c;
	int r = poll(&p, 1, (int)t->timeout_ms);

	if (r <= 0 || read(t->efd[v], &c, sizeof(c)) != (ssize_t)sizeof(c)) {
		return -1;
	}
	clock_gettime(CLOCK_MONOTONIC, &t1);
	return (t1.tv_sec - t0->tv_sec) * 1000000L + (t1.tv_nsec - t0->tv_nsec) / 1000L;
}

static void vt_ring(struct vt *t)
{
	unsigned int w = width;

	width = 32;
	bar_wr(&t->bar[t->db_bar], (unsigned int)t->db_bar, t->db_off, t->db_data);
	width = w;
}

static int vt_command(struct vt *t, uint32_t cmd, int vec, long *us)
{
	struct timespec t0;

	vt_irq_drain(t);
	reg_wr(t, EPT_STATUS, 0);
	reg_wr(t, EPT_IRQ_TYPE, (uint32_t)t->irq_type);
	reg_wr(t, EPT_IRQ_NUMBER, (uint32_t)vec);
	clock_gettime(CLOCK_MONOTONIC, &t0);
	reg_wr(t, EPT_COMMAND, cmd);
	if (t->kick && t->db_bar >= 0) {
		/* the endpoint runs the command now instead of at its next poll */
		vt_ring(t);
	}
	*us = vt_wait(t, vec - 1, &t0);
	if (*us < 0) {
		/* the endpoint may still be moving data: stop it before going on */
		vt_master(t, false);
		return -1;
	}
	return 0;
}

/* pcictl's VF token, "pcictl" in the first bytes; PCICTL_VF_TOKEN overrides it */
static const char *vf_token(void)
{
	const char *t = getenv("PCICTL_VF_TOKEN");

	return t != NULL ? t : "70636963-746c-4000-8000-000000000000";
}

static int uuid_parse(const char *s, uint8_t u[16])
{
	unsigned int i = 0;

	while (*s != '\0' && i < 16U) {
		unsigned int b;

		if (*s == '-') {
			s++;
			continue;
		}
		if (sscanf(s, "%2x", &b) != 1) {
			return -1;
		}
		u[i++] = (uint8_t)b;
		s += 2;
	}
	return i == 16U && *s == '\0' ? 0 : -1;
}

/* the driver of the PF a VF belongs to, "" for a PF */
static const char *pf_driver(const char *bdf, char *buf, size_t len)
{
	char path[PATH_MAX], link[256];
	ssize_t n;
	const char *p;

	snprintf(path, sizeof(path), "%s/%s/physfn/driver", SYSFS, bdf);
	n = readlink(path, link, sizeof(link) - 1);
	if (n <= 0) {
		snprintf(path, sizeof(path), "%s/%s/physfn", SYSFS, bdf);
		snprintf(buf, len, "%s", access(path, F_OK) == 0 ? "-" : "");
		return buf;
	}
	link[n] = '\0';
	p = strrchr(link, '/');
	snprintf(buf, len, "%s", p != NULL ? p + 1 : link);
	return buf;
}

/* VFIO_DEVICE_FEATURE_PCI_VF_TOKEN on an open vfio-pci PF; 0 if it has none */
static int vf_token_feature(int dev)
{
	struct {
		struct vfio_device_feature f;
		uint8_t uuid[16];
	} req = {.f = {.argsz = sizeof(req),
		       .flags = VFIO_DEVICE_FEATURE_SET | VFIO_DEVICE_FEATURE_PCI_VF_TOKEN}};

	if (uuid_parse(vf_token(), req.uuid) != 0) {
		die("PCICTL_VF_TOKEN %s: not a UUID", vf_token());
	}
	if (ioctl(dev, VFIO_DEVICE_FEATURE, &req) != 0) {
		return errno == ENOTTY ? 0 : -errno; /* no SR-IOV, or enable_sriov off */
	}
	return 0;
}

/* open the vfio-pci PF just to set the VF token (it stays after close) */
static void vf_token_set(const char *bdf)
{
	char path[PATH_MAX], link[256], *g;
	int container, group, dev, ret;
	ssize_t n;

	snprintf(path, sizeof(path), "%s/%s/iommu_group", SYSFS, bdf);
	n = readlink(path, link, sizeof(link) - 1);
	if (n < 0) {
		die("%s: no IOMMU group after binding vfio-pci", bdf);
	}
	link[n] = '\0';
	g = strrchr(link, '/');
	container = open("/dev/vfio/vfio", O_RDWR | O_CLOEXEC);
	snprintf(path, sizeof(path), "/dev/vfio/noiommu-%s", g != NULL ? g + 1 : link);
	group = open(path, O_RDWR | O_CLOEXEC);
	if (container < 0 || group < 0 || ioctl(group, VFIO_GROUP_SET_CONTAINER, &container) != 0 ||
	    ioctl(container, VFIO_SET_IOMMU, VFIO_NOIOMMU_IOMMU) != 0) {
		die("%s: vfio group %s: %s", bdf, path, strerror(errno));
	}
	dev = ioctl(group, VFIO_GROUP_GET_DEVICE_FD, bdf);
	if (dev < 0) {
		die("%s: device fd: %s", bdf, strerror(errno));
	}
	ret = vf_token_feature(dev);
	if (ret != 0) {
		die("%s: VF token: %s", bdf, strerror(-ret));
	}
	close(dev);
	close(group);
	close(container);
}

static void vt_open(struct vt *t, struct pci_dev *d)
{
	char path[PATH_MAX], link[256], *g;
	struct vfio_group_status gs = {.argsz = sizeof(gs)};
	struct vfio_device_info di = {.argsz = sizeof(di)};
	struct vfio_region_info ri = {.argsz = sizeof(ri)};
	ssize_t n;

	t->d = d;
	snprintf(t->bdf, sizeof(t->bdf), "%04x:%02x:%02x.%u", d->domain, d->bus, d->dev, d->func);
	dev_path(d, "iommu_group", path, sizeof(path));
	n = readlink(path, link, sizeof(link) - 1);
	if (n < 0) {
		die("%s: no IOMMU group: not bound to vfio-pci "
		    "(driverctl set-override %s vfio-pci)", t->bdf, t->bdf);
	}
	link[n] = '\0';
	g = strrchr(link, '/');
	t->container = open("/dev/vfio/vfio", O_RDWR | O_CLOEXEC);
	if (t->container < 0) {
		die("/dev/vfio/vfio: %s (modprobe vfio-pci)", strerror(errno));
	}
	if (ioctl(t->container, VFIO_GET_API_VERSION) != VFIO_API_VERSION ||
	    ioctl(t->container, VFIO_CHECK_EXTENSION, VFIO_NOIOMMU_IOMMU) <= 0) {
		die("no No-IOMMU support (modprobe vfio enable_unsafe_noiommu_mode=1)");
	}
	snprintf(path, sizeof(path), "/dev/vfio/noiommu-%s", g != NULL ? g + 1 : link);
	t->group = open(path, O_RDWR | O_CLOEXEC);
	if (t->group < 0) {
		die("%s: %s (bound to vfio-pci? an IOMMU group would be /dev/vfio/N, not handled)",
		    path, strerror(errno));
	}
	if (ioctl(t->group, VFIO_GROUP_GET_STATUS, &gs) != 0 ||
	    !(gs.flags & VFIO_GROUP_FLAGS_VIABLE)) {
		die("%s: group not viable (other functions in it not bound to vfio-pci)", path);
	}
	if (ioctl(t->group, VFIO_GROUP_SET_CONTAINER, &t->container) != 0 ||
	    ioctl(t->container, VFIO_SET_IOMMU, VFIO_NOIOMMU_IOMMU) != 0) {
		die("%s: container setup: %s", path, strerror(errno));
	}
	/* opening the device resets it: an endpoint that cannot do that hangs */
	dev_path(d, "reset_method", path, sizeof(path));
	{
		FILE *f = fopen(path, "r");
		char m[128] = "";

		if (f != NULL && fgets(m, sizeof(m), f) != NULL && m[0] != '\n') {
			fprintf(stderr, "note: vfio-pci resets the function on open (%.*s); if the "
				"endpoint cannot, first: echo > %s\n",
				(int)strcspn(m, "\n"), m, path);
		}
		if (f != NULL) {
			fclose(f);
		}
	}
	{
		/* a VF of a vfio-pci PF opens only with the PF's VF token */
		char name[128], pfdrv[256];

		if (strcmp(pf_driver(t->bdf, pfdrv, sizeof(pfdrv)), "vfio-pci") == 0) {
			snprintf(name, sizeof(name), "%s vf_token=%s", t->bdf, vf_token());
		} else {
			snprintf(name, sizeof(name), "%s", t->bdf);
		}
		t->dev = ioctl(t->group, VFIO_GROUP_GET_DEVICE_FD, name);
		if (t->dev < 0) {
			die("%s: device fd: %s%s", t->bdf, strerror(errno),
			    strcmp(pfdrv, "vfio-pci") == 0 ? " (VF token: PCICTL_VF_TOKEN?)" : "");
		}
		/* a PF with SR-IOV on vfio-pci: keep the VF token pcictl's own */
		if (pfdrv[0] == '\0' && vf_token_feature(t->dev) != 0) {
			fprintf(stderr, "note: %s: VF token not set\n", t->bdf);
		}
	}
	if (ioctl(t->dev, VFIO_DEVICE_GET_INFO, &di) != 0) {
		die("%s: device info: %s", t->bdf, strerror(errno));
	}
	ri.index = VFIO_PCI_CONFIG_REGION_INDEX;
	if (ioctl(t->dev, VFIO_DEVICE_GET_REGION_INFO, &ri) != 0) {
		die("%s: config region: %s", t->bdf, strerror(errno));
	}
	t->cfg_off = ri.offset;
	for (unsigned int i = 0; i < 6U; i++) {
		void *va;

		ri = (struct vfio_region_info){.argsz = sizeof(ri), .index = i};
		if (ioctl(t->dev, VFIO_DEVICE_GET_REGION_INFO, &ri) != 0 || ri.size == 0U ||
		    !(ri.flags & VFIO_REGION_INFO_FLAG_MMAP)) {
			continue;
		}
		va = mmap(NULL, ri.size, PROT_READ | PROT_WRITE, MAP_SHARED, t->dev,
			  (off_t)ri.offset);
		if (va == MAP_FAILED) {
			die("BAR%u mmap: %s", i, strerror(errno));
		}
		t->bar[i].va = va;
		t->bar[i].size = ri.size;
	}
	if (t->bar[t->reg_bar].va == NULL) {
		die("test register BAR%u not mappable", t->reg_bar);
	}
	vt_cur = t;
	atexit(vt_exit);
	if (vt_alive(t) != 0) {
		exit(4);
	}
	vt_master(t, true);
	t->caps = reg_rd(t, EPT_CAPS);
}

static void report(const char *what, int ok, long us, size_t bytes)
{
	if (ok && bytes != 0U && us > 0) {
		printf("%-28s OKAY  %7ld us %8.1f MB/s\n", what, us, (double)bytes / (double)us);
	} else if (ok) {
		printf("%-28s OKAY  %7ld us\n", what, us);
	} else {
		printf("%-28s NOT OKAY%s\n", what, us < 0 ? " (timeout, bus mastering off)" : "");
	}
}

/* COMMAND_ENABLE_DOORBELL: the endpoint retargets a BAR at its doorbell */
static int vt_db_on(struct vt *t)
{
	bool kick = t->kick;
	long us;
	uint32_t bar;

	if (!(t->caps & EPT_CAP_DOORBELL)) {
		printf("doorbell: not offered by the endpoint (caps %08x)\n", t->caps);
		return -1;
	}
	if (t->irq_type == 0) {
		vt_irq_on(t, EPT_IRQ_MSI, 1);
	}
	t->kick = false;
	if (vt_command(t, EPT_CMD_DB_ON, 1, &us) != 0 || !(reg_rd(t, EPT_STATUS) & EPT_ST_DB_ON)) {
		t->kick = kick;
		printf("doorbell: enable failed, status %08x\n", reg_rd(t, EPT_STATUS));
		return -1;
	}
	t->kick = kick;
	bar = reg_rd(t, EPT_DB_BAR);
	if (bar > 5U || t->bar[bar].va == NULL) {
		printf("doorbell: BAR %u not mapped\n", bar);
		return -1;
	}
	t->db_bar = (int)bar;
	t->db_off = reg_rd(t, EPT_DB_OFFSET);
	t->db_data = reg_rd(t, EPT_DB_DATA);
	printf("doorbell: BAR%u+0x%x, data 0x%x\n", bar, t->db_off, t->db_data);
	return 0;
}

static void vt_db_off(struct vt *t)
{
	long us;

	if (t->db_bar < 0) {
		return;
	}
	t->kick = false;
	t->db_bar = -1;
	if (vt_command(t, EPT_CMD_DB_OFF, 1, &us) != 0 ||
	    !(reg_rd(t, EPT_STATUS) & EPT_ST_DB_OFF)) {
		printf("doorbell: disable failed, status %08x\n", reg_rd(t, EPT_STATUS));
	}
}

/* as pci_endpoint_test: ring with no command, the endpoint answers with an IRQ */
static int vt_test_doorbell(struct vt *t, int n)
{
	bool was_on = t->db_bar >= 0;
	long us, min = -1, max = 0, sum = 0;
	int fails = 0;

	if (!was_on && vt_db_on(t) != 0) {
		report("doorbell", 0, 0, 0);
		return 1;
	}
	for (int i = 0; i < n; i++) {
		struct timespec t0;

		vt_irq_drain(t);
		reg_wr(t, EPT_STATUS, 0);
		reg_wr(t, EPT_IRQ_TYPE, (uint32_t)t->irq_type);
		reg_wr(t, EPT_IRQ_NUMBER, 1);
		clock_gettime(CLOCK_MONOTONIC, &t0);
		vt_ring(t);
		us = vt_wait(t, 0, &t0);
		if (us < 0 || !(reg_rd(t, EPT_STATUS) & EPT_ST_DB)) {
			fails++;
			continue;
		}
		min = (min < 0 || us < min) ? us : min;
		max = us > max ? us : max;
		sum += us;
	}
	if (fails == 0) {
		printf("%-28s OKAY  %7ld us (min %ld, max %ld, %d rings)\n", "doorbell",
		       sum / n, min, max, n);
	} else {
		printf("%-28s NOT OKAY (%d of %d rings unanswered, status %08x)\n", "doorbell",
		       fails, n, reg_rd(t, EPT_STATUS));
	}
	if (!was_on) {
		vt_db_off(t);
	}
	return fails != 0;
}

static int vt_test_bar(struct vt *t, unsigned int b)
{
	struct bar_map *m = &t->bar[b];
	unsigned int w = width;
	uint64_t n, i;
	uint32_t pat = 0xa0a0a0a0U + 0x01010101U * b;
	char what[48];
	int ok = 1;

	if (m->va == NULL) {
		return 0;
	}
	if ((int)b == t->db_bar) {
		printf("BAR%u                         skipped (doorbell)\n", b);
		return 0;
	}
	/*
	 * The register block: only the first word, as pci_endpoint_test does.
	 * Others: every word of the first MiB, beyond it one word per 4 KiB
	 * (reads cost a round trip each); the pattern holds the offset, so a
	 * window that wraps (inbound translation not aligned to the BAR size)
	 * reads back the wrong words.
	 */
	n = (b == t->reg_bar) ? 0 : m->size;
	width = 32;
	for (i = 0; i < n; i += (i < (1U << 20)) ? 4 : 4096) {
		bar_wr(m, b, i, pat ^ (uint32_t)i);
	}
	for (i = 0; i < n; i += (i < (1U << 20)) ? 4 : 4096) {
		uint32_t got = (uint32_t)bar_rd(m, b, i);

		if (got != (pat ^ (uint32_t)i)) {
			printf("BAR%u: offset 0x%" PRIx64 " reads 0x%08x, wrote 0x%08x\n", b, i,
			       got, pat ^ (uint32_t)i);
			ok = 0;
			break;
		}
	}
	width = w;
	snprintf(what, sizeof(what), "BAR%u (%" PRIu64 " bytes)", b, n);
	if (n != 0U) {
		report(what, ok, 0, 0);
	}
	return ok ? 0 : 1;
}

static int vt_test_irq(struct vt *t, int type, int vec)
{
	char what[32];
	long us;
	int ok;

	vt_irq_on(t, type, vec);
	ok = vt_command(t, type == EPT_IRQ_MSIX ? EPT_CMD_RAISE_MSIX : EPT_CMD_RAISE_MSI, vec,
			&us) == 0;
	snprintf(what, sizeof(what), "%s%d", type == EPT_IRQ_MSIX ? "MSI-X" : "MSI", vec);
	report(what, ok, us, 0);
	return ok ? 0 : 1;
}

/* host buffer offset: the EP's alignment unless it takes unaligned addresses */
static size_t vt_xfer_off(const struct vt *t, size_t size, bool unaligned)
{
	(void)size;
	return (unaligned && (t->caps & EPT_CAP_UNALIGNED)) ? 3U : 0U;
}

static int vt_test_xfer(struct vt *t, const char *kind, size_t size, bool unaligned)
{
	struct dbuf a = hugebuf(t), b = {.fd = -1};
	size_t off = vt_xfer_off(t, size, unaligned);
	char what[48];
	uint32_t crc = 0, st;
	long us;
	int ok;

	if (size == 0U || size + off > HUGE_SIZE) {
		die("%s: size 1..%lu", kind, HUGE_SIZE - off);
	}
	if (t->irq_type == 0) {
		vt_irq_on(t, EPT_IRQ_MSI, 1);
	}
	reg_wr(t, EPT_FLAGS, t->dma ? EPT_FLAG_DMA : 0U);
	reg_wr(t, EPT_SIZE, (uint32_t)size);
	if (strcmp(kind, "write") == 0) {
		/* host -> EP: the EP reads our buffer and checks the CRC */
		if (getrandom(a.va + off, size, 0) != (ssize_t)size) {
			die("getrandom: %s", strerror(errno));
		}
		crc = crc32_le(~0U, a.va + off, size);
		cache_flush(a.va, size + off);
		reg_wr(t, EPT_CHECKSUM, crc);
		reg_wr(t, EPT_SRC_LO, (uint32_t)(a.pa + off));
		reg_wr(t, EPT_SRC_HI, (uint32_t)((a.pa + off) >> 32));
		ok = vt_command(t, EPT_CMD_READ, 1, &us) == 0;
		ok = ok && (reg_rd(t, EPT_STATUS) & EPT_ST_READ_OK);
	} else if (strcmp(kind, "read") == 0) {
		/* EP -> host: the EP fills our buffer and gives its CRC */
		cache_flush(a.va, size + off);
		reg_wr(t, EPT_DST_LO, (uint32_t)(a.pa + off));
		reg_wr(t, EPT_DST_HI, (uint32_t)((a.pa + off) >> 32));
		ok = vt_command(t, EPT_CMD_WRITE, 1, &us) == 0;
		if (ok) {
			cache_flush(a.va, size + off);
			st = reg_rd(t, EPT_STATUS);
			ok = (st & EPT_ST_WRITE_OK) &&
			     crc32_le(~0U, a.va + off, size) == reg_rd(t, EPT_CHECKSUM);
		}
	} else {
		/* copy: the EP copies between two host buffers */
		b = hugebuf(t);
		if (getrandom(a.va + off, size, 0) != (ssize_t)size) {
			die("getrandom: %s", strerror(errno));
		}
		crc = crc32_le(~0U, a.va + off, size);
		cache_flush(a.va, size + off);
		cache_flush(b.va, size + off);
		reg_wr(t, EPT_SRC_LO, (uint32_t)(a.pa + off));
		reg_wr(t, EPT_SRC_HI, (uint32_t)((a.pa + off) >> 32));
		reg_wr(t, EPT_DST_LO, (uint32_t)(b.pa + off));
		reg_wr(t, EPT_DST_HI, (uint32_t)((b.pa + off) >> 32));
		ok = vt_command(t, EPT_CMD_COPY, 1, &us) == 0;
		if (ok) {
			cache_flush(b.va, size + off);
			ok = (reg_rd(t, EPT_STATUS) & EPT_ST_COPY_OK) &&
			     crc32_le(~0U, b.va + off, size) == crc;
		}
	}
	snprintf(what, sizeof(what), "%s %s(%zu bytes%s)", kind, t->dma ? "DMA " : "", size,
		 off ? ", unaligned" : "");
	report(what, ok, ok ? us : (us < 0 ? -1 : 0), ok ? size : 0U);
	/* bus mastering is off after a timeout: the pages can go */
	dbuf_free(&a);
	dbuf_free(&b);
	return ok ? 0 : 1;
}

/*
 * test <bdf> [-b bar] [-t ms] [-d] [-l limit] item...
 *   bars | bar<n> | msi<n> | msi | msix<n> | read:<size> | write:<size> |
 *   copy:<size> (suffix u: unaligned host buffer) | all
 */
static int cmd_test(struct pci_dev *d, int argc, char **argv)
{
	struct vt t = {.container = -1, .group = -1, .dev = -1, .timeout_ms = 2000,
		       .limit = 1ULL << 32, .db_bar = -1};
	int fails = 0, i = 0;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
			t.reg_bar = (unsigned int)num(argv[++i]);
		} else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
			t.timeout_ms = (unsigned int)num(argv[++i]);
		} else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
			t.limit = num(argv[++i]);
		} else if (strcmp(argv[i], "-d") == 0) {
			t.dma = true;
		} else if (strcmp(argv[i], "-k") == 0) {
			t.kick = true;
		} else {
			usage();
		}
	}
	if (i == argc || t.reg_bar > 5U) {
		usage();
	}
	vt_open(&t, d);
	printf("%s %04x:%04x, test registers in BAR%u, caps %08x, timeout %u ms\n", t.bdf,
	       d->vendor_id, d->device_id, t.reg_bar, t.caps, t.timeout_ms);
	if (t.kick && vt_db_on(&t) != 0) {
		die("-k: the endpoint has no doorbell");
	}

	for (; i < argc; i++) {
		const char *a = argv[i];
		char *colon = strchr(a, ':');
		size_t size = colon != NULL ? (size_t)strtoull(colon + 1, NULL, 0) : 0;
		bool unaligned = colon != NULL && colon[strlen(colon) - 1] == 'u';

		if (vt_alive(&t) != 0) {
			fails++;
			break;
		}
		vt_master(&t, true);	/* back on after a timeout */
		if (strcmp(a, "all") == 0) {
			for (unsigned int b = 0; b < 6U; b++) {
				fails += vt_test_bar(&t, b);
			}
			for (int v = 1; v <= vt_irq_max(&t, EPT_IRQ_MSI) && fails == 0; v++) {
				fails += vt_test_irq(&t, EPT_IRQ_MSI, v);
			}
			vt_irq_on(&t, EPT_IRQ_MSI, 1);
			for (size_t s = 1; s <= (1U << 20); s *= 32) {
				fails += vt_test_xfer(&t, "write", s, false);
				fails += vt_test_xfer(&t, "read", s, false);
				fails += vt_test_xfer(&t, "copy", s, false);
			}
		} else if (strcmp(a, "doorbell") == 0) {
			fails += vt_test_doorbell(&t, 16);
		} else if (strcmp(a, "bars") == 0) {
			for (unsigned int b = 0; b < 6U; b++) {
				fails += vt_test_bar(&t, b);
			}
		} else if (strncmp(a, "bar", 3) == 0) {
			fails += vt_test_bar(&t, (unsigned int)num(a + 3));
		} else if (strcmp(a, "msix") == 0) {
			for (int v = 1; v <= vt_irq_max(&t, EPT_IRQ_MSIX); v++) {
				fails += vt_test_irq(&t, EPT_IRQ_MSIX, v);
			}
		} else if (strncmp(a, "msix", 4) == 0) {
			fails += vt_test_irq(&t, EPT_IRQ_MSIX, (int)num(a + 4));
		} else if (strcmp(a, "msi") == 0) {
			for (int v = 1; v <= vt_irq_max(&t, EPT_IRQ_MSI); v++) {
				fails += vt_test_irq(&t, EPT_IRQ_MSI, v);
			}
		} else if (strncmp(a, "msi", 3) == 0) {
			fails += vt_test_irq(&t, EPT_IRQ_MSI, (int)num(a + 3));
		} else if (colon != NULL && (strncmp(a, "read:", 5) == 0 ||
					     strncmp(a, "write:", 6) == 0 ||
					     strncmp(a, "copy:", 5) == 0)) {
			*colon = '\0';
			fails += vt_test_xfer(&t, a, size, unaligned);
		} else {
			die("%s: bars, bar<n>, msi, msi<n>, msix<n>, read:<size>, write:<size>, "
			    "copy:<size>[u], doorbell, all", a);
		}
	}
	vt_db_off(&t);
	vt_irq_off(&t);
	return fails ? 1 : 0;
}

/*
 * buf [-l limit] [ms]: a 2 MiB DMA buffer for the endpoint's own requests,
 * without vfio: its physical (= bus) address, the first 4 KiB filled with
 * 0xb0f00000 + word index, then every word the endpoint changes, until ^C.
 * The function needs Memory Space and Bus Master (pcictl cmd <bdf> +mem +bme).
 */
static int cmd_buf(int argc, char **argv)
{
	struct vt t = {.limit = 1ULL << 32};
	uint64_t ms = 100;
	struct dbuf b;
	uint32_t *w, shadow[1024];
	struct timespec t0, now;

	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
			t.limit = num(argv[++i]);
		} else {
			ms = num(argv[i]);
		}
	}
	b = hugebuf(&t);
	w = (uint32_t *)b.va;
	for (unsigned int i = 0; i < ARRAY_SIZE(shadow); i++) {
		w[i] = 0xb0f00000U + i;
		shadow[i] = w[i];
	}
	cache_flush(b.va, sizeof(shadow));
	printf("buffer at 0x%" PRIx64 " (2 MiB), first 4 KiB = 0xb0f00000 + word index\n"
	       "endpoint: pcie controller ep host <fn> 0x%" PRIx64 " 4 | pcie controller ep host "
	       "<fn> 0x%" PRIx64 " =<val>\n", b.pa, b.pa, b.pa + 0x10);
	fflush(stdout);
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (;;) {
		usleep((useconds_t)ms * 1000U);
		cache_flush(b.va, sizeof(shadow));
		for (unsigned int i = 0; i < ARRAY_SIZE(shadow); i++) {
			if (w[i] != shadow[i]) {
				clock_gettime(CLOCK_MONOTONIC, &now);
				printf("%9.3f +0x%03x: %08x -> %08x\n",
				       (double)(now.tv_sec - t0.tv_sec) +
					       (now.tv_nsec - t0.tv_nsec) / 1e9,
				       i * 4U, shadow[i], w[i]);
				fflush(stdout);
				shadow[i] = w[i];
			}
		}
	}
	return 0;
}

int main(int argc, char **argv)
{
	struct sigaction sa = {.sa_handler = on_sigbus};
	char path[PATH_MAX];
	const char *c;
	int opt;
	const char *dump_file = NULL;
	int ret = 0;

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

	if (strcmp(c, "buf") == 0) {
		return cmd_buf(argc - 1, argv + 1);
	}
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
		} else if (strcmp(c, "test") == 0) {
			ret = cmd_test(d, n, a);
		} else if (strcmp(c, "resize") == 0) {
			cmd_resize(d, n, a);
		} else if (strcmp(c, "sriov") == 0) {
			cmd_sriov(d, n, a);
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
	return ret;
}
