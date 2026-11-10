/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * LLEXT self test for the lab agent: relocations of all the usual kinds
 * (calls into the firmware, rodata, data, bss, a function pointer
 * table), writes to the extension's own data, and the lab api table.
 */

#include <stdint.h>

#include <zephyr/lab/lab.h>
#include <zephyr/lab/mod.h>
#include <zephyr/llext/symbol.h>
#include <zephyr/sys/printk.h>

static const char msg[] = "lab hello ext";
static const uint32_t table[4] = {0x01234567, 0x89abcdef, 0xdeadbeef, 0x55aa55aa};
static uint32_t counter = 7;
static uint32_t zeroed[64];

static uint32_t op_add(uint32_t a, uint32_t b)
{
	return a + b;
}

static uint32_t op_xor(uint32_t a, uint32_t b)
{
	return a ^ b;
}

static uint32_t (*const ops[])(uint32_t, uint32_t) = {op_add, op_xor};

static uint32_t crc32(const uint8_t *d, uint32_t n)
{
	uint32_t c = 0xffffffff;

	while (n--) {
		c ^= *d++;
		for (int k = 0; k < 8; k++) {
			c = (c >> 1) ^ (0xedb88320 & (0 - (c & 1)));
		}
	}
	return ~c;
}

int lab_ext_main(const struct lab_ext_api *api, int argc, char **argv)
{
	int fail = 0;
	uint64_t t0, t1;
	uint32_t acc = 0;
	uint8_t *buf;

	/* direct call to an exported kernel symbol */
	printk("%s: argc %d, argv[0] %s\n", msg, argc, argc > 0 ? argv[0] : "-");

	if (api->version != LAB_EXT_API_VERSION || api->size < sizeof(*api)) {
		printk("api version %u size %u, want %u/%u\n", api->version, api->size,
		       LAB_EXT_API_VERSION, (uint32_t)sizeof(*api));
		return -1;
	}

	for (int i = 0; i < 64; i++) {
		if (zeroed[i] != 0) {
			api->printf("bss not zero at %d\n", i);
			fail++;
			break;
		}
		zeroed[i] = i;
	}
	if (counter != 7) {
		api->printf("data: counter %u, want 7 (loaded twice without reload?)\n", counter);
		fail++;
	}
	counter++;

	for (int i = 0; i < 4; i++) {
		acc = ops[i & 1](acc, table[i]);
	}
	if (acc != (((0x01234567U ^ 0x89abcdefU) + 0xdeadbeefU) ^ 0x55aa55aaU)) {
		api->printf("function table / rodata: %08x\n", acc);
		fail++;
	}
	if (crc32((const uint8_t *)"123456789", 9) != 0xcbf43926) {
		api->printf("crc32 check value wrong\n");
		fail++;
	}

	t0 = api->now_ns();
	api->busy_wait_us(1000);
	t1 = api->now_ns();
	if (t1 - t0 < 900000 || t1 - t0 > 5000000) {
		api->printf("busy_wait 1 ms took %llu ns\n", t1 - t0);
		fail++;
	}

	buf = api->alloc(4096, 64);
	if (buf == NULL) {
		api->printf("alloc failed\n");
		fail++;
	} else {
		for (int i = 0; i < 4096; i++) {
			buf[i] = (uint8_t)i;
		}
		api->cache_flush(buf, 4096);
		api->cache_invd(buf, 4096);
		if (crc32(buf, 4096) != crc32(buf, 4096) || buf[4095] != 0xff) {
			fail++;
		}
		api->free(buf);
	}

	api->printf("role %s, peer offset %lld ns, %d failures\n", api->role(),
		    api->peer_offset_ns(), fail);
	return fail;
}
EXPORT_SYMBOL(lab_ext_main);
LAB_IMAGE_TAG();
