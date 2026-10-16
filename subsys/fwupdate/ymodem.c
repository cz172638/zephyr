/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal YMODEM (batch) receiver, CRC-16 mode, as described in
 * Chuck Forsberg's "XMODEM/YMODEM Protocol Reference" (1988).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "ymodem.h"

#define SOH    0x01
#define STX    0x02
#define EOT    0x04
#define ACK    0x06
#define NAK    0x15
#define CAN    0x18
#define CRC_C  0x43
#define CTRL_C 0x03

#define BYTE_TIMEOUT_MS   1000
#define PACKET_TIMEOUT_MS 3000
#define MAX_ERRORS        10

enum state {
	WAIT_HEADER, /* block 0 with name and size */
	WAIT_DATA,
	WAIT_EOT2, /* first EOT NAKed, waiting for the repeat */
};

struct ymodem {
	const struct ymodem_ops *ops;
	void *ctx;
	uint8_t buf[1024];
};

static void tx(struct ymodem *y, uint8_t c)
{
	y->ops->send(y->ctx, &c, 1);
}

static void cancel(struct ymodem *y)
{
	static const uint8_t can[] = {CAN, CAN, CAN, CAN, CAN};

	y->ops->send(y->ctx, can, sizeof(can));
}

/* Drop input until the line has been quiet for a while */
static void purge(struct ymodem *y)
{
	uint8_t c;

	while (y->ops->getc(y->ctx, &c, 200) == 0) {
	}
}

static int read_block(struct ymodem *y, size_t n, uint8_t *blk)
{
	uint8_t b0, b1, crc_hi, crc_lo;

	if (y->ops->getc(y->ctx, &b0, BYTE_TIMEOUT_MS) ||
	    y->ops->getc(y->ctx, &b1, BYTE_TIMEOUT_MS)) {
		return -EAGAIN;
	}
	for (size_t i = 0; i < n; i++) {
		if (y->ops->getc(y->ctx, &y->buf[i], BYTE_TIMEOUT_MS)) {
			return -EAGAIN;
		}
	}
	if (y->ops->getc(y->ctx, &crc_hi, BYTE_TIMEOUT_MS) ||
	    y->ops->getc(y->ctx, &crc_lo, BYTE_TIMEOUT_MS)) {
		return -EAGAIN;
	}
	if ((uint8_t)(b0 ^ b1) != 0xff) {
		return -EBADMSG;
	}
	if (crc16_itu_t(0, y->buf, n) != ((crc_hi << 8) | crc_lo)) {
		return -EBADMSG;
	}
	*blk = b0;
	return 0;
}

int ymodem_receive(const struct ymodem_ops *ops, void *ctx, int start_timeout_s)
{
	static struct ymodem y;
	enum state st = WAIT_HEADER;
	size_t size = 0, received = 0;
	uint8_t expect = 1;
	bool started = false;
	int files = 0, errors = 0, waited = 0;
	int ret;

	y.ops = ops;
	y.ctx = ctx;

	tx(&y, CRC_C);

	while (true) {
		uint8_t c, blk;
		size_t n;

		ret = ops->getc(ctx, &c, started ? PACKET_TIMEOUT_MS : 1000);
		if (ret == -EAGAIN) {
			if (!started) {
				if (++waited >= start_timeout_s) {
					cancel(&y);
					return -ETIMEDOUT;
				}
				tx(&y, CRC_C);
				continue;
			}
			if (st == WAIT_HEADER && files > 0 && errors >= 2) {
				/* Sender did not close the batch with an empty header */
				return files;
			}
			if (++errors > MAX_ERRORS) {
				cancel(&y);
				return -ETIMEDOUT;
			}
			tx(&y, st == WAIT_HEADER ? CRC_C : NAK);
			continue;
		}

		switch (c) {
		case SOH:
		case STX:
			n = (c == SOH) ? 128 : 1024;
			ret = read_block(&y, n, &blk);
			if (ret < 0) {
				purge(&y);
				if (++errors > MAX_ERRORS) {
					cancel(&y);
					return -ETIMEDOUT;
				}
				tx(&y, NAK);
				continue;
			}
			started = true;
			errors = 0;

			if (st == WAIT_HEADER) {
				if (blk != 0) {
					/* Repeat of the last data block after a lost ACK */
					tx(&y, ACK);
					continue;
				}
				if (y.buf[0] == '\0') {
					/* Empty name: end of batch */
					tx(&y, ACK);
					return files;
				}
				y.buf[n - 1] = '\0';
				size = strtoul((char *)y.buf + strlen((char *)y.buf) + 1, NULL, 10);
				received = 0;
				expect = 1;
				ret = ops->open(ctx, (char *)y.buf, size);
				if (ret < 0) {
					cancel(&y);
					return ret;
				}
				tx(&y, ACK);
				tx(&y, CRC_C);
				st = WAIT_DATA;
				continue;
			}

			if (blk == (uint8_t)(expect - 1)) {
				/* Duplicate, our ACK got lost */
				tx(&y, ACK);
				continue;
			}
			if (blk != expect) {
				cancel(&y);
				return -EPROTO;
			}
			if (size != 0) {
				n = MIN(n, size - MIN(received, size));
			}
			ret = ops->data(ctx, received, y.buf, n);
			if (ret < 0) {
				cancel(&y);
				return ret;
			}
			received += n;
			expect++;
			st = WAIT_DATA;
			tx(&y, ACK);
			break;

		case EOT:
			if (st == WAIT_DATA) {
				/* NAK the first EOT, as the protocol suggests */
				st = WAIT_EOT2;
				tx(&y, NAK);
			} else if (st == WAIT_EOT2) {
				ret = ops->close(ctx, received);
				if (ret < 0) {
					cancel(&y);
					return ret;
				}
				files++;
				st = WAIT_HEADER;
				tx(&y, ACK);
				tx(&y, CRC_C);
			} else {
				tx(&y, ACK);
			}
			break;

		case CAN:
			if (ops->getc(ctx, &c, BYTE_TIMEOUT_MS) == 0 && c == CAN) {
				return -ECANCELED;
			}
			break;

		case CTRL_C:
			if (!started) {
				cancel(&y);
				return -ECANCELED;
			}
			break;

		default:
			/* Line noise, e.g. the shell prompt echo */
			break;
		}
	}
}
