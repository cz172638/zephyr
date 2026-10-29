/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal YMODEM (batch) and XMODEM receiver, as described in Chuck
 * Forsberg's "XMODEM/YMODEM Protocol Reference" (1988). YMODEM in CRC-16
 * mode; XMODEM in CRC-16 mode, falling back to the original 8-bit
 * checksum when the sender does not answer the 'C'.
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
/*
 * XMODEM: 'C' only for this many seconds, then 'C' and NAK alternate, so
 * both a CRC-only sender (tio, which ignores NAK) and a checksum-only one
 * get their request whenever they are started
 */
#define XMODEM_CRC_TRIES  3

enum state {
	WAIT_HEADER, /* block 0 with name and size */
	WAIT_DATA,
	WAIT_EOT2, /* first EOT NAKed, waiting for the repeat */
};

struct ymodem {
	const struct ymodem_ops *ops;
	void *ctx;
	bool crc;
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
	uint8_t b0, b1, crc_hi, crc_lo = 0;

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
	    (y->crc && y->ops->getc(y->ctx, &crc_lo, BYTE_TIMEOUT_MS))) {
		return -EAGAIN;
	}
	if ((uint8_t)(b0 ^ b1) != 0xff) {
		return -EBADMSG;
	}
	if (y->crc) {
		if (crc16_itu_t(0, y->buf, n) != ((crc_hi << 8) | crc_lo)) {
			return -EBADMSG;
		}
	} else {
		uint8_t sum = 0;

		for (size_t i = 0; i < n; i++) {
			sum += y->buf[i];
		}
		if (sum != crc_hi) {
			return -EBADMSG;
		}
	}
	*blk = b0;
	return 0;
}

static int receive(const struct ymodem_ops *ops, void *ctx, int start_timeout_s, bool xmodem)
{
	static struct ymodem y;
	enum state st = xmodem ? WAIT_DATA : WAIT_HEADER;
	size_t size = 0, received = 0;
	uint8_t expect = 1;
	bool started = false;
	int files = 0, errors = 0, waited = 0;
	int ret;

	y.ops = ops;
	y.ctx = ctx;
	y.crc = true;

	if (xmodem) {
		/* No header block: one file, name and size unknown */
		ret = ops->open(ctx, "xmodem", 0);
		if (ret < 0) {
			return ret;
		}
	}

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
				/*
				 * The mode is the one of the last request: a sender
				 * answers at once, the next request is 1 s later
				 */
				y.crc = !xmodem || waited < XMODEM_CRC_TRIES || (waited & 1) == 0;
				tx(&y, y.crc ? CRC_C : NAK);
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

			if (st == WAIT_EOT2 && blk == 0) {
				/*
				 * The sender took our NAK of the first EOT as the end of
				 * the file and goes on with the next header (lrzsz sb):
				 * the file is complete.
				 */
				ret = ops->close(ctx, received);
				if (ret < 0) {
					cancel(&y);
					return ret;
				}
				files++;
				st = WAIT_HEADER;
			}

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
			if (xmodem && started) {
				/* Single file, no batch: done at the first EOT */
				ret = ops->close(ctx, received);
				if (ret < 0) {
					cancel(&y);
					return ret;
				}
				tx(&y, ACK);
				return 1;
			}
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

int ymodem_receive(const struct ymodem_ops *ops, void *ctx, int start_timeout_s)
{
	return receive(ops, ctx, start_timeout_s, false);
}

int xmodem_receive(const struct ymodem_ops *ops, void *ctx, int start_timeout_s)
{
	return receive(ops, ctx, start_timeout_s, true);
}
