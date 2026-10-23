/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_FWUPDATE_YMODEM_H_
#define ZEPHYR_SUBSYS_FWUPDATE_YMODEM_H_

#include <stddef.h>
#include <stdint.h>

struct ymodem_ops {
	/* Read one byte, return 0 or -EAGAIN on timeout */
	int (*getc)(void *ctx, uint8_t *c, int timeout_ms);
	/* Send bytes to the sender */
	void (*send)(void *ctx, const uint8_t *buf, size_t len);
	/* A file starts; size is 0 if the sender did not tell */
	int (*open)(void *ctx, const char *name, size_t size);
	/* File data, offset from the start of the file */
	int (*data)(void *ctx, size_t offset, const uint8_t *buf, size_t len);
	/* The file ended */
	int (*close)(void *ctx, size_t size);
};

/*
 * Receive one YMODEM batch (CRC-16, 128 and 1024 byte blocks).
 *
 * @return number of files received (>= 0) or a negative errno:
 *   -ECANCELED  cancelled by the sender (CAN CAN) or by Ctrl-C before start
 *   -ETIMEDOUT  no sender within the start timeout, or too many errors
 *   other       error returned by an ops callback (transfer cancelled)
 */
int ymodem_receive(const struct ymodem_ops *ops, void *ctx, int start_timeout_s);

/*
 * Receive one file with XMODEM: 128 (XMODEM) or 1024 byte (XMODEM-1K)
 * blocks, CRC-16, or the 8-bit checksum if the sender ignores the 'C'.
 * There is no header: open() gets the name "xmodem" and size 0, and the
 * file is padded by the sender to a whole block (usually with 0x1a).
 *
 * @return 1 when a file was received, or a negative errno as above
 */
int xmodem_receive(const struct ymodem_ops *ops, void *ctx, int start_timeout_s);

#endif /* ZEPHYR_SUBSYS_FWUPDATE_YMODEM_H_ */
