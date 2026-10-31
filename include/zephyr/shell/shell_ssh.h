/*
 * Copyright (c) 2024 Grant Ramsay <grant.ramsay@hotmail.com>
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for the SSH shell backend.
 * @ingroup shell_ssh
 */

#ifndef ZEPHYR_INCLUDE_SHELL_SHELL_SSH_H_
#define ZEPHYR_INCLUDE_SHELL_SHELL_SSH_H_

#include <zephyr/net/socket.h>
#include <zephyr/net/ssh/server.h>
#include <zephyr/shell/shell.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup shell_ssh SSH shell backend
 * @ingroup shell_backends
 * @brief Shell access over an SSH connection.
 * @{
 */

/** Maximum password length supported by the shell SSH transport. */
#define SHELL_SSH_MAX_PASSWORD_LEN 32

/** @cond INTERNAL_HIDDEN */

struct ssh_channel;
struct shell_ssh;

/** @endcond */

/** @brief SSH password storage. */
struct shell_ssh_passwd {
	/** Buffer to hold the password. */
	char buf[SHELL_SSH_MAX_PASSWORD_LEN + 1];
	/** Length of the password stored in the buffer. */
	size_t len;
};

/** @brief SSH transport user data.
 *
 * This structure holds the user data for the shell SSH transport. It is
 * passed to the transport event callback and can be used to store any
 * necessary context for the transport.
 */
struct shell_ssh_context {
	/** Shell instance associated with this transport. */
	const struct shell *sh;
	/** Shell SSH transport API. */
	struct shell_ssh *ssh;
	/** SSH channel associated with this transport. */
	struct ssh_channel *channel;
	/** SSH transport associated with this user data. */
	struct ssh_transport *transport;
	/** SSH server password storage */
	struct shell_ssh_passwd password;
	/** Instance of the SSH connection (could be client or server) */
	int ssh_instance;
	/** Is this context in use (server-side pool tracking) */
	bool in_use;
};

#if defined(CONFIG_SHELL_SSH_SCP) || defined(__DOXYGEN__)
/**
 * @brief Destination of files sent with legacy scp (scp -O host:<path>).
 *
 * The callbacks run in the SSH exec work queue thread.
 */
struct shell_ssh_scp_sink {
	/**
	 * Start a file. path: target given to scp, name: source file name,
	 * size: announced length. 0 to accept, negative errno to refuse.
	 */
	int (*open)(const char *path, const char *name, size_t size, void **handle);
	/** File data, in order. Negative errno aborts the transfer. */
	int (*write)(void *handle, const uint8_t *data, size_t len);
	/**
	 * End of the file. ok: all data arrived. A one-line message for the
	 * user can be left in msg. 0 or a negative errno.
	 */
	int (*close)(void *handle, bool ok, char *msg, size_t msg_len);
};

/** @brief Register the scp sink (one, replaces the previous one). */
void shell_ssh_scp_sink_set(const struct shell_ssh_scp_sink *sink);
#endif

/** @cond INTERNAL_HIDDEN */
extern const struct shell_transport_api shell_ssh_transport_api;
/** @endcond */

/** Helper macro to construct internal symbol names for a shell SSH transport instance. */
#define SHELL_SSH_GET_NAME(_name) _name##_shell_ssh

/**
 * @brief Define a shell SSH transport instance.
 *
 * This macro defines a shell transport backed by SSH. Use it together
 * with SHELL_DEFINE() to create a shell instance that is accessible
 * over SSH.
 *
 * @param _name  Name of the transport instance. This token is used to
 *               construct the internal symbol names.
 */
#define SHELL_SSH_DEFINE(_name)					\
	static struct shell_ssh SHELL_SSH_GET_NAME(_name);	\
	static struct shell_transport _name = {			\
		.api = &shell_ssh_transport_api,		\
		.ctx = &_name##_shell_ssh,			\
	}

/**
 * @brief SSH transport API
 */
struct shell_ssh {
	/** Transport handler */
	shell_transport_handler_t handler;
	/** SSH channel */
	struct ssh_channel *ssh_channel;
	/** Transport context */
	void *context;
	/** Is this transport in use */
	bool in_use;
};

/** @cond INTERNAL_HIDDEN */

#if defined(CONFIG_SSH_SERVER)
int shell_sshd_event_callback(struct ssh_server *ssh_server,
			      const struct ssh_server_event *event,
			      void *user_data);
int shell_sshd_transport_event_callback(struct ssh_transport *transport,
					const struct ssh_transport_event *event,
					void *user_data);
#endif /* CONFIG_SSH_SERVER */

/** @endcond */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SHELL_SHELL_SSH_H_ */
