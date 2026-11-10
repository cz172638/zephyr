/*
 * Copyright (c) 2024 Grant Ramsay <grant.ramsay@hotmail.com>
 * Copyright (c) 2026 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(shell_ssh, CONFIG_SHELL_SSH_LOG_LEVEL);

#include <mbedtls/platform_util.h>

#include <zephyr/kernel.h>
#include <zephyr/net/ssh/server.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_ssh.h>
#if defined(CONFIG_SHELL_SSH_EXEC)
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/shell/shell_dummy.h>
#endif

#if defined(CONFIG_SSH_SERVER)

static K_MUTEX_DEFINE(ssh_lock);

#define SHELL_SSH_NAME(n, _) &shell_ssh_##n

#define SHELL_SSH_DEFINE_ALL(n, _)					\
	SHELL_SSH_DEFINE(shell_transport_ssh_##n);			\
	SHELL_DEFINE(shell_ssh_##n,					\
		     CONFIG_SHELL_PROMPT_SSH,				\
		     &shell_transport_ssh_##n,				\
		     CONFIG_SHELL_BACKEND_SERIAL_LOG_MESSAGE_QUEUE_SIZE, \
		     CONFIG_SHELL_BACKEND_SERIAL_LOG_MESSAGE_QUEUE_TIMEOUT, \
		     SHELL_FLAG_OLF_CRLF)

LISTIFY(CONFIG_SSH_SERVER_SHELL_COUNT, SHELL_SSH_DEFINE_ALL, (;));

static const struct shell *const ssh_shell_instances[] = {
	LISTIFY(CONFIG_SSH_SERVER_SHELL_COUNT, SHELL_SSH_NAME, (,), shell_ssh)
};

static struct shell_ssh_context ssh_server_contexts[CONFIG_SSH_SERVER_SHELL_COUNT];

static struct shell_ssh_context *ssh_server_context_alloc(void)
{
	k_mutex_lock(&ssh_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(ssh_server_contexts); i++) {
		if (!ssh_server_contexts[i].in_use) {
			memset(&ssh_server_contexts[i], 0,
			       sizeof(ssh_server_contexts[i]));
			ssh_server_contexts[i].in_use = true;
			k_mutex_unlock(&ssh_lock);
			return &ssh_server_contexts[i];
		}
	}

	k_mutex_unlock(&ssh_lock);

	return NULL;
}

static void ssh_server_context_free(struct shell_ssh_context *ctx)
{
	k_mutex_lock(&ssh_lock, K_FOREVER);
	ctx->in_use = false;
	k_mutex_unlock(&ssh_lock);
}

static const struct shell *ssh_server_shell_instance_alloc(struct shell_ssh_context *ctx)
{
	k_mutex_lock(&ssh_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(ssh_shell_instances); i++) {
		const struct shell *sh = ssh_shell_instances[i];
		struct shell_ssh *sh_ssh = sh->iface->ctx;

		if (!sh_ssh->in_use) {
			sh_ssh->in_use = true;
			ctx->ssh = sh_ssh;
			k_mutex_unlock(&ssh_lock);
			return sh;
		}
	}

	k_mutex_unlock(&ssh_lock);

	return NULL;
}

static void ssh_server_shell_instance_free(const struct shell *sh)
{
	struct shell_ssh *sh_ssh;

	k_mutex_lock(&ssh_lock, K_FOREVER);

	sh_ssh = sh->iface->ctx;
	sh_ssh->in_use = false;

	k_mutex_unlock(&ssh_lock);
}
#endif

static int shell_ssh_init(const struct shell_transport *transport,
			  const void *config,
			  shell_transport_handler_t evt_handler,
			  void *context)
{
	struct shell_ssh *sh_ssh = (struct shell_ssh *)transport->ctx;

	sh_ssh->handler = evt_handler;
	sh_ssh->context = context;
	sh_ssh->ssh_channel = (void *)config;

	return 0;
}

static int shell_ssh_uninit(const struct shell_transport *transport)
{
	ARG_UNUSED(transport);
	return 0;
}

static int shell_ssh_enable(const struct shell_transport *transport, bool blocking)
{
	ARG_UNUSED(transport);
	ARG_UNUSED(blocking);
	return 0;
}

static int shell_ssh_write(const struct shell_transport *transport,
			   const void *data, size_t length, size_t *cnt)
{
	struct shell_ssh *sh_ssh = (struct shell_ssh *)transport->ctx;
	int ret;

	ret = ssh_channel_write(sh_ssh->ssh_channel, data, length);
	if (ret < 0) {
		return ret;
	}

	*cnt = ret;

	return 0;
}

static int shell_ssh_read(const struct shell_transport *transport,
			  void *data, size_t length, size_t *cnt)
{
	struct shell_ssh *sh_ssh = (struct shell_ssh *)transport->ctx;
	int ret;

	ret = ssh_channel_read(sh_ssh->ssh_channel, data, length);
	if (ret < 0) {
		return ret;
	}

	*cnt = ret;
	return 0;
}

const struct shell_transport_api shell_ssh_transport_api = {
	.init = shell_ssh_init,
	.uninit = shell_ssh_uninit,
	.enable = shell_ssh_enable,
	.write = shell_ssh_write,
	.read = shell_ssh_read
};

#if defined(CONFIG_SSH_SERVER)
int shell_sshd_event_callback(struct ssh_server *sshd,
			      const struct ssh_server_event *event,
			      void *user_data)
{
	ARG_UNUSED(sshd);
	ARG_UNUSED(user_data);

	switch (event->type) {
	case SSH_SERVER_EVENT_CLOSED:
		LOG_INF("Server closed");
		break;
	case SSH_SERVER_EVENT_CLIENT_CONNECTED:
		LOG_INF("Server client connected");
		break;
	case SSH_SERVER_EVENT_CLIENT_DISCONNECTED:
		LOG_INF("Server client disconnected");
		break;
	default:
		return -1;
	}

	return 0;
}

static int sshd_channel_event_callback(struct ssh_channel *channel,
				       const struct ssh_channel_event *event,
				       void *user_data);

int shell_sshd_transport_event_callback(struct ssh_transport *transport,
					const struct ssh_transport_event *event,
					void *user_data)
{
	ARG_UNUSED(transport);
	ARG_UNUSED(user_data);

	switch (event->type) {
	case SSH_TRANSPORT_EVENT_CHANNEL_OPEN: {
		struct shell_ssh_context *ctx;

		ctx = ssh_server_context_alloc();
		if (ctx == NULL) {
			LOG_ERR("No SSH server contexts available");
			return ssh_channel_open_result(
				event->channel_open.channel, false,
				NULL, NULL);
		}

		return ssh_channel_open_result(event->channel_open.channel, true,
					       sshd_channel_event_callback,
					       ctx);
	}
	default:
		break;
	}

	return 0;
}

static int shell_ssh_setup(struct shell_ssh_context *ctx, struct ssh_channel *channel)
{
	static const struct shell_backend_config_flags cfg_flags =
		SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;
	const struct shell *sh;
	int ret;

	sh = ssh_server_shell_instance_alloc(ctx);
	if (sh != NULL) {
		ret = shell_init(sh, channel, cfg_flags, false, LOG_LEVEL_NONE);
		if (ret == 0) {
			ctx->sh = sh;
		} else {
			LOG_DBG("Cannot init ssh shell (%d)", ret);
			ssh_server_shell_instance_free(sh);
		}
	} else {
		LOG_ERR("No SSH server shell instances remaining");
		ret = -ENOMEM;
	}

	return ret;
}


#if defined(CONFIG_SHELL_SSH_EXEC)
/*
 * "exec" requests (ssh host 'cmd'): the command runs on the dummy shell
 * backend from a work queue, its output goes back on the channel, then
 * exit-status, EOF and CLOSE. With CONFIG_SHELL_SSH_SCP, "scp -t <path>"
 * (scp -O, the legacy protocol) is served by a registered sink instead.
 */
enum {
	EX_RUNNING,
	EX_EOF,
	EX_CLOSED,
};

struct shell_ssh_exec {
	struct k_work work;
	struct shell_ssh_context *ctx;
	struct ssh_channel *channel;
	struct k_sem sem;
	atomic_t flags;
	char cmd[CONFIG_SHELL_CMD_BUFF_SIZE];
};

static struct shell_ssh_exec ssh_exec[CONFIG_SSH_SERVER_SHELL_COUNT];
static K_THREAD_STACK_DEFINE(ssh_exec_stack, CONFIG_SHELL_SSH_EXEC_STACK_SIZE);
static struct k_work_q ssh_exec_q;
static bool ssh_exec_q_started;

static struct shell_ssh_exec *ssh_exec_get(struct shell_ssh_context *ctx)
{
	return &ssh_exec[ctx - ssh_server_contexts];
}

static int ex_write(struct shell_ssh_exec *ex, const void *buf, size_t len, bool err)
{
	const uint8_t *p = buf;

	while (len > 0) {
		int n;

		if (atomic_test_bit(&ex->flags, EX_CLOSED)) {
			return -ECONNRESET;
		}
		n = err ? ssh_channel_write_stderr(ex->channel, p, len)
			: ssh_channel_write(ex->channel, p, len);
		if (n < 0) {
			return n;
		}
		p += n;
		len -= n;
		if (n == 0) {
			(void)k_sem_take(&ex->sem, K_MSEC(100));
		}
	}
	return 0;
}

static void ex_printf(struct shell_ssh_exec *ex, bool err, const char *fmt, ...)
{
	char buf[160];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintk(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	(void)ex_write(ex, buf, MIN(n, (int)sizeof(buf) - 1), err);
}

/* > 0 bytes, 0 on EOF, -ETIMEDOUT, -ECONNRESET */
static int ex_read(struct shell_ssh_exec *ex, void *buf, size_t len, k_timeout_t timeout)
{
	k_timepoint_t end = sys_timepoint_calc(timeout);

	while (true) {
		bool eof = atomic_test_bit(&ex->flags, EX_EOF);
		int n;

		if (atomic_test_bit(&ex->flags, EX_CLOSED)) {
			return -ECONNRESET;
		}
		n = ssh_channel_read(ex->channel, buf, len);
		if (n != 0) {
			return n;
		}
		if (eof) {
			/* All data comes before the EOF message */
			return 0;
		}
		if (k_sem_take(&ex->sem, sys_timepoint_timeout(end)) != 0) {
			return -ETIMEDOUT;
		}
	}
}

#if defined(CONFIG_SHELL_SSH_SCP)
static const struct shell_ssh_scp_sink *scp_sink;

void shell_ssh_scp_sink_set(const struct shell_ssh_scp_sink *sink)
{
	scp_sink = sink;
}

#define SCP_TIMEOUT K_SECONDS(30)

static int scp_read_line(struct shell_ssh_exec *ex, char *line, size_t size)
{
	size_t n = 0;

	while (true) {
		char c;
		int ret = ex_read(ex, &c, 1, SCP_TIMEOUT);

		if (ret <= 0) {
			return ret == 0 ? (n == 0 ? 0 : -EPROTO) : ret;
		}
		if (c == '\n') {
			line[n] = '\0';
			return 1;
		}
		if (n + 1 >= size) {
			return -EPROTO;
		}
		line[n++] = c;
	}
}

static int scp_ack(struct shell_ssh_exec *ex)
{
	return ex_write(ex, "", 1, false);
}

static int scp_nak(struct shell_ssh_exec *ex, const char *what, int err)
{
	ex_printf(ex, false, "\x01scp: %s: error %d\n", what, err);
	return 1;
}

static int scp_receive(struct shell_ssh_exec *ex, const char *path)
{
	static uint8_t buf[1024];
	char line[160], msg[128];
	int ret;

	if (scp_sink == NULL) {
		return scp_nak(ex, path, -ENOSYS);
	}
	if (scp_ack(ex) < 0) {
		return 1;
	}

	while (true) {
		unsigned long mode;
		size_t size, left;
		char *p, *name;
		void *h;
		bool ok;

		ret = scp_read_line(ex, line, sizeof(line));
		if (ret == 0) {
			return 0; /* EOF: done */
		}
		if (ret < 0) {
			LOG_ERR("scp: read (%d)", ret);
			return 1;
		}

		switch (line[0]) {
		case 'T': /* times (-p) */
			(void)scp_ack(ex);
			continue;
		case 'C':
			break;
		case '\x01':
		case '\x02':
			LOG_ERR("scp: sender: %s", &line[1]);
			return 1;
		default:
			return scp_nak(ex, "only single files are supported", -ENOTSUP);
		}

		mode = strtoul(&line[1], &p, 8);
		size = strtoull(p, &name, 10);
		if (p == &line[1] || name == p || *name != ' ') {
			return scp_nak(ex, "bad C line", -EPROTO);
		}
		name++;
		ARG_UNUSED(mode);

		ret = scp_sink->open(path, name, size, &h);
		if (ret < 0) {
			return scp_nak(ex, path, ret);
		}
		(void)scp_ack(ex);

		ok = true;
		for (left = size; left > 0;) {
			ret = ex_read(ex, buf, MIN(left, sizeof(buf)), SCP_TIMEOUT);
			if (ret <= 0) {
				ok = false;
				break;
			}
			if (ok && scp_sink->write(h, buf, ret) < 0) {
				ok = false;
				break;
			}
			left -= ret;
		}
		if (ok) {
			/* The sender's status byte after the data */
			ret = ex_read(ex, buf, 1, SCP_TIMEOUT);
			ok = ret == 1 && buf[0] == 0;
		}

		msg[0] = '\0';
		ret = scp_sink->close(h, ok, msg, sizeof(msg));
		if (!ok) {
			LOG_ERR("scp: %s: transfer incomplete", name);
			return 1;
		}
		if (ret < 0) {
			ex_printf(ex, false, "\x01scp: %s: %s (%d)\n", path,
				  msg[0] != '\0' ? msg : "failed", ret);
			return 1;
		}
		(void)scp_ack(ex);
		if (msg[0] != '\0') {
			ex_printf(ex, true, "%s\n", msg);
		}
	}
}

/* "scp [-v] [-d] [-p] -t [--] <path>": the path, or NULL */
static char *scp_sink_path(char *cmd)
{
	char *argv[8];
	bool to = false;
	int argc = 0;

	for (char *p = cmd; *p != '\0' && argc < ARRAY_SIZE(argv);) {
		while (*p == ' ') {
			*p++ = '\0';
		}
		if (*p == '\0') {
			break;
		}
		argv[argc++] = p;
		while (*p != '\0' && *p != ' ') {
			p++;
		}
	}
	if (argc < 3 || strcmp(argv[0], "scp") != 0) {
		return NULL;
	}
	for (int i = 1; i < argc - 1; i++) {
		if (strcmp(argv[i], "-t") == 0) {
			to = true;
		}
	}
	return to ? argv[argc - 1] : NULL;
}
#endif /* CONFIG_SHELL_SSH_SCP */

static int ex_shell(struct shell_ssh_exec *ex)
{
	const struct shell *sh = shell_backend_dummy_get_ptr();
	const char *out;
	size_t len;
	int ret;

	shell_backend_dummy_lock();
	shell_backend_dummy_clear_output(sh);
	ret = shell_execute_cmd(sh, ex->cmd);
	out = shell_backend_dummy_get_output(sh, &len);
	/* The shell starts with a new line after the (not echoed) command */
	while (len > 0 && (*out == '\r' || *out == '\n')) {
		out++;
		len--;
	}
	(void)ex_write(ex, out, len, false);
	if (len >= CONFIG_SHELL_BACKEND_DUMMY_BUF_SIZE - 1) {
		ex_printf(ex, true, "[output truncated at %d bytes]\n",
			  CONFIG_SHELL_BACKEND_DUMMY_BUF_SIZE - 1);
	}
	shell_backend_dummy_unlock();
	if (ret == 0) {
		return 0;
	}
	ret = (ret < 0 ? -ret : ret) & 0xff;
	return ret != 0 ? ret : 1;
}

static void ex_work(struct k_work *work)
{
	struct shell_ssh_exec *ex = CONTAINER_OF(work, struct shell_ssh_exec, work);
	int status = -1;
	bool closed;

	LOG_INF("exec: %s", ex->cmd);

#if defined(CONFIG_SHELL_SSH_SCP)
	if (strncmp(ex->cmd, "scp ", 4) == 0) {
		char tmp[sizeof(ex->cmd)];
		char *path;

		strcpy(tmp, ex->cmd);
		path = scp_sink_path(tmp);
		if (path != NULL) {
			status = scp_receive(ex, path);
		}
	}
#endif
	if (status < 0) {
		status = ex_shell(ex);
	}

	k_mutex_lock(&ssh_lock, K_FOREVER);
	atomic_clear_bit(&ex->flags, EX_RUNNING);
	closed = atomic_test_bit(&ex->flags, EX_CLOSED);
	k_mutex_unlock(&ssh_lock);

	if (closed) {
		ssh_server_context_free(ex->ctx);
	} else {
		(void)ssh_channel_exit(ex->channel, status);
	}
}

/* Called from the transport thread: queue the command, reply right away */
static int ex_start(struct shell_ssh_context *ctx, struct ssh_channel *channel,
		    const struct ssh_channel_event_channel_request *req)
{
	struct shell_ssh_exec *ex = ssh_exec_get(ctx);
	int ret;

	if (ctx->sh != NULL || atomic_test_bit(&ex->flags, EX_RUNNING) ||
	    req->data == NULL || req->data_len == 0 || req->data_len >= sizeof(ex->cmd)) {
		return req->want_reply ? ssh_channel_request_result(channel, false) : 0;
	}

	if (!ssh_exec_q_started) {
		static const struct k_work_queue_config cfg = {.name = "ssh_exec"};

		k_work_queue_start(&ssh_exec_q, ssh_exec_stack,
				   K_THREAD_STACK_SIZEOF(ssh_exec_stack),
				   CONFIG_SHELL_SSH_EXEC_PRIORITY, &cfg);
		ssh_exec_q_started = true;
	}

	memcpy(ex->cmd, req->data, req->data_len);
	ex->cmd[req->data_len] = '\0';
	ex->ctx = ctx;
	ex->channel = channel;
	k_sem_init(&ex->sem, 0, 1);
	atomic_set(&ex->flags, BIT(EX_RUNNING));
	k_work_init(&ex->work, ex_work);

	/* The reply is queued before anything the command sends */
	ret = req->want_reply ? ssh_channel_request_result(channel, true) : 0;
	(void)k_work_submit_to_queue(&ssh_exec_q, &ex->work);

	return ret;
}

/* Channel events while a command runs; true if consumed */
static bool ex_event(struct shell_ssh_context *ctx, const struct ssh_channel_event *event)
{
	struct shell_ssh_exec *ex = ssh_exec_get(ctx);
	bool running;

	if (!atomic_test_bit(&ex->flags, EX_RUNNING)) {
		return false;
	}

	switch (event->type) {
	case SSH_CHANNEL_EVENT_EOF:
		atomic_set_bit(&ex->flags, EX_EOF);
		break;
	case SSH_CHANNEL_EVENT_CLOSED:
		k_mutex_lock(&ssh_lock, K_FOREVER);
		atomic_set_bit(&ex->flags, EX_CLOSED);
		running = atomic_test_bit(&ex->flags, EX_RUNNING);
		k_mutex_unlock(&ssh_lock);
		k_sem_give(&ex->sem);
		/* The worker frees the context when it is done */
		return running;
	default:
		break;
	}
	k_sem_give(&ex->sem);
	return event->type != SSH_CHANNEL_EVENT_CLOSED;
}
#endif /* CONFIG_SHELL_SSH_EXEC */

static int sshd_channel_event_callback(struct ssh_channel *channel,
				       const struct ssh_channel_event *event,
				       void *user_data)
{
	struct shell_ssh_context *ctx = user_data;

#if defined(CONFIG_SHELL_SSH_EXEC)
	if (event->type == SSH_CHANNEL_EVENT_REQUEST &&
	    event->channel_request.type == SSH_CHANNEL_REQUEST_EXEC) {
		return ex_start(ctx, channel, &event->channel_request);
	}
	if (ex_event(ctx, event)) {
		return 0;
	}
#endif

	switch (event->type) {
	case SSH_CHANNEL_EVENT_REQUEST: {
		bool success = false;

		LOG_DBG("Server channel request (%d)", event->channel_request.type);

		switch (event->channel_request.type) {
		case SSH_CHANNEL_REQUEST_SHELL: {
			int ret;

			ret = shell_ssh_setup(ctx, channel);
			if (ret != 0) {
				LOG_ERR("shell_init error (%d)", ret);
			} else {
				success = true;
			}

			break;
		}

		case SSH_CHANNEL_REQUEST_PSEUDO_TERMINAL:
			LOG_DBG("PTY request received, ignoring");
			success = true;
			break;
		case SSH_CHANNEL_REQUEST_ENV_VAR:
			LOG_DBG("Environment variable request received, ignoring");
			success = true;
			break;
		case SSH_CHANNEL_REQUEST_WINDOW_CHANGE:
			LOG_DBG("Window change request received, ignoring");
			success = true;
			break;

		default:
			break;
		}

		if (event->channel_request.want_reply) {
			return ssh_channel_request_result(channel, success);
		}

		return 0;
	}

	case SSH_CHANNEL_EVENT_RX_DATA_READY: {
		struct shell_ssh *sh_ssh = ctx->ssh;

		if (sh_ssh != NULL && sh_ssh->handler != NULL) {
			sh_ssh->handler(SHELL_TRANSPORT_EVT_RX_RDY, sh_ssh->context);
		}
		break;
	}

	case SSH_CHANNEL_EVENT_TX_DATA_READY: {
		struct shell_ssh *sh_ssh = ctx->ssh;

		if (sh_ssh != NULL && sh_ssh->handler != NULL) {
			sh_ssh->handler(SHELL_TRANSPORT_EVT_TX_RDY, sh_ssh->context);
		}
		break;
	}

	case SSH_CHANNEL_EVENT_RX_STDERR_DATA_READY:
		LOG_INF("Server channel RX ext data ready");
		break;

	case SSH_CHANNEL_EVENT_EOF:
		LOG_INF("Server channel EOF");
		break;

	case SSH_CHANNEL_EVENT_CLOSED:
		LOG_INF("Server channel closed");
		if (ctx->sh != NULL) {
			shell_uninit(ctx->sh, NULL);
			ssh_server_shell_instance_free(ctx->sh);
			ctx->sh = NULL;
			ctx->ssh = NULL;
		}
		ssh_server_context_free(ctx);
		break;

	default:
		return -1;
	}

	return 0;
}
#endif /* CONFIG_SSH_SERVER */
