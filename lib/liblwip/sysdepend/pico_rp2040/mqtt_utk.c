/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the publish drain below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 *
 *	CSC28 PicoCar (B1.2.5, SHARED): the radio task only forwards bytes.
 *	Every payload, topic and command decision comes from mqtt_bridge.c
 *	(subsystem 1) through the kernel-free mqtt_bridge.h; this file calls
 *	no tk_* function. Changes: 4 s keepalive, 1/2/4/8 s reconnect
 *	back-off, retained last will, command topic subscription, and a
 *	50 ms drain that publishes queued telemetry with one pending payload.
 *	Packed build (B1.4): back-off and pending-payload state live in the
 *	16-bit PRV_MQTT word (tools/layout.csv), radio task only.
 *
 *	2026-11-06 (debug over Wi-Fi): the drain also publishes mirrored
 *	console lines (LOG payloads on c28/l, QoS 0, best effort), and a UDP
 *	beacon from the laptop (tools/dashboard/car_link.py, port 18830,
 *	payload starting "C28B") gives the broker address, so a new hotspot
 *	or a new laptop IP needs no rebuild. Without a beacon the car uses
 *	MQTT_BROKER_IP from wifi_secrets.h as before.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"	/* client->conn, for nodelay */
#include "lwip/altcp.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/mqtt_bridge.h"
#include "../../../../app_program/app_types.h"
#include "../../../../app_program/app_layout.h"
#include "../../../../app_program/app_bits.h"
#include "../../../../app_program/subsystem1/cfg_telem.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S    4	/* loss detected <= 7 s, lwipopts.h */
#define MQTT_UTK_RETRY_BASE_MS  1000	/* back-off 1, 2, 4, 8 s */
#define MQTT_UTK_BACKOFF_MAX    3
#define MQTT_UTK_DRAIN_MS       50
#define MQTT_UTK_DRAIN_BURST    2	/* publishes per drain call */
#define MQTT_UTK_PEND_TRIES_MAX 2	/* then the payload is dropped */
#define MQTT_UTK_AVAIL_MAX      16
#define MQTT_UTK_LOG_BURST      2	/* console lines per drain call */
#define MQTT_UTK_LOG_MAX        (1 + TELEM_LOG_TEXT_MAX)	/* hdr + text */
/* Console lines only while the output ring is at most this full, so a
 * slow link fills it with telemetry first (S1-U38 sizes it for that). */
#define MQTT_UTK_LOG_RING_MAX   (MQTT_OUTPUT_RINGBUF_SIZE / 4)
#define MQTT_UTK_BEACON_PORT    18830	/* car_link.py broadcasts here */
#define MQTT_UTK_BEACON_MAGIC_LEN 4	/* "C28B" */

#if (MQTT_UTK_BACKOFF_MAX > PRV_MQTT_BACKOFF_MAX) || \
    (MQTT_UTK_PEND_TRIES_MAX > PRV_MQTT_PEND_TRIES_MAX) || \
    (APP_MQ_PAYLOAD_MAX > PRV_MQTT_PEND_LEN_MAX)
#error "PRV_MQTT fields no longer hold the radio state: widen them"
#endif

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * longest command (2 bytes packed, 15 bytes baseline); anything longer is
 * truncated and flagged rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

/* PRV_MQTT: BACKOFF (retry in 1 << n s), PEND_TRIES, PEND_LEN (0 = none)
 * and PEND_TYPE of the one pending payload, which is kept in pend_buf
 * until it is sent or refused twice. */
static u16_t		prv_mqtt;
static u8_t		pend_buf[APP_MQ_PAYLOAD_MAX];

/* Last-will payload; must outlive mqtt_client_connect(). */
static u8_t		will_buf[MQTT_UTK_AVAIL_MAX];

static u8_t		log_buf[MQTT_UTK_LOG_MAX];

/* Broker beacon: the last address heard, and whether it is new (set by
 * the UDP callback, handled by mqtt_utk_start() in the poll loop, which
 * may print). All on the radio task. */
static struct udp_pcb	*beacon_pcb;
static ip_addr_t	beacon_addr;
static u8_t		beacon_heard;
static u8_t		beacon_new;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

static void drain_timeout(void *arg);
static void retry_arm(void);

static u32_t mq_get(u32_t pos, u32_t bits)
{
	return app_bits_get(prv_mqtt, pos, bits);
}

static void mq_put(u32_t pos, u32_t bits, u32_t value)
{
	prv_mqtt = (u16_t)app_bits_put(prv_mqtt, pos, bits, value);
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;
}

static int topic_is_cmd(void)
{
	const char	*cmd = mqtt_bridge_cmd_topic();
	u32_t		i;

	for(i = 0; cmd[i] != '\0' && rx_topic[i] == cmd[i]; i++) {
	}
	return (cmd[i] == '\0' && rx_topic[i] == '\0');
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	/* Raw bytes: a command payload is binary in the packed build. */
	for(i = 0; i < len; i++) {
		if(rx_len < MQTT_UTK_RX_DATA_MAX) {
			rx_data[rx_len++] = data[i];
		} else {
			rx_truncated = 1;
		}
	}

	/* The bridge validates, rate-limits and posts to the mission queue.
	 * A truncated payload is still handed over: its length is wrong, so
	 * the bridge rejects and counts it. */
	if((flags & MQTT_DATA_FLAG_LAST) != 0 && topic_is_cmd()) {
		mqtt_bridge_post_command(rx_data, rx_len);
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	/* No console output from callbacks (B1.2.5g); a failed subscribe
	 * shows up as remote commands having no effect. */
	(void)arg;
	(void)result;
}

/* QoS 1 publishes report here once the broker answers or the request
 * times out; a late failure is a lost payload, counted like a refusal. */
static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		mqtt_bridge_count_publish_drop();
	}
}

/* Bytes waiting in the client's output ring: lwIP keeps put and get
 * within 0..MQTT_OUTPUT_RINGBUF_SIZE-1 (same sum as mqtt_ringbuf_len()). */
static u32_t ring_used(void)
{
	u16_t	put = client->output.put;
	u16_t	get = client->output.get;

	if(put >= get) {
		return (u32_t)(put - get);
	}
	return (u32_t)MQTT_OUTPUT_RINGBUF_SIZE - get + put;
}

static void pend_clear(void)
{
	mq_put(PRV_MQTT_PEND_LEN_POS, PRV_MQTT_PEND_LEN_BITS, 0);
	mq_put(PRV_MQTT_PEND_TRIES_POS, PRV_MQTT_PEND_TRIES_BITS, 0);
}

/* Every 50 ms while the chain lives: up to two publishes, one pending
 * payload kept across calls. A payload refused twice (ERR_MEM: request
 * slots or output buffer full) is dropped and counted. */
static void drain_timeout(void *arg)
{
	const char	*topic;
	u8_t		qos;
	bool		retain;
	u8_t		type;
	u32_t		len;
	u32_t		tries;
	err_t		err;
	int		n;
	u8_t		k11_blocked = 0;

	(void)arg;

	for(n = 0; n < MQTT_UTK_DRAIN_BURST
		   && mqtt_client_is_connected(client); n++) {
		len = mq_get(PRV_MQTT_PEND_LEN_POS, PRV_MQTT_PEND_LEN_BITS);
		if(len == 0) {
			len = mqtt_bridge_take(pend_buf, sizeof(pend_buf),
					       &type);
			if(len == 0) {
				break;
			}
			/* a new payload starts with no tries (S1-U15) */
			mq_put(PRV_MQTT_PEND_LEN_POS, PRV_MQTT_PEND_LEN_BITS,
			       len);	/* <= APP_MQ_PAYLOAD_MAX */
			mq_put(PRV_MQTT_PEND_TYPE_POS, PRV_MQTT_PEND_TYPE_BITS,
			       type);
			mq_put(PRV_MQTT_PEND_TRIES_POS,
			       PRV_MQTT_PEND_TRIES_BITS, 0);
		}
		type = (u8_t)mq_get(PRV_MQTT_PEND_TYPE_POS,
				    PRV_MQTT_PEND_TYPE_BITS);
		mqtt_bridge_route(type, &topic, &qos, &retain);
		err = mqtt_publish(client, topic, pend_buf, (u16_t)len, qos,
				   retain? 1: 0, publish_done_cb, NULL);
		if(err == ERR_OK) {
			pend_clear();
		} else {
			tries = mq_get(PRV_MQTT_PEND_TRIES_POS,
				       PRV_MQTT_PEND_TRIES_BITS) + 1;
			mq_put(PRV_MQTT_PEND_TRIES_POS,
			       PRV_MQTT_PEND_TRIES_BITS, tries);
			if(tries >= MQTT_UTK_PEND_TRIES_MAX) {
				mqtt_bridge_count_publish_drop();
				pend_clear();
			}
			k11_blocked = 1;
			break;
		}
	}

	/* Mirrored console lines after the binary telemetry, and only when
	 * the output buffer took it: telemetry always goes first. A line the
	 * buffer refuses is lost (debug only; not counted in DROPS). */
	for(n = 0; n < MQTT_UTK_LOG_BURST && !k11_blocked
		   && mqtt_client_is_connected(client)
		   && ring_used() <= MQTT_UTK_LOG_RING_MAX; n++) {
		len = mqtt_bridge_take_log(log_buf, sizeof(log_buf));
		if(len == 0) {
			break;
		}
		mqtt_bridge_route(APP_MQ_TYPE_LOG, &topic, &qos, &retain);
		err = mqtt_publish(client, topic, log_buf, (u16_t)len, qos,
				   retain? 1: 0, NULL, NULL);
		if(err != ERR_OK) {
			break;
		}
	}

	/* Re-arms itself every call; connection_cb() cancels any old chain
	 * before starting a new one, so exactly one chain is alive. */
	sys_timeout(MQTT_UTK_DRAIN_MS, drain_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

/* Reconnect after 1, 2, 4, 8, 8, ... s. connecting stays set until the
 * timer fires, so the poll loop's mqtt_utk_start() cannot skip the wait. */
static void retry_arm(void)
{
	u32_t	exp = mq_get(PRV_MQTT_BACKOFF_POS, PRV_MQTT_BACKOFF_BITS);
	u32_t	delay_ms = (u32_t)MQTT_UTK_RETRY_BASE_MS << exp;

	connecting = 1;
	sys_untimeout(retry_timeout, NULL);
	sys_timeout(delay_ms, retry_timeout, NULL);
	if(exp < MQTT_UTK_BACKOFF_MAX) {
		mq_put(PRV_MQTT_BACKOFF_POS, PRV_MQTT_BACKOFF_BITS, exp + 1);
	}
	tm_printf((UB*)"mqtt: retry in %d ms\n", (int)delay_ms);
}

static void publish_online(void)
{
	u8_t		buf[MQTT_UTK_AVAIL_MAX];
	u32_t		len;
	const char	*topic;
	u8_t		qos;
	bool		retain;

	len = mqtt_bridge_availability(true, buf, sizeof(buf));
	mqtt_bridge_route(APP_MQ_TYPE_ONLINE, &topic, &qos, &retain);
	if(len > 0) {
		(void)mqtt_publish(client, topic, buf, (u16_t)len, qos,
				   retain? 1: 0, publish_done_cb, NULL);
	}
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		mq_put(PRV_MQTT_BACKOFF_POS, PRV_MQTT_BACKOFF_BITS, 0);

		/* Nagle off (2026-11-03 on-car capture): with it on, one lost
		 * segment held every later publish back, so no duplicate ACKs
		 * came and only the 1-2 s retransmit timer recovered it (STATE
		 * gaps of 1.3-2.1 s on a phone hotspot with ~3 % loss). Sent at
		 * once, later publishes draw duplicate ACKs and fast retransmit
		 * repairs the loss sooner. Telemetry is a few small packets a
		 * second, so the extra headers cost nothing. */
		if(client->conn != NULL) {
			altcp_nagle_disable(client->conn);
		}

		tm_printf((UB*)"mqtt: connected to %s port %d\n",
			  ipaddr_ntoa(&broker_addr), (int)MQTT_BROKER_PORT);
		mqtt_bridge_link_changed(true);
		publish_online();

		/* Callbacks must be registered before subscribing, or the first
		 * delivery has nowhere to go. Commands come in at QoS 1. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		(void)mqtt_subscribe(client, mqtt_bridge_cmd_topic(), 1 /* qos */,
				     subscribe_done_cb, NULL);

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating drain, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(drain_timeout, NULL);
		sys_timeout(MQTT_UTK_DRAIN_MS, drain_timeout, NULL);
	} else {
		/* lwIP does not reconnect by itself, neither after a dropped
		 * keepalive nor after a refused connection, so retry here on
		 * the back-off delay: a broker that keeps rejecting cannot turn
		 * this into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d\n", (int)status);
		mqtt_bridge_link_changed(false);
		retry_arm();
	}
}

/* A datagram on the beacon port: if it starts with "C28B", its sender is
 * the laptop running the broker. Only records it; mqtt_utk_start() acts. */
static void beacon_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
			   const ip_addr_t *addr, u16_t port)
{
	char	magic[MQTT_UTK_BEACON_MAGIC_LEN];

	(void)arg;
	(void)pcb;
	(void)port;

	if(p == NULL) {
		return;
	}
	if(addr != NULL && IP_IS_V4(addr)
	   && pbuf_copy_partial(p, magic, sizeof(magic), 0) == sizeof(magic)
	   && magic[0] == 'C' && magic[1] == '2' && magic[2] == '8'
	   && magic[3] == 'B') {
		if(!beacon_heard || !ip_addr_cmp(&beacon_addr, addr)) {
			ip_addr_copy(beacon_addr, *addr);
			beacon_heard = 1;
			beacon_new = 1;
		}
	}
	pbuf_free(p);
}

static void beacon_listen(void)
{
	if(beacon_pcb != NULL) {
		return;
	}
	beacon_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
	if(beacon_pcb == NULL) {
		tm_printf((UB*)"mqtt: no UDP pcb for the broker beacon\n");
		return;
	}
	if(udp_bind(beacon_pcb, IP4_ADDR_ANY, MQTT_UTK_BEACON_PORT) != ERR_OK) {
		udp_remove(beacon_pcb);
		beacon_pcb = NULL;
		tm_printf((UB*)"mqtt: beacon port bind failed\n");
		return;
	}
	udp_recv(beacon_pcb, beacon_recv_cb, NULL);
	tm_printf((UB*)"mqtt: listening for the broker beacon on UDP %d\n",
		  (int)MQTT_UTK_BEACON_PORT);
}

/* A new beacon address while not connected: abort a connect still trying
 * the old address (mqtt_disconnect() does not call connection_cb) and
 * skip the back-off, so the next mqtt_utk_start() tries the new one. */
static void beacon_service(void)
{
	if(!beacon_new) {
		return;
	}
	beacon_new = 0;
	tm_printf((UB*)"mqtt: broker beacon from %s\n",
		  ipaddr_ntoa(&beacon_addr));
	if(client != NULL && mqtt_client_is_connected(client)) {
		return;			/* keep a working connection */
	}
	if(ip_addr_cmp(&beacon_addr, &broker_addr)) {
		return;			/* already trying that address */
	}
	if(client != NULL) {
		mqtt_disconnect(client);
	}
	sys_untimeout(retry_timeout, NULL);
	mq_put(PRV_MQTT_BACKOFF_POS, PRV_MQTT_BACKOFF_BITS, 0);
	connecting = 0;
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	beacon_listen();
	beacon_service();

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(beacon_heard) {
		ip_addr_copy(broker_addr, beacon_addr);
	} else {
		if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
			tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
			return;
		}
		IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);
	}
	tm_printf((UB*)"mqtt: connecting to %s%s\n", ipaddr_ntoa(&broker_addr),
		  beacon_heard? " (beacon)": " (wifi_secrets.h)");

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	/* Retained last will: the broker publishes "offline" for us if the
	 * link dies silently. lwIP 2.2.1 has will_msg_len, so the payload
	 * needs no NUL (the packed byte 0x31 would be fine either way). */
	{
		const char	*will_topic;
		u8_t		will_qos;
		bool		will_retain;
		u32_t		will_len;

		will_len = mqtt_bridge_availability(false, will_buf,
						    sizeof(will_buf));
		mqtt_bridge_route(APP_MQ_TYPE_OFFLINE, &will_topic, &will_qos,
				  &will_retain);
		if(will_len > 0) {
			client_info.will_topic = will_topic;
			client_info.will_msg = (const char *)will_buf;
			client_info.will_msg_len = (u8_t)will_len;
			client_info.will_qos = will_qos;
			client_info.will_retain = will_retain? 1: 0;
		}
	}

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		/* An immediate refusal takes the same back-off path as a
		 * refused connection (retry_arm keeps connecting set). */
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		retry_arm();
	}
}
