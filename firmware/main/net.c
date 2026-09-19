/* net.c - UDP rx/tx and sequence handling.
 *
 * W-5: UDP, never TCP, for motor commands - a retransmitted stale command is
 * worse than a dropped one.  The firmware never initiates: it answers and
 * reports, and telemetry goes back to whatever address last sent it a valid
 * command, so there is no configured laptop IP to get wrong.
 *
 * F-5: no retries, no reconnect backoff that delays the watchdog.  The Wi-Fi
 * reconnect path lives in its own event handler; while it runs, safety.c keeps
 * the watchdog tripped and the motors stopped, which is the correct behaviour.
 */

#include "net.h"

#include <string.h>

#include "app_config.h"
#include "drive.h"
#include "safety.h"
#include "tilt.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char *TAG = "net";

static int s_sock = -1;
static struct sockaddr_in s_peer;
static bool s_have_peer;

static net_command_t s_cmd;
static bool s_have_cmd;
static uint16_t s_last_seq;
static bool s_seq_valid;

static net_stats_t s_stats;
static uint32_t s_worst_loop_us;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------ */
/* Wi-Fi                                                              */
/* ------------------------------------------------------------------ */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Immediate retry, no backoff.  A backoff here would be a delay the
         * watchdog cannot see, and the rover is already stopped. */
        ESP_LOGW(TAG, "disconnected, reconnecting");
        s_have_peer = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip " IPSTR, IP2STR(&ev->ip_info.ip));
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, ROVER_WIFI_SSID, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, ROVER_WIFI_PASS, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    /* Latency beats throughput here.  Modem sleep adds tens of milliseconds of
     * jitter to a 30 Hz command stream and eats the watchdog's margin. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ------------------------------------------------------------------ */
/* Socket                                                             */
/* ------------------------------------------------------------------ */

void net_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    wifi_start();

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(ROVER_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(s_sock);
        s_sock = -1;
        return;
    }

    /* A short receive timeout keeps the rx task responsive to shutdown and
     * stops it parking forever on a dead link. */
    struct timeval tv = {.tv_sec = 0, .tv_usec = 100000};
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "listening on udp/%d, config %s", ROVER_CMD_PORT, CFG_HASH);
}

/* ------------------------------------------------------------------ */
/* Receive                                                            */
/* ------------------------------------------------------------------ */

void net_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];

    while (1) {
        struct sockaddr_in src;
        socklen_t srclen = sizeof(src);
        int n = recvfrom(s_sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &srclen);
        if (n <= 0) {
            continue;   /* timeout; safety.c owns the consequences */
        }

        portENTER_CRITICAL(&s_lock);
        s_stats.rx_total++;
        portEXIT_CRITICAL(&s_lock);

        if (n != CMD_SIZE) {
            portENTER_CRITICAL(&s_lock);
            s_stats.rx_bad_len++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }

        /* Split the version check out of proto_parse_command() only for the
         * statistic; the parse itself re-checks both. */
        if (buf[0] != PROTOCOL_VERSION) {
            portENTER_CRITICAL(&s_lock);
            s_stats.rx_bad_version++;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGW(TAG, "protocol version %u, expected %u - is protocol.py in sync?",
                     buf[0], PROTOCOL_VERSION);
            continue;
        }

        cmd_packet_t pkt;
        if (!proto_parse_command(buf, (size_t)n, &pkt)) {
            portENTER_CRITICAL(&s_lock);
            s_stats.rx_bad_crc++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }

        /* W-5.3: reject a non-advancing sequence.  This is what makes a stuck
         * sender indistinguishable from silence - it does NOT feed the
         * watchdog, so the motors stop. */
        bool fresh;
        portENTER_CRITICAL(&s_lock);
        fresh = !s_seq_valid || proto_seq_advanced(pkt.seq, s_last_seq);
        portEXIT_CRITICAL(&s_lock);

        if (!fresh) {
            portENTER_CRITICAL(&s_lock);
            s_stats.rx_stale_seq++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;

        portENTER_CRITICAL(&s_lock);
        s_last_seq = pkt.seq;
        s_seq_valid = true;
        s_cmd.left = pkt.left;
        s_cmd.right = pkt.right;
        s_cmd.tilt_deg = (float)pkt.tilt_deg_x10 / 10.0f;
        s_cmd.enable = (pkt.flags & FLAG_ENABLE) != 0;
        s_cmd.tilt_release = (pkt.flags & FLAG_TILT_RELEASE) != 0;
        s_cmd.zero_tilt = (pkt.flags & FLAG_ZERO_TILT) != 0;
        s_cmd.seq = pkt.seq;
        s_cmd.rx_ms = now_ms;
        s_have_cmd = true;
        s_stats.rx_accepted++;
        s_peer = src;
        s_have_peer = true;
        portEXIT_CRITICAL(&s_lock);

        safety_feed(now_ms);
    }
}

bool net_get_command(net_command_t *out)
{
    bool have;
    portENTER_CRITICAL(&s_lock);
    have = s_have_cmd;
    *out = s_cmd;
    portEXIT_CRITICAL(&s_lock);
    return have;
}

void net_get_stats(net_stats_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_lock);
}

void net_report_loop_us(uint32_t us)
{
    portENTER_CRITICAL(&s_lock);
    if (us > s_worst_loop_us) {
        s_worst_loop_us = us;
    }
    portEXIT_CRITICAL(&s_lock);
}

/* ------------------------------------------------------------------ */
/* Telemetry                                                          */
/* ------------------------------------------------------------------ */

void net_telemetry_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, period);

        if (s_sock < 0 || !s_have_peer) {
            continue;   /* nothing to answer; the firmware never initiates */
        }

        safety_status_t st;
        safety_get(&st);
        tilt_state_t tilt;
        tilt_get_state(&tilt);

        net_command_t cmd;
        bool have = net_get_command(&cmd);
        bool enabled = have && cmd.enable && safety_motion_allowed(cmd.enable, &st);

        uint32_t worst;
        uint16_t seq_echo;
        portENTER_CRITICAL(&s_lock);
        worst = s_worst_loop_us;
        s_worst_loop_us = 0;          /* W-5.5: worst since the LAST report */
        seq_echo = s_last_seq;
        s_stats.tx_telemetry++;
        portEXIT_CRITICAL(&s_lock);

        tlm_packet_t pkt = {
            .version = PROTOCOL_VERSION,
            .status = safety_status_bits(enabled, tilt.moving),
            .seq_echo = seq_echo,
            .tilt_steps = tilt.position,
            .range_mm = st.range_mm,
            .vbat_mv = st.vbat_mv,
            .loop_us = (uint16_t)(worst > 0xFFFF ? 0xFFFF : worst),
        };

        uint8_t buf[TLM_SIZE];
        proto_build_telemetry(&pkt, buf);

        struct sockaddr_in dst = s_peer;
        dst.sin_port = htons(ROVER_TLM_PORT);
        sendto(s_sock, buf, sizeof(buf), 0,
               (struct sockaddr *)&dst, sizeof(dst));
    }
}
