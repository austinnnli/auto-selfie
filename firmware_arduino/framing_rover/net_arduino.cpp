/* net_arduino.cpp - net.h implemented with WiFi.h and WiFiUDP.
 *
 * The ESP-IDF build uses firmware/main/net.c instead.  Same interface, same
 * rules, far less boilerplate: the Arduino core owns the event loop and NVS,
 * so joining a network is three lines instead of forty.
 *
 * The rules that matter are unchanged, because they are protocol rules and
 * not platform ones (W-5):
 *
 *   - UDP, never TCP.  A retransmitted stale command is worse than a dropped
 *     one.
 *   - The firmware never initiates.  Telemetry goes back to whatever address
 *     last sent a VALID command, so there is no laptop IP to configure wrong.
 *   - A non-advancing sequence does not feed the watchdog, which makes a
 *     stuck sender indistinguishable from silence.
 *   - No reconnect backoff that delays the watchdog (F-5).  While Wi-Fi is
 *     down the watchdog stays tripped and the motors stay stopped, which is
 *     the correct behaviour, so there is nothing to hurry.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_timer.h>

extern "C" {
#include "app_config.h"
#include "net.h"
#include "protocol.h"
#include "safety.h"
}

static WiFiUDP s_udp;
static IPAddress s_peer;
static uint16_t s_peer_port = 0;
static bool s_have_peer = false;

static net_command_t s_cmd;
static bool s_have_cmd = false;
static uint16_t s_last_seq = 0;
static bool s_seq_valid = false;

static net_stats_t s_stats;
static uint32_t s_worst_loop_us = 0;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------ */

extern "C" void net_init(void)
{
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    /* Latency beats throughput here.  Modem sleep adds tens of milliseconds
     * of jitter to a 30 Hz command stream and eats the watchdog's margin. */
    WiFi.setSleep(false);

    WiFi.begin(ROVER_WIFI_SSID, ROVER_WIFI_PASS);
    log_i("joining \"%s\"", ROVER_WIFI_SSID);

    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    s_udp.begin(ROVER_CMD_PORT);

    /* THIS is the address the laptop needs.  Put it in config.json as
     * net.firmware_host, or pass it as: python -m app.run --host <ip> */
    Serial.println();
    Serial.println("=======================================================");
    Serial.print("  rover IP: ");
    Serial.println(WiFi.localIP());
    Serial.printf("  listening on udp/%d, protocol v%d, config %s\n",
                  ROVER_CMD_PORT, PROTOCOL_VERSION, CFG_HASH);
    Serial.println("  put this IP in config.json -> net.firmware_host");
    Serial.println("=======================================================");
    Serial.println();
}

/* ------------------------------------------------------------------ */

extern "C" void net_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];

    while (true) {
        int size = s_udp.parsePacket();
        if (size <= 0) {
            /* No packet.  Yield rather than spin: safety.c owns the
             * consequences of silence and is already handling them. */
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        int n = s_udp.read(buf, sizeof(buf));

        portENTER_CRITICAL(&s_lock);
        s_stats.rx_total++;
        portEXIT_CRITICAL(&s_lock);

        if (n != CMD_SIZE) {
            portENTER_CRITICAL(&s_lock);
            s_stats.rx_bad_len++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }

        if (buf[0] != PROTOCOL_VERSION) {
            portENTER_CRITICAL(&s_lock);
            s_stats.rx_bad_version++;
            portEXIT_CRITICAL(&s_lock);
            log_w("protocol version %u, expected %u - is app/protocol.py in sync?",
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

        bool fresh;
        portENTER_CRITICAL(&s_lock);
        fresh = !s_seq_valid || proto_seq_advanced(pkt.seq, s_last_seq);
        portEXIT_CRITICAL(&s_lock);

        if (!fresh) {
            /* W-5.3.  Note this does NOT call safety_feed(), so a sender
             * stuck on one sequence number stops the motors exactly as
             * silence would. */
            portENTER_CRITICAL(&s_lock);
            s_stats.rx_stale_seq++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        IPAddress from = s_udp.remoteIP();

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
        portEXIT_CRITICAL(&s_lock);

        s_peer = from;
        s_peer_port = ROVER_TLM_PORT;
        s_have_peer = true;

        safety_feed(now_ms);
    }
}

extern "C" bool net_get_command(net_command_t *out)
{
    bool have;
    portENTER_CRITICAL(&s_lock);
    have = s_have_cmd;
    *out = s_cmd;
    portEXIT_CRITICAL(&s_lock);
    return have;
}

extern "C" void net_get_stats(net_stats_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_lock);
}

extern "C" void net_report_loop_us(uint32_t us)
{
    portENTER_CRITICAL(&s_lock);
    if (us > s_worst_loop_us) {
        s_worst_loop_us = us;
    }
    portEXIT_CRITICAL(&s_lock);
}

/* ------------------------------------------------------------------ */

extern "C" void net_telemetry_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last, period);

        if (!s_have_peer) {
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

        tlm_packet_t pkt;
        pkt.version = PROTOCOL_VERSION;
        pkt.status = safety_status_bits(enabled, tilt.moving);
        pkt.seq_echo = seq_echo;
        pkt.tilt_steps = tilt.position;
        pkt.range_mm = st.range_mm;
        pkt.vbat_mv = st.vbat_mv;
        pkt.loop_us = (uint16_t)(worst > 0xFFFF ? 0xFFFF : worst);
        pkt.crc16 = 0;

        uint8_t buf[TLM_SIZE];
        proto_build_telemetry(&pkt, buf);

        s_udp.beginPacket(s_peer, s_peer_port);
        s_udp.write(buf, sizeof(buf));
        s_udp.endPacket();
    }
}
