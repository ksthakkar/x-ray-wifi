#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "credentials.h"

// --- CONFIGURATION ---
// Array sizes must be true compile-time constants in C, so those two live in
// an enum; everything else is a plain const (no #define).
enum
{
    CSI_MAX_FRAME_LEN = 384, // max raw CSI payload length ESP-IDF may report
    TX_BUF_LEN = 512,        // scratch buffer for header + CSI payload
    PRESENCE_MAX_SC = 192,   // CSI_MAX_FRAME_LEN / 2 I/Q pairs
    PRESENCE_BAR_WIDTH = 40, // serial bar-graph width, in characters
};

// Presence smoke test tuning.
static const float PRESENCE_BASELINE_ALPHA = 0.02f; // slow: tracks the empty room
static const float PRESENCE_SMOOTH_ALPHA = 0.25f;   // fast: smooths frame jitter
static const uint16_t PRESENCE_MIN_SC = 8;          // ignore implausibly short frames
static const uint32_t PRESENCE_WARMUP_FRAMES = 100; // let the baseline settle
static const uint32_t PRESENCE_LOG_EVERY = 10;      // ~5 Hz at the 50 Hz cap
static const float PRESENCE_BAR_SCALE = 0.8f;       // score 50 fills the bar

static const char *TARGET_IP = CSI_TARGET_IP;
static const uint16_t TARGET_PORT = CSI_TARGET_PORT;
static const uint8_t NODE_ID = 1;
static const uint32_t ADR018_MAGIC = 0xC5110001;
static const uint8_t NUM_ANTENNAS = 1;
static const uint32_t WIFI_CHANNEL_FREQ_MHZ = 2412; // channel 1, 2.4 GHz
static const UBaseType_t CSI_QUEUE_DEPTH = 10; // frames
static const uint32_t CSI_TX_THROTTLE_MS = 20; // ~50 Hz cap
static const uint32_t CSI_TX_TASK_STACK_WORDS = 4096;
static const UBaseType_t CSI_TX_TASK_PRIORITY = 5;

// Diagnostics: how often to dump the running health report, and how often to
// repeat an identical sendto() error. Bursty errno spam at 50 Hz is useless and
// it pushes the interesting lines out of the scrollback, so identical errors
// collapse into a periodic count.
static const uint32_t STATS_REPORT_MS = 5000;
static const uint32_t SEND_ERR_LOG_EVERY = 100;
// ---------------------

// --- RUNTIME COUNTERS (diagnostics only) ---
static struct
{
    uint32_t csi_frames;      // frames delivered by the Wi-Fi driver
    uint32_t csi_oversized;   // dropped: len > CSI_MAX_FRAME_LEN
    uint32_t queue_dropped;   // dropped: queue full (TX task too slow)
    uint32_t tx_ok;           // sendto() succeeded
    uint32_t tx_err;          // sendto() failed (any errno)
    uint32_t tx_err_nomem;    // sendto() failed with ENOMEM (errno 12)
    uint32_t no_socket;       // skipped: socket not up yet
    uint32_t last_errno;      // most recent sendto() errno
} s_stats;

static const char *TAG = "ESP32_CSI_NODE";
static int sock = -1;
static struct sockaddr_in dest_addr;
static uint32_t seq_num = 0;
static QueueHandle_t csi_queue = NULL;

typedef struct __attribute__((packed))
{
    uint32_t magic;           // 0xC5110001
    uint8_t node_id;          // Node ID
    uint8_t num_antennas;     // Rx antennas (1)
    uint16_t num_subcarriers; // Subcarrier I/Q count
    uint32_t freq_mhz;        // Channel frequency
    uint32_t sequence;        // Sequence counter
    int8_t rssi;              // Signal strength (dBm)
    int8_t noise_floor;       // Noise floor
    uint16_t motion_q8;       // Presence score, fixed-point (score * 256), 0xFFFF = warming up
} adr018_header_t;

typedef struct
{
    uint16_t len;
    int8_t rssi;
    int8_t noise_floor;
    uint8_t buf[CSI_MAX_FRAME_LEN];
} csi_packet_t;

// --- PRESENCE SMOKE TEST ---
// Goal: prove a human near the antenna moves the needle, before building the
// full pipeline. Raw CSI looks like noise because each frame's absolute scale
// and phase depend on the radio's AGC and packet timing, not on the room.
// Amplitude *shape* is far more stable, so we measure how far the current
// frame's shape sits from a slowly-adapting baseline. Empty room reads near 0;
// a hand over the board spikes hard.
static float s_baseline[PRESENCE_MAX_SC];
static uint16_t s_baseline_sc = 0; // subcarrier count the baseline was built for
static uint32_t s_baseline_frames = 0;
static float s_motion_smoothed = 0.0f;

// Returns a smoothed motion score (0..~100), or -1 while still warming up.
static float presence_update(const uint8_t *buf, uint16_t len)
{
    uint16_t n = len / 2;
    if (n < PRESENCE_MIN_SC)
        return -1.0f;
    if (n > PRESENCE_MAX_SC)
        n = PRESENCE_MAX_SC;

    // CSI buffer holds interleaved int8 pairs; ESP-IDF orders them (imag, real).
    float amp[PRESENCE_MAX_SC];
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        float im = (float)(int8_t)buf[2 * i];
        float re = (float)(int8_t)buf[2 * i + 1];
        amp[i] = sqrtf(re * re + im * im);
        sum += amp[i];
    }

    // Normalize by the frame's own mean. This is the step that makes the signal
    // legible: it cancels AGC gain jumps, which otherwise dwarf a human.
    if (sum < 1e-3f)
        return -1.0f;
    float mean = sum / (float)n;
    for (uint16_t i = 0; i < n; i++)
        amp[i] /= mean;

    // Subcarrier count varies by frame type (LLTF vs HT-LTF). Mixing types would
    // corrupt the baseline, so rebuild it when the width changes.
    if (n != s_baseline_sc)
    {
        s_baseline_sc = n;
        s_baseline_frames = 0;
        s_motion_smoothed = 0.0f;
        memcpy(s_baseline, amp, n * sizeof(float));
    }

    // Mean absolute deviation from baseline, scaled into a readable range.
    float dev = 0.0f;
    for (uint16_t i = 0; i < n; i++)
        dev += fabsf(amp[i] - s_baseline[i]);
    dev = (dev / (float)n) * 100.0f;

    for (uint16_t i = 0; i < n; i++)
        s_baseline[i] += PRESENCE_BASELINE_ALPHA * (amp[i] - s_baseline[i]);

    // Report nothing until the baseline settles, else startup reads as motion.
    if (s_baseline_frames < PRESENCE_WARMUP_FRAMES)
    {
        s_baseline_frames++;
        return -1.0f;
    }

    s_motion_smoothed += PRESENCE_SMOOTH_ALPHA * (dev - s_motion_smoothed);
    return s_motion_smoothed;
}

// Log a bar graph so motion is obvious by eye on the serial monitor.
static void presence_log(float score, uint16_t n_sc, int8_t rssi)
{
    char bar[PRESENCE_BAR_WIDTH + 1];
    int fill = (int)(score * PRESENCE_BAR_SCALE);
    if (fill > PRESENCE_BAR_WIDTH)
        fill = PRESENCE_BAR_WIDTH;
    if (fill < 0)
        fill = 0;
    memset(bar, '#', fill);
    memset(bar + fill, '.', PRESENCE_BAR_WIDTH - fill);
    bar[PRESENCE_BAR_WIDTH] = '\0';

    ESP_LOGI(TAG, "motion %6.2f |%s| sc=%u rssi=%d", score, bar, n_sc, rssi);
}

// Pack the score for the wire. 0xFFFF is the "warming up" sentinel.
static uint16_t presence_to_q8(float score)
{
    if (score < 0.0f)
        return 0xFFFF;
    float q = score * 256.0f;
    if (q > 65534.0f)
        q = 65534.0f;
    return (uint16_t)q;
}

// --- CSI ISR / CALLBACK (Non-blocking, enqueues packets) ---
static void wifi_csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || !csi_queue)
        return;

    s_stats.csi_frames++;

    // Drop oversized frames safely
    if (info->len > CSI_MAX_FRAME_LEN)
    {
        s_stats.csi_oversized++;
        return;
    }

    csi_packet_t pkt;
    pkt.len = info->len;
    pkt.rssi = info->rx_ctrl.rssi;
    pkt.noise_floor = info->rx_ctrl.noise_floor;
    memcpy(pkt.buf, info->buf, info->len);

    // Non-blocking queue send (drops packet if queue is full instead of stalling Wi-Fi task)
    if (xQueueSend(csi_queue, &pkt, 0) != pdTRUE)
        s_stats.queue_dropped++;
}

// Human-readable name for the errnos this socket path realistically hits, so
// the log says what to do instead of leaving a bare number to look up.
static const char *errno_hint(int e)
{
    switch (e)
    {
    case ENOMEM:
        return "ENOMEM: lwIP out of TX buffers - raise UDP TX buffers or slow the send rate";
    case ENOBUFS:
        return "ENOBUFS: lwIP buffer pool exhausted - same fix as ENOMEM";
    case EHOSTUNREACH:
        return "EHOSTUNREACH: no route - check target IP is on this subnet";
    case ENETUNREACH:
        return "ENETUNREACH: network down - Wi-Fi dropped?";
    case EBADF:
        return "EBADF: socket closed underneath us";
    case EAGAIN:
        return "EAGAIN: socket would block";
    default:
        return "see lwIP errno list";
    }
}

// Periodic health report. This is the main diagnostic: it separates "the radio
// isn't giving me CSI" from "CSI is fine but the network is dropping it", which
// look identical from a silent dashboard.
static void log_stats(void)
{
    uint32_t heap = esp_get_free_heap_size();
    uint32_t total = s_stats.tx_ok + s_stats.tx_err;
    uint32_t pct = total ? (s_stats.tx_ok * 100u) / total : 0u;

    ESP_LOGI(TAG, "---- stats ----");
    ESP_LOGI(TAG, "  csi:  %lu frames, %lu oversized, %lu queue-dropped",
             (unsigned long)s_stats.csi_frames,
             (unsigned long)s_stats.csi_oversized,
             (unsigned long)s_stats.queue_dropped);
    ESP_LOGI(TAG, "  tx:   %lu ok, %lu err (%lu ENOMEM), %lu%% delivered, %lu no-socket",
             (unsigned long)s_stats.tx_ok,
             (unsigned long)s_stats.tx_err,
             (unsigned long)s_stats.tx_err_nomem,
             (unsigned long)pct,
             (unsigned long)s_stats.no_socket);
    ESP_LOGI(TAG, "  heap: %lu bytes free, min-ever %lu",
             (unsigned long)heap,
             (unsigned long)esp_get_minimum_free_heap_size());
    if (s_stats.last_errno)
        ESP_LOGW(TAG, "  last sendto errno %lu (%s)",
                 (unsigned long)s_stats.last_errno, errno_hint((int)s_stats.last_errno));

    // ENOMEM here almost always means lwIP's UDP TX buffers can't keep up with
    // the CSI rate, not a leak. Call it out so it isn't read as a heap problem.
    if (s_stats.tx_err_nomem > 0 && s_stats.tx_err_nomem >= s_stats.tx_ok / 4)
        ESP_LOGW(TAG, "  ^ high ENOMEM rate: send rate exceeds lwIP TX capacity. "
                      "Raise CSI_TX_THROTTLE_MS or CONFIG_LWIP_UDP_SNDBUF / mbuf counts.");
}

// --- WORKER TASK (Handles network socket transmission) ---
static void csi_tx_task(void *pvParameters)
{
    csi_packet_t pkt;
    uint8_t tx_buf[TX_BUF_LEN];
    uint32_t log_ctr = 0;
    uint32_t err_ctr = 0;
    uint32_t next_stats_ms = STATS_REPORT_MS;

    while (1)
    {
        if (xQueueReceive(csi_queue, &pkt, portMAX_DELAY) == pdTRUE)
        {
            uint16_t num_subcarriers = pkt.len / 2;

            // Runs on every frame (the baseline needs them all) but only logs
            // periodically, so the serial monitor stays readable.
            float motion = presence_update(pkt.buf, pkt.len);
            if (motion >= 0.0f && (log_ctr++ % PRESENCE_LOG_EVERY) == 0)
                presence_log(motion, num_subcarriers, pkt.rssi);

            // Emit the periodic report even while the socket is down, so a node
            // that never connects still reports whether CSI is arriving.
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (now_ms >= next_stats_ms)
            {
                next_stats_ms = now_ms + STATS_REPORT_MS;
                log_stats();
            }

            if (sock < 0)
            {
                s_stats.no_socket++;
                continue;
            }

            size_t payload_len = sizeof(adr018_header_t) + pkt.len;

            if (payload_len > sizeof(tx_buf))
            {
                ESP_LOGW(TAG, "payload %u > tx_buf %u, dropping",
                         (unsigned)payload_len, (unsigned)sizeof(tx_buf));
                continue;
            }

            adr018_header_t *hdr = (adr018_header_t *)tx_buf;
            hdr->magic = ADR018_MAGIC;
            hdr->node_id = NODE_ID;
            hdr->num_antennas = NUM_ANTENNAS;
            hdr->num_subcarriers = num_subcarriers;
            hdr->freq_mhz = WIFI_CHANNEL_FREQ_MHZ;
            hdr->sequence = seq_num++;
            hdr->rssi = pkt.rssi;
            hdr->noise_floor = pkt.noise_floor;
            hdr->motion_q8 = presence_to_q8(motion);

            memcpy(tx_buf + sizeof(adr018_header_t), pkt.buf, pkt.len);

            int err = sendto(sock, tx_buf, payload_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            // ENOMEM/ENOBUFS is transient: lwIP just has no free TX buffer this
            // instant. One short retry recovers most frames; without it a brief
            // buffer squeeze looks like a dead stream on the dashboard.
            if (err < 0 && (errno == ENOMEM || errno == ENOBUFS))
            {
                vTaskDelay(pdMS_TO_TICKS(2));
                err = sendto(sock, tx_buf, payload_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            }

            if (err < 0)
            {
                int e = errno;
                s_stats.tx_err++;
                s_stats.last_errno = (uint32_t)e;
                if (e == ENOMEM || e == ENOBUFS)
                    s_stats.tx_err_nomem++;

                // Rate-limited: at 50 Hz an unfiltered errno log floods the port
                // and hides everything else.
                if ((err_ctr++ % SEND_ERR_LOG_EVERY) == 0)
                    ESP_LOGE(TAG, "sendto failed: errno %d (%s) [%lu errors so far, %u bytes -> %s:%u]",
                             e, errno_hint(e), (unsigned long)s_stats.tx_err,
                             (unsigned)payload_len, TARGET_IP, (unsigned)TARGET_PORT);
            }
            else
            {
                s_stats.tx_ok++;
            }

            vTaskDelay(pdMS_TO_TICKS(CSI_TX_THROTTLE_MS));
        }
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Connecting to Wi-Fi SSID \"%s\"...", WIFI_SSID);
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Wi-Fi disconnected from SSID \"%s\" (reason=%d), retrying...", WIFI_SSID, disconn->reason);
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected. Got IP " IPSTR ", gateway " IPSTR ", netmask " IPSTR,
                 IP2STR(&evt->ip_info.ip), IP2STR(&evt->ip_info.gw), IP2STR(&evt->ip_info.netmask));

        int8_t rssi_now = 0;
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        {
            rssi_now = ap.rssi;
            ESP_LOGI(TAG, "AP \"%s\" on channel %u, RSSI %d dBm", (char *)ap.ssid, ap.primary, rssi_now);
        }

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "socket() failed: errno %d (%s) - no data will be sent",
                     errno, errno_hint(errno));
            return;
        }

        dest_addr.sin_addr.s_addr = inet_addr(TARGET_IP);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(TARGET_PORT);

        if (dest_addr.sin_addr.s_addr == INADDR_NONE)
        {
            ESP_LOGE(TAG, "CSI_TARGET_IP \"%s\" is not a valid IPv4 address", TARGET_IP);
        }

        // The target is reachable only if it shares this subnet (no route beyond
        // the gateway is configured). Flag a mismatch now rather than after a
        // few thousand silent failures.
        uint32_t my_ip = evt->ip_info.ip.addr;
        uint32_t mask = evt->ip_info.netmask.addr;
        if ((my_ip & mask) != (dest_addr.sin_addr.s_addr & mask))
        {
            ESP_LOGW(TAG, "Target %s is on a different subnet than the ESP32 - "
                          "packets will go via the gateway and may be dropped.", TARGET_IP);
        }

        // Promiscuous mode setup for reliable packet capturing
        wifi_promiscuous_filter_t filter = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_MGMT};
        esp_err_t r = esp_wifi_set_promiscuous_filter(&filter);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_promiscuous_filter failed: %s", esp_err_to_name(r));
        r = esp_wifi_set_promiscuous(true);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_promiscuous failed: %s", esp_err_to_name(r));

        wifi_csi_config_t csi_config = {
            .lltf_en = true,
            .htltf_en = true,
            .stbc_htltf2_en = false,
            .ltf_merge_en = true,
            .channel_filter_en = false,
            .manu_scale = false,
            .shift = false,
        };

        // Report each step rather than aborting on the first failure: knowing
        // which of these three failed tells you whether CSI is even possible.
        r = esp_wifi_set_csi_config(&csi_config);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_csi_config failed: %s", esp_err_to_name(r));
        r = esp_wifi_set_csi_rx_cb(wifi_csi_cb, NULL);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_csi_rx_cb failed: %s", esp_err_to_name(r));
        r = esp_wifi_set_csi(true);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_csi(true) failed: %s - no CSI frames will arrive", esp_err_to_name(r));
        else
            ESP_LOGI(TAG, "CSI capture enabled");

        esp_wifi_set_ps(WIFI_PS_NONE); // Disable sleep

        ESP_LOGI(TAG, "Streaming CSI -> %s:%u (UDP), header %u bytes",
                 TARGET_IP, (unsigned)TARGET_PORT, (unsigned)sizeof(adr018_header_t));
        ESP_LOGI(TAG, "Waiting for CSI frames; stats every %lums",
                 (unsigned long)STATS_REPORT_MS);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_LOGI(TAG, "=== x-ray-wifi CSI node ===");
    ESP_LOGI(TAG, "node_id=%u target=%s:%u throttle=%lums queue=%u frames",
             NODE_ID, TARGET_IP, (unsigned)TARGET_PORT,
             (unsigned long)CSI_TX_THROTTLE_MS, (unsigned)CSI_QUEUE_DEPTH);
    ESP_LOGI(TAG, "free heap at boot: %lu bytes", (unsigned long)esp_get_free_heap_size());

    csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_packet_t));
    if (csi_queue == NULL)
    {
        ESP_LOGE(TAG, "xQueueCreate failed (needed %u bytes) - out of heap, aborting",
                 (unsigned)(CSI_QUEUE_DEPTH * sizeof(csi_packet_t)));
        return;
    }

    // Spawn CSI TX Worker Task
    // ESP32-C3 is single-core, so this runs unpinned instead of on core 1.
    if (xTaskCreate(csi_tx_task, "csi_tx_task", CSI_TX_TASK_STACK_WORDS, NULL,
                    CSI_TX_TASK_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "xTaskCreate failed - out of heap, aborting");
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/***
docker stop ruview && docker rm ruview

docker run -d \
  --name ruview \
  --restart always \
  --net=host \
  -e RUVIEW_ALLOW_UNAUTHENTICATED=1 \
  -e SENSING_ALLOWED_HOSTS="192.168.1.51,192.168.1.51:8080,192.168.1.51:8765,localhost,localhost:8080,localhost:8765,127.0.0.1,*" \
  -e WDP_DISABLE_HOST_VALIDATION=1 \
  -e CSI_SOURCE=esp32 \
  --entrypoint /app/sensing-server \
  ruvnet/wifi-densepose:latest \
  --source esp32 \
  --bind-addr 0.0.0.0 \
  --udp-port 5005 \
  --http-port 8080 \
  --ws-port 8765
 *
 *
 */
