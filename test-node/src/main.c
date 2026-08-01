/*
 * x-ray-wifi CSI capture node.
 *
 * Deliberately a DUMB FIREHOSE: it does no signal processing at all. Every CSI
 * frame the radio delivers is timestamped, wrapped with as much per-frame
 * metadata as the driver exposes, and streamed over UDP. All analysis happens
 * off-board (see tools/csi_record.py and the analysis scripts).
 *
 * Rationale: on-device processing (baselines, thresholds, band-averaged scores)
 * throws away exactly the narrow-band, slow-timescale information that presence
 * detection needs, and costs CPU that can make the node drop frames. Capture
 * everything, decide later.
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_mac.h"
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
// Array sizes must be true compile-time constants in C, so those live in an
// enum; everything else is a plain const (no #define).
enum
{
    CSI_MAX_FRAME_LEN = 384, // max raw CSI payload length ESP-IDF may report
    TX_BUF_LEN = 512,        // scratch buffer for header + CSI payload
    // Deep queue: capture is bursty (frames arrive in clumps when traffic
    // bursts), and dropping frames biases the dataset. RAM is cheaper than
    // gaps, so buffer generously.
    CSI_QUEUE_DEPTH = 64,
};

static const char *TARGET_IP = CSI_TARGET_IP;
static const uint16_t TARGET_PORT = CSI_TARGET_PORT;
static const uint8_t NODE_ID = 1;
// Magic 0xC5110003: capture format v3 (v1 raw, v2 added on-device presence).
static const uint32_t ADR018_MAGIC = 0xC5110003;
static const uint8_t NUM_ANTENNAS = 1;
static const uint32_t CSI_TX_TASK_STACK_WORDS = 4096;
static const UBaseType_t CSI_TX_TASK_PRIORITY = 5;

// Diagnostics cadence.
static const uint32_t STATS_REPORT_MS = 5000;
static const uint32_t SEND_ERR_LOG_EVERY = 100;
static const uint32_t QUEUE_DROP_LOG_EVERY = 200;
// Warn when the queue high-water mark reaches this fraction of its depth.
static const uint32_t QUEUE_WARN_NUM = 3, QUEUE_WARN_DEN = 4; // 3/4 full
// ---------------------

static const char *TAG = "ESP32_CSI_NODE";
static int sock = -1;
static struct sockaddr_in dest_addr;
static uint32_t seq_num = 0;
static QueueHandle_t csi_queue = NULL;

// --- RUNTIME COUNTERS (diagnostics only) ---
static struct
{
    uint32_t csi_frames;    // frames delivered by the Wi-Fi driver
    uint32_t csi_oversized; // dropped: len > CSI_MAX_FRAME_LEN
    uint32_t queue_dropped; // dropped: queue full (TX task too slow)
    uint32_t tx_ok;         // sendto() succeeded
    uint32_t tx_err;        // sendto() failed (any errno)
    uint32_t tx_err_nomem;  // sendto() failed with ENOMEM (errno 12)
    uint32_t no_socket;     // skipped: socket not up yet
    uint32_t last_errno;    // most recent sendto() errno
    uint32_t queue_peak;    // high-water mark of queue occupancy
} s_stats;

/*
 * Capture header. Everything the ESP-IDF CSI callback exposes that could
 * plausibly matter for offline analysis is recorded, because a field not
 * captured is a field that cannot be recovered later.
 *
 * Notably included, and why:
 *   - timestamp_us: the driver's own frame timestamp. The true sample rate is
 *     variable and NOT the nominal 50 Hz; every frequency-domain analysis needs
 *     real timestamps, so this is the single most important added field.
 *   - rate/sig_mode/mcs/cwb/stbc/...: identify the frame TYPE. Subcarrier count
 *     and layout differ between non-HT and HT frames; mixing them corrupts any
 *     per-subcarrier statistic, so analysis must be able to group by type.
 *   - rx_state: the driver's own error/validity flag for the frame.
 *   - first_word_invalid: when set, the first CSI word is garbage (a known
 *     ESP32 quirk) and must be skipped.
 *   - mac: which transmitter the frame came from, so analysis can restrict to
 *     one link instead of blending several.
 */
typedef struct __attribute__((packed))
{
    uint32_t magic;           // 0xC5110003
    uint8_t node_id;          // Node ID
    uint8_t num_antennas;     // Rx antennas (1)
    uint16_t num_subcarriers; // Subcarrier I/Q pair count
    uint32_t sequence;        // Sequence counter (gaps => lost packets)
    uint64_t timestamp_us;    // Driver frame timestamp (microseconds)
    int8_t rssi;              // Signal strength (dBm)
    int8_t noise_floor;       // Noise floor (dBm)
    uint8_t channel;          // Primary channel
    uint8_t secondary_channel;// Secondary channel (HT40)
    uint8_t rate;             // Rate index
    uint8_t sig_mode;         // 0=non-HT, 1=HT, 3=VHT
    uint8_t mcs;              // MCS index
    uint8_t cwb;              // Channel bandwidth: 0=20MHz, 1=40MHz
    uint8_t smoothing;        // PHY smoothing flag
    uint8_t not_sounding;     // PHY not-sounding flag
    uint8_t aggregation;      // AMPDU aggregation flag
    uint8_t stbc;             // Space-time block coding
    uint8_t fec_coding;       // 0=BCC, 1=LDPC
    uint8_t sgi;              // Short guard interval
    uint8_t ampdu_cnt;        // AMPDU count
    uint8_t rx_state;         // Driver RX state / error flags
    uint8_t first_word_invalid; // 1 => discard the first CSI word
    uint8_t phy_variant;      // 0=legacy rx_ctrl fields, 1=HE (cur_bb_format/second)
    uint8_t mac[6];           // Transmitter MAC
    uint16_t csi_len;         // Raw CSI byte count that follows
} csi_capture_header_t;

typedef struct
{
    uint16_t len;
    uint64_t timestamp_us;
    wifi_pkt_rx_ctrl_t rx_ctrl;
    uint8_t mac[6];
    uint8_t first_word_invalid;
    uint8_t buf[CSI_MAX_FRAME_LEN];
} csi_packet_t;

// --- CSI CALLBACK (runs in the Wi-Fi task: must stay cheap) ---
static void wifi_csi_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
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
    // Take the timestamp here, as close to arrival as possible; queue latency
    // would otherwise smear the inter-frame intervals that analysis depends on.
    pkt.timestamp_us = (uint64_t)esp_timer_get_time();
    pkt.rx_ctrl = info->rx_ctrl;
    memcpy(pkt.mac, info->mac, sizeof(pkt.mac));
    pkt.first_word_invalid = info->first_word_invalid ? 1 : 0;
    memcpy(pkt.buf, info->buf, info->len);

    // Track occupancy before pushing: a high-water mark near the depth is the
    // early warning that we are close to losing frames.
    UBaseType_t waiting = uxQueueMessagesWaiting(csi_queue);
    if (waiting > s_stats.queue_peak)
        s_stats.queue_peak = (uint32_t)waiting;

    // Non-blocking: drop rather than stall the Wi-Fi task.
    if (xQueueSend(csi_queue, &pkt, 0) != pdTRUE)
    {
        s_stats.queue_dropped++;

        // Logging here is in the driver's context, so keep it cheap and rare.
        // First drop is logged immediately (it marks when overload began).
        if (s_stats.queue_dropped == 1)
            ESP_LOGW(TAG, "CSI QUEUE FULL - DROPPING FRAMES. Capture has gaps from here "
                          "(queue depth %u).", (unsigned)CSI_QUEUE_DEPTH);
        else if ((s_stats.queue_dropped % QUEUE_DROP_LOG_EVERY) == 0)
            ESP_LOGW(TAG, "CSI queue full: %lu frames dropped so far",
                     (unsigned long)s_stats.queue_dropped);
    }
}

// Human-readable name for the errnos this socket path realistically hits.
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

// Periodic health report. For a capture run the critical lines are the drop
// counters: a run with drops has gaps, and gaps bias any statistic computed
// from it, so they must be visible while recording rather than discovered later.
static void log_stats(void)
{
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
    ESP_LOGI(TAG, "  queue: peak %lu/%u used",
             (unsigned long)s_stats.queue_peak, (unsigned)CSI_QUEUE_DEPTH);
    ESP_LOGI(TAG, "  heap: %lu bytes free, min-ever %lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size());

    if (s_stats.queue_dropped > 0)
        ESP_LOGW(TAG, "  ^ %lu frames DROPPED - this capture has GAPS.",
                 (unsigned long)s_stats.queue_dropped);
    else if (s_stats.queue_peak * QUEUE_WARN_DEN >= (uint32_t)CSI_QUEUE_DEPTH * QUEUE_WARN_NUM)
        ESP_LOGW(TAG, "  ^ queue reached %lu/%u - close to dropping frames.",
                 (unsigned long)s_stats.queue_peak, (unsigned)CSI_QUEUE_DEPTH);

    if (s_stats.last_errno)
        ESP_LOGW(TAG, "  last sendto errno %lu (%s)",
                 (unsigned long)s_stats.last_errno, errno_hint((int)s_stats.last_errno));

    if (s_stats.tx_err_nomem > 0 && s_stats.tx_err_nomem >= s_stats.tx_ok / 4)
        ESP_LOGW(TAG, "  ^ high ENOMEM rate: send rate exceeds lwIP TX capacity. "
                      "Raise CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM.");

    // Peak reflects the last window, not all time.
    s_stats.queue_peak = 0;
}

// --- WORKER TASK (network transmission only; no processing) ---
static void csi_tx_task(void *pvParameters)
{
    (void)pvParameters;
    csi_packet_t pkt;
    uint8_t tx_buf[TX_BUF_LEN];
    uint32_t err_ctr = 0;
    uint32_t next_stats_ms = STATS_REPORT_MS;

    while (1)
    {
        if (xQueueReceive(csi_queue, &pkt, portMAX_DELAY) == pdTRUE)
        {
            // Report even while the socket is down, so a node that never
            // connects still shows whether CSI is arriving at all.
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

            size_t payload_len = sizeof(csi_capture_header_t) + pkt.len;
            if (payload_len > sizeof(tx_buf))
            {
                ESP_LOGW(TAG, "payload %u > tx_buf %u, dropping",
                         (unsigned)payload_len, (unsigned)sizeof(tx_buf));
                continue;
            }

            csi_capture_header_t *hdr = (csi_capture_header_t *)tx_buf;
            hdr->magic = ADR018_MAGIC;
            hdr->node_id = NODE_ID;
            hdr->num_antennas = NUM_ANTENNAS;
            hdr->num_subcarriers = pkt.len / 2;
            hdr->sequence = seq_num++;
            hdr->timestamp_us = pkt.timestamp_us;
            hdr->rssi = pkt.rx_ctrl.rssi;
            hdr->noise_floor = pkt.rx_ctrl.noise_floor;
            hdr->channel = pkt.rx_ctrl.channel;
            hdr->rate = pkt.rx_ctrl.rate;
            hdr->rx_state = pkt.rx_ctrl.rx_state;

            // rx_ctrl's PHY-descriptor fields differ by silicon: HE-capable
            // chips (C6/C5) expose cur_bb_format/second, while pre-HE chips
            // (C3/S3/ESP32) expose sig_mode/cwb/stbc/... Mirror the branch
            // ruview uses so one source builds for either. Fields absent on
            // this target are zero-filled and flagged via phy_variant, so the
            // analysis side knows which set is meaningful.
#if defined(CONFIG_SOC_WIFI_HE_SUPPORT)
            hdr->phy_variant = 1; // HE: cur_bb_format/second are valid
            hdr->sig_mode = pkt.rx_ctrl.cur_bb_format;
            hdr->secondary_channel = pkt.rx_ctrl.second;
            hdr->mcs = 0;
            hdr->cwb = 0;
            hdr->smoothing = 0;
            hdr->not_sounding = 0;
            hdr->aggregation = 0;
            hdr->stbc = 0;
            hdr->fec_coding = 0;
            hdr->sgi = 0;
            hdr->ampdu_cnt = 0;
#else
            hdr->phy_variant = 0; // legacy: sig_mode/cwb/stbc/... are valid
            hdr->sig_mode = pkt.rx_ctrl.sig_mode;
            hdr->secondary_channel = pkt.rx_ctrl.secondary_channel;
            hdr->mcs = pkt.rx_ctrl.mcs;
            hdr->cwb = pkt.rx_ctrl.cwb;
            hdr->smoothing = pkt.rx_ctrl.smoothing;
            hdr->not_sounding = pkt.rx_ctrl.not_sounding;
            hdr->aggregation = pkt.rx_ctrl.aggregation;
            hdr->stbc = pkt.rx_ctrl.stbc;
            hdr->fec_coding = pkt.rx_ctrl.fec_coding;
            hdr->sgi = pkt.rx_ctrl.sgi;
            hdr->ampdu_cnt = pkt.rx_ctrl.ampdu_cnt;
#endif
            hdr->first_word_invalid = pkt.first_word_invalid;
            memcpy(hdr->mac, pkt.mac, sizeof(hdr->mac));
            hdr->csi_len = pkt.len;

            memcpy(tx_buf + sizeof(csi_capture_header_t), pkt.buf, pkt.len);

            int err = sendto(sock, tx_buf, payload_len, 0,
                             (struct sockaddr *)&dest_addr, sizeof(dest_addr));

            // ENOMEM/ENOBUFS is transient: lwIP has no free TX buffer this
            // instant. One short retry recovers most frames.
            if (err < 0 && (errno == ENOMEM || errno == ENOBUFS))
            {
                vTaskDelay(pdMS_TO_TICKS(2));
                err = sendto(sock, tx_buf, payload_len, 0,
                             (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            }

            if (err < 0)
            {
                int e = errno;
                s_stats.tx_err++;
                s_stats.last_errno = (uint32_t)e;
                if (e == ENOMEM || e == ENOBUFS)
                    s_stats.tx_err_nomem++;

                if ((err_ctr++ % SEND_ERR_LOG_EVERY) == 0)
                    ESP_LOGE(TAG, "sendto failed: errno %d (%s) [%lu errors so far, %u bytes -> %s:%u]",
                             e, errno_hint(e), (unsigned long)s_stats.tx_err,
                             (unsigned)payload_len, TARGET_IP, (unsigned)TARGET_PORT);
            }
            else
            {
                s_stats.tx_ok++;
            }

            // NOTE: no throttle. For data capture we want every frame the radio
            // gives us; rate-limiting here would silently decimate the dataset
            // and alias any periodic signal (e.g. breathing) we hope to find.
        }
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Connecting to Wi-Fi SSID \"%s\"...", WIFI_SSID);
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Wi-Fi disconnected from SSID \"%s\" (reason=%d), retrying...",
                 WIFI_SSID, disconn->reason);
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected. Got IP " IPSTR ", gateway " IPSTR ", netmask " IPSTR,
                 IP2STR(&evt->ip_info.ip), IP2STR(&evt->ip_info.gw), IP2STR(&evt->ip_info.netmask));

        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            ESP_LOGI(TAG, "AP \"%s\" on channel %u, RSSI %d dBm",
                     (char *)ap.ssid, ap.primary, ap.rssi);

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
            ESP_LOGE(TAG, "CSI_TARGET_IP \"%s\" is not a valid IPv4 address", TARGET_IP);

        uint32_t my_ip = evt->ip_info.ip.addr;
        uint32_t mask = evt->ip_info.netmask.addr;
        if ((my_ip & mask) != (dest_addr.sin_addr.s_addr & mask))
            ESP_LOGW(TAG, "Target %s is on a different subnet than the ESP32 - "
                          "packets will go via the gateway and may be dropped.", TARGET_IP);

        // Promiscuous mode for reliable packet capturing
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
            .manu_scale = false, // no driver scaling: keep the raw values
            .shift = false,
        };

        r = esp_wifi_set_csi_config(&csi_config);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_csi_config failed: %s", esp_err_to_name(r));
        r = esp_wifi_set_csi_rx_cb(wifi_csi_cb, NULL);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_csi_rx_cb failed: %s", esp_err_to_name(r));
        r = esp_wifi_set_csi(true);
        if (r != ESP_OK)
            ESP_LOGE(TAG, "set_csi(true) failed: %s - no CSI frames will arrive",
                     esp_err_to_name(r));
        else
            ESP_LOGI(TAG, "CSI capture enabled");

        esp_wifi_set_ps(WIFI_PS_NONE); // Disable sleep

        ESP_LOGI(TAG, "Streaming raw CSI -> %s:%u (UDP), header %u bytes, magic 0x%08lX",
                 TARGET_IP, (unsigned)TARGET_PORT,
                 (unsigned)sizeof(csi_capture_header_t), (unsigned long)ADR018_MAGIC);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_LOGI(TAG, "=== x-ray-wifi CSI capture node (raw, no processing) ===");
    ESP_LOGI(TAG, "node_id=%u target=%s:%u queue=%u frames header=%u bytes",
             NODE_ID, TARGET_IP, (unsigned)TARGET_PORT,
             (unsigned)CSI_QUEUE_DEPTH, (unsigned)sizeof(csi_capture_header_t));
    ESP_LOGI(TAG, "free heap at boot: %lu bytes", (unsigned long)esp_get_free_heap_size());

    csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_packet_t));
    if (csi_queue == NULL)
    {
        ESP_LOGE(TAG, "xQueueCreate failed (needed %u bytes) - out of heap, aborting",
                 (unsigned)(CSI_QUEUE_DEPTH * sizeof(csi_packet_t)));
        return;
    }

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
