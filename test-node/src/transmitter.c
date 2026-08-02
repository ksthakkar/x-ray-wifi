/*
 * x-ray-wifi CSI TRANSMITTER.
 *
 * Broadcasts a small ESP-NOW frame at a fixed rate. Its only job is to be a
 * steady, known source of OFDM frames so the receivers' CSI engines fire at a
 * controlled rate instead of depending on ambient Wi-Fi traffic.
 *
 * Why this exists
 * ---------------
 * With an access point as the only source, CSI frames arrive when packets happen
 * to arrive: measured ~20 Hz, drifting between 15 and 23 Hz across experimental
 * conditions. That is a confound -- a rate difference between conditions can
 * look exactly like a presence signal. A dedicated transmitter fixes the rate.
 *
 * It also makes cross-receiver comparison meaningful: every receiver measures
 * THE SAME broadcast through a different path, so a difference between receivers
 * is geometry rather than "they happened to hear different frames".
 *
 * Why ESP-NOW broadcast
 * ---------------------
 * Connectionless (no association, no DHCP), one send reaches every receiver at
 * once, works on a fixed channel, and the frames are OFDM so they drive the CSI
 * engine. Mirrors the approach in the ruview reference node.
 *
 * Note it is UNACKNOWLEDGED: frames will be lost, and loss will correlate with
 * body position (a body blocking a link causes both attenuation and loss). The
 * sequence number in each beacon lets receivers measure per-link delivery ratio,
 * turning that confound into a feature rather than a hidden bias.
 */
#include "role.h"

#if CSI_ROLE == CSI_ROLE_TX

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_idf_version.h" // ESP_IDF_VERSION / ESP_IDF_VERSION_VAL for the cb guard
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "credentials.h"

// Beacon rate. 50 Hz gives Nyquist 25 Hz -- far above breathing (0.1-0.5 Hz) and
// heart rate (0.8-2 Hz), with headroom for lost frames. Higher rates risk
// saturating the receivers' queues and the channel.
#ifndef CSI_TX_INTERVAL_MS
#define CSI_TX_INTERVAL_MS 20
#endif

// Must match the receivers' channel exactly: CSI only appears for frames the
// radio is actually tuned to.
#ifndef CSI_CHANNEL
#define CSI_CHANNEL 1
#endif

static const char *TAG = "CSI_TX";
static const uint32_t BEACON_MAGIC = 0xC5117800;

// Payload is deliberately small and CONSTANT except for the counters: a varying
// payload length would change frame duration and perturb the very channel
// measurement we are trying to keep stable.
typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t sequence;    // gaps at a receiver == frames lost on that link
    uint64_t tx_time_us;  // transmitter clock; NOT synchronised to receivers
    uint8_t pad[16];      // keeps the frame a fixed, non-trivial size
} csi_beacon_t;

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint32_t s_seq;
static uint32_t s_sent, s_failed;

/*
 * ESP-IDF changed esp_now_send_cb_t from
 *     void (*)(const uint8_t *mac, esp_now_send_status_t)
 * to
 *     void (*)(const esp_now_send_info_t *tx_info, esp_now_send_status_t)
 *
 * The guard has to be the full version triple, not ESP_IDF_VERSION_MAJOR:
 * Espressif backported the new signature to v5.5, where esp_now_send_info_t is
 * a typedef of wifi_tx_info_t. (Same fix as the ruview reference node.)
 *
 * The body is identical either way -- only `status` is used, and broadcast is
 * never acked, so "failure" here only reflects local queueing.
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void on_send(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status != ESP_NOW_SEND_SUCCESS)
        s_failed++;
}
#else
static void on_send(const uint8_t *mac, esp_now_send_status_t status)
{
    (void)mac;
    if (status != ESP_NOW_SEND_SUCCESS)
        s_failed++;
}
#endif

static void tx_task(void *arg)
{
    (void)arg;
    uint32_t next_report = 5000;

    while (1)
    {
        csi_beacon_t b = {
            .magic = BEACON_MAGIC,
            .sequence = s_seq++,
            .tx_time_us = (uint64_t)esp_timer_get_time(),
        };
        memset(b.pad, 0xA5, sizeof(b.pad));

        esp_err_t r = esp_now_send(BROADCAST_MAC, (const uint8_t *)&b, sizeof(b));
        if (r == ESP_OK)
            s_sent++;
        else
            s_failed++;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms >= next_report)
        {
            next_report = now_ms + 5000;
            ESP_LOGI(TAG, "sent %lu, failed %lu, seq %lu, %u Hz nominal on ch %u",
                     (unsigned long)s_sent, (unsigned long)s_failed,
                     (unsigned long)s_seq, 1000u / CSI_TX_INTERVAL_MS,
                     (unsigned)CSI_CHANNEL);
        }

        vTaskDelay(pdMS_TO_TICKS(CSI_TX_INTERVAL_MS));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // STA mode without connecting: ESP-NOW needs the radio up but no AP. This
    // means the transmitter works with no router present at all.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // The ESP32-C3 radio is 2.4 GHz only: valid channels are 1-14. A 5 GHz
    // channel number (36, 100, 149, ...) is rejected with ESP_ERR_INVALID_ARG.
    // Check before calling so this reports the actual mistake instead of an
    // ESP_ERROR_CHECK abort loop.
#if (CSI_CHANNEL < 1) || (CSI_CHANNEL > 14)
#error "CSI_CHANNEL must be 1-14. ESP32 Wi-Fi is 2.4 GHz only; 5 GHz channels \
(36/40/.../100/149) are not supported. If your router runs 5 GHz, use its 2.4 GHz \
band (or any free 2.4 GHz channel -- the transmitter does not need a router)."
#endif

    esp_err_t rc = esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (rc != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_channel(%u) failed: %s",
                 (unsigned)CSI_CHANNEL, esp_err_to_name(rc));
        ESP_LOGE(TAG, "Valid 2.4 GHz channels are 1-14. Rebuild with "
                      "-DCSI_CHANNEL=<1-14>.");
        return; // no point beaconing on the wrong channel
    }

    esp_wifi_set_ps(WIFI_PS_NONE); // never sleep: a gap in the beacon is a gap in data

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send));

    // Broadcast still needs a peer entry, with the channel pinned so ESP-NOW
    // does not follow a STA connection elsewhere.
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = CSI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    ESP_LOGI(TAG, "=== x-ray-wifi CSI TRANSMITTER ===");
    ESP_LOGI(TAG, "MAC %02x:%02x:%02x:%02x:%02x:%02x  <-- receivers see this as the source",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "broadcasting every %u ms (%u Hz) on channel %u, payload %u bytes",
             (unsigned)CSI_TX_INTERVAL_MS, 1000u / CSI_TX_INTERVAL_MS,
             (unsigned)CSI_CHANNEL, (unsigned)sizeof(csi_beacon_t));
    ESP_LOGI(TAG, "receivers MUST be built with the same CSI_CHANNEL");

    xTaskCreate(tx_task, "csi_tx_beacon", 4096, NULL, 5, NULL);
}

#endif /* CSI_ROLE == CSI_ROLE_TX */
