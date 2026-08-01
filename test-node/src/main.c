#include <stdio.h>
#include <string.h>
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
};

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
// ---------------------

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
    uint16_t reserved;        // Padding
} adr018_header_t;

typedef struct
{
    uint16_t len;
    int8_t rssi;
    int8_t noise_floor;
    uint8_t buf[CSI_MAX_FRAME_LEN];
} csi_packet_t;

// --- CSI ISR / CALLBACK (Non-blocking, enqueues packets) ---
static void wifi_csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || !csi_queue)
        return;

    // Drop oversized frames safely
    if (info->len > CSI_MAX_FRAME_LEN)
        return;

    csi_packet_t pkt;
    pkt.len = info->len;
    pkt.rssi = info->rx_ctrl.rssi;
    pkt.noise_floor = info->rx_ctrl.noise_floor;
    memcpy(pkt.buf, info->buf, info->len);

    // Non-blocking queue send (drops packet if queue is full instead of stalling Wi-Fi task)
    xQueueSend(csi_queue, &pkt, 0);
}

// --- WORKER TASK (Handles network socket transmission) ---
static void csi_tx_task(void *pvParameters)
{
    csi_packet_t pkt;
    uint8_t tx_buf[TX_BUF_LEN];

    while (1)
    {
        if (xQueueReceive(csi_queue, &pkt, portMAX_DELAY) == pdTRUE)
        {
            if (sock < 0)
                continue;

            uint16_t num_subcarriers = pkt.len / 2;
            size_t payload_len = sizeof(adr018_header_t) + pkt.len;

            if (payload_len > sizeof(tx_buf))
                continue;

            adr018_header_t *hdr = (adr018_header_t *)tx_buf;
            hdr->magic = ADR018_MAGIC;
            hdr->node_id = NODE_ID;
            hdr->num_antennas = NUM_ANTENNAS;
            hdr->num_subcarriers = num_subcarriers;
            hdr->freq_mhz = WIFI_CHANNEL_FREQ_MHZ;
            hdr->sequence = seq_num++;
            hdr->rssi = pkt.rssi;
            hdr->noise_floor = pkt.noise_floor;
            hdr->reserved = 0;

            memcpy(tx_buf + sizeof(adr018_header_t), pkt.buf, pkt.len);

            int err = sendto(sock, tx_buf, payload_len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error during sendto: errno %d", errno);
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
        ESP_LOGI(TAG, "Wi-Fi Connected! Setting up UDP stream...");

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        dest_addr.sin_addr.s_addr = inet_addr(TARGET_IP);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(TARGET_PORT);

        // Promiscuous mode setup for reliable packet capturing
        wifi_promiscuous_filter_t filter = {
            .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_MGMT};
        esp_wifi_set_promiscuous_filter(&filter);
        esp_wifi_set_promiscuous(true);

        wifi_csi_config_t csi_config = {
            .lltf_en = true,
            .htltf_en = true,
            .stbc_htltf2_en = false,
            .ltf_merge_en = true,
            .channel_filter_en = false,
            .manu_scale = false,
            .shift = false,
        };

        ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
        ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_cb, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_csi(true));
        esp_wifi_set_ps(WIFI_PS_NONE); // Disable sleep

        ESP_LOGI(TAG, "Streaming CSI -> %s:%d", TARGET_IP, TARGET_PORT);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    csi_queue = xQueueCreate(CSI_QUEUE_DEPTH, sizeof(csi_packet_t));

    // Spawn CSI TX Worker Task
    // ESP32-C3 is single-core, so this runs unpinned instead of on core 1.
    xTaskCreate(csi_tx_task, "csi_tx_task", CSI_TX_TASK_STACK_WORDS, NULL,
                CSI_TX_TASK_PRIORITY, NULL);

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
