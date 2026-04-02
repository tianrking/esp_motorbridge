#include "net/wifi_udp_bridge.h"

#include <string.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "app/command_router.h"
#include "config/app_config.h"

static const char *TAG = "wifi_udp";
static EventGroupHandle_t s_wifi_events;

#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void udp_server_task(void *arg)
{
    const app_config_t *cfg = app_config_get();
    const int port = cfg->udp_port;
    uint8_t buf[128];

    while (1) {
        xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in bind_addr;
        memset(&bind_addr, 0, sizeof(bind_addr));
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind_addr.sin_port = htons((uint16_t)port);

        if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "UDP listening on :%d", port);

        while (1) {
            struct sockaddr_in source_addr;
            socklen_t source_addr_len = sizeof(source_addr);
            int len = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&source_addr, &source_addr_len);
            if (len <= 0) {
                break;
            }

            // UDP frame format:
            // [0..3] can_id u32 little-endian, [4] dlc, [5..12] data (8 bytes)
            if (len >= 13) {
                twai_message_t msg;
                memset(&msg, 0, sizeof(msg));

                uint32_t can_id = 0;
                memcpy(&can_id, &buf[0], sizeof(can_id));
                uint8_t dlc = buf[4];
                if (dlc > 8) {
                    dlc = 8;
                }

                msg.identifier = can_id;
                msg.extd = 0;
                msg.rtr = 0;
                msg.data_length_code = dlc;
                memcpy(msg.data, &buf[5], 8);

                command_router_handle_can_rx(&msg);
            }
        }

        close(sock);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

esp_err_t wifi_udp_bridge_start(void)
{
    const app_config_t *cfg = app_config_get();
    if (!cfg->wifi_udp_enabled) {
        ESP_LOGI(TAG, "WiFi UDP bridge disabled");
        return ESP_OK;
    }

    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t ev = esp_event_loop_create_default();
    if (ev != ESP_OK && ev != ESP_ERR_INVALID_STATE) {
        return ev;
    }

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    wifi_config_t wifi_cfg;
    memset(&wifi_cfg, 0, sizeof(wifi_cfg));
    strlcpy((char *)wifi_cfg.ap.ssid, cfg->wifi_ssid, sizeof(wifi_cfg.ap.ssid));
    strlcpy((char *)wifi_cfg.ap.password, cfg->wifi_password, sizeof(wifi_cfg.ap.password));
    wifi_cfg.ap.ssid_len = (uint8_t)strlen(cfg->wifi_ssid);
    wifi_cfg.ap.max_connection = 4;
    wifi_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(cfg->wifi_password) < 8) {
        wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;
        wifi_cfg.ap.password[0] = '\0';
    }

    ESP_LOGI(TAG, "starting AP ssid=%s", cfg->wifi_ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreatePinnedToCore(udp_server_task, "udp_srv", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    return ESP_OK;
}
