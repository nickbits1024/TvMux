#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "ble.h"
#include "wifi.h"
#include "led.h"
#include "config.h"
#include "webserver.h"
#include "config.h"
#include "health.h"
#include "wii.h"
#include "cec.h"
#include "ddc.h"
#include "util.h"
#include "cJSON.h"
#include "TvMux.h"

#define TAG "MAIN"

void app_main(void)
{
    ESP_LOGI(TAG, "app_main()");

    ESP_ERROR_CHECK(led_init());
    led_set_rgb_color(0, 255, 0);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    esp_vfs_spiffs_conf_t spiffs = 
    {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };

    bool setup_enabled;
 
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs));
    ESP_ERROR_CHECK(wifi_init());
    ESP_ERROR_CHECK(webserver_init());
    ESP_ERROR_CHECK(ddc_init());
    ESP_ERROR_CHECK(cec_init());
    ESP_ERROR_CHECK(tvmux_init(&setup_enabled));
    if (setup_enabled)
    {
        ESP_LOGI(TAG, "setup mode enabled");
        ESP_ERROR_CHECK(ble_init());
    }
    else
    {
        ESP_LOGI(TAG, "setup mode disabled");
        ESP_ERROR_CHECK(wii_init());
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    led_set_rgb_color(0, 1, 0);
}
