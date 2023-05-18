#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"
#include "ping/ping_sock.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "health.h"
#include "wifi_int.h"
#include "wifi.h"

#define TAG "WIFI"

static nvs_handle_t wifi_nvs_handle;
static EventGroupHandle_t wifi_event_group;
static portMUX_TYPE wifi_mux = portMUX_INITIALIZER_UNLOCKED;
static char wifi_ssid[WIFI_SSID_SIZE];
static char wifi_password[WIFI_PASSWORD_SIZE];

esp_err_t wifi_init()
{
    ESP_RETURN_ON_ERROR(nvs_open(WIFI_NVS_NAME, NVS_READWRITE, &wifi_nvs_handle),
        TAG, "nvs_open failed (%s)", esp_err_to_name(err_rc_));

    size_t size = sizeof(wifi_ssid);
    esp_err_t ret = nvs_get_blob(wifi_nvs_handle, WIFI_SSID_NVS_KEY, wifi_ssid, &size);
    if (ret == ESP_OK && size == sizeof(wifi_ssid))
    {
        ESP_LOGI(TAG, "saved ssid = %s", wifi_ssid);
    }
    size = sizeof(wifi_password);
    ret = nvs_get_blob(wifi_nvs_handle, WIFI_PASSWORD_NVS_KEY, wifi_password, &size);
    if (ret == ESP_OK && size == sizeof(wifi_password))
    {
        ESP_LOGI(TAG, "saved password = ********");
    }

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    xTaskCreate(wifi_task, "wifi", 4000, NULL, 1, NULL);

    return ESP_OK;
}

esp_err_t wifi_get_ssid(char* ssid, int ssid_size)
{
    if (ssid_size < sizeof(wifi_ssid))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    taskENTER_CRITICAL(&wifi_mux);
    memcpy(ssid, wifi_ssid, sizeof(wifi_ssid));
    taskEXIT_CRITICAL(&wifi_mux);

    return ESP_OK;
}

esp_err_t wifi_set_ssid(const char* ssid)
{
    ESP_LOGI(TAG, "wifi_set_ssid %s", ssid);

    char ssid_copy[sizeof(wifi_ssid)];

    taskENTER_CRITICAL(&wifi_mux);
    strncpy(wifi_ssid, ssid, sizeof(wifi_ssid));
    wifi_ssid[sizeof(wifi_ssid) - 1] = 0;
    memcpy(ssid_copy, wifi_ssid, sizeof(ssid_copy));
    taskEXIT_CRITICAL(&wifi_mux);

    ESP_ERROR_CHECK(nvs_set_blob(wifi_nvs_handle, WIFI_SSID_NVS_KEY, ssid_copy, sizeof(ssid_copy)));

    ESP_RETURN_ON_ERROR(wifi_reset(), TAG, "wifi reset failed (%s)", esp_err_to_name(err_rc_));

    return ESP_OK;
}

esp_err_t wifi_set_password(const char* password)
{
    ESP_LOGI(TAG, "wifi_set_password %s", password);

    char password_copy[sizeof(wifi_password)];

    taskENTER_CRITICAL(&wifi_mux);
    strncpy(wifi_password, password, sizeof(wifi_password));
    wifi_password[sizeof(wifi_password) - 1] = 0;
    memcpy(password_copy, wifi_password, sizeof(password_copy));
    taskEXIT_CRITICAL(&wifi_mux);
   
    ESP_ERROR_CHECK(nvs_set_blob(wifi_nvs_handle, WIFI_PASSWORD_NVS_KEY, password_copy, sizeof(password_copy)));

    ESP_RETURN_ON_ERROR(wifi_reset(), TAG, "wifi reset failed (%s)", esp_err_to_name(err_rc_));

    return ESP_OK;
}

esp_err_t wifi_reset()
{
    wifi_ap_record_t wifi_info;

    esp_err_t ret = esp_wifi_sta_get_ap_info(&wifi_info);
    if (ret == ESP_OK)
    {
        ESP_RETURN_ON_ERROR(esp_wifi_stop(), TAG, "esp_wifi_stop failed (%s)", esp_err_to_name(err_rc_));
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_NULL), TAG, "esp_wifi_set_mode failed (%s)", esp_err_to_name(err_rc_));
        //ESP_RETURN_ON_ERROR(esp_wifi_restore(), TAG, "esp_wifi_restore failed (%s)", esp_err_to_name(err_rc_));
    }

    return ESP_OK;
}

esp_err_t wifi_connect()
{
    esp_err_t ret = ESP_OK;

    char ssid[sizeof(wifi_ssid)];
    char password[sizeof(wifi_password)];
    
    taskENTER_CRITICAL(&wifi_mux);
    memcpy(ssid, wifi_ssid, sizeof(ssid));
    memcpy(password, wifi_password, sizeof(password));
    taskEXIT_CRITICAL(&wifi_mux);

    if (strlen(ssid) == 0 || strlen(password) == 0)
    {
        ESP_LOGI(TAG, "ssid/password not set");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "starting wifi...");

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { };

    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);
    //wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    //wifi_config.sta.pmf_cfg.capable = true;
    //wifi_config.sta.pmf_cfg.required = false;

    ESP_GOTO_ON_ERROR(esp_wifi_restore(), cleanup, TAG, "esp_wifi_restore failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), cleanup, TAG, "esp_wifi_set_mode failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), cleanup, TAG, "esp_wifi_set_config failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(esp_wifi_start(), cleanup, TAG, "esp_wifi_start failed (%s)", esp_err_to_name(err_rc_));

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to SSID: %s password: ********", wifi_ssid);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "failed to connect to SSID: %s, password: ********", wifi_ssid);
    }
    ESP_ERROR_CHECK(health_start());
cleanup:
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));

    return ret;
}

static void wifi_task(void* param)
{
    wifi_ap_record_t wifi_info;

    for (;;)
    {
        esp_err_t ret = esp_wifi_sta_get_ap_info(&wifi_info);
        //printf("esp_wifi_sta_get_ap_info=%s mac=%s signal=%d\n", esp_err_to_name(ret), wifi_info.ssid, wifi_info.rssi);
        if (ret != ESP_OK)
        {
            ESP_LOGI(TAG, "connection lost, reconnecting...");
            ESP_ERROR_CHECK(wifi_reset());
            //ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            wifi_connect();
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGI(TAG, "event (%s).%ld", event_base, event_id);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
