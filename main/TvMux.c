#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "ddc.h"
#include "cec.h"
#include "wii.h"
#include "util.h"
#include "led.h"
#include "TvMux.h"
#include "TvMux_int.h"

#define TAG "TVMUX"

bool tvmux_device_state[CEC_MAX_ADDRESS + 1];
SemaphoreHandle_t tvmux_retry_sem;
portMUX_TYPE tvmux_mux = portMUX_INITIALIZER_UNLOCKED;
bool tvmux_steam_on_pending;
bool tvmux_tv_on_pending;
bool tvmux_wii_on_pending;
tvmux_retry_type_t tvmux_active_retry_func;

esp_err_t tvmux_init(bool* setup_enabled)
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = TVMUX_SETUP_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    //cec_init();
    
    // FIXME
    //xTaskCreate(tvmux_hdmi_task, "hdmi", 2000, NULL, 1, NULL);

    *setup_enabled = gpio_get_level(TVMUX_SETUP_GPIO_NUM) == TVMUX_SETUP_ENABLED;

    tvmux_retry_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(tvmux_retry_sem);    

    return ESP_OK;
}

bool tvmux_call_with_retry(tvmux_retry_type_t retry_type, tvmux_retry_func_t f, tvmux_retry_check_func_t check, void* retry_param)
{
    int retry = 0;
    bool success;

    if (xSemaphoreTake(tvmux_retry_sem, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "Retry already in progress.  Function %u cancelled.", retry_type);
        return false;
    }

    led_push_rgb_color(0, 0, 255);

    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_active_retry_func = retry_type;
    taskEXIT_CRITICAL(&tvmux_mux);

    do
    {
        if (retry != 0)
        {
            ESP_LOGE(TAG, "Control retry #%d in %dms", retry, TVMUX_RETRY_CHECK_WAIT_MS);
            vTaskDelay(TVMUX_RETRY_WAIT_MS / portTICK_PERIOD_MS);
        }
        f(retry_param);
        vTaskDelay(TVMUX_RETRY_CHECK_WAIT_MS / portTICK_PERIOD_MS);
        success = check(retry_param);
        //cec_queue_clear();
    } while (retry++ < TVMUX_RETRY_MAX && !success);

    xSemaphoreGive(tvmux_retry_sem);

    led_pop_rgb_color();

    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_active_retry_func = NO_RETRY_FUNC;
    taskEXIT_CRITICAL(&tvmux_mux);

    return success;
}

bool check_retry_busy(tvmux_retry_type_t func)
{
    if (xSemaphoreTake(tvmux_retry_sem, 0) == pdTRUE)
    {
        xSemaphoreGive(tvmux_retry_sem);

        return false;        
    }

    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_retry_type_t active_retry_func = tvmux_active_retry_func;
    taskEXIT_CRITICAL(&tvmux_mux);

    return active_retry_func == func;
}

bool tvmux_steam_power_on()
{
    // for (int i = 0; i < 10; i++)
    // {
    //     if (tvmux_steam_is_on())
    //     {
    //         return true;
    //     }
    //     printf("send WOL (%d)\n", i + 1);
    //     //WOL.sendMagicPacket(TVMUX_STEAM_PC_MAC);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    send_WOL(TVMUX_STEAM_MAC, 10, 1000);

    return false;
}


cJSON* tvmux_steam_get_json(const char* path)
{
    char url[100];
    snprintf(url, sizeof(url), "http://%s:%d%s", TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PORT, path);

    return get_json(url);
}

bool tvmux_steam_get(const char* path)
{
    cJSON* json = tvmux_steam_get_json(path);
    if (json == NULL)
    {
        return false;
    }

    cJSON_Delete(json);

    return true;
}

void tvmux_steam_start()
{
    // HTTPClient http;

    // String host(STEAM_PC_HOSTNAME);
    // uint16_t port = TVMUX_STEAM_PC_PORT;
    // String path("/api/steam/bigpicture/start");
   
    // int http_code;
    // http.setTimeout(30000);
    // http.begin(host, port, path);
    // if (HTTP_SUCCESS(http_code = http.GET()))
    // {
    //     auto json = http.getString();
    //     ESP_LOGI(TAG, "%s: %s", path.c_str(), json.c_str());      
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    // }
    // http.end();

    if (tvmux_steam_get("/api/steam/bigpicture/start"))
    {
        ESP_LOGI(TAG, "tvmux_steam_start succeeded");
    }
    else
    {
        ESP_LOGE(TAG, "tvmux_steam_start failed");
    }
}

void tvmux_steam_close()
{
    // HTTPClient http;

    // String host(STEAM_PC_HOSTNAME);
    // uint16_t port = STEAM_PC_PORT;
    // String path("/api/steam/close");
   
    // int http_code;
    // http.setTimeout(30000);
    // http.begin(host, port, path);
    // if (HTTP_SUCCESS(http_code = http.GET()))
    // {
    //     auto json = http.getString();
    //     ESP_LOGI(TAG, "%s: %s", path.c_str(), json.c_str());      
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    // }
    // http.end();
    
    if (tvmux_steam_get("/api/steam/close"))
    {
        ESP_LOGE(TAG, "tvmux_steam_close succeeded");
    }
    else
    {
        ESP_LOGE(TAG, "tvmux_steam_close failed");
    }       
}

void steam_power_off()
{
    // HTTPClient http;

    // String host(STEAM_PC_HOSTNAME);
    // uint16_t port = STEAM_PC_PORT;
    // String path("/api/computer/off");
   
    // int http_code;
    // http.setTimeout(30000);
    // http.begin(host, port, path);
    // if (HTTP_SUCCESS(http_code = http.GET()))
    // {
    //     auto json = http.getString();
    //     ESP_LOGI(TAG, "%s: %s", path.c_str(), json.c_str());      
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    // }
    // http.end();
    if (tvmux_steam_get("/api/computer/off"))
    {
        ESP_LOGE(TAG, "steam_power_off succeeded");
    }
    else
    {
        ESP_LOGE(TAG, "steam_power_off failed");
    }   
}

bool check_steam_topology()
{
    bool external = false;

    // HTTPClient http;

    // String host(STEAM_PC_HOSTNAME);
    // uint16_t port = STEAM_PC_PORT;
    // String path("/api/monitor/topology");
   
    // http.setTimeout(30000);
    // http.begin(host, port, path);
    // int http_code;
    // if (HTTP_SUCCESS(http_code = http.GET()))
    // {
    //     auto json = http.getString();
    //     cJSON* doc = cJSON_Parse(json.c_str());
    //     if (doc != NULL)
    //     {
    //         auto topology_prop = cJSON_GetObjectItem(doc, "topology");
    //         if (topology_prop != NULL)
    //         {
    //             auto topology_value = cJSON_GetStringValue(topology_prop);
    //             if (topology_value != NULL)
    //             {
    //                 external = strcasecmp(topology_value, "External") == 0;
    //                 ESP_LOGI(TAG, "topology: %s", topology_value);
    //             }
    //         }
    //         cJSON_Delete(doc);
    //     }
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    // }
    // http.end();

    // return external;

    cJSON* json = tvmux_steam_get_json("/api/monitor/topology");
    if (json != NULL)
    {
        cJSON* topology_prop = cJSON_GetObjectItem(json, "topology");
        if (topology_prop != NULL)
        {
            const char* topology_value = cJSON_GetStringValue(topology_prop);
            if (topology_value != NULL)
            {
                external = strcasecmp(topology_value, "External") == 0;
                ESP_LOGI(TAG, "topology: %s %d", topology_value, external);
            }
        }
        cJSON_Delete(json);
    }
    else
    {
        ESP_LOGE(TAG, "check_steam_topology failed");
    }

    return external;
}

bool tvmux_steam_is_on()
{
    int pongs;
    return ping(TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PING_COUNT, TVMUX_STEAM_PING_INTERVAL, &pongs) == ESP_OK && pongs > 0;
}

bool tvmux_steam_is_open()
{
    bool open = false;

    // HTTPClient http;

    // String host(STEAM_PC_HOSTNAME);
    // uint16_t port = STEAM_PC_PORT;
    // String path("/api/steam/status");
   
    // http.setTimeout(30000);
    // http.begin(host, port, path);
    // int http_code;
    // if (HTTP_SUCCESS(http_code = http.GET()))
    // {
    //     auto json = http.getString();
    //     cJSON* doc = cJSON_Parse(json.c_str());
    //     if (doc != NULL)
    //     {
    //         auto status_prop = cJSON_GetObjectItem(doc, "steamStatus");
    //         if (status_prop != NULL)
    //         {
    //             auto status_value = cJSON_GetStringValue(status_prop);
    //             if (status_value != NULL)
    //             {
    //                 open = strcasecmp(status_value, "Open") == 0;
    //                 ESP_LOGI(TAG, "open: %s", status_value);
    //             }
    //         }
    //         cJSON_Delete(doc);
    //     }
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    // }
    // http.end();

    // return open;

    cJSON* json = tvmux_steam_get_json("/api/steam/status");
    if (json != NULL)
    {
        cJSON* status_prop = cJSON_GetObjectItem(json, "steamStatus");
        if (status_prop != NULL)
        {
            const char* status_value = cJSON_GetStringValue(status_prop);
            if (status_value != NULL)
            {
                open = strcasecmp(status_value, "Open") == 0;
                ESP_LOGI(TAG, "open: %s %d", status_value, open);
            }
        }
        cJSON_Delete(json);
    }
    else
    {
        ESP_LOGE(TAG, "tvmux_steam_is_open failed");
    }
    return open;
}

esp_err_t tvmux_steam_state(bool and_mode, bool exclusive, bool* state, bool* pending)
{
    if (pending != NULL)
    {        
        *pending = false;

        if (check_retry_busy(STEAM_RETRY_FUNC))
        {
            taskENTER_CRITICAL(&tvmux_mux);
            bool steam_on_pending = tvmux_steam_on_pending;
            taskEXIT_CRITICAL(&tvmux_mux);

            ESP_LOGI(TAG, "Returning pending steam state %d", (bool)steam_on_pending);

            *pending = true;

            return steam_on_pending;
        }
    }

    cec_combine_devices_state(state, and_mode, true, true, false);

    ESP_LOGI(TAG, "steam status start %d", *state);

    if (and_mode)
    {
        if (state)
        {
            int pongs;
            *state &= ping(TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PING_COUNT, TVMUX_STEAM_PING_INTERVAL, &pongs) == ESP_OK && pongs > 0;
            ESP_LOGI(TAG, "steam status ping %d", *state);
        }
        if (state)
        {
            *state &= check_steam_topology();
            ESP_LOGI(TAG, "steam status topology %d", *state);            
        }
        if (state)
        {
            *state &= tvmux_steam_is_open();
            ESP_LOGI(TAG, "steam status open %d", *state);
        }
    }
    else
    {
        if (exclusive)
        {
            int pongs;
            *state = ping(TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PING_COUNT, TVMUX_STEAM_PING_INTERVAL, &pongs) == ESP_OK && pongs > 0 && tvmux_steam_is_open();
        }
        else
        {
            if (!state)
            {
                int pongs;
                *state |= ping(TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PING_COUNT, TVMUX_STEAM_PING_INTERVAL, &pongs) == ESP_OK && pongs > 0;
            }
        }
    }
    ESP_LOGI(TAG, "steam status end %d", *state);

    return ESP_OK;
}

esp_err_t tvmux_steam_power(bool power_on)
{
    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_steam_on_pending = power_on;
    taskEXIT_CRITICAL(&tvmux_mux);

    xTaskCreate(tvmux_steam_state_task, "steam_state", 4000, (void*)power_on, 1, NULL);

    return ESP_OK;
}

esp_err_t tvmux_wii_power(bool power_on)
{
    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_wii_on_pending = power_on;
    taskEXIT_CRITICAL(&tvmux_mux);

    cec_wii_power(power_on);

    return ESP_OK;
}

esp_err_t tvmux_cec_log_write(httpd_req_t* request)
{
    cec_log_write(request);

    return ESP_OK;
}

esp_err_t tvmux_cec_log_clear()
{
    cec_log_clear();

    return ESP_OK;
}

esp_err_t tvmux_cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{
    cec_control(target_address, request, request_size, reply_filter, reply, reply_size);

    return ESP_OK;
}

esp_err_t tvmux_tv_pending_get(bool* pending)
{
    taskENTER_CRITICAL(&tvmux_mux);
    *pending = tvmux_tv_on_pending;
    taskEXIT_CRITICAL(&tvmux_mux);

    return ESP_OK;
}

esp_err_t tvmux_tv_state_get(bool* tv_state, bool* pending)
{
    *pending = false;

    if (check_retry_busy(TV_RETRY_FUNC))
    {
        tvmux_tv_pending_get(tv_state);
        *pending = true;
    }
    else
    {
        ESP_RETURN_ON_ERROR(cec_combine_devices_state(tv_state, false, true, true, true), TAG, "%s/tvmux_combine_devices_state failed (%s)", __func__, esp_err_to_name(err_rc_));
    }

    return ESP_OK;
}

esp_err_t tvmux_wii_state_get(wii_power_state_t* wii_power_state, bool* pending)
{
    *pending = false;

    if (check_retry_busy(WII_RETRY_FUNC))
    {
        taskENTER_CRITICAL(&tvmux_mux);
        bool wii_on_pending = tvmux_wii_on_pending;
        taskEXIT_CRITICAL(&tvmux_mux);

        *wii_power_state = wii_on_pending ? WII_POWER_STATE_ON : WII_POWER_STATE_OFF;
        *pending = true;
    }
    else
    {
        *wii_power_state = wii_query_power_state();
    }

    return ESP_OK;
}

void tvmux_steam_state_set(void* retry_param)
{
    bool desired_state = (bool)retry_param;

    if (desired_state)
    {
        uint16_t addr = CEC_TV_HDMI_INPUT << 12 | CEC_STEAM_HDMI_INPUT << 8;

        cec_as_set(addr);
        cec_sam_request(addr);
        cec_tv_on();

        ESP_LOGI(TAG, "checking steam pc...");
        if (!tvmux_steam_is_on())
        {
            ESP_LOGI(TAG, "powering on steam pc...");
            if (!tvmux_steam_power_on())
            {
                return;
            }
            //vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "opening steam...");
        tvmux_steam_start();
    }
    else
    {
        ESP_LOGI(TAG, "turn steam off\n");
        cec_standby();
        tvmux_steam_close();
        steam_power_off();
    }
}

bool tvmux_steam_state_check(void* retry_param)
{
    bool desired_state = (bool)retry_param;

    bool state;
    return tvmux_steam_state(desired_state, false, &state, NULL) == ESP_OK && state == desired_state;
}

void tvmux_steam_state_task(void* param)
{
    bool desired_state = (bool)param;

    ESP_LOGI(TAG, "set_steam_state = %u requested", desired_state);

    if (tvmux_call_with_retry(STEAM_RETRY_FUNC, tvmux_steam_state_set, tvmux_steam_state_check, (void*)desired_state))
    {
        ESP_LOGI(TAG, "set_steam_state = %u succeeded\n", desired_state);
    }
    else
    {
        ESP_LOGE(TAG, "set_steam_state = %u failed\n", desired_state);
    }

    vTaskDelete(NULL);
}

esp_err_t tvmux_tv_power(bool power_on)
{
    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_tv_on_pending = power_on;
    taskEXIT_CRITICAL(&tvmux_mux);

    return cec_tv_power(power_on);
}