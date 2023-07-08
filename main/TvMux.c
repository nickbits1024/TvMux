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
        cec_queue_clear();
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
    int retry = 0;
    do
    {

        send_WOL(TVMUX_STEAM_MAC, 10, 100);
        vTaskDelay(TVMUX_STEAM_RETRY_WAIT / portTICK_PERIOD_MS);
        if (tvmux_steam_is_on())
        {
            vTaskDelay(TVMUX_STEAM_ON_WAIT / portTICK_PERIOD_MS);
            return true;
        }
    }
    while (++retry < TVMUX_STEAM_RETRY_MAX);

    return false;
}

cJSON* tvmux_steam_get_json(const char* path)
{
    char url[100];
    snprintf(url, sizeof(url), "http://%s:%d%s", TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PORT, path);

    return get_json(url);
}

cJSON* tvmux_steam_post_json(const char* path, cJSON* post_object)
{
    char url[100];
    snprintf(url, sizeof(url), "http://%s:%d%s", TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PORT, path);

    return post_json(url, post_object);
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
    if (tvmux_steam_get("/api/computer/off"))
    {
        ESP_LOGE(TAG, "steam_power_off succeeded");
    }
    else
    {
        ESP_LOGE(TAG, "steam_power_off failed");
    }   
}

bool tvmux_steam_topology_check(const char* topology)
{
    bool match = false;

    cJSON* json = tvmux_steam_get_json("/api/monitor/topology");
    if (json != NULL)
    {
        cJSON* topology_prop = cJSON_GetObjectItem(json, "topology");
        if (topology_prop != NULL)
        {
            const char* topology_value = cJSON_GetStringValue(topology_prop);
            if (topology_value != NULL)
            {
                match = strcasecmp(topology_value, topology) == 0;
                ESP_LOGI(TAG, "topology: %s match: %s", topology_value, match ? "yes" : "no");
            }
        }
        cJSON_Delete(json);
    }
    else
    {
        ESP_LOGE(TAG, "tvmux_steam_topology_check failed");
    }

    return match;
}

bool tvmux_steam_is_on()
{
    int pongs;
    return ping(TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PING_COUNT, TVMUX_STEAM_PING_INTERVAL, &pongs) == ESP_OK && pongs > 0;
}

bool tvmux_steam_is_open()
{
    bool open = false;

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

void tvmux_steam_topology(const char* topology)
{
    cJSON* post_object = cJSON_CreateObject();
    
    if (cJSON_AddStringToObject(post_object, "topology", topology) == NULL) goto cleanup;

    cJSON* json = tvmux_steam_post_json("/api/monitor/topology", post_object);
    if (json != NULL)
    {
        cJSON* topology_prop = cJSON_GetObjectItem(json, "topology");
        if (topology_prop != NULL)
        {
            const char* topology_value = cJSON_GetStringValue(topology_prop);
            if (topology_value != NULL)
            {
                ESP_LOGI(TAG, "topology: %s", topology_value);
            }
        }
        cJSON_Delete(json);
    }

cleanup:
    if (post_object != NULL)
    {
        cJSON_Delete(post_object);
    }
}

esp_err_t tvmux_steam_state(bool and_mode, bool exclusive, bool* state, bool* pending)
{
    if (pending != NULL)
    {        
        *pending = false;

        if (check_retry_busy(STEAM_RETRY_FUNC))
        {
            taskENTER_CRITICAL(&tvmux_mux);
            *state = tvmux_steam_on_pending;
            taskEXIT_CRITICAL(&tvmux_mux);

            ESP_LOGI(TAG, "Returning pending steam state %d", *state);

            *pending = true;

            return ESP_OK;
        }
    }

    tvmux_combine_devices_state(state, and_mode, true, true, false, true);

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
            *state &= tvmux_steam_topology_check(TVMUX_STEAM_TOPOLOGY_EXTERNAL);
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

void wii_state_set(void* retry_param)
{
    bool desired_state = (bool)retry_param;

    if (desired_state)
    {
        wii_power_on();
        uint16_t addr = CEC_TV_HDMI_INPUT << 12 | CEC_WII_HDMI_INPUT << 8;

        cec_active_source(addr);
        cec_image_view_on(CEC_LA_TV);
        cec_system_audio_mode_request(addr);
    }
    else
    {
        wii_power_off();
        cec_standby();
        vTaskDelay(TVMUX_STANDBY_DELAY / portTICK_PERIOD_MS);
    }
}

bool wii_state_check(void* retry_param)
{
    bool desired_av_state = (bool)retry_param;

    wii_power_state_t desired_wii_state = desired_av_state ? WII_POWER_STATE_ON : WII_POWER_STATE_OFF;

    bool state;
    return wii_query_power_state() == desired_wii_state &&
        tvmux_combine_devices_state(&state, desired_av_state, true, true, false, false) == ESP_OK && state == desired_av_state;
}

void wii_state_task(void* param)
{
    bool desired_state = (bool)param;

    ESP_LOGI(TAG, "set_wii_state = %u requested", desired_state);

    if (tvmux_call_with_retry(WII_RETRY_FUNC, wii_state_set, wii_state_check, (void*)desired_state))
    {
        ESP_LOGI(TAG, "set_wii_state = %u succeeded", desired_state);
    }
    else
    {
        ESP_LOGE(TAG, "set_wii_state = %u failed", desired_state);
    }

    vTaskDelete(NULL);
}

esp_err_t tvmux_wii_power(bool power_on)
{
    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_wii_on_pending = power_on;
    taskEXIT_CRITICAL(&tvmux_mux);

    xTaskCreate(wii_state_task, "wii_state", 4000, (void*)power_on, 1, NULL);

    return ESP_OK;
}

esp_err_t tvmux_tv_pending_get(bool* pending)
{
    taskENTER_CRITICAL(&tvmux_mux);
    *pending = tvmux_tv_on_pending;
    taskEXIT_CRITICAL(&tvmux_mux);

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

        ESP_LOGI(TAG, "checking steam pc...");
        if (!tvmux_steam_is_on())
        {
            ESP_LOGI(TAG, "powering on steam pc...");
            if (!tvmux_steam_power_on())
            {
                return;
            }
        }
        cec_image_view_on(CEC_LA_TV);
        cec_active_source(addr);
        cec_system_audio_mode_request(addr);
        tvmux_steam_topology(TVMUX_STEAM_TOPOLOGY_INTERNAL);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "opening steam...");
        tvmux_steam_start();
    }
    else
    {
        ESP_LOGI(TAG, "turn steam off");
        cec_standby();
        tvmux_steam_close();
        steam_power_off();
        vTaskDelay(TVMUX_STANDBY_DELAY / portTICK_PERIOD_MS);
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
        ESP_LOGI(TAG, "set_steam_state = %u succeeded", desired_state);
    }
    else
    {
        ESP_LOGE(TAG, "set_steam_state = %u failed", desired_state);
    }

    vTaskDelete(NULL);
}

esp_err_t tvmux_steam_power(bool power_on)
{
    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_steam_on_pending = power_on;
    taskEXIT_CRITICAL(&tvmux_mux);

    xTaskCreate(tvmux_steam_state_task, "steam_state", 4000, (void*)power_on, 1, NULL);

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
        ESP_RETURN_ON_ERROR(tvmux_combine_devices_state(tv_state, false, true, true, true, true), TAG, "%s/tvmux_combine_devices_state failed (%s)", __func__, esp_err_to_name(err_rc_));
    }

    return ESP_OK;
}

void tvmux_tv_state_set(void* retry_param)
{
    bool desired_state = (bool)retry_param;

    if (desired_state)
    {
        uint16_t addr = CEC_TV_HDMI_INPUT << 12 | CEC_ATV_HDMI_INPUT << 8;
        cec_active_source(addr);
        cec_image_view_on(CEC_LA_TV);
        cec_system_audio_mode_request(addr);
        cec_user_control_pressed(CEC_LA_PLAYBACK_DEVICE_1, CEC_OP_UCC_POWER_ON_FUNCTION);

        if (tvmux_steam_is_on())
        {
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            tvmux_steam_topology(TVMUX_STEAM_TOPOLOGY_INTERNAL);
        }
    }
    else
    {
        cec_standby();
        vTaskDelay(TVMUX_STANDBY_DELAY / portTICK_PERIOD_MS);
    }
}

bool tvmux_tv_state_check(void* retry_param)
{
    bool desired_state = (bool)retry_param;

    bool state;
    return (!tvmux_steam_is_on() || tvmux_steam_topology_check(TVMUX_STEAM_TOPOLOGY_INTERNAL)) &&
        tvmux_combine_devices_state(&state, desired_state, true, true, true, false) == ESP_OK && state == desired_state;
}

void tvmux_tv_state_task(void* param)
{
    bool desired_state = (bool)param;

    ESP_LOGI(TAG, "set_tv_state = %u requested", desired_state);

    if (tvmux_call_with_retry(TV_RETRY_FUNC, tvmux_tv_state_set, tvmux_tv_state_check, (void*)desired_state))
    {
        ESP_LOGI(TAG, "set_tv_state = %u succeeded", desired_state);
    }
    else
    {
        ESP_LOGE(TAG, "set_tv_state = %u failed", desired_state);
    }

    vTaskDelete(NULL);
}

esp_err_t tvmux_tv_power(bool power_on)
{
    taskENTER_CRITICAL(&tvmux_mux);
    tvmux_tv_on_pending = power_on;
    taskEXIT_CRITICAL(&tvmux_mux);

    xTaskCreate(tvmux_tv_state_task, "tv_state", 4000, (void*)power_on, 1, NULL);

    return ESP_OK;
}

esp_err_t tvmux_device_check(bool and_mode, cec_logical_address_t log_addr, bool* combined_on, const char* name, bool use_cache)
{
    cec_power_status_t power_status;

    esp_err_t result = cec_power_status(log_addr, &power_status, use_cache);
    if (result != ESP_OK) 
    {
        ESP_LOGE(TAG, "dev check %d result %s", log_addr, esp_err_to_name(result));
        return result;
    }

    bool device_on = power_status == CEC_PS_ON || power_status == CEC_PS_TRANS_ON;            

    ESP_LOGI(TAG, "%s device_on %d power_status %d", name, device_on, power_status);

    if (and_mode)
    {
        *combined_on &= device_on;
    }
    else
    {
        *combined_on |= device_on;
    }

    ESP_LOGI(TAG, "%s is %s, all combined is %s", name, device_on ? "on" : "off", *combined_on ? "on" : "off");

    return ESP_OK;
}

esp_err_t tvmux_combine_devices_state(bool* state, bool and_mode, bool tv, bool audio, bool atv, bool use_cache)
{
    bool combined_on = and_mode;

    if (tv)
    {
        ESP_RETURN_ON_ERROR(tvmux_device_check(and_mode, CEC_LA_TV, &combined_on, "tv", use_cache), TAG, "tvmux_device_check failed (%s)", esp_err_to_name(err_rc_));
    }
    if (audio)
    {
        ESP_RETURN_ON_ERROR(tvmux_device_check(and_mode, CEC_LA_AUDIO_SYSTEM, &combined_on, "audio", use_cache), TAG, "tvmux_device_check failed (%s)", esp_err_to_name(err_rc_));
    }
    if (atv)
    {
        ESP_RETURN_ON_ERROR(tvmux_device_check(and_mode, CEC_LA_PLAYBACK_DEVICE_1, &combined_on, "atv", use_cache), TAG, "tvmux_device_check failed (%s)", esp_err_to_name(err_rc_));
    }

    *state = combined_on;

    return ESP_OK;
}