#include <atomic>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "ddc.h"
#include "cec.h"
#include "wii.h"
#include "util.h"
#include "TvMux.h"
#include "TvMux_int.h"

#define TAG "TVMUX"

uint16_t cec_physical_address;
nvs_handle config_nvs_handle;
httpd_handle_t http_server_handle;
//WiFiUDP UDP;
//WakeOnLan WOL(UDP);
bool device_state[CEC_MAX_ADDRESS + 1];
SemaphoreHandle_t retry_sem;
std::atomic_bool steam_on_pending;
std::atomic_bool tv_on_pending;
std::atomic_bool wii_on_pending;
std::atomic<retry_function_t> retry_function;

esp_err_t tvmux_init()
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = TVMUX_HOTPLUG_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    esp_err_t err = nvs_open("default", NVS_READWRITE, &config_nvs_handle);
    ESP_ERROR_CHECK(err);

#ifdef HDMI_CEC
    ESP_LOGI(TAG, "Waiting for hotplug signal...");
    while (gpio_get_level(TVMUX_HOTPLUG_GPIO_NUM) == 0)
    { 
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Hotplug signal detected!\n");
    //last_hpd_time = uptimed();

    uint8_t edid[DDC_EDID_LENGTH];
    uint8_t edid_extension[DDC_EDID_EXTENSION_LENGTH];

    do
    {
        for (int i = 0; i < DDC_EDID_LENGTH; i++)
        {
            ESP_ERROR_CHECK(ddc_read_byte(DDC_EDID_ADDRESS, i, &edid[i]));
        }

        ESP_LOGI(TAG, "Received EDID");

    } while (!parse_edid(edid));

    if (edid[DDC_EDID_EXTENSION_FLAG])
    {
        do
        {
            for (int i = 0; i < DDC_EDID_EXTENSION_LENGTH; i++)
            {
                ESP_ERROR_CHECK(ddc_read_byte(DDC_EDID_ADDRESS, DDC_EDID_LENGTH + i, &edid_extension[i]));
            }

            ESP_LOGI(TAG, "Received EDID extension");

        } while (!parse_edid_extension(edid, edid_extension));
    }

    cec_init();
    
#endif

    retry_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(retry_sem);    

    return ESP_OK;
}

bool tvmux_call_with_retry(retry_function_t func, std::function<void()> f, std::function<bool()> check, int check_wait_ms, int retry_wait_ms)
{
    int retry = 0;
    bool success;

    if (xSemaphoreTake(retry_sem, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "Retry already in progress.  Function %u cancelled.", func);
        return false;
    }

    retry_function = func;

    do
    {
        if (retry != 0)
        {
            ESP_LOGE(TAG, "Control retry #%d in %dms", retry, retry_wait_ms);
            vTaskDelay(retry_wait_ms / portTICK_PERIOD_MS);
        }
        f();
        vTaskDelay(check_wait_ms / portTICK_PERIOD_MS);
        success = check();
        cec_queue_clear();
    } while (retry++ < TVMUX_MAX_COMMAND_RETRY && !success);

    xSemaphoreGive(retry_sem);

    retry_function = NO_RETRY_FUNC;

    return success;
}

bool check_retry_busy(retry_function_t func)
{
    if (xSemaphoreTake(retry_sem, 0) == pdTRUE)
    {
        xSemaphoreGive(retry_sem);

        return false;        
    }

    return retry_function == func;
}

bool tvmux_steam_state(bool and_mode)
{
    bool state;
    return tvmux_steam_state(and_mode, false, &state, NULL) == ESP_OK && state;
}

bool tvmux_steam_power_on()
{
    for (int i = 0; i < 10; i++)
    {
        if (tvmux_steam_is_on())
        {
            return true;
        }
        printf("send WOL (%d)\n", i + 1);
        //WOL.sendMagicPacket(TVMUX_STEAM_PC_MAC);
        // FIXME
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return false;
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

    // FIXME
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
    // FIXME
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
    // FIXME
}

bool check_steam_topology()
{
    // bool external = false;

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

// FIXME
    return false;
}

bool tvmux_steam_is_on()
{
    int pongs;
    return ping(TVMUX_STEAM_HOSTNAME, TVMUX_STEAM_PING_COUNT, TVMUX_STEAM_PING_INTERVAL, &pongs) == ESP_OK && pongs > 0;
}

bool tvmux_steam_is_open()
{
    // bool open = false;

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

// FIXME
    return false;
}

esp_err_t tvmux_steam_state(bool and_mode, bool exclusive, bool* state, bool* pending)
{
    if (pending != NULL)
    {        
        *pending = false;

        if (check_retry_busy(STEAM_RETRY_FUNC))
        {
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
            // FIXME
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
    steam_on_pending = power_on;

    xTaskCreate(tvmux_steam_state_task, "steam_state", 4000, (void*)power_on, 1, NULL);

    return ESP_OK;
}

esp_err_t tvmux_wii_power(bool power_on)
{
    wii_on_pending = power_on;

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

    return ESP_OK;
}

esp_err_t tvmux_cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{
    cec_control(target_address, request, request_size, reply_filter, reply, reply_size);

    return ESP_OK;
}

bool parse_edid(unsigned char* edid)
{
    uint32_t header0 = edid[0] << 24 | edid[1] << 16 | edid[2] << 8 | edid[3];
    uint32_t header1 = edid[4] << 24 | edid[5] << 16 | edid[6] << 8 | edid[7];

    uint32_t sum = 0;
    for (int i = 0; i < DDC_EDID_LENGTH; i++)
    {
        sum += (uint32_t)edid[i];
    }

    ESP_LOGI(TAG, "EDID checksum: %08lx\n", sum);
    ESP_LOGI(TAG, "EDID header: %08lx%08lx\n", header0, header1);

    if (header0 != 0x00ffffff || header1 != 0xffffff00)
    {
        return false;
    }

    if (sum % 256 != 0)
    {
        return false;
    }

    uint16_t manufacturer0 = edid[0x08];
    uint16_t manufacturer1 = edid[0x09];
    char manufacturer[4];
    manufacturer[0] = '@' + (int)((manufacturer0 >> 2) & 0x1f);
    manufacturer[1] = '@' + (int)((manufacturer0 & 3) << 3 | ((manufacturer1 >> 5) & 0x7));
    manufacturer[2] = '@' + (int)(manufacturer1 & 0x1f);
    manufacturer[3] = 0;

    uint8_t version = edid[0x12];
    uint8_t revision = edid[0x13];
    //uint8_t extension_flag = edid[EDID_EXTENSION_FLAG];

    ESP_LOGI(TAG, "EDID version: %u.%u\n", version, revision);
    ESP_LOGI(TAG, "EDID manufacturer: %s\n", manufacturer);
    //ESP_LOGI(TAG, "EDID extension_flag: %d\n", extension_flag);

    return true;
    /*

      uint8_t ieee0 = edid[0x95];
      uint8_t ieee1 = edid[0x96];
      uint8_t ieee2 = edid[0x97];
      uint16_t physicalAddress = edid[0x98] << 8 | edid[0x99];

      ESP_LOGI(TAG, "IEEE ID: %02x%02x%02x\n", ieee0, ieee1, ieee2);
      uint8_t a0 = physicalAddress >> 12;
      uint8_t a1 = (physicalAddress >> 8) & 0xf;
      uint8_t a2 = (physicalAddress >> 4) & 0xf;
      uint8_t a3 = physicalAddress & 0xf;
      ESP_LOGI(TAG, "CEC Physical Address: %u.%u.%u.%u\n", a0, a1, a2, a3);

      return true;*/
}

bool parse_edid_extension(uint8_t* edid2, uint8_t* ext)
{
    uint32_t sum = 0;
    for (int i = 0; i < DDC_EDID_EXTENSION_LENGTH; i++)
    {
        sum += (uint32_t)ext[i];
    }

    if (sum % 256 != 0)
    {
        return false;
    }

    //ESP_LOGI(TAG, "EDID ext checksum: %08x\n", sum);

    uint8_t tag = ext[0];
    //uint8_t revision = ext[1];
    uint8_t dtd_offset = ext[2];
    uint8_t offset = 4;

    // ESP_LOGI(TAG, "EDID ext tag: %u\n", tag);
    // ESP_LOGI(TAG, "EDID ext revision: %u\n", revision);
    // ESP_LOGI(TAG, "EDID ext dtd_offset: %u\n", dtd_offset);

    if (tag != 2)
    {
        return false;
    }

    // for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
    // {
    //   ESP_LOGI(TAG, "0x%02x, ", ext[i]);
    // }
    // ESP_LOGI(TAG, );

    uint8_t index = offset;

    while (index < dtd_offset)
    {
        uint8_t* p = ext + index;
        uint8_t tag = p[0] >> 5;
        uint8_t length = p[0] & 0x1f;

        //ESP_LOGI(TAG, "EDID ext tag: %d length: %d\n", tag, length);

        switch (tag)
        {
            case 3:
            {
                uint8_t ieee[3];
                ieee[0] = p[3];
                ieee[1] = p[2];
                ieee[2] = p[1];
                //ESP_LOGI(TAG, "EDID IEEE %02x %02x %02x\n", ieee[0], ieee[1], ieee[2]);
                if (ieee[0] == 0x00 && ieee[1] == 0x0c && ieee[2] == 0x03)
                {
                    cec_physical_address = (uint16_t)p[4] << 8 | p[5];
                    uint8_t a0 = cec_physical_address >> 12;
                    uint8_t a1 = (cec_physical_address >> 8) & 0xf;
                    uint8_t a2 = (cec_physical_address >> 4) & 0xf;
                    uint8_t a3 = cec_physical_address & 0xf;

                    ESP_LOGI(TAG, "CEC Physical Address: %u.%u.%u.%u\n", a0, a1, a2, a3);

                }
                break;
            }
        }

        index += 1 + length;
    }

    return true;
}

double uptimed()
{
    return esp_timer_get_time() / 1000000.0;
}

esp_err_t tvmux_tv_pending_get(bool* pending)
{
    *pending = tv_on_pending;

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
        ESP_RETURN_ON_ERROR(cec_combine_devices_state(tv_state, false), TAG, "%s/tvmux_combine_devices_state failed (%s)", __func__, esp_err_to_name(err_rc_));
    }

    return ESP_OK;
}

esp_err_t tvmux_wii_state_get(wii_power_state_t* wii_power_state, bool* pending)
{
    *pending = false;

    if (check_retry_busy(WII_RETRY_FUNC))
    {
        *wii_power_state = wii_on_pending ? WII_POWER_STATE_ON : WII_POWER_STATE_OFF;
        *pending = true;
    }
    else
    {
        *wii_power_state = wii_query_power_state();
    }

    return ESP_OK;
}

void tvmux_steam_state_task(void* param)
{
    bool desired_state = (bool)param;

    ESP_LOGI(TAG, "set_steam_state = %u requested", desired_state);

    void (*change_state)() = NULL;

    if (desired_state)
    {
        change_state = [] {
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
        };
    }
    else
    {
        change_state = [] { 
            ESP_LOGI(TAG, "turn steam off\n");
            cec_standby();
            tvmux_steam_close();
            steam_power_off();
        };
    }

    if (tvmux_call_with_retry(STEAM_RETRY_FUNC, change_state, [desired_state] { return tvmux_steam_state(desired_state) == desired_state; }, 2000, 5000))
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
    tv_on_pending = power_on;

    return cec_tv_power(power_on);
}
