#include <string.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <ESP32Ping.h>
#include <WiFiUdp.h>
#include <WakeOnLan.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include "cJSON.h"
#include <list>
#include <functional>
#include "hometv.h"
#include "cec.h"
#include "wii.h"

#define TAG "main"

#define HDMI_CEC

#define WIFI_SSID             "wifi.nickpalmer.net"
#define WIFI_PASS             "B16b00b5"

#define HOTPLUG_GPIO          19

#define TV_HDMI_INPUT           1
#define ATV_HDMI_INPUT          1
#define WII_HDMI_INPUT          2
#define STEAM_HDMI_INPUT        4

#define STEAM_PC_HOSTNAME       "seattle.home.nickpalmer.net"
#define STEAM_PC_PORT           1410
#define STEAM_PC_MAC            "58:11:22:B4:B6:D9"
#define STEAM_PC_PING_COUNT     3

#define HTTP_SUCCESS(http_code) ((http_code) >= 200 && (http_code) <= 299)

uint16_t cec_physical_address;
HomeTvCec device;
bool wii_pair_request;
nvs_handle config_nvs_handle;
httpd_handle_t http_server_handle;
WiFiUDP UDP;
WakeOnLan WOL(UDP);
bool device_state[CEC_MAX_ADDRESS + 1];

void cec_loop(void* param);

void connect_wiFi()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    do
    {
        WiFi.mode(WIFI_OFF);
        delay(1000);
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();  //ESP has tendency to store old SSID and PASSword and tries to connect
        delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        Serial.print("Connecting to WiFi...");
        for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++)
        {
            delay(2000);
            Serial.print(".");
        }

    } while (WiFi.status() != WL_CONNECTED);
    Serial.println();
    Serial.println("Connected to WiFi");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
}

bool call_with_retry(std::function<bool()> f, int wait_ms)
{
    int retry = 0;
    bool success;
    do
    {
        if (retry != 0)
        {
            printf("control retry #%d\n", retry);
            delay(wait_ms);
        }
        success = f();
    } while (retry++ < MAX_COMMAND_RETRY && !success);

    return success;
}

bool call_with_retry(std::function<void()> f, std::function<bool()> check, int check_wait_ms, int retry_wait_ms)
{
    int retry = 0;
    bool success;
    do
    {
        if (retry != 0)
        {
            printf("control retry #%d in %dms\n", retry, retry_wait_ms);
            delay(retry_wait_ms);
        }
        f();
        delay(check_wait_ms);
        success = check();
        device.ClearPending();
    } while (retry++ < MAX_COMMAND_RETRY && !success);

    return success;
}

void complete_request(httpd_req_t* request, cJSON* response_doc)
{
    char* json = cJSON_Print(response_doc);
    if (json != NULL)
    {
        httpd_resp_set_type(request, "application/json");
        httpd_resp_send(request, json, HTTPD_RESP_USE_STRLEN);
        cJSON_free(json);
    }
    else
    {
        httpd_resp_send_500(request);
    }
    cJSON_Delete(response_doc);
}

esp_err_t handle_heap(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_INTERNAL);

    cJSON_AddNumberToObject(response_doc, "total_free_bytes", heap_info.total_free_bytes);
    cJSON_AddNumberToObject(response_doc, "total_allocated_bytes", heap_info.total_allocated_bytes);
    cJSON_AddNumberToObject(response_doc, "free_size", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(response_doc, "minimum_free_size", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(response_doc, "largest_free_block", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t handle_standby_get(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    device.StandBy();

    cJSON_AddStringToObject(response_doc, "status", "ok");

    complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t handle_tv_pause_get(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    device.UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, 0x46);

    cJSON_AddStringToObject(response_doc, "status", "ok");

    complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t handle_tv_play_get(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    device.UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, 0x44);

    cJSON_AddStringToObject(response_doc, "status", "ok");

    complete_request(request, response_doc);

    return ESP_OK;
}

bool combine_devices_state(bool and_mode, bool tv = true, bool audio = true, bool atv = true)
{
#ifndef HDMI_CEC
    return false;
#endif

    bool combined_on = and_mode;

    // auto check_device = [and_mode, &tv_on](int target_address) {
    //     uint8_t reply[CEC_MAX_MSG_SIZE];
    //     auto get_power_state = [target_address, &reply]() {
    //         uint8_t cmd[] = { 0x8f };
    //         int reply_size = CEC_MAX_MSG_SIZE;
    //         return device.Control(target_address, cmd, sizeof(cmd), CEC_POWER_STATUS, reply, &reply_size) && reply_size == 3;
    //     };

    //     if ((!tv_on && !and_mode) || (tv_on && and_mode))
    //     {
    //         if (call_with_retry(get_power_state, 100))
    //         {
    //             //printf("reply %02x:%02x:%02x\n", reply[0], reply[1], reply[2]);
    //             bool device_on = reply[2] == 0 || reply[2] == 2;
    //             if (and_mode)
    //             {
    //                 tv_on &= device_on;
    //             }
    //             else
    //             {
    //                 tv_on |= device_on;
    //             }
    //         }
    //         else
    //         {
    //             if (and_mode)
    //             {
    //                 printf("control failed, assuming off\n");
    //                 tv_on = false;
    //             }
    //             else
    //             {
    //                 printf("control failed, keeping tv_on=%u\n", tv_on);
    //             }
    //         }
    //     }
    // };

    auto check_device = [and_mode, &combined_on](int target_address) {

        auto power_state = device.GetPowerState(target_address);
        bool device_on = power_state == CEC_POWER_ON || power_state == CEC_POWER_TRANS_ON;

        if (and_mode)
        {
            combined_on &= device_on;
        }
        else
        {
            combined_on |= device_on;
        }
    };

    if (tv)
    {
        printf("tv on?\n");
        check_device(CEC_TV_ADDRESS);
    }
    if (audio)
    {
        printf("avr on?\n");
        check_device(CEC_AUDIO_SYSTEM_ADDRESS);
    }
    if (atv)
    {
        printf("atv on?\n");
        check_device(CEC_PLAYBACK_DEVICE_1_ADDRESS);
    }

    return combined_on;
}

esp_err_t handle_tv_get(httpd_req_t* request)
{
    auto state = combine_devices_state(false) ? "on" : "off";

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddStringToObject(response_doc, "state", state);

    complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t handle_wii_get(httpd_req_t* request)
{
    auto state = wii_query_power_state();
    auto state_string = state == WII_POWER_STATE_ON ? "on" : state == WII_POWER_STATE_OFF ? "off" : "error";

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddStringToObject(response_doc, "state", state_string);

    complete_request(request, response_doc);

    return ESP_OK;
}

cJSON* parse_request(httpd_req_t* request)
{
    char json[MAX_REQUEST_SIZE];

    if (request->content_len > MAX_REQUEST_SIZE)
    {
        return NULL;
    }

    int ret = httpd_req_recv(request, json, request->content_len);
    if (ret <= 0)
    {
        return NULL;
    }

    return cJSON_Parse(json);
}

esp_err_t handle_tv_post(httpd_req_t* request)
{
    auto request_doc = parse_request(request);
    if (request_doc == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    auto response_doc = cJSON_CreateObject();

    cJSON* state = cJSON_GetObjectItem(request_doc, "state");

    if (state != NULL)
    {
        auto state_value = cJSON_GetStringValue(state);
        auto desired_state = false;
        void (*change_state)() = NULL;

        if (strcmp(state_value, "on") == 0)
        {
            printf("turn tv on\n");
            desired_state = true;
            change_state = [] {
                uint16_t addr = TV_HDMI_INPUT << 12 | ATV_HDMI_INPUT << 8;
                device.SetActiveSource(addr);
                device.TvScreenOn();
                device.SystemAudioModeRequest(addr);
                device.UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, CEC_USER_CONTROL_POWER_ON);
            };
        }
        else if (strcmp(state_value, "off") == 0)
        {
            printf("turn tv off\n");
            desired_state = false;
            change_state = [] { device.StandBy(); };
        }

        if (change_state != NULL)
        {
            if (call_with_retry(change_state, [desired_state] { return combine_devices_state(desired_state) == desired_state; }, 10000, 5000))
            {
                printf("change_tv_state = %u succeeded\n", desired_state);
            }
            else
            {
                printf("change_tv_state = %u failed\n", desired_state);
            }
        }
    }

    auto state_string = combine_devices_state(false) ? "on" : "off";

    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddStringToObject(response_doc, "status", "ok");

    complete_request(request, response_doc);
    cJSON_Delete(request_doc);

    return ESP_OK;
}

bool check_steam_topology()
{
    bool external = false;

    HTTPClient http;

    String host(STEAM_PC_HOSTNAME);
    uint16_t port = STEAM_PC_PORT;
    String path("/api/monitor/topology");
   
    http.setTimeout(30000);
    http.begin(host, port, path);
    int http_code;
    if (HTTP_SUCCESS(http_code = http.GET()))
    {
        auto json = http.getString();
        cJSON* doc = cJSON_Parse(json.c_str());
        if (doc != NULL)
        {
            auto topology_prop = cJSON_GetObjectItem(doc, "topology");
            if (topology_prop != NULL)
            {
                auto topology_value = cJSON_GetStringValue(topology_prop);
                if (topology_value != NULL)
                {
                    external = strcasecmp(topology_value, "External") == 0;
                    ESP_LOGI(TAG, "topology: %s", topology_value);
                }
            }
            cJSON_Delete(doc);
        }
    }
    else
    {
        ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    }
    http.end();

    return external;
}

bool steam_state(bool and_mode)
{
    auto state = combine_devices_state(and_mode, true, true, false);

    ESP_LOGI(TAG, "steam status start %d", state);

    if (and_mode)
    {
        if (state)
        {
            state &= Ping.ping(STEAM_PC_HOSTNAME, STEAM_PC_PING_COUNT);
            ESP_LOGI(TAG, "steam status ping %d", state);            
        }
        if (state)
        {
            state &= check_steam_topology();
            ESP_LOGI(TAG, "steam status topology %d", state);            
        }
    }
    else
    {
        if (!state)
        {
            state |= Ping.ping(STEAM_PC_HOSTNAME, STEAM_PC_PING_COUNT);
        }
        // if (!state)
        // {
        //     state |= check_steam_topology();
        // }
    }
    ESP_LOGI(TAG, "steam status end %d", state);
     
    return state;
}

bool steam_is_on()
{
    return Ping.ping(STEAM_PC_HOSTNAME, STEAM_PC_PING_COUNT);
}

bool steam_power_on()
{
    for (int i = 0; i < 10; i++)
    {
        if (steam_is_on())
        {
            return true;
        }
        printf("send WOL (%d)\n", i + 1);
        WOL.sendMagicPacket(STEAM_PC_MAC);
        delay(2000);
    }
    return false;
}

void steam_start()
{
    HTTPClient http;

    String host(STEAM_PC_HOSTNAME);
    uint16_t port = STEAM_PC_PORT;
    String path("/api/steam/bigpicture/start");
   
    int http_code;
    http.setTimeout(30000);
    http.begin(host, port, path);
    if (HTTP_SUCCESS(http_code = http.GET()))
    {
        auto json = http.getString();
        ESP_LOGI(TAG, "%s: %s", path.c_str(), json.c_str());      
    }
    else
    {
        ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    }
    http.end();
}

void steam_power_off()
{
    HTTPClient http;

    String host(STEAM_PC_HOSTNAME);
    uint16_t port = STEAM_PC_PORT;
    String path("/api/computer/off");
   
    int http_code;
    http.setTimeout(30000);
    http.begin(host, port, path);
    if (HTTP_SUCCESS(http_code = http.GET()))
    {
        auto json = http.getString();
        ESP_LOGI(TAG, "%s: %s", path.c_str(), json.c_str());      
    }
    else
    {
        ESP_LOGE(TAG, "%s: http %d %s", path.c_str(), http_code, http.errorToString(http_code).c_str());
    }
    http.end();
}

esp_err_t handle_steam_get(httpd_req_t* request)
{
    auto state = steam_state(false) ? "on" : "off";

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddStringToObject(response_doc, "state", state);

    complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t handle_steam_post(httpd_req_t* request)
{
    auto request_doc = parse_request(request);
    if (request_doc == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    auto response_doc = cJSON_CreateObject();

    cJSON* state = cJSON_GetObjectItem(request_doc, "state");

    if (state != NULL)
    {
        auto state_value = cJSON_GetStringValue(state);
        auto desired_state = false;
        void (*change_state)() = NULL;

        if (strcmp(state_value, "on") == 0)
        {
            desired_state = true;
            change_state = [] {
                uint16_t addr = TV_HDMI_INPUT << 12 | STEAM_HDMI_INPUT << 8;
                ESP_LOGI(TAG, "powering on steam...");
                if (!steam_is_on())
                {
                    ESP_LOGI(TAG, "pc off, powering on...");
                    if (!steam_power_on())
                    {
                        return;
                    }
                    delay(10000);
                }
                device.SetActiveSource(addr);
                device.SystemAudioModeRequest(addr);               
                delay(10000);
                ESP_LOGI(TAG, "opening steam...");
                steam_start();
                device.TvScreenOn();
            };
        }
        else if (strcmp(state_value, "off") == 0)
        {
            desired_state = false;
            change_state = [] { 
                printf("turn steam off\n");
                device.StandBy(); 
                steam_power_off();
            };
        }

        if (change_state != NULL)
        {
            if (call_with_retry(change_state, [desired_state] { return steam_state(desired_state) == desired_state; }, 2000, 5000))
            {
                printf("change_tv_state = %u succeeded\n", desired_state);
            }
            else
            {
                printf("change_tv_state = %u failed\n", desired_state);
            }
        }
    }

    auto state_string = steam_state(false) ? "on" : "off";

    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddStringToObject(response_doc, "status", "ok");

    complete_request(request, response_doc);
    cJSON_Delete(request_doc);

    return ESP_OK;
}

esp_err_t handle_wii_post(httpd_req_t* request)
{
    auto request_doc = parse_request(request);
    if (request_doc == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    auto response_doc = cJSON_CreateObject();

    cJSON* state = cJSON_GetObjectItem(request_doc, "state");

    wii_power_status_t status = WII_POWER_STATUS_UNKNOWN;
    wii_power_status_t(*change_state)() = NULL;
    bool desired_av_state;
    wii_power_state_t desired_wii_state;

    if (state != NULL)
    {
        auto state_value = cJSON_GetStringValue(state);
        if (strcmp(state_value, "on") == 0)
        {
            desired_wii_state = WII_POWER_STATE_ON;
            desired_av_state = true;
            change_state = [] {
                auto status = wii_power_on();
                uint16_t addr = TV_HDMI_INPUT << 12 | WII_HDMI_INPUT << 8;
                device.SetActiveSource(addr);
                device.TvScreenOn();
                device.SystemAudioModeRequest(addr);
                //delay(5000);
                //device.SetSystemAudioMode(true);
                return status;
            };
        }
        else if (strcmp(state_value, "off") == 0)
        {
            desired_wii_state = WII_POWER_STATE_OFF;
            desired_av_state = false;
            change_state = [] {
                auto status = wii_power_off();
                device.StandBy();
                return status;
            };
        }
    }

    if (change_state != NULL)
    {
        if (!call_with_retry(change_state, [desired_av_state, desired_wii_state]
            { return wii_query_power_state() == desired_wii_state &&
            combine_devices_state(desired_av_state, true, true, false) == desired_av_state; }, 5000, 1000))
        {
            printf("wii control failed\n");
        }
    }

    cJSON_AddBoolToObject(response_doc, "power_toggled", status == WII_POWER_STATUS_TOGGLED);
    cJSON_AddStringToObject(response_doc, "status", "ok");

    if (status == WII_POWER_STATUS_ERROR)
    {
        cJSON_AddStringToObject(response_doc, "state", "error");
    }
    else
    {
        wii_power_state_t state = wii_query_power_state();
        auto state_string = state == WII_POWER_STATE_ON ? "on" : state == WII_POWER_STATE_OFF ? "off" : "error";
        cJSON_AddStringToObject(response_doc, "state", state_string);
    }

    complete_request(request, response_doc);
    cJSON_Delete(request_doc);

    return ESP_OK;
}

esp_err_t handle_cec_log_get(httpd_req_t* request)
{
    httpd_resp_set_type(request, "text/plain");
    
    device.WriteLog(request);

    size_t qs_size = httpd_req_get_url_query_len(request) + 1;
    char* qs = new char[qs_size];

    if (qs == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    if (httpd_req_get_url_query_str(request, qs, qs_size) != ESP_OK)
    {
        delete[] qs;
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    char clear_string[10];
    bool clear = false;

    if (httpd_query_key_value(qs, "clear", clear_string, sizeof(clear_string)) == ESP_OK)
    {
        clear = atoi(clear_string) > 0;
    }

    delete[] qs;

    if (clear)
    {
        device.ClearLog();
    }    

    return ESP_OK;
}

esp_err_t handle_cec_get(httpd_req_t* request)
{
    size_t qs_size = httpd_req_get_url_query_len(request) + 1;
    char* qs = new char[qs_size];

    if (qs == NULL)
    {
        return ESP_FAIL;
    }

    if (httpd_req_get_url_query_str(request, qs, qs_size) != ESP_OK)
    {
        delete[] qs;
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    char addr_string[3];
    char cmd_string[CEC_MAX_MSG_SIZE * 3];

    if (httpd_query_key_value(qs, "addr", addr_string, sizeof(addr_string)) != ESP_OK ||
        httpd_query_key_value(qs, "cmd", cmd_string, sizeof(cmd_string)) != ESP_OK)
    {
        delete[] qs;
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    char reply_cmd_string[3];
    bool has_reply = httpd_query_key_value(qs, "reply", reply_cmd_string, sizeof(reply_cmd_string)) == ESP_OK;

    delete[] qs;

    int addr = atoi(addr_string);
    if (addr == -1)
    {
        addr = 0xf;
    }

    uint8_t cmd[CEC_MAX_MSG_SIZE];
    uint8_t reply[CEC_MAX_MSG_SIZE];
    int len = (strlen(cmd_string) + 1) / 3;
    if (len == 0 ||
        len > sizeof(cmd) ||
        (has_reply && strlen(reply_cmd_string) != 2))
    {
        delete[] qs;
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    printf("cec cmd=%s reply_command=%s\n", cmd_string, reply_cmd_string);
    const char* hex = cmd_string;
    unsigned int temp;
    for (int i = 0; i < len; i++)
    {
        sscanf(hex + i * 3, "%02x", &temp);
        cmd[i] = (unsigned char)temp;
    }

    int reply_filter = -1;
    if (has_reply)
    {
        hex = reply_cmd_string;
        sscanf(hex, "%02x", &reply_filter);
    }

    std::string reply_string;
    const char* status = "ok";

    if (has_reply)
    {
        int reply_size = CEC_MAX_MSG_SIZE;
        if (device.Control(addr, cmd, len, (uint8_t)reply_filter, reply, &reply_size))
        {
            format_bytes(reply_string, reply + 1, reply_size - 1);
        }
        else
        {
            status = "error";
        }
    }
    else
    {
        if (!device.Control(addr, cmd, len, 0, NULL, NULL))
        {
            status = "error";
        }
    }

    auto response_doc = cJSON_CreateObject();

    cJSON_AddNumberToObject(response_doc, "addr", addr);
    cJSON_AddStringToObject(response_doc, "cmd", cmd_string);
    cJSON_AddStringToObject(response_doc, "status", status);
    if (has_reply)
    {
        cJSON_AddStringToObject(response_doc, "reply", reply_string.c_str());
    }

    complete_request(request, response_doc);

    return ESP_OK;
}

bool parse_edid(unsigned char* edid)
{
    uint32_t header0 = edid[0] << 24 | edid[1] << 16 | edid[2] << 8 | edid[3];
    uint32_t header1 = edid[4] << 24 | edid[5] << 16 | edid[6] << 8 | edid[7];

    uint32_t sum = 0;
    for (int i = 0; i < EDID_LENGTH; i++)
    {
        sum += (uint32_t)edid[i];
    }

    //Serial.printf("EDID checksum: %08x\n", sum);
    //Serial.printf("EDID header: %08x%08x\n", header0, header1);

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

    Serial.printf("EDID version: %u.%u\n", version, revision);
    Serial.printf("EDID manufacturer: %s\n", manufacturer);
    //Serial.printf("EDID extension_flag: %d\n", extension_flag);

    return true;
    /*

      uint8_t ieee0 = edid[0x95];
      uint8_t ieee1 = edid[0x96];
      uint8_t ieee2 = edid[0x97];
      uint16_t physicalAddress = edid[0x98] << 8 | edid[0x99];

      Serial.printf("IEEE ID: %02x%02x%02x\n", ieee0, ieee1, ieee2);
      uint8_t a0 = physicalAddress >> 12;
      uint8_t a1 = (physicalAddress >> 8) & 0xf;
      uint8_t a2 = (physicalAddress >> 4) & 0xf;
      uint8_t a3 = physicalAddress & 0xf;
      Serial.printf("CEC Physical Address: %u.%u.%u.%u\n", a0, a1, a2, a3);

      return true;*/
}

bool parse_edid_extension(uint8_t* edid2, uint8_t* ext)
{
    uint32_t sum = 0;
    for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
    {
        sum += (uint32_t)ext[i];
    }

    if (sum % 256 != 0)
    {
        return false;
    }

    //Serial.printf("EDID ext checksum: %08x\n", sum);

    uint8_t tag = ext[0];
    //uint8_t revision = ext[1];
    uint8_t dtd_offset = ext[2];
    uint8_t offset = 4;

    // Serial.printf("EDID ext tag: %u\n", tag);
    // Serial.printf("EDID ext revision: %u\n", revision);
    // Serial.printf("EDID ext dtd_offset: %u\n", dtd_offset);

    if (tag != 2)
    {
        return false;
    }

    // for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
    // {
    //   Serial.printf("0x%02x, ", ext[i]);
    // }
    // Serial.println();

    uint8_t index = offset;

    while (index < dtd_offset)
    {
        uint8_t* p = ext + index;
        uint8_t tag = p[0] >> 5;
        uint8_t length = p[0] & 0x1f;

        //Serial.printf("EDID ext tag: %d length: %d\n", tag, length);

        switch (tag)
        {
            case 3:
            {
                uint8_t ieee[3];
                ieee[0] = p[3];
                ieee[1] = p[2];
                ieee[2] = p[1];
                //Serial.printf("EDID IEEE %02x %02x %02x\n", ieee[0], ieee[1], ieee[2]);
                if (ieee[0] == 0x00 && ieee[1] == 0x0c && ieee[2] == 0x03)
                {
                    cec_physical_address = (uint16_t)p[4] << 8 | p[5];
                    uint8_t a0 = cec_physical_address >> 12;
                    uint8_t a1 = (cec_physical_address >> 8) & 0xf;
                    uint8_t a2 = (cec_physical_address >> 4) & 0xf;
                    uint8_t a3 = cec_physical_address & 0xf;

                    Serial.printf("CEC Physical Address: %u.%u.%u.%u\n", a0, a1, a2, a3);

                }
                break;
            }
        }

        index += 1 + length;
    }

    return true;
}

byte readI2CByte(byte data_addr)
{
    byte data = 0xff;
    Wire.beginTransmission(EDID_ADDRESS);
    Wire.write(data_addr);
    Wire.endTransmission();
    Wire.requestFrom(EDID_ADDRESS, 1); //retrieve 1 returned byte
    delay(1);
    if (Wire.available())
    {
        data = Wire.read();
    }
    return data;
}

double uptimed()
{
    return esp_timer_get_time() / 1000000.0;
}

void IRAM_ATTR wii_pair_irq_handler()
{
    wii_pair_request = true;
}

void setup()
{
    digitalWrite(ONBOARD_LED_GPIO, HIGH);
    pinMode(WII_PAIR_GPIO, INPUT_PULLUP);
    pinMode(HOTPLUG_GPIO, INPUT_PULLUP);
    pinMode(CEC_GPIO_INPUT, INPUT_PULLUP);
    pinMode(CEC_GPIO_OUTPUT, OUTPUT_OPEN_DRAIN);
    digitalWrite(CEC_GPIO_OUTPUT, HIGH);
    pinMode(ONBOARD_LED_GPIO, OUTPUT);
    delay(50);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    err = nvs_open("default", NVS_READWRITE, &config_nvs_handle);
    ESP_ERROR_CHECK(err);

    // disableCore0WDT();
    // disableCore1WDT();
    // disableLoopWDT(); 

    Serial.begin(115200);
    while (!Serial) delay(50);

    attachInterrupt(digitalPinToInterrupt(WII_PAIR_GPIO), wii_pair_irq_handler, FALLING);


#ifdef HDMI_CEC
    Serial.println("Waiting for hotplug signal...");
    while (digitalRead(HOTPLUG_GPIO) == LOW) delay(50);

    Serial.printf("Hotplug signal detected!\n");
    //last_hpd_time = uptimed();

    Wire.begin();
    uint8_t edid[EDID_LENGTH];
    uint8_t edid_extension[EDID_EXTENSION_LENGTH];
    do
    {
        for (int i = 0; i < EDID_LENGTH; i++)
        {
            edid[i] = readI2CByte(i);
        }

        Serial.println("Received EDID");

    } while (!parse_edid(edid));

    if (edid[EDID_EXTENSION_FLAG])
    {
        do
        {
            for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
            {
                edid_extension[i] = readI2CByte(EDID_LENGTH + i);
            }

            Serial.println("Received EDID extension");

        } while (!parse_edid_extension(edid, edid_extension));
    }

    device.Initialize(cec_physical_address, CEC_DEVICE_TYPE, true); // Promiscuous mode}
    xTaskCreate(cec_loop, "cec_loop", 10000, NULL, 1, NULL);
#endif

    wii_init();
    connect_wiFi();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;

    ESP_ERROR_CHECK(httpd_start(&http_server_handle, &config));

    httpd_uri_t httpd_uri = { };

    httpd_uri =
    {
        .uri = "/heap",
        .method = HTTP_GET,
        .handler = handle_heap,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/tv",
      .method = HTTP_GET,
      .handler = handle_tv_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/tv",
      .method = HTTP_POST,
      .handler = handle_tv_post
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/steam",
      .method = HTTP_GET,
      .handler = handle_steam_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/steam",
      .method = HTTP_POST,
      .handler = handle_steam_post
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/wii",
      .method = HTTP_GET,
      .handler = handle_wii_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/wii",
      .method = HTTP_POST,
      .handler = handle_wii_post
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/cec/log",
      .method = HTTP_GET,
      .handler = handle_cec_log_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/cec",
      .method = HTTP_GET,
      .handler = handle_cec_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/standby",
      .method = HTTP_GET,
      .handler = handle_standby_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/tv/play",
      .method = HTTP_GET,
      .handler = handle_tv_play_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));

    httpd_uri =
    {
      .uri = "/tv/pause",
      .method = HTTP_GET,
      .handler = handle_tv_pause_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &httpd_uri));


    device.LoadPowerState(CEC_TV_ADDRESS);
    device.LoadPowerState(CEC_AUDIO_SYSTEM_ADDRESS);
    device.LoadPowerState(CEC_PLAYBACK_DEVICE_1_ADDRESS);

    digitalWrite(ONBOARD_LED_GPIO, LOW);

}

void loop()
{
#ifdef HDMI_CEC
    static double hpd_time;
#endif
    connect_wiFi();

#ifdef HDMI_CEC
    bool hpd = digitalRead(HOTPLUG_GPIO) == HIGH;
    if (hpd)
    {
        hpd_time = uptimed();
    }

    if (!hpd && (uptimed() - hpd_time) > 5.0)
    {
        Serial.println("HDMI unplugged, rebooting in 1s...");
        delay(1000);
        ESP.restart();
    }
#endif
    if (wii_pair_request)
    {
        wii_pair_request = false;
        wii_pair();
    }

    delay(500);
}

void cec_loop(void* param)
{
    while (1)
    {
        device.Run();
    }
}