#ifndef TVMUX_H
#define TVMUX_H

#include "esp_http_server.h"
#include "nvs_flash.h"
#include "wii.h"

#ifdef __cplusplus
#include <functional>
extern "C" 
{
#endif

#define TVMUX_MAX_COMMAND_RETRY          3

#ifdef __cplusplus
extern "C" nvs_handle config_nvs_handle;
#else
extern nvs_handle config_nvs_handle;
#endif

typedef enum 
{
    NO_RETRY_FUNC,
    TV_RETRY_FUNC,
    STEAM_RETRY_FUNC,
    WII_RETRY_FUNC
} retry_function_t;

esp_err_t tvmux_init();
esp_err_t tvmux_steam_power(bool power_on);
esp_err_t tvmux_wii_power(bool power_on);
esp_err_t tvmux_cec_log_write(httpd_req_t* request);
esp_err_t tvmux_cec_log_clear();
esp_err_t tvmux_cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size);
esp_err_t tvmux_tv_state_get(bool* tv_state, bool* pending);
esp_err_t tvmux_wii_state_get(wii_power_state_t* wii_power_state, bool* pending);
esp_err_t tvmux_steam_state(bool and_mode, bool exclusive, bool* state, bool* pending);
esp_err_t tvmux_tv_power(bool power_on);

#ifdef __cplusplus
bool tvmux_call_with_retry(retry_function_t func, std::function<void()> f, std::function<bool()> check, int check_wait_ms, int retry_wait_ms);


}
#endif

#endif