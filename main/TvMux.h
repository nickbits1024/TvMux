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

// #ifdef __cplusplus
// extern "C" nvs_handle config_nvs_handle;
// #else
// extern nvs_handle config_nvs_handle;
// #endif

typedef enum 
{
    NO_RETRY_FUNC,
    TV_RETRY_FUNC,
    STEAM_RETRY_FUNC,
    WII_RETRY_FUNC
} tvmux_retry_type_t;

typedef void (*tvmux_retry_func_t)(void* param);
typedef bool (*tvmux_retry_check_func_t)(void* param);

esp_err_t tvmux_init(bool* setup_enabled);
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
bool tvmux_call_with_retry(tvmux_retry_type_t retry_type, tvmux_retry_func_t f, tvmux_retry_check_func_t check, void* retry_param);


}
#endif

#define HDMI_CEC

#endif