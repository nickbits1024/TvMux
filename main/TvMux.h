#ifndef TVMUX_H
#define TVMUX_H

#include "esp_http_server.h"
#include "nvs_flash.h"
#include "wii.h"

typedef enum 
{
    NO_RETRY_FUNC,
    TV_RETRY_FUNC,
    STEAM_RETRY_FUNC,
    WII_RETRY_FUNC
}
tvmux_retry_type_t;

typedef void (*tvmux_retry_func_t)(void* param);
typedef bool (*tvmux_retry_check_func_t)(void* param);

esp_err_t tvmux_init(bool* setup_enabled);

esp_err_t tvmux_tv_power(bool power_on);
esp_err_t tvmux_wii_power(bool power_on);
esp_err_t tvmux_steam_power(bool power_on);

esp_err_t tvmux_tv_state_get(bool* tv_state, bool* pending);
esp_err_t tvmux_wii_state_get(wii_power_state_t* wii_power_state, bool* pending);
esp_err_t tvmux_steam_state(bool and_mode, bool exclusive, bool* state, bool* pending);

#define HDMI_CEC

#endif