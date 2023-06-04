#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "ddc.h"
#include "cec.h"
#include "cec_int.h"
#include "TvMux.h"

esp_err_t cec_init()
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = HDMI_HOTPLUG_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = HDMI_CEC_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = HDMI_CEC_GPIO_2_SEL;
    io_conf.mode = GPIO_MODE_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_NUM, 1));
    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_2_NUM, 1));

    return ESP_OK;
}

esp_err_t cec_queue_clear()
{

    return ESP_OK;
}

esp_err_t cec_standby()
{

    return ESP_OK;
}

esp_err_t cec_pause()
{

    return ESP_OK;
}

esp_err_t cec_play()
{

    return ESP_OK;
}

esp_err_t cec_tv_power(bool power_on)
{

    return ESP_OK;
}

esp_err_t cec_wii_power(bool power_on)
{

    return ESP_OK;
}

esp_err_t cec_log_write(httpd_req_t* request)
{

    return ESP_OK;
}

esp_err_t cec_log_clear()
{

    return ESP_OK;
}

esp_err_t cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{

    return ESP_OK;
}

esp_err_t cec_sam_request(uint16_t addr)
{

    return ESP_OK;
}

esp_err_t cec_as_set(uint16_t addr)
{

    return ESP_OK;
}

esp_err_t cec_tv_on()
{

    return ESP_OK;
}

esp_err_t cec_combine_devices_state(bool* state, bool and_mode, bool tv, bool audio, bool atv)
{

    return ESP_OK;
}

