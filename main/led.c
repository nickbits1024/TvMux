#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "led_strip.h"
#include "led_int.h"
#include "led.h"

#define TAG "LED"

bool led_enabled = true;
uint8_t last_r;
uint8_t last_g;
uint8_t last_b;
led_strip_t* led_strip;

portMUX_TYPE led_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t led_history[LED_HISTORY_SIZE][3];
uint8_t led_history_index;

esp_err_t led_init()
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = LED_GPIO_SEL;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // vTaskDelay(10000 / portTICK_PERIOD_MS);

    // printf("swapping...\n");
    // gpio_set_level(LED_GPIO_NUM, 0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // gpio_set_level(LED_GPIO_NUM, 1);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // gpio_set_level(LED_GPIO_NUM, 0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // gpio_set_level(LED_GPIO_NUM, 1);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // gpio_set_level(LED_GPIO_NUM, 0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // printf("end swapping...\n");

    led_strip = led_strip_init(LED_CHANNEL, LED_GPIO_NUM, 1);
    ets_delay_us(5000);

    ESP_ERROR_CHECK(led_strip->set_pixel(led_strip, 0, 0, 0, 0));
    ESP_ERROR_CHECK(led_strip->refresh(led_strip, 100));

    return ESP_OK;
}

esp_err_t led_enable(bool enable)
{
    led_enabled = enable;

    if (enable)
    {
        ESP_ERROR_CHECK(led_strip->set_pixel(led_strip, 0, last_r, last_g, last_b));
        ESP_ERROR_CHECK(led_strip->refresh(led_strip, 100));
    }
    else
    {
        ESP_ERROR_CHECK(led_strip->clear(led_strip, 100));
    }

    return ESP_OK;
}

esp_err_t led_set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    ESP_LOGI(TAG, "led set (%d, %d, %d) en=%d", r, g, b, led_enabled);
    if (led_enabled)
    {
        ESP_ERROR_CHECK(led_strip->set_pixel(led_strip, 0, r, g, b));
        ESP_ERROR_CHECK(led_strip->refresh(led_strip, 100));
    }

    last_r = r;
    last_g = g;
    last_b = b;

    return ESP_OK;
}

esp_err_t led_set_rgb_color_isr(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_enabled)
    {
        ESP_ERROR_CHECK(led_strip->set_pixel(led_strip, 0, r, g, b));
        ESP_ERROR_CHECK(led_strip->refresh(led_strip, 100));
    }

    return ESP_OK;
}

esp_err_t led_get_rgb_color(uint8_t* r, uint8_t* g, uint8_t* b)
{
    *r = last_r;
    *g = last_g;
    *b = last_b;

    return ESP_OK;
}

esp_err_t led_push_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
     taskENTER_CRITICAL(&led_mux);

    if (led_history_index == LED_HISTORY_SIZE)
    {
        taskEXIT_CRITICAL(&led_mux);
        return ESP_ERR_NO_MEM;
    }

    led_history[led_history_index][0] = last_r;
    led_history[led_history_index][1] = last_g;
    led_history[led_history_index][2] = last_b;
    led_history_index++;

    taskEXIT_CRITICAL(&led_mux);

    ESP_LOGI(TAG, "push (%u, %u, %u)", last_r, last_g, last_b);

    return led_set_rgb_color(r, g, b);
}

esp_err_t led_pop_rgb_color()
{
    taskENTER_CRITICAL(&led_mux);

    if (led_history_index == 0)
    {
        taskEXIT_CRITICAL(&led_mux);
        return ESP_ERR_INVALID_STATE;
    }
    
    led_history_index--;

    uint8_t r = led_history[led_history_index][0];
    uint8_t g = led_history[led_history_index][1];
    uint8_t b = led_history[led_history_index][2];   

    taskEXIT_CRITICAL(&led_mux);

    ESP_LOGI(TAG, "pop (%u, %u, %u)", r, g, b);

    return led_set_rgb_color(r, g, b);

}
