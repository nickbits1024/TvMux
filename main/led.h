#ifndef LED_H
#define LED_H

esp_err_t led_init();
esp_err_t led_enable(bool enable);
esp_err_t led_set_rgb_color(uint8_t r, uint8_t g, uint8_t b);
esp_err_t led_get_rgb_color(uint8_t* r, uint8_t* g, uint8_t* b);
esp_err_t led_push_rgb_color(uint8_t r, uint8_t g, uint8_t b);
esp_err_t led_pop_rgb_color();

#endif