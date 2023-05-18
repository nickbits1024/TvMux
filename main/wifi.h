#ifndef WIFI_H
#define WIFI_H

#define WIFI_SSID_SIZE      32
#define WIFI_PASSWORD_SIZE  64

esp_err_t wifi_init();
esp_err_t wifi_get_ssid(char* ssid, int ssid_size);
esp_err_t wifi_set_ssid(const char* ssid);
esp_err_t wifi_set_password(const char* ssid);

#endif