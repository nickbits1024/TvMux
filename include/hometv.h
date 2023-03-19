#ifndef HOMETV_H
#define HOMETV_H

#define ONBOARD_LED_GPIO           2
#define MAX_COMMAND_RETRY          3
#define MAX_REQUEST_SIZE           1000

extern nvs_handle config_nvs_handle;

esp_err_t cec_log_write(char direction, const uint8_t* buffer, uint8_t size, bool ack);

#endif