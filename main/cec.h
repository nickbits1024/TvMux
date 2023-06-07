#ifndef CEC2_H
#define CEC2_H

#include "esp_http_server.h"

#define CEC_MAX_MSG_SIZE                (16)
#define CEC_MAX_ADDRESS                 (0x0f)

#define CEC_TV_HDMI_INPUT               (1)
#define CEC_ATV_HDMI_INPUT              (1)
#define CEC_WII_HDMI_INPUT              (2)
#define CEC_STEAM_HDMI_INPUT            (4)

#define CEC_POWER_ON                    (0)
#define CEC_POWER_OFF                   (1)
#define CEC_POWER_TRANS_ON              (2)
#define CEC_POWER_TRANS_OFF             (3)
#define CEC_POWER_UNKNOWN               (0xff)

typedef enum 
{
    CLA_TV = 0,
    CLA_RECORDING_DEVICE_1,
    CLA_RECORDING_DEVICE_2,
    CLA_TUNER_1,
    CLA_PLAYBACK_DEVICE_1,
    CLA_AUDIO_SYSTEM,
    CLA_TUNER_2,
    CLA_TUNER_3,
    CLA_PLAYBACK_DEVICE_2,
    CLA_RECORDING_DEVICE_3,
    CLA_TUNER_4,
    CLA_PLAYBACK_DEVICE_3,
    CLA_RESERVED_1,
    CLA_RESERVED_2,
    CLA_FREE_USE,
    CLA_UNREGISTERED,
} 
cec_logical_address_t;

typedef struct
{
    uint8_t targetAddress;
    uint8_t size;
    uint8_t data[CEC_MAX_MSG_SIZE];
} CEC_MESSAGE;

typedef struct 
{
    uint8_t direction;
    uint8_t size;
    bool ack;
    uint8_t data[CEC_MAX_MSG_SIZE];
} CEC_LOG_ENTRY;

esp_err_t cec_init();
esp_err_t cec_queue_clear();
esp_err_t cec_standby();
esp_err_t cec_pause();
esp_err_t cec_play();
esp_err_t cec_tv_power(bool power_on);
esp_err_t cec_wii_power(bool power_on);
esp_err_t cec_log_write(httpd_req_t* request);
esp_err_t cec_log_clear();
esp_err_t cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size);
esp_err_t cec_sam_request(uint16_t addr);
esp_err_t cec_as_set(uint16_t addr);
esp_err_t cec_tv_on();

esp_err_t cec_combine_devices_state(bool* state, bool and_mode, bool tv, bool audio, bool atv);

#endif