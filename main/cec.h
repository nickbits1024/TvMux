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


#define CEC_TV_ADDRESS                  (0)
#define CEC_RECORDING_DEVICE_1_ADDRESS  (1)
#define CEC_RECORDING_DEVICE_2_ADDRESS  (2)
#define CEC_TUNER_1_ADDRESS             (3)
#define CEC_PLAYBACK_DEVICE_1_ADDRESS   (4)
#define CEC_AUDIO_SYSTEM_ADDRESS        (5)
#define CEC_TUNER_2_ADDRESS             (6)
#define CEC_TUNER_3_ADDRESS             (7)
#define CEC_PLAYBACK_DEVICE_2_ADDRESS   (8)
#define CEC_RECORDING_DEVICE_3_ADDRESS  (9)
#define CEC_TUNER_4_ADDRESS             (10)
#define CEC_PLAYBACK_DEVICE_3_ADDRESS   (11)
#define CEC_RESERVED_1_ADDRESS          (12)
#define CEC_RESERVED_2_ADDRESS          (13)
#define CEC_FREE_USE_ADDRESS            (14)
#define CEC_BROADCAST_ADDRESS           (15)

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