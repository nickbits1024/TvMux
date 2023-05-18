#include "freertos/queue.h"
#include "CEC_Device.h"

#define CEC_GPIO_INPUT_NUM    (GPIO_NUM_5)
#define CEC_GPIO_INPUT_SEL    (1ull << CEC_GPIO_INPUT_NUM)
#define CEC_GPIO_OUTPUT_NUM   (GPIO_NUM_17)
#define CEC_GPIO_OUTPUT_SEL   (1ull << CEC_GPIO_OUTPUT_NUM)
#define CEC_DEVICE_TYPE       CEC_Device::CDT_TUNER
#define CEC_MAX_LOG_ENTRIES   256

#define CEC_SET_ACTIVE_SOURCE       0x82
#define CEC_USER_CONTROL_PLAY       0x60
#define CEC_USER_CONTROL_PAUSE      0x61
#define CEC_USER_CONTROL_POWER_ON   0x6d 
#define CEC_USER_CONTROL_POWER_OFF  0x6c
#define CEC_POWER_STATUS            0x90

#define CEC_REQUEST_WAIT               5000
#define CEC_RESPONSE_WAIT              1000

void cec_loop(void* param);