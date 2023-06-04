#include "freertos/queue.h"
#include "CEC_Device.h"

#define HDMI_HOTPLUG_GPIO_NUM           (GPIO_NUM_13)
#define HDMI_HOTPLUG_GPIO_SEL           (1ull << HDMI_HOTPLUG_GPIO_NUM)
#define HDMI_CEC_GPIO_NUM               (GPIO_NUM_5)
#define HDMI_CEC_GPIO_SEL               (1ull << HDMI_CEC_GPIO_NUM)
#define HDMI_CEC_GPIO_2_NUM             (GPIO_NUM_17)
#define HDMI_CEC_GPIO_2_SEL             (1ull << HDMI_CEC_GPIO_2_NUM)
#define CEC_DEVICE_TYPE                 CEC_Device::CDT_TUNER
#define CEC_MAX_LOG_ENTRIES             256

#define CEC_SET_ACTIVE_SOURCE           0x82
#define CEC_USER_CONTROL_PLAY           0x60
#define CEC_USER_CONTROL_PAUSE          0x61
#define CEC_USER_CONTROL_POWER_ON       0x6d 
#define CEC_USER_CONTROL_POWER_OFF      0x6c
#define CEC_POWER_STATUS                0x90

#define CEC_REQUEST_WAIT               5000
#define CEC_RESPONSE_WAIT              1000

void cec_loop(void* param);
bool cec_edid_parse(unsigned char* edid);
bool cec_edid_extension_parse(uint8_t* edid2, uint8_t* ext);