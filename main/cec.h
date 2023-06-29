#ifndef CEC2_H
#define CEC2_H

#include "esp_http_server.h"

#define CEC_FRAME_SIZE_MAX              (16)
#define CEC_FRAME_DATA_SIZE_MAX         (15)

#define CEC_TV_HDMI_INPUT               (1)
#define CEC_ATV_HDMI_INPUT              (1)
#define CEC_WII_HDMI_INPUT              (2)
#define CEC_STEAM_HDMI_INPUT            (4)

#define CEC_AS_FORMAT                   "%d.%d.%d.%d"
#define CEC_AS_ARGS(as)                 (as >> 12) & 0xf, (as >> 8) & 0xf, (as >> 4) & 0xf, as & 0xf

typedef enum 
{
    CEC_LA_TV = 0,
    CEC_LA_RECORDING_DEVICE_1,
    CEC_LA_RECORDING_DEVICE_2,
    CEC_LA_TUNER_1,
    CEC_LA_PLAYBACK_DEVICE_1,
    CEC_LA_AUDIO_SYSTEM,
    CEC_LA_TUNER_2,
    CEC_LA_TUNER_3,
    CEC_LA_PLAYBACK_DEVICE_2,
    CEC_LA_RECORDING_DEVICE_3,
    CEC_LA_TUNER_4,
    CEC_LA_PLAYBACK_DEVICE_3,
    CEC_LA_RESERVED_1,
    CEC_LA_RESERVED_2,
    CEC_LA_FREE_USE,
    CEC_LA_UNREGISTERED,
    CEC_LA_BROADCAST = 0xf,
    CEC_LA_MAX = CEC_LA_FREE_USE
} 
cec_logical_address_t;

typedef enum
{
    CEC_FRAME_RX,
    CEC_FRAME_TX    
} 
cec_frame_type_t;

typedef enum
{
    CEC_OP_FEATURE_ABORT = 0x00,
    CEC_OP_IMAGE_VIEW_ON = 0x04,
    CEC_OP_TUNER_STEP_INCREMENT = 0x05,
    CEC_OP_TUNER_STEP_DECREMENT = 0x06,
    CEC_OP_TUNER_DEVICE_STATUS = 0x07,
    CEC_OP_GIVE_TUNER_DEVICE_STATUS = 0x08,
    CEC_OP_RECORD_ON = 0x09,
    CEC_OP_RECORD_STATUS = 0x0A,
    CEC_OP_RECORD_OFF = 0x0B,
    CEC_OP_TEXT_VIEW_ON = 0x0D,
    CEC_OP_RECORD_TV_SCREEN = 0x0F,
    CEC_OP_GIVE_DECK_STATUS = 0x1A,
    CEC_OP_DECK_STATUS = 0x1B,
    CEC_OP_SET_MENU_LANGUAGE = 0x32,
    CEC_OP_CLEAR_ANALOGUE_TIMER = 0x33,
    CEC_OP_SET_ANALOGUE_TIMER = 0x34,
    CEC_OP_TIMER_STATUS = 0x35,
    CEC_OP_STANDBY = 0x36,
    CEC_OP_PLAY = 0x41,
    CEC_OP_DECK_CONTROL = 0x42,
    CEC_OP_TIMER_CLEARED_STATUS = 0x43,
    CEC_OP_USER_CONTROLPRESSED = 0x44,
    CEC_OP_USER_CONTROL_PRESSED = 0x44,
    CEC_OP_USER_CONTROLRELEASED = 0x45,
    CEC_OP_USER_CONTROL_RELEASED = 0x45,
    CEC_OP_GIVE_OSD_NAME = 0x46,
    CEC_OP_SET_OSD_NAME = 0x47,
    CEC_OP_SET_OSD_STRING = 0x64,
    CEC_OP_SET_TIMER_PROGRAM_TITLE = 0x67,
    CEC_OP_SYSTEM_AUDIO_MODE_REQUEST = 0x70,
    CEC_OP_GIVE_AUDIO_STATUS = 0x71,
    CEC_OP_SET_SYSTEM_AUDIO_MODE = 0x72,
    CEC_OP_REPORT_AUDIO_STATUS = 0x7A,
    CEC_OP_GIVE_SYSTEM_AUDIO_MODE_STATUS = 0x7D,
    CEC_OP_SYSTEM_AUDIO_MODE_STATUS = 0x7E,
    CEC_OP_ROUTING_CHANGE = 0x80,
    CEC_OP_ROUTING_INFORMATION = 0x81,
    CEC_OP_ACTIVE_SOURCE = 0x82,
    CEC_OP_GIVE_PHYSICAL_ADDRESS = 0x83,
    CEC_OP_REPORT_PHYSICAL_ADDRESS = 0x84,
    CEC_OP_REQUEST_ACTIVE_SOURCE = 0x85,
    CEC_OP_SET_STREAM_PATH = 0x86,
    CEC_OP_DEVICE_VENDOR_ID = 0x87,
    CEC_OP_VENDOR_COMMAND = 0x89,
    CEC_OP_VENDOR_REMOTE_BUTTON_DOWN = 0x8A,
    CEC_OP_VENDOR_REMOTE_BUTTON_UP = 0x8B,
    CEC_OP_GIVE_DEVICE_VENDOR_ID = 0x8C,
    CEC_OP_MENU_REQUEST = 0x8D,
    CEC_OP_MENU_STATUS = 0x8E,
    CEC_OP_GIVE_DEVICE_POWER_STATUS = 0x8F,
    CEC_OP_REPORT_POWER_STATUS = 0x90,
    CEC_OP_GET_MENU_LANGUAGE = 0x91,
    CEC_OP_SELECT_ANALOGUE_SERVICE = 0x92,
    CEC_OP_SELECT_DIGITAL_SERVICE = 0x93,
    CEC_OP_SET_DIGITAL_TIMER = 0x97,
    CEC_OP_CLEAR_DIGITAL_TIMER = 0x99,
    CEC_OP_SET_AUDIO_RATE = 0x9A,
    CEC_OP_INACTIVE_SOURCE = 0x9D,
    CEC_OP_CEC_VERSION = 0x9E,
    CEC_OP_GET_CEC_VERSION = 0x9F,
    CEC_OP_VENDOR_COMMAND_WITH_ID = 0xA0,
    CEC_OP_CLEAR_EXTERNAL_TIMER = 0xA1,
    CEC_OP_SET_EXTERNAL_TIMER = 0xA2,
    CEC_OP_ABORT = 0xFF
} 
cec_op_code_t;

typedef enum
{
    CEC_OP_UCC_DISPLAY_INFORMATION = 0x35,
    CEC_OP_UCC_HELP = 0x36,
    CEC_OP_UCC_PAGE_UP = 0x37,
    CEC_OP_UCC_PAGE_DOWN = 0x38,
    CEC_OP_UCC_POWER = 0x40,
    CEC_OP_UCC_VOLUME_UP = 0x41,
    CEC_OP_UCC_VOLUME_DOWN = 0x42,
    CEC_OP_UCC_MUTE = 0x43,
    CEC_OP_UCC_PLAY = 0x44,
    CEC_OP_UCC_STOP = 0x45,
    CEC_OP_UCC_PAUSE = 0x46,
    CEC_OP_UCC_RECORD = 0x47,
    CEC_OP_UCC_REWIND = 0x48,
    CEC_OP_UCC_FAST_FORWARD = 0x49,
    CEC_OP_UCC_EJECT = 0x4A,
    CEC_OP_UCC_FORWARD = 0x4B,
    CEC_OP_UCC_BACKWARD = 0x4C,
    CEC_OP_UCC_STOP_RECORD = 0x4D,
    CEC_OP_UCC_PAUSE_RECORD = 0x4E,
    CEC_OP_UCC_ANGLE = 0x50,
    CEC_OP_UCC_SUB_PICTURE = 0x51,
    CEC_OP_UCC_VIDEO_ON_DEMAND = 0x52,
    CEC_OP_UCC_ELECTRONIC_PROGRAM_GUIDE = 0x53,
    CEC_OP_UCC_TIMER_PROGRAMMING = 0x54,
    CEC_OP_UCC_INITIAL_CONFIGURATION = 0x55,
    CEC_OP_UCC_PLAY_FUNCTION = 0x60,
    CEC_OP_UCC_PAUSE_PLAY_FUNCTION = 0x61,
    CEC_OP_UCC_RECORD_FUNCTION = 0x62,
    CEC_OP_UCC_PAUSE_RECORD_FUNCTION = 0x63,
    CEC_OP_UCC_STOP_FUNCTION = 0x64,
    CEC_OP_UCC_MUTE_FUNCTION = 0x65,
    CEC_OP_UCC_RESTORE_VOLUME_FUNCTION = 0x66,
    CEC_OP_UCC_TUNE_FUNCTION = 0x67,
    CEC_OP_UCC_SELECT_MEDIA_FUNCTION = 0x68,
    CEC_OP_UCC_SELECT_AV_INPUT_FUNCTION = 0x69,
    CEC_OP_UCC_SELECT_AUDIO_INPUT_FUNCTION = 0x6A,
    CEC_OP_UCC_POWER_TOGGLE_FUNCTION = 0x6B,
    CEC_OP_UCC_POWER_OFF_FUNCTION = 0x6C,
    CEC_OP_UCC_POWER_ON_FUNCTION = 0x6D,
    CEC_OP_UCC_SELECT = 0x00,
    CEC_OP_UCC_UP = 0x01,
    CEC_OP_UCC_DOWN = 0x02,
    CEC_OP_UCC_LEFT = 0x03,
    CEC_OP_UCC_RIGHT = 0x04,
    CEC_OP_UCC_RIGHT_UP = 0x05,
    CEC_OP_UCC_RIGHT_DOWN = 0x06,
    CEC_OP_UCC_LEFT_UP = 0x07,
    CEC_OP_UCC_LEFT_DOWN = 0x08,
    CEC_OP_UCC_EXIT = 0x0D,
    CEC_OP_UCC_NUMBER_0 = 0x20,
    CEC_OP_UCC_NUMBER_1 = 0x21,
    CEC_OP_UCC_NUMBER_2 = 0x22,
    CEC_OP_UCC_NUMBER_3 = 0x23,
    CEC_OP_UCC_NUMBER_4 = 0x24,
    CEC_OP_UCC_NUMBER_5 = 0x25,
    CEC_OP_UCC_NUMBER_6 = 0x26,
    CEC_OP_UCC_NUMBER_7 = 0x27,
    CEC_OP_UCC_NUMBER_8 = 0x28,
    CEC_OP_UCC_NUMBER_9 = 0x29,
    CEC_OP_UCC_ROOT_MENU = 0x09,
    CEC_OP_UCC_SETUP_MENU = 0x0A,
    CEC_OP_UCC_CONTENTS_MENU = 0x0B,
    CEC_OP_UCC_FAVORITE_MENU = 0x0C,
    CEC_OP_UCC_DOT = 0x2A,
    CEC_OP_UCC_ENTER = 0x2B,
    CEC_OP_UCC_CLEAR = 0x2C,
    CEC_OP_UCC_NEXT_FAVORITE = 0x2F,
    CEC_OP_UCC_CHANNEL_UP = 0x30,
    CEC_OP_UCC_CHANNEL_DOWN = 0x31,
    CEC_OP_UCC_PREVIOUS_CHANNEL = 0x32,
    CEC_OP_UCC_SOUND_SELECT = 0x33,
    CEC_OP_UCC_INPUT_SELECT = 0x34,
    CEC_OP_UCC_F1 = 0x71,
    CEC_OP_UCC_F2 = 0x72,
    CEC_OP_UCC_F3 = 0x73,
    CEC_OP_UCC_F4 = 0x74,
    CEC_OP_UCC_F5 = 0x75,
    CEC_OP_UCC_BLUE = 0x71,
    CEC_OP_UCC_RED = 0x72,
    CEC_OP_UCC_GREEN = 0x73,
    CEC_OP_UCC_YELLOW = 0x74
}
cec_user_control_code_t;

typedef enum 
{
    CEC_DT_TV = 0,
    CEC_DT_RECORDING_DEVICE = 1,
    CEC_DT_RESERVED = 2,
    CEC_DT_TUNER = 3,
    CEC_DT_PLAYBACK_DEVICE = 4,
    CEC_DT_AUDIO_SYSTEM = 5
}
cec_device_type_t;

typedef enum
{
    CEC_PS_ON = 0x00,
    CEC_PS_OFF = 0x01,
    CEC_PS_TRANS_ON = 0x02,
    CEC_PS_TRANS_OFF = 0x03,
    CEC_PS_UNKNOWN = 0xff
}
cec_power_status_t;

typedef struct
{
    cec_frame_type_t type;
    uint8_t src_addr : 4;
    uint8_t dest_addr : 4;
    uint8_t data_size;
    uint8_t data[CEC_FRAME_DATA_SIZE_MAX];
    bool ack;
    bool bit_error;
} 
cec_frame_t;

esp_err_t cec_init();
esp_err_t cec_queue_clear();

esp_err_t cec_log_write(httpd_req_t* request);
esp_err_t cec_log_clear();

esp_err_t cec_standby();
esp_err_t cec_user_control_pressed(cec_logical_address_t log_addr, cec_user_control_code_t ucc);
esp_err_t cec_image_view_on(cec_logical_address_t addr);
esp_err_t cec_control(cec_logical_address_t addr, const uint8_t* request, int request_size, cec_op_code_t reply_op_code, uint8_t* reply, int* reply_size);
esp_err_t cec_system_audio_mode_request(uint16_t phy_addr);
esp_err_t cec_active_source(uint16_t phy_addr);
esp_err_t cec_power_status(cec_logical_address_t log_addr, cec_power_status_t* power_status, bool use_cache);
esp_err_t cec_report_physical_address();


esp_err_t cec_test();
esp_err_t cec_test2();
esp_err_t cec_test3();

#endif