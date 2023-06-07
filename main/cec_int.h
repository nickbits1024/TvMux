

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

#define CEC_REQUEST_WAIT                5000
#define CEC_RESPONSE_WAIT               1000

typedef struct
{
    bool low;
    int64_t time;
    int64_t last_time;
    int64_t last_low_time;
}
cec_bit_t;

#define CEC_BIT_QUEUE_LENGTH            (500)

#define CEC_START_MARGIN                (200)
#define CEC_START0_TIME                 (3700)
#define CEC_START0_MIN                  (CEC_START0_TIME - CEC_START_MARGIN)
#define CEC_START0_MAX                  (CEC_START0_TIME + CEC_START_MARGIN)
#define CEC_START_TIME                  (4500)
#define CEC_START_MIN                   (CEC_START_TIME - CEC_START_MARGIN)
#define CEC_START_MAX                   (CEC_START_TIME + CEC_START_MARGIN)

#define CEC_BIT_TIME                   (2400)

#define CEC_BIT_TIME_MIN                (1300)
#define CEC_BIT_TIME_MAX                (2750)

#define CEC_DATA1_MIN                   (400)
#define CEC_DATA1_MAX                   (800)
#define CEC_DATA0_MIN                   (1300)
#define CEC_DATA0_MAX                   (1700)

#define CEC_BLOCK_EOM_MASK                (2)
#define CEC_BLOCK_ACK_MASK                (1)
#define CEC_BLOCK_EOM(x)                  (((x) & CEC_BLOCK_EOM_MASK) != 0)
#define CEC_BLOCK_ACK(x)                  (((x) & CEC_BLOCK_ACK_MASK) != 0)
#define CEC_BLOCK_DATA(x)                 (((x) >> 2) & 0xff)

#define CEC_HEAD_INIT(x)                  (((x) >> 6) & 0xf)
#define CEC_HEAD_DEST(x)                  (((x) >> 2) & 0xf)

typedef enum
{
    CEC_IDLE = 0,
    CEC_START,
    CEC_HEAD_0,
    CEC_HEAD_1,
    CEC_HEAD_2,
    CEC_HEAD_3,
    CEC_HEAD_4,
    CEC_HEAD_5,
    CEC_HEAD_6,
    CEC_HEAD_7,
    CEC_HEAD_EOM,
    CEC_HEAD_ACK,
    CEC_DATA_0,
    CEC_DATA_1,
    CEC_DATA_2,
    CEC_DATA_3,
    CEC_DATA_4,
    CEC_DATA_5,
    CEC_DATA_6,
    CEC_DATA_7,
    CEC_DATA_EOM,
    CEC_DATA_ACK
} cec_state_t;

static void cec_loop(void* param);
static void cec_bit_handle(bool bit, int64_t time_ms, int64_t last_low_time_ms);
static bool cec_edid_parse(unsigned char* edid);
static bool cec_edid_extension_parse(uint8_t* edid2, uint8_t* ext);
static void cec_isr(void* param);
