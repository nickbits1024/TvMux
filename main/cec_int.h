

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

#define CEC_FRAME_QUEUE_LENGTH          (100)
#define CEC_DEBUG_BIT_QUEUE_LENGTH      (1000)

#define CEC_BIT_SAFETY                  (100)
#define CEC_START_MARGIN                (200)
#define CEC_START0_TIME                 (3700)
#define CEC_START0_MIN                  (CEC_START0_TIME - CEC_START_MARGIN - CEC_BIT_SAFETY)
#define CEC_START0_MAX                  (CEC_START0_TIME + CEC_START_MARGIN + CEC_BIT_SAFETY)
#define CEC_START_TIME                  (4500)
#define CEC_START_MIN                   (CEC_START_TIME - CEC_START_MARGIN - CEC_BIT_SAFETY)
#define CEC_START_MAX                   (CEC_START_TIME + CEC_START_MARGIN + CEC_BIT_SAFETY)

#define CEC_BIT_TIME                    (2400)
#define CEC_NEXT_START_MIN_TIME         (2050)

#define CEC_FRAME_RETRY_MAX             (5)

#define CEC_BIT_TIME_MIN                (1300)
#define CEC_BIT_TIME_MAX                (2750)
#define CEC_BIT_TIME_SAMPLE             (1050)

#define CEC_DATA1_MIN                   (400 - CEC_BIT_SAFETY)
#define CEC_DATA1_TIME                  (600)
#define CEC_DATA1_MAX                   (800 + CEC_BIT_SAFETY)
#define CEC_DATA0_MIN                   (1300 - CEC_BIT_SAFETY)
#define CEC_DATA0_TIME                  (1500)
#define CEC_DATA0_MAX                   (1700 + CEC_BIT_SAFETY)

#define CEC_WAIT_NEW                    (5)
#define CEC_WAIT_CONTINUE               (7)
#define CEC_WAIT_RETRY                  (3)
#define CEC_WAIT_ERROR                  (3)
#define CEC_LINE_ERROR_TIME             (3600)

#define CEC_BLOCK_EOM_MASK              (2)
#define CEC_BLOCK_ACK_MASK              (1)
#define CEC_BLOCK_EOM(x)                (((x) & CEC_BLOCK_EOM_MASK) != 0)
#define CEC_BLOCK_ACK(x)                (((x) & CEC_BLOCK_ACK_MASK) == 0)
#define CEC_BLOCK_DATA(x)               (((x) >> 2) & 0xff)

#define CEC_HEAD_SRC(x)                 (((x) >> 6) & 0xf)
#define CEC_HEAD_DEST(x)                (((x) >> 2) & 0xf)

#define CEC_ACK_OK(broadcast, ack)      (((broadcast) && !(ack)) || (!(broadcast) && (ack)))

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
    CEC_DATA_ACK,
    CEC_STATE_END
} cec_state_t;

static void cec_loop_task(void* param);
static void cec_ack_timer_callback(void* param);

#if 0
static void cec_loop_debug_task(void* param);
static void cec_loop_debug2(_taskvoid* param);
static void cec_isr_debug(void* param);
#endif

static bool cec_bit_handle(cec_frame_t* frame, bool bit, int64_t time_ms, int64_t last_low_time_ms, bool debug);
static void cec_frame_handle(const cec_frame_t* frame, bool debug);
static bool cec_edid_parse(unsigned char* edid);
static bool cec_edid_extension_parse(uint8_t* edid2, uint8_t* ext);
static void cec_isr(void* param);
static bool cec_frame_transmit_byte(uint8_t data, bool eom, bool* ack);
static bool cec_frame_read_ack(bool* ack);
static bool  cec_frame_transmit_bit(bool bit_value, int bit_wait);
static esp_err_t cec_frame_queue_add(cec_frame_t* frame);
static esp_err_t cec_frame_transmit(cec_frame_t* frame);
static bool cec_transmit_start();