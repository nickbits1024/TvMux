#ifndef WII_H
#define WII_H

#define WII_PAIR_GPIO_NUM           (32)
#define WII_PAIR_GPIO_SEL           (1ull < WII_PAIR_GPIO_NUM)

#define WII_REMOTE_COD              (0x002504)
#define WII_COD                     (0x000448)
#define WII_REMOTE_NAME             "Nintendo RVL-CNT-01"
#define WII_PACKET_TYPES            (0xff1e)

#define HOST_ACL_BUFFER_SIZE        (0xffff)
#define HOST_NUM_ACL_BUFFERS        (6)
#define HOST_SCO_BUFFER_SIZE        (0xff)
#define HOST_NUM_SCO_BUFFERS        (1)

#define BT_QUEUE_SIZE               (32)

#define WII_ADDR_BLOB_NAME          "wii_addr"
#define LINK_KEY_BLOB_NAME          "link_key"

#define SDP_PSM                     (0x01)
#define WII_CONTROL_PSM             (0x11)
#define WII_DATA_PSM                (0x13)
#define SDP_LOCAL_CID               (0x40)
#define WII_CONTROL_LOCAL_CID       (0x41)
#define WII_DATA_LOCAL_CID          (0x42)
#define WII_SDP_MTU                 (256)
#define WII_SDP_FLUSH_TIMEOUT       (0xffff)
#define WII_REMOTE_SDP_MTU          (48)
#define WII_REMOTE_CONTROL_MTU      (185)
#define WII_REMOTE_DATA_MTU         (185)
#define WII_MTU                     (640)
#define WII_FLUSH_TIMEOUT           (0xffff)

#define WII_TIMEOUT_TICKS            (1000 / portTICK_PERIOD_MS)

typedef enum 
{
    WII_POWER_STATE_UNKNOWN,
    WII_POWER_STATE_ERROR,
    WII_POWER_STATE_ON,
    WII_POWER_STATE_OFF
} wii_power_state_t;

typedef enum
{
    WII_POWER_STATUS_UNKNOWN,
    WII_POWER_STATUS_ERROR,
    WII_POWER_STATUS_NOT_TOGGLED,
    WII_POWER_STATUS_TOGGLED,
}
wii_power_status_t;

typedef struct
{
    const char* request;
    uint16_t request_size;
    const char* response;
    uint16_t response_size;
} WII_REQUEST_RESPONSE;

extern WII_REQUEST_RESPONSE wii_sdp_request_responses[];
extern int wii_sdp_request_responses_size;

extern WII_REQUEST_RESPONSE wii_hid_request_responses[];
extern int wii_hid_request_responses_size;

#ifdef __cplusplus
extern "C" 
{
#endif

esp_err_t wii_init();
wii_power_state_t wii_query_power_state();
wii_power_status_t wii_power_on();
wii_power_status_t wii_power_off();
void wii_pair();

#ifdef __cplusplus
}
#endif


#endif