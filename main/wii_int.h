
#include "bthci.h"

#define WII_PAIR_GPIO_NUM           (GPIO_NUM_32)
#define WII_PAIR_GPIO_SEL           (1ull << WII_PAIR_GPIO_NUM)

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

#define WII_TIMEOUT_TICKS            (2000 / portTICK_PERIOD_MS)
typedef struct
{
    const char* request;
    uint16_t request_size;
    const char* response;
    uint16_t response_size;
} wii_request_response_t;

extern wii_request_response_t wii_sdp_request_responses[];
extern int wii_sdp_request_responses_size;

extern wii_request_response_t wii_hid_request_responses[];
extern int wii_hid_request_responses_size;

#ifdef __cplusplus
extern "C" 
{
#endif

void wii_packet_handler(uint8_t* packet, uint16_t size);
void post_bt_packet(BT_PACKET_ENVELOPE* env);
void wii_connect();
void open_control_channel(uint16_t con_handle);
void post_l2ap_config_mtu_request(uint16_t con_handle, uint16_t remote_cid, uint16_t mtu);
void post_hid_report_packet(uint16_t con_handle, const uint8_t* report, uint16_t report_size);
void post_sdp_packet(uint16_t con_handle, uint16_t l2cap_size, uint8_t* data, uint16_t data_size);
void post_sdp_packet_fragment(uint16_t con_handle, uint8_t* data, uint16_t data_size);
//void wii_pair_irq_handler(void* arg);
wii_state_t wii_state_get();
void wii_state_set(wii_state_t state);


#ifdef __cplusplus
}
#endif
