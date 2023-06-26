
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

static void wii_packet_handler(uint8_t* packet, uint16_t size);
static void post_bt_packet(BT_PACKET_ENVELOPE* env);
static void wii_connect();
static void open_control_channel(uint16_t con_handle);
static void post_l2ap_config_mtu_request(uint16_t con_handle, uint16_t remote_cid, uint16_t mtu);
static void post_hid_report_packet(uint16_t con_handle, const uint8_t* report, uint16_t report_size);
static void post_sdp_packet(uint16_t con_handle, uint16_t l2cap_size, uint8_t* data, uint16_t data_size);
static void post_sdp_packet_fragment(uint16_t con_handle, uint8_t* data, uint16_t data_size);
static wii_state_t wii_state_get();
static void wii_state_set(wii_state_t state);