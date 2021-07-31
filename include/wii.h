#ifndef WII_H
#define WII_H

#define WII_REMOTE_COD              0x002504
#define WII_COD                     0x000448
#define WII_REMOTE_NAME             "Nintendo RVL-CNT-01"
#define WII_PACKET_TYPES            0xff1e

#define HOST_ACL_BUFFER_SIZE        0xffff
#define HOST_NUM_ACL_BUFFERS        6
#define HOST_SCO_BUFFER_SIZE        0xff
#define HOST_NUM_SCO_BUFFERS        1

#define BT_QUEUE_SIZE               32

#define WII_ADDR_BLOB_NAME          "wii_addr"
#define LINK_KEY_BLOB_NAME          "link_key"

#define WII_CONTROL_PSM             0x11
#define WII_DATA_PSM                0x13
#define SDP_LOCAL_CID               0x40
#define WII_CONTROL_LOCAL_CID       0x41
#define WII_DATA_LOCAL_CID          0x42

#define WII_TIMEOUT_TICKS            (1000 / portTICK_PERIOD_MS)

extern "C"
{
void wii_init();
bool wii_query_power_state();
bool wii_power_on();
bool wii_power_off();
void wii_start_pairing();
}

#endif