#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp32-hal-bt.h>
#include <nvs_flash.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include "bthci.c"
#include "btdump.c"
#include "Wii.h"

typedef enum
{
    WII_PAIRING = 1,
    WII_POWER_ON,
    WII_POWER_OFF,
    WII_QUERY_POWER_STATE
} wii_state_t;

wii_state_t wii_state;

xSemaphoreHandle wii_response_sem;
nvs_handle wii_nvs_handle;
bd_addr_t device_addr;
bd_addr_t wii_addr;
xSemaphoreHandle output_queue_ready_sem;
xQueueHandle queue_handle;
xSemaphoreHandle all_controller_buffers_sem;
int all_controller_buffers_sem_count;
bool wii_on;
uint16_t wii_con_handle;
uint8_t wii_on_disconnect_reason;

void wii_packet_handler(uint8_t* packet, uint16_t size);
void post_bt_packet(BT_PACKET_ENVELOPE* env);
void wii_connect();
void open_control_channel(uint16_t con_handle);

void queue_io_task(void* p)
{
    bool first_acl_send = true;
    while (true)
    {
        BT_PACKET_ENVELOPE* env;

        if (xQueueReceive(queue_handle, &env, portMAX_DELAY) == pdPASS)
        {
            dump_packet(env->io_direction, env->packet, env->size);

            HCI_PACKET* hci_packet = (HCI_PACKET*)env->packet;

            if (env->io_direction == OUTPUT_PACKET)
            {
                if (hci_packet->type == HCI_ACL_PACKET_TYPE)
                {
                    //HCI_ACL_PACKET* acl_packet = (HCI_ACL_PACKET*)hci_packet;
                    if (first_acl_send)
                    {
                        xSemaphoreTake(output_queue_ready_sem, portMAX_DELAY);
                        first_acl_send = false;
                    }
                    xSemaphoreTake(all_controller_buffers_sem, portMAX_DELAY);
                    all_controller_buffers_sem_count--;
                }

                while (!esp_vhci_host_check_send_available());
                esp_vhci_host_send_packet(env->packet, env->size);
            }
            else
            {
                wii_packet_handler(env->packet, env->size);

                if (hci_packet->type == HCI_ACL_PACKET_TYPE)
                {
                    //printf("acl sent...\n");

                    HCI_ACL_PACKET* acl_packet = (HCI_ACL_PACKET*)hci_packet;
                    uint16_t con_handles[1] = { acl_packet->con_handle };
                    uint16_t num_complete[1] = { 1 };
                    BT_PACKET_ENVELOPE* hncp_env = create_hci_host_number_of_completed_packets_packet(1, con_handles, num_complete);

                    while (!esp_vhci_host_check_send_available());
                    esp_vhci_host_send_packet(hncp_env->packet, hncp_env->size);

                    //printf("acl sent\n");

                    free(hncp_env);
                    hncp_env = NULL;
                }


            }

            free(env);
            env = NULL;
        }

        //vTaskDelay(1);
    }
}

void handle_read_bd_addr_complete(HCI_AUTH_READ_BD_ADDR_COMPLETE_PACKET* packet)
{
    memcpy(device_addr, packet->addr, BDA_SIZE);
}

void handle_read_buffer_size_complete(HCI_READ_BUFFER_SIZE_COMPLETE_PACKET* packet)
{
    if (all_controller_buffers_sem == NULL)
    {
        int all_slots = packet->hc_total_num_acl_data_packets;
        all_controller_buffers_sem_count = all_slots;
        all_controller_buffers_sem = xSemaphoreCreateCounting(all_slots, all_slots);

        xSemaphoreGive(output_queue_ready_sem);
    }
}

void handle_role_change(HCI_ROLE_CHANGE_EVENT_PACKET* packet)
{
    switch (wii_state)
    {
        case WII_QUERY_POWER_STATE:
            wii_on = packet->new_role == HCI_ROLE_SLAVE;
            break;
        default:
            break;
    }
}

void handle_connection_complete(HCI_CONNECTION_COMPLETE_EVENT_PACKET* packet)
{
    switch (packet->status)
    {
        case ERROR_CODE_SUCCESS:
            wii_con_handle = packet->con_handle;
            switch (wii_state)
            {
                case WII_QUERY_POWER_STATE:
                    open_control_channel(packet->con_handle);
                    //xSemaphoreGive(wii_response_sem);
                    break;
                // case WII_CONSOLE_PAIRING_PENDING:
                //     memcpy(wii_addr, packet->addr, BDA_SIZE);
                //     wii_controller.state = WII_CONSOLE_PAIRING_STARTED;
                //     break;
                case WII_POWER_ON:
                    //xSemaphoreGive(wii_response_sem);
                    //printf("wii is %s\n", wii_on ? "on" : "off");
                    open_control_channel(packet->con_handle);
                    break;
                case WII_POWER_OFF:
                    printf("wii is %s\n", wii_on ? "on" : "off");
                    //wii_state = WII_CONSOLE_POWER_OFF_CONNECTED;
                    open_control_channel(packet->con_handle);
                    break;
                case WII_PAIRING:
                    break;
            }
            break;
        case ERROR_CODE_ACL_CONNECTION_ALREADY_EXISTS:
            vTaskDelay(500 / portTICK_PERIOD_MS);
            printf("retrying connection...\n");
            wii_connect();
            break;
    }
}

void handle_connection_request(HCI_CONNECTION_REQUEST_EVENT_PACKET* packet)
{
    uint32_t cod = uint24_bytes_to_uint32(packet->class_of_device);
    printf("connection request from %s cod %06x type %u\n", bda_to_string(packet->addr), cod, packet->link_type);

    if (packet->link_type == HCI_LINK_TYPE_ACL && cod == WII_COD)
    {
        printf("accepting wii connection...\n");
        post_bt_packet(create_hci_accept_connection_request_packet(packet->addr, HCI_ROLE_SLAVE));
    }
    else
    {
        printf("rejecting unknown connection...\n");
        post_bt_packet(create_hci_reject_connection_request_packet(packet->addr, ERROR_CODE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR));
    }
}

void handle_pin_code_request(HCI_PIN_CODE_REQUEST_EVENT_PACKET* packet)
{
    printf("pin code request from %s...\n", bda_to_string(packet->addr));

    uint8_t pin[6];
    memcpy(pin, packet->addr, BDA_SIZE);
    printf("sending pin code %02x %02x %02x %02x %02x %02x\n", pin[0], pin[1], pin[2], pin[3], pin[4], pin[5]);

    post_bt_packet(create_hci_pin_code_request_reply_packet(packet->addr, pin, BDA_SIZE));
}


void handle_link_key_request(HCI_LINK_KEY_REQUEST_EVENT_PACKET* packet)
{
    printf("link key request from %s...\n", bda_to_string(packet->addr));

    switch (wii_state)
    {
        case WII_PAIRING:
            printf("rejecting link key request from %s...\n", bda_to_string(packet->addr));
            post_bt_packet(create_hci_link_key_request_negative_packet(packet->addr));
            break;
        default:
        {
            uint8_t link_key[HCI_LINK_KEY_SIZE];
            size_t size = HCI_LINK_KEY_SIZE;
            esp_err_t err = nvs_get_blob(wii_nvs_handle, LINK_KEY_BLOB_NAME, link_key, &size);
            if (err == ESP_OK && size == HCI_LINK_KEY_SIZE)
            {
                printf("returning stored link key");
                for (int i = 0; i < HCI_LINK_KEY_SIZE; i++)
                {
                    printf(" %02x", link_key[i]);
                }
                printf("\n");
                post_bt_packet(create_hci_link_key_request_reply_packet(packet->addr, link_key));
            }
            break;
        }
    }
}

void peek_number_of_completed_packets(uint8_t* packet, uint16_t size)
{
    uint8_t num_handles = packet[3];
    uint8_t* p = packet + 4;
    for (int i = 0; i < num_handles; i++)
    {
        //uint16_t con_handle = read_uint16(p);
        uint16_t num_completed = read_uint16(p + 2);

        //printf("peek number_of_completed_packets handle 0x%x completed %u\n", con_handle, num_completed);

        for (int j = 0; j < num_completed; j++)
        {
            xSemaphoreGive(all_controller_buffers_sem);
            all_controller_buffers_sem_count++;

            //xSemaphoreGive(get_queue_sem(con_handle, 1));
        }

        p += 4;
    }
}

void handle_command_complete(uint8_t* packet, uint16_t size)
{
    uint16_t op_code = read_uint16(packet + 4);
    switch (op_code)
    {
        case HCI_OPCODE_READ_BD_ADDR:
            handle_read_bd_addr_complete((HCI_AUTH_READ_BD_ADDR_COMPLETE_PACKET*)packet);
            break;
        case HCI_OPCODE_READ_BUFFER_SIZE:
            handle_read_buffer_size_complete((HCI_READ_BUFFER_SIZE_COMPLETE_PACKET*)packet);
            break;
    }
}

void handle_l2cap_connection_response(L2CAP_CONNECTION_RESPONSE_PACKET* packet)
{
    if (packet->status == ERROR_CODE_SUCCESS && packet->result == L2CAP_CONNECTION_RESULT_SUCCESS)
    {
        switch (wii_state)
        {
        case WII_QUERY_POWER_STATE:
        case WII_POWER_ON:
        case WII_POWER_OFF:
            wii_on = true;
            post_bt_packet(create_hci_disconnect_packet(packet->con_handle, wii_on_disconnect_reason));
            break;
        case WII_PAIRING:
            break;
        }
    }
}

void handle_l2cap_signal_channel(L2CAP_SIGNAL_CHANNEL_PACKET* packet)
{
    switch (packet->code)
    {
        case L2CAP_CONNECTION_RESPONSE:
            handle_l2cap_connection_response((L2CAP_CONNECTION_RESPONSE_PACKET*)packet);
            break;
        default:
            printf("unhandled signal channel code 0x%02x\n", packet->code);
            break;
    }
}

void handle_disconnection_complete(HCI_DISCONNECTION_COMPLETE_EVENT_PACKET* packet)
{
    wii_con_handle = INVALID_CON_HANDLE;
    xSemaphoreGive(wii_response_sem);
}

int queue_packet_handler(uint8_t* packet, uint16_t size)
{
    HCI_EVENT_PACKET* event_packet = (HCI_EVENT_PACKET*)packet;
    if (event_packet->type == HCI_EVENT_PACKET_TYPE &&
        event_packet->event_code == HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS)
    {
        peek_number_of_completed_packets(packet, size);
    }

    BT_PACKET_ENVELOPE* env = create_packet_envelope(size);
    env->io_direction = INPUT_PACKET;
    memcpy(env->packet, packet, size);

    if (xQueueSend(queue_handle, &env, 0) != pdPASS)
    {
        printf("queue full (recv)\n");
        xQueueSend(queue_handle, &env, portMAX_DELAY);
    }

    return 0;
}

void post_bt_packet(BT_PACKET_ENVELOPE* env)
{
    env->io_direction = OUTPUT_PACKET;
    if (xQueueSend(queue_handle, &env, 0) != pdTRUE)
    {
        printf("queue full (send)\n");
        xQueueSend(queue_handle, &env, portMAX_DELAY);
    }
}

void wii_packet_handler(uint8_t* packet, uint16_t size)
{
    HCI_ACL_PACKET* acl_packet = (HCI_ACL_PACKET*)packet;
    HCI_EVENT_PACKET* event_packet = (HCI_EVENT_PACKET*)packet;

    switch (acl_packet->type)
    {
        case HCI_EVENT_PACKET_TYPE:
            switch (event_packet->event_code)
            {
                case HCI_EVENT_COMMAND_COMPLETE:
                    handle_command_complete(packet, size);
                    break;
                case HCI_EVENT_LINK_KEY_REQUEST:
                    handle_link_key_request((HCI_LINK_KEY_REQUEST_EVENT_PACKET*)packet);
                    break;
                case HCI_EVENT_PIN_CODE_REQUEST:
                    handle_pin_code_request((HCI_PIN_CODE_REQUEST_EVENT_PACKET*)packet);
                    break;
                case HCI_EVENT_CONNECTION_REQUEST:
                    handle_connection_request((HCI_CONNECTION_REQUEST_EVENT_PACKET*)packet);
                    break;
                case HCI_EVENT_CONNECTION_COMPLETE:
                    handle_connection_complete((HCI_CONNECTION_COMPLETE_EVENT_PACKET*)packet);
                    break;
                case HCI_EVENT_ROLE_CHANGE:
                    handle_role_change((HCI_ROLE_CHANGE_EVENT_PACKET*)packet);
                    break;
                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    handle_disconnection_complete((HCI_DISCONNECTION_COMPLETE_EVENT_PACKET*)packet);
                    break;
            }
            break;        
        case HCI_ACL_PACKET_TYPE:
        {
            if (acl_packet->packet_boundary_flag == L2CAP_PB_FIRST_FLUSH || acl_packet->packet_boundary_flag == L2CAP_PB_DEFAULT)
            {
                L2CAP_PACKET* l2cap_packet = (L2CAP_PACKET*)packet;

                switch (l2cap_packet->channel)
                {
                    case L2CAP_SIGNAL_CHANNEL:
                        handle_l2cap_signal_channel((L2CAP_SIGNAL_CHANNEL_PACKET*)l2cap_packet);
                        break;
                    default:
                        printf("unhandled l2cap channel 0x%x con_handle 0x%x\n", l2cap_packet->channel, l2cap_packet->con_handle);
                        break;
                }
            }
            else
            {
                printf("bad packet_boundary_flag 0x%x\n", acl_packet->packet_boundary_flag);
            }
            break;
        }
    }
}

void wii_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    err = nvs_open("default", NVS_READWRITE, &wii_nvs_handle);
    ESP_ERROR_CHECK(err);

    size_t size = BDA_SIZE;
    esp_err_t ret = nvs_get_blob(wii_nvs_handle, WII_ADDR_BLOB_NAME, wii_addr, &size);
    if (ret == ESP_OK && size == BDA_SIZE)
    {
        printf("wii addr %s\n", bda_to_string(wii_addr));
    }

    bd_addr_t addr = { 0x00, 0x19, 0x1d, 0x54, 0xd1, 0xa0 };

    err = esp_base_mac_addr_set(addr);
    ESP_ERROR_CHECK(err);

    if (!btStart())
    {
        printf("btStart failed\n");
        abort();
    }

    static esp_vhci_host_callback_t vhci_host_cb =
    {
        .notify_host_send_available = NULL,
        .notify_host_recv = queue_packet_handler,
    };
    err = esp_vhci_host_register_callback(&vhci_host_cb);
    ESP_ERROR_CHECK(err);

    output_queue_ready_sem = xSemaphoreCreateBinary();
    wii_response_sem = xSemaphoreCreateBinary();
    queue_handle = xQueueCreate(BT_QUEUE_SIZE, sizeof(BT_PACKET_ENVELOPE*));

    err = xTaskCreatePinnedToCore(queue_io_task, "queue_io", 8192, NULL, 1, NULL, 0) == pdPASS ? ESP_OK : ESP_FAIL;
    ESP_ERROR_CHECK(err);

    post_bt_packet(create_hci_reset_packet());
    post_bt_packet(create_hci_cmd_packet(HCI_OPCODE_READ_BD_ADDR, 0));
    post_bt_packet(create_hci_read_buffer_size_packet());
    post_bt_packet(create_hci_write_default_link_policy_settings_packet(
        HCI_LINK_POLICY_ENABLE_ROLE_SWITCH |
        HCI_LINK_POLICY_ENABLE_SNIFF_MODE));
    post_bt_packet(create_hci_write_class_of_device_packet(WII_REMOTE_COD));
    post_bt_packet(create_hci_write_local_name(WII_REMOTE_NAME));
    post_bt_packet(create_hci_current_iac_lap_packet(GAP_IAC_LIMITED_INQUIRY));
    post_bt_packet(create_hci_write_scan_enable_packet(HCI_PAGE_SCAN_ENABLE | HCI_INQUIRY_SCAN_ENABLE));
    post_bt_packet(create_hci_host_buffer_size_packet(HOST_ACL_BUFFER_SIZE, HOST_SCO_BUFFER_SIZE, HOST_NUM_ACL_BUFFERS, HOST_NUM_SCO_BUFFERS));
    post_bt_packet(create_hci_set_controller_to_host_flow_control_packet(HCI_FLOW_CONTROL_ACL));
}

void wii_connect()
{
    bd_addr_t null_addr = { };
    if (memcmp(wii_addr, null_addr, BDA_SIZE) == 0)
    {
        printf("can't connect to wii--addr not set\n");
        return;
    }
    post_bt_packet(create_hci_create_connection_packet(wii_addr, WII_PACKET_TYPES, 1, true, 0, 1));
}

bool wii_command(wii_state_t state, uint8_t on_disconnect_reason, uint8_t off_disconnect_reason)
{
    wii_state = state;
    wii_on = false;
    wii_con_handle = INVALID_CON_HANDLE;
    wii_on_disconnect_reason = on_disconnect_reason;

    xSemaphoreTake(wii_response_sem, 0);
    wii_connect();
    xSemaphoreTake(wii_response_sem, WII_TIMEOUT_TICKS);
    printf("wii is %s\n", wii_on ? "on" : "off");
    if (wii_con_handle != INVALID_CON_HANDLE)
    {
        post_bt_packet(create_hci_disconnect_packet(wii_con_handle, off_disconnect_reason));
    }

    return wii_on;
}

bool wii_query_power_state()
{
    return wii_command(WII_QUERY_POWER_STATE, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION);
}

bool wii_power_on()
{
    return !wii_command(WII_POWER_ON, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION, ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF);
}

bool wii_power_off()
{
    return wii_command(WII_POWER_OFF, ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION);
}

void open_control_channel(uint16_t con_handle)
{
    post_bt_packet(create_l2cap_connection_request_packet(con_handle, WII_CONTROL_PSM, WII_CONTROL_LOCAL_CID));
}
