#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <nvs_flash.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <esp_check.h>
#include <esp_mac.h>
#include "driver/gpio.h"
#include "bthci.h"
#include "btdump.h"
#include "tvmux.h"
#include "led.h"
#include "wii.h"
#include "wii_int.h"

#define TAG "wii"

 wii_state_t wii_state;
//bool wii_pair_request;

SemaphoreHandle_t wii_response_sem;
bd_addr_t device_addr;
bd_addr_t wii_addr;
SemaphoreHandle_t output_queue_ready_sem;
QueueHandle_t queue_handle;
SemaphoreHandle_t all_controller_buffers_sem;
//TaskHandle_t pairing_task_handle;
int all_controller_buffers_sem_count;
static portMUX_TYPE wii_mux = portMUX_INITIALIZER_UNLOCKED;
bool wii_on;
uint16_t wii_con_handle;
uint16_t wii_sdp_cid;
uint16_t wii_control_cid;
uint16_t wii_data_cid;
uint8_t wii_disconnect_reason;

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
                    //ESP_LOGI(TAG, "acl sent...");

                    HCI_ACL_PACKET* acl_packet = (HCI_ACL_PACKET*)hci_packet;
                    uint16_t con_handles[1] = { acl_packet->con_handle };
                    uint16_t num_complete[1] = { 1 };
                    BT_PACKET_ENVELOPE* hncp_env = create_hci_host_number_of_completed_packets_packet(1, con_handles, num_complete);

                    while (!esp_vhci_host_check_send_available());
                    esp_vhci_host_send_packet(hncp_env->packet, hncp_env->size);

                    //ESP_LOGI(TAG, "acl sent");

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

void pairing_task(void* p)
{
    for (;;)
    {
        while (gpio_get_level(WII_PAIR_GPIO_NUM) == 1) 
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "pairing started");

        wii_state_set(WII_PAIRING_PENDING);

        led_push_rgb_color(255, 255, 0);

        while (gpio_get_level(WII_PAIR_GPIO_NUM) == 0) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        //ESP_LOGI(TAG, "pairing button released");

        post_bt_packet(create_hci_write_scan_enable_packet(HCI_PAGE_SCAN_ENABLE | HCI_INQUIRY_SCAN_ENABLE));

        for (int i = 0; i < 30 || wii_state_get() == WII_PAIRING; i++)
        {
            if (wii_state_get() != WII_PAIRING_PENDING && wii_state_get() != WII_PAIRING)
            {
                ESP_LOGE(TAG, "wii_state changed to %u", wii_state_get());
                break;
            }
            // if (wii_state_get() == WII_PAIRING)
            // {
            //     led_set_rgb_color(0, 0, 255);
            // }
            //led_set_rgb_color(0, 0, 255);
            // vTaskDelay(500 / portTICK_PERIOD_MS);
            // if (wii_state == WII_PAIRING_PENDING)
            // {
            //     led_set_rgb_color(0, 255, 0);
            // }        
            //vTaskDelay(500 / portTICK_PERIOD_MS);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        if (wii_state_get() == WII_PAIRING_PENDING)
        {
            ESP_LOGE(TAG, "pairing incomplete");
            wii_state_set(WII_IDLE);

            led_set_rgb_color(255, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        post_bt_packet(create_hci_write_scan_enable_packet(0));

        led_pop_rgb_color();
    }
    //pairing_task_handle = NULL;
    vTaskDelete(NULL);
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
    switch (wii_state_get())
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
            switch (wii_state_get())
            {
                case WII_QUERY_POWER_STATE:
                case WII_POWER_ON:
                case WII_POWER_OFF:
                    if (wii_on) // sometimes the wii sends a role change if on, sometimes not
                    {
                        ESP_LOGI(TAG, "wii is on (via role change)");
                        post_bt_packet(create_hci_disconnect_packet(packet->con_handle, wii_disconnect_reason));
                    }
                    else
                    {
                        open_control_channel(packet->con_handle);
                    }                    
                    break;
                case WII_PAIRING:
                    memcpy(wii_addr, packet->addr, BDA_SIZE);
                    break;
                case WII_ABORT:
                    post_bt_packet(create_hci_disconnect_packet(packet->con_handle, wii_disconnect_reason));
                    break;
                default:
                    break;
            }
            break;
        case ERROR_CODE_ACL_CONNECTION_ALREADY_EXISTS:
            vTaskDelay(500 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "retrying connection...");
            wii_connect();
            break;
        default:
            wii_state_set(WII_ERROR);
            //led_set_rgb_color(255, 0, 0);
            xSemaphoreGive(wii_response_sem);
            break;
    }
}

void handle_connection_request(HCI_CONNECTION_REQUEST_EVENT_PACKET* packet)
{
    uint32_t cod = uint24_bytes_to_uint32(packet->class_of_device);
    ESP_LOGI(TAG, "connection request from %s cod %06lx type %u", bda_to_string(packet->addr), cod, packet->link_type);

    if (wii_state_get() == WII_PAIRING_PENDING && packet->link_type == HCI_LINK_TYPE_ACL && cod == WII_COD)
    {
        wii_state_set(WII_PAIRING);
        ESP_LOGI(TAG, "accepting wii connection...");
        post_bt_packet(create_hci_accept_connection_request_packet(packet->addr, HCI_ROLE_SLAVE));
    }
    else
    {
        ESP_LOGI(TAG, "rejecting unknown connection...");
        post_bt_packet(create_hci_reject_connection_request_packet(packet->addr, ERROR_CODE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR));
    }
}

void handle_pin_code_request(HCI_PIN_CODE_REQUEST_EVENT_PACKET* packet)
{
    ESP_LOGI(TAG, "pin code request from %s...", bda_to_string(packet->addr));

    uint8_t pin[6];
    memcpy(pin, packet->addr, BDA_SIZE);
    ESP_LOGI(TAG, "sending pin code %02x %02x %02x %02x %02x %02x", pin[0], pin[1], pin[2], pin[3], pin[4], pin[5]);

    post_bt_packet(create_hci_pin_code_request_reply_packet(packet->addr, pin, BDA_SIZE));
}

void handle_mode_change(HCI_MODE_CHANGE_EVENT_PACKET* packet)
{
    if (packet->current_mode == HCI_MODE_SNIFF)
    {
        switch (wii_state_get())
        {
            case WII_PAIRING:
                wii_state_set(WII_IDLE);
                ESP_LOGI(TAG, "pairing complete!");
                post_bt_packet(create_hci_disconnect_packet(packet->con_handle, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION));
                break;
            default:
                break;
        }
    }
}

void handle_link_key_request(HCI_LINK_KEY_REQUEST_EVENT_PACKET* packet)
{
    ESP_LOGI(TAG, "link key request from %s...", bda_to_string(packet->addr));

    switch (wii_state_get())
    {
        case WII_PAIRING:
            ESP_LOGI(TAG, "rejecting link key request from %s...", bda_to_string(packet->addr));
            post_bt_packet(create_hci_link_key_request_negative_packet(packet->addr));
            break;
        default:
        {
            uint8_t link_key[HCI_LINK_KEY_SIZE];
            size_t size = HCI_LINK_KEY_SIZE;
            esp_err_t err = nvs_get_blob(config_nvs_handle, LINK_KEY_BLOB_NAME, link_key, &size);
            if (err == ESP_OK && size == HCI_LINK_KEY_SIZE)
            {
                ESP_LOGI(TAG, "returning stored link key");
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

void handle_link_key_notification(HCI_LINK_KEY_NOTIFICATION_EVENT_PACKET* packet)
{
    esp_err_t err = nvs_set_blob(config_nvs_handle, LINK_KEY_BLOB_NAME, packet->link_key, HCI_LINK_KEY_SIZE);
    ESP_ERROR_CHECK(err);
}

void peek_number_of_completed_packets(uint8_t* packet, uint16_t size)
{
    uint8_t num_handles = packet[3];
    uint8_t* p = packet + 4;
    for (int i = 0; i < num_handles; i++)
    {
        //uint16_t con_handle = read_uint16(p);
        uint16_t num_completed = read_uint16(p + 2);

        //ESP_LOGI(TAG, "peek number_of_completed_packets handle 0x%x completed %u", con_handle, num_completed);

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

void handle_l2cap_connection_request(L2CAP_CONNECTION_REQUEST_PACKET* packet)
{
    //ESP_LOGI(TAG, "l2cap connection request con_handle 0x%x id 0x%x psm 0x%x source_cid 0x%x", packet->con_handle, packet->identifier, packet->psm, packet->source_cid);

    wii_con_handle = packet->con_handle;
    uint16_t response_dest_cid;
    //uint16_t mtu = 0;
    uint16_t result = L2CAP_CONNECTION_RESULT_SUCCESS;
    switch (packet->psm)
    {
        case SDP_PSM:
            wii_sdp_cid = packet->source_cid;
            response_dest_cid = SDP_LOCAL_CID;
            ESP_LOGI(TAG, "set wii_con_handle 0x%x wii_sdp_cid=0x%x", wii_con_handle, wii_sdp_cid);
            break;
        case WII_CONTROL_PSM:
            wii_control_cid = packet->source_cid;
            response_dest_cid = WII_CONTROL_LOCAL_CID;
            //result = L2CAP_CONNECTION_RESULT_PENDING;
            ESP_LOGI(TAG, "set wii_con_handle 0x%x wii_control_cid=0x%x", wii_con_handle, wii_control_cid);
            break;
        case WII_DATA_PSM:
            wii_data_cid = packet->source_cid;
            response_dest_cid = WII_DATA_LOCAL_CID;
            //result = L2CAP_CONNECTION_RESULT_PENDING;
            ESP_LOGI(TAG, "set wii_con_handle 0x%x wii_data_cid=0x%x", wii_con_handle, wii_data_cid);
            break;
        default:
            response_dest_cid = 0;
            break;
    }

    if (response_dest_cid == 0)
    {
        ESP_LOGI(TAG, "connection request no matching psm 0x%x", packet->psm);
        return;
    }

    post_bt_packet(create_l2cap_connection_response_packet(packet->con_handle, packet->identifier, response_dest_cid, packet->source_cid, result, ERROR_CODE_SUCCESS));
    if (result == L2CAP_CONNECTION_RESULT_PENDING)
    {
        post_bt_packet(create_l2cap_connection_response_packet(packet->con_handle, packet->identifier, response_dest_cid, packet->source_cid, L2CAP_CONNECTION_RESULT_SUCCESS, ERROR_CODE_SUCCESS));
    }
    // if (mtu != 0)
    // {
    //     post_l2ap_config_mtu_request(packet->con_handle, packet->source_cid, mtu);
    // }
}

void handle_l2cap_connection_response(L2CAP_CONNECTION_RESPONSE_PACKET* packet)
{
    if (packet->status == ERROR_CODE_SUCCESS && packet->result == L2CAP_CONNECTION_RESULT_SUCCESS)
    {
        switch (wii_state_get())
        {
        case WII_QUERY_POWER_STATE:
        case WII_POWER_ON:
        case WII_POWER_OFF:
            wii_on = true;
            post_bt_packet(create_hci_disconnect_packet(packet->con_handle, wii_disconnect_reason));
            break;
        case WII_PAIRING:
            break;
        default:
            break;
        }
    }
}

void handle_l2cap_config_request(L2CAP_CONFIG_REQUEST_PACKET* packet)
{
    //uint16_t options_size = request_packet->payload_size - 4;

    uint16_t cid;
    uint16_t mtu = 0;
    switch (packet->dest_cid)
    {
        case SDP_LOCAL_CID:
            cid = wii_sdp_cid;
            mtu = WII_REMOTE_SDP_MTU;
            break;
        case WII_CONTROL_LOCAL_CID:
            cid = wii_control_cid;
            mtu = WII_REMOTE_CONTROL_MTU;
            break;
        case WII_DATA_LOCAL_CID:
            cid = wii_data_cid;
            mtu = WII_REMOTE_DATA_MTU;
            break;
        default:
            cid = 0;
            break;
    }

    if (cid == 0)
    {
        ESP_LOGI(TAG, "l2cap config request no matching cid for 0x%x", packet->dest_cid);
        return;
    }

    post_bt_packet(create_l2cap_config_response_packet(packet->con_handle, packet->identifier, cid, 0, 0));

    if (mtu != 0)
    {
        post_l2ap_config_mtu_request(packet->con_handle, cid, mtu);
    }

}

void handle_l2cap_config_response(L2CAP_CONFIG_RESPONSE_PACKET* packet)
{
}

void handle_l2cap_disconnection_request(L2CAP_DISCONNECTION_REQUEST_PACKET* packet)
{
    post_bt_packet(create_l2cap_disconnection_response_packet(packet->con_handle, packet->identifier, packet->dest_cid, packet->source_cid));
}

void handle_l2cap_disconnection_response(L2CAP_DISCONNECTION_RESPONSE_PACKET* packet)
{
}

void handle_l2cap_signal_channel(L2CAP_SIGNAL_CHANNEL_PACKET* packet)
{
    switch (packet->code)
    {
        case L2CAP_CONNECTION_REQUEST:
            handle_l2cap_connection_request((L2CAP_CONNECTION_REQUEST_PACKET*)packet);
            break;
        case L2CAP_CONNECTION_RESPONSE:
            handle_l2cap_connection_response((L2CAP_CONNECTION_RESPONSE_PACKET*)packet);
            break;
        case L2CAP_CONFIG_REQUEST:
            handle_l2cap_config_request((L2CAP_CONFIG_REQUEST_PACKET*)packet);
            break;
        case L2CAP_CONFIG_RESPONSE:
            handle_l2cap_config_response((L2CAP_CONFIG_RESPONSE_PACKET*)packet);
            break;
        case L2CAP_COMMAND_REJECT:
            //handle_l2cap_command_reject((L2CAP_COMMAND_REJECT_PACKET*)packet);
            break;
        case L2CAP_DISCONNECTION_REQUEST:
            handle_l2cap_disconnection_request((L2CAP_DISCONNECTION_REQUEST_PACKET*)packet);
            break;
        case L2CAP_DISCONNECTION_RESPONSE:
            handle_l2cap_disconnection_response((L2CAP_DISCONNECTION_RESPONSE_PACKET*)packet);
            break;
        default:
            ESP_LOGI(TAG, "unhandled signal channel code 0x%02x", packet->code);
            break;
    }
}

void handle_disconnection_complete(HCI_DISCONNECTION_COMPLETE_EVENT_PACKET* packet)
{
    //led_set_rgb_color(0, 255, 0);
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
        ESP_LOGI(TAG, "queue full (recv)");
        xQueueSend(queue_handle, &env, portMAX_DELAY);
    }

    return 0;
}

void handle_authentication_complete(HCI_AUTHENTICATION_COMPLETE_EVENT_PACKET* packet)
{
    ESP_LOGI(TAG, "auth complete con_handle 0x%x status 0x%x", packet->con_handle, packet->status);

    switch (wii_state_get())
    {
        case WII_PAIRING:
            if (packet->status == ERROR_CODE_SUCCESS)
            {
                ESP_LOGI(TAG, "storing wii address %s", bda_to_string(wii_addr));
                nvs_set_blob(config_nvs_handle, WII_ADDR_BLOB_NAME, wii_addr, BDA_SIZE);
            }
            break;
        default:
            break;
    }
}

void post_hid_request_reponse(uint16_t con_handle, HID_REPORT_PACKET* packet, uint16_t size)
{
    uint8_t* p = (uint8_t*)packet;

    for (int i = 0; i < wii_hid_request_responses_size; i++)
    {
        const wii_request_response_t* rr = &wii_hid_request_responses[i];
        if (size == rr->request_size && memcmp(p, rr->request, size) == 0)
        {
            post_hid_report_packet(con_handle, (uint8_t*)rr->response, rr->response_size);
            for (int j = i + 1; j < wii_hid_request_responses_size; j++)
            {
                const wii_request_response_t* rr2 = &wii_hid_request_responses[j];
                if (rr2->request_size == 0 && rr2->request == NULL && rr2->response_size > 0)
                {
                    post_hid_report_packet(con_handle, (uint8_t*)rr2->response, rr2->response_size);
                }
                else
                {
                    break;
                }
            }
            return;
        }
    }

    ESP_LOGI(TAG, "no hid request response post_wii_remote_hid_report_packet(, \"");
    for (int i = 0; i < size; i++)
    {
        printf("\\x%02x", p[i]);
    }
    printf("\", %u);\n", size);

    // if (size == 23 && p[1] == 0x16)
    // {
    //     post_hid_report_packet(wii_con_handle, (uint8_t*)"\xa1\x22\x00\x00\x16\x00", 6);
    // }
    // else
    // {
    //     send_power_toggle_disconnect(wii_con_handle);
    // }
}

int sdp_packet_index = -1;
int sdp_fragment_index = 0;

void post_sdp_packet(uint16_t con_handle, uint16_t l2cap_size, uint8_t* data, uint16_t data_size)
{
    sdp_packet_index++;
    sdp_fragment_index = 0;

    ESP_LOGI(TAG, "sdp request %d.%d \"", sdp_packet_index, sdp_fragment_index);
    for (int i = 0; i < data_size; i++)
    {
        printf("\\x%02x", data[i]);
    }
    printf("\", %u\n", data_size);

    post_bt_packet(create_l2cap_packet(con_handle, l2cap_size, wii_sdp_cid, data, data_size));
}

void post_sdp_packet_fragment(uint16_t con_handle, uint8_t* data, uint16_t data_size)
{
    sdp_fragment_index++;

    ESP_LOGI(TAG, "sdp request %d.%d \"", sdp_packet_index, sdp_fragment_index);
    for (int i = 0; i < data_size; i++)
    {
        printf("\\x%02x", data[i]);
    }
    printf("\", %u\n", data_size);

    post_bt_packet(create_acl_packet(con_handle, wii_sdp_cid, L2CAP_PB_FRAGMENT, L2CAP_BROADCAST_NONE, data, data_size));
}


void handle_data_channel(uint16_t con_handle, HID_REPORT_PACKET* packet, uint16_t size)
{
    switch (wii_state_get())
    {
    case WII_PAIRING:
        switch (packet->report_type)
        {
            case HID_OUTPUT_REPORT:
            {
                post_hid_request_reponse(con_handle, packet, size);
                break;
            }
            default:
                ESP_LOGI(TAG, "unhandled HID report type 0x%x", packet->report_type);
                break;
        }
        break;
    default:
        break;
    }
}

void handle_sdp_channel(L2CAP_PACKET* packet)
{
    for (int i = 0; i < wii_sdp_request_responses_size; i++)
    {
        const wii_request_response_t* rr = &wii_sdp_request_responses[i];
        if (packet->l2cap_size == rr->request_size && memcmp(packet->data, rr->request, packet->l2cap_size) == 0)
        {
            uint16_t l2cap_size = rr->response_size;
            for (int j = i + 1; j < wii_sdp_request_responses_size; j++)
            {
                const wii_request_response_t* rr2 = &wii_sdp_request_responses[j];
                if (rr2->request_size == 0 && rr2->request == NULL && rr2->response_size > 0)
                {
                    l2cap_size += rr2->response_size;
                }
                else
                {
                    break;
                }
            }

            post_sdp_packet(wii_con_handle, l2cap_size, (uint8_t*)rr->response, rr->response_size);
            for (int j = i + 1; j < wii_sdp_request_responses_size; j++)
            {
                const wii_request_response_t* rr2 = &wii_sdp_request_responses[j];
                if (rr2->request_size == 0 && rr2->request == NULL && rr2->response_size > 0)
                {
                    post_sdp_packet_fragment(wii_con_handle, (uint8_t*)rr2->response, rr2->response_size);
                }
                else
                {
                    break;
                }
            }
            return;
        }
    }

    ESP_LOGI(TAG, "no sdp response request post_sdp_packet(L2CAP_AUTO_SIZE, (uint8_t*)\"");
    for (int i = 0; i < packet->l2cap_size; i++)
    {
        printf("\\x%02x", packet->data[i]);
    }
    printf("\", %u);\n", packet->l2cap_size);
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
                case HCI_EVENT_LINK_KEY_NOTIFICATION:
                    handle_link_key_notification((HCI_LINK_KEY_NOTIFICATION_EVENT_PACKET*)packet);
                    break;
                case HCI_EVENT_PIN_CODE_REQUEST:
                    handle_pin_code_request((HCI_PIN_CODE_REQUEST_EVENT_PACKET*)packet);
                    break;
                case HCI_EVENT_AUTHENTICATION_COMPLETE:
                    handle_authentication_complete((HCI_AUTHENTICATION_COMPLETE_EVENT_PACKET*)packet);
                    break; // WAS THIS A BUG ???
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
                case HCI_EVENT_MODE_CHANGE:
                    handle_mode_change((HCI_MODE_CHANGE_EVENT_PACKET*)packet);
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
                    case SDP_LOCAL_CID:
                        handle_sdp_channel(l2cap_packet);
                        break;
                    case WII_DATA_LOCAL_CID:
                        handle_data_channel(l2cap_packet->con_handle, (HID_REPORT_PACKET*)l2cap_packet->data, l2cap_packet->l2cap_size);
                        break;
                    default:
                        ESP_LOGI(TAG, "unhandled l2cap channel 0x%x con_handle 0x%x", l2cap_packet->channel, l2cap_packet->con_handle);
                        break;
                }
            }
            else
            {
                ESP_LOGI(TAG, "bad packet_boundary_flag 0x%x", acl_packet->packet_boundary_flag);
            }
            break;
        }
    }
}

void post_l2ap_config_mtu_request(uint16_t con_handle, uint16_t remote_cid, uint16_t mtu)
{
    uint16_t options_size = sizeof(L2CAP_CONFIG_MTU_OPTION);
    BT_PACKET_ENVELOPE* env = create_l2cap_config_request_packet(con_handle, remote_cid, 0, options_size);
    L2CAP_CONFIG_REQUEST_PACKET* config_packet = (L2CAP_CONFIG_REQUEST_PACKET*)env->packet;

    L2CAP_CONFIG_MTU_OPTION* mtu_option = (L2CAP_CONFIG_MTU_OPTION*)config_packet->options;
    mtu_option->type = L2CAP_CONFIG_MTU_OPTION_TYPE;
    mtu_option->size = sizeof(L2CAP_CONFIG_MTU_OPTION) - sizeof(L2CAP_CONFIG_OPTION);
    mtu_option->mtu = mtu;

    post_bt_packet(env);
}

void post_hid_report_packet(uint16_t con_handle, const uint8_t* report, uint16_t report_size)
{
    ESP_LOGI(TAG, "send hid report \"");
    for (int i = 0; i < report_size; i++)
    {
        printf("\\x%02x", report[i]);
    }
    printf("\", %u\n", report_size);

    post_bt_packet(create_l2cap_packet(con_handle, L2CAP_AUTO_SIZE, wii_data_cid, report, report_size));
}

void post_bt_packet(BT_PACKET_ENVELOPE* env)
{
    env->io_direction = OUTPUT_PACKET;
    if (xQueueSend(queue_handle, &env, 0) != pdTRUE)
    {
        ESP_LOGI(TAG, "queue full (send)");
        xQueueSend(queue_handle, &env, portMAX_DELAY);
    }
}

esp_err_t wii_init()
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = WII_PAIR_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    //ESP_ERROR_CHECK(gpio_isr_register(wii_pair_irq_handler, NULL, 0, NULL));

    size_t size = BDA_SIZE;
    esp_err_t err = nvs_get_blob(config_nvs_handle, WII_ADDR_BLOB_NAME, wii_addr, &size);
    if (err == ESP_OK && size == BDA_SIZE)
    {
        ESP_LOGI(TAG, "wii addr %s", bda_to_string(wii_addr));
    }

    bd_addr_t addr = { 0x00, 0x19, 0x1d, 0x54, 0xd1, 0xa0 };

    err = esp_base_mac_addr_set(addr);
    ESP_ERROR_CHECK(err);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_mode_t mode = ESP_BT_MODE_CLASSIC_BT;
    bt_cfg.mode = mode;
    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&bt_cfg),TAG, "%s/esp_bt_controller_init enable controller failed: %s", __func__, esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(mode), TAG, "%s/esp_bt_controller_enable failed: %s", __func__, esp_err_to_name(err_rc_));

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
    post_bt_packet(create_hci_host_buffer_size_packet(HOST_ACL_BUFFER_SIZE, HOST_SCO_BUFFER_SIZE, HOST_NUM_ACL_BUFFERS, HOST_NUM_SCO_BUFFERS));
    post_bt_packet(create_hci_set_controller_to_host_flow_control_packet(HCI_FLOW_CONTROL_ACL));

    xTaskCreate(pairing_task, "wii_pairing", 8000, NULL, 1, NULL);

    return ESP_OK;
}

void wii_connect()
{
    bd_addr_t null_addr = { };
    if (memcmp(wii_addr, null_addr, BDA_SIZE) == 0)
    {
        ESP_LOGI(TAG, "can't connect to wii--addr not set");
        return;
    }
    //led_set_rgb_color(0, 0, 255);
    post_bt_packet(create_hci_create_connection_packet(wii_addr, WII_PACKET_TYPES, 1, true, 0, 1));
}

bool wii_command(wii_state_t state, uint8_t on_disconnect_reason, uint8_t off_disconnect_reason)
{
    wii_state_set(state);
    wii_on = false;
    wii_con_handle = INVALID_CON_HANDLE;
    wii_disconnect_reason = on_disconnect_reason;

    xSemaphoreTake(wii_response_sem, 0);
    wii_connect();
    xSemaphoreTake(wii_response_sem, WII_TIMEOUT_TICKS);
    if (wii_state_get() == WII_ERROR)
    {
        ESP_LOGI(TAG, "wii command error");
        return false;
    }
    ESP_LOGI(TAG, "wii is %s", wii_on ? "on" : "off");
    wii_state_set(WII_ABORT);
    wii_disconnect_reason = off_disconnect_reason;
    if (wii_con_handle != INVALID_CON_HANDLE)
    {
        post_bt_packet(create_hci_disconnect_packet(wii_con_handle, off_disconnect_reason));
    }

    return true;
}

void open_control_channel(uint16_t con_handle)
{
    post_bt_packet(create_l2cap_connection_request_packet(con_handle, WII_CONTROL_PSM, WII_CONTROL_LOCAL_CID));
}

wii_power_state_t wii_query_power_state()
{
    if (!wii_command(WII_QUERY_POWER_STATE, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION))
    {
        return WII_POWER_STATE_ERROR;
    }

    return wii_on ? WII_POWER_STATE_ON : WII_POWER_STATE_OFF;
}

wii_power_status_t wii_power_on()
{
    if (!wii_command(WII_POWER_ON, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION, ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF))
    {
        return WII_POWER_STATUS_ERROR;
    }

    return wii_on ? WII_POWER_STATUS_NOT_TOGGLED : WII_POWER_STATUS_TOGGLED;
}

wii_power_status_t wii_power_off()
{
    if (!wii_command(WII_POWER_OFF, ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF, ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION))
    {
        return WII_POWER_STATUS_ERROR;
    }

    return wii_on ? WII_POWER_STATUS_TOGGLED : WII_POWER_STATUS_NOT_TOGGLED;
}

// void wii_pair()
// {
//     if (wii_state == WII_PAIRING_PENDING || wii_state == WII_PAIRING)
//     {
//         return;
//     }
//     if (pairing_task_handle != NULL)
//     {
//         vTaskDelete(pairing_task_handle);
//         pairing_task_handle = NULL;
//     }
//     wii_state = WII_PAIRING_PENDING;
//     xTaskCreate(pairing_task, "wii_pairing", 8000, NULL, 1, &pairing_task_handle);
// }

// void wii_pair_irq_handler(void* arg)
// {
//     wii_pair_request = true;
//     ets_ESP_LOGI(TAG, "wii pair request");
// }

wii_state_t wii_state_get()
{
    taskENTER_CRITICAL(&wii_mux);
    wii_state_t state = wii_state;
    taskEXIT_CRITICAL(&wii_mux);
    return state;
}

void wii_state_set(wii_state_t state)
{
    taskENTER_CRITICAL(&wii_mux);
    wii_state_t old_state = wii_state;
    wii_state = state;
    taskEXIT_CRITICAL(&wii_mux);

    ESP_LOGI(TAG, "wii state set  to %u", state);

    if (old_state == WII_PAIRING || old_state == WII_PAIRING_PENDING)
    {
        switch (state)
        {
        case WII_PAIRING:
            led_set_rgb_color(0, 0, 255);
            break;
        case WII_PAIRING_PENDING:
            break;
        case WII_IDLE:    
            break;
        default:
            led_set_rgb_color(255, 0, 0);
            break;
        }
    }

}