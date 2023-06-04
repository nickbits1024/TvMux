#include <list>
#include <iomanip>
#include <string>
#include <sstream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "ddc.h"
#include "cec.h"
#include "cec_int.h"
#include "TvMux.h"

extern uint16_t cec_physical_address;

#define TAG "CEC"

static HomeTvCec* cec_device;

esp_err_t cec_init()
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = HDMI_HOTPLUG_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = HDMI_CEC_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = HDMI_CEC_GPIO_2_SEL;
    io_conf.mode = GPIO_MODE_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_NUM, 1));
    //ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_2_NUM, 1));

#ifdef HDMI_CEC
    xTaskCreate(cec_loop, "cec_loop", 10000, NULL, 1, NULL);
#endif

    return ESP_OK;
}

HomeTvCec::HomeTvCec() :
    pending_message(NULL),
    response_mux(portMUX_INITIALIZER_UNLOCKED),
    state_mux(portMUX_INITIALIZER_UNLOCKED),
    reply(NULL),
    reply_size(NULL),
    reply_address(-1),
    active_source(-1),
    power_states { CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, 
                   CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, 
                   CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, 
                   CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN, CEC_POWER_UNKNOWN }
{
    this->queue_handle = xQueueCreate(100, sizeof(CEC_MESSAGE*));
    request_sem = xSemaphoreCreateBinary();
    //response_sem = xSemaphoreCreateBinary();
    responded_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(request_sem);
    xSemaphoreGive(responded_sem);

}

bool HomeTvCec::LineState()
{
    int state = gpio_get_level(HDMI_CEC_GPIO_NUM);
    return state != 0;
}

void HomeTvCec::SetLineState(bool state)
{
    gpio_set_level(HDMI_CEC_GPIO_NUM, state ? 1 : 0);
    // give enough time for the line to settle before sampling it
    ets_delay_us(50);
}

void HomeTvCec::OnReady(int logicalAddress)
{
    // This is called after the logical address has been allocated

    //unsigned char buf[4] = { 0x84, CEC_PHYSICAL_ADDRESS >> 8, CEC_PHYSICAL_ADDRESS & 0xff, CEC_DEVICE_TYPE };
    unsigned char buf[4] =
    {
      0x84,
      (unsigned char)(cec_physical_address >> 8),
      (unsigned char)(cec_physical_address & 0xff),
      CEC_DEVICE_TYPE
    };

    ESP_LOGI(TAG, "CEC Logical Address: : %d", logicalAddress);

    TransmitFrame(0xf, buf, 4); // <Report Physical Address>
}

void HomeTvCec::PrintIO(char direction, unsigned char* buffer, int size, bool ack)
{
    if (size >= 256)
    {
        return;
    }

    printf("cec %cx: ", direction);
    for (int i = 0; i < size; i++)
    {
        printf("%s%02x", i == 0 ? "" : ":", buffer[i]);
    }
    if (!ack)
    {
        printf(" !");
    }
    printf("\n");

    portENTER_CRITICAL(&this->state_mux);

    while (this->log_entries.size() >= CEC_MAX_LOG_ENTRIES)
    {
        this->log_entries.pop_front();
    }
    CEC_LOG_ENTRY entry;
    entry.size = (int8_t)size;
    entry.direction = direction;
    entry.ack = ack;
    memcpy(entry.data, buffer, size);

    this->log_entries.push_back(entry);
    portEXIT_CRITICAL(&this->state_mux);
}

void HomeTvCec::OnReceiveComplete(unsigned char* buffer, int size, bool ack)
{
    // No command received?
    if (size < 2)
        return;

    PrintIO('r', buffer, size, ack);

    uint8_t source_addr = buffer[0] >> 4;

    switch (buffer[1])
    {
        case CEC_SET_ACTIVE_SOURCE:
        {
            uint16_t active_source = buffer[2] << 8 | buffer[3];

            if (this->active_source != 0xffff && this->active_source != active_source && source_addr == CEC_TV_ADDRESS)
            {
                ESP_LOGI(TAG, "TV trying to steal active source, restoring source %02x", this->active_source);
                SetActiveSource(this->active_source);
            }
            else if (this->active_source != active_source)
            {
                ESP_LOGI(TAG, "Saving source %02x", this->active_source);
                this->active_source = active_source;
            }
            break;
        }
        case CEC_POWER_STATUS:
            portENTER_CRITICAL(&state_mux);    
            this->power_states[source_addr] = buffer[2];
            portEXIT_CRITICAL(&state_mux);
            ESP_LOGI(TAG, "addr 0x%02x got power state %d", source_addr, buffer[2]);
            break;
    }

    // Ignore messages not sent to us
    if ((buffer[0] & 0xf) != LogicalAddress())
        return;

    portENTER_CRITICAL(&response_mux);
    //if (xSemaphoreTake(response_sem, 0))
    {
        if (this->reply != NULL && this->reply_size != NULL)
        {
            if ((reply_address == -1 || (buffer[0] >> 4) == reply_address) &&
                buffer[1] == reply_filter && size <= CEC_MAX_MSG_SIZE)
            {
                // if (size >= 3)
                // {
                //   ets_printf("copy reply %02x:%02x:%02x\n", buffer[0], buffer[1], buffer[2]);
                // }
                memcpy(this->reply, buffer, size);
                *this->reply_size = size;
                xSemaphoreGive(responded_sem);
            }
        }
        //xSemaphoreGive(response_sem);
    }
    portEXIT_CRITICAL(&response_mux);
    //portENTER_CRITICAL(&cecMux);
    switch (buffer[1])
    {
        case 0x83:
        { // <Give Physical Address>
          //unsigned char buf[4] = { 0x84, CEC_PHYSICAL_ADDRESS >> 8, CEC_PHYSICAL_ADDRESS & 0xff, CEC_DEVICE_TYPE };
            uint8_t buf[4] =
            {
              0x84,
              (uint8_t)(cec_physical_address >> 8),
              (uint8_t)(cec_physical_address & 0xff),
              CEC_DEVICE_TYPE
            };
            TransmitFrame(0xf, buf, 4); // <Report Physical Address>
            break;
        }
        case 0x8c: // <Give Device Vendor ID>
            TransmitFrame(0xf, (unsigned char*)"\x87\x01\x23\x45", 4); // <Device Vendor ID>
            break;
    }
    //portEXIT_CRITICAL(&cecMux);
}

void HomeTvCec::OnTransmitComplete(unsigned char* buffer, int count, bool ack)
{
    // // This is called after a frame is transmitted.
    // DbgPrint("Packet sent at %ld: %02X", millis(), buffer[0]);
    // for (int i = 1; i < count; i++)
    //   DbgPrint(":%02X", buffer[i]);
    // if (!ack)
    //   DbgPrint(" NAK");
    // DbgPrint("\n");

    if (count >= 3 && buffer[1] == CEC_SET_ACTIVE_SOURCE)
    {
        this->active_source = buffer[2] << 8 | buffer[3];
    }

    PrintIO('t', buffer, count, ack);
}

// void HomeTvCec::TransmitFrame(int fromAddress, int targetAddress, const unsigned char* buffer, int count)
// {
//   CEC_MESSAGE* msg = new CEC_MESSAGE;
//   msg->fromAddress = fromAddress;
//   msg->targetAddress = targetAddress;
//   msg->size = count;
//   memcpy(msg->data, buffer, count);
//   xQueueSend(this->queue_handle, &msg, portMAX_DELAY);
// }

void HomeTvCec::TransmitFrame(int targetAddress, const unsigned char* buffer, int count)
{
    CEC_MESSAGE* msg = new CEC_MESSAGE;
    //msg->fromAddress = fromAddress;
    msg->targetAddress = targetAddress;
    msg->size = count;
    memcpy(msg->data, buffer, count);

    printf("cec qx: ");
    for (int i = 0; i < msg->size; i++)
    {
      printf("%s%02x", i != 0 ? ":" : "", msg->data[i]);
    }
    printf("\n");

    xQueueSend(this->queue_handle, &msg, portMAX_DELAY);
}

bool HomeTvCec::Control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{
    if (LogicalAddress() == -1)
    {
        ESP_LOGE(TAG, "cec address not set");
        return false;
    }

    if ((reply == NULL && reply_size != NULL) || (reply_size != NULL && *reply_size < CEC_MAX_MSG_SIZE))
    {
        return false;
    }

    bool success = true;

    if (xSemaphoreTake(this->request_sem, CEC_REQUEST_WAIT / portTICK_PERIOD_MS))
    {
        portENTER_CRITICAL(&response_mux);
        xSemaphoreTake(responded_sem, 0);
        if (reply != NULL)
        {
            //ets_printf("zero reply %u\n", *reply_size);
            memset(reply, 0, *reply_size);
            this->reply = reply;
            this->reply_size = reply_size;
            this->reply_address = target_address == CEC_BROADCAST_ADDRESS ? -1 : target_address;
            this->reply_filter = reply_filter;
        }
        else
        {
            this->reply_size = 0;
        }
        portEXIT_CRITICAL(&response_mux);

        TransmitFrame(target_address, (unsigned char*)request, request_size);

        if (reply != NULL)
        {
            printf("cec wait: %02x:%02x\n", target_address << 4 | LogicalAddress(), reply_filter);
            if (xSemaphoreTake(responded_sem, CEC_RESPONSE_WAIT / portTICK_PERIOD_MS) != pdTRUE)
            {
                //memcpy(reply, this->reply, this->reply_size);
                //*reply_size = this->reply_size;
                success = false;
            }
        }
        //xSemaphoreTake(this->response_sem, portMAX_DELAY);
        //xSemaphoreGive(this->responded_sem);

        portENTER_CRITICAL(&response_mux);
        this->reply = NULL;
        this->reply_size = NULL;
        this->reply_address = -1;
        portEXIT_CRITICAL(&response_mux);

        xSemaphoreGive(this->request_sem);
    }

    return success;
}

bool HomeTvCec::Run()
{
    if (this->pending_message == NULL)
    {
        CEC_MESSAGE* msg;
        if (xQueueReceive(this->queue_handle, &msg, 0) == pdTRUE)
        {
            this->pending_message = msg;

            // printf("loaded %u ", this->pending_message->size);
            // for (int i = 0; i < this->pending_message->size; i++)
            // {
            //   printf("%s%02x", i != 0 ? ":" : "", this->pending_message->data[i]);
            // }
            // printf("\n");
        }
    }

    if (this->pending_message != NULL)
    {
        if (//(this->pending_message->fromAddress != -1 && CEC_Device::Transmit(this->pending_message->fromAddress, this->pending_message->targetAddress, this->pending_message->data, this->pending_message->size)) ||
            CEC_Device::TransmitFrame(this->pending_message->targetAddress, this->pending_message->data, this->pending_message->size))
        {
            // printf("transmitted ");
            // for (int i = 0; i < this->pending_message->size; i++)
            // {
            //   printf("%s%02x", i != 0 ? ":" : "", this->pending_message->data[i]);
            // }
            // printf("\n");
            delete this->pending_message;
            this->pending_message = NULL;
        }
    }

    return CEC_Device::Run();
}

void HomeTvCec::ClearPending()
{
    CEC_MESSAGE* msg;
    while (xQueueReceive(this->queue_handle, &msg, 0) == pdTRUE)
    {
        delete msg;
    }
}

void HomeTvCec::StandBy()
{
    uint8_t cmd[] = { 0x36 };

    TransmitFrame(CEC_BROADCAST_ADDRESS, cmd, sizeof(cmd));
}

void HomeTvCec::TvScreenOn()
{
    uint8_t cmd[] = { 0x04 };

    TransmitFrame(CEC_TV_ADDRESS, cmd, sizeof(cmd));
}

// void HomeTvCec::SetSystemAudioMode(bool on)
// {
//   uint8_t cmd[] = { 0x04, (uint8_t)(on ? 0x01 : 0x00) };
//   TransmitFrame(CEC_AUDIO_SYSTEM_ADDRESS, cmd, sizeof(cmd));
// }

bool HomeTvCec::SystemAudioModeRequest(uint16_t addr)
{
    uint8_t cmd[] = { 0x70, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xff) };
    uint8_t reply[CEC_MAX_MSG_SIZE];
    int reply_size = CEC_MAX_MSG_SIZE;
    return Control(CEC_AUDIO_SYSTEM_ADDRESS, cmd, sizeof(cmd), 0x72, reply, &reply_size) &&
        reply_size == 2 &&
        reply[2] == 0x01;
}

void HomeTvCec::SetActiveSource(uint16_t addr)
{
    uint8_t cmd[] = { CEC_SET_ACTIVE_SOURCE, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xff) };
    TransmitFrame(CEC_BROADCAST_ADDRESS, cmd, sizeof(cmd));
}

void HomeTvCec::UserControlPressed(int targetAddress, uint8_t userControl)
{
    uint8_t cmd[] = { 0x44, userControl };
    TransmitFrame(targetAddress, cmd, sizeof(cmd));
}

void HomeTvCec::WriteLog(httpd_req_t* request)
{
    portENTER_CRITICAL(&this->state_mux);
    log_entry_list entries_copy(this->log_entries);
    portEXIT_CRITICAL(&this->state_mux);

    char buf[32];

    for (log_entry_list::iterator it = entries_copy.begin(); it != entries_copy.end(); it++)
    {
        CEC_LOG_ENTRY& entry = *it;
        snprintf(buf, sizeof(buf), "%cx:", entry.direction);
        httpd_resp_send_chunk(request, buf, 3);
        for (int i = 0; i < entry.size; i++)
        {
            snprintf(buf, sizeof(buf), "%s%02x", i == 0 ? " " : ":", entry.data[i]);
            httpd_resp_send_chunk(request, buf, 3);
        }
        if (!entry.ack)
        {
            httpd_resp_send_chunk(request, " !", 2);
        }
        httpd_resp_send_chunk(request, "\n", 1);
    }

    httpd_resp_send_chunk(request, "END", 3);
    httpd_resp_send_chunk(request, NULL, 0);
}

void HomeTvCec::ClearLog()
{
    portENTER_CRITICAL(&this->state_mux);
    this->log_entries.clear();
    portEXIT_CRITICAL(&this->state_mux);
}

CEC_POWER_STATE HomeTvCec::GetPowerState(uint8_t addr)
{
    if (addr > CEC_MAX_ADDRESS)
    {
        return CEC_POWER_UNKNOWN;
    }
    
    portENTER_CRITICAL(&this->state_mux);
    uint8_t power_state = this->power_states[addr];
    portEXIT_CRITICAL(&this->state_mux);

    if (power_state == CEC_POWER_UNKNOWN)
    {
        return LoadPowerState(addr);
    }

    return (CEC_POWER_STATE)power_state;
}

CEC_POWER_STATE HomeTvCec::LoadPowerState(uint8_t addr)
{
    uint8_t reply[CEC_MAX_MSG_SIZE];
    uint8_t cmd[] = { 0x8f };
    int reply_size = CEC_MAX_MSG_SIZE;
    
    if (Control(addr, cmd, sizeof(cmd), CEC_POWER_STATUS, reply, &reply_size) && reply_size ==3)
    {
        ESP_LOGI(TAG, "addr 0x%02x loaded power state %d", addr, reply[2]);
        return (CEC_POWER_STATE)reply[2];
    }
    return CEC_POWER_UNKNOWN;
}

void cec_loop(void* param)
{
    for (;;)
    {
        ESP_LOGI(TAG, "Waiting for hotplug signal...");
        // FIXME
        // while (gpio_get_level(HDMI_HOTPLUG_GPIO_NUM) == 1)
        // { 
        //     vTaskDelay(50 / portTICK_PERIOD_MS);
        // }

        ESP_LOGI(TAG, "Hotplug signal detected!");

        uint8_t edid[DDC_EDID_LENGTH];
        uint8_t edid_extension[DDC_EDID_EXTENSION_LENGTH];

        do
        {
            for (int i = 0; i < DDC_EDID_LENGTH; i++)
            {
                ESP_ERROR_CHECK(ddc_read_byte(DDC_EDID_ADDRESS, i, &edid[i]));
                //printf("%02x ", edid[i]);
            }
            // printf("\n");
            ESP_LOGI(TAG, "Received EDID");        
            // vTaskDelay(5000 / portTICK_PERIOD_MS);
        } while (!cec_edid_parse(edid));

        if (edid[DDC_EDID_EXTENSION_FLAG])
        {
            do
            {
                for (int i = 0; i < DDC_EDID_EXTENSION_LENGTH; i++)
                {
                    ESP_ERROR_CHECK(ddc_read_byte(DDC_EDID_ADDRESS, DDC_EDID_LENGTH + i, &edid_extension[i]));
                }

                ESP_LOGI(TAG, "Received EDID extension");

            } while (!cec_edid_extension_parse(edid, edid_extension));
        }
        cec_device = new HomeTvCec;
        
        cec_device->Initialize(cec_physical_address, CEC_DEVICE_TYPE, true); // Promiscuous mode}

        for (;;)
        {
            // FIXME
            // if (gpio_get_level(gpio_get_level(HDMI_HOTPLUG_GPIO_NUM) == 1)
            // {
            //     ESP_LOGE(TAG, "hdmi cable unplugged");
            //     break;
            // }

            int busy = 0;

            int64_t start_time = esp_timer_get_time();

            while (cec_device->Run())
            {
                busy++;
            }
            // int64_t busy_ms = (esp_timer_get_time() - start_time) / 1000;

            // if (busy > 0)
            // {
            //     ESP_LOGI(TAG, "cec busy for %d iterations / %lld ms", busy, busy_ms);
            // }

            taskYIELD();
        }

        delete cec_device;
    }

    vTaskDelete(NULL);
}

esp_err_t cec_queue_clear()
{
    if (cec_device != NULL)
    {
        cec_device->ClearPending();
    }
    return ESP_OK;
}


esp_err_t cec_standby()
{
    if (cec_device != NULL)
    {
        cec_device->StandBy();
    }

    return ESP_OK;
}

esp_err_t cec_pause()
{
    if (cec_device != NULL)
    {
        cec_device->UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, 0x46);
    }

    return ESP_OK;
}

esp_err_t cec_play()
{
    if (cec_device != NULL)
    {
        cec_device->UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, 0x44);
    }

    return ESP_OK;
}

esp_err_t cec_combine_devices_state(bool* state, bool and_mode, bool tv, bool audio, bool atv)
{
    if (cec_device == NULL)
    {
        ESP_LOGE(TAG, "%s cec_device is NULL", __func__);
        *state = false;
        return ESP_OK;
    }

    bool combined_on = and_mode;

    // auto check_device = [and_mode, &tv_on](int target_address) {
    //     uint8_t reply[CEC_MAX_MSG_SIZE];
    //     auto get_power_state = [target_address, &reply]() {
    //         uint8_t cmd[] = { 0x8f };
    //         int reply_size = CEC_MAX_MSG_SIZE;
    //         return device.Control(target_address, cmd, sizeof(cmd), CEC_POWER_STATUS, reply, &reply_size) && reply_size == 3;
    //     };

    //     if ((!tv_on && !and_mode) || (tv_on && and_mode))
    //     {
    //         if (call_with_retry(get_power_state, 100))
    //         {
    //             //printf("reply %02x:%02x:%02x\n", reply[0], reply[1], reply[2]);
    //             bool device_on = reply[2] == 0 || reply[2] == 2;
    //             if (and_mode)
    //             {
    //                 tv_on &= device_on;
    //             }
    //             else
    //             {
    //                 tv_on |= device_on;
    //             }
    //         }
    //         else
    //         {
    //             if (and_mode)
    //             {
    //                 printf("control failed, assuming off\n");
    //                 tv_on = false;
    //             }
    //             else
    //             {
    //                 printf("control failed, keeping tv_on=%u\n", tv_on);
    //             }
    //         }
    //     }
    // };

    auto check_device = [and_mode, &combined_on](int target_address, const char* name) {

        if (cec_device != NULL)
        {
            auto power_state = cec_device->GetPowerState(target_address);
            bool device_on = power_state == CEC_POWER_ON || power_state == CEC_POWER_TRANS_ON;            

            if (and_mode)
            {
                combined_on &= device_on;
            }
            else
            {
                combined_on |= device_on;
            }

            ESP_LOGI(TAG, "%s is %s, all combined is %s", name, device_on ? "on" : "off", combined_on ? "on" : "off");
        }        
    };

    if (tv)
    {
        check_device(CEC_TV_ADDRESS, "tv");
    }
    if (audio)
    {
        check_device(CEC_AUDIO_SYSTEM_ADDRESS, "audio");
    }
    if (atv)
    {
        check_device(CEC_PLAYBACK_DEVICE_1_ADDRESS, "atv");
    }

    *state = combined_on;

    return ESP_OK;
}

void cec_tv_state_task(void* param)
{
    bool desired_state = (bool)param;

    ESP_LOGI(TAG, "set_tv_state = %u requested", desired_state);

    void (*change_state)() = NULL;

    if (desired_state)
    {
        change_state = [] {
            if (cec_device != NULL)
            {
                uint16_t addr = CEC_TV_HDMI_INPUT << 12 | CEC_ATV_HDMI_INPUT << 8;
                cec_device->SetActiveSource(addr);
                cec_device->TvScreenOn();
                cec_device->SystemAudioModeRequest(addr);
                cec_device->UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, CEC_USER_CONTROL_POWER_ON);
            }
        };
    }
    else
    {
        change_state = [] 
        { 
            if (cec_device != NULL)
            {
                cec_device->StandBy(); 
            }
        };
    }

    if (tvmux_call_with_retry(TV_RETRY_FUNC, change_state, [desired_state] 
        { 
            bool state;
            if (cec_combine_devices_state(&state, desired_state) == ESP_OK)
            {
                return state == desired_state;
            }
            return false;
        ; }))
    {
        ESP_LOGI(TAG, "set_tv_state = %u succeeded", desired_state);
    }
    else
    {
        ESP_LOGE(TAG, "set_tv_state = %u failed", desired_state);
    }

    vTaskDelete(NULL);
}

esp_err_t cec_tv_power(bool power_on)
{
    xTaskCreate(cec_tv_state_task, "tv_state", 4000, (void*)power_on, 1, NULL);

    return ESP_OK;
}


void wii_state_task(void* param)
{
    bool desired_state = (bool)param;

    wii_power_status_t(*change_state)() = NULL;
    bool desired_av_state = desired_state;
    wii_power_state_t desired_wii_state = desired_av_state ? WII_POWER_STATE_ON : WII_POWER_STATE_OFF;

        ESP_LOGI(TAG, "set_wii_state = %u requested", desired_state);

    if (desired_state)
    {
        change_state = [] {
            auto status = wii_power_on();
            uint16_t addr = CEC_TV_HDMI_INPUT << 12 | CEC_WII_HDMI_INPUT << 8;
            if (cec_device != NULL)
            {
                cec_device->SetActiveSource(addr);
                cec_device->TvScreenOn();
                cec_device->SystemAudioModeRequest(addr);
            }
            return status;
        };
    }
    else
    {
        change_state = [] {
            auto status = wii_power_off();
            if (cec_device != NULL)
            {
                cec_device->StandBy();
            }
            return status;
        };
    }

    if (tvmux_call_with_retry(WII_RETRY_FUNC, change_state, [desired_av_state, desired_wii_state]
        { 
            bool state;
            return wii_query_power_state() == desired_wii_state &&
                cec_combine_devices_state(&state, desired_av_state, true, true, false) == ESP_OK && state == desired_av_state; 
        }))
    {
        ESP_LOGI(TAG, "set_wii_state = %u succeeded", desired_state);
    }
    else
    {
        ESP_LOGE(TAG, "set_wii_state = %u failed", desired_state);
    }

    vTaskDelete(NULL);
}

esp_err_t cec_wii_power(bool power_on)
{
    xTaskCreate(wii_state_task, "wii_state", 4000, (void*)power_on, 1, NULL);
    return ESP_OK;
}

esp_err_t cec_log_write(httpd_req_t* request)
{
    if (cec_device != NULL)
    {
        cec_device->WriteLog(request);
    }

    return ESP_OK;
}

esp_err_t cec_log_clear()
{
    if (cec_device != NULL)
    {
        cec_device->ClearLog();
    }

    return ESP_OK;
}


esp_err_t cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{
    if (cec_device != NULL)
    {
        cec_device->Control(target_address, request, request_size, reply_filter, reply, reply_size);
    }

    return ESP_OK;
}

esp_err_t cec_as_set(uint16_t addr)
{
    if (cec_device != NULL)
    {
        cec_device->SetActiveSource(addr);
    }

    return ESP_OK;
}

esp_err_t cec_sam_request(uint16_t addr)
{
    if (cec_device != NULL)
    {
        cec_device->SystemAudioModeRequest(addr);               
    }

    return ESP_OK;
}

esp_err_t cec_tv_on()
{
    if (cec_device != NULL)
    {
        cec_device->TvScreenOn();
    }

    return ESP_OK;
}

bool cec_edid_parse(unsigned char* edid)
{
    uint32_t header0 = edid[0] << 24 | edid[1] << 16 | edid[2] << 8 | edid[3];
    uint32_t header1 = edid[4] << 24 | edid[5] << 16 | edid[6] << 8 | edid[7];

    uint32_t sum = 0;
    for (int i = 0; i < DDC_EDID_LENGTH; i++)
    {
        sum += (uint32_t)edid[i];
    }

    //ESP_LOGI(TAG, "EDID checksum: %08lx", sum);
    //ESP_LOGI(TAG, "EDID header: %08lx%08lx", header0, header1);

    if (header0 != 0x00ffffff || header1 != 0xffffff00)
    {
        return false;
    }

    if (sum % 256 != 0)
    {
        return false;
    }

    uint16_t manufacturer0 = edid[0x08];
    uint16_t manufacturer1 = edid[0x09];
    char manufacturer[4];
    manufacturer[0] = '@' + (int)((manufacturer0 >> 2) & 0x1f);
    manufacturer[1] = '@' + (int)((manufacturer0 & 3) << 3 | ((manufacturer1 >> 5) & 0x7));
    manufacturer[2] = '@' + (int)(manufacturer1 & 0x1f);
    manufacturer[3] = 0;

    uint8_t version = edid[0x12];
    uint8_t revision = edid[0x13];
    //uint8_t extension_flag = edid[EDID_EXTENSION_FLAG];

    ESP_LOGI(TAG, "EDID version: %u.%u", version, revision);
    ESP_LOGI(TAG, "EDID manufacturer: %s", manufacturer);
    //ESP_LOGI(TAG, "EDID extension_flag: %d", extension_flag);

    return true;
    /*

      uint8_t ieee0 = edid[0x95];
      uint8_t ieee1 = edid[0x96];
      uint8_t ieee2 = edid[0x97];
      uint16_t physicalAddress = edid[0x98] << 8 | edid[0x99];

      ESP_LOGI(TAG, "IEEE ID: %02x%02x%02x", ieee0, ieee1, ieee2);
      uint8_t a0 = physicalAddress >> 12;
      uint8_t a1 = (physicalAddress >> 8) & 0xf;
      uint8_t a2 = (physicalAddress >> 4) & 0xf;
      uint8_t a3 = physicalAddress & 0xf;
      ESP_LOGI(TAG, "cPhysical Address: %u.%u.%u.%u", a0, a1, a2, a3);

      return true;*/
}

bool cec_edid_extension_parse(uint8_t* edid2, uint8_t* ext)
{
    uint32_t sum = 0;
    for (int i = 0; i < DDC_EDID_EXTENSION_LENGTH; i++)
    {
        sum += (uint32_t)ext[i];
    }

    if (sum % 256 != 0)
    {
        return false;
    }

    //ESP_LOGI(TAG, "EDID ext checksum: %08x", sum);

    uint8_t tag = ext[0];
    //uint8_t revision = ext[1];
    uint8_t dtd_offset = ext[2];
    uint8_t offset = 4;

    // ESP_LOGI(TAG, "EDID ext tag: %u", tag);
    // ESP_LOGI(TAG, "EDID ext revision: %u", revision);
    // ESP_LOGI(TAG, "EDID ext dtd_offset: %u", dtd_offset);

    if (tag != 2)
    {
        return false;
    }

    // for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
    // {
    //   ESP_LOGI(TAG, "0x%02x, ", ext[i]);
    // }
    // ESP_LOGI(TAG, );

    uint8_t index = offset;

    while (index < dtd_offset)
    {
        uint8_t* p = ext + index;
        uint8_t tag = p[0] >> 5;
        uint8_t length = p[0] & 0x1f;

        //ESP_LOGI(TAG, "EDID ext tag: %d length: %d", tag, length);

        switch (tag)
        {
            case 3:
            {
                uint8_t ieee[3];
                ieee[0] = p[3];
                ieee[1] = p[2];
                ieee[2] = p[1];
                //ESP_LOGI(TAG, "EDID IEEE %02x %02x %02x", ieee[0], ieee[1], ieee[2]);
                if (ieee[0] == 0x00 && ieee[1] == 0x0c && ieee[2] == 0x03)
                {
                    cec_physical_address = (uint16_t)p[4] << 8 | p[5];
                    uint8_t a0 = cec_physical_address >> 12;
                    uint8_t a1 = (cec_physical_address >> 8) & 0xf;
                    uint8_t a2 = (cec_physical_address >> 4) & 0xf;
                    uint8_t a3 = cec_physical_address & 0xf;

                    ESP_LOGI(TAG, "CEC Physical Address: %u.%u.%u.%u", a0, a1, a2, a3);

                }
                break;
            }
        }

        index += 1 + length;
    }

    return true;
}
