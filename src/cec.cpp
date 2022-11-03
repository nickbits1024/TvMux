#include <Arduino.h>
#include <list>
#include <iomanip>
#include <string>
#include <sstream>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_http_server.h>
#include "cec.h"
#include "hometv.h"

//extern std::list<std::string> history;
extern uint16_t cec_physical_address;
//extern portMUX_TYPE cecMux;
//extern portMUX_TYPE historyMux;
// extern xSemaphoreHandle response_sem;
// extern xSemaphoreHandle responded_sem;
// extern unsigned char reply[CEC_MAX_MSG_SIZE];
// extern int reply_length;
// extern unsigned char reply_filter;
// extern int reply_address;


HomeTvCec::HomeTvCec() :
    pending_message(NULL),
    response_mux(portMUX_INITIALIZER_UNLOCKED),
    state_mux(portMUX_INITIALIZER_UNLOCKED),
    reply(NULL),
    reply_size(NULL),
    reply_address(-1),
    active_source(-1),
    power_states { CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF, 
                   CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF, 
                   CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF, 
                   CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF, CEC_POWER_OFF }
{
    this->queue_handle = xQueueCreate(100, sizeof(CEC_MESSAGE*));
    request_sem = xSemaphoreCreateBinary();
    //response_sem = xSemaphoreCreateBinary();
    responded_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(request_sem);
    xSemaphoreGive(responded_sem);

    //PrintIO('r', (uint8_t*)"123", 3, false);
    //PrintIO('t', (uint8_t*)"456789", 6, true);
}

bool HomeTvCec::LineState()
{
    int state = digitalRead(CEC_GPIO_INPUT);
    return state != LOW;
}

void HomeTvCec::SetLineState(bool state)
{
    digitalWrite(CEC_GPIO_OUTPUT, state ? HIGH : LOW);
    // give enough time for the line to settle before sampling it
    delayMicroseconds(50);
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

void format_bytes(std::stringstream& ss, unsigned char* buffer, int count)
{
    for (int i = 0; i < count; i++)
    {
        if (i != 0)
        {
            ss << ":";
        }
        ss << std::hex << std::setfill('0') << std::setw(2) << (int)buffer[i];
    }
}

void format_bytes(std::string& s, unsigned char* buffer, int count)
{
    std::stringstream ss;

    format_bytes(ss, buffer, count);

    s = ss.str();
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

    int8_t addr;

    switch (buffer[1])
    {
        case CEC_SET_ACTIVE_SOURCE:
        {
            ESP_LOGI(TAG, "TV trying to steal active source!");

            if (this->active_source != -1)
            {
                ESP_LOGI(TAG, "Restoring source %02x", this->active_source);
                SetActiveSource(this->active_source);
            }
            else
            {
                this->active_source = buffer[2] << 8 | buffer[3];
                ESP_LOGI(TAG, "Saving source %02x", this->active_source);
            }
            break;
        }
        case CEC_POWER_STATUS:
            portENTER_CRITICAL(&state_mux);    
            addr = buffer[0] >> 4;
            this->power_states[addr] = buffer[2];
            portEXIT_CRITICAL(&state_mux);
            ets_printf("addr 0x%02x power state = %u\n", addr, buffer[2]);
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

    // printf("queue ");
    // for (int i = 0; i < msg->size; i++)
    // {
    //   printf("%s%02x", i != 0 ? ":" : "", msg->data[i]);
    // }
    // printf("\n");

    xQueueSend(this->queue_handle, &msg, portMAX_DELAY);
}

bool HomeTvCec::Control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{
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

void HomeTvCec::Run()
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

    CEC_Device::Run();
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
        return INVALID_POWER_STATE;
    }
    
    portENTER_CRITICAL(&this->state_mux);
    uint8_t power_state = this->power_states[addr];
    portEXIT_CRITICAL(&this->state_mux);

    return (CEC_POWER_STATE)power_state;
}

void HomeTvCec::LoadPowerState(uint8_t addr)
{
    uint8_t reply[CEC_MAX_MSG_SIZE];
    uint8_t cmd[] = { 0x8f };
    int reply_size = CEC_MAX_MSG_SIZE;
    
    Control(addr, cmd, sizeof(cmd), CEC_POWER_STATUS, reply, &reply_size);
}