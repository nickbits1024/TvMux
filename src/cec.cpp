#include <Arduino.h>
#include <list>
#include <iomanip>
#include <string>
#include <sstream>
#include "cec.h"

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


HomeTvCec::HomeTvCec()  :
  pendingMessage(NULL),
  response_mux(portMUX_INITIALIZER_UNLOCKED),
  reply(NULL),
  reply_size(NULL),
  reply_address(-1)
{  
  this->queueHandle = xQueueCreate(100, sizeof(CEC_MESSAGE*));
  request_sem = xSemaphoreCreateBinary();
  //response_sem = xSemaphoreCreateBinary();
  responded_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(request_sem);
  xSemaphoreGive(responded_sem);
}

bool HomeTvCec::LineState()
{
  int state = digitalRead(CEC_GPIO_INPUT);
  return state != LOW;
}

void HomeTvCec::SetLineState(bool state)
{
  if (state)
  {
    //pinMode(CEC_GPIO, INPUT_PULLUP);
    digitalWrite(CEC_GPIO_OUTPUT, HIGH);
  }
  else
  {
    //digitalWrite(CEC_GPIO, LOW);
    //pinMode(CEC_GPIO, OUTPUT);
    digitalWrite(CEC_GPIO_OUTPUT, LOW);
  }
  // give enough time for the line to settle before sampling it
  delayMicroseconds(16);
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

  DbgPrint("CEC Logical Address: : %d\n", logicalAddress);

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

void print_io(const char* prefix, unsigned char* buffer, int size, bool ack)
{
  // std::stringstream ss;

  // ss << millis() << " " << prefix << " ";


  // format_bytes(ss, buffer, count);

  // if (!ack)
  // {
  //   ss << " !";
  // }

  // // // This is called when a frame is received.  To transmit
  // // // a frame call TransmitFrame.  To receive all frames, even
  // // // those not addressed to this device, set Promiscuous to true.
  // // DbgPrint("Packet received at %ld: %02x", millis(), buffer[0]);
  // // for (int i = 1; i < count; i++)
  // //   DbgPrint(":%02X", buffer[i]);
  // // if (!ack)
  // //   DbgPrint(" NAK");
  // // DbgPrint("\n");

  // // std::string s;

  // // for (int i = 0; i < count; i++)
  // // {
  // //   char hex[3];
  // //   snprintf(hex, 3, "%02x", buffer[i]);
  // //   s += hex;
  // // }

  // std::string s(ss.str());

  // println(s.c_str());

  // portENTER_CRITICAL(&historyMux);
  // history.push_front(s);
  // if (history.size() > CEC_MAX_HISTORY)
  // {
  //   history.pop_back();
  // }
  // portEXIT_CRITICAL(&historyMux);

  printf("%s:", prefix);
  for (int i = 0; i < size; i++)
  {
    printf(" %02x", buffer[i]);
  } 
  if (!ack)
  {
    printf(" !");
  }
  printf("\n");
}

void HomeTvCec::OnReceiveComplete(unsigned char* buffer, int size, bool ack)
{
  // No command received?
  if (size < 2)
    return;

  print_io("rx", buffer, size, ack);

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

  print_io("tx", buffer, count, ack);
}

// void HomeTvCec::TransmitFrame(int fromAddress, int targetAddress, const unsigned char* buffer, int count)
// {
//   CEC_MESSAGE* msg = new CEC_MESSAGE;
//   msg->fromAddress = fromAddress;
//   msg->targetAddress = targetAddress;
//   msg->size = count;
//   memcpy(msg->data, buffer, count);
//   xQueueSend(this->queueHandle, &msg, portMAX_DELAY);
// }

void HomeTvCec::TransmitFrame(int targetAddress, const unsigned char* buffer, int count)
{
  CEC_MESSAGE* msg = new CEC_MESSAGE;
  //msg->fromAddress = fromAddress;
  msg->targetAddress = targetAddress;
  msg->size = count;
  memcpy(msg->data, buffer, count);
  xQueueSend(this->queueHandle, &msg, portMAX_DELAY);
}

bool HomeTvCec::Control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{
  if ((reply == NULL && reply_size != NULL) || (reply_size != NULL && *reply_size < CEC_MAX_MSG_SIZE))
  {
    return false;
  }

  bool success = true;

  if (xSemaphoreTake(request_sem, 5000 / portTICK_PERIOD_MS))
  {
    portENTER_CRITICAL(&response_mux);
    xSemaphoreTake(responded_sem, 0);
    if (reply != NULL)
    {
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
    //xSemaphoreTake(responded_sem, portMAX_DELAY);
    //xSemaphoreGive(response_sem);

    TransmitFrame(target_address, (unsigned char*)request, request_size);

    if (reply != NULL)
    {
      printf("Waiting for reply %02x\n", reply_filter);
      if (xSemaphoreTake(responded_sem, 5000 / portTICK_PERIOD_MS) != pdTRUE)
      {
        //memcpy(reply, this->reply, this->reply_size);
        //*reply_size = this->reply_size;
        success = false;
      }
    }
    //xSemaphoreTake(this->response_sem, portMAX_DELAY);
    //xSemaphoreGive(this->responded_sem);
    xSemaphoreGive(this->request_sem);
  }

  portENTER_CRITICAL(&response_mux);
  this->reply = NULL;
  this->reply_size = NULL;
  this->reply_address = -1;
  portEXIT_CRITICAL(&response_mux);

  return success;
}

void HomeTvCec::Run()
{
  if (this->pendingMessage == NULL)
  {
    CEC_MESSAGE* msg;
    if (xQueueReceive(this->queueHandle, &msg, 0) == pdTRUE)
    {
      this->pendingMessage = msg;
    }
  }

  if (this->pendingMessage != NULL)
  {
    if (//(this->pendingMessage->fromAddress != -1 && CEC_Device::Transmit(this->pendingMessage->fromAddress, this->pendingMessage->targetAddress, this->pendingMessage->data, this->pendingMessage->size)) ||
      CEC_Device::TransmitFrame(this->pendingMessage->targetAddress, this->pendingMessage->data, this->pendingMessage->size))
    {
      delete this->pendingMessage;
      this->pendingMessage = NULL;
    }
  }

  CEC_Device::Run();
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

void HomeTvCec::SetSystemAudioMode(bool on)
{
  uint8_t cmd[] = { 0x04, (uint8_t)(on ? 0x01 : 0x00) };
  TransmitFrame(CEC_AUDIO_SYSTEM_ADDRESS, cmd, sizeof(cmd));
}

void HomeTvCec::SystemAudioModeRequest(uint16_t addr)
{
  uint8_t cmd[] = { 0x70, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xff) };
  TransmitFrame(CEC_AUDIO_SYSTEM_ADDRESS, cmd, sizeof(cmd)); 
}

// void HomeTvCec::SetActiveSource(uint16_t addr)
// {
//   uint8_t cmd[] = { 0x82, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xff) };
//   TransmitFrame(CEC_BROADCAST_ADDRESS, cmd, sizeof(cmd)); 
// }

void HomeTvCec::UserControlPressed(int targetAddress, uint8_t userControl)
{
  uint8_t cmd[] = { 0x44, userControl };
  TransmitFrame(targetAddress, cmd, sizeof(cmd));
}