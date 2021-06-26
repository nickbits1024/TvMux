#include <Arduino.h>
#include <list>
#include <iomanip>
#include <string>
#include <sstream>
#include "cec.h"

extern std::list<std::string> history;
extern uint16_t cec_physical_address;
extern portMUX_TYPE cecMux;
extern portMUX_TYPE historyMux;
extern xSemaphoreHandle response_sem;
extern xSemaphoreHandle responded_sem;
extern unsigned char reply[CEC_MAX_MSG_SIZE];
extern int reply_length;
extern unsigned char reply_filter;


HomeTvCec::HomeTvCec()
{

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

void add_history(const char* prefix, unsigned char* buffer, int count, bool ack)
{
  std::stringstream ss;

  ss << millis() << " " << prefix << " ";


  format_bytes(ss, buffer, count);

  if (!ack)
  {
    ss << " !";
  }

  // // This is called when a frame is received.  To transmit
  // // a frame call TransmitFrame.  To receive all frames, even
  // // those not addressed to this device, set Promiscuous to true.
  // DbgPrint("Packet received at %ld: %02x", millis(), buffer[0]);
  // for (int i = 1; i < count; i++)
  //   DbgPrint(":%02X", buffer[i]);
  // if (!ack)
  //   DbgPrint(" NAK");
  // DbgPrint("\n");

  // std::string s;

  // for (int i = 0; i < count; i++)
  // {
  //   char hex[3];
  //   snprintf(hex, 3, "%02x", buffer[i]);
  //   s += hex;
  // }

  std::string s(ss.str());

  Serial.println(s.c_str());

  portENTER_CRITICAL(&historyMux);
  history.push_front(s);
  if (history.size() > CEC_MAX_HISTORY)
  {
    history.pop_back();
  }
  portEXIT_CRITICAL(&historyMux);
}

void HomeTvCec::OnReceiveComplete(unsigned char* buffer, int count, bool ack)
{
  // No command received?
  if (count < 2)
    return;

  add_history("rx", buffer, count, ack);

  // Ignore messages not sent to us
  if ((buffer[0] & 0xf) != LogicalAddress())
    return;

  //portENTER_CRITICAL(&dataMux);
  if (xSemaphoreTake(response_sem, 0))
  {
    if (reply_length != 0)
    {
      if (buffer[1] == reply_filter && count <= CEC_MAX_MSG_SIZE)
      {
        memcpy(reply, buffer, count);
        reply_length = count;
        xSemaphoreGive(responded_sem);
      }
    }
    xSemaphoreGive(response_sem);
  }
  //portEXIT_CRITICAL(&dataMux);
  portENTER_CRITICAL(&cecMux);
  switch (buffer[1])
  {
  case 0x83:
    { // <Give Physical Address>
      //unsigned char buf[4] = { 0x84, CEC_PHYSICAL_ADDRESS >> 8, CEC_PHYSICAL_ADDRESS & 0xff, CEC_DEVICE_TYPE };
      unsigned char buf[4] =
      {
        0x84,
        (unsigned char)(cec_physical_address >> 8),
        (unsigned char)(cec_physical_address & 0xff),
        CEC_DEVICE_TYPE
      };
      TransmitFrame(0xf, buf, 4); // <Report Physical Address>
      break;
    }
  case 0x8c: // <Give Device Vendor ID>
    TransmitFrame(0xf, (unsigned char*)"\x87\x01\x23\x45", 4); // <Device Vendor ID>
    break;
  }
  portEXIT_CRITICAL(&cecMux);
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

  add_history("tx", buffer, count, ack);
}
