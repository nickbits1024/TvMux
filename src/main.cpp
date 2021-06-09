#include <Arduino.h>
#include "CEC_Device.h"

#define CEC_GPIO 5
#define CEC_DEVICE_TYPE CEC_Device::CDT_PLAYBACK_DEVICE
#define CEC_PHYSICAL_ADDRESS 0x2000

class MyCEC_Device : public CEC_Device
{
protected:
  virtual bool LineState();
  virtual void SetLineState(bool);
  virtual void OnReady(int logicalAddress);
  virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack);
  virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack);
};

bool MyCEC_Device::LineState()
{
  int state = digitalRead(CEC_GPIO);
  return state != LOW;
}

void MyCEC_Device::SetLineState(bool state)
{
  if (state)
  {
    pinMode(CEC_GPIO, INPUT_PULLUP);
  }
  else
  {
    digitalWrite(CEC_GPIO, LOW);
    pinMode(CEC_GPIO, OUTPUT);
  }
  // give enough time for the line to settle before sampling it
  delayMicroseconds(50);
}

void MyCEC_Device::OnReady(int logicalAddress)
{
  // This is called after the logical address has been allocated

  unsigned char buf[4] = { 0x84, CEC_PHYSICAL_ADDRESS >> 8, CEC_PHYSICAL_ADDRESS & 0xff, CEC_DEVICE_TYPE };

  DbgPrint("Device ready, Logical address assigned: %d\n", logicalAddress);

  TransmitFrame(0xf, buf, 4); // <Report Physical Address>
}

void MyCEC_Device::OnReceiveComplete(unsigned char* buffer, int count, bool ack)
{
  // This is called when a frame is received.  To transmit
  // a frame call TransmitFrame.  To receive all frames, even
  // those not addressed to this device, set Promiscuous to true.
  DbgPrint("Packet received at %ld: %02X", millis(), buffer[0]);
  for (int i = 1; i < count; i++)
    DbgPrint(":%02X", buffer[i]);
  if (!ack)
    DbgPrint(" NAK");
  DbgPrint("\n");

  // Ignore messages not sent to us
  if ((buffer[0] & 0xf) != LogicalAddress())
    return;

  // No command received?
  if (count < 1)
    return;

  switch (buffer[1])
  {
  case 0x83: { // <Give Physical Address>
    unsigned char buf[4] = { 0x84, CEC_PHYSICAL_ADDRESS >> 8, CEC_PHYSICAL_ADDRESS & 0xff, CEC_DEVICE_TYPE };
    TransmitFrame(0xf, buf, 4); // <Report Physical Address>
    break;
  }
  case 0x8c: // <Give Device Vendor ID>
    TransmitFrame(0xf, (unsigned char*)"\x87\x01\x23\x45", 4); // <Device Vendor ID>
    break;
  }
}

void MyCEC_Device::OnTransmitComplete(unsigned char* buffer, int count, bool ack)
{
  // This is called after a frame is transmitted.
  DbgPrint("Packet sent at %ld: %02X", millis(), buffer[0]);
  for (int i = 1; i < count; i++)
    DbgPrint(":%02X", buffer[i]);
  if (!ack)
    DbgPrint(" NAK");
  DbgPrint("\n");
}


MyCEC_Device device;

void setup()
{
  pinMode(CEC_GPIO, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial);

  device.Initialize(CEC_PHYSICAL_ADDRESS, CEC_DEVICE_TYPE, true); // Promiscuous mode}
}

void loop()
{
  if (Serial.available())
  {
    unsigned char c = Serial.read();
    unsigned char buffer[3];

    switch (c)
    {
    case '1':
      buffer[0] = 0x36;
      device.TransmitFrame(4, buffer, 1);
      break;
    }
  }
  device.Run();
}