#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <AsyncJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <vector>
#include "CEC_Device.h"
#include "DDCVCP.h"

#define WIFI_SSID             "ac55.wifi.nickpalmer.net"
#define WIFI_PASS             "B16b00b5"
#define CEC_GPIO              5
#define CEC_DEVICE_TYPE       CEC_Device::CDT_PLAYBACK_DEVICE
#define CEC_PHYSICAL_ADDRESS  0x4000
#define ONBOARD_LED           2
#define EDID_ADDRESS          0x50
#define EDID_LENGTH           128

AsyncWebServer server(80);
DDCVCP ddc;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
std::vector<std::string> history;

class MyCEC_Device : public CEC_Device
{
protected:
  virtual bool LineState();
  virtual void SetLineState(bool);
  virtual void OnReady(int logicalAddress);
  virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack);
  virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack);

public:
  MyCEC_Device();
};

MyCEC_Device::MyCEC_Device()
{

}

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
  // No command received?
  if (count < 1)
    return;

  // This is called when a frame is received.  To transmit
  // a frame call TransmitFrame.  To receive all frames, even
  // those not addressed to this device, set Promiscuous to true.
  DbgPrint("Packet received at %ld: %02x", millis(), buffer[0]);
  for (int i = 1; i < count; i++)
    DbgPrint(":%02X", buffer[i]);
  if (!ack)
    DbgPrint(" NAK");
  DbgPrint("\n");

  std::string s;

  for (int i = 0; i < count; i++)
  {
    char hex[3];
    snprintf(hex, 3, "%02x", buffer[i]);
    s += hex;
  }

  // Ignore messages not sent to us
  if ((buffer[0] & 0xf) != LogicalAddress())
    return;   

  portENTER_CRITICAL(&mux);
  history.push_back(s);
  if (history.size() > 16)
  {
    history.erase(history.begin());
  }
  portEXIT_CRITICAL(&mux);

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

MyCEC_Device device;

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

void connectWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  do
  {
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  //ESP has tendency to store old SSID and PASSword and tries to connect
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("Connecting to WiFi...");
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++)
    {
      delay(2000);
      Serial.print("*");
    }

  } while (WiFi.status() != WL_CONNECTED);

  Serial.println("Connected to WiFi");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print(rssi);
  Serial.println(" dBm");
}

void handle404(AsyncWebServerRequest* request)
{
  request->send(404);
}

void handleHeap(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();

  multi_heap_info_t info;
  heap_caps_get_info(&info, MALLOC_CAP_INTERNAL);

  doc["heap_caps_get_largest_free_block"] = ESP.getMaxAllocHeap();
  doc["total_free_bytes"] = info.total_free_bytes;
  doc["total_allocated_bytes"] = info.total_allocated_bytes;
  doc["heap_caps_get_minimum_free_size"] = ESP.getMinFreeHeap();  

  response->setLength();
  request->send(response);
}

void handleHistory(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(true, 256);
  auto doc = response->getRoot();

  portENTER_CRITICAL(&mux);
  for (std::vector<std::string>::iterator it = history.begin(); it != history.end(); it++)
  {
    doc.add(it->c_str());
  }
  portEXIT_CRITICAL(&mux);

  response->setLength();
  request->send(response);
}

void handleStandby(AsyncWebServerRequest* request)
{
  portENTER_CRITICAL(&mux);
  char buffer[1];

  buffer[0] = 0x36;
  device.TransmitFrame(0xf, (unsigned char*)buffer, 1);
  
  portEXIT_CRITICAL(&mux);

  auto response = new PrettyAsyncJsonResponse(false, 16);

  response->setLength();
  request->send(response);
}

void handleTransmit(AsyncWebServerRequest* request)
{
  if (!request->hasArg("target") || !request->hasArg("cmd"))
  {
    request->send(500);
    return;
  }

  int target = request->arg("target").toInt();
  String cmd = request->arg("cmd");

  char buffer[256];
  int len = cmd.length() / 2;
  if (len > sizeof(buffer))
  {
    request->send(500);
    return;
  }
  Serial.printf("cmd=%s\n", cmd.c_str());
  for (int i = 0; i < len; i++)
  {
    String hex = cmd.substring(i * 2, i * 2 + 2);
    Serial.printf("hex=%s\n", hex.c_str());
    unsigned int temp;
    sscanf(hex.c_str(), "%02x", &temp);
    buffer[i] = (char)temp;
  }

  String temp;

  for (int i = 0; i < len; i++)
  {
    char hex[3];
    snprintf(hex, 3, "%02x", buffer[i]);
    temp += hex;
  }
  Serial.printf("reconstructed: %s\n", temp.c_str());

  portENTER_CRITICAL(&mux);
  
  device.TransmitFrame(target, (unsigned char*)buffer, len);

  portEXIT_CRITICAL(&mux);

  auto response = new PrettyAsyncJsonResponse(true, 256);
  auto doc = response->getRoot();

  doc["target"] = target;
  doc["cmd"] = temp;

  request->send(response);
}

void parseEdid(unsigned char* edid)
{
  uint32_t header = *((uint32_t*)edid);
  uint16_t manufacturer = *((uint16_t*)(edid + 0x08));
  uint8_t version = edid[0x12];
  uint8_t revision = edid[0x13];
  uint8_t ieee0 = edid[0x95];
  uint8_t ieee1 = edid[0x96];
  uint8_t ieee2 = edid[0x97];
  uint16_t physicalAddress = *((uint16_t*)(edid + 0x98));

  uint8_t sum = 0;
  for (int i = 0; i < EDID_LENGTH; i++)
  {
    sum += edid[0];
  }
  Serial.printf("EDID checksum: %u\n", sum);
  Serial.printf("EDID header: %08x\n", header);
  Serial.printf("EDID version: %u revision: %u\n", version, revision);
  Serial.printf("EDID manufacturer: %04x\n", manufacturer);
  Serial.printf("IEEE ID: %02x%02x%02x\n", ieee0, ieee1, ieee2);
  uint8_t a0 = physicalAddress >> 12;
  uint8_t a1 = (physicalAddress >> 8) & 0xf;
  uint8_t a2 = (physicalAddress >> 4) & 0xf;
  uint8_t a3 = physicalAddress & 0xf;
  Serial.printf("CEC Physical Address: %u.%u.%u.%u\n", a0, a1, a2, a3);
}

void setup()
{
  digitalWrite(ONBOARD_LED, HIGH); 
  pinMode(CEC_GPIO, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial) delay(50);

  Wire.begin();
  Wire.requestFrom(EDID_ADDRESS, EDID_LENGTH, true);

  uint8_t edid[EDID_LENGTH];
  int totalRead = 0;
  while (totalRead < EDID_LENGTH)
  {
    int read = Wire.readBytes(edid + totalRead, EDID_LENGTH - totalRead);
    if (read == 0)
    {
      Serial.println("EDID timeout");
      delay(1000);
    }

    totalRead += read;
  }

  Serial.println("EDID Received");

  parseEdid(edid);

  BLEDevice::init("TvHdmiCec"); 

  while (!ddc.begin()) 
  {
    Serial.println("Unable to find DDC/CI. Trying again in 5 seconds.");
    delay(5000);
  }
  Serial.println("Found DDC/CI successfully"); 
  
  device.Initialize(CEC_PHYSICAL_ADDRESS, CEC_DEVICE_TYPE, true); // Promiscuous mode}

  server.on("/heap", HTTP_GET, handleHeap);
  server.on("/history", HTTP_GET, handleHistory);
  server.on("/standby", HTTP_GET, handleStandby);
  server.on("/transmit", HTTP_GET, handleTransmit);
  server.onNotFound(handle404);

  connectWiFi();

  server.begin();

  digitalWrite(ONBOARD_LED, LOW);
}

void loop()
{
  connectWiFi();
  // if (Serial.available())
  // {
  //   unsigned char c = Serial.read();
  //   unsigned char buffer[3];

  //   switch (c)
  //   {
  //   case '1':
  //     buffer[0] = 0x36;
  //     device.TransmitFrame(0x4f, buffer, 1);
  //     break;
  //   }
  // }
  device.Run();
}