#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <AsyncJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <vector>
#include "CEC_Device.h"
//#include "DDCVCP.h"

#define WIFI_SSID             "ac55.wifi.nickpalmer.net"
#define WIFI_PASS             "B16b00b5"
#define CEC_GPIO              5
#define CEC_DEVICE_TYPE       CEC_Device::CDT_PLAYBACK_DEVICE
//#define CEC_PHYSICAL_ADDRESS  0x4000
#define MAX_HISTORY           64
#define ONBOARD_LED           2
#define EDID_ADDRESS          0x50
#define EDID_LENGTH           128
#define EDID_EXTENSION_LENGTH 128
#define EDID_EXTENSION_DATA_LENGTH  125
#define EDID_EXTENSION_FLAG   0x7e

#define HOTPLUG_GPIO          23

AsyncWebServer server(80);
//DDCVCP ddc;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
std::vector<std::string> history;
uint16_t cec_physical_address;
//volatile bool hdmi_unplugged;

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

  //unsigned char buf[4] = { 0x84, CEC_PHYSICAL_ADDRESS >> 8, CEC_PHYSICAL_ADDRESS & 0xff, CEC_DEVICE_TYPE };
  unsigned char buf[4] = { 0x84, cec_physical_address >> 8, cec_physical_address & 0xff, CEC_DEVICE_TYPE };

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
  if (history.size() > MAX_HISTORY)
  {
    history.erase(history.begin());
  }
  portEXIT_CRITICAL(&mux);

  switch (buffer[1])
  {
  case 0x83:
    { // <Give Physical Address>
      //unsigned char buf[4] = { 0x84, CEC_PHYSICAL_ADDRESS >> 8, CEC_PHYSICAL_ADDRESS & 0xff, CEC_DEVICE_TYPE };
      unsigned char buf[4] = { 0x84, cec_physical_address >> 8, cec_physical_address & 0xff, CEC_DEVICE_TYPE };
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

void connect_wiFi()
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

void handle_404(AsyncWebServerRequest* request)
{
  request->send(404);
}

void handle_heap(AsyncWebServerRequest* request)
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

void handle_history(AsyncWebServerRequest* request)
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

void handle_standby(AsyncWebServerRequest* request)
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

void handle_transmit(AsyncWebServerRequest* request)
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
    //Serial.printf("hex=%s\n", hex.c_str());
    unsigned int temp;
    sscanf(hex.c_str(), "%02x", &temp);
    buffer[i] = (char)temp;
  }

  String temp;

  for (int i = 0; i < len; i++)
  {
    if (i != 0)
    {
      temp += ":";
    }
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

  response->setLength();
  request->send(response);
}

bool parse_edid(unsigned char* edid)
{
  uint32_t header0 = edid[0] << 24 | edid[1] << 16 | edid[2] << 8 | edid[3];
  uint32_t header1 = edid[4] << 24 | edid[5] << 16 | edid[6] << 8 | edid[7];

  uint32_t sum = 0;
  for (int i = 0; i < EDID_LENGTH; i++)
  {
    sum += (uint32_t)edid[i];
  }

  Serial.printf("EDID checksum: %08x\n", sum);
  Serial.printf("EDID header: %08x%08x\n", header0, header1);

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
  uint8_t extension_flag = edid[EDID_EXTENSION_FLAG];

  Serial.printf("EDID version: %u revision: %u\n", version, revision);
  Serial.printf("EDID manufacturer: %s\n", manufacturer);
  Serial.printf("EDID extension_flag: %d\n", extension_flag);

  return true;
  /*

    uint8_t ieee0 = edid[0x95];
    uint8_t ieee1 = edid[0x96];
    uint8_t ieee2 = edid[0x97];
    uint16_t physicalAddress = edid[0x98] << 8 | edid[0x99];

    Serial.printf("IEEE ID: %02x%02x%02x\n", ieee0, ieee1, ieee2);
    uint8_t a0 = physicalAddress >> 12;
    uint8_t a1 = (physicalAddress >> 8) & 0xf;
    uint8_t a2 = (physicalAddress >> 4) & 0xf;
    uint8_t a3 = physicalAddress & 0xf;
    Serial.printf("CEC Physical Address: %u.%u.%u.%u\n", a0, a1, a2, a3);

    return true;*/
}

const char*
audio_format(unsigned char x)
{
  switch (x)
  {
  case 0: return "RESERVED";
  case 1: return "Linear PCM";
  case 2: return "AC-3";
  case 3: return "MPEG 1 (Layers 1 & 2)";
  case 4: return "MPEG 1 Layer 3 (MP3)";
  case 5: return "MPEG2 (multichannel)";
  case 6: return "AAC";
  case 7: return "DTS";
  case 8: return "ATRAC";
  case 9: return "One Bit Audio";
  case 10: return "Dolby Digital+";
  case 11: return "DTS-HD";
  case 12: return "MAT (MLP)";
  case 13: return "DST";
  case 14: return "WMA Pro";
  case 15: return "RESERVED";
  }
  return "BROKEN"; /* can't happen */
}

void cea_audio_block(unsigned char* x)
{
  int i, format;
  int length = x[0] & 0x1f;

  if (length % 3)
  {
    Serial.printf("Broken CEA audio block length %d\n", length);
    /* XXX non-conformant */
    return;
  }

  for (i = 1; i < length; i += 3)
  {
    format = (x[i] & 0x78) >> 3;
    Serial.printf("    %s, max channels %d\n",
      audio_format(format), x[i] & 0x07);
    Serial.printf(
      "    Supported sample rates (kHz):%s%s%s%s%s%s%s\n",
      (x[i + 1] & 0x40) ? " 192" : "",
      (x[i + 1] & 0x20) ? " 176.4" : "",
      (x[i + 1] & 0x10) ? " 96" : "",
      (x[i + 1] & 0x08) ? " 88.2" : "",
      (x[i + 1] & 0x04) ? " 48" : "",
      (x[i + 1] & 0x02) ? " 44.1" : "",
      (x[i + 1] & 0x01) ? " 32" : "");
    if (format == 1)
    {
      Serial.printf(
        "    Supported sample sizes (bits):%s%s%s\n",
        (x[2] & 0x04) ? " 24" : "",
        (x[2] & 0x02) ? " 20" : "",
        (x[2] & 0x01) ? " 16" : "");
    }
    else if (format <= 8)
    {
      Serial.printf(
        "    Maximum bit rate: %d kHz\n", x[2] * 8);
    }
  }
}


bool parse_edid_extension(uint8_t* edid2, uint8_t* ext)
{
  uint32_t sum = 0;
  for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
  {
    sum += (uint32_t)ext[i];
  }

  if (sum % 256 != 0)
  {
    return false;
  }

  Serial.printf("EDID ext checksum: %08x\n", sum);

  uint8_t tag = ext[0];
  uint8_t revision = ext[1];
  uint8_t dtd_offset = ext[2];
  uint8_t offset = 4;

  Serial.printf("EDID ext tag: %u\n", tag);
  Serial.printf("EDID ext revision: %u\n", revision);
  Serial.printf("EDID ext dtd_offset: %u\n", dtd_offset);

  if (tag != 2)
  {
    return false;
  }

  for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
  {
    Serial.printf("0x%02x, ", ext[i]);
  }
  Serial.println();

  uint8_t index = offset;

  while (index < dtd_offset)
  {
    uint8_t* p = ext + index;
    uint8_t tag = p[0] >> 5;
    uint8_t length = p[0] & 0x1f;

    Serial.printf("EDID ext tag: %d length: %d\n", tag, length);

    switch (tag)
    {
    case 1:
      cea_audio_block(p);
      break;
    case 3:
      {
        uint8_t ieee[3];
        ieee[0] = p[3];
        ieee[1] = p[2];
        ieee[2] = p[1];
        Serial.printf("EDID IEEE %02x %02x %02x\n", ieee[0], ieee[1], ieee[2]);
        if (ieee[0] == 0x00 && ieee[1] == 0x0c && ieee[2] == 0x03)
        {
          cec_physical_address = (uint16_t)p[4] << 8 | p[5];
          uint8_t a0 = cec_physical_address >> 12;
          uint8_t a1 = (cec_physical_address >> 8) & 0xf;
          uint8_t a2 = (cec_physical_address >> 4) & 0xf;
          uint8_t a3 = cec_physical_address & 0xf;

          Serial.printf("CEC Physical Address: %u.%u.%u.%u\n", a0, a1, a2, a3);

        }
        break;
      }
    }

    index += 1 + length;
  }

  return true;
}

byte readI2CByte(byte data_addr)
{
  byte data = 0xff;
  Wire.beginTransmission(EDID_ADDRESS);
  Wire.write(data_addr);
  Wire.endTransmission();
  Wire.requestFrom(EDID_ADDRESS, 1); //retrieve 1 returned byte
  delay(1);
  if (Wire.available())
  {
    data = Wire.read();
  }
  return data;
}

// void IRAM_ATTR on_hdmi_unplugged()
// {
//   hdmi_unplugged = true;
// }

void setup()
{
  pinMode(HOTPLUG_GPIO, INPUT_PULLDOWN);
  pinMode(CEC_GPIO, INPUT_PULLUP);
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(50);

  Serial.println("Waiting for hotplug signal...");
  while (digitalRead(HOTPLUG_GPIO) == LOW) delay(50);

  Serial.println("Hotplug signal detected!");

  //attachInterrupt(digitalPinToInterrupt(HOTPLUG_GPIO), on_hdmi_unplugged, FALLING);


  Wire.begin();
  uint8_t edid[EDID_LENGTH];
  uint8_t edid_extension[EDID_EXTENSION_LENGTH];
  do
  {
    // Wire.requestFrom(EDID_ADDRESS, EDID_LENGTH, true);
    // delay(50);
    // int totalRead = 0;
    // while (totalRead < EDID_LENGTH)
    // {
    //   int read = Wire.readBytes(edid + totalRead, EDID_LENGTH - totalRead);
    //   if (read == 0)
    //   {
    //     Serial.println("EDID timeout");
    //     delay(1000);
    //   }

    //   totalRead += read;
    // }
    for (int i = 0; i < EDID_LENGTH; i++)
    {
      edid[i] = readI2CByte(i);
    }

    Serial.println("EDID Received");

  } while (!parse_edid(edid));

  if (edid[EDID_EXTENSION_FLAG])
  {
    do
    {
      for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
      {
        edid_extension[i] = readI2CByte(EDID_LENGTH + i);
      }
      Serial.println("EDID EXT Received");
    } while (!parse_edid_extension(edid, edid_extension));
  }

  BLEDevice::init("TvHdmiCec");

  // while (!ddc.begin()) 
  // {
  //   Serial.println("Unable to find DDC/CI. Trying again in 5 seconds.");
  //   delay(5000);
  // }
  // Serial.println("Found DDC/CI successfully"); 

  device.Initialize(cec_physical_address, CEC_DEVICE_TYPE, true); // Promiscuous mode}

  server.on("/heap", HTTP_GET, handle_heap);
  server.on("/history", HTTP_GET, handle_history);
  server.on("/standby", HTTP_GET, handle_standby);
  server.on("/transmit", HTTP_GET, handle_transmit);
  server.onNotFound(handle_404);

  connect_wiFi();

  server.begin();

  digitalWrite(ONBOARD_LED, LOW);
}

void loop()
{
  connect_wiFi();

  device.Run();

  if (digitalRead(HOTPLUG_GPIO) == LOW)
  {
    unsigned long start = millis();
    //Serial.println("HPD down");
    //unsigned long end = start;
    while (digitalRead(HOTPLUG_GPIO) == LOW && (millis() - start) < 2000)
    {
    }
    unsigned long t = millis() - start;
    if (t >= 2000)
    {
      Serial.println("HDMI unplugged, rebooting in 5s...");
      delay(5000);
      ESP.restart();
    }
  }
}