#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <nvs_flash.h>
#include <ESPAsyncWebServer.h>
#include "cJSON.h"
#include <list>
#include "hometv.h"
#include "cec.h"
#include "wii.h"

#define HDMI_CEC

#define WIFI_SSID             "wifi.nickpalmer.net"
#define WIFI_PASS             "B16b00b5"

#define HOTPLUG_GPIO          19
//#define HOTPLUG_ANALOG_GPIO   36
//#define HOTPLUG_LOW_VOLTAGE   0.4

AsyncWebServer server(80);
portMUX_TYPE cecMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE historyMux = portMUX_INITIALIZER_UNLOCKED;
std::list<std::string> history;
uint16_t cec_physical_address;
//volatile bool hdmi_unplugged;
xSemaphoreHandle request_sem;
xSemaphoreHandle response_sem;
xSemaphoreHandle responded_sem;
unsigned char reply[CEC_MAX_MSG_SIZE];
int reply_length;
unsigned char reply_filter;
//double last_hpd_time;
HomeTvCec device;
bool wii_pair_request;
nvs_handle config_nvs_handle;

void cec_loop(void* param);

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
    Serial.print("Connecting to WiFi...");
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++)
    {
      delay(2000);
      Serial.print(".");
    }

  } while (WiFi.status() != WL_CONNECTED);
  Serial.println();
  Serial.println("Connected to WiFi");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
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

  portENTER_CRITICAL(&historyMux);
  for (std::list<std::string>::iterator it = history.begin(); it != history.end(); it++)
  {
    doc.add(it->c_str());
  }
  portEXIT_CRITICAL(&historyMux);

  response->setLength();
  request->send(response);
}

void handle_standby(AsyncWebServerRequest* request)
{
  portENTER_CRITICAL(&cecMux);
  device.StandBy();
  portEXIT_CRITICAL(&cecMux);

  auto response = new PrettyAsyncJsonResponse(false, 16);
  auto doc = response->getRoot();

  doc["status"] = "ok";

  response->setLength();
  request->send(response);
}

void handle_tv_get(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();

  doc["status"] = "ok";

  response->setLength();
  request->send(response);
}

void handle_wii_get(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();

  wii_power_state_t state = wii_query_power_state();
  doc["state"] = state == WII_POWER_STATE_ON ? "on" : state == WII_POWER_STATE_OFF ? "off" : "error";

  response->setLength();
  request->send(response);
}

void handle_wii_pair_get(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();

  wii_pair();
  doc["status"] = "started";

  response->setLength();
  request->send(response);
}

void handle_tv_post(AsyncWebServerRequest* request, JsonVariant& json)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();
  const JsonObject& jsonObj = json.as<JsonObject>();

  if (jsonObj.containsKey("state"))
  {
    if (jsonObj["state"] == "on")
    {
      portENTER_CRITICAL(&cecMux);
      device.TvScreenOn();
      device.SystemAudioModeRequest(0x5200);
      device.SetSystemAudioMode(true);
      portEXIT_CRITICAL(&cecMux);
    }
    else if (jsonObj["state"] == "off")
    {
      portENTER_CRITICAL(&cecMux);
      device.StandBy();
      portEXIT_CRITICAL(&cecMux);
    }
  }
  doc["status"] = "ok";

  response->setLength();
  request->send(response);
}

void handle_wii_post(AsyncWebServerRequest* request, JsonVariant& json)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();
  const JsonObject& jsonObj = json.as<JsonObject>();

  wii_power_status_t status = WII_POWER_STATUS_UNKNOWN;

  int toggle_delay = 0;
  if (jsonObj.containsKey("state"))
  {
    if (jsonObj["state"] == "on")
    {
      status = wii_power_on();
      portENTER_CRITICAL(&cecMux);
      device.TvScreenOn();
      device.SystemAudioModeRequest(0x4200);
      device.SetSystemAudioMode(true);
      portEXIT_CRITICAL(&cecMux);
      toggle_delay = 5000;
    }
    else if (jsonObj["state"] == "off")
    {
      status = wii_power_off();
      portENTER_CRITICAL(&cecMux);
      device.StandBy();
      portEXIT_CRITICAL(&cecMux);
      toggle_delay = 0;
    }
  }
  doc["power_toggled"] = status == WII_POWER_STATUS_TOGGLED;
  doc["status"] = "ok";

  if (status == WII_POWER_STATUS_TOGGLED)
  {
    delay(toggle_delay);
  }

  if (status == WII_POWER_STATUS_ERROR)
  {
    doc["state"] = "error";
  }
  else
  {
    wii_power_state_t state = wii_query_power_state();
    doc["state"] = state == WII_POWER_STATE_ON ? "on" : state == WII_POWER_STATE_OFF ? "off" : "error";
  }

  response->setLength();
  request->send(response);  
}

void handle_send(AsyncWebServerRequest* request)
{
  int target = request->pathArg(0).toInt();
  String cmd = request->arg("cmd");
  String reply_command = request->arg("reply");
  //Serial.println("send 1");
  char buffer[CEC_MAX_MSG_SIZE];
  int len = (cmd.length() + 1) / 3;
  if (len == 0 ||
    len > sizeof(buffer) ||
    (reply_command.length() != 0 && reply_command.length() != 2))
  {
    request->send(500);
    return;
  }
  Serial.printf("cmd=%s reply_command=%s\n", cmd.c_str(), reply_command.c_str());
  const char* hex = cmd.c_str();
  unsigned int temp;
  for (int i = 0; i < len; i++)
  {
    sscanf(hex + i * 3, "%02x", &temp);
    buffer[i] = (unsigned char)temp;
  }
  // String temp;

  // for (int i = 0; i < len; i++)
  // {
  //   if (i != 0)
  //   {
  //     temp += ":";
  //   }
  //   char hex[3];
  //   snprintf(hex, 3, "%02x", buffer[i]);
  //   temp += hex;
  // }
  //Serial.printf("send %u bytes, %s\n", len, temp.c_str());

  int reply_command_value = -1;
  if (reply_command.length() == 2)
  {
    hex = reply_command.c_str();
    sscanf(hex, "%02x", &reply_command_value);
  }
  //Serial.println("enter data");
  std::string reply_string;
  const char* reply_status = "ok";

  if (xSemaphoreTake(request_sem, 5000 / portTICK_PERIOD_MS))
  {
    //portENTER_CRITICAL(&dataMux);    
    if (reply_command.length() == 2)
    {
      hex = reply_command.c_str();
      unsigned int temp;
      sscanf(hex, "%02x", &temp);
      reply_length = CEC_MAX_MSG_SIZE;
      reply_filter = (unsigned char)reply_command_value;
    }
    else
    {
      reply_length = 0;
    }
    xSemaphoreTake(responded_sem, portMAX_DELAY);
    xSemaphoreGive(response_sem);
    //portEXIT_CRITICAL(&dataMux);
    //Serial.println("leave data, enter cec");
    portENTER_CRITICAL(&cecMux);
    device.TransmitFrame(target, (unsigned char*)buffer, len);
    portEXIT_CRITICAL(&cecMux);
    //Serial.println("leave data, leave cec");

    if (reply_command_value != -1)
    {
      Serial.printf("Waiting for reply %02x\n", reply_command_value);
      if (xSemaphoreTake(responded_sem, 5000 / portTICK_PERIOD_MS))
      {
        //portENTER_CRITICAL(&dataMux);
        if (reply_length > 1)
        {
          format_bytes(reply_string, reply + 1, reply_length - 1);
        }
        //portEXIT_CRITICAL(&dataMux);
      }
      else
      {
        reply_status = "error";
      }
    }
    xSemaphoreTake(response_sem, portMAX_DELAY);
    xSemaphoreGive(responded_sem);
    xSemaphoreGive(request_sem);
  }

  auto response = new PrettyAsyncJsonResponse(false, 256);

  auto doc = response->getRoot();

  doc["target"] = target;
  doc["cmd"] = cmd;
  if (reply_command_value != -1)
  {
    doc["reply_status"] = reply_status;
    doc["reply"] = reply_string.c_str();
  }

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

  cmd.replace(":", "");

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

  portENTER_CRITICAL(&cecMux);

  device.TransmitFrame(target, (unsigned char*)buffer, len);

  portEXIT_CRITICAL(&cecMux);

  auto response = new PrettyAsyncJsonResponse(false, 256);
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

  //Serial.printf("EDID checksum: %08x\n", sum);
  //Serial.printf("EDID header: %08x%08x\n", header0, header1);

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

  Serial.printf("EDID version: %u.%u\n", version, revision);
  Serial.printf("EDID manufacturer: %s\n", manufacturer);
  //Serial.printf("EDID extension_flag: %d\n", extension_flag);

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

  //Serial.printf("EDID ext checksum: %08x\n", sum);

  uint8_t tag = ext[0];
  //uint8_t revision = ext[1];
  uint8_t dtd_offset = ext[2];
  uint8_t offset = 4;

  // Serial.printf("EDID ext tag: %u\n", tag);
  // Serial.printf("EDID ext revision: %u\n", revision);
  // Serial.printf("EDID ext dtd_offset: %u\n", dtd_offset);

  if (tag != 2)
  {
    return false;
  }

  // for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
  // {
  //   Serial.printf("0x%02x, ", ext[i]);
  // }
  // Serial.println();

  uint8_t index = offset;

  while (index < dtd_offset)
  {
    uint8_t* p = ext + index;
    uint8_t tag = p[0] >> 5;
    uint8_t length = p[0] & 0x1f;

    //Serial.printf("EDID ext tag: %d length: %d\n", tag, length);

    switch (tag)
    {
    case 3:
      {
        uint8_t ieee[3];
        ieee[0] = p[3];
        ieee[1] = p[2];
        ieee[2] = p[1];
        //Serial.printf("EDID IEEE %02x %02x %02x\n", ieee[0], ieee[1], ieee[2]);
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
//   //hdmi_unplugged = true;
//   ESP.restart();
// }

// double get_hotplug_voltage()
// {
//   return analogRead(HOTPLUG_ANALOG_GPIO) * 5.0 / 4095.0;
// }

double uptimed()
{
  return esp_timer_get_time() / 1000000.0;
}

void IRAM_ATTR wii_pair_irq_handler()
{
  wii_pair_request = true;
}

void setup()
{
  digitalWrite(ONBOARD_LED_GPIO, HIGH);
  pinMode(WII_PAIR_GPIO, INPUT_PULLUP);
  pinMode(HOTPLUG_GPIO, INPUT_PULLUP);
  pinMode(CEC_GPIO_INPUT, INPUT_PULLUP);
  pinMode(CEC_GPIO_OUTPUT, OUTPUT_OPEN_DRAIN);
  digitalWrite(CEC_GPIO_OUTPUT, HIGH);
  pinMode(ONBOARD_LED_GPIO, OUTPUT);
  delay(50);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }

  err = nvs_open("default", NVS_READWRITE, &config_nvs_handle);
  ESP_ERROR_CHECK(err);

  request_sem = xSemaphoreCreateBinary();
  response_sem = xSemaphoreCreateBinary();
  responded_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(request_sem);
  xSemaphoreGive(responded_sem);

// disableCore0WDT();
// disableCore1WDT();
// disableLoopWDT(); 

  Serial.begin(921600);
  while (!Serial) delay(50);

  attachInterrupt(digitalPinToInterrupt(WII_PAIR_GPIO), wii_pair_irq_handler, FALLING);


#ifdef HDMI_CEC
  Serial.println("Waiting for hotplug signal...");
  while (digitalRead(HOTPLUG_GPIO) == LOW) delay(50);

  //double hpv = 0;
  //while ((hpv = get_hotplug_voltage()) < HOTPLUG_LOW_VOLTAGE) delay(50);
  //delay(500);

  Serial.printf("Hotplug signal detected!\n");
  //last_hpd_time = uptimed();

  //delay(10000);
  //attachInterrupt(digitalPinToInterrupt(HOTPLUG_GPIO), on_hdmi_unplugged, FALLING);

  Wire.begin();
  uint8_t edid[EDID_LENGTH];
  uint8_t edid_extension[EDID_EXTENSION_LENGTH];
  do
  {
    for (int i = 0; i < EDID_LENGTH; i++)
    {
      edid[i] = readI2CByte(i);
    }

    Serial.println("Received EDID");

  } while (!parse_edid(edid));

  if (edid[EDID_EXTENSION_FLAG])
  {
    do
    {
      for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
      {
        edid_extension[i] = readI2CByte(EDID_LENGTH + i);
      }

      Serial.println("Received EDID extension");

    } while (!parse_edid_extension(edid, edid_extension));
  }

  device.Initialize(cec_physical_address, CEC_DEVICE_TYPE, true); // Promiscuous mode}
  xTaskCreate(cec_loop, "cec_loop", 10000, NULL, 2, NULL);
#endif

  wii_init();

  server.on("/heap", HTTP_GET, handle_heap);
  server.on("/history", HTTP_GET, handle_history);
  server.on("/standby", HTTP_GET, handle_standby);
  server.on("/wii/pair", HTTP_GET, handle_wii_pair_get);
  server.on("/wii", HTTP_GET, handle_wii_get);
  server.on("/tv", HTTP_GET, handle_tv_get);
  //server.on("/transmit", HTTP_GET, handle_transmit);
  server.on("^\\/device\\/([0-9]+)\\/send$", HTTP_GET, handle_send);
  server.onNotFound(handle_404);
  server.addHandler(new AsyncCallbackJsonWebHandler("/wii", handle_wii_post));
  server.addHandler(new AsyncCallbackJsonWebHandler("/tv", handle_tv_post));

  connect_wiFi();

  server.begin();

  digitalWrite(ONBOARD_LED_GPIO, LOW);
}

void loop()
{
  connect_wiFi();

#ifdef HDMI_CEC
  //double now = uptimed();
  bool hpd = digitalRead(HOTPLUG_GPIO) == HIGH;

  //portENTER_CRITICAL(&dataMux);
  //device.Run();
  // if (hpd)
  // {
  //   last_hpd_time = now;
  // }
  //hpd_time = last_hpd_time;
  //portEXIT_CRITICAL(&dataMux);
  //Serial.printf("hpd: %d\n", hpd);
  if (!hpd)
  {
    // Serial.println("HPD down");
    // while (digitalRead(HOTPLUG_GPIO) == LOW)
    // {
    //   delay(50);
    //   if (uptimed() - last_hpd_time > 2.5)
    //   {
    Serial.println("HDMI unplugged, rebooting in 1s...");
    delay(1000);
    ESP.restart();
    //   }
    // }
    // Serial.printf("HPD down for %.5fs\n", uptimed() - last_hpd_time);

    // unsigned long start = millis();

    //delay(100);
    // //unsigned long end = start;
    // while (digitalRead(HOTP.LUG_GPIO) == LOW && (millis() - start) < 5000)
    // {
    // }
    // unsigned long t = millis() - start;
    // if (t >= 5000)
    // {

    //double v = analogRead(HOTPLUG_ANALOG_GPIO) * 5.0 / 4095.0;
    // Serial.printf("HDMI unplugged, rebooting in 5s (h=%d, v=%.2fV)...\n", hph, hpv);
    // delay(5000);
    // ESP.restart();
    //}
  }
#endif
  if (wii_pair_request)
  {
    wii_pair_request = false;
    wii_pair();
  }

  delay(500);
}

void cec_loop(void* param)
{
  while (1)
  {
    bool activity = digitalRead(CEC_GPIO_INPUT) == LOW;
    double now = uptimed();
    portENTER_CRITICAL(&cecMux);
    do 
    {      
      device.Run();
      
      bool more_activity = digitalRead(CEC_GPIO_INPUT) == LOW;
      if (more_activity)
      {
        now = uptimed();
      }
    }
    while (activity && uptimed() - now < 1.0);
    portEXIT_CRITICAL(&cecMux);
  }
}