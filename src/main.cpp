#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <nvs_flash.h>
#include <ESPAsyncWebServer.h>
#include "cJSON.h"
#include <list>
#include <functional>
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
//portMUX_TYPE cecMux = portMUX_INITIALIZER_UNLOCKED;
//portMUX_TYPE historyMux = portMUX_INITIALIZER_UNLOCKED;
//std::list<std::string> history;
uint16_t cec_physical_address;
//volatile bool hdmi_unplugged;
//double last_hpd_time;
HomeTvCec device;
bool wii_pair_request;
nvs_handle config_nvs_handle;
xTaskHandle change_tv_state_task_handle;

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

bool call_with_retry(std::function<bool()> f, int wait_ms)
{
  int retry = 0;
  bool success;
  do
  {
    if (retry != 0)
    {
      printf("control retry #%d\n", retry);
      delay(wait_ms);
    }
    success = f();
  } while (retry++ < MAX_COMMAND_RETRY && !success);

  return success;
}

bool call_with_retry(std::function<void()> f, std::function<bool()> check, int check_wait_ms, int retry_wait_ms)
{
  int retry = 0;
  bool success;
  do
  {
    if (retry != 0)
    {
      printf("control retry #%d\n", retry);
      delay(retry_wait_ms);
    }
    f();
    delay(check_wait_ms);
    success = check();
  } while (retry++ < MAX_COMMAND_RETRY && !success);

  return success;
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

// void handle_history(AsyncWebServerRequest* request)
// {
//   auto response = new PrettyAsyncJsonResponse(true, 256);
//   auto doc = response->getRoot();

//   portENTER_CRITICAL(&historyMux);
//   for (std::list<std::string>::iterator it = history.begin(); it != history.end(); it++)
//   {
//     doc.add(it->c_str());
//   }
//   portEXIT_CRITICAL(&historyMux);

//   response->setLength();
//   request->send(response);
// }

void handle_standby(AsyncWebServerRequest* request)
{
  //portENTER_CRITICAL(&cecMux);
  device.StandBy();
  //portEXIT_CRITICAL(&cecMux);

  auto response = new PrettyAsyncJsonResponse(false, 16);
  auto doc = response->getRoot();

  doc["status"] = "ok";

  response->setLength();
  request->send(response);
}

void handle_tv_pause_get(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();

  device.UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, 0x46);

  doc["status"] = "ok";

  response->setLength();
  request->send(response);
}

void handle_tv_play_get(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();

  device.UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, 0x44);

  doc["status"] = "ok";

  response->setLength();
  request->send(response);
}

bool combine_devices_state(bool and_mode, bool tv = true, bool audio = true, bool atv = true)
{
  bool tv_on = and_mode;

  auto check_device = [and_mode, &tv_on](int target_address) {
    uint8_t reply[CEC_MAX_MSG_SIZE];
    auto get_power_state = [target_address, &reply]() {
      uint8_t cmd[] = { 0x8f };
      int reply_size = CEC_MAX_MSG_SIZE;
      return device.Control(target_address, cmd, sizeof(cmd), 0x90, reply, &reply_size) && reply_size == 3;
    };

    if (!tv_on || and_mode)
    {
      if (call_with_retry(get_power_state, 100))
      {
        //printf("reply %02x:%02x:%02x\n", reply[0], reply[1], reply[2]);
        if (and_mode)
        {
          tv_on &= reply[2] == 0;
        }
        else
        {
          tv_on |= reply[2] == 0;
        }
      }
      else
      {
        if (and_mode)
        {
          printf("control failed, assuming off\n");
          tv_on = false;
        }
        else
        {
          printf("control failed, keeping tv_on=%u\n", tv_on);
        }
      }
    }
  };

  if (tv)
  {
    printf("tv on?\n");
    check_device(CEC_TV_ADDRESS);
  }
  if (audio)
  {
    printf("avr on?\n");
    check_device(CEC_AUDIO_SYSTEM_ADDRESS);
  }
  if (atv)
  {
    printf("atv on?\n");
    check_device(CEC_PLAYBACK_DEVICE_1_ADDRESS);
  }

  return tv_on;
}

void handle_tv_get(AsyncWebServerRequest* request)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();

  doc["status"] = "ok";
  doc["state"] = combine_devices_state(false) ? "on" : "off";

  response->setLength();
  request->send(response);
}

void handle_wii_get(AsyncWebServerRequest* request)
{
  //printf("wii/get from %s\n", request->client()->remoteIP().toString().c_str());
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

void change_tv_state_task(void* p)
{
  bool desired_state = (bool)(intptr_t)p;
  void (*change_state)();

  if (desired_state)
  {
    change_state = [] { device.UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, CEC_USER_CONTROL_POWER_ON); };
  }
  else
  {
    change_state = [] { device.StandBy(); };
  }

  if (call_with_retry(change_state, [desired_state] { return combine_devices_state(desired_state) == desired_state; }, 10000, 1000))
  {
    printf("change_tv_state = %u succeeded\n", desired_state);
  }
  else
  {
    printf("change_tv_state = %u failed\n", desired_state);
  }

  change_tv_state_task_handle = NULL;
  vTaskDelete(NULL);
}

void handle_tv_post(AsyncWebServerRequest* request, JsonVariant& json)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();
  const JsonObject& jsonObj = json.as<JsonObject>();

  if (jsonObj.containsKey("state"))
  {
    auto desired_state = false;
    auto change_state = false;

    //void (*change_state)() = NULL;

    if (jsonObj["state"] == "on")
    {
      printf("turn tv on\n");
      change_state = true;
      desired_state = true;
      //change_state = [] { device.UserControlPressed(CEC_PLAYBACK_DEVICE_1_ADDRESS, CEC_USER_CONTROL_POWER_ON); };
    }
    else if (jsonObj["state"] == "off")
    {
      printf("turn tv off\n");
      //change_state = [] { device.StandBy(); };
      change_state = true;
      desired_state = false;
    }

    if (change_state)
    {
      if (change_tv_state_task_handle != NULL)
      {
        vTaskDelete(change_tv_state_task_handle);
        change_tv_state_task_handle = NULL;
      }
      xTaskCreate(change_tv_state_task, "change_tv_state", 8000, (void*)(intptr_t)desired_state, 1, &change_tv_state_task_handle);
    }
  }

  //doc["state"] = combine_devices_state(false) ? "on" : "off";
  doc["status"] = "pending";

  response->setLength();
  request->send(response);
}

void handle_wii_post(AsyncWebServerRequest* request, JsonVariant& json)
{
  auto response = new PrettyAsyncJsonResponse(false, 256);
  auto doc = response->getRoot();
  const JsonObject& jsonObj = json.as<JsonObject>();

  wii_power_status_t status = WII_POWER_STATUS_UNKNOWN;
  wii_power_status_t (*change_state)() = NULL;
  bool desired_av_state;
  wii_power_state_t desired_wii_state;

  if (jsonObj.containsKey("state"))
  {
    if (jsonObj["state"] == "on")
    {
      desired_wii_state = WII_POWER_STATE_ON;
      desired_av_state = true;
      change_state = [] {
        auto status = wii_power_on();
        //portENTER_CRITICAL(&cecMux);
        device.TvScreenOn();
        device.SystemAudioModeRequest(0x4200);
        delay(5000);
        //device.SetSystemAudioMode(true);
        //portEXIT_CRITICAL(&cecMux);
        return status;
      };
    }
    else if (jsonObj["state"] == "off")
    {
      desired_wii_state = WII_POWER_STATE_OFF;
      desired_av_state = false;
      change_state = [] {
        auto status = wii_power_off();
        //portENTER_CRITICAL(&cecMux);
        device.StandBy();
        //portEXIT_CRITICAL(&cecMux);
        return status;
      };
    }
  }

  if (change_state != NULL)
  {
    if (!call_with_retry(change_state, [desired_av_state, desired_wii_state] 
        { return wii_query_power_state() == desired_wii_state && 
                 combine_devices_state(desired_av_state, true, true, false) == desired_av_state; }, 5000, 1000))
    {
      printf("wii control failed\n");
    }
  }

  doc["power_toggled"] = status == WII_POWER_STATUS_TOGGLED;
  doc["status"] = "ok";

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

void handle_control(AsyncWebServerRequest* request)
{
  int target = request->pathArg(0).toInt();
  //int from = request->hasArg("from") ? request->arg("from").toInt() : -1;
  String cmd = request->arg("cmd");
  String reply_command = request->arg("reply");
  //Serial.println("send 1");
  uint8_t request_buffer[CEC_MAX_MSG_SIZE];
  uint8_t reply_buffer[CEC_MAX_MSG_SIZE];
  int len = (cmd.length() + 1) / 3;
  if (len == 0 ||
    len > sizeof(request_buffer) ||
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
    request_buffer[i] = (unsigned char)temp;
  }

  int reply_filter = -1;
  if (reply_command.length() == 2)
  {
    hex = reply_command.c_str();
    sscanf(hex, "%02x", &reply_filter);
    printf("reply_filter=%02x\n", reply_filter);
  }

  std::string reply_string;
  const char* reply_status = "ok";

  if (reply_filter != -1)
  {
    int reply_buffer_size = CEC_MAX_MSG_SIZE;
    if (device.Control(target, request_buffer, len, (uint8_t)reply_filter, reply_buffer, &reply_buffer_size))
    {
      format_bytes(reply_string, reply_buffer + 1, reply_buffer_size - 1);
    }
    else
    {
      reply_status = "error";
    }
  }
  else
  {
    if (!device.Control(target, request_buffer, len, 0, NULL, NULL))
    {
      reply_status = "error";
    }
  }

  auto response = new PrettyAsyncJsonResponse(false, 256);

  auto doc = response->getRoot();

  doc["target"] = target;
  doc["cmd"] = cmd;
  doc["reply_status"] = reply_status;
  if (reply_filter != -1)
  {
    doc["reply"] = reply_string.c_str();
  }

  response->setLength();
  request->send(response);
}

// void handle_control(AsyncWebServerRequest* request)
// {
//   if (!request->hasArg("target") || !request->hasArg("cmd"))
//   {
//     request->send(500);
//     return;
//   }

//   int target = request->arg("target").toInt();
//   String cmd = request->arg("cmd");

//   cmd.replace(":", "");

//   char buffer[256];
//   int len = cmd.length() / 2;
//   if (len > sizeof(buffer))
//   {
//     request->send(500);
//     return;
//   }
//   Serial.printf("cmd=%s\n", cmd.c_str());
//   for (int i = 0; i < len; i++)
//   {
//     String hex = cmd.substring(i * 2, i * 2 + 2);
//     //Serial.printf("hex=%s\n", hex.c_str());
//     unsigned int temp;
//     sscanf(hex.c_str(), "%02x", &temp);
//     buffer[i] = (char)temp;
//   }

//   String temp;

//   for (int i = 0; i < len; i++)
//   {
//     if (i != 0)
//     {
//       temp += ":";
//     }
//     char hex[3];
//     snprintf(hex, 3, "%02x", buffer[i]);
//     temp += hex;
//   }
//   Serial.printf("reconstructed: %s\n", temp.c_str());

//   //portENTER_CRITICAL(&cecMux);

//   device.Control(target, (unsigned char*)buffer, len, 0, NULL, NULL);

//   //portEXIT_CRITICAL(&cecMux);

//   auto response = new PrettyAsyncJsonResponse(false, 256);
//   auto doc = response->getRoot();

//   doc["target"] = target;
//   doc["cmd"] = temp;

//   response->setLength();
//   request->send(response);
// }

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

  // disableCore0WDT();
  // disableCore1WDT();
  // disableLoopWDT(); 

  Serial.begin(115200);
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
  xTaskCreate(cec_loop, "cec_loop", 10000, NULL, 1, NULL);
#endif

  wii_init();

  server.on("/heap", HTTP_GET, handle_heap);
  //server.on("/history", HTTP_GET, handle_history);
  server.on("/standby", HTTP_GET, handle_standby);
  server.on("/wii/pair", HTTP_GET, handle_wii_pair_get);
  server.on("/wii", HTTP_GET, handle_wii_get);
  server.on("/tv/play", HTTP_GET, handle_tv_play_get);
  server.on("/tv/pause", HTTP_GET, handle_tv_pause_get);
  server.on("/tv", HTTP_GET, handle_tv_get);
  server.on("^\\/cec\\/([0-9]+)\\/device$", HTTP_GET, handle_control);
  server.onNotFound(handle_404);
  server.addHandler(new AsyncCallbackJsonWebHandler("/wii", handle_wii_post));
  server.addHandler(new AsyncCallbackJsonWebHandler("/tv", handle_tv_post));

  connect_wiFi();

  server.begin();

  digitalWrite(ONBOARD_LED_GPIO, LOW);
}

void loop()
{
  static double hpd_time;

  connect_wiFi();

#ifdef HDMI_CEC
  bool hpd = digitalRead(HOTPLUG_GPIO) == HIGH;
  if (hpd)
  {
    hpd_time = uptimed();
  }

  if (!hpd && (uptimed() - hpd_time) > 5.0)
  {
    Serial.println("HDMI unplugged, rebooting in 1s...");
    delay(1000);
    ESP.restart();
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
    //portENTER_CRITICAL(&cecMux);
    device.Run();
    //portEXIT_CRITICAL(&cecMux);
  }
}