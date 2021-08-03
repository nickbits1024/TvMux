#ifndef CEC_H
#define CEC_H
#include "freertos/queue.h"
#include "CEC_Device.h"

#define CEC_GPIO_INPUT        5
#define CEC_GPIO_OUTPUT       17
#define CEC_DEVICE_TYPE       CEC_Device::CDT_PLAYBACK_DEVICE
#define CEC_MAX_MSG_SIZE      16
#define CEC_MAX_HISTORY       64
#define EDID_ADDRESS          0x50
#define EDID_LENGTH           128
#define EDID_EXTENSION_LENGTH 128
#define EDID_EXTENSION_DATA_LENGTH  125
#define EDID_EXTENSION_FLAG   0x7e

#define CEC_USER_CONTROL_PLAY       0x60
#define CEC_USER_CONTROL_PAUSE      0x61
#define CEC_USER_CONTROL_POWER_ON   0x6d
#define CEC_USER_CONTROL_POWER_OFF  0x6c

// CEC locical address handling
typedef enum {
  CEC_TV_ADDRESS = 0,
  CEC_RECORDING_DEVICE_1_ADDRESS,
  CEC_RECORDING_DEVICE_2_ADDRESS,
  CEC_TUNER_1_ADDRESS,
  CEC_PLAYBACK_DEVICE_1_ADDRESS,
  CEC_AUDIO_SYSTEM_ADDRESS,
  CEC_TUNER_2_ADDRESS,
  CEC_TUNER_3_ADDRESS,
  CEC_PLAYBACK_DEVICE_2_ADDRESS,
  CEC_RECORDING_DEVICE_3_ADDRESS,
  CEC_TUNER_4_ADDRESS,
  CEC_PLAYBACK_DEVICE_3_ADDRESS,
  CEC_RESERVED_1_ADDRESS,
  CEC_RESERVED_2_ADDRESS,
  CEC_FREE_USE_ADDRESS,
  CEC_BROADCAST_ADDRESS
} CEC_DEVICE_ADDRESS;

struct CEC_MESSAGE
{
  //int fromAddress;
  int targetAddress;
  int size;
  uint8_t data[CEC_MAX_MSG_SIZE];
};

class HomeTvCec : public CEC_Device
{
private:
  xQueueHandle queueHandle;
  CEC_MESSAGE* pendingMessage;

  xSemaphoreHandle request_sem;
  portMUX_TYPE response_mux;
  xSemaphoreHandle responded_sem;
  uint8_t* reply;
  int* reply_size;
  int reply_address;
  uint8_t reply_filter;

protected:
  virtual bool LineState();
  virtual void SetLineState(bool);
  virtual void OnReady(int logicalAddress);
  virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack);
  virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack);
  void TransmitFrame(int targetAddress, const uint8_t* buffer, int count);
  //void TransmitFrame(int fromAddress, int targetAddress, const unsigned char* buffer, int count);

public:
  HomeTvCec();

  void Run();

  bool Control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size);

  void StandBy();
  void TvScreenOn();
  void SetSystemAudioMode(bool on);
  //void SetActiveSource(uint16_t addr);
  void SystemAudioModeRequest(uint16_t addr);
  void UserControlPressed(int targetAddress, uint8_t userControl);
};

void format_bytes(std::stringstream& ss, unsigned char* buffer, int count);
void format_bytes(std::string& s, unsigned char* buffer, int count);

#endif