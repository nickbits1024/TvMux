#ifndef CEC_H
#define CEC_H
#include "CEC_Device.h"

#define CEC_GPIO              5
#define CEC_DEVICE_TYPE       CEC_Device::CDT_PLAYBACK_DEVICE
#define CEC_MAX_MSG_SIZE      16
#define CEC_MAX_HISTORY       64
#define EDID_ADDRESS          0x50
#define EDID_LENGTH           128
#define EDID_EXTENSION_LENGTH 128
#define EDID_EXTENSION_DATA_LENGTH  125
#define EDID_EXTENSION_FLAG   0x7e

class HomeTvCec : public CEC_Device
{
protected:
  virtual bool LineState();
  virtual void SetLineState(bool);
  virtual void OnReady(int logicalAddress);
  virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack);
  virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack);

public:
  HomeTvCec();
};

void format_bytes(std::stringstream& ss, unsigned char* buffer, int count);
void format_bytes(std::string& s, unsigned char* buffer, int count);

#endif