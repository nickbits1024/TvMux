#ifndef CEC_H
#define CEC_H

#define CEC_MAX_MSG_SIZE      16
#define CEC_MAX_ADDRESS       0x0f

#define CEC_TV_HDMI_INPUT           (1)
#define CEC_ATV_HDMI_INPUT          (1)
#define CEC_WII_HDMI_INPUT          (2)
#define CEC_STEAM_HDMI_INPUT        (4)


enum CEC_POWER_STATE
{
    CEC_POWER_ON = 0x00,
    CEC_POWER_OFF = 0x01,
    CEC_POWER_TRANS_ON = 0x02,
    CEC_POWER_TRANS_OFF = 0x03,
    CEC_POWER_UNKNOWN = 0xff
};

// CEC locical address handling
typedef enum
{
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
    uint8_t targetAddress;
    uint8_t size;
    uint8_t data[CEC_MAX_MSG_SIZE];
};

struct CEC_LOG_ENTRY
{
    uint8_t direction;
    uint8_t size;
    bool ack;
    uint8_t data[CEC_MAX_MSG_SIZE];
};

#ifdef __cplusplus

#include <list>
#include "esp_http_server.h"
#include "CEC_Device.h"

typedef std::list<CEC_LOG_ENTRY> log_entry_list;

class HomeTvCec : public CEC_Device
{
private:
    QueueHandle_t queue_handle;
    CEC_MESSAGE* pending_message;

    SemaphoreHandle_t request_sem;
    portMUX_TYPE response_mux;
    SemaphoreHandle_t responded_sem;
    portMUX_TYPE state_mux;
    uint8_t* reply;
    int* reply_size;
    int reply_address;
    uint8_t reply_filter;
    log_entry_list log_entries;
    uint16_t active_source;
    uint8_t power_states[CEC_MAX_ADDRESS + 1];

protected:
    virtual bool LineState();
    virtual void SetLineState(bool);
    virtual void OnReady(int logicalAddress);
    virtual void OnReceiveComplete(unsigned char* buffer, int count, bool ack);
    virtual void OnTransmitComplete(unsigned char* buffer, int count, bool ack);
    void TransmitFrame(int targetAddress, const uint8_t* buffer, int count);
    //void TransmitFrame(int fromAddress, int targetAddress, const unsigned char* buffer, int count);
    void PrintIO(char direction, unsigned char* buffer, int size, bool ack);

public:
    HomeTvCec();

    bool Run();

    bool Control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size);

    void StandBy();
    void TvScreenOn();
    //void SetSystemAudioMode(bool on);
    void SetActiveSource(uint16_t addr);
    bool SystemAudioModeRequest(uint16_t addr);
    void UserControlPressed(int targetAddress, uint8_t userControl);
    void ClearPending();
    void WriteLog(httpd_req_t* request);
    void ClearLog();
    CEC_POWER_STATE GetPowerState(uint8_t addr);
    CEC_POWER_STATE LoadPowerState(uint8_t addr);
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t cec_init();
esp_err_t cec_queue_clear();
esp_err_t cec_standby();
esp_err_t cec_pause();
esp_err_t cec_play();
esp_err_t cec_tv_power(bool power_on);
esp_err_t cec_wii_power(bool power_on);
esp_err_t cec_log_write(httpd_req_t* request);
esp_err_t cec_log_clear();
esp_err_t cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size);
esp_err_t cec_sam_request(uint16_t addr);
esp_err_t cec_as_set(uint16_t addr);
esp_err_t cec_tv_on();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
esp_err_t cec_combine_devices_state(bool* state, bool and_mode, bool tv = true, bool audio = true, bool atv = true);
#else
esp_err_t cec_combine_devices_state(bool* state, bool and_mode, bool tv, bool audio, bool atv);
#endif



#endif