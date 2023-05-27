#ifndef WII_H
#define WII_H

typedef enum 
{
    WII_POWER_STATE_UNKNOWN,
    WII_POWER_STATE_ERROR,
    WII_POWER_STATE_ON,
    WII_POWER_STATE_OFF
} wii_power_state_t;

typedef enum
{
    WII_POWER_STATUS_UNKNOWN,
    WII_POWER_STATUS_ERROR,
    WII_POWER_STATUS_NOT_TOGGLED,
    WII_POWER_STATUS_TOGGLED,
}
wii_power_status_t;

typedef enum
{
    WII_IDLE,
    WII_ABORT,
    WII_ERROR,
    WII_PAIRING_PENDING,
    WII_PAIRING,
    WII_POWER_ON,
    WII_POWER_OFF,
    WII_QUERY_POWER_STATE
} wii_state_t;

#ifdef __cplusplus
extern "C" 
{
#endif

esp_err_t wii_init();
wii_power_state_t wii_query_power_state();
wii_power_status_t wii_power_on();
wii_power_status_t wii_power_off();
//void wii_pair();

#ifdef __cplusplus
}
#endif


#endif