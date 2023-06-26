#ifndef WII_IO_H
#define WII_IO_H

typedef struct
{
    const char* request;
    uint16_t request_size;
    const char* response;
    uint16_t response_size;
} 
wii_request_response_t;

extern wii_request_response_t wii_sdp_request_responses[];
extern int wii_sdp_request_responses_size;

extern wii_request_response_t wii_hid_request_responses[];
extern int wii_hid_request_responses_size;

#endif