#ifndef UTIL_H
#define UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cJSON.h"

void format_bytes(char* s, int n, const unsigned char* buffer, int count);
esp_err_t ping(const char* host_name, int count, int interval, int* pongs);
cJSON* get_json(const char* url);

void send_WOL(const char* mac, int repeat, int repeat_delay_ms);

#ifdef __cplusplus
}
#endif

#endif