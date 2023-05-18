#ifndef UTIL_H
#define UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

void format_bytes(char* s, int n, const unsigned char* buffer, int count);
esp_err_t ping(const char* host_name, int count, int interval, int* pongs);

#ifdef __cplusplus
}
#endif

#endif