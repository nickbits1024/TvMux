#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/err.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "ping/ping_sock.h"
#include "esp_log.h"
#include "esp_check.h"
#include "util_int.h"
#include "util.h"

#define TAG "UTIL"

void format_bytes(char* s, int n, const unsigned char* buffer, int count)
{
    for (int i = 0, j = 0; i < count && j + 1< n; i++)
    {
        if (i != 0)
        {
            s[j++] = ':';
        }
        snprintf(&s[j], 3, "%02x", buffer[i]);
        j += 2;
    }
}


static void ping_success(esp_ping_handle_t hdl, void* args)
{
    ESP_LOGI(TAG, "ping_success");   

    int* pongs = (int*)args;

    (*pongs)++;
}

static void ping_timeout(esp_ping_handle_t hdl, void* args)
{
    ESP_LOGI(TAG, "ping_timeout");
}

static void ping_end(esp_ping_handle_t hdl, void* args)
{
    ESP_LOGI(TAG, "ping_end");
}

typedef struct
{
    SemaphoreHandle_t sem;
    ip_addr_t* addr;
}
ping_callback_data_t;

void ping_dns_found(const char* name, const ip_addr_t* addr, void* arg)
{
    ping_callback_data_t* data = (ping_callback_data_t*)arg;

    if (addr != NULL)
    {
        *data->addr = *addr;
        xSemaphoreGive(data->sem);
    }
}

esp_err_t ping(const char* host_name, int count, int interval, int* pongs)
{
    esp_err_t ret = ESP_OK;
    ip_addr_t addr;
    SemaphoreHandle_t sem = xSemaphoreCreateBinary();  

    bool success = false;

    ping_callback_data_t data = 
    {
        .sem = sem,
        .addr = &addr
    };

    err_t dns_ret = dns_gethostbyname(host_name, &addr, ping_dns_found, &data);
    if (dns_ret == ERR_INPROGRESS)
    {
        success = xSemaphoreTake(sem, PING_DNS_TIMEOUT) == pdTRUE;
    }
    else if (dns_ret != ERR_OK)
    {
        goto error;
    }
    
    if (!success)
    {
        goto error;
    }

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.target_addr = addr;
    ping_config.task_stack_size = 4000;
    ping_config.count = count;
    ping_config.interval_ms = interval;

    *pongs = 0;

    /* set callback functions */
    esp_ping_callbacks_t cbs = { };
    cbs.on_ping_success = ping_success;
    cbs.on_ping_timeout = ping_timeout;
    cbs.on_ping_end = ping_end;
    cbs.cb_args = pongs;

    esp_ping_handle_t ping_handle = NULL;

    ESP_GOTO_ON_ERROR(esp_ping_new_session(&ping_config, &cbs, &ping_handle), 
            error, TAG, "esp_ping_new_session (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(esp_ping_start(ping_handle), 
            error, TAG, "esp_ping_start (%s)", esp_err_to_name(err_rc_));

    vTaskDelay((ping_config.count * ping_config.interval_ms + 500) / portTICK_PERIOD_MS);

    esp_ping_delete_session(ping_handle);

    ESP_LOGI(TAG, "ping %08lx pong %d", addr.addr, *pongs);

    return ESP_OK;

error:
    vSemaphoreDelete(sem);
    if (ping_handle != NULL)
    {
        esp_ping_delete_session(ping_handle);

    }
    return ret;
}