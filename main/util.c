#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/err.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "lwip/sockets.h"
#include "ping/ping_sock.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_netif.h"
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
    //ESP_LOGI(TAG, "ping_success");   

    int* pongs = (int*)args;

    (*pongs)++;
}

static void ping_timeout(esp_ping_handle_t hdl, void* args)
{
    //ESP_LOGI(TAG, "ping_timeout");
}

static void ping_end(esp_ping_handle_t hdl, void* args)
{
    //ESP_LOGI(TAG, "ping_end");
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
        goto cleanup;
    }
    
    if (!success)
    {
        goto cleanup;
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
            cleanup, TAG, "esp_ping_new_session (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(esp_ping_start(ping_handle), 
            cleanup, TAG, "esp_ping_start (%s)", esp_err_to_name(err_rc_));

    vTaskDelay((ping_config.count * ping_config.interval_ms + 500) / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "ping " IPSTR " tx %d rx %d", IP2STR(&addr), count, *pongs);

cleanup:
    vSemaphoreDelete(sem);
    if (ping_handle != NULL)
    {
        esp_ping_stop(ping_handle);
        esp_ping_delete_session(ping_handle);
    }
    return ret;
}

typedef struct http_chunk_t http_chunk_t;

struct http_chunk_t
{
    uint8_t* buffer;
    size_t size;
    http_chunk_t* next_chunk;
};

typedef struct 
{
    esp_err_t result;
    size_t response_size;
    http_chunk_t* first_chunk;
    http_chunk_t* last_chunk;
} http_context_t;

static esp_err_t http_event_handler(esp_http_client_event_t* evt)
{
    http_context_t* ctx = evt->user_data;

    switch(evt->event_id) 
    {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            //ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (ctx != NULL && ctx->response_size != -1)
            {
                uint8_t* chunk_buffer = malloc(evt->data_len);
                if (chunk_buffer != NULL)
                {
                    http_chunk_t* chunk = calloc(sizeof(http_chunk_t), 1);
                    if (chunk != NULL)
                    {
                        memcpy(chunk_buffer, evt->data, evt->data_len);
                        //ESP_LOG_BUFFER_HEX(TAG, evt->data, evt->data_len);
                        chunk->buffer = chunk_buffer;
                        chunk->size = evt->data_len;
                        if (ctx->first_chunk == NULL)
                        {
                            //ESP_LOGI(TAG, "first chunk %p buffer %p size %d next %p", chunk, chunk->buffer, chunk->size, chunk->next_chunk);
                            ctx->first_chunk = chunk;
                        }
                        else
                        {
                            //ESP_LOGI(TAG, "chunk %p buffer %p size %d next %p", chunk, chunk->buffer, chunk->size, chunk->next_chunk);
                        }
                        if (ctx->last_chunk != NULL)
                        {
                            ctx->last_chunk->next_chunk = chunk;
                        }
                        ctx->last_chunk = chunk;
                        ctx->response_size += evt->data_len;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "could not allocate chunk");
                        ctx->result = ESP_ERR_NO_MEM;
                        free(chunk_buffer);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "could not allocate chunk buffer size %u", evt->data_len);
                    ctx->result = ESP_ERR_NO_MEM;
                }
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

void http_cleanup(http_context_t* ctx)
{
    if (ctx && ctx->first_chunk != NULL)
    {
        http_chunk_t* chunk = ctx->first_chunk;
        while (chunk != NULL)
        {
            http_chunk_t* next_chunk = chunk->next_chunk;

            free(chunk->buffer);
            free(chunk);

            chunk = next_chunk;
        }
        ctx->first_chunk = NULL;
        ctx->last_chunk = NULL;
    }
}

uint8_t* http_assemble(http_context_t* ctx)
{
    if (ctx == NULL)
    {
        return NULL;
    }
    if (ctx->result != ESP_OK)
    {
        ESP_LOGE(TAG, "http data error at %u bytes (%s)", ctx->response_size, esp_err_to_name(ctx->result));
        http_cleanup(ctx);

        return NULL;
    }

    ESP_LOGI(TAG, "response size: %u", ctx->response_size);

    uint8_t* buffer = (uint8_t*)malloc(ctx->response_size + 1);
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "http assemble error for %u bytes", ctx->response_size);
        http_cleanup(ctx);
        return NULL;
    }
    http_chunk_t* chunk = ctx->first_chunk;
    //ESP_LOGI(TAG, "assemble chunk %p", chunk);
    size_t offset = 0;
    while (chunk != NULL)
    {
        //ESP_LOGI(TAG, "assemble chunk %p buffer %p size %d next %p", chunk, chunk->buffer, chunk->size, chunk->next_chunk);
        //ESP_LOG_BUFFER_HEX(TAG, chunk->buffer, chunk->size);
        memcpy(buffer + offset, chunk->buffer, chunk->size);
        offset += chunk->size;

        http_chunk_t* next_chunk = chunk->next_chunk;

        free(chunk->buffer);
        free(chunk);

        chunk = next_chunk;
    }
    buffer[ctx->response_size] = 0;

    ctx->first_chunk = NULL;
    ctx->last_chunk = NULL;

    //ESP_LOG_BUFFER_HEX(TAG, buffer, *json_size);

    return buffer;
}

cJSON* get_json(const char* url)
{
    esp_err_t ret = ESP_OK;

    cJSON* json = NULL;
    char* json_string = NULL;

    http_context_t ctx = { .result = ESP_OK };

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .disable_auto_redirect = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    ESP_GOTO_ON_ERROR(esp_http_client_perform(client), cleanup, TAG, "%s/esp_http_client_perform failed (%s)", __func__, esp_err_to_name(err_rc_));

    int http_status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "%s status: %d", url, http_status);
    if (http_status >= 200 && http_status < 300)
    {
        json_string = (char*)http_assemble(&ctx);
        if (json_string != NULL)
        {
            //ESP_LOGI(TAG, "json: %s", json_string);

            json = cJSON_ParseWithLength(json_string, ctx.response_size);
        }
    }    

cleanup:
    http_cleanup(&ctx);

    if (client != NULL)
    {
        esp_http_client_cleanup(client);
    }
    if (json_string != NULL)
    {
        free(json_string);
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "get_json failed (%s)", esp_err_to_name(ret));
    }

    return json;
}

void send_WOL(const char* mac, int repeat, int repeat_delay_ms)
{
    unsigned int mac_addr_temp[6];

    if (strlen(mac) != 17 || mac[2] != ':' || mac[2] != ':' || mac[5] != ':' || mac[8] != ':' || mac[11] != ':' || mac[14] != ':' ||
        sscanf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", &mac_addr_temp[0], &mac_addr_temp[1], &mac_addr_temp[2], &mac_addr_temp[3], &mac_addr_temp[4], &mac_addr_temp[5]) != 6)
    {
        ESP_LOGE(TAG, "send_WOL mac (%s) must be in XX:XX:XX:XX:XX:XX format", mac);
        return;
    }  

    uint8_t mac_addr[6];

    for (int i = 0; i < 6; i++)
    {
        mac_addr[i] = (uint8_t)mac_addr_temp[i];
    }

    struct sockaddr_in addr = {};

    addr.sin_len = sizeof(addr);
    addr.sin_addr.s_addr = 0xffffffff;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(WOL_PORT);

    uint8_t magic_packet[102];

    memset(magic_packet, 0xff, 6);

    for (int i = 0; i < 16; i++)
    {
        memcpy(magic_packet + 6 + i * 6, mac_addr, 6);
    }

    for (int i = 0; i < repeat; i++)
    {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "send_WOL/socket failed: %d", sock);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        int optval = 1;
        int error;

        if ((error = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&optval, sizeof(optval))) < 0)
        {
            ESP_LOGE(TAG, "send_WOL/setsocketopt failed: %d", error);
            closesocket(sock);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        if ((error = sendto(sock, magic_packet, sizeof(magic_packet), 0, (struct sockaddr*)&addr, sizeof(addr))) < 0)
        {
            ESP_LOGE(TAG, "send_WOL/sendto failed: %d", error);
            closesocket(sock);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        closesocket(sock);

        vTaskDelay(repeat_delay_ms / portTICK_PERIOD_MS);
    }
}

void delay_until(int64_t time)
{
    //portDISABLE_INTERRUPTS();
    int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() < time)
    {
        __asm__ __volatile__ ("nop");
    }
    if (esp_timer_get_time() - start - time > 200)
    {
        ESP_LOGE(TAG, "delay_until off desired %lld actual %lld", (time - start), (esp_timer_get_time() - start));
    }
    //portENABLE_INTERRUPTS();
}

void delay_us(int64_t time)
{
    delay_until(esp_timer_get_time() + time);
    //ets_delay_us(time);
}