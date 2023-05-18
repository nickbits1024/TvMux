#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "lwip/inet.h"
#include "ping/ping_sock.h"
#include "health_int.h"
#include "health.h"

#define TAG "health"

static int health_ping_timeouts;
static int health_ping_retries;
static int health_ping_replies;
static esp_ping_handle_t health_ping_handle;
static esp_timer_handle_t health_timer_handle;
static portMUX_TYPE health_mux = portMUX_INITIALIZER_UNLOCKED;

static void health_timer_callback(void* arg)
{
    static int last_ping_replies = -1;
    taskENTER_CRITICAL(&health_mux);
    int replies = health_ping_replies;
    taskEXIT_CRITICAL(&health_mux);

    ESP_LOGI(TAG, "Ping check: last=%d now=%d", last_ping_replies, replies);

    if (last_ping_replies == replies)
    {
        ESP_LOGE(TAG, "Ping quit responding, restarting...");
        esp_restart();
    }

    last_ping_replies = replies;
}

esp_err_t health_start()
{
    if (health_ping_handle != NULL)
    {        
        ESP_RETURN_ON_ERROR(esp_ping_stop(health_ping_handle), 
            TAG, "esp_ping_stop (%s)", esp_err_to_name(err_rc_));
        ESP_RETURN_ON_ERROR(esp_ping_delete_session(health_ping_handle), 
            TAG, "esp_ping_delete_session (%s)", esp_err_to_name(err_rc_));
        health_ping_handle = NULL;
    }
    if (health_timer_handle != NULL)
    {
        ESP_RETURN_ON_ERROR(esp_timer_stop(health_timer_handle), 
            TAG, "esp_timer_stop (%s)", esp_err_to_name(err_rc_));
        ESP_RETURN_ON_ERROR(esp_timer_delete(health_timer_handle), 
            TAG, "esp_timer_delete (%s)", esp_err_to_name(err_rc_));

        health_timer_handle = NULL;
    }

    health_ping_timeouts = 0;
    health_ping_replies = 0;

    ip_addr_t target_addr = IPADDR4_INIT(0x08080808);  // google dns

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.target_addr = target_addr;
    ping_config.task_stack_size = 4000;
    ping_config.count = HEALTH_PING_COUNT;
    ping_config.interval_ms = HEALTH_PING_INTERVAL;

    /* set callback functions */
    esp_ping_callbacks_t cbs = { };
    cbs.on_ping_success = health_ping_success;
    cbs.on_ping_timeout = health_ping_timeout;
    cbs.on_ping_end = health_ping_end;

    ESP_RETURN_ON_ERROR(esp_ping_new_session(&ping_config, &cbs, &health_ping_handle), 
            TAG, "esp_ping_new_session (%s)", esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_ping_start(health_ping_handle), 
            TAG, "esp_ping_start (%s)", esp_err_to_name(err_rc_));

    const esp_timer_create_args_t check_timer = {
            .callback = &health_timer_callback,
            .arg = NULL,
            .name = "health_check_timer"
    };    

    ESP_RETURN_ON_ERROR(esp_timer_create(&check_timer, &health_timer_handle),
        TAG, "esp_timer_create failed(%s)", esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(health_timer_handle, HEALTH_PING_INTERVAL * 2 * 1000),
        TAG, "esp_timer_start_periodic failed(%s)", esp_err_to_name(err_rc_));

    return ESP_OK;
}

static void health_ping_success(esp_ping_handle_t hdl, void *args)
{
    taskENTER_CRITICAL(&health_mux);
    health_ping_replies++;
    taskEXIT_CRITICAL(&health_mux);

    health_ping_timeouts = 0;
    health_ping_retries = 0;

    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    ESP_LOGI(TAG, "Ping reply from %s: seq=%u bytes=%lu time=%lums TTL=%u",
           inet_ntoa(target_addr.addr), seqno, recv_len, elapsed_time, ttl);
}

static void health_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    ESP_LOGI(TAG, "Ping timeout from %s icmp_seq=%d", inet_ntoa(target_addr.addr), seqno);

    taskENTER_CRITICAL(&health_mux);
    health_ping_replies++;
    taskEXIT_CRITICAL(&health_mux);

    health_ping_timeouts++;
    if (health_ping_timeouts == HEALTH_PING_MAX_TIMEOUTS)
    {
        health_ping_retries++;
        if (health_ping_retries == HEALTH_PING_MAX_FAILURES)
        {
            ESP_LOGE(TAG, "Max ping failures reached, restarting...");
            esp_restart();
        }
        else
        {
            health_ping_timeouts = 0;
            ESP_LOGE(TAG, "Max ping failures reached, reconnecting to health...");
            esp_wifi_disconnect();
        }
    }
}

static void health_ping_end(esp_ping_handle_t hdl, void *args)
{
    uint32_t transmitted;
    uint32_t received;
    uint32_t total_time_ms;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));
    ESP_LOGI(TAG, "Ping ended transmitted=%lu received=%lu time=%lums\n", transmitted, received, total_time_ms);

    ESP_ERROR_CHECK(esp_ping_start(health_ping_handle));
}
