#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_check.h"
#include "config.h"
#include "led.h"
#include "cec.h"
#include "util.h"
#include "TvMux.h"
#include "webserver_int.h"
#include "webserver.h"

#define TAG "WEBSERVER"

static httpd_handle_t webserver_handle;

esp_err_t webserver_init()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.stack_size = 8192;

    ESP_ERROR_CHECK(httpd_start(&webserver_handle, &config));

    httpd_uri_t httpd_uri = { };

    httpd_uri = (httpd_uri_t)
    {
        .uri = "/heap",
        .method = HTTP_GET,
        .handler = webserver_heap_get,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/tv",
      .method = HTTP_GET,
      .handler = webserver_tv_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/tv",
      .method = HTTP_POST,
      .handler = webserver_tv_post
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/steam",
      .method = HTTP_GET,
      .handler = webserver_steam_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/steam",
      .method = HTTP_POST,
      .handler = webserver_steam_post
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/wii",
      .method = HTTP_GET,
      .handler = webserver_wii_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/wii",
      .method = HTTP_POST,
      .handler = webserver_wii_post
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/cec/log",
      .method = HTTP_GET,
      .handler = webserver_cec_log_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/cec/test",
      .method = HTTP_GET,
      .handler = webserver_cec_test_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/cec/test2",
      .method = HTTP_GET,
      .handler = webserver_cec_test2_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/cec/test3",
      .method = HTTP_GET,
      .handler = webserver_cec_test3_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));


    httpd_uri = (httpd_uri_t)
    {
      .uri = "/cec",
      .method = HTTP_GET,
      .handler = webserver_cec_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/standby",
      .method = HTTP_GET,
      .handler = webserver_standby_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/tv/play",
      .method = HTTP_GET,
      .handler = webserver_tv_play_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    httpd_uri = (httpd_uri_t)
    {
      .uri = "/tv/pause",
      .method = HTTP_GET,
      .handler = webserver_tv_pause_get
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(webserver_handle, &httpd_uri));

    return ESP_OK;
}

void webserver_complete_request(httpd_req_t* request, cJSON* response_doc)
{
    char* json = cJSON_Print(response_doc);
    if (json != NULL)
    {
        httpd_resp_set_type(request, "application/json");
        httpd_resp_send(request, json, HTTPD_RESP_USE_STRLEN);
        cJSON_free(json);
    }
    else
    {
        httpd_resp_send_500(request);
    }
    cJSON_Delete(response_doc);
}

esp_err_t webserver_heap_get(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_INTERNAL);

    cJSON_AddNumberToObject(response_doc, "total_free_bytes", heap_info.total_free_bytes);
    cJSON_AddNumberToObject(response_doc, "total_allocated_bytes", heap_info.total_allocated_bytes);
    cJSON_AddNumberToObject(response_doc, "free_size", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(response_doc, "minimum_free_size", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(response_doc, "largest_free_block", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_standby_get(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    cec_standby();

    cJSON_AddStringToObject(response_doc, "status", "ok");

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_tv_pause_get(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    cec_pause();

    cJSON_AddStringToObject(response_doc, "status", "ok");

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_tv_play_get(httpd_req_t* request)
{
    cJSON* response_doc = cJSON_CreateObject();

    cec_play();

    cJSON_AddStringToObject(response_doc, "status", "ok");

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}



esp_err_t webserver_tv_get(httpd_req_t* request)
{
    bool pending;
    bool tv_state;    

    tvmux_tv_state_get(&tv_state, &pending);

    const char* state_string = tv_state ? "on" : "off";

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddBoolToObject(response_doc, "pending", pending);

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_wii_get(httpd_req_t* request)
{
    bool pending;
    wii_power_state_t wii_power_state;

    tvmux_wii_state_get(&wii_power_state, &pending);

    const char* state_string = wii_power_state == WII_POWER_STATE_ON ? "on" : wii_power_state == WII_POWER_STATE_OFF ? "off" : "error";

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddBoolToObject(response_doc, "pending", pending);

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

cJSON* parse_request(httpd_req_t* request)
{
    char json[WEBSERVER_MAX_REQUEST_SIZE];

    if (request->content_len > WEBSERVER_MAX_REQUEST_SIZE)
    {
        return NULL;
    }

    int ret = httpd_req_recv(request, json, request->content_len);
    if (ret <= 0)
    {
        return NULL;
    }

    return cJSON_Parse(json);
}

esp_err_t webserver_tv_post(httpd_req_t* request)
{
    cJSON* request_doc = parse_request(request);
    if (request_doc == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    cJSON* response_doc = cJSON_CreateObject();

    cJSON* state = cJSON_GetObjectItem(request_doc, "state");

    if (state == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    const char* state_value = cJSON_GetStringValue(state);
    bool desired_state = false;

    if (strcmp(state_value, "on") == 0)
    {
        printf("turn tv on\n");
        desired_state = true;
    }
    else if (strcmp(state_value, "off") == 0)
    {
        printf("turn tv off\n");
        desired_state = false;
    }
    else
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    tvmux_tv_power(desired_state);

    const char* state_string = desired_state ? "on" : "off";

    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddBoolToObject(response_doc, "pending", true);

    webserver_complete_request(request, response_doc);
    cJSON_Delete(request_doc);

    return ESP_OK;
}

esp_err_t webserver_steam_get(httpd_req_t* request)
{
    bool state;
    bool pending;
    
    tvmux_steam_state(false, true, &state, &pending);

    const char* state_string = state ? "on" : "off";

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddBoolToObject(response_doc, "pending", pending);

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_steam_post(httpd_req_t* request)
{
    cJSON* request_doc = parse_request(request);
    if (request_doc == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    cJSON* response_doc = cJSON_CreateObject();

    cJSON* state = cJSON_GetObjectItem(request_doc, "state");

    if (state == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    const char* state_value = cJSON_GetStringValue(state);
    bool desired_state = false;

    if (strcmp(state_value, "on") == 0)
    {

        desired_state = true;
    }
    else if (strcmp(state_value, "off") == 0)
    {
        desired_state = false;
    }
    else
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    tvmux_steam_power(desired_state);

    const char* state_string = desired_state ? "on" : "off";

    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddBoolToObject(response_doc, "pending", true);

    webserver_complete_request(request, response_doc);
    cJSON_Delete(request_doc);

    return ESP_OK;
}


esp_err_t webserver_wii_post(httpd_req_t* request)
{
    cJSON* request_doc = parse_request(request);
    if (request_doc == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    cJSON* response_doc = cJSON_CreateObject();

    cJSON* state = cJSON_GetObjectItem(request_doc, "state");

    if (state == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    bool desired_state = false;

    const char* state_value = cJSON_GetStringValue(state);
    if (strcmp(state_value, "on") == 0)
    {
        desired_state = true;
    }
    else if (strcmp(state_value, "off") == 0)
    {
        desired_state = false;
    }
    else
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    tvmux_wii_power(desired_state);

    const char* state_string = desired_state ? "on" : "off";

    cJSON_AddStringToObject(response_doc, "state", state_string);
    cJSON_AddStringToObject(response_doc, "status", "ok");
    cJSON_AddBoolToObject(response_doc, "pending", true);
    
    webserver_complete_request(request, response_doc);
    cJSON_Delete(request_doc);

    return ESP_OK;

}

esp_err_t webserver_cec_test_get(httpd_req_t* request)
{
    esp_err_t result = cec_test();

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status",  result == ESP_OK ? "ok" : "error");
    cJSON_AddNumberToObject(response_doc, "error", result);

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_cec_test2_get(httpd_req_t* request)
{
    esp_err_t result = cec_test2();

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status",  result == ESP_OK ? "ok" : "error");
    cJSON_AddNumberToObject(response_doc, "error", result);

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_cec_test3_get(httpd_req_t* request)
{
    esp_err_t result = cec_test3();

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddStringToObject(response_doc, "status",  result == ESP_OK ? "ok" : "error");
    cJSON_AddNumberToObject(response_doc, "error", result);

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

esp_err_t webserver_cec_log_get(httpd_req_t* request)
{
    httpd_resp_set_type(request, "text/plain");
    
    tvmux_cec_log_write(request);

    size_t qs_size = httpd_req_get_url_query_len(request) + 1;
    char* qs = (char*)malloc(qs_size);

    if (qs == NULL)
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    if (httpd_req_get_url_query_str(request, qs, qs_size) != ESP_OK)
    {
        free(qs);
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    char clear_string[10];
    bool clear = false;

    if (httpd_query_key_value(qs, "clear", clear_string, sizeof(clear_string)) == ESP_OK)
    {
        clear = atoi(clear_string) > 0;
    }

    free(qs);

    if (clear)
    {
        tvmux_cec_log_clear();
    }    

    return ESP_OK;
}

esp_err_t webserver_cec_get(httpd_req_t* request)
{
    size_t qs_size = httpd_req_get_url_query_len(request) + 1;
    char* qs = (char*)malloc(qs_size);

    if (qs == NULL)
    {
        return ESP_FAIL;
    }

    if (httpd_req_get_url_query_str(request, qs, qs_size) != ESP_OK)
    {
        free(qs);
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    char addr_string[3];
    char cmd_string[CEC_FRAME_SIZE_MAX * 3];

    if (httpd_query_key_value(qs, "addr", addr_string, sizeof(addr_string)) != ESP_OK ||
        httpd_query_key_value(qs, "cmd", cmd_string, sizeof(cmd_string)) != ESP_OK)
    {
        free(qs);
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    char reply_cmd_string[3] = { };
    bool has_reply = httpd_query_key_value(qs, "reply", reply_cmd_string, sizeof(reply_cmd_string)) == ESP_OK;

    free(qs);

    int addr = atoi(addr_string);
    if (addr == -1)
    {
        addr = 0xf;
    }

    uint8_t cmd[CEC_FRAME_SIZE_MAX];
    uint8_t reply[CEC_FRAME_SIZE_MAX];
    int len = (strlen(cmd_string) + 1) / 3;
    if (len == 0 ||
        len > sizeof(cmd) ||
        (has_reply && strlen(reply_cmd_string) != 2))
    {
        httpd_resp_send_500(request);
        return ESP_FAIL;
    }

    printf("cec cmd=%s reply_command=%s\n", cmd_string, reply_cmd_string);
    const char* hex = cmd_string;
    unsigned int temp;
    for (int i = 0; i < len; i++)
    {
        sscanf(hex + i * 3, "%02x", &temp);
        cmd[i] = (unsigned char)temp;
    }

    int reply_filter = -1;
    if (has_reply)
    {
        hex = reply_cmd_string;
        sscanf(hex, "%02x", &reply_filter);
    }

    //std::string reply_string;
    char reply_string[CEC_FRAME_SIZE_MAX * 3 + 1] = { };
    const char* status = "ok";

    if (has_reply)
    {
        int reply_size = CEC_FRAME_SIZE_MAX;
        if (tvmux_cec_control(addr, cmd, len, (uint8_t)reply_filter, reply, &reply_size) == ESP_OK)
        {
            format_bytes(reply_string, sizeof(reply_string), reply + 1, reply_size - 1);
        }
        else
        {
            status = "error";
        }
    }
    else
    {
        if (tvmux_cec_control(addr, cmd, len, 0, NULL, NULL) != ESP_OK)
        {
            status = "error2";
        }
    }

    cJSON* response_doc = cJSON_CreateObject();

    cJSON_AddNumberToObject(response_doc, "addr", addr);
    cJSON_AddStringToObject(response_doc, "cmd", cmd_string);
    cJSON_AddStringToObject(response_doc, "status", status);
    if (has_reply)
    {
        cJSON_AddStringToObject(response_doc, "reply", reply_string);
    }

    webserver_complete_request(request, response_doc);

    return ESP_OK;
}

