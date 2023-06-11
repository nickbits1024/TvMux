#define WEBSERVER_MAX_REQUEST_SIZE           1000

esp_err_t webserver_heap_get(httpd_req_t* req);
esp_err_t webserver_tv_get(httpd_req_t* req);
esp_err_t webserver_tv_post(httpd_req_t* req);
esp_err_t webserver_steam_get(httpd_req_t* req);
esp_err_t webserver_steam_post(httpd_req_t* req);
esp_err_t webserver_wii_get(httpd_req_t* req);
esp_err_t webserver_wii_post(httpd_req_t* req);
esp_err_t webserver_cec_log_get(httpd_req_t* req);
esp_err_t webserver_cec_get(httpd_req_t* req);
esp_err_t webserver_cec_test_get(httpd_req_t* req);
esp_err_t webserver_cec_test2_get(httpd_req_t* req);
esp_err_t webserver_standby_get(httpd_req_t* req);
esp_err_t webserver_tv_play_get(httpd_req_t* req);
esp_err_t webserver_tv_pause_get(httpd_req_t* req);

void webserver_complete_request(httpd_req_t* request, cJSON* response_doc);
esp_err_t webserver_heap_get(httpd_req_t* request);