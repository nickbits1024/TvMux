
#define WIFI_NVS_NAME           "wifi"
#define WIFI_SSID_NVS_KEY       "ssid"
#define WIFI_PASSWORD_NVS_KEY   "pw"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static esp_err_t wifi_connect();
static esp_err_t wifi_reset();
static void wifi_task(void* param);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
