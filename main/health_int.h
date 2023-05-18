#define HEALTH_PING_MAX_TIMEOUTS  5
#define HEALTH_PING_MAX_FAILURES  5
#define HEALTH_PING_INTERVAL      60000
#define HEALTH_PING_COUNT         10

static void health_ping_success(esp_ping_handle_t hdl, void *args);
static void health_ping_timeout(esp_ping_handle_t hdl, void *args);
static void health_ping_end(esp_ping_handle_t hdl, void *args);