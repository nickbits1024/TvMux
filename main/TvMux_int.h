#define TVMUX_SETUP_GPIO_NUM            (GPIO_NUM_18)
#define TVMUX_SETUP_GPIO_SEL            (1ull << TVMUX_SETUP_GPIO_NUM)

#define TVMUX_SETUP_ENABLED             (1)

#define TVMUX_STEAM_HOSTNAME            "seattle.home.nickpalmer.net"
#define TVMUX_STEAM_PORT                (1410)
#define TVMUX_STEAM_MAC                 "58:11:22:B4:B6:D9"
#define TVMUX_STEAM_PING_COUNT          (2)
#define TVMUX_STEAM_PING_INTERVAL       (500)

#define TVMUX_STEAM_RETRY_WAIT          (5000)
#define TVMUX_STEAM_RETRY_MAX           (6)

#define TVMUX_RETRY_MAX                 (3)
#define TVMUX_RETRY_WAIT_MS             (10000)
#define TVMUX_RETRY_CHECK_WAIT_MS       (15000)

#define HTTP_SUCCESS(http_code)         ((http_code) >= 200 && (http_code) <= 299)

uint8_t readI2CByte(uint8_t data_addr);
bool parse_edid(unsigned char* edid);
bool parse_edid_extension(uint8_t* edid2, uint8_t* ext);

bool tvmux_steam_is_on();
bool tvmux_steam_is_open();
void tvmux_steam_state_task(void* param);
esp_err_t tvmux_combine_devices_state(bool* state, bool and_mode, bool tv, bool audio, bool atv, bool use_cache);