#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "ddc.h"
#include "cec.h"
#include "cec_int.h"
#include "TvMux.h"

#define TAG "cec"

QueueHandle_t cec_bit_queue;
cec_state_t cec_state = CEC_IDLE;
cec_logical_address_t cec_log_addr;
uint16_t cec_phy_addr;

esp_err_t cec_init()
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = HDMI_HOTPLUG_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = HDMI_CEC_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = HDMI_CEC_GPIO_2_SEL;
    io_conf.mode = GPIO_MODE_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    //cec_bit_queue = xQueueCreate(160 * 10, sizeof(cec_bit_times_t));
    cec_bit_queue = xQueueCreate(CEC_BIT_QUEUE_LENGTH, sizeof(cec_bit_t));

    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_NUM, 1));
    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_2_NUM, 1));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HDMI_CEC_GPIO_NUM, cec_isr, NULL));

    xTaskCreate(cec_loop, "cec_loop", 10000, NULL, 2, NULL);

    return ESP_OK;
}

static void IRAM_ATTR cec_isr(void* param)
{
    static volatile bool busy = false;
     static int64_t last_low_time;
    // static int64_t last_high_time;
    static int64_t last_time;

    if (busy)
    {
        ets_printf("cec_isr is not re-entrant");
        esp_restart();
        return;
    }
    busy = true;

    // flipped since bit was low in the past
    bool low = gpio_get_level(HDMI_CEC_GPIO_NUM) == 0;
    bool was_low = !low;

    int64_t time = esp_timer_get_time();

    if (last_time != 0)
    {
        cec_bit_t bit;

        bit.time = time;
        bit.last_time = last_time;
        bit.last_low_time = last_low_time;
        bit.low = was_low;

        while (xQueueSendFromISR(cec_bit_queue, &bit, NULL) != pdTRUE) { }
    }

    last_time = time;
    if (low)
    {
        last_low_time = time;
    }

    busy = false;
}

void cec_loop(void* param)
{
    cec_bit_t bit;
    for (;;)
    {
        if (xQueueReceive(cec_bit_queue, &bit, portMAX_DELAY) == pdTRUE)
        {
            bool low = bit.low;
            int64_t time_us = bit.time - bit.last_time;
            int64_t last_low_time_us = bit.last_time - bit.last_low_time;
            //int64_t lag_us = esp_timer_get_time() - bit.time;
            //ESP_LOGI(TAG, "%s %llu us last low %llu lag %llu us", low ? "low" : "high", time_us, last_low_time_us, lag_us);

            cec_bit_handle(low, time_us, last_low_time_us);
        }
    }
    vTaskDelete(NULL);
}

static void cec_bit_handle(bool low, int64_t time_us, int64_t last_low_time_us)
{
    static uint16_t block;

    switch (cec_state)        
    {
        case CEC_IDLE:
            if (low)
            {
                if (time_us >= CEC_START0_MIN && time_us <= CEC_START0_MAX)
                {
                    //ESP_LOGI(TAG, "got start1 bit %llu", time_us);
                    cec_state = CEC_START;
                }        
                else
                {
                    ESP_LOGE(TAG, "bad start1 bit %llu", time_us);
                }
            }
            break;
        case CEC_START:
            if (!low && last_low_time_us + time_us >= CEC_START_MIN && last_low_time_us + time_us <= CEC_START_MAX)
            {
                //ESP_LOGI(TAG, "got start bit %llu", last_low_time_us + time_us);
                cec_state = CEC_HEAD_0;
                block = 0;
            }
            else
            {
                ESP_LOGE(TAG, "bad start bit %llu", last_low_time_us + time_us);
                cec_state = CEC_IDLE;
            }
            break;
        case CEC_HEAD_0:
        case CEC_HEAD_1:
        case CEC_HEAD_2:
        case CEC_HEAD_3:
        case CEC_HEAD_4:
        case CEC_HEAD_5:
        case CEC_HEAD_6:
        case CEC_HEAD_7:
        case CEC_HEAD_EOM:
        case CEC_HEAD_ACK:
        case CEC_DATA_0:
        case CEC_DATA_1:
        case CEC_DATA_2:
        case CEC_DATA_3:
        case CEC_DATA_4:
        case CEC_DATA_5:
        case CEC_DATA_6:
        case CEC_DATA_7:
        case CEC_DATA_EOM:
        case CEC_DATA_ACK:
            if (low)
            {
                block <<= 1;
                if (time_us >= CEC_DATA1_MIN && time_us <= CEC_DATA1_MAX)
                {                  
                    block |= 1;
                }
                else if (time_us >= CEC_DATA0_MIN && time_us <= CEC_DATA0_MAX)
                {
                }
                else 
                {
                    ESP_LOGI(TAG, "bit error state %u time %llu us last_low %llu us", cec_state, time_us, last_low_time_us);
                    cec_state = CEC_IDLE;
                    break;
                }

                if (cec_state == CEC_HEAD_ACK)
                {
                    ESP_LOGI(TAG, "head %02x from %u to %u eom %d ack %d", CEC_BLOCK_DATA(block), CEC_HEAD_INIT(block), CEC_HEAD_DEST(block), CEC_BLOCK_EOM(block), CEC_BLOCK_ACK(block));

                    if (CEC_BLOCK_EOM(block))
                    {
                        cec_state = CEC_IDLE;
                    }
                    else
                    {
                        cec_state = CEC_DATA_0;
                    }

                    block = 0;
                }
                else if (cec_state == CEC_DATA_ACK)
                {
                    ESP_LOGI(TAG, "data %02x eom %d ack %d", CEC_BLOCK_DATA(block), CEC_BLOCK_EOM(block), CEC_BLOCK_ACK(block));

                    if (CEC_BLOCK_EOM(block))
                    {
                        cec_state = CEC_IDLE;
                    }
                    else
                    {
                        cec_state = CEC_DATA_0;
                    }

                    block = 0;
                }
                else
                {
                    cec_state++;
                }
            }
            else
            {
                if (!CEC_BLOCK_EOM(block))
                {
                    uint64_t bit_time = last_low_time_us + time_us;
                    if (bit_time < CEC_BIT_TIME_MIN || bit_time > CEC_BIT_TIME_MAX)
                    {
                        ESP_LOGI(TAG, "bit error state %u time %llu us last_low %llu us", cec_state, time_us, last_low_time_us);
                        cec_state = CEC_IDLE;
                        break;
                    }
                }
            }
            break;
        default:
            break;
    }
}

esp_err_t cec_queue_clear()
{

    return ESP_OK;
}

esp_err_t cec_standby()
{

    return ESP_OK;
}

esp_err_t cec_pause()
{

    return ESP_OK;
}

esp_err_t cec_play()
{

    return ESP_OK;
}

esp_err_t cec_tv_power(bool power_on)
{

    return ESP_OK;
}

esp_err_t cec_wii_power(bool power_on)
{

    return ESP_OK;
}

esp_err_t cec_log_write(httpd_req_t* request)
{

    return ESP_OK;
}

esp_err_t cec_log_clear()
{

    return ESP_OK;
}

esp_err_t cec_control(int target_address, const uint8_t* request, int request_size, uint8_t reply_filter, uint8_t* reply, int* reply_size)
{

    return ESP_OK;
}

esp_err_t cec_sam_request(uint16_t addr)
{

    return ESP_OK;
}

esp_err_t cec_as_set(uint16_t addr)
{

    return ESP_OK;
}

esp_err_t cec_tv_on()
{

    return ESP_OK;
}

esp_err_t cec_combine_devices_state(bool* state, bool and_mode, bool tv, bool audio, bool atv)
{

    return ESP_OK;
}

bool cec_edid_parse(unsigned char* edid)
{
    uint32_t header0 = edid[0] << 24 | edid[1] << 16 | edid[2] << 8 | edid[3];
    uint32_t header1 = edid[4] << 24 | edid[5] << 16 | edid[6] << 8 | edid[7];

    uint32_t sum = 0;
    for (int i = 0; i < DDC_EDID_LENGTH; i++)
    {
        sum += (uint32_t)edid[i];
    }

    //ESP_LOGI(TAG, "EDID checksum: %08lx", sum);
    //ESP_LOGI(TAG, "EDID header: %08lx%08lx", header0, header1);

    if (header0 != 0x00ffffff || header1 != 0xffffff00)
    {
        return false;
    }

    if (sum % 256 != 0)
    {
        return false;
    }

    uint16_t manufacturer0 = edid[0x08];
    uint16_t manufacturer1 = edid[0x09];
    char manufacturer[4];
    manufacturer[0] = '@' + (int)((manufacturer0 >> 2) & 0x1f);
    manufacturer[1] = '@' + (int)((manufacturer0 & 3) << 3 | ((manufacturer1 >> 5) & 0x7));
    manufacturer[2] = '@' + (int)(manufacturer1 & 0x1f);
    manufacturer[3] = 0;

    uint8_t version = edid[0x12];
    uint8_t revision = edid[0x13];
    //uint8_t extension_flag = edid[EDID_EXTENSION_FLAG];

    ESP_LOGI(TAG, "EDID version: %u.%u", version, revision);
    ESP_LOGI(TAG, "EDID manufacturer: %s", manufacturer);
    //ESP_LOGI(TAG, "EDID extension_flag: %d", extension_flag);

    return true;
    /*

      uint8_t ieee0 = edid[0x95];
      uint8_t ieee1 = edid[0x96];
      uint8_t ieee2 = edid[0x97];
      uint16_t physicalAddress = edid[0x98] << 8 | edid[0x99];

      ESP_LOGI(TAG, "IEEE ID: %02x%02x%02x", ieee0, ieee1, ieee2);
      uint8_t a0 = physicalAddress >> 12;
      uint8_t a1 = (physicalAddress >> 8) & 0xf;
      uint8_t a2 = (physicalAddress >> 4) & 0xf;
      uint8_t a3 = physicalAddress & 0xf;
      ESP_LOGI(TAG, "cPhysical Address: %u.%u.%u.%u", a0, a1, a2, a3);

      return true;*/
}

bool cec_edid_extension_parse(uint8_t* edid2, uint8_t* ext)
{
    uint32_t sum = 0;
    for (int i = 0; i < DDC_EDID_EXTENSION_LENGTH; i++)
    {
        sum += (uint32_t)ext[i];
    }

    if (sum % 256 != 0)
    {
        return false;
    }

    //ESP_LOGI(TAG, "EDID ext checksum: %08x", sum);

    uint8_t tag = ext[0];
    //uint8_t revision = ext[1];
    uint8_t dtd_offset = ext[2];
    uint8_t offset = 4;

    // ESP_LOGI(TAG, "EDID ext tag: %u", tag);
    // ESP_LOGI(TAG, "EDID ext revision: %u", revision);
    // ESP_LOGI(TAG, "EDID ext dtd_offset: %u", dtd_offset);

    if (tag != 2)
    {
        return false;
    }

    // for (int i = 0; i < EDID_EXTENSION_LENGTH; i++)
    // {
    //   ESP_LOGI(TAG, "0x%02x, ", ext[i]);
    // }
    // ESP_LOGI(TAG, );

    uint8_t index = offset;

    while (index < dtd_offset)
    {
        uint8_t* p = ext + index;
        uint8_t tag = p[0] >> 5;
        uint8_t length = p[0] & 0x1f;

        //ESP_LOGI(TAG, "EDID ext tag: %d length: %d", tag, length);

        switch (tag)
        {
            case 3:
            {
                uint8_t ieee[3];
                ieee[0] = p[3];
                ieee[1] = p[2];
                ieee[2] = p[1];
                //ESP_LOGI(TAG, "EDID IEEE %02x %02x %02x", ieee[0], ieee[1], ieee[2]);
                if (ieee[0] == 0x00 && ieee[1] == 0x0c && ieee[2] == 0x03)
                {
                    cec_phy_addr = (uint16_t)p[4] << 8 | p[5];
                    uint8_t a0 = cec_phy_addr >> 12;
                    uint8_t a1 = (cec_phy_addr >> 8) & 0xf;
                    uint8_t a2 = (cec_phy_addr >> 4) & 0xf;
                    uint8_t a3 = cec_phy_addr & 0xf;

                    ESP_LOGI(TAG, "CEC Physical Address: %u.%u.%u.%u", a0, a1, a2, a3);

                }
                break;
            }
        }

        index += 1 + length;
    }

    return true;
}
