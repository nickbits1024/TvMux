#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "ddc.h"
#include "util.h"
#include "cec.h"
#include "cec_int.h"
#include "TvMux.h"

#define TAG "cec"

QueueHandle_t cec_frame_queue_handle;
//QueueHandle_t cec_debug_bit_queue_handle;
//QueueHandle_t cec_debug_bit_queue2_handle;
esp_timer_handle_t cec_ack_timer_handle;

//cec_state_t cec_state = CEC_IDLE;
SemaphoreHandle_t cec_ack_sem;
//int64_t cec_ack_time;
atomic_bool cec_line_error;
atomic_uchar cec_last_source = CEC_LA_UNREGISTERED;
atomic_uchar cec_log_addr = CEC_LA_UNREGISTERED;
uint16_t cec_phy_addr;
atomic_int_least64_t cec_last_activity_time_us;

const char* CEC_STATE_NAMES[] =
{
    "CEC_IDLE",
    "CEC_START",
    "CEC_HEAD_0",
    "CEC_HEAD_1",
    "CEC_HEAD_2",
    "CEC_HEAD_3",
    "CEC_HEAD_4",
    "CEC_HEAD_5",
    "CEC_HEAD_6",
    "CEC_HEAD_7",
    "CEC_HEAD_EOM",
    "CEC_HEAD_ACK",
    "CEC_DATA_0",
    "CEC_DATA_1",
    "CEC_DATA_2",
    "CEC_DATA_3",
    "CEC_DATA_4",
    "CEC_DATA_5",
    "CEC_DATA_6",
    "CEC_DATA_7",
    "CEC_DATA_EOM",
    "CEC_DATA_ACK"
};

esp_err_t cec_init()
{
    assert(CEC_STATE_END == sizeof(CEC_STATE_NAMES) / sizeof(CEC_STATE_NAMES[0]));

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
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE; // GPIO_INTR_ANYEDGE

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    cec_frame_queue_handle = xQueueCreate(CEC_FRAME_QUEUE_LENGTH, sizeof(cec_frame_t));
    //cec_debug_bit_queue_handle = xQueueCreate(CEC_DEBUG_BIT_QUEUE_LENGTH, sizeof(cec_bit_t));
    //cec_debug_bit_queue2_handle = xQueueCreate(CEC_DEBUG_BIT_QUEUE_LENGTH, sizeof(cec_bit_t));

    cec_ack_sem = xSemaphoreCreateBinary();    

    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_NUM, 1));
    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_2_NUM, 1));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_EDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HDMI_CEC_GPIO_NUM, cec_isr, NULL));
    //ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_EDGE));
    //ESP_ERROR_CHECK(gpio_isr_handler_add(HDMI_CEC_GPIO_2_NUM, cec_isr_debug, NULL));

    esp_timer_create_args_t timer_args = 
    {
        .callback = cec_ack_timer_callback,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "cec_ack_timer",
        .skip_unhandled_events = true
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &cec_ack_timer_handle));

    xTaskCreatePinnedToCore(cec_loop, "cec_loop", 8000, NULL, 20, NULL, APP_CPU_NUM);
    //xTaskCreatePinnedToCore(cec_ack_loop, "cec_ack_loop", 8000, NULL, 20, NULL, APP_CPU_NUM);
    //xTaskCreate(cec_loop_debug, "cec_loop_debug", 8000, NULL, 20, NULL);
    //xTaskCreate(cec_loop_debug2, "cec_loop_debug2", 8000, NULL, 20, NULL);

    return ESP_OK;
}

#if 0
static void IRAM_ATTR cec_isr_debug(void* param)
{
    static atomic_bool busy = false;
    static int64_t last_low_time;
    static int64_t last_time;

    if (atomic_load(&busy))
    {
        ets_printf("cec_isr is not re-entrant\n");
        esp_restart();
        return;
    }
    atomic_store(&busy, true);
    busy = true;

    bool low = gpio_get_level(HDMI_CEC_GPIO_2_NUM) == 0;
    bool was_low = !low;

    int64_t time = esp_timer_get_time();

    if (last_time != 0)
    {
        cec_bit_t bit;

        bit.time = time;
        bit.last_time = last_time;
        bit.last_low_time = last_low_time;
        bit.low = was_low;

        bool first = true;

        while (xQueueSendFromISR(cec_debug_bit_queue_handle, &bit, NULL) != pdTRUE)
        {
            if (first)
            {
                ets_printf("cec_isr_debug queue full\n");
                first = false;
            }
        }
    }

    last_time = time;
    if (low)
    {
        last_low_time = time;
    }

    atomic_store(&busy, false);
}
#endif

static void IRAM_ATTR cec_isr(void* param)
{
    static atomic_bool busy = false;
    static int64_t last_low_time;
    static int64_t last_time;
    static cec_frame_t frame = { 0 };

    if (atomic_load(&busy))
    {
        ESP_DRAM_LOGE(TAG, "cec_isr is not re-entrant");
        //esp_restart();
        return;
    }
    atomic_store(&busy, true);
    busy = true;

    // flipped since bit was low in the past
    bool low = gpio_get_level(HDMI_CEC_GPIO_NUM) == 0;
    bool was_low = !low;

    int64_t time = esp_timer_get_time();

    atomic_store(&cec_last_activity_time_us, time);

    if (last_time != 0)
    {
        int64_t time_us = time - last_time;
        int64_t last_low_time_us = last_time - last_low_time;

        if (cec_bit_handle(&frame, was_low, time_us, last_low_time_us, false))
        {
            bool first = true;

            while (xQueueSendFromISR(cec_frame_queue_handle, &frame, NULL) != pdTRUE) 
            { 
                if (first)
                {
                    ESP_DRAM_LOGE(TAG, "cec_isr queue full\n");
                    first = false;
                }
            }
            
            memset(&frame, 0, sizeof(cec_frame_t));
        }        
    }

    last_time = time;
    if (low)
    {
        last_low_time = time;
    }

    atomic_store(&busy, false);
}

#if 0
void cec_loop_debug(void* param)
{
    cec_bit_t bit;
    for (;;)
    {
        if (xQueueReceive(cec_debug_bit_queue_handle, &bit, portMAX_DELAY) == pdTRUE)
        {
			if (xQueueSend(cec_debug_bit_queue2_handle, &bit, 0) != pdTRUE)
			{
				ESP_LOGE(TAG, "cec_debug_bit_queue2 full");
				continue;
			}

            // bool low = bit.low;
            // int64_t time_us = bit.time - bit.last_time;
            // int64_t last_low_time_us = bit.last_time - bit.last_low_time;
            // //int64_t lag_us = esp_timer_get_time() - bit.time;
            // //ESP_LOGI(TAG, "%s %llu us last low %llu lag %llu us", low ? "low" : "high", time_us, last_low_time_us, lag_us);

            // if (cec_bit_handle(&frame, low, time_us, last_low_time_us, true))
            // {
            //     cec_frame_handle(&frame, true);
            //     memset(&frame, 0, sizeof(cec_frame_t));
            // }            
        }
    }

    vTaskDelete(NULL);
}

void cec_loop_debug2(void* param)
{
    cec_frame_t frame = { 0 };
    cec_bit_t bit;
    for (;;)
    {
        if (xQueueReceive(cec_debug_bit_queue2_handle, &bit, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "bits incoming...");
            vTaskDelay(5000 / portTICK_PERIOD_MS);

            do
            {
                bool low = bit.low;
                int64_t time_us = bit.time - bit.last_time;
                int64_t last_low_time_us = bit.last_time - bit.last_low_time;
                //int64_t lag_us = esp_timer_get_time() - bit.time;
                //ESP_LOGI(TAG, "%s %llu us last low %llu lag %llu us", low ? "low" : "high", time_us, last_low_time_us, lag_us);

                if (cec_bit_handle(&frame, low, time_us, last_low_time_us, true))
                {
                    cec_frame_handle(&frame, true);
                    memset(&frame, 0, sizeof(cec_frame_t));
                }            
            }
            while (xQueueReceive(cec_debug_bit_queue2_handle, &bit, 0) == pdTRUE);
        }
    }

    vTaskDelete(NULL);
}
#endif

void cec_loop(void* param)
{
    for (;;)
    {

        ESP_LOGI(TAG, "Waiting for hotplug signal...");
        // FIXME
        // while (gpio_get_level(HDMI_HOTPLUG_GPIO_NUM) == 1)
        // { 
        //     vTaskDelay(50 / portTICK_PERIOD_MS);
        // }

        ESP_LOGI(TAG, "Hotplug signal detected!");

        uint8_t edid[DDC_EDID_LENGTH];
        uint8_t edid_extension[DDC_EDID_EXTENSION_LENGTH];

        do
        {
            for (int i = 0; i < DDC_EDID_LENGTH; i++)
            {
                ESP_ERROR_CHECK(ddc_read_byte(DDC_EDID_ADDRESS, i, &edid[i]));
                //printf("%02x ", edid[i]);
            }
            // printf("\n");
            ESP_LOGI(TAG, "EDID received");        
            // vTaskDelay(5000 / portTICK_PERIOD_MS);
        } 
        while (!cec_edid_parse(edid));

        if (edid[DDC_EDID_EXTENSION_FLAG])
        {
            do
            {
                for (int i = 0; i < DDC_EDID_EXTENSION_LENGTH; i++)
                {
                    ESP_ERROR_CHECK(ddc_read_byte(DDC_EDID_ADDRESS, DDC_EDID_LENGTH + i, &edid_extension[i]));
                }

                ESP_LOGI(TAG, "EDID extension received");

            } 
            while (!cec_edid_extension_parse(edid, edid_extension));
        }

        cec_frame_t frame = { 0 };

        atomic_store(&cec_log_addr, CEC_LA_UNREGISTERED);

        cec_logical_address_t la_candidates[] = { CEC_LA_TUNER_1, CEC_LA_TUNER_2, CEC_LA_TUNER_3, CEC_LA_TUNER_4 };

        for (int i = 0; i < sizeof(la_candidates) / sizeof(cec_logical_address_t); i++)
        {
            cec_logical_address_t la = la_candidates[i];

            ESP_LOGI(TAG, "poll la %d", la);

            frame.type = CEC_FRAME_TX;
            frame.src_addr = la;
            frame.dest_addr = la;
            
            esp_err_t result = cec_frame_transmit(&frame);

            if (result == ESP_ERR_TIMEOUT)
            {
                ESP_LOGI(TAG, "la %d free", la);
                atomic_store(&cec_log_addr, la);
                break;
            }
            else if (result == ESP_OK)
            {
                ESP_LOGI(TAG, "la %d in use", la);
            }
            else
            {
                ESP_LOGI(TAG, "la %d error (%s)", la, esp_err_to_name(result));
            }
        }

        if (atomic_load(&cec_log_addr) == CEC_LA_UNREGISTERED)
        {
            ESP_LOGI(TAG, "No free logical addresses, using unregistered address (%01x)", CEC_LA_UNREGISTERED);
        }

        for (;;)
        {
            if (xQueueReceive(cec_frame_queue_handle, &frame, portMAX_DELAY) == pdTRUE)
            {
                if (frame.type == CEC_FRAME_TX)
                {
                    cec_frame_transmit(&frame);
                }

                cec_frame_handle(&frame, false);
            }
        }
    }
    vTaskDelete(NULL);
}

static void cec_ack_timer_callback(void* param)
{
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
}

static bool cec_bit_handle(cec_frame_t* frame, bool low, int64_t time_us, int64_t last_low_time_us, bool debug)
{
    static uint16_t block;
    static cec_state_t cec_state = CEC_IDLE;
    bool complete = false;

    if (atomic_load(&cec_line_error))
    {
        cec_state = CEC_IDLE;
        atomic_store(&cec_line_error, false);
    }

    switch (cec_state)        
    {
        case CEC_IDLE:
            if (low)
            {
                if (time_us >= CEC_START0_MIN && time_us <= CEC_START0_MAX)
                {
                    if (debug) ESP_LOGI(TAG, "got start1 bit %llu", time_us);
                    cec_state = CEC_START;
                }        
                else
                {
                    if (debug) ESP_LOGE(TAG, "bad start1 bit %llu", time_us);
                    else ESP_DRAM_LOGE(TAG, "bad start1 bit %llu", time_us);
                }
            }
            break;
        case CEC_START:
            if (!low && last_low_time_us + time_us >= CEC_START_MIN && last_low_time_us + time_us <= CEC_START_MAX)
            {
                if (debug) ESP_LOGI(TAG, "got start bit %llu", last_low_time_us + time_us);
                cec_state = CEC_HEAD_0;
                block = 0;
            }
            else
            {
                if (debug) ESP_LOGE(TAG, "bad start bit %llu", last_low_time_us + time_us);
                else ESP_DRAM_LOGE(TAG, "bad start bit %llu", last_low_time_us + time_us);
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
                    if (debug) ESP_LOGI(TAG, "%s bit 1 %llu us", CEC_STATE_NAMES[cec_state], time_us);
                }
                else if (time_us >= CEC_DATA0_MIN && time_us <= CEC_DATA0_MAX)
                {
                    if (debug) ESP_LOGI(TAG, "%s bit 0 %llu us", CEC_STATE_NAMES[cec_state], time_us);
                }
                else 
                {
                    if (debug) ESP_LOGE(TAG, "%s bit low error time %llu us last_low %llu us", CEC_STATE_NAMES[cec_state], time_us, last_low_time_us);
                    else ESP_DRAM_LOGE(TAG, "%s bit low error time %llu us last_low %llu us", CEC_STATE_NAMES[cec_state], time_us, last_low_time_us);

                    cec_state = CEC_IDLE;
                    break;
                }

                if (cec_state == CEC_HEAD_ACK)
                {
                    if (debug) ESP_LOGI(TAG, "%s head %02x from %u to %u eom %d ack %d", CEC_STATE_NAMES[cec_state], CEC_BLOCK_DATA(block), CEC_HEAD_SRC(block), CEC_HEAD_DEST(block), CEC_BLOCK_EOM(block), CEC_BLOCK_ACK(block));

                    if (CEC_BLOCK_EOM(block))
                    {
                        cec_state = CEC_IDLE;
                        complete = true;
                    }
                    else
                    {
                        cec_state = CEC_DATA_0;
                    }

                    frame->src_addr = CEC_HEAD_SRC(block);
                    frame->dest_addr = CEC_HEAD_DEST(block);
                    frame->ack = CEC_BLOCK_ACK(block);

                    atomic_store(&cec_last_source, frame->src_addr);

                    block = 0;
                }
                else if (cec_state == CEC_DATA_ACK)
                {
                    if (debug) ESP_LOGI(TAG, "%s data %02x eom %d ack %d", CEC_STATE_NAMES[cec_state], CEC_BLOCK_DATA(block), CEC_BLOCK_EOM(block), CEC_BLOCK_ACK(block));

                    frame->ack &= CEC_BLOCK_ACK(block);
                    if (frame->data_size + 1 < CEC_FRAME_DATA_SIZE_MAX)
                    {
                        frame->data[frame->data_size++] = CEC_BLOCK_DATA(block);    
                    }

                    if (CEC_BLOCK_EOM(block))
                    {
                        cec_state = CEC_IDLE;
                        complete = true;
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
                if (cec_state == CEC_HEAD_ACK || cec_state == CEC_DATA_ACK)
                {
                    //int8_t level = gpio_get_level(HDMI_CEC_GPIO_NUM);
                    int8_t la = atomic_load(&cec_log_addr);
                    int8_t src = cec_state == CEC_DATA_ACK ? frame->src_addr : (block >> 5) & 0xf;
                    int8_t dest = cec_state == CEC_DATA_ACK ? frame->dest_addr : (block >> 1) & 0xf;
                    if (src != dest && la == dest)
                    {
                        gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
                        esp_err_t result = esp_timer_start_once(cec_ack_timer_handle, CEC_DATA0_TIME);
                        if (result != ESP_OK)
                        {
                            ESP_DRAM_LOGE(TAG, "ack_timer esp_timer_start_once failed (%s)", esp_err_to_name(result));
                        }

                        //ESP_DRAM_LOGI(TAG, "ack set (%lld us low %lld us) level %d block %02x la %d src %d dest %d", time_us, last_low_time_us, level, block, la, src, dest);
                    }
                    else
                    {
                        //ESP_DRAM_LOGI(TAG, "ack not set (%lld us) level %d block %02x la %d src %d dest %d", time_us, level, block, la, src, dest);
                    }
                }

            }
            break;
        default:
            break;
    }

    return complete;
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
                    // TODO save and respond to cec inq
                }
                break;
            }
        }

        index += 1 + length;
    }

    return true;
}

static void cec_frame_handle(const cec_frame_t* frame, bool debug)
{
    char data_string[CEC_FRAME_DATA_SIZE_MAX * 3 + 1];

    data_string[0] = 0;

    for (int i = 0; i < frame->data_size; i++)
    {
        snprintf(data_string + i * 3, 4, ":%02x", frame->data[i]);
    }

    ESP_LOGI(TAG, "%s%s (%d) %01x%01x%s", frame->type == CEC_FRAME_RX ? "rx" : "tx", debug ? "d" : "", 
        frame->data_size, frame->src_addr, frame->dest_addr, data_string);

}

static void cec_line_wait_free(int bit_periods)
{
    int64_t now = esp_timer_get_time();
    while (esp_timer_get_time() - now < bit_periods * CEC_BIT_TIME)
    {
        if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
        {
            now = esp_timer_get_time();
        }
    }
}

static bool cec_transmit_start(int64_t frame_start)
{
    //portDISABLE_INTERRUPTS();
    if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    {
        return false;
    }
#if 0
    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    delay_until(frame_start + CEC_START0_TIME);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
    //delay_until(frame_start + CEC_START_TIME);
    //portENABLE_INTERRUPTS();
#else
    int64_t bit_start = esp_timer_get_time();

    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    delay_us(CEC_START0_TIME);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
    delay_until(bit_start + CEC_START_TIME);

#endif

    return true;
}

//static int lag0;
//static int lag1;

static bool  cec_frame_transmit_bit(int64_t frame_start, int8_t* bit_index, bool bit_value, int bit_wait)
{
    if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    {
        return false;
    }

#if 0
    //ESP_LOGI(TAG, "tx bit %d", bit_value ? 1 : 0);
    int64_t bit_start = frame_start + CEC_START_TIME + *bit_index * CEC_BIT_TIME;
    delay_until(bit_start);
    //int64_t bit_start = esp_timer_get_time();
    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    // while (gpio_get_level(HDMI_CEC_GPIO_NUM) == 1)
    // {
    //     lag0++;
    //     //__asm__ __volatile__ ("nop");
    // }
    //delay_until(bit_start + (bit_value ? CEC_DATA1_TIME : CEC_DATA0_TIME));
    delay_until(bit_start + (bit_value ? CEC_DATA1_TIME : CEC_DATA0_TIME));
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
    // while (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    // {
    //     lag1++;
    //     //__asm__ __volatile__ ("nop");
    // }
    // if (bit_wait > 0)
    // {
    //     delay_until(bit_start + bit_wait);
    // }

   (*bit_index)++;
#else
    int64_t bit_start = esp_timer_get_time();
    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    delay_us(bit_value ? CEC_DATA1_TIME : CEC_DATA0_TIME);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
    delay_until(bit_start + bit_wait);
#endif
    return true;
}

static bool cec_frame_read_ack(int64_t frame_start, int8_t* bit_index, bool* ack)
{
#if 0
    int64_t bit_start = frame_start + CEC_START_TIME + *bit_index * CEC_BIT_TIME;

    delay_until(bit_start + CEC_BIT_TIME_SAMPLE);

    (*bit_index)++;

    return gpio_get_level(HDMI_CEC_GPIO_NUM) == 0;
#else
    int64_t bit_start = esp_timer_get_time();
    if (!cec_frame_transmit_bit(0, NULL, true, CEC_BIT_TIME_SAMPLE))
    {
        return false;
    }
    *ack = gpio_get_level(HDMI_CEC_GPIO_NUM) == 0;

    delay_until(bit_start + CEC_BIT_TIME);

    return true;
#endif

    //int64_t bit_start = esp_timer_get_time(); //frame_start + CEC_START_TIME + *bit_index * CEC_BIT_TIME;
    // int64_t bit_start_min = bit_start;

    // if (*bit_index > 0)
    // {
    //     bit_start_min -= CEC_BIT_TIME;
    //     bit_start_min += CEC_NEXT_START_MIN_TIME;
    // }

    // //int zeros = 0;
    // int64_t now = esp_timer_get_time();
    // while (esp_timer_get_time() < bit_start + CEC_BIT_TIME)
    // {
    //     //__asm__ __volatile__ ("nop");

    //     if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    //     {
    //         zeros++;
    //     }
    // }

    //ets_delay_us(CEC_BIT_TIME);

    //return true;

    // while (gpio_get_level(HDMI_CEC_GPIO_NUM) == 1)
    // {
    //     __asm__ __volatile__ ("nop");
    //     //taskYIELD();
    // }

    // ets_delay_us(CEC_BIT_SAMPLE_TIME);

    // bool ack = gpio_get_level(HDMI_CEC_GPIO_NUM) == 0;

    // ESP_LOGI(TAG, "bit ack %d", ack);

    // return ack;


    //ESP_LOGI(TAG, "bit zeros %d time %lld lag %d %d", zeros, esp_timer_get_time() - now, lag0, lag1);

    //return zeros > 0;

    //delay_until(bit_start_min);

    // bool bit;
    // while ((bit = gpio_get_level(HDMI_CEC_GPIO_NUM) == 1) && esp_timer_get_time() < bit_start + CEC_BIT_TIME)
    // {
    //     __asm__ __volatile__ ("nop");
    // }
    // if (!bit)
    // {
    //     int64_t low_time = esp_timer_get_time();
    //     delay_until(low_time + CEC_BIT_SAMPLE_TIME);
    //     return gpio_get_level(HDMI_CEC_GPIO_NUM) == 1;
    //     // while (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    //     // {
    //     //     __asm__ __volatile__ ("nop");
    //     // }
    //     // int64_t time_us = esp_timer_get_time() - low_time;
    //     // ESP_LOGI(TAG, "read bit time %llu", time_us);

    //     // if (time_us >= CEC_DATA1_MIN && time_us <= CEC_DATA1_MAX)
    //     // {                  
    //     //     return true;
    //     // }
    //     // else if (time_us >= CEC_DATA0_MIN && time_us <= CEC_DATA0_MAX)
    //     // {
    //     //     return false;
    //     // }
    // }
}


static bool cec_frame_transmit_byte(int64_t frame_start, int8_t* bit_index, uint8_t data, bool eom, bool* ack)
{
    *ack = false;

    //portDISABLE_INTERRUPTS();
    uint8_t shift = 7;

    for (int i = 0; i < 8; i++)
    {
        //ESP_LOGI(TAG, "bit shift %d value %d", shift, (data >> shift) & 1);
        if (!cec_frame_transmit_bit(frame_start, bit_index, (data >> shift) & 1, CEC_BIT_TIME))
        {
            return false;
        }
        shift--;
    }
    if (!cec_frame_transmit_bit(frame_start, bit_index, eom, CEC_BIT_TIME))
    {
        return false;
    }
    //int bi = *bit_index;
    if (!cec_frame_read_ack(frame_start, bit_index, ack))
    {
        return false;
    }
    //portENABLE_INTERRUPTS();
    //ESP_DRAM_LOGI(TAG, "tx ack %d", *ack);
    return true;
}


// void IRAM_ATTR test_timer_callback(void* param)
// {
//     int* ticks = (int*)param;

//     (*ticks)++;
// }

static esp_err_t cec_frame_transmit(cec_frame_t* frame)
{
    bool broadcast = frame->dest_addr == CEC_LA_BROADCAST;

    if (frame->type != CEC_FRAME_TX ||
        (frame->src_addr != atomic_load(&cec_log_addr) && frame->src_addr != frame->dest_addr && !broadcast))
    {
        return ESP_ERR_INVALID_ARG;
    }

    int retry = 0;
    int8_t wait = atomic_load(&cec_last_source) == frame->src_addr ? CEC_WAIT_CONTINUE : CEC_WAIT_NEW;
    do
    {
        frame->ack = false;
        cec_line_wait_free(wait);

        portDISABLE_INTERRUPTS();

        int8_t bit_index = 0;
        int64_t frame_start = esp_timer_get_time();
        if (!cec_transmit_start(frame_start))
        {
            ESP_DRAM_LOGE(TAG, "line error");
            atomic_store(&cec_line_error, true);
            continue;
        }
        uint8_t addr = (frame->src_addr << 4) | frame->dest_addr;

        if (!cec_frame_transmit_byte(frame_start, &bit_index, addr, frame->data_size == 0, &frame->ack))
        {
            ESP_DRAM_LOGE(TAG, "line error");
            
            atomic_store(&cec_line_error, true);
            continue;
        }
        //ESP_DRAM_LOGI(TAG, "loop ack %d", frame->ack);

        for (int i = 0; CEC_ACK_OK(broadcast, frame->ack) && i < frame->data_size; i++)
        {
            //bool ack = false;
            if (!cec_frame_transmit_byte(frame_start, &bit_index, frame->data[i], i == frame->data_size - 1, &frame->ack))
            {
                ESP_DRAM_LOGE(TAG, "line error");
                atomic_store(&cec_line_error, true);
                continue;
            }
            //frame->ack &= ack;
        }

        portENABLE_INTERRUPTS();
        taskYIELD();

        if (!CEC_ACK_OK(broadcast, frame->ack))
        {
            ESP_LOGE(TAG, "frame failed");
        }
        //frame->ack = ack;
        wait = CEC_WAIT_RETRY;
    }
    while (retry++ < CEC_FRAME_RETRY_MAX && !CEC_ACK_OK(broadcast, frame->ack));

    //ESP_LOGI(TAG, "exit ack %d", frame->ack);

    return CEC_ACK_OK(broadcast, frame->ack) ? ESP_OK : broadcast ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT;
}

static esp_err_t cec_frame_queue_add(cec_frame_t* frame)
{
    return xQueueSend(cec_frame_queue_handle, frame, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t cec_test()
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = CEC_LA_TUNER_1;
    frame.dest_addr = CEC_LA_TUNER_1;
    frame.data_size = 0;

    return cec_frame_queue_add(&frame);
}

esp_err_t cec_test2()
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = CEC_LA_AUDIO_SYSTEM;
    frame.data_size = 1;
    frame.data[0] = 0x83;

    return cec_frame_queue_add(&frame);
}

esp_err_t cec_test3()
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = CEC_LA_AUDIO_SYSTEM;
    frame.data_size = 1;
    frame.data[0] = 0x8f;

    return cec_frame_queue_add(&frame);
}