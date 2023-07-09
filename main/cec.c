//#define CEC_DEBUG

#include <stdio.h>
#include <stdlib.h>
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
#ifdef CEC_DEBUG
QueueHandle_t cec_debug_bit_queue_handle;
QueueHandle_t cec_debug_bit_queue2_handle;
#endif
esp_timer_handle_t cec_ack_timer_handle;

atomic_bool cec_line_error;
atomic_bool cec_ack_set;
atomic_uchar cec_last_source = CEC_LA_UNREGISTERED;
atomic_uchar cec_log_addr = CEC_LA_UNREGISTERED;
uint16_t cec_phy_addr;
uint16_t cec_active_source_address = 0xffff;


portMUX_TYPE cec_mux = portMUX_INITIALIZER_UNLOCKED;
cec_log_entry_t* cec_log_head;
cec_log_entry_t* cec_log_tail;
int cec_log_size;

SemaphoreHandle_t cec_control_request_sem;
SemaphoreHandle_t cec_control_response_sem;
portMUX_TYPE cec_control_response_mux = portMUX_INITIALIZER_UNLOCKED;

cec_logical_address_t cec_control_reply_from;
uint8_t cec_control_reply[CEC_FRAME_SIZE_MAX];
int cec_control_reply_size;
uint8_t cec_power_statuses[CEC_LA_MAX + 1] = 
    { CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, 
      CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, 
      CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, 
      CEC_PS_UNKNOWN, CEC_PS_UNKNOWN, CEC_PS_UNKNOWN };


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
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
#ifdef CEC_DEBUG
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
#else
    io_conf.mode = GPIO_MODE_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
#endif

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    cec_frame_queue_handle = xQueueCreate(CEC_FRAME_QUEUE_LENGTH, sizeof(cec_frame_t));
#ifdef CEC_DEBUG
    cec_debug_bit_queue_handle = xQueueCreate(CEC_DEBUG_BIT_QUEUE_LENGTH, sizeof(cec_bit_t));
    cec_debug_bit_queue2_handle = xQueueCreate(CEC_DEBUG_BIT_QUEUE_LENGTH, sizeof(cec_bit_t));
#endif

    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_NUM, 1));
    ESP_ERROR_CHECK(gpio_set_level(HDMI_CEC_GPIO_2_NUM, 1));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_EDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HDMI_CEC_GPIO_NUM, cec_isr, NULL));

#ifdef CEC_DEBUG
    ESP_ERROR_CHECK(gpio_isr_handler_add(HDMI_CEC_GPIO_2_NUM, cec_isr_debug, NULL));
#endif

    esp_timer_create_args_t timer_args = 
    {
        .callback = cec_ack_timer_callback,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "cec_ack_timer",
        .skip_unhandled_events = true
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &cec_ack_timer_handle));


    cec_control_request_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(cec_control_request_sem);
    cec_control_response_sem = xSemaphoreCreateBinary();
    //xSemaphoreGive(cec_control_response_sem);

    xTaskCreatePinnedToCore(cec_loop_task, "cec_loop", 8000, NULL, 5, NULL, APP_CPU_NUM);
#ifdef CEC_DEBUG
    xTaskCreate(cec_loop_debug_task, "cec_loop_debug", 8000, NULL, 5, NULL);
    xTaskCreate(cec_loop_debug2_task, "cec_loop_debug2", 8000, NULL, 5, NULL);
#endif

    return ESP_OK;
}

#ifdef CEC_DEBUG
static void IRAM_ATTR cec_isr_debug(void* param)
{
    static atomic_bool busy = false;
    static int64_t last_low_time;
    static int64_t last_time;

    if (atomic_load(&busy))
    {
        ESP_DRAM_LOGE(TAG, "cec_isr is not re-entrant");
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
                ESP_DRAM_LOGE(TAG, "cec_isr_debug queue full");
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

    //atomic_store(&cec_last_activity_time_us, time);

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
                    ESP_DRAM_LOGE(TAG, "cec_isr queue full");
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

#ifdef CEC_DEBUG
void cec_loop_debug_task(void* param)
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

void cec_loop_debug2_task(void* param)
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

// void cec_report_physical_address_task(void* param)
// {
//     cec_report_physical_address();
//     vTaskDelete(NULL);
// }

void cec_loop_task(void* param)
{
    for (;;)
    {
        ESP_LOGI(TAG, "Waiting for hotplug signal...");
        while (gpio_get_level(HDMI_HOTPLUG_GPIO_NUM) == 1)
        { 
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

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

        //xTaskCreate(cec_report_physical_address_task, "cec_report_physical_address", 2000, NULL, 1, NULL);
        cec_report_physical_address();

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

static void IRAM_ATTR cec_ack_timer_callback(void* param)
{
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
}

static bool IRAM_ATTR cec_bit_handle(cec_frame_t* frame, bool low, int64_t time_us, int64_t last_low_time_us, bool debug)
{
    static uint16_t block;
    static cec_state_t cec_state = CEC_IDLE;
    bool complete = false;

    if (atomic_load(&cec_line_error))
    {
        cec_state = CEC_IDLE;
        memset(frame, 0, sizeof(cec_frame_t));
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
                    //else ESP_DRAM_LOGE(TAG, "bad start1 bit %llu", time_us);
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
                //else ESP_DRAM_LOGE(TAG, "bad start bit %llu", last_low_time_us + time_us);
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
                else if (time_us >= CEC_START0_MIN && time_us <= CEC_START0_MAX)
                {
                    // something goofed
                    cec_state = CEC_START;
                    memset(frame, 0, sizeof(cec_frame_t));
                    break;
                }
                else
                {
                    if (debug) ESP_LOGE(TAG, "%s bit low error time %llu us last_low %llu us", CEC_STATE_NAMES[cec_state], time_us, last_low_time_us);
                    //else ESP_DRAM_LOGE(TAG, "%s bit low error time %llu us last_low %llu us", CEC_STATE_NAMES[cec_state], time_us, last_low_time_us);

                    frame->bit_error = true;
                }

                if (cec_state == CEC_HEAD_ACK)
                {
                    if (debug) ESP_LOGI(TAG, "%s head %02x from %u to %u eom %d ack %d", CEC_STATE_NAMES[cec_state], CEC_BLOCK_DATA(block), CEC_HEAD_SRC(block), CEC_HEAD_DEST(block), CEC_BLOCK_EOM(block), CEC_BLOCK_ACK(block));

                    if (!frame->bit_error || CEC_ACK_OK(frame->dest_addr == CEC_LA_BROADCAST, frame->ack))
                    {
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
                    }
                    else
                    {
                        cec_state = CEC_HEAD_0;
                        memset(frame, 0, sizeof(cec_frame_t));
                    }

                    block = 0;
                }
                else if (cec_state == CEC_DATA_ACK)
                {
                    if (debug) ESP_LOGI(TAG, "%s data %02x eom %d ack %d", CEC_STATE_NAMES[cec_state], CEC_BLOCK_DATA(block), CEC_BLOCK_EOM(block), CEC_BLOCK_ACK(block));

                    if (!frame->bit_error || CEC_ACK_OK(frame->dest_addr == CEC_LA_BROADCAST, frame->ack))
                    {
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
            else if (!debug)
            {
                if (cec_state == CEC_HEAD_ACK || cec_state == CEC_DATA_ACK)
                {
                    int8_t la = atomic_load(&cec_log_addr);
                    //int8_t src = cec_state == CEC_DATA_ACK ? frame->src_addr : (block >> 5) & 0xf;
                    int8_t dest = cec_state == CEC_DATA_ACK ? frame->dest_addr : (block >> 1) & 0xf;
                    if (la == dest)
                    {
                        if (!frame->bit_error)
                        {
                            atomic_store(&cec_ack_set, true);
                            gpio_set_level(HDMI_CEC_GPIO_NUM, 0);        
                            esp_err_t result = esp_timer_start_once(cec_ack_timer_handle, CEC_DATA0_TIME);
                            if (result != ESP_OK)
                            {
                                ESP_DRAM_LOGE(TAG, "ack_timer esp_timer_start_once failed (%s)", esp_err_to_name(result));
                            }

                            //ESP_DRAM_LOGI(TAG, "ack set (%lld us low %lld us) block %02x la %d src %d dest %d", time_us, last_low_time_us,  block, la, src, dest);
                        }
                        else
                        {
                            //ESP_DRAM_LOGI(TAG, "ack not set due to bit error (%lld us) block %02x la %d src %d dest %d", time_us, block, la, src, dest);
                        }
                    }
                    else
                    {
                        //ESP_DRAM_LOGI(TAG, "ack not set (%lld us) block %02x la %d src %d dest %d", time_us, block, la, src, dest);
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
    xQueueReset(cec_frame_queue_handle);

    return ESP_OK;
}

esp_err_t cec_standby()
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = CEC_LA_BROADCAST;
    frame.data_size = 1;
    frame.data[0] = CEC_OP_STANDBY;

    return cec_frame_queue_add(&frame);
}

esp_err_t cec_user_control_pressed(cec_logical_address_t log_addr, cec_user_control_code_t ucc)
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = log_addr;
    frame.data_size = 3;
    frame.data[0] = CEC_OP_USER_CONTROL_PRESSED;
    frame.data[1] = ucc;

    return cec_frame_queue_add(&frame);

    return ESP_OK;
}

esp_err_t cec_log_write(httpd_req_t* request)
{
    taskENTER_CRITICAL(&cec_mux);

    cec_log_entry_t* entry = cec_log_head;
    cec_log_entry_t* copy_head = NULL;
    cec_log_entry_t* copy_tail = NULL;
    int log_size = cec_log_size;
    while (entry != NULL)
    {
        cec_log_entry_t* copy = calloc(1, sizeof(cec_log_entry_t));
        if (copy != NULL)
        {
            memcpy(&copy->frame, &entry->frame, sizeof(cec_frame_t));
        }
        else
        {
            ESP_LOGE(TAG, "out of memory while cloning log");
            break;
        }
        if (copy_tail != NULL)
        {
            copy_tail->next = copy;
        }
        else
        {
            copy_head = copy;
        }
        copy_tail = copy;
        entry = entry->next;
    }

    taskEXIT_CRITICAL(&cec_mux);
    ESP_LOGI(TAG, "log size %d", log_size);

    entry = copy_head;
    while (entry != NULL)
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%cx (%d): ", entry->frame.type == CEC_FRAME_RX ? 'r' : entry->frame.type == CEC_FRAME_TX ? 't' : 'u' , 1 + entry->frame.data_size);
        httpd_resp_send_chunk(request, buf, HTTPD_RESP_USE_STRLEN);

        snprintf(buf, sizeof(buf), "%1x%1x", entry->frame.src_addr, entry->frame.dest_addr);
        httpd_resp_send_chunk(request, buf, 2);

        for (int i = 0; i < entry->frame.data_size; i++)
        {
            snprintf(buf, sizeof(buf), ":%02x", entry->frame.data[i]);
            httpd_resp_send_chunk(request, buf, 3);
        }

        if (!CEC_ACK_OK(entry->frame.dest_addr == CEC_LA_BROADCAST, entry->frame.ack))
        {
            httpd_resp_send_chunk(request, " !", 2);
        }
        httpd_resp_send_chunk(request, "\n", 1);

        entry = entry->next;
    }

    httpd_resp_send_chunk(request, "END", 3);
    httpd_resp_send_chunk(request, NULL, 0);

    entry = copy_head;
    while (entry != NULL)
    {
        cec_log_entry_t* next = entry->next;
        free(entry);

        entry = next;
    }

    return ESP_OK;
}

esp_err_t cec_log_clear()
{
    taskENTER_CRITICAL(&cec_mux);

    cec_log_entry_t* entry = cec_log_head;
    while (entry != NULL)
    {
        cec_log_entry_t* next = entry->next;
        free(entry);
        entry = next;
    }
    cec_log_size = 0;
    cec_log_head = NULL;
    cec_log_tail = NULL;
    taskEXIT_CRITICAL(&cec_mux);

    return ESP_OK;
}

esp_err_t cec_control(cec_logical_address_t addr, const uint8_t* request, int request_size, cec_op_code_t reply_op_code, uint8_t* reply, int* reply_size)
{
    if (reply == NULL && reply_size != NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!xSemaphoreTake(cec_control_request_sem, CEC_CONTROL_REQUEST_TIMEOUT / portTICK_PERIOD_MS))
    {
        return ESP_ERR_TIMEOUT;
    }

    taskENTER_CRITICAL(&cec_control_response_mux);
    xSemaphoreTake(cec_control_response_sem, 0);
    cec_control_reply_from = addr;
    cec_control_reply_size = 1;
    cec_control_reply[0] = reply_op_code;
    taskEXIT_CRITICAL(&cec_control_response_mux);

    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = addr;
    frame.data_size = request_size;
    memcpy(&frame.data, request, request_size);

    esp_err_t result = cec_frame_queue_add(&frame);
    if (result != ESP_OK) return result;

    ESP_LOGI(TAG, "wait %02x", reply_op_code);

    if (xSemaphoreTake(cec_control_response_sem, CEC_CONTROL_RESPONSE_TIMEOUT / portTICK_PERIOD_MS))
    {
        int copy_size = cec_control_reply_size <= *reply_size ? cec_control_reply_size : *reply_size;
        if (copy_size != cec_control_reply_size)
        {
            result = ESP_ERR_INVALID_SIZE;
        }
        if (reply_size != NULL)
        {
            *reply_size = cec_control_reply_size;
        }
        if (reply != NULL)
        {
            memcpy(reply, cec_control_reply, copy_size);
        }
    }
    else
    {
        result = ESP_ERR_TIMEOUT;
    }

    taskENTER_CRITICAL(&cec_control_response_mux);
    cec_control_reply_size = 0;
    taskEXIT_CRITICAL(&cec_control_response_mux);

    xSemaphoreGive(cec_control_request_sem);

    return result;
}

esp_err_t cec_system_audio_mode_request(uint16_t phy_addr)
{
    uint8_t request[3];
    uint8_t reply[2];
    int reply_size = sizeof(reply);

    request[0] = CEC_OP_SYSTEM_AUDIO_MODE_REQUEST;
    request[1] = (phy_addr >> 8) & 0xff;
    request[2] = phy_addr & 0xff;

    esp_err_t result = cec_control(CEC_LA_AUDIO_SYSTEM, request, sizeof(request), CEC_OP_SET_SYSTEM_AUDIO_MODE, reply, &reply_size);
    if (result != ESP_OK) return result;

    return reply_size == 2 && reply[1] == 0x01 ? ESP_OK : ESP_FAIL;
}

esp_err_t cec_active_source(uint16_t phy_addr)
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = CEC_LA_BROADCAST;
    frame.data_size = 3;
    frame.data[0] = CEC_OP_ACTIVE_SOURCE;
    frame.data[1] = (phy_addr >> 8) & 0xff;
    frame.data[2] = phy_addr & 0xff;

    return cec_frame_queue_add(&frame);
}

esp_err_t cec_image_view_on(cec_logical_address_t addr)
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = addr;
    frame.data_size = 1;
    frame.data[0] = CEC_OP_IMAGE_VIEW_ON;

    return cec_frame_queue_add(&frame);
}

esp_err_t cec_power_status(cec_logical_address_t log_addr, cec_power_status_t* power_status, bool use_cache)
{
    *power_status = CEC_PS_UNKNOWN;

    if (log_addr > CEC_LA_MAX)
    {
        ESP_LOGE(TAG, "invalid power status query for la %d", log_addr);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (use_cache)
    {
        taskENTER_CRITICAL(&cec_mux);
        *power_status = cec_power_statuses[log_addr];
        taskEXIT_CRITICAL(&cec_mux);

        ESP_LOGI(TAG, "la %d cached power state %d", log_addr, *power_status);

        if (*power_status != CEC_PS_UNKNOWN)
        {
            return ESP_OK;
        }
    }

    uint8_t request[1];
    uint8_t reply[2];
    int reply_size = sizeof(reply);

    request[0] = CEC_OP_GIVE_DEVICE_POWER_STATUS;

    esp_err_t result = cec_control(log_addr, request, sizeof(request), CEC_OP_REPORT_POWER_STATUS, reply, &reply_size);
    if (result != ESP_OK) return result;
    if (reply_size != 2) return ESP_FAIL;

    *power_status = reply[1];

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

                    ESP_LOGI(TAG, "cec phy address: %u.%u.%u.%u", a0, a1, a2, a3);
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

    ESP_LOGI(TAG, "%s%s (%d) %01x%01x%s %s%s", frame->type == CEC_FRAME_RX ? "rx" : "tx", debug ? "d" : "", 
        1 + frame->data_size, frame->src_addr, frame->dest_addr, data_string, 
        CEC_ACK_OK(frame->dest_addr == CEC_LA_BROADCAST, frame->ack) ? "" : "!", frame->bit_error ? "*" : "");

    taskENTER_CRITICAL(&cec_mux);
    cec_log_entry_t* entry = calloc(1, sizeof(cec_log_entry_t));
    memcpy(&entry->frame, frame, sizeof(cec_frame_t));
    if (cec_log_size >= CEC_MAX_LOG_ENTRIES)
    {
        cec_log_entry_t* old_head = cec_log_head;
        cec_log_head = old_head->next;
        if (cec_log_tail == old_head)
        {
            cec_log_tail = cec_log_head;
        }
        free(old_head);
        cec_log_size--;
    }
    if (cec_log_tail != NULL)
    {
        cec_log_tail->next = entry;
    }
    else
    {            
        cec_log_head = entry;
    }
    cec_log_tail = entry;
    cec_log_size++;
    taskEXIT_CRITICAL(&cec_mux);

    if (frame->type == CEC_FRAME_RX)
    {
        taskENTER_CRITICAL(&cec_control_response_mux);
        if (cec_control_reply_size == 1)
        {
            // ESP_DRAM_LOGI(TAG, "src_addr %d ack ok %d from %d size %d op %02x reply %02x", 
            //     frame->src_addr, 
            //     CEC_ACK_OK(frame->dest_addr == CEC_LA_BROADCAST, frame->ack), 
            //     cec_control_reply_from, frame->data_size, cec_control_reply[0], 
            //     frame->data_size >= 1 ? frame->data[0] : -1);
            if (CEC_ACK_OK(frame->dest_addr == CEC_LA_BROADCAST, frame->ack) &&
                (cec_control_reply_from == CEC_LA_BROADCAST || cec_control_reply_from == frame->src_addr) &&
                frame->data_size >= 1 && 
                frame->data[0] == cec_control_reply[0])
            {
                memcpy(cec_control_reply, frame->data, frame->data_size);
                cec_control_reply_size = frame->data_size;

                xSemaphoreGive(cec_control_response_sem);
            }
        }
        taskEXIT_CRITICAL(&cec_control_response_mux);
    }

    if (frame->data_size == 0)
    {
        return;
    }
  
    if (frame->dest_addr == atomic_load(&cec_log_addr) || frame->dest_addr == CEC_LA_BROADCAST)
    {
        switch (frame->data[0])
        {
            case CEC_OP_GIVE_PHYSICAL_ADDRESS:
                cec_report_physical_address();
                break;
        }
    }

    switch (frame->data[0])
    {
        case CEC_OP_ACTIVE_SOURCE:
            if (frame->data_size == 3)
            {
                uint16_t active_source = frame->data[1] << 8 | frame->data[2];

                if (cec_active_source_address != 0xffff && active_source != cec_active_source_address && frame->src_addr == CEC_LA_TV)
                {
                    ESP_LOGI(TAG, "restoring TV stolen active source " CEC_AS_FORMAT, CEC_AS_ARGS(cec_active_source_address));
                    cec_active_source(cec_active_source_address);
                }
                else if (cec_active_source_address != active_source)
                {
                    ESP_LOGI(TAG, "saving active source " CEC_AS_FORMAT " from %d", CEC_AS_ARGS(active_source), frame->src_addr);
                    cec_active_source_address = active_source;
                }
            }
            break;
        case CEC_OP_REPORT_POWER_STATUS:
            if (frame->data_size == 2)
            {
                taskENTER_CRITICAL(&cec_mux);
                cec_power_statuses[frame->src_addr] = frame->data[1];
                taskEXIT_CRITICAL(&cec_mux);
                ESP_LOGI(TAG, "la %d power_state %d", frame->src_addr, frame->data[1]);
            }
            break;
    }


}

static void cec_line_wait_free(int bit_periods)
{
    int64_t now = esp_timer_get_time();
    while (esp_timer_get_time() - now < bit_periods * CEC_BIT_TIME)
    {
        if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
        {
            portENABLE_INTERRUPTS();
            taskYIELD();            
            portDISABLE_INTERRUPTS();
            now = esp_timer_get_time();
        }
    }
}

static bool cec_transmit_start()
{
    if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    {
        return false;
    }
    int64_t bit_start = esp_timer_get_time();

    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    delay_us(CEC_START0_TIME);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
    delay_until(bit_start + CEC_START_TIME);

    return true;
}

static bool  cec_frame_transmit_ack(bool* ack)
{
    if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    {
        return false;
    }

    int64_t bit_start = esp_timer_get_time();
    atomic_store(&cec_ack_set, 0);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    delay_us(CEC_DATA1_TIME);
    if (atomic_load(&cec_ack_set) == 0)
    {
        gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
    }
    delay_until(bit_start + CEC_BIT_TIME_SAMPLE);
    *ack = gpio_get_level(HDMI_CEC_GPIO_NUM) == 0;
    delay_until(bit_start + CEC_BIT_TIME);

    return true;
}

static bool  cec_frame_transmit_bit(bool bit_value)
{
    if (gpio_get_level(HDMI_CEC_GPIO_NUM) == 0)
    {
        return false;
    }

    int64_t bit_start = esp_timer_get_time();
    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    delay_us(bit_value ? CEC_DATA1_TIME : CEC_DATA0_TIME);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
    delay_until(bit_start + CEC_BIT_TIME);

    return true;
}

static bool cec_frame_transmit_byte(uint8_t data, bool eom, bool* ack)
{
    *ack = false;

    uint8_t shift = 7;

    for (int i = 0; i < 8; i++)
    {
        //ESP_LOGI(TAG, "bit shift %d value %d", shift, (data >> shift) & 1);
        if (!cec_frame_transmit_bit((data >> shift) & 1))
        {
            return false;
        }
        shift--;
    }
    if (!cec_frame_transmit_bit(eom))
    {
        return false;
    }
    if (!cec_frame_transmit_ack(ack))
    {
        return false;
    }
    return true;
}

static void cec_line_error_set()
{
    ESP_DRAM_LOGE(TAG, "line error");
    atomic_store(&cec_line_error, true);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 0);
    delay_us(CEC_LINE_ERROR_TIME);
    gpio_set_level(HDMI_CEC_GPIO_NUM, 1);
}

static esp_err_t cec_frame_transmit(cec_frame_t* frame)
{
    if (xPortGetCoreID() != APP_CPU_NUM)
    {
        ESP_LOGE(TAG, "cec_frame_transmit can only be called from APP_CPU");
        return ESP_ERR_INVALID_STATE;
    }

    bool broadcast = frame->dest_addr == CEC_LA_BROADCAST;

    if (frame->type != CEC_FRAME_TX ||
        (frame->src_addr != atomic_load(&cec_log_addr) && frame->src_addr != frame->dest_addr && !broadcast))
    {
        return ESP_ERR_INVALID_ARG;
    }

    int retry = 0;
    bool line_error;
    int8_t wait = atomic_load(&cec_last_source) == frame->src_addr ? CEC_WAIT_CONTINUE : CEC_WAIT_NEW;
            
    portDISABLE_INTERRUPTS();
    do
    {
        line_error = false;
        frame->ack = false;

        cec_line_wait_free(wait);

        if (!cec_transmit_start())
        {
            cec_line_error_set();
            line_error = true;
            continue;
        }
        uint8_t addr = (frame->src_addr << 4) | frame->dest_addr;

        if (!cec_frame_transmit_byte(addr, frame->data_size == 0, &frame->ack))
        {
            cec_line_error_set();
            line_error = true;
            continue;
        }
        for (int i = 0; CEC_ACK_OK(broadcast, frame->ack) && i < frame->data_size; i++)
        {
            if (!cec_frame_transmit_byte(frame->data[i], i == frame->data_size - 1, &frame->ack))
            {
                cec_line_error_set();
                line_error = true;
                continue;
            }
        }

        wait = CEC_WAIT_RETRY;
    }
    while ((retry++ < CEC_FRAME_RETRY_MAX && !CEC_ACK_OK(broadcast, frame->ack)) || line_error);
    portENABLE_INTERRUPTS();

    //ESP_LOGI(TAG, "exit ack %d", frame->ack);

    return CEC_ACK_OK(broadcast, frame->ack) ? ESP_OK : broadcast ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT;
}

static esp_err_t cec_frame_queue_add(cec_frame_t* frame)
{
    return xQueueSend(cec_frame_queue_handle, frame, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t cec_report_physical_address()
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = CEC_LA_BROADCAST;
    frame.data_size = 1;
    frame.data[0] = CEC_OP_REPORT_PHYSICAL_ADDRESS;
    frame.data[1] = (cec_phy_addr >> 8) & 0xff;
    frame.data[2] = cec_phy_addr & 0xff;
    frame.data[3] = CEC_DT_TUNER;

    return cec_frame_queue_add(&frame);
}

esp_err_t cec_test()
{
    cec_power_status_t ps;
    esp_err_t result =  cec_power_status(CEC_LA_AUDIO_SYSTEM, &ps, true);

    ESP_LOGI(TAG, "cec_test result %s ps %d", esp_err_to_name(result), ps);

    return result;
}

esp_err_t cec_test2()
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = CEC_LA_TV;
    frame.data_size = 1;
    frame.data[0] = CEC_OP_GIVE_PHYSICAL_ADDRESS;

    return cec_frame_queue_add(&frame);
}

esp_err_t cec_test3()
{
    cec_frame_t frame = { 0 };

    frame.type = CEC_FRAME_TX;
    frame.src_addr = atomic_load(&cec_log_addr);
    frame.dest_addr = CEC_LA_AUDIO_SYSTEM;
    frame.data_size = 1;
    frame.data[0] = CEC_OP_REQUEST_ACTIVE_SOURCE;

    return cec_frame_queue_add(&frame);
}
