#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/periph_ctrl.h"
#include "rom/ets_sys.h"
#include "ddc_int.h"
#include "ddc.h"

#define TAG "DDC"

esp_err_t ddc_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = DDC_SDA_GPIO_NUM;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = DDC_SCL_GPIO_NUM;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    //conf.slave.addr_10bit_en = 0;
    //conf.slave.slave_addr = DDC_I2C_ADDRESS;
    conf.master.clk_speed = DDC_MASTER_FREQ_HZ;

    ESP_RETURN_ON_ERROR(i2c_param_config(DDC_I2C_PORT, &conf), TAG, "DDC config failed (%s)", esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(i2c_driver_install(DDC_I2C_PORT, conf.mode, DDC_MASTER_RX_BUF_DISABLE, DDC_MASTER_TX_BUF_DISABLE, 0), 
        TAG, "DDC driver install failed (%s)", esp_err_to_name(err_rc_));

    return ESP_OK;
}

esp_err_t ddc_reset()
{
    ESP_RETURN_ON_ERROR(i2c_reset_tx_fifo(DDC_I2C_PORT), TAG, "i2c_reset_tx_fifo failed (%s)", esp_err_to_name(err_rc_));
	ESP_RETURN_ON_ERROR(i2c_reset_rx_fifo(DDC_I2C_PORT), TAG, "i2c_reset_tx_fifo i2c_reset_rx_fifo (%s)", esp_err_to_name(err_rc_));
	periph_module_disable(PERIPH_I2C0_MODULE);
    vTaskDelay(100 / portTICK_PERIOD_MS);
	periph_module_enable(PERIPH_I2C0_MODULE);
	ESP_RETURN_ON_ERROR(i2c_driver_delete(DDC_I2C_PORT), TAG, "i2c_driver_delete failed (%s)", esp_err_to_name(err_rc_));

    vTaskDelay(100 / portTICK_PERIOD_MS);
	
    return ddc_init();
}

esp_err_t i2c_read_byte(uint8_t address, uint8_t offset, uint8_t* value)
{
    //esp_err_t ret = ESP_OK;

    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    *value = 0;

    //uint8_t cmd_buffer[2];

    //cmd_buffer[0] = (address << 1) | I2C_MASTER_WRITE;
    //cmd_buffer[1] = offset;
    // //uint8_t checksum = request_buffer[0];
    // // for (int i = 1; i < sizeof(request_buffer) - 1; i++)
    // // {
    // //     checksum ^= request_buffer[i];
    // // }
    // // request_buffer[2] = checksum;

    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // ESP_GOTO_ON_ERROR(i2c_master_start(cmd),
    //     cleanup, TAG, "%s/i2c_master_start failed (%s)", __func__, esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, request_buffer[0], true),
    //     cleanup, TAG, "%s/get_vcp write i2c_master_write_byte failed (%s)", __func__, esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(%s/i2c_master_write(cmd, request_buffer + 1, sizeof(request_buffer) - 1, true),
    //     cleanup, TAG, "%s/get_vcp write i2c_master_write failed (%s)", __func__, esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(%s/i2c_master_stop(cmd), cleanup, TAG, "%s/get_vcp write i2c_master_stop failed (%s)", esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(i2c_master_cmd_begin(DDC_I2C_PORT, cmd, DDC_I2C_TIMEOUT), 
    //     cleanup, TAG, "%s/get_vcp write i2c_master_cmd_begin failed (%s)", __func__, esp_err_to_name(err_rc_));
    // i2c_cmd_link_delete(cmd);
    // cmd = NULL;

    // ets_delay_us(40000);

    // memset(cmd_buffer, 0, sizeof(response_buffer));
    // cmd_buffer[0] = (address << 1) | I2C_MASTER_READ;

    // cmd = i2c_cmd_link_create();
    // ESP_GOTO_ON_ERROR(i2c_master_start(cmd), 
    //     cleanup, TAG, "%s/i2c_master_start failed (%s)", esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, cmd_buffer[0], true), 
    //     cleanup, TAG, "%s/i2c_master_write_byte failed (%s)", __func__, esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(i2c_master_read(cmd, value, 1, I2C_MASTER_LAST_NACK), 
    //     cleanup, TAG, "%s/i2c_master_read failed (%s)", __func__, esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(i2c_master_stop(cmd), cleanup, TAG, "get_vcp read i2c_master_stop failed (%s)", __func__, esp_err_to_name(err_rc_));
    // ESP_GOTO_ON_ERROR(i2c_master_cmd_begin(DDC_I2C_PORT, cmd, DDC_I2C_TIMEOUT), 
    //     cleanup, TAG, "%s/i2c_master_cmd_begin failed (%s)", __func__, esp_err_to_name(err_rc_));
    // i2c_cmd_link_delete(cmd);
    // cmd = NULL;

    // vTaskDelay(30 / portTICK_PERIOD_MS);

    return i2c_master_write_read_device(DDC_I2C_PORT, address, &offset, 1, value, 1, DDC_I2C_TIMEOUT);
}

esp_err_t ddc_get_vcp_internal(uint8_t op, uint16_t* value)
{
    esp_err_t ret = ESP_OK;

    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    *value = 0;

    uint8_t request_buffer[6];

    request_buffer[0] = (DDC_I2C_ADDRESS << 1) | I2C_MASTER_WRITE;
    request_buffer[1] = 0x51;
    request_buffer[2] = 0x82;
    request_buffer[3] = 0x01;
    request_buffer[4] = op;
    uint8_t checksum = request_buffer[0];
    for (int i = 1; i < sizeof(request_buffer) - 1; i++)
    {
        checksum ^= request_buffer[i];
    }
    request_buffer[5] = checksum;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_GOTO_ON_ERROR(i2c_master_start(cmd),
        cleanup, TAG, "get_vcp write i2c_master_start failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, request_buffer[0], true),
        cleanup, TAG, "get_vcp write i2c_master_write_byte failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_write(cmd, request_buffer + 1, sizeof(request_buffer) - 1, true),
        cleanup, TAG, "get_vcp write i2c_master_write failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_stop(cmd), cleanup, TAG, "get_vcp write i2c_master_stop failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_cmd_begin(DDC_I2C_PORT, cmd, DDC_I2C_TIMEOUT), 
        cleanup, TAG, "get_vcp write i2c_master_cmd_begin failed (%s)", esp_err_to_name(err_rc_));
    i2c_cmd_link_delete(cmd);
    cmd = NULL;

    ets_delay_us(40000);

    uint8_t response_buffer[12];

    memset(response_buffer, 0, sizeof(response_buffer));
    response_buffer[0] = (DDC_I2C_ADDRESS << 1) | I2C_MASTER_READ;

    cmd = i2c_cmd_link_create();
    ESP_GOTO_ON_ERROR(i2c_master_start(cmd), 
        cleanup, TAG, "get_vcp read i2c_master_start failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_write_byte(cmd, response_buffer[0], true), 
        cleanup, TAG, "get_vcp read i2c_master_write_byte failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_read(cmd, response_buffer + 1, sizeof(response_buffer) - 1, I2C_MASTER_LAST_NACK), 
        cleanup, TAG, "get_vcp read i2c_master_read failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_stop(cmd), cleanup, TAG, "get_vcp read i2c_master_stop failed (%s)", esp_err_to_name(err_rc_));
    ESP_GOTO_ON_ERROR(i2c_master_cmd_begin(DDC_I2C_PORT, cmd, DDC_I2C_TIMEOUT), 
        cleanup, TAG, "get_vcp read i2c_master_cmd_begin failed (%s)", esp_err_to_name(err_rc_));
    i2c_cmd_link_delete(cmd);
    cmd = NULL;

    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("response:");
    for (int i = 0; i < sizeof(response_buffer); i++)
    {
        printf(" %02x", response_buffer[i]);
    }
    printf("\n");

    *value = response_buffer[9] << 8 | response_buffer[10];

cleanup:
    if (cmd != NULL)
    {
        i2c_cmd_link_delete(cmd);
        cmd = NULL;
    }
    return ret;
}

esp_err_t ddc_get_vcp(uint8_t op, uint16_t* value)
{
    esp_err_t result = ESP_FAIL;
    for (int i = 1; i <= 10; i++)
    {
        result = ddc_get_vcp_internal(op, value);
        if (result == ESP_OK)
        {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "ddc_get_vcp error %s, retry #%d", esp_err_to_name(result), i);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(ddc_reset());
    }
    return result;
}
