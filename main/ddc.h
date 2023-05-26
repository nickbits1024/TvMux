#ifndef DDC_H
#define DDC_H

#define DDC_EDID_ADDRESS                0x50
#define DDC_EDID_LENGTH                 128
#define DDC_EDID_EXTENSION_LENGTH       128
#define DDC_EDID_EXTENSION_DATA_LENGTH  125
#define DDC_EDID_EXTENSION_FLAG         0x7e

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ddc_init();
esp_err_t ddc_read_byte(uint8_t address, uint8_t offset, uint8_t* value);

#ifdef __cplusplus
}
#endif

#endif