#define DDC_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define DDC_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define DDC_MASTER_FREQ_HZ          100000     /*!< I2C master clock frequency */

#define DDC_SCL_GPIO_NUM    (22)
#define DDC_SDA_GPIO_NUM    (21)
#define DDC_I2C_PORT        I2C_NUM_0
#define DDC_I2C_ADDRESS     0x37

#define DDC_I2C_TIMEOUT     (5000 / portTICK_PERIOD_MS)
