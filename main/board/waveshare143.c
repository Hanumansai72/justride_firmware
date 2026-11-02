#include "sdkconfig.h"
#if CONFIG_JUSTRIDE_ROUND_AMOLED
#include "waveshare143.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
static bool ready;
esp_err_t waveshare143_i2c_init(void) {
    if(ready)return ESP_OK;
    i2c_config_t c={.mode=I2C_MODE_MASTER,.sda_io_num=WS143_I2C_SDA,
        .scl_io_num=WS143_I2C_SCL,.sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,.master.clk_speed=400000};
    esp_err_t e=i2c_param_config(I2C_NUM_0,&c);
    if(e==ESP_OK)e=i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0);
    ready=e==ESP_OK;return e;
}
esp_err_t waveshare143_read(uint8_t address,uint8_t reg,void *data,size_t len) {
    if(!ready || !data || !len)return ESP_ERR_INVALID_STATE;
    return i2c_master_write_read_device(I2C_NUM_0,address,&reg,1,data,len,pdMS_TO_TICKS(25));
}
esp_err_t waveshare143_write(uint8_t address,uint8_t reg,uint8_t value) {
    if(!ready)return ESP_ERR_INVALID_STATE;
    uint8_t bytes[]={reg,value};
    return i2c_master_write_to_device(I2C_NUM_0,address,bytes,2,pdMS_TO_TICKS(25));
}
#endif
