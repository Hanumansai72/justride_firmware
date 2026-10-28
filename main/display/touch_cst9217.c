// CST9217 protocol verified against SensorLib TouchDrvCST92xx (Lewis He):
// https://github.com/lewisxhe/SensorLib/blob/master/src/touch/TouchDrvCST92xx.cpp
// D000 report, D000AB acknowledge, D204 chip identity, D109 normal mode.
#include "touch.h"
#include "round_panel.h"
#include "round_geometry.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TP_BUS I2C_NUM_1 // Separate from MPU6500 on I2C0 / GPIO21,22.
#define TP_ADDR 0x5A
static bool ready;
static esp_err_t write_bytes(const uint8_t *p,size_t n) {
    return i2c_master_write_to_device(TP_BUS,TP_ADDR,p,n,pdMS_TO_TICKS(20));
}
static esp_err_t read_reg(uint16_t reg,uint8_t *p,size_t n) {
    uint8_t address[]={reg>>8,reg};
    return i2c_master_write_read_device(TP_BUS,TP_ADDR,address,2,p,n,pdMS_TO_TICKS(20));
}
esp_err_t touch_init(void) {
    if(ready)return ESP_OK;
    gpio_config_t rst={.pin_bit_mask=1ULL<<CONFIG_JR_TOUCH_RST,.mode=GPIO_MODE_OUTPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&rst),"CST9217","reset GPIO");
    gpio_config_t irq={.pin_bit_mask=1ULL<<CONFIG_JR_TOUCH_IRQ,.mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&irq),"CST9217","IRQ GPIO");
    i2c_config_t bus={.mode=I2C_MODE_MASTER,.sda_io_num=CONFIG_JR_TOUCH_SDA,
        .scl_io_num=CONFIG_JR_TOUCH_SCL,.sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,.master.clk_speed=100000};
    ESP_RETURN_ON_ERROR(i2c_param_config(TP_BUS,&bus),"CST9217","I2C config");
    ESP_RETURN_ON_ERROR(i2c_driver_install(TP_BUS,I2C_MODE_MASTER,0,0,0),"CST9217","I2C install");
    gpio_set_level(CONFIG_JR_TOUCH_RST,0);vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CONFIG_JR_TOUCH_RST,1);vTaskDelay(pdMS_TO_TICKS(50));
    const uint8_t debug[]={0xD1,0x01}, normal[]={0xD1,0x09};
    uint8_t id[4]={0};
    ESP_RETURN_ON_ERROR(write_bytes(debug,2),"CST9217","debug mode");
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_err_t err=read_reg(0xD204,id,4);
    esp_err_t exit_err=write_bytes(normal,2);
    if(err!=ESP_OK)return err;
    if(exit_err!=ESP_OK)return exit_err;
    if(((uint16_t)id[3]<<8|id[2])!=0x9217) {
        ESP_LOGE("CST9217","Unexpected chip ID %02x%02x",id[3],id[2]);
        return ESP_ERR_NOT_SUPPORTED;
    }
    vTaskDelay(pdMS_TO_TICKS(10));ready=true;return ESP_OK;
}
bool touch_get_point(touch_point_t *point) {
    if(!point)return false;
    memset(point,0,sizeof(*point));
    if(!ready)return false;
    uint8_t report[15]={0};
    if(read_reg(0xD000,report,sizeof(report))!=ESP_OK)return false;
    const uint8_t ack[]={0xD0,0x00,0xAB};
    if(write_bytes(ack,3)!=ESP_OK)return false;
    int x,y;
    if(!cst9217_decode(report,&x,&y))return false;
#if CONFIG_JR_TOUCH_SWAP_XY
    int swap=x;x=y;y=swap;
#endif
#if CONFIG_JR_TOUCH_MIRROR_X
    x=465-x;
#endif
#if CONFIG_JR_TOUCH_MIRROR_Y
    y=465-y;
#endif
    if(!round_panel_touch(x,y,&point->x,&point->y))return false;
    point->is_pressed=true;point->pressure=1;return true;
}
