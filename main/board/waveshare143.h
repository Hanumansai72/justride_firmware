#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
// Fixed onboard nets from Waveshare's schematic.
#define WS143_I2C_SDA 47
#define WS143_I2C_SCL 48
#define WS143_LCD_CS 9
#define WS143_LCD_CLK 10
#define WS143_LCD_D0 11
#define WS143_LCD_D1 12
#define WS143_LCD_D2 13
#define WS143_LCD_D3 14
#define WS143_LCD_RST 21
#define WS143_LCD_EN 42
esp_err_t waveshare143_i2c_init(void);
esp_err_t waveshare143_read(uint8_t address,uint8_t reg,void *data,size_t len);
esp_err_t waveshare143_write(uint8_t address,uint8_t reg,uint8_t value);
