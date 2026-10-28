#include "sdkconfig.h"
#if CONFIG_JUSTRIDE_ROUND_AMOLED
#include "round_panel.h"
#include "round_geometry.h"
#include "display_tft.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static spi_device_handle_t panel;
static bool full_round;
static esp_err_t transfer_error;
static int wx, wy, ww, wh, row, column;
static uint8_t logical_row[320*2];
static DMA_ATTR uint8_t expanded_row[466*2*2];
static void transaction(uint8_t opcode, uint8_t reg, const void *bytes,
                        size_t length, bool quad) {
    spi_transaction_t t={.cmd=opcode,.addr=(uint32_t)reg<<8,
        .length=length*8,.tx_buffer=bytes,.flags=quad?SPI_TRANS_MODE_QIO:0};
    esp_err_t err=spi_device_polling_transmit(panel,&t);
    if(err!=ESP_OK && transfer_error==ESP_OK) {
        transfer_error=err;
        ESP_LOGE("CO5300","QSPI transfer failed: %s",esp_err_to_name(err));
    }
}
static void command(uint8_t reg,const void *bytes,size_t length) {
    transaction(0x02,reg,bytes,length,false);
}
static void native_window(int x,int y,int w,int h) {
    uint8_t a[]={x>>8,x,(x+w-1)>>8,x+w-1}; command(0x2A,a,4);
    uint8_t b[]={y>>8,y,(y+h-1)>>8,y+h-1}; command(0x2B,b,4);
}
void round_panel_window(int x,int y,int w,int h) {
    wx=x;wy=y;ww=w;wh=h;row=column=0;
}
void round_panel_pixels(const uint8_t *data,size_t length) {
    if(!data || (length&1) || ww<1 || ww>320 || wh<1) return;
    while(length && row<wh) {
        size_t n=(ww-column)*2;
        if(n>length)n=length;
        memcpy(logical_row+column*2,data,n);
        column+=n/2;data+=n;length-=n;
        if(column!=ww)continue;
        int left=round_map_x(wx,full_round), right=round_map_x(wx+ww,full_round);
        int top=round_map_y(wy+row,full_round), bottom=round_map_y(wy+row+1,full_round);
        int width=right-left, height=bottom-top;
        if(width<=0 || width>466 || height<1 || height>2)return;
        for(int x=0;x<ww;x++) {
            int a=round_map_x(wx+x,full_round)-left;
            int b=round_map_x(wx+x+1,full_round)-left;
            for(int xx=a;xx<b;xx++) {
                expanded_row[xx*2]=logical_row[x*2];
                expanded_row[xx*2+1]=logical_row[x*2+1];
            }
        }
        if(height==2)memcpy(expanded_row+width*2,expanded_row,width*2);
        native_window(left,top,width,height);
        // 0x32: instruction+24-bit address single lane, pixels on four lanes.
        transaction(0x32,0x2C,expanded_row,width*height*2,true);
        row++;column=0;
    }
}
void round_panel_brightness(uint8_t percent) {
    if(!panel)return;
    uint8_t value=(percent>100?100:percent)*255/100;
    tft_spi_lock();command(0x51,&value,1);tft_spi_unlock();
}
void round_panel_canvas(bool round) {
    if(full_round==round)return;
    full_round=round;
    // Clear unused margins only on a layout transition; live updates use strips.
    memset(expanded_row,0,466*2);
    for(int y=0;y<466;y++) {
        native_window(0,y,466,1);transaction(0x32,0x2C,expanded_row,466*2,true);
    }
}
bool round_panel_is_round(void){return full_round;}
bool round_panel_touch(int raw_x,int raw_y,int16_t *x,int16_t *y) {
    return round_unmap_touch(raw_x,raw_y,full_round,x,y);
}
esp_err_t round_panel_init(void) {
    gpio_config_t io={.pin_bit_mask=1ULL<<CONFIG_JR_PANEL_RST,.mode=GPIO_MODE_OUTPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&io),"CO5300","reset GPIO");
    gpio_set_level(CONFIG_JR_PANEL_RST,0);vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(CONFIG_JR_PANEL_RST,1);vTaskDelay(pdMS_TO_TICKS(120));
    spi_bus_config_t bus={.mosi_io_num=CONFIG_JR_QSPI_D0,.miso_io_num=CONFIG_JR_QSPI_D1,
        .quadwp_io_num=CONFIG_JR_QSPI_D2,.quadhd_io_num=CONFIG_JR_QSPI_D3,
        .sclk_io_num=CONFIG_JR_QSPI_CLK,.max_transfer_sz=466*4};
    esp_err_t err=spi_bus_initialize(TFT_SPI_HOST,&bus,SPI_DMA_CH_AUTO);
    if(err!=ESP_OK)return err;
    spi_device_interface_config_t device={.command_bits=8,.address_bits=24,
        .clock_speed_hz=40*1000*1000,.mode=0,.spics_io_num=CONFIG_JR_QSPI_CS,
        .queue_size=1,.flags=SPI_DEVICE_HALFDUPLEX};
    err=spi_bus_add_device(TFT_SPI_HOST,&device,&panel);if(err!=ESP_OK)return err;
    // CO5300 DCS startup; module-specific vendor tuning is not supplied in
    // the module specification's empty initialization-code section.
    command(0x01,NULL,0);vTaskDelay(pdMS_TO_TICKS(120));
    round_panel_brightness(0);
    uint8_t rgb565=0x55, orientation=0, control=0x20;
    command(0x3A,&rgb565,1);command(0x36,&orientation,1);command(0x53,&control,1);
    command(0x11,NULL,0);vTaskDelay(pdMS_TO_TICKS(120));
    full_round=true;round_panel_canvas(false);
    command(0x29,NULL,0);vTaskDelay(pdMS_TO_TICKS(20));
    return transfer_error;
}
#endif
