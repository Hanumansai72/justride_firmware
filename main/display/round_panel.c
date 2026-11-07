#include "sdkconfig.h"
#if CONFIG_JUSTRIDE_ROUND_AMOLED
#include "round_panel.h"
#include "round_geometry.h"
#include "waveshare143.h"
#include "display_tft.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static spi_device_handle_t panel;
static bool full_round;
static bool display_off;
static esp_err_t transfer_error;
static int wx,wy,ww,wh,row,column,x_offset;
static uint8_t *frame;
static int dirty_top=466,dirty_bottom;
static uint8_t logical_row[640];
static uint8_t previous_row[640];
static uint16_t scale_x[466];
static uint8_t scale_x_weight[466];
static bool scale_x_ready,scale_previous_valid,scale_window;
static int scale_previous_y,scale_next_y,scale_expected_y;
static DMA_ATTR uint8_t dma_rows[466*16*2];
static void transaction(uint8_t opcode,uint8_t reg,const void *bytes,size_t length,bool quad) {
    spi_transaction_t t={.cmd=opcode,.addr=(uint32_t)reg<<8,.length=length*8,
        .tx_buffer=bytes,.flags=quad?SPI_TRANS_MODE_QIO:0};
    esp_err_t err=spi_device_polling_transmit(panel,&t);
    if(err!=ESP_OK && transfer_error==ESP_OK) {
        transfer_error=err;ESP_LOGE("AMOLED143","QSPI: %s",esp_err_to_name(err));
    }
}
static void command(uint8_t reg,const void *p,size_t n){transaction(0x02,reg,p,n,false);}
static void parameter(uint8_t reg,uint8_t p){command(reg,&p,1);}
static void native_window(int y,int height) {
    int end=x_offset+465;
    uint8_t a[]={x_offset>>8,x_offset,end>>8,end};command(0x2A,a,4);
    uint8_t b[]={y>>8,y,(y+height-1)>>8,y+height-1};command(0x2B,b,4);
}
void round_panel_flush(void) {
    if(!panel || !frame || display_off || dirty_top>=dirty_bottom)return;
    // Even-row alignment matches the vendor rounder. PSRAM -> internal DMA.
    int top=dirty_top&~1,bottom=(dirty_bottom+1)&~1;
    for(int y=top;y<bottom;y+=16) {
        int h=bottom-y>16?16:bottom-y;
        memcpy(dma_rows,frame+y*466*2,h*466*2);
        native_window(y,h);transaction(0x32,0x2C,dma_rows,h*466*2,true);
    }
    dirty_top=466;dirty_bottom=0;
}
void round_panel_window(int x,int y,int w,int h){
    wx=x;wy=y;ww=w;wh=h;row=column=0;
    scale_window=full_round && x==0 && w==320;
    if(!scale_window || y==0 || y!=scale_expected_y) {
        scale_previous_valid=false;
        scale_next_y=round_map_y(y,full_round);
    }
    scale_expected_y=y+h;
}
void round_panel_blit_rle(int x,int y,int width,int height,const uint16_t *runs,size_t pairs) {
    if(!frame || !runs || x<0 || y<0 || width<1 || height<1 || x+width>466 || y+height>466)return;
    size_t offset=0,total=(size_t)width*height;
    for(size_t i=0;i<pairs;i++) {
        unsigned count=runs[i*2];uint16_t color=runs[i*2+1];
        if(count>total-offset)return;
        while(count--) {
            size_t at=((y+offset/width)*466+x+offset%width)*2;
            frame[at]=color>>8;frame[at+1]=color;offset++;
        }
    }
    if(y<dirty_top)dirty_top=y;
    if(y+height>dirty_bottom)dirty_bottom=y+height;
}
void round_panel_pixels(const uint8_t *data,size_t length) {
    if(!frame || !data || (length&1) || ww<1 || ww>320 || wh<1 ||
       wx<0 || wy<0 || wx+ww>320 || wy+wh>(full_round?320:240))return;
    if(scale_window && !scale_x_ready) {
        for(int x=0;x<466;x++) {
            int fixed=((2*x+1)*320*128/466)-128;
            if(fixed<0)fixed=0;
            if(fixed>319*256)fixed=319*256;
            scale_x[x]=(uint16_t)(fixed>>8);
            scale_x_weight[x]=(uint8_t)(fixed&255);
        }
        scale_x_ready=true;
    }
    while(length && row<wh) {
        size_t n=(ww-column)*2;if(n>length)n=length;
        memcpy(logical_row+column*2,data,n);column+=n/2;data+=n;length-=n;
        if(column!=ww)continue;
        if(scale_window) {
            int source_y=wy+row;
            if(!scale_previous_valid || scale_previous_y+1!=source_y) {
                memcpy(previous_row,logical_row,ww*2);
                scale_previous_y=source_y;scale_previous_valid=true;
                row++;column=0;
                continue;
            }
            bool changed=false;
            while(scale_next_y<466) {
                int fixed=((2*scale_next_y+1)*320*128/466)-128;
                if(source_y<319 && fixed>source_y*256)break;
                if(fixed<0)fixed=0;
                if(fixed>319*256)fixed=319*256;
                int min_y=scale_previous_y*256,max_y=source_y*256;
                if(fixed<min_y)fixed=min_y;
                if(fixed>max_y)fixed=max_y;
                unsigned fy=(unsigned)(fixed-min_y);
                uint8_t *out=frame+scale_next_y*466*2;
                for(int x=0;x<466;x++) {
                    unsigned x0=scale_x[x],x1=x0<319?x0+1:x0,fx=scale_x_weight[x];
                    unsigned p=x0*2,q=x1*2;
                    uint16_t a=(uint16_t)((previous_row[p]<<8)|previous_row[p+1]);
                    uint16_t b=(uint16_t)((previous_row[q]<<8)|previous_row[q+1]);
                    uint16_t c=(uint16_t)((logical_row[p]<<8)|logical_row[p+1]);
                    uint16_t d=(uint16_t)((logical_row[q]<<8)|logical_row[q+1]);
                    unsigned ar=(((a>>11)&31)*(256-fx)+((b>>11)&31)*fx+128)>>8;
                    unsigned ag=(((a>>5)&63)*(256-fx)+((b>>5)&63)*fx+128)>>8;
                    unsigned ab=((a&31)*(256-fx)+(b&31)*fx+128)>>8;
                    unsigned cr=(((c>>11)&31)*(256-fx)+((d>>11)&31)*fx+128)>>8;
                    unsigned cg=(((c>>5)&63)*(256-fx)+((d>>5)&63)*fx+128)>>8;
                    unsigned cb=((c&31)*(256-fx)+(d&31)*fx+128)>>8;
                    uint16_t color=(uint16_t)(((((ar*(256-fy)+cr*fy+128)>>8)&31)<<11)|
                        ((((ag*(256-fy)+cg*fy+128)>>8)&63)<<5)|
                        (((ab*(256-fy)+cb*fy+128)>>8)&31));
                    size_t at=x*2;
                    if(out[at]!=(uint8_t)(color>>8)||out[at+1]!=(uint8_t)color) {
                        out[at]=(uint8_t)(color>>8);out[at+1]=(uint8_t)color;changed=true;
                    }
                }
                if(changed) {
                    if(scale_next_y<dirty_top)dirty_top=scale_next_y;
                    if(scale_next_y+1>dirty_bottom)dirty_bottom=scale_next_y+1;
                }
                scale_next_y++;
            }
            memcpy(previous_row,logical_row,ww*2);
            scale_previous_y=source_y;
            row++;column=0;
            continue;
        }
        int top=round_map_y(wy+row,full_round),bottom=round_map_y(wy+row+1,full_round);
        bool changed=false;
        for(int y=top;y<bottom;y++)for(int x=0;x<ww;x++) {
            int a=round_map_x(wx+x,full_round),b=round_map_x(wx+x+1,full_round);
            for(int xx=a;xx<b;xx++) {
                uint8_t *p=frame+(y*466+xx)*2;
                if(p[0]!=logical_row[x*2] || p[1]!=logical_row[x*2+1]) {
                    p[0]=logical_row[x*2];p[1]=logical_row[x*2+1];changed=true;
                }
            }
        }
        if(changed){if(top<dirty_top)dirty_top=top;if(bottom>dirty_bottom)dirty_bottom=bottom;}
        row++;column=0;
    }
}
void round_panel_brightness(uint8_t p) {
    if(!panel)return;
    tft_spi_lock();parameter(0x51,(p>100?100:p)*255/100);tft_spi_unlock();
}
void round_panel_standby(bool asleep) {
    if(!panel)return;
    tft_spi_lock();
    if(display_off!=asleep) {
        if(asleep) {
            command(0x28,NULL,0); // DCS display off; retain controller/GRAM and brightness.
            display_off=true;
        } else {
            display_off=false;
            dirty_top=0;dirty_bottom=466;
            round_panel_flush(); // Restore the whole shadow frame before display on.
            command(0x29,NULL,0);
        }
    }
    tft_spi_unlock();
}
void round_panel_canvas(bool round) {
    if(full_round==round)return;
    full_round=round;
    scale_previous_valid=false;scale_expected_y=0;scale_next_y=0;
    if(frame){memset(frame,0,466*466*2);dirty_top=0;dirty_bottom=466;}
}
bool round_panel_is_round(void){return full_round;}
bool round_panel_touch(int raw_x,int raw_y,int16_t *x,int16_t *y) {
    return round_unmap_touch(raw_x,raw_y,full_round,x,y);
}
static void id_byte(uint8_t value) {
    for(int bit=7;bit>=0;bit--) {
        gpio_set_level(WS143_LCD_CLK,0);gpio_set_level(WS143_LCD_D0,(value>>bit)&1);
        esp_rom_delay_us(1);gpio_set_level(WS143_LCD_CLK,1);esp_rom_delay_us(1);
    }
}
static uint8_t read_panel_id(void) {
    // Single-wire 03 00 DA 00 read from the official Waveshare V3 demo.
    gpio_set_level(WS143_LCD_CS,0);
    id_byte(0x03);id_byte(0);id_byte(0xDA);id_byte(0);
    gpio_set_direction(WS143_LCD_D0,GPIO_MODE_INPUT);
    uint8_t id=0;
    for(int bit=0;bit<8;bit++) {
        gpio_set_level(WS143_LCD_CLK,0);esp_rom_delay_us(1);
        id=(id<<1)|gpio_get_level(WS143_LCD_D0);
        gpio_set_level(WS143_LCD_CLK,1);esp_rom_delay_us(1);
    }
    gpio_set_level(WS143_LCD_CS,1);gpio_set_direction(WS143_LCD_D0,GPIO_MODE_OUTPUT);
    return id;
}
esp_err_t round_panel_init(void) {
    gpio_config_t pins={.pin_bit_mask=(1ULL<<WS143_LCD_EN)|(1ULL<<WS143_LCD_RST)|
        (1ULL<<WS143_LCD_CS)|(1ULL<<WS143_LCD_CLK)|(1ULL<<WS143_LCD_D0),
        .mode=GPIO_MODE_OUTPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&pins),"AMOLED143","GPIO");
    gpio_set_level(WS143_LCD_EN,1);gpio_set_level(WS143_LCD_CS,1);
    gpio_set_level(WS143_LCD_RST,0);vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(WS143_LCD_RST,1);vTaskDelay(pdMS_TO_TICKS(120));
    uint8_t id=read_panel_id();
#if CONFIG_JR_PANEL_SH8601
    bool sh=true;
#elif CONFIG_JR_PANEL_CO5300
    bool sh=false;
#else
    if(id!=0x86 && id!=0xff){
        ESP_LOGE("AMOLED143","Unknown ID %02X; select actual panel in menuconfig",id);
        return ESP_ERR_NOT_SUPPORTED;
    }
    bool sh=id==0x86;
#endif
    x_offset=sh?0:6;
    ESP_LOGI("AMOLED143","%s ID=%02X, X offset=%d, 466x466 QSPI",sh?"SH8601":"CO5300",id,x_offset);
    frame=heap_caps_calloc(466*466,2,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if(!frame)return ESP_ERR_NO_MEM;
    spi_bus_config_t bus={.mosi_io_num=WS143_LCD_D0,.miso_io_num=WS143_LCD_D1,
        .quadwp_io_num=WS143_LCD_D2,.quadhd_io_num=WS143_LCD_D3,
        .sclk_io_num=WS143_LCD_CLK,.max_transfer_sz=sizeof(dma_rows)};
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TFT_SPI_HOST,&bus,SPI_DMA_CH_AUTO),"AMOLED143","QSPI bus");
    spi_device_interface_config_t device={.command_bits=8,.address_bits=24,
        .clock_speed_hz=40*1000*1000,.mode=0,.spics_io_num=WS143_LCD_CS,
        .queue_size=1,.flags=SPI_DEVICE_HALFDUPLEX};
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TFT_SPI_HOST,&device,&panel),"AMOLED143","panel");
    parameter(0x3A,0x55);parameter(0x36,0);
    command(0x11,NULL,0);vTaskDelay(pdMS_TO_TICKS(sh?120:80));
    if(sh){uint8_t line[]={1,0xD1};command(0x44,line,2);parameter(0x35,0);}
    else parameter(0xC4,0x80);
    parameter(0x53,0x20);if(!sh)parameter(0x63,0xff);
    parameter(0x51,0);vTaskDelay(pdMS_TO_TICKS(10));
    dirty_top=0;dirty_bottom=466;round_panel_flush();
    command(0x29,NULL,0);vTaskDelay(pdMS_TO_TICKS(10));
    full_round=false;return transfer_error;
}
#endif
