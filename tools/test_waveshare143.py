"""Host-check the actual QSPI transport and FT3168 driver with mocked I/O.
No hardware timing, electrical or radio claims. Requires MSVC Build Tools.
"""
from pathlib import Path
import ctypes, os, subprocess
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'build/board-tests';INC=OUT/'include'
(INC/'driver').mkdir(parents=True,exist_ok=True)
(INC/'freertos').mkdir(exist_ok=True)
headers={
'stdint.h':'typedef unsigned char uint8_t;typedef signed char int8_t;typedef unsigned short uint16_t;typedef short int16_t;typedef unsigned int uint32_t;typedef int int32_t;typedef unsigned long long uint64_t;',
'stddef.h':'typedef unsigned long long size_t;\n#define NULL ((void*)0)',
'stdbool.h':'#define bool _Bool\n#define true 1\n#define false 0',
'string.h':'#include <stddef.h>\nvoid *memcpy(void*,const void*,size_t);void *memset(void*,int,size_t);int memcmp(const void*,const void*,size_t);',
'stdlib.h':'void abort(void);',
'assert.h':'#include <stdlib.h>\n#define assert(x) ((x)?(void)0:abort())',
'esp_err.h':'typedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_NO_MEM 1\n#define ESP_ERR_NOT_SUPPORTED 2\n#define ESP_ERR_INVALID_STATE 3\n#define ESP_FAIL 4\n#define ESP_ERR_INVALID_RESPONSE 5\nconst char *esp_err_to_name(int e);',
'esp_log.h':'#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGE(...) ((void)0)',
'esp_check.h':'#define ESP_RETURN_ON_ERROR(e,tag,msg) do{int err=(e);if(err)return err;}while(0)',
'esp_heap_caps.h':'#include <stddef.h>\n#define MALLOC_CAP_SPIRAM 1\n#define MALLOC_CAP_8BIT 2\n#define DMA_ATTR\nvoid *heap_caps_calloc(size_t,size_t,int);',
'esp_rom_sys.h':'void esp_rom_delay_us(unsigned);',
'freertos/FreeRTOS.h':'#define pdMS_TO_TICKS(n) (n)',
'freertos/task.h':'void vTaskDelay(unsigned);',
'sdkconfig.h':'#define CONFIG_JUSTRIDE_ROUND_AMOLED 1\n#define CONFIG_JR_PANEL_AUTO 1',
'driver/gpio.h':'''#include <stdint.h>
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_INPUT 0
#define GPIO_PULLUP_ENABLE 1
typedef struct {uint64_t pin_bit_mask;int mode,pull_up_en;} gpio_config_t;
int gpio_config(const gpio_config_t*);
void gpio_set_level(int,int);
int gpio_get_level(int);
void gpio_set_direction(int,int);
''',
'host_display.h':'''#include <stddef.h>
#define TFT_SPI_HOST 2
#define SPI_DMA_CH_AUTO 0
#define SPI_TRANS_MODE_QIO 1
#define SPI_DEVICE_HALFDUPLEX 2
typedef void *spi_device_handle_t;
typedef struct {int cmd;unsigned addr,length;const void *tx_buffer;unsigned flags;} spi_transaction_t;
typedef struct {int mosi_io_num,miso_io_num,quadwp_io_num,quadhd_io_num,sclk_io_num,max_transfer_sz;} spi_bus_config_t;
typedef struct {int command_bits,address_bits,clock_speed_hz,mode,spics_io_num,queue_size,flags;} spi_device_interface_config_t;
int spi_device_polling_transmit(void*,spi_transaction_t*);
int spi_bus_initialize(int,const spi_bus_config_t*,int);
int spi_bus_add_device(int,const spi_device_interface_config_t*,void**);
void tft_spi_lock(void);
void tft_spi_unlock(void);
''',
}
for name,content in headers.items():(INC/name).write_text('#pragma once\n'+content+'\n')
# Only replace the display include with the mock SPI boundary; compile every
# production transport/driver function unchanged.
panel=(ROOT/'main/display/round_panel.c').read_text(encoding='utf-8-sig').replace('#include "display_tft.h"','#include "host_display.h"')
touch=(ROOT/'main/display/touch_ft3168.c').read_text()
(OUT/'round_panel.c').write_text(panel)
(OUT/'touch_ft3168.c').write_text(touch)
fixture=r'''
#include <assert.h>
#include <string.h>
#include "host_display.h"
#include "waveshare143.h"
#include "round_panel.c"
#include "touch_ft3168.c"
static uint8_t storage[466*466*2],gram[466*466*2],id_value=0x86;
static unsigned id_bit,transfers;
static int x1,x2,y1,y2,power,brightness,format_value;
static uint8_t touch_report[5];
static bool read_fail;
const char *esp_err_to_name(int e){return "";}
void *heap_caps_calloc(size_t n,size_t size,int caps){assert(n*size==sizeof storage);memset(storage,0,sizeof storage);return storage;}
int gpio_config(const gpio_config_t *p){return 0;}
void gpio_set_level(int pin,int value){if(pin==WS143_LCD_EN)power=value;}
int gpio_get_level(int pin){return (id_value>>(7-(id_bit++%8)))&1;}
void gpio_set_direction(int pin,int mode){}
void esp_rom_delay_us(unsigned n){}
void vTaskDelay(unsigned n){}
void tft_spi_lock(void){}
void tft_spi_unlock(void){}
int spi_bus_initialize(int bus,const spi_bus_config_t *p,int dma){
 assert(bus==2 && p->mosi_io_num==11 && p->miso_io_num==12 &&
 p->quadwp_io_num==13 && p->quadhd_io_num==14 && p->sclk_io_num==10);return 0;
}
int spi_bus_add_device(int bus,const spi_device_interface_config_t *p,void **out){
 assert(p->spics_io_num==9 && p->address_bits==24 && p->command_bits==8);*out=(void*)1;return 0;
}
int spi_device_polling_transmit(void *dev,spi_transaction_t *t){
 ++transfers;
 const uint8_t *p=t->tx_buffer;
 unsigned reg=t->addr>>8;
 if(reg==0x51)brightness=p[0];
 if(reg==0x3a)format_value=p[0];
 if(reg==0x2a){x1=(p[0]<<8)|p[1];x2=(p[2]<<8)|p[3];}
 if(reg==0x2b){y1=(p[0]<<8)|p[1];y2=(p[2]<<8)|p[3];}
 if(reg==0x2c){
  assert(t->cmd==0x32 && t->flags==SPI_TRANS_MODE_QIO);
  assert(x1==x_offset && x2==x_offset+465);
  assert(y1>=0 && y2<466 && !(y1&1) && (y2&1));
  assert(t->length==(y2-y1+1)*466*16);
  assert(t->length<=sizeof(dma_rows)*8);
  memcpy(gram+y1*466*2,p,t->length/8);
 }else assert(t->cmd==2);
 return 0;
}
int waveshare143_i2c_init(void){return 0;}
int waveshare143_write(uint8_t addr,uint8_t reg,uint8_t value){assert(addr==0x38 && reg==0 && value==0);return 0;}
int waveshare143_read(uint8_t addr,uint8_t reg,void *p,size_t len){
 assert(addr==0x38 && reg==2 && len==5);if(read_fail)return ESP_FAIL;
 memcpy(p,touch_report,5);return 0;
}
__declspec(dllexport) int run_board_checks(void){
 for(unsigned model=0;model<2;model++){
  id_value=model?0xff:0x86;id_bit=0;panel=NULL;frame=NULL;transfer_error=0;
  dirty_top=466;dirty_bottom=0;full_round=false;
  assert(round_panel_init()==0 && power==1 && format_value==0x55 && brightness==0);
  assert(x_offset==(model?6:0));
  round_panel_brightness(25);assert(brightness==63);
  round_panel_canvas(true);round_panel_flush();
  uint8_t colors[]={0xff,0xff,0xf8,0};
  round_panel_window(160,160,2,1);round_panel_pixels(colors,2);round_panel_pixels(colors+2,2);
  round_panel_flush();
  assert(!memcmp(storage,gram,sizeof storage));
  unsigned before=transfers;
  round_panel_window(160,160,2,1);round_panel_pixels(colors,4);round_panel_flush();
  assert(transfers==before); // unchanged rows must not be flushed
  round_panel_window(-1,0,2,1);round_panel_pixels(colors,4);round_panel_flush();
  assert(transfers==before);
  round_panel_canvas(false);round_panel_flush();
  round_panel_window(0,0,2,1);round_panel_pixels(colors,4);round_panel_flush();
  assert(!memcmp(storage,gram,sizeof storage));
  assert(storage[(101*466+57)*2]==0xff); // legacy pane origin
  assert(storage[0]==0); // unused round margins stay black
 }
 assert(touch_init()==0);
 touch_point_t point={.is_pressed=true};
 memcpy(touch_report,(uint8_t[]){1,0x80,233,0,250},5);
 round_panel_canvas(true);
 assert(touch_get_point(&point) && point.is_pressed && point.x==160 && point.y==171);
 touch_report[1]=0x40;assert(!touch_get_point(&point) && !point.is_pressed);
 touch_report[1]=0x80;read_fail=true;
 assert(!touch_get_point(&point) && !point.is_pressed && point.pressure==0);
 return 0;
}
'''
(OUT/'fixture.c').write_text(fixture)
(OUT/'ucrt.def').write_text('LIBRARY ucrtbase.dll\nEXPORTS\nmemcpy\nmemset\nmemcmp\nabort\n')
compiler=sorted(Path('C:/Program Files (x86)/Microsoft Visual Studio').glob('*/BuildTools/VC/Tools/MSVC/*/bin/Hostx64/x64/cl.exe'))[-1]
env=dict(os.environ,PATH=str(compiler.parent)+os.pathsep+os.environ['PATH'])
subprocess.run([str(compiler.parent/'lib.exe'),'/nologo','/def:ucrt.def','/machine:x64','/out:ucrt.lib'],cwd=OUT,env=env,check=True)
subprocess.run([str(compiler),'/nologo','/std:c11','/GS-','/Zl','/LD','/Iinclude',f'/I{ROOT}/main/display',f'/I{ROOT}/main/board','fixture.c','/Feboard.dll','/link','/noentry','/nodefaultlib','ucrt.lib'],cwd=OUT,env=env,check=True)
dll=ctypes.CDLL(str(OUT/'board.dll'));assert dll.run_board_checks()==0
print('Actual QSPI transport (SH8601/CO5300 offsets, chunking, aligned DMA, dirty rows) and FT3168 driver checks passed.')
