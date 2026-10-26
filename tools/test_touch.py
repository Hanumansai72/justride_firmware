"""Compile the production touch driver and gesture FSM against host SPI stubs.

Requires Windows MSVC Build Tools. No hardware access, firmware flashing or
hardware latency claims. Generated headers/binaries stay under build/touch-tests.
"""
from pathlib import Path
import argparse
import os
import subprocess
import ctypes

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build' / 'touch-tests'
parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, default=ROOT / 'main' / 'display')
args = parser.parse_args()
OUT.mkdir(parents=True, exist_ok=True)
(OUT / 'driver').mkdir(exist_ok=True)

headers = {
    'stdint.h': '#pragma once\ntypedef unsigned char uint8_t; typedef short int16_t; typedef unsigned short uint16_t; typedef unsigned int uint32_t; typedef unsigned long long uint64_t;\n',
    'stddef.h': '#pragma once\ntypedef unsigned long long size_t;\n#define NULL ((void*)0)\n',
    'stdbool.h': '#pragma once\n#define bool _Bool\n#define true 1\n#define false 0\n',
    'stdlib.h': '#pragma once\nstatic int abs(int n){return n<0?-n:n;}\n',
    'string.h': '#pragma once\n#include <stddef.h>\nstatic void *memset(void *p,int c,size_t n){unsigned char *b=p;for(size_t i=0;i<n;i++)b[i]=(unsigned char)c;return p;}\n',
    'assert.h': '#pragma once\nextern int test_assert_failures;\n#define assert(e) ((e)?(void)0:(void)(test_assert_failures++))\n',
    'esp_err.h': '''#pragma once
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 257
static inline const char *esp_err_to_name(int e) { (void)e; return "stub"; }
''',
    'esp_log.h': '''#pragma once
#define ESP_LOGD(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_LOGE(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) ((void)(tag))
''',
    'driver/gpio.h': '''#pragma once
#include <stdint.h>
#define GPIO_MODE_INPUT 0
#define GPIO_PULLUP_ENABLE 1
#define GPIO_INTR_DISABLE 0
typedef struct { uint64_t pin_bit_mask; int mode,pull_up_en,pull_down_en,intr_type; } gpio_config_t;
int gpio_config(const gpio_config_t *config);
''',
    'driver/spi_master.h': '''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void *spi_device_handle_t;
typedef int spi_host_device_t;
#define SPI2_HOST 2
#define SPI_TRANS_USE_TXDATA 1
#define SPI_TRANS_USE_RXDATA 2
typedef struct { unsigned flags; size_t length,rxlength; union { const void *tx_buffer; uint8_t tx_data[4]; }; union { void *rx_buffer; uint8_t rx_data[4]; }; } spi_transaction_t;
typedef struct { int clock_speed_hz,mode,spics_io_num,queue_size; void *pre_cb,*post_cb; } spi_device_interface_config_t;
int spi_device_polling_transmit(spi_device_handle_t device, spi_transaction_t *transaction);
int spi_bus_add_device(int host, const spi_device_interface_config_t *config, spi_device_handle_t *device);
int spi_bus_remove_device(spi_device_handle_t device);
''',
}
for name, text in headers.items():
    (OUT / name).write_text(text)

harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "touch.c"
int test_assert_failures, _fltused;
__declspec(dllexport) int failed_lines[1024];
static int checks,failures,reads,fail_at,locks,unlocks,removed,gpio_error;
static unsigned raw_x=2000,raw_y=2000,z1=500,z2=1000;
#define CHECK(expr) do { checks++; if(!(expr)){ failed_lines[failures++]=__LINE__; } } while(0)
void tft_spi_lock(void){locks++;}
void tft_spi_unlock(void){unlocks++;}
int tft_get_spi_host(void){return SPI2_HOST;}
int gpio_config(const gpio_config_t *config){(void)config;return gpio_error;}
int spi_bus_add_device(int host,const spi_device_interface_config_t *c,spi_device_handle_t *d){
 (void)host;assert(c->clock_speed_hz==2000000 && c->mode==0);*d=(void*)1;return 0;
}
int spi_bus_remove_device(spi_device_handle_t d){(void)d;removed++;return 0;}
int spi_device_polling_transmit(spi_device_handle_t d,spi_transaction_t *t){
 (void)d;reads++;assert(t->length==24);
 if(reads==fail_at)return ESP_FAIL;
 const uint8_t *tx=(t->flags&SPI_TRANS_USE_TXDATA)?t->tx_data:t->tx_buffer;
 uint8_t *rx=(t->flags&SPI_TRANS_USE_RXDATA)?t->rx_data:t->rx_buffer;
 unsigned v=tx[0]==0xD0?raw_x:tx[0]==0x90?raw_y:tx[0]==0xB0?z1:z2;
 assert(tx[0]==0xD0||tx[0]==0x90||tx[0]==0xB0||tx[0]==0xC0);
 rx[0]=0;rx[1]=(uint8_t)(v>>5);rx[2]=(uint8_t)(v<<3);return 0;
}
static void reset(void){reads=locks=unlocks=fail_at=0;raw_x=raw_y=2000;z1=500;z2=1000;}
static void driver_checks(void){
 touch_point_t p={.is_pressed=true,.pressure=123};
 CHECK(!touch_get_point(NULL));
 CHECK(!touch_get_point(&p));CHECK(!p.is_pressed && p.pressure==0);
 reset();CHECK(touch_init()==ESP_OK);CHECK(locks==unlocks);
 reset();CHECK(touch_get_point(&p));CHECK(p.is_pressed && p.pressure==500);CHECK(reads==12);
 // Inject failure at every channel read. No ADC error may become a valid point.
 for(int i=1;i<=12;i++){
   reset();fail_at=i;p.is_pressed=true;p.pressure=123;
   CHECK(!touch_get_point(&p));CHECK(!p.is_pressed && p.pressure==0);CHECK(locks==unlocks);
 }
 reset();z1=TOUCH_PRESSURE_THRESHOLD;CHECK(!touch_get_point(&p));CHECK(reads==2);
 CHECK(!p.is_pressed && p.pressure==0);
 reset();z2=4000;CHECK(!touch_get_point(&p));CHECK(!p.is_pressed && p.pressure==0);
 reset();raw_x=0;CHECK(!touch_get_point(&p));CHECK(!p.is_pressed && p.pressure==0);
 reset();raw_x=TOUCH_RAW_MIN_X;CHECK(!touch_get_point(&p));CHECK(!p.is_pressed && p.pressure==0);
 // Check the existing calibrated transfer curve at two unequal interior points.
 for(int i=0;i<2;i++){
   reset();raw_x=i?3600:800;raw_y=i?3500:1200;
   int px=(int)((float)((int)raw_x-TOUCH_RAW_MIN_X)*240.0f/(TOUCH_RAW_MAX_X-TOUCH_RAW_MIN_X));
   int py=(int)((float)((int)raw_y-TOUCH_RAW_MIN_Y)*320.0f/(TOUCH_RAW_MAX_Y-TOUCH_RAW_MIN_Y));
   CHECK(touch_get_point(&p));
#if DISPLAY_ROTATION==TFT_ROTATION_LANDSCAPE
   CHECK(p.x==319-py && p.y==px);
#elif DISPLAY_ROTATION==TFT_ROTATION_LANDSCAPE_FLIP
   CHECK(p.x==py && p.y==239-px);
#elif DISPLAY_ROTATION==TFT_ROTATION_PORTRAIT_FLIP
   CHECK(p.x==239-px && p.y==319-py);
#else
   CHECK(p.x==px && p.y==py);
#endif
 }
 reset();raw_y=3500;CHECK(touch_get_point(&p)); // Portrait bottom remains reachable.
 // Initialization failure must not leave an apparently initialized driver.
 s_touch_spi=NULL;reset();fail_at=2;CHECK(touch_init()!=ESP_OK);CHECK(s_touch_spi==NULL);
 reset();CHECK(touch_init()==ESP_OK);
}

// Replay real main.c FSM logic using deterministic frames (20ms per poll).
typedef uint32_t TickType_t;
typedef enum { TOUCH_GESTURE_TAP=0,TOUCH_GESTURE_SWIPE_RIGHT,TOUCH_GESTURE_SWIPE_LEFT,TOUCH_GESTURE_SWIPE_DOWN,TOUCH_GESTURE_SWIPE_UP } touch_gesture_t;
static touch_point_t frames[32];
static int frame_count,frame_index,event_count,last_gesture;
#define portTICK_PERIOD_MS 10
#define pdMS_TO_TICKS(ms) ((ms)/portTICK_PERIOD_MS)
static uint32_t xTaskGetTickCount(void){return (uint32_t)frame_index*2;}
static bool fake_point(touch_point_t *p){*p=frames[frame_index];return p->is_pressed;}
static void vTaskDelay(uint32_t ticks){assert(ticks==2);frame_index++;}
static void display_handle_touch_event(int16_t sx,int16_t sy,int16_t ex,int16_t ey,touch_gesture_t g){
 (void)sx;(void)sy;(void)ex;(void)ey;event_count++;last_gesture=g;
}
#define touch_get_point fake_point
FSM_SOURCE
#undef touch_get_point
static touch_point_t down(int x,int y){touch_point_t p={(int16_t)x,(int16_t)y,500,true};return p;}
static void replay(int count,int expected_count,int expected_gesture){
 frame_count=count;frame_index=event_count=0;last_gesture=-1;
 touch_task(NULL);
 CHECK(event_count==expected_count);if(expected_count)CHECK(last_gesture==expected_gesture);
 memset(frames,0,sizeof(frames));
}
static void gesture_checks(void){
 frames[0]=down(100,100);replay(5,1,TOUCH_GESTURE_TAP);
 frames[0]=down(100,100);frames[4]=down(110,100);replay(9,2,TOUCH_GESTURE_TAP);
 for(int i=0;i<26;i++)frames[i]=down(100,100);replay(30,1,TOUCH_GESTURE_TAP); // Existing long-hold behavior.
 frames[0]=down(100,100);frames[1]=down(104,102);frames[2]=down(98,99);replay(7,1,TOUCH_GESTURE_TAP);
 frames[0]=down(100,100);frames[1]=down(125,101);frames[2]=down(160,100);replay(7,1,TOUCH_GESTURE_SWIPE_RIGHT);
 frames[0]=down(160,100);frames[1]=down(120,101);frames[2]=down(100,100);replay(7,1,TOUCH_GESTURE_SWIPE_LEFT);
 frames[0]=down(100,50);frames[1]=down(101,100);replay(6,1,TOUCH_GESTURE_SWIPE_DOWN);
 frames[0]=down(100,100);frames[1]=down(101,50);replay(6,1,TOUCH_GESTURE_SWIPE_UP);
 frames[0]=down(100,100);frames[1]=down(110,110);frames[4]=down(170,101);replay(9,1,TOUCH_GESTURE_SWIPE_RIGHT);
 frames[0]=down(100,100);frames[1]=down(160,160);replay(6,0,0);
 frames[0]=down(100,100);frames[1]=down(160,100);frames[2]=down(100,100);replay(7,0,0);
 replay(32,0,0); // 640ms synthetic idle; not a real ghost-touch trial.
}
__declspec(dllexport) int run_touch_tests(void){driver_checks();gesture_checks();return failures+test_assert_failures;}
__declspec(dllexport) int check_count(void){return checks;}
'''
main = (ROOT / 'main/main.c').read_text(encoding='utf-8')
fsm = main[main.index('#define SWIPE_MIN_PX'):main.index('void app_main(void)')]
# Bound only the task lifetime in the host harness; leave its FSM unchanged.
fsm = fsm.replace('while (1)', 'while (frame_index < frame_count)')
harness = harness.replace('FSM_SOURCE', fsm)
(OUT / 'driver_tests.c').write_text(harness, encoding='utf-8')
(OUT / 'touch.h').write_text((args.source / 'touch.h').read_text(encoding='utf-8'), encoding='utf-8')
(OUT / 'touch.c').write_text((args.source / 'touch.c').read_text(encoding='utf-8'), encoding='utf-8')
compilers = sorted(Path('C:/Program Files (x86)/Microsoft Visual Studio').glob(
    '*/BuildTools/VC/Tools/MSVC/*/bin/Hostx64/x64/cl.exe'))
compiler = Path(os.environ['MSVC_CL']) if 'MSVC_CL' in os.environ else (compilers[-1] if compilers else None)
if not compiler or not compiler.exists():
    raise SystemExit('Set MSVC_CL to your x64 MSVC cl.exe path.')
environment = dict(os.environ, PATH=str(compiler.parent) + os.pathsep + os.environ['PATH'])
failed = False
for rotation in range(4):
    header = (ROOT / 'main/display/display_tft.h').read_text(encoding='utf-8')
    header = header.replace('#define DISPLAY_ROTATION    TFT_ROTATION_LANDSCAPE', f'#define DISPLAY_ROTATION    {rotation}')
    (OUT / 'display_tft.h').write_text(header, encoding='utf-8')
    library = OUT / f'touch-tests-{rotation}.dll'
    # No Windows SDK dependency: only test declarations and no CRT calls.
    subprocess.run([str(compiler), '/nologo', '/W3', '/std:c11', '/utf-8', '/GS-',
                    '/Zl', '/LD', '/I.', 'driver_tests.c', f'/Fe:{library}',
                    '/link', '/noentry', '/nodefaultlib'], cwd=OUT, env=environment, check=True)
    dll = ctypes.CDLL(str(library))
    failures = dll.run_touch_tests()
    lines = (ctypes.c_int * 1024).in_dll(dll, 'failed_lines')
    print(f'rotation={rotation} checks={dll.check_count()} failures={failures}', flush=True)
    for line in sorted(set(lines) - {0}):
        print(f'  failed driver_tests.c:{line}: {harness.splitlines()[line-1].strip()}', flush=True)
    failed |= failures != 0
raise SystemExit(1 if failed else 0)
