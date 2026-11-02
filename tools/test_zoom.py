"""Host-check actual map receiver/camera code and render Android fixture packets.
Requires Pillow + MSVC Build Tools. No device, credentials or downloaded maps.
Run the app's test/touch_zoom_test.dart with JR_ZOOM_FIXTURE_OUT pointing to
build/zoom-tests/packets.json first to generate the comparison image.
"""
from pathlib import Path
import ctypes
import json
import os
import subprocess
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'build/zoom-tests';OUT.mkdir(parents=True,exist_ok=True)
INC=OUT/'include';INC.mkdir(exist_ok=True)
headers={
 'stdint.h':'typedef unsigned char uint8_t;typedef signed char int8_t;typedef unsigned short uint16_t;typedef short int16_t;typedef unsigned int uint32_t;typedef int int32_t;typedef unsigned long long uint64_t;typedef long long int64_t;',
 'stddef.h':'typedef unsigned long long size_t;\n#define NULL ((void*)0)',
 'stdbool.h':'#define bool _Bool\n#define true 1\n#define false 0',
 'stdlib.h':'int abs(int);void abort(void);',
 'assert.h':'#include <stdlib.h>\n#define assert(x) ((x)?(void)0:abort())',
 'ctype.h':'int toupper(int);',
 'string.h':'#include <stddef.h>\nvoid *memset(void*,int,size_t);void *memcpy(void*,const void*,size_t);int memcmp(const void*,const void*,size_t);size_t strlen(const char*);char *strcpy(char*,const char*);char *strstr(const char*,const char*);int strcmp(const char*,const char*);',
 'stdio.h':'#include <stddef.h>\ntypedef void FILE;int _snprintf(char*,size_t,const char*,...);\n#define snprintf _snprintf\nFILE *fopen(const char*,const char*);size_t fwrite(const void*,size_t,size_t,FILE*);int fclose(FILE*);',
 'math.h':'float cosf(float);float sinf(float);float fminf(float,float);float fmaxf(float,float);long lroundf(float);float roundf(float);\n#define isfinite(x) ((x)==(x))',
 'esp_log.h':'#define ESP_LOGI(...) ((void)0)',
 'esp_heap_caps.h':'#include <stddef.h>\n#define MALLOC_CAP_8BIT 1\nsize_t heap_caps_get_free_size(int);void *heap_caps_malloc(size_t,int);',
 'esp_timer.h':'#include <stdint.h>\nint64_t esp_timer_get_time(void);',
 'esp_rom_crc.h':'#include <stdint.h>\n#include <stddef.h>\nuint32_t esp_rom_crc32_le(uint32_t,const uint8_t*,size_t);',
}
for name,text in headers.items():(INC/name).write_text('#pragma once\n'+text+'\n')
tft=(ROOT/'main/display/display_tft.h').read_text(encoding='utf-8')
enum=tft[tft.index('typedef enum {'):tft.index('} nav_direction_t;')+len('} nav_direction_t;')]
apis=tft[tft.index('void tft_fill_screen'):tft.index('spi_host_device_t tft_get_spi_host')]
driver='#pragma once\n#include <stdint.h>\n#include <stddef.h>\n#include <stdbool.h>\n#include "display_theme.h"\n'+enum+'\n#define COLOR_SURFACE_DARK 0\n#define COLOR_WHITE JR_TEXT\n'+apis
(OUT/'display_tft.h').write_text(driver,encoding='utf-8');(OUT/'hud_capture_driver.h').write_text(driver,encoding='utf-8')
source=(ROOT/'main/display/display_tft.c').read_text(encoding='utf-8')
font=source[source.index('static const uint8_t font5x8'):source.index('};',source.index('static const uint8_t font5x8'))+2]
primitives=source[source.index('// One reusable 5 KiB strip'):]
raster=r'''
#include "display_tft.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define DMA_ATTR
typedef void *TaskHandle_t;
static TaskHandle_t xTaskGetCurrentTaskHandle(void){return (void*)1;}
static int s_width=320,s_height=240;
static uint8_t framebuffer[320*240*2];
static unsigned cursor,x0,y0,x1,y1;
void tft_spi_lock(void){} void tft_spi_unlock(void){}
static void tft_set_addr_window(unsigned a,unsigned b,unsigned c,unsigned d){
 assert(a<=c && b<=d && c<320 && d<240);x0=a;y0=b;x1=c;y1=d;cursor=0;}
static void tft_send_data(const uint8_t *p,size_t n){
 assert(!(n&1));for(size_t i=0;i<n;i+=2){unsigned x=x0+cursor%(x1-x0+1),y=y0+cursor/(x1-x0+1);
 assert(x<=x1 && y<=y1);framebuffer[(y*320+x)*2]=p[i];framebuffer[(y*320+x)*2+1]=p[i+1];cursor++;}}
void capture_save(const char *path){FILE *f=fopen(path,"wb");assert(f);fwrite(framebuffer,1,sizeof(framebuffer),f);fclose(f);}
'''
(OUT/'raster.c').write_text(raster+font+'\n'+primitives,encoding='utf-8')
display=(ROOT/'main/display/display.c').read_text(encoding='utf-8')
display_header=(ROOT/'main/display/display.h').read_text(encoding='utf-8')
segment=display_header[display_header.index('#define ROUTE_MAX_POINTS'):display_header.index('typedef struct {\n    char city')]
camera_enums=display_header[display_header.index('typedef enum {\n    NAV_CAMERA_FOLLOW'):display_header.index('} nav_zoom_level_t;')+len('} nav_zoom_level_t;')]
arrow=display[display.index('static void draw_hud_rider_chevron'):display.index('// ── 5. LARGE')]
route=display[display.index('typedef struct { int x, y, w, h; } route_region_t;'):display.index('static void draw_hud_road_corridor')]
camera=display[display.index('static void nav_zoom_changed'):display.index('// LATEST NOTIFICATION APIS')]
fixture=r'''
#include "hud_ui.h"
#include "display_tft.h"
#include <math.h>
#include <string.h>
#include <assert.h>
int _fltused=0;
void capture_save(const char*);
static int64_t now_us;
int64_t esp_timer_get_time(void){return now_us;}
static uint8_t memory[2][226*177*2];static unsigned allocations;
static bool fail_alloc;static size_t free_bytes=1000000;
size_t heap_caps_get_free_size(int caps){(void)caps;return free_bytes;}
void *heap_caps_malloc(size_t n,int caps){(void)caps;assert(n<=sizeof(memory[0]));
 if(fail_alloc)return NULL;assert(allocations<2);return memory[allocations++];}
uint32_t esp_rom_crc32_le(uint32_t crc,const uint8_t *p,size_t n){crc=~crc;
 for(size_t i=0;i<n;i++){crc^=p[i];for(int bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1)?0xEDB88320:0);}return ~crc;}
#include "map_frame.c"
static void start_packet(uint8_t *p,unsigned id,uint16_t color){
 p[0]=0x14;p[1]=id;p[2]=id>>8;p[3]=226;p[4]=0;p[5]=177;p[6]=0;
 static uint8_t raw[226*177*2];for(unsigned i=0;i<sizeof(raw);i+=2){raw[i]=color>>8;raw[i+1]=color;}
 uint32_t crc=esp_rom_crc32_le(0,raw,sizeof(raw));for(int i=0;i<4;i++)p[7+i]=crc>>(8*i);}
static void transfer(unsigned id,uint16_t color){
 unsigned left=226*177;uint16_t seq=0;
 while(left){unsigned count=left>255?255:left;
 uint8_t p[]={0x15,id,id>>8,seq,seq>>8,count,color>>8,color};
 assert(map_frame_packet(p,sizeof(p)));assert(map_frame_packet(p,sizeof(p)));
 left-=count;seq++;}
 uint8_t commit[]={0x16,id,id>>8};assert(map_frame_packet(commit,3));assert(map_frame_packet(commit,3));}
__declspec(dllexport) int receiver_checks(void){
 uint8_t p[11];start_packet(p,65534,0x1234);
 fail_alloc=true;assert(!map_frame_packet(p,11));assert(!valid);fail_alloc=false;
 assert(map_frame_packet(p,11));assert(!map_frame_active());transfer(65534,0x1234);assert(valid && pixels[0]==0x12);
 map_frame_draw(true);uint8_t *front=pixels;
 start_packet(p,65535,0x5678);free_bytes=1;assert(!map_frame_packet(p,11));assert(valid && pixels==front);free_bytes=1000000;
 assert(map_frame_packet(p,11));uint8_t data[]={0x15,255,255,0,0,1,0x56,0x78};
 assert(map_frame_packet(data,8));assert(pixels==front && pixels[0]==0x12);
 map_frame_restore_region(110,60,10,10); // Front buffer stays readable during transfer.
 uint8_t badcommit[]={0x16,255,255};assert(!map_frame_packet(badcommit,3));assert(valid && pixels==front);
 assert(map_frame_packet(p,11)); // Same failed frame may be retried, not mistaken for a stale committed frame.
 start_packet(p,0,0x5678);assert(map_frame_packet(p,11));transfer(0,0x5678);assert(frame_id==0 && pixels[0]==0x56);
 start_packet(p,65535,0x9999);assert(!map_frame_packet(p,11)); // Old frame cannot replace new.
 start_packet(p,1,0x9999);assert(map_frame_packet(p,11));now_us+=10000001;map_frame_draw(false);assert(!receiving && valid);
 assert(map_frame_packet(p,11)); // A timed-out transfer can restart with its existing ID.
 map_frame_reset_session();start_packet(p,0,0xABCD);assert(map_frame_packet(p,11));transfer(0,0xABCD);
 assert(pixels[0]==0xAB && allocations==2); // New peer, same ID, genuinely new pixels.
 map_frame_invalidate();assert(!map_frame_active());return 0;
}
SEGMENT
CAMERA_ENUMS
static route_segment_t s_route_seg;
static nav_zoom_level_t s_zoom_level=NAV_ZOOM_LEVEL_2;
static nav_camera_mode_t s_camera_mode=NAV_CAMERA_FOLLOW;
static bool s_zoom_pending,s_route_seg_dirty,s_is_initialized=true;
static int64_t s_zoom_due_us;
static int s_current_screen=1,s_pan_offset_x,s_pan_offset_y,s_drawn_motion_degrees;
#define SCREEN_NAV 1
#define TAG "host"
#define COLOR_NAV_ROUTE JR_PRIMARY
#define COLOR_NAV_ROUTE_HALO JR_PRIMARY_DARK
static float rider_motion_degrees(void){return 0;}
ARROW
ROUTE
CAMERA
__declspec(dllexport) int camera_checks(void){
 now_us=100;display_nav_zoom_out();assert(s_zoom_due_us==250100);
 now_us=100000;display_nav_zoom_out();assert(s_zoom_due_us==350000);
 for(int i=0;i<20;i++)display_nav_zoom_in();assert(s_zoom_level==1);
 s_zoom_pending=false;display_nav_zoom_in();assert(!s_zoom_pending);
 for(int i=0;i<20;i++)display_nav_zoom_out();assert(s_zoom_level==5 && s_camera_mode==NAV_CAMERA_OVERVIEW);
 s_zoom_pending=false;display_nav_zoom_out();assert(!s_zoom_pending);
 display_nav_zoom_in();assert(s_zoom_level==4 && s_camera_mode==NAV_CAMERA_FOLLOW);
 display_nav_recenter();assert(s_zoom_level==2 && s_pan_offset_x==0 && s_pan_offset_y==0);
 display_nav_toggle_overview();assert(s_zoom_level==5);
 display_nav_toggle_overview();assert(s_zoom_level==2);return 0;
}
__declspec(dllexport) int render_packet(const uint8_t *p,unsigned n,const char *path){
 assert(p[0]==0x0B && p[1]<=ROUTE_MAX_POINTS && n>=8+p[1]*4);
 memset(&s_route_seg,0,sizeof(s_route_seg));s_route_seg.count=p[1];
 s_route_seg.is_destination_visible=p[2]&1;s_route_seg.overview=p[2]&0x20;s_route_seg.camera_projected=p[2]&0x40;
 s_route_seg.completed_segment_count=(p[2]>>1)&15;s_route_seg.dest_x=p[3]|p[4]<<8;s_route_seg.dest_y=p[5]|p[6]<<8;
 for(unsigned i=0;i<p[1];i++){s_route_seg.points[i].x=p[7+i*4]|p[8+i*4]<<8;s_route_seg.points[i].y=p[9+i*4]|p[10+i*4]<<8;}
 tft_fill_screen(HUD_BG);paint_route_region(HUD_MAP_X,HUD_MAP_Y,HUD_MAP_W,HUD_MAP_H);
 hud_model_t m={.speed=42,.speed_valid=true,.heading=0,.direction=5,.distance_m=180,.remaining_km=1.2,.duration="4 min",.instruction="TURN RIGHT"};
 hud_speed(&m);hud_poi(&m,0);hud_poi(&m,1);hud_trip(&m);hud_compass(0);hud_maneuver(&m);
 capture_save(path);return 0;
}
'''
for key,value in [('SEGMENT',segment),('CAMERA_ENUMS',camera_enums),('ARROW',arrow),('ROUTE',route),('CAMERA',camera)]:fixture=fixture.replace('\n'+key+'\n','\n'+value+'\n')
(OUT/'fixture.c').write_text(fixture,encoding='utf-8')
(OUT/'map_frame.c').write_text((ROOT/'main/display/map_frame.c').read_text())
exports='cosf sinf fminf fmaxf lroundf roundf abs abort memset memcpy memcmp strlen strcpy strstr strcmp toupper fopen fwrite fclose'.split()
(OUT/'ucrt.def').write_text('LIBRARY ucrtbase.dll\nEXPORTS\n'+'\n'.join(exports)+'\n')
(OUT/'msvcrt.def').write_text('LIBRARY msvcrt.dll\nEXPORTS\n_snprintf\n')
compilers=sorted(Path('C:/Program Files (x86)/Microsoft Visual Studio').glob('*/BuildTools/VC/Tools/MSVC/*/bin/Hostx64/x64/cl.exe'))
if not compilers:raise SystemExit('MSVC Build Tools required')
compiler=compilers[-1];env=dict(os.environ,PATH=str(compiler.parent)+os.pathsep+os.environ['PATH'])
for name in ['ucrt','msvcrt']:subprocess.run([str(compiler.parent/'lib.exe'),'/nologo',f'/def:{name}.def','/machine:x64',f'/out:{name}.lib'],cwd=OUT,env=env,check=True)
subprocess.run([str(compiler),'/nologo','/std:c11','/utf-8','/GS-','/Zl','/LD','/DHUD_CAPTURE','/Iinclude','/I.',f'/I{ROOT}/main/display',str(ROOT/'main/display/hud_ui.c'),'raster.c','fixture.c','/Fezoom.dll','/link','/noentry','/nodefaultlib','ucrt.lib','msvcrt.lib'],cwd=OUT,env=env,check=True)
dll=ctypes.CDLL(str(OUT/'zoom.dll'));assert dll.receiver_checks()==0;assert dll.camera_checks()==0
if (OUT/'packets.json').exists():
    packets=json.loads((OUT/'packets.json').read_text());sheet=Image.new('RGB',(1480,430),'#0b100d');draw=ImageDraw.Draw(sheet)
    for i,(packet,label) in enumerate(zip(packets,['DETAILED / 150 m','INTERMEDIATE / 700 m','REMAINING ROUTE OVERVIEW'])):
        path=OUT/f'zoom-{i}.rgb565';data=(ctypes.c_ubyte*len(packet))(*packet)
        assert dll.render_packet(data,len(packet),str(path).encode())==0
        raw=path.read_bytes();rgb=[]
        for j in range(0,len(raw),2):
            c=raw[j]<<8|raw[j+1];rgb.append((((c>>11)&31)*255//31,((c>>5)&63)*255//63,(c&31)*255//31))
        im=Image.new('RGB',(320,240));im.putdata(rgb);im.save(path.with_suffix('.png'))
        draw.text((i*490+12,12),label,fill='white');sheet.paste(im.resize((480,360),Image.Resampling.NEAREST),(i*490+5,40))
    draw.text((12,410),'Actual phone-projected fixture packets + production C renderer. Test coordinates, not live roads/GPS. No invented street tiles.',fill='#aab5ad')
    sheet.save(OUT/'touch-zoom-comparison.png')
print('Production map receiver and zoom-bound checks passed. Preview:',OUT/'touch-zoom-comparison.png')
