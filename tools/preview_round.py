"""Build the production round UI with a host raster sink; no ESP32 required.

Windows: Python with Pillow, MSVC Build Tools (no Windows SDK needed).
The small import headers are host-only. The labeled initial demo and splash
use the exact production renderer, including the reference-derived roads.
Outputs: build/round-preview/{navigation,splash,clock,music,call,notification}.png.
"""
from pathlib import Path
import argparse
import ctypes
import os
import subprocess
import sys
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--reference',type=Path,help='Optional untouched reference for a side-by-side comparison')
args=parser.parse_args()
OUT=ROOT/'build/round-preview'
OUT.mkdir(parents=True,exist_ok=True)
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
    'math.h':'float cosf(float);float sinf(float);float fminf(float,float);long lroundf(float);float sqrtf(float);float atan2f(float,float);float roundf(float);',
}
for name,text in headers.items():(INC/name).write_text('#pragma once\n'+text+'\n')
header=(ROOT/'main/display/display_tft.h').read_text()
enum=header[header.index('typedef enum {'):header.index('} nav_direction_t;')+len('} nav_direction_t;')]
apis=header[header.index('void tft_fill_screen'):header.index('spi_host_device_t tft_get_spi_host')]
(OUT/'hud_capture_driver.h').write_text('#pragma once\n#include <stdint.h>\n#include <stddef.h>\n#include <stdbool.h>\n'+enum+'\n#define COLOR_SURFACE_DARK 0\n'+apis)
source=(ROOT/'main/display/display_tft.c').read_text()
font=source[source.index('static const uint8_t font5x8'):source.index('};',source.index('static const uint8_t font5x8'))+2]
primitives=source[source.index('// One reusable 5 KiB strip'):]
shim=r'''
#include "hud_capture_driver.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define DMA_ATTR
typedef void *TaskHandle_t;
int _fltused=0;
static TaskHandle_t xTaskGetCurrentTaskHandle(void){return (void*)1;}
static int s_width=320,s_height=320;
static unsigned char framebuffer[320*320*2];
static unsigned cursor,x0,y0,x1,y1;
void tft_spi_lock(void){}
void tft_spi_unlock(void){}
static void tft_set_addr_window(unsigned a,unsigned b,unsigned c,unsigned d){
assert(a<=c && b<=d && c<320 && d<320);x0=a;y0=b;x1=c;y1=d;cursor=0;}
static void tft_send_data(const uint8_t *data,size_t len){
assert((len&1)==0);
for(size_t i=0;i<len;i+=2){unsigned x=x0+cursor%(x1-x0+1),y=y0+cursor/(x1-x0+1);
assert(x<=x1 && y<=y1);framebuffer[(y*320+x)*2]=data[i];framebuffer[(y*320+x)*2+1]=data[i+1];cursor++;}}
void capture_save(const char *path){FILE *f=fopen(path,"wb");assert(f);fwrite(framebuffer,1,sizeof(framebuffer),f);fclose(f);}
const uint8_t *capture_pixels(void){return framebuffer;}
'''
(OUT/'raster.c').write_text(shim+'\n'+font+'\n'+primitives)
fixture=r'''
#include "round_ui.h"
#include "round_geometry.h"
#include "hud_capture_driver.h"
#include <assert.h>
#include <string.h>
void capture_save(const char*);
const uint8_t *capture_pixels(void);
static uint8_t map_fixture[320*344*2];
static uint8_t previous_frame[320*320*2]; // Host-only comparison scratch.
static void reference_map_fixture(void) {
    // Synthetic phone-packet input only for exercising raster-map transforms.
    // Deliberately no generated grid, even in test data.
    tft_fill_screen(JR_RGB565(0x090E0D));
    tft_draw_thick_line(0,241,117,181,10,JR_RGB565(0x424A46));
    tft_draw_thick_line(117,181,86,144,10,JR_RGB565(0x424A46));
    tft_draw_thick_line(86,144,265,69,8,JR_RGB565(0x424A46));
    memcpy(map_fixture,capture_pixels(),320*320*2);
}
__declspec(dllexport) int render_round(void) {
    int16_t x,y;
    assert(JR_RGB565((60u<<16)|(60u<<8)|60u)==JR_RGB565(0x3C3C3C));
    // Boundary rejection, round inverse mapping, legacy-pane inverse mapping.
    assert(!round_unmap_touch(0,0,true,&x,&y));
    assert(!round_unmap_touch(466,233,true,&x,&y));
    for(int py=30;py<290;py+=7)for(int px=30;px<290;px+=7) {
        int dx=px-160,dy=py-160;if(dx*dx+dy*dy>150*150)continue;
        assert(round_unmap_touch(round_map_x(px,true),round_map_y(py,true),true,&x,&y));
        assert(abs(x-px)<=1 && abs(y-py)<=1);
    }
    for(int py=0;py<240;py+=5)for(int px=0;px<320;px+=5) {
        assert(round_unmap_touch(round_map_x(px,false),round_map_y(py,false),false,&x,&y));
        assert(abs(x-px)<=1 && abs(y-py)<=1);
    }
    int raw_x,raw_y;uint8_t p[15]={6,14,15,0x9A,0,1,0xAB};
    assert(cst9217_decode(p,&raw_x,&raw_y) && raw_x==233 && raw_y==250);
    p[0]=0x04;assert(!cst9217_decode(p,&raw_x,&raw_y));
    p[0]=6;p[6]=0;assert(!cst9217_decode(p,&raw_x,&raw_y));
    round_ui_splash(0);capture_save("splash-start.rgb565");
    for(unsigned i=240*320*2;i<320*320*2;i++)assert(capture_pixels()[i]==0);
    memcpy(map_fixture,capture_pixels(),320*320*2);
    round_ui_splash_progress(65);capture_save("splash.rgb565");
    round_ui_splash_progress(100);capture_save("splash-complete.rgb565");
    // Animation must not touch the wordmark, taglines or plain black footer.
    for(int py=0;py<320;py++)for(int px=0;px<320;px++) {
        if(py>=185 && py<189 && px>=65 && px<255)continue;
        unsigned i=(py*320+px)*2;
        assert(map_fixture[i]==capture_pixels()[i] && map_fixture[i+1]==capture_pixels()[i+1]);
    }
    round_ui_navigation_demo();capture_save("navigation.rgb565");
    memcpy(map_fixture,capture_pixels(),320*320*2);
    round_ui_splash(100);round_ui_navigation_demo();
    assert(!memcmp(map_fixture,capture_pixels(),320*320*2));
    // Live route/map fixtures are separate from the production demo.
    reference_map_fixture();
    round_route_point_t points[]={{160,160},{160,142},{160,129},{172,118},{191,106},
        {197,94},{192,84},{177,76},{156,68},{149,60},{155,51},{173,41},{175,33},{164,21},{163,12}};
    round_nav_t v={.nav={.speed=70,.speed_limit=70,.speed_valid=true,.direction=5,.distance_m=300,.heading=0},
        .points=points,.count=15,.zoom_percent=140,.map=map_fixture,.map_w=320,.map_h=344,
        .marker_x=155,.marker_y=211,.map_contains_route=false};
    round_ui_navigation(&v);capture_save("live-vector.rgb565");
    const uint8_t *raster=capture_pixels();
    // Text edges must preserve the map; no opaque black glyph rectangles.
    int non_black_footer=0;
    for(int py=233;py<275;py++)for(int px=115;px<215;px++)
        if(raster[(py*320+px)*2] || raster[(py*320+px)*2+1])non_black_footer++;
    assert(non_black_footer>1500);
    memcpy(previous_frame,capture_pixels(),sizeof(previous_frame));
    v.nav.direction=4;round_ui_navigation(&v);capture_save("navigation-left.rgb565");
    assert(memcmp(previous_frame,capture_pixels(),sizeof(previous_frame)));
    memcpy(previous_frame,capture_pixels(),sizeof(previous_frame));
    v.nav.distance_m=120;round_ui_navigation(&v);capture_save("live-distance-update.rgb565");
    assert(memcmp(previous_frame,capture_pixels(),sizeof(previous_frame)));
    v.nav.distance_m=300;
    v.nav.speed_limit=0;round_ui_navigation(&v);capture_save("unknown-speed-limit.rgb565");
    v.nav.speed_valid=false;strcpy(v.nav.instruction,"GPS LOST");
    v.nav.speed_limit=0;
    round_ui_navigation(&v);capture_save("gps-lost.rgb565");
    // Exercise the production phone-map path, including edge marker crops.
    v.map_contains_route=true;round_ui_navigation(&v);capture_save("phone-map.rgb565");
    v.nav.heading=185;v.nav.distance_m=4294967295u;v.nav.speed=300;
    round_ui_navigation(&v);capture_save("navigation-bounds.rgb565");
    v.nav.heading=0;v.nav.distance_m=300;v.nav.speed=70;
    v.marker_x=20;v.marker_y=20;round_ui_navigation(&v);
    v.map_contains_route=false;v.marker_x=155;v.marker_y=211;
    round_ui_clock(10,24,"27 OCT 2026",128.6f);capture_save("clock.rgb565");
    round_ui_music("Highway Star","Deep Purple","",true,31,true);capture_save("music.rgb565");
    round_ui_music("No player","","CONNECT A PLAYER ON PHONE",false,0,false);capture_save("music-offline.rgb565");
    round_ui_call("Rohit",3,true);capture_save("call.rgb565");
    round_ui_call("A caller with a very long name",0,false);capture_save("call-offline.rgb565");
    round_ui_navigation(&v);round_ui_notification("WhatsApp","Rahul","On my way!");capture_save("notification.rgb565");
    return 0;
}
'''
(OUT/'fixture.c').write_text(fixture)
(OUT/'ucrt.def').write_text('LIBRARY ucrtbase.dll\nEXPORTS\ncosf\nsinf\nfminf\nlroundf\nsqrtf\natan2f\nroundf\nabs\nabort\nmemset\nmemcpy\nmemcmp\nstrlen\nstrcpy\nstrstr\nstrcmp\ntoupper\nfopen\nfwrite\nfclose\n')
(OUT/'msvcrt.def').write_text('LIBRARY msvcrt.dll\nEXPORTS\n_snprintf\n')
compilers=sorted(Path('C:/Program Files (x86)/Microsoft Visual Studio').glob('*/BuildTools/VC/Tools/MSVC/*/bin/Hostx64/x64/cl.exe'))
if not compilers:raise SystemExit('MSVC Build Tools required')
compiler=compilers[-1];env=dict(os.environ,PATH=str(compiler.parent)+os.pathsep+os.environ['PATH'])
for name in ['ucrt','msvcrt']:
    subprocess.run([str(compiler.parent/'lib.exe'),'/nologo',f'/def:{name}.def','/machine:x64',f'/out:{name}.lib'],cwd=OUT,env=env,check=True)
subprocess.run([str(compiler),'/nologo','/std:c11','/utf-8','/GS-','/Zl','/LD','/DHUD_CAPTURE','/Iinclude','/I.',f'/I{ROOT}/main/display',
    str(ROOT/'main/display/hud_ui.c'),str(ROOT/'main/display/round_ui.c'),'raster.c','fixture.c','/Fepreview.dll',
    '/link','/noentry','/nodefaultlib','ucrt.lib','msvcrt.lib'],cwd=OUT,env=env,check=True)
os.chdir(OUT)
dll=ctypes.CDLL(str(OUT/'preview.dll'));assert dll.render_round()==0
for file in OUT.glob('*.rgb565'):
    raw=file.read_bytes();assert len(raw)==320*320*2
    rgb=[]
    for i in range(0,len(raw),2):
        v=raw[i]<<8|raw[i+1];rgb.append((((v>>11)&31)*255//31,((v>>5)&63)*255//63,(v&31)*255//31))
    # Exact integer pixel replication from round_panel_pixels(), not Pillow's
    # center-sampled nearest resize (which differs at some row boundaries).
    indices=[((p+1)*320-1)//466 for p in range(466)]
    canvas=Image.new('RGB',(466,466));canvas.putdata([rgb[y*320+x] for y in indices for x in indices])
    mask=Image.new('L',(466,466));ImageDraw.Draw(mask).ellipse((0,0,465,465),fill=255)
    result=Image.new('RGB',(506,506),'#111719');result.paste(canvas,(20,20),mask)
    result.save(file.with_suffix('.png'))
sheet=Image.new('RGB',(1530,1100),'#111719');draw=ImageDraw.Draw(sheet)
labels=[('navigation','NAVIGATION / RIGHT'),('clock','CLOCK'),('music','MUSIC'),('navigation-left','NAVIGATION / LEFT'),('call','INCOMING CALL'),('notification','NOTIFICATION')]
for i,(name,label) in enumerate(labels):
    x=(i%3)*510;y=(i//3)*550
    draw.text((x+28,y+12),label,fill='white')
    sheet.paste(Image.open(OUT/(name+'.png')),(x,y+35))
sheet.save(OUT/'screens.png')
if args.reference:
    reference=Image.open(args.reference).convert('RGB')
    comparison=Image.new('RGB',(1040,570),'#111719');d=ImageDraw.Draw(comparison)
    d.text((16,16),'YOUR REFERENCE (physical enclosure included)',fill='white')
    d.text((535,16),'FIRMWARE RENDERER / LABELED INITIAL DEMO',fill='white')
    comparison.paste(reference,(16,46))
    comparison.paste(Image.open(OUT/'navigation.png'),(525,45))
    comparison.save(OUT/'reference-comparison.png')
if (OUT/'navigation-before.png').exists():
    comparison=Image.new('RGB',(1020,545),'#111719');d=ImageDraw.Draw(comparison)
    d.text((20,10),'BEFORE: REGULAR PREVIEW GRID',fill='white')
    d.text((530,10),'AFTER: IRREGULAR INITIAL DEMO',fill='white')
    comparison.paste(Image.open(OUT/'navigation-before.png'),(0,35))
    comparison.paste(Image.open(OUT/'navigation.png'),(510,35))
    comparison.save(OUT/'before-after.png')
print('Production UI captures and coordinate/report checks passed:',OUT/'screens.png')
