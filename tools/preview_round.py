"""Build the production round UI with a host raster sink; no ESP32 required.

Windows: Python with Pillow, MSVC Build Tools (no Windows SDK needed).
The small import headers are host-only. All images use the production renderer with explicitly synthetic telemetry.
No map tiles or demo navigation are embedded in the device firmware.
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
#include "../board/board_decode.h"
#include "hud_capture_driver.h"
#include <assert.h>
#include <string.h>
void capture_save(const char*);
const uint8_t *capture_pixels(void);
static uint8_t previous[320*320*2];
__declspec(dllexport) int render_round(void) {
    int16_t x,y;
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
    int tx,ty;
    uint8_t p[]={1,0x80,233,0,250};
    assert(ft3168_decode(p,&tx,&ty) && tx==233 && ty==250);
    p[1]=0x40;assert(!ft3168_decode(p,&tx,&ty));
    p[1]=0x80;p[0]=2;assert(!ft3168_decode(p,&tx,&ty));
    p[0]=1;p[1]=0x81;p[2]=210;assert(!ft3168_decode(p,&tx,&ty)); // x=466
    assert(qmi8658_s16((uint8_t[]){0,0x40})==16384);
    assert(qmi8658_s16((uint8_t[]){0,0xc0})==-16384);
    assert(qmi8658_s16((uint8_t[]){0xff,0xff})==-1);
    round_ui_splash(0);capture_save("splash-start.rgb565");
    memcpy(previous,capture_pixels(),sizeof previous);
    round_ui_splash_progress(65);capture_save("splash.rgb565");
    for(int yy=0;yy<320;yy++)for(int xx=0;xx<320;xx++) {
        if(yy>=185 && yy<189 && xx>=65 && xx<255)continue;
        unsigned i=(yy*320+xx)*2;
        assert(previous[i]==capture_pixels()[i] && previous[i+1]==capture_pixels()[i+1]);
    }
    for(unsigned i=240*320*2;i<sizeof previous;i++)assert(capture_pixels()[i]==0);
    // SAMPLE TELEMETRY ONLY: same production renderer, no device/GPS claims.
    round_route_point_t points[]={{160,160},{160,135},{160,110},{194,90},{208,66},{178,42},{166,8}};
    round_nav_t v={.nav={.speed=45,.speed_valid=true,.direction=5,.distance_m=300,.heading=0},
        .points=points,.count=7,.zoom_percent=100,.trip_valid=true,.arrival_time="14:42"};
    v.nav.remaining_km=12.8f;
    v.nav.direction=8;v.nav.distance_m=70;v.nav.speed=28;
    round_ui_navigation(&v);capture_save("u-turn.rgb565");
    v.nav.direction=11;v.nav.distance_m=150;v.nav.speed=45;
    strcpy(v.nav.instruction,"Take flyover");
    round_ui_navigation(&v);capture_save("flyover-current.rgb565");
    v.nav.direction=5;v.nav.distance_m=300;strcpy(v.nav.instruction,"");
    round_ui_navigation(&v);capture_save("navigation.rgb565");
    memcpy(previous,capture_pixels(),sizeof previous);
    v.nav.direction=4;round_ui_navigation(&v);capture_save("navigation-left.rgb565");
    assert(memcmp(previous,capture_pixels(),sizeof previous));
    v.nav.distance_m=120;round_ui_navigation(&v);capture_save("distance-update.rgb565");
    v.nav.distance_m=80;v.maneuver_state=3;v.blink_on=true;
    round_ui_navigation(&v);capture_save("turn-left-on.rgb565");
    memcpy(previous,capture_pixels(),sizeof previous);
    v.blink_on=false;round_ui_navigation(&v);capture_save("turn-left-off.rgb565");
    assert(memcmp(previous,capture_pixels(),sizeof previous));
    for(int yy=0;yy<320;yy++)for(int xx=0;xx<320;xx++) {
        unsigned at=(yy*320+xx)*2;
        if(xx<32 || xx>=109 || yy<127 || yy>=177)
            assert(previous[at]==capture_pixels()[at] && previous[at+1]==capture_pixels()[at+1]);
    }
    memcpy(previous,capture_pixels(),sizeof previous);
    v.maneuver_state=5;round_ui_navigation(&v);
    assert(!memcmp(previous,capture_pixels(),sizeof previous)); // Blink off preserves the full map and pointer.
    v.maneuver_state=3;
    v.nav.direction=5;v.blink_on=true;
    round_ui_navigation(&v);capture_save("turn-right-on.rgb565");
    v.blink_on=false;round_ui_navigation(&v);capture_save("turn-right-off.rgb565");
    v.recovery=RIDE_STALE;round_ui_navigation(&v);capture_save("stale-data.rgb565");
    v.recovery=RIDE_RECONNECTING;round_ui_navigation(&v);capture_save("reconnecting.rgb565");
    v.recovery=RIDE_GPS_WEAK;round_ui_navigation(&v);capture_save("gps-weak.rgb565");
    v.recovery=RIDE_REROUTING;strcpy(v.nav.instruction,"Off route - rerouting");
    round_ui_navigation(&v);capture_save("off-route-recalculating.rgb565");
    strcpy(v.nav.instruction,"Off route - retrying");
    round_ui_navigation(&v);capture_save("off-route-retrying.rgb565");
    strcpy(v.nav.instruction,"");
    v.recovery=RIDE_LIVE;v.nav.distance_m=300;v.battery_low=true;v.battery_percent=12;
    round_ui_navigation(&v);capture_save("low-battery.rgb565");v.battery_low=false;
    round_ui_arrival();capture_save("arrival.rgb565");
    v.nav.distance_m=300;v.nav.speed_valid=false;strcpy(v.nav.instruction,"GPS LOST");
    v.recovery=RIDE_GPS_WEAK;
    round_ui_navigation(&v);capture_save("gps-lost.rgb565");
    strcpy(v.nav.instruction,"PHONE OFFLINE");v.recovery=RIDE_RECONNECTING;round_ui_navigation(&v);capture_save("phone-offline.rgb565");
    strcpy(v.nav.instruction,"");v.recovery=RIDE_LIVE;v.nav.speed_valid=true;v.nav.direction=5;v.overview=true;
    round_ui_navigation(&v);capture_save("overview.rgb565");
    v.overview=false;v.count=0;strcpy(v.nav.instruction,"SYNCING");
    round_ui_navigation(&v);capture_save("waiting-for-route.rgb565");
    v.count=7;v.nav.distance_m=4294967295u;v.nav.heading=359;v.nav.speed=300;
    round_ui_navigation(&v);capture_save("bounds.rgb565");
    round_ui_clock(10,24,"MON, 02 NOV 2026",12.8f);capture_save("clock.rgb565");
    round_ui_music("Highway Star","Deep Purple","",true,31,true);capture_save("music.rgb565");
    round_ui_music("No player","","CONNECT A PLAYER ON PHONE",false,0,false);capture_save("music-offline.rgb565");
    round_ui_call("Rohit",3,true);capture_save("call.rgb565");
    round_ui_call("A caller with a very long name",0,false);capture_save("call-offline.rgb565");
    v.nav.distance_m=300;v.nav.direction=5;strcpy(v.nav.instruction,"");
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
subprocess.run([str(compiler),'/nologo','/std:c11','/utf-8','/GS-','/Zl','/LD','/DHUD_CAPTURE','/DCONFIG_JUSTRIDE_ROUND_AMOLED=1','/Iinclude','/I.',f'/I{ROOT}/main/display',
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
    canvas.save(file.with_name(file.stem+'-native.png'))
sheet=Image.new('RGB',(1530,1100),'#111719');draw=ImageDraw.Draw(sheet)
labels=[('navigation','NAVIGATION / RIGHT'),('clock','CLOCK'),('music','MUSIC'),('navigation-left','NAVIGATION / LEFT'),('call','INCOMING CALL'),('notification','NOTIFICATION')]
for i,(name,label) in enumerate(labels):
    x=(i%3)*510;y=(i//3)*550
    draw.text((x+28,y+12),label,fill='white')
    sheet.paste(Image.open(OUT/(name+'.png')),(x,y+35))
draw.text((20,1080),'Production C renderer with SAMPLE data. Physical panel/touch/radio operation requires hardware testing.',fill='white')
sheet.save(OUT/'screens.png')
ux=Image.new('RGB',(1530,1650),'#111719');d=ImageDraw.Draw(ux)
for i,(name,label) in enumerate([
    ('turn-left-on','LEFT TURN / BLINK ON'),('turn-right-on','RIGHT TURN / BLINK ON'),('turn-left-off','LEFT TURN / BLINK OFF'),
    ('navigation','LIVE ETA + REMAINING'),('reconnecting','RECONNECTING / STALE ROUTE'),('gps-weak','GPS WEAK / STALE ROUTE'),
    ('low-battery','LOW BATTERY'),('arrival','ARRIVAL'),('stale-data','NO RECENT PHONE DATA')]):
    x=(i%3)*510;y=(i//3)*550;d.text((x+24,y+12),label,fill='white')
    ux.paste(Image.open(OUT/(name+'.png')),(x,y+35))
d.text((20,1630),'Production renderer / SAMPLE data. Each round display is 466 x 466 pixels. Hardware test still required.',fill='white')
ux.save(OUT/'ride-experience.png')
recovery=Image.new('RGB',(1530,570),'#111719');d=ImageDraw.Draw(recovery)
for i,(name,label) in enumerate([('turn-left-on','MEDIUM LEFT-SIDE TURN ALERT / MAP VISIBLE'),('off-route-recalculating','CONFIRMED OFF ROUTE / RECALCULATING'),('off-route-retrying','REQUEST FAILED / ALERT RETAINED')]):
    d.text((i*510+15,12),label,fill='white');recovery.paste(Image.open(OUT/(name+'.png')),(i*510,35))
d.text((15,550),'Production C renderer / SAMPLE telemetry. Physical board and live navigation still need testing.',fill='white')
recovery.save(OUT/'left-alert-rerouting.png')
guide=Image.new('RGB',(1020,570),'#111719');d=ImageDraw.Draw(guide)
for i,(name,label) in enumerate([('u-turn','U-TURN / CURRENT IMPLEMENTATION'),('flyover-current','FLYOVER REQUEST / CURRENT GENERIC EXIT FALLBACK')]):
    d.text((i*510+15,12),label,fill='white')
    guide.paste(Image.open(OUT/(name+'.png')),(i*510,35))
d.text((15,550),'Production C renderer, SAMPLE route. Flyover-specific instructions/road level are not implemented.',fill='white')
guide.save(OUT/'u-turn-flyover-current.png')
for side in ['left','right']:
    frames=[Image.open(OUT/f'turn-{side}-{phase}.png') for phase in ['on','off']]
    frames[0].save(OUT/f'turn-{side}-blink.gif',save_all=True,append_images=frames[1:],duration=500,loop=0)
    native=[Image.open(OUT/f'turn-{side}-{phase}-native.png') for phase in ['on','off']]
    native[0].save(OUT/f'turn-{side}-blink-466.gif',save_all=True,append_images=native[1:],duration=500,loop=0)
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
