"""Prepare a host capture of production HUD C + production TFT primitives.
Run this script, compile the generated driver with MSVC, run hud_capture.exe,
then use --images to convert the resulting RGB565 captures to PNG.
Host captures use sample values, not a live device or hardware timing.
"""
from pathlib import Path
import argparse, json, math
from PIL import Image, ImageDraw
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'build/capture'
OUT.mkdir(parents=True,exist_ok=True)

def images():
    for path in OUT.glob('*.rgb565'):
        if len(path.read_bytes()) != 320*240*2: continue
        data=path.read_bytes(); rgb=[]
        for i in range(0,len(data),2):
            v=(data[i]<<8)|data[i+1]
            rgb.append((((v>>11)&31)*255//31,((v>>5)&63)*255//63,(v&31)*255//31))
        im=Image.new('RGB',(320,240));im.putdata(rgb);im.save(path.with_suffix('.png'))
        im.resize((960,720),Image.Resampling.NEAREST).save(path.with_name(path.stem+'-3x.png'))
    ref=Path('C:/Users/hanum/AppData/Local/Temp/codex-clipboard-f14df32c-06d4-48e8-9de1-a1f20e2cbbe0.png')
    if ref.exists():
        ref=Image.open(ref).convert('RGB').transform((320,240),Image.Transform.QUAD,(180,167,168,878,1226,877,1223,162),Image.Resampling.BICUBIC)
        ref.save(OUT/'reference-rectified.png')
        result=Image.new('RGB',(1304,530),'#09121b');d=ImageDraw.Draw(result)
        d.text((8,8),'Reference (photographed screen rectified)',fill='white')
        d.text((664,8),'Actual C renderer / 2D map / sample values',fill='white')
        result.paste(ref.resize((640,480)),(8,32))
        result.paste(Image.open(OUT/'notification.png').resize((640,480),Image.Resampling.NEAREST),(664,32))
        result.save(OUT/'comparison.png')

if '--images' in __import__('sys').argv:
    images();raise SystemExit
header=(ROOT/'main/display/display_tft.h').read_text(encoding='utf-8')
enum=header[header.index('typedef enum {'):header.index('} nav_direction_t;')+len('} nav_direction_t;')]
apis=header[header.index('void tft_fill_screen'):header.index('spi_host_device_t tft_get_spi_host')]
(OUT/'hud_capture_driver.h').write_text('#pragma once\n#include <stdint.h>\n#include <stddef.h>\n#include <stdbool.h>\n'+enum+'\n#define COLOR_SURFACE_DARK 0x0000\n'+apis)
src=(ROOT/'main/display/display_tft.c').read_text(encoding='utf-8')
font=src[src.index('static const uint8_t font5x8'):src.index('};',src.index('static const uint8_t font5x8'))+2]
primitives=src[src.index('// One reusable 5 KiB strip'):]
shim=r'''#include "hud_capture_driver.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define DMA_ATTR
typedef void* TaskHandle_t;
static TaskHandle_t xTaskGetCurrentTaskHandle(void){return (void*)1;}
static int s_width=320,s_height=240;
static unsigned char framebuffer[320*240*2];
static unsigned cursor,x0,y0,x1,y1;
void tft_spi_lock(void){}
void tft_spi_unlock(void){}
static void tft_set_addr_window(unsigned a,unsigned b,unsigned c,unsigned d){
 assert(a<=c && b<=d && c<320 && d<240);x0=a;y0=b;x1=c;y1=d;cursor=0;
}
static void tft_send_data(const uint8_t *data,size_t len){
 assert((len&1)==0);
 for(size_t i=0;i<len;i+=2){unsigned x=x0+cursor%(x1-x0+1),y=y0+cursor/(x1-x0+1);
 assert(x<=x1 && y<=y1);framebuffer[(y*320+x)*2]=data[i];framebuffer[(y*320+x)*2+1]=data[i+1];cursor++;}
}
void capture_save(const char *path){FILE *f=fopen(path,"wb");assert(f);fwrite(framebuffer,1,sizeof(framebuffer),f);fclose(f);}
'''
(OUT/'tft_raster.c').write_text(shim+'\n'+font+'\n'+primitives,encoding='utf-8')
main=r'''#include "hud_ui.h"
#include "hud_capture_driver.h"
#include <stdio.h>
#include <assert.h>
void capture_save(const char *path);
int main(void){
 tft_fill_screen(HUD_BG);
 FILE *f=fopen("build/capture/map.rgb","rb");
 if(f){unsigned char row[226*2];for(int y=0;y<177;y++){assert(fread(row,1,sizeof(row),f)==sizeof(row));tft_draw_raw_pixels(94,y,226,1,row);}fclose(f);}
 hud_model_t m={.speed=52,.speed_valid=true,.heading=0,.direction=5,.distance_m=400,.remaining_km=8.0,
 .duration="11 min",.instruction="TURN RIGHT",.poi_valid={false,false}};
 hud_speed(&m);hud_poi(&m,0);hud_poi(&m,1);hud_trip(&m);hud_compass(m.heading);hud_maneuver(&m);
 capture_save("build/capture/navigation.rgb565");
 hud_notification("WhatsApp","Rahul","On my way!",1600);capture_save("build/capture/notification.rgb565");
 hud_call("Caller name",1,true);capture_save("build/capture/incoming-call.rgb565");
 return 0;
}
'''
(OUT/'capture.c').write_text(main)
# Preview-only real LocationIQ sample. Route, if present, is actual OSRM geometry.
mapfile=OUT/'map.jpg'
if mapfile.exists():
    im=Image.open(mapfile).convert('RGB');assert im.size==(226,177)
    original=im.copy()
    im=im.point(lambda v:min(255,int(v*1.8)))
    pixels=im.load()
    for y in range(153):
        for x in range(226):
            r,g,b=pixels[x,y];pixels[x,y]=(min(255,r+4),min(255,g+10),min(255,b+18))
    im.paste(original.crop((0,153,226,177)),(0,153))
    routefile=OUT/'route.json'
    if routefile.exists():
        coordinates=json.loads(routefile.read_text())['routes'][0]['geometry']['coordinates']
        def world(lon,lat):
            scale=256*2**16;s=math.sin(math.radians(lat))
            return ((lon+180)/360*scale,(.5-math.log((1+s)/(1-s))/(4*math.pi))*scale)
        cx,cy=world(78.4867,17.385737)
        points=[(world(lon,lat)[0]-cx+113,world(lon,lat)[1]-cy+88.5) for lon,lat in coordinates]
        overlay=im.copy();d=ImageDraw.Draw(overlay);d.line(points,fill='#00bfff',width=7,joint='curve');d.line(points,fill='#00e5ff',width=2)
        until=32.0
        for a,b in zip(points,points[1:]):
            dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
            if length<.1:continue
            ux,uy=dx/length,dy/length;along=until
            while along<length:
                px,py=a[0]+ux*along,a[1]+uy*along
                if 8<=px<=218 and 35<=py<=145:
                    d.line([(px-ux*3-uy*3,py-uy*3+ux*3),(px+ux*2,py+uy*2),(px-ux*3+uy*3,py-uy*3-ux*3)],fill='#b8f8ff',width=1)
                along+=32
            until=(until-length)%32
        # Same attribution exclusion as the production Canvas renderer.
        im.paste(overlay.crop((0,0,226,153)),(0,0))
        d=ImageDraw.Draw(im);d.polygon([(113,116),(106,134),(113,130),(120,134)],fill='white',outline='#00e5ff')
    out=bytearray()
    for r,g,b in im.get_flattened_data():
        v=((r>>3)<<11)|((g>>2)<<5)|(b>>3);out.extend((v>>8,v&255))
    (OUT/'map.rgb').write_bytes(out)
print('Capture sources prepared in',OUT)
