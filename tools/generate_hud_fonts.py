from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
root=Path('main/display/assets'); src=Path('C:/src/flutter/bin/cache/artifacts/material_fonts')
rows=['/* Generated from Apache-2.0 Roboto; see ROBOTO_LICENSE.txt. */','#pragma once','#include <stdint.h>','typedef struct { uint32_t offset; uint8_t width,height,advance; int8_t left,top; } hud_glyph_t;','typedef struct { const hud_glyph_t *glyphs; const uint8_t *pixels; uint8_t height; } hud_font_t;']
for size in [8,10,12,16,22,28,36]:
    font=ImageFont.truetype(str(src/'roboto-bold.ttf'),size)
    data=[];glyphs=[]
    for cp in range(32,127):
        box=font.getbbox(chr(cp)); w=max(0,box[2]-box[0]); h=max(0,box[3]-box[1]); im=Image.new('L',(w,h)); ImageDraw.Draw(im).text((-box[0],-box[1]),chr(cp),font=font,fill=255)
        assert w<=32, (size,cp,w)
        values=[v//17 for v in im.get_flattened_data()]; offset=len(data)
        for i in range(0,len(values),2):data.append((values[i]<<4)|(values[i+1] if i+1<len(values) else 0))
        glyphs.append((offset,w,h,round(font.getlength(chr(cp))),box[0],box[1]))
    rows.append('static const hud_glyph_t hud_glyphs_%d[95]={'%size+','.join('{%s}'%','.join(map(str,g)) for g in glyphs)+'};')
    rows.append('static const uint8_t hud_pixels_%d[]={ '%size+','.join(map(str,data))+' };')
    rows.append('static const hud_font_t hud_font_%d={hud_glyphs_%d,hud_pixels_%d,%d};'%(size,size,size,size))
(root/'hud_fonts.h').write_text('\n'.join(rows),encoding='utf-8');(root/'ROBOTO_LICENSE.txt').write_bytes((src/'roboto_license.txt').read_bytes())
print('Generated fonts:',(root/'hud_fonts.h').stat().st_size,'source bytes')
