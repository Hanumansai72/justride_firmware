"""Use the Flutter SDK's standard call/call_end glyphs; no hand-drawn handset."""
import argparse
from pathlib import Path
import re
from PIL import Image, ImageDraw, ImageFont

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--flutter',type=Path,required=True)
args=parser.parse_args()
icons=(args.flutter/'packages/flutter/lib/src/material/icons.dart').read_text(encoding='utf-8')
font=ImageFont.truetype(str(args.flutter/'bin/cache/artifacts/material_fonts/materialicons-regular.otf'),28)
lines=['/* Google Material Icons, Apache-2.0; see tools/fonts/LICENSE.txt. */','#pragma once','#include <stdint.h>']
for name in ('call','call_end'):
    code=int(re.search(rf'IconData {name} = IconData\(0x([0-9a-f]+)',icons)[1],16)
    value=chr(code);left,top,right,bottom=font.getbbox(value)
    mask=Image.new('L',(28,28));ImageDraw.Draw(mask).text(((28-right+left)/2-left,(28-bottom+top)/2-top),value,font=font,fill=255)
    pixels=[v//17 for v in mask.get_flattened_data()]
    packed=[pixels[i]<<4|pixels[i+1] for i in range(0,len(pixels),2)]
    lines.append(f'static const uint8_t round_{name}_icon[392]={{'+','.join(map(str,packed))+'};')
output=Path(__file__).resolve().parents[1]/'main/display/assets/call_icons.h'
output.write_text('\n'.join(lines)+'\n',encoding='utf-8')
print('Generated standard answer and hang-up icons from the installed Flutter SDK.')
