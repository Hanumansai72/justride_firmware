"""Rasterize the bundled reference-style fonts for the round display only."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

root = Path(__file__).resolve().parents[1]
assets = root / 'main/display/assets'
source = root / 'tools/fonts'
rows = ['/* Roboto, Apache-2.0; see tools/fonts/LICENSE.txt. */',
        '#pragma once', '#include <stdint.h>',
        'typedef struct { uint32_t offset; uint8_t width,height,advance; int8_t left,top; } hud_glyph_t;',
        'typedef struct { const hud_glyph_t *glyphs; const uint8_t *pixels; uint8_t height; } hud_font_t;']
for size in [8, 10, 12, 16, 18, 20, 22, 24, 28, 32, 36, 38, 52]:
    face = 'roboto-bold.ttf' if size == 32 else 'roboto-medium.ttf' if size in (36,38,52) else 'roboto-regular.ttf'
    font = ImageFont.truetype(str(source / face), size)
    data, glyphs = [], []
    for cp in range(32, 127):
        box = font.getbbox(chr(cp))
        w, h = max(0, box[2]-box[0]), max(0, box[3]-box[1])
        assert w <= 64 and h <= 255
        image = Image.new('L', (w, h))
        ImageDraw.Draw(image).text((-box[0], -box[1]), chr(cp), font=font, fill=255)
        values = [v//17 for v in image.get_flattened_data()]
        offset = len(data)
        for i in range(0, len(values), 2):
            data.append(values[i]<<4 | (values[i+1] if i+1 < len(values) else 0))
        glyphs.append((offset, w, h, round(font.getlength(chr(cp))), box[0], box[1]))
    rows.append(f'static const hud_glyph_t hud_glyphs_{size}[95]={{' + ','.join('{' + ','.join(map(str,g)) + '}' for g in glyphs) + '};')
    rows.append(f'static const uint8_t hud_pixels_{size}[]={{' + ','.join(map(str,data)) + '};')
    rows.append(f'static const hud_font_t hud_font_{size}={{hud_glyphs_{size},hud_pixels_{size},{size}}};')
(assets / 'round_fonts.h').write_text('\n'.join(rows)+'\n', encoding='utf-8')
print('Generated reference-style Roboto glyphs for the round display.')
