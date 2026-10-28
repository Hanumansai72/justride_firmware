#pragma once
// Website palette: justride-customer/src/index.css :root.
// Convert 24-bit sRGB literals to the TFT's RGB565 format at compile time.
#define JR_RGB565(rgb) (((((rgb) >> 19) & 31) << 11) | ((((rgb) >> 10) & 63) << 5) | (((rgb) >> 3) & 31))
#define JR_BG           JR_RGB565(0x0b100d)
#define JR_PANEL        JR_RGB565(0x141a16)
#define JR_PANEL_ALT    JR_RGB565(0x1a211c)
#define JR_TEXT         JR_RGB565(0xf5f7f5)
#define JR_MUTED        JR_RGB565(0xaab5ad)
#define JR_PRIMARY      JR_RGB565(0x34d399)
#define JR_PRIMARY_DARK JR_RGB565(0x087c59)
// Website --line: 11% white composited over --panel.
#define JR_BORDER       JR_RGB565(0x2e3330)
#define JR_ACCENT_DIM   JR_RGB565(0x113528)
#define JR_DANGER       JR_RGB565(0xef4444)
#define JR_WARNING      JR_RGB565(0xfbbf24)
