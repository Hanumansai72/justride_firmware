#pragma once
#include <stdbool.h>
#include <stdint.h>

// Shared by the QSPI transport, I2C touch driver and host verification.
// 352x264 diagonal=440, inside the physical 466-pixel circle.
static inline int round_map_x(int x, bool round) {
    return round ? x * 466 / 320 : 57 + x * 352 / 320;
}
static inline int round_map_y(int y, bool round) {
    return round ? y * 466 / 320 : 101 + y * 264 / 240;
}
static inline bool round_unmap_touch(int px, int py, bool round,
                                    int16_t *x, int16_t *y) {
    if (px < 0 || py < 0 || px >= 466 || py >= 466) return false;
    int dx=2*px-465, dy=2*py-465;
    if (dx*dx + dy*dy > 465*465) return false;
    if (round) { *x=px*320/466; *y=py*320/466; }
    else {
        if (px<57 || px>=409 || py<101 || py>=365) return false;
        *x=(px-57)*320/352; *y=(py-101)*240/264;
    }
    return true;
}
static inline bool cst9217_decode(const uint8_t *p, int *x, int *y) {
    // CST92xx report at D000: event=6, count byte5, trailer=AB.
    if (p[0]==0 || p[0]==0xAB || p[6]!=0xAB || (p[4]&0x80) ||
        (p[5]&0x7F)==0 || (p[5]&0x7F)>2 || (p[0]>>4)>=2 || (p[0]&15)!=6) return false;
    *x=(p[1]<<4)|(p[3]>>4); *y=(p[2]<<4)|(p[3]&15);
    return *x<466 && *y<466;
}
