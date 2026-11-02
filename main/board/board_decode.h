#pragma once
#include <stdbool.h>
#include <stdint.h>
static inline bool ft3168_decode(const uint8_t p[5],int *x,int *y) {
    // One stable contact; ignore releases, invalid IDs and multitouch.
    if((p[0]&15)!=1 || (p[1]>>6)==1 || (p[3]>>4)!=0)return false;
    *x=((p[1]&15)<<8)|p[2];*y=((p[3]&15)<<8)|p[4];
    return *x<466 && *y<466;
}
static inline int16_t qmi8658_s16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0]|(uint16_t)p[1]<<8);
}
