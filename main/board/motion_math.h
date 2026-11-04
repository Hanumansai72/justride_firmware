#pragma once
#include <math.h>
#include <stdint.h>
static inline float motion_turn_raw(const int16_t gyro[3],float ax,float ay,float az) {
    float length=sqrtf(ax*ax+ay*ay+az*az);
    return length>1 ? (gyro[0]*ax+gyro[1]*ay+gyro[2]*az)/length : 0;
}
static inline float motion_signed_degrees(float heading) {
    while(heading>180)heading-=360;
    while(heading< -180)heading+=360;
    return heading;
}
static inline float motion_integrate(float heading,float rate,float dt) {
    if(!isfinite(rate) || !isfinite(dt) || dt<=0 || dt>0.25f)return heading;
    if(fabsf(rate)<0.5f)return heading;
    return motion_signed_degrees(heading+rate*dt);
}
