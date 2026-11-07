#pragma once
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
static inline bool motion_calibration_stationary(const int16_t accel[3],const int16_t gyro[3]) {
    float gravity=(float)accel[0]*accel[0]+(float)accel[1]*accel[1]+(float)accel[2]*accel[2];
    // Calibration needs gravity near 1g and no rotation on any sensor axis.
    if(gravity<(0.9f*16384)*(0.9f*16384) || gravity>(1.1f*16384)*(1.1f*16384))return false;
    for(int i=0;i<3;i++)if(gyro[i]<=-128 || gyro[i]>=128)return false;
    return true;
}
static inline float motion_roll(float ax,float ay,float az) {
    return atan2f(ax,sqrtf(ay*ay+az*az))*57.295779513f;
}
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
// Local visual lean supplements yaw; it never changes the phone's route turn.
static inline float motion_pointer_degrees(float yaw,float roll,bool yaw_fresh) {
    if(!isfinite(roll))roll=0;
    roll=fmaxf(-45,fminf(45,roll));
    return motion_signed_degrees((yaw_fresh && isfinite(yaw)?yaw:0)+roll);
}
static inline float motion_pointer_step(float current,float target,float seconds) {
    if(!isfinite(target))return current;
    if(!isfinite(current) || !isfinite(seconds) || seconds<=0 || seconds>0.25f)return target;
    float delta=motion_signed_degrees(target-current);
    if(fabsf(delta)<0.1f)return target;
    return motion_signed_degrees(current+delta*seconds/(0.08f+seconds));
}
