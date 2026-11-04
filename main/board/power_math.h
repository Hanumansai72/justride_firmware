#pragma once
#include <stdbool.h>
#include <stdint.h>

// GPIO4 sees VCC through 200k/100k, not the ETA6098 BATS terminal.
// Values are supply-source inferences for the stock USB-C / 1S LiPo paths.
typedef struct {
    uint16_t rail_mv, battery_mv;
    uint8_t source, percent; // 0 unknown, 1 battery inferred, 2 USB inferred
    bool valid;
} device_power_t;
static inline bool power_is_low(device_power_t p) {
    return p.valid && p.source==1 && p.percent<=15;
}
static inline bool power_ota_allowed(device_power_t p) {
    return p.valid && (p.source==2 || (p.source==1 && p.percent>=30 && p.percent<=100));
}
typedef struct {unsigned samples;bool warned;} power_warning_t;
static inline bool power_warning_update(power_warning_t *w,device_power_t p) {
    if(p.valid && (p.source==2 || (p.source==1 && p.percent>=20)))w->warned=false;
    if(!power_is_low(p)){w->samples=0;return false;}
    if(w->samples<5)w->samples++;
    if(w->samples<5 || w->warned)return false;
    w->warned=true;return true;
}
static inline uint8_t power_percent(uint16_t mv) {
    static const uint16_t volts[]={3300,3500,3600,3700,3750,3800,3900,4000,4100,4200};
    static const uint8_t pct[]={0,10,20,40,50,60,75,85,95,100};
    if(mv<=volts[0])return 0;
    for(unsigned i=1;i<10;i++)if(mv<volts[i])
        return pct[i-1]+(mv-volts[i-1])*(pct[i]-pct[i-1])/(volts[i]-volts[i-1]);
    return 100;
}
static inline device_power_t power_measurement(int adc_mv,int gain,uint8_t previous_source) {
    device_power_t p={.percent=255};
    int rail=adc_mv*3*gain/1000;
    if(rail<2800 || rail>5500)return p;
    p.valid=true;p.rail_mv=rail;
    if(rail>=4450 || (previous_source==2 && rail>=4350))p.source=2;
    else if(rail>=3000 && rail<=4300) {
        p.source=1;p.battery_mv=rail;p.percent=power_percent(rail);
    }
    return p;
}
static inline void power_encode(device_power_t p,uint8_t out[10]) {
    out[0]=0x1d;out[1]=1;
    out[2]=(p.valid?1:0)|(p.percent<=100?2:0);
    out[3]=p.source;out[4]=255; // STAT is not connected to the MCU.
    out[5]=p.percent;
    out[6]=p.rail_mv;out[7]=p.rail_mv>>8;
    out[8]=p.battery_mv;out[9]=p.battery_mv>>8;
}
