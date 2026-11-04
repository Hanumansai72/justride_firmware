#pragma once
#include <stdbool.h>
#include <stdint.h>
enum { RIDE_LIVE, RIDE_RECONNECTING, RIDE_SYNCING, RIDE_GPS_WEAK,
       RIDE_REROUTING, RIDE_PAUSED, RIDE_STALE };
static inline uint8_t ride_recovery(bool connected,bool ready,bool gps,bool rerouting,
                                    bool paused,int64_t age_us) {
    if(!connected)return RIDE_RECONNECTING;
    if(!ready)return RIDE_SYNCING;
    if(age_us>=5000000)return RIDE_STALE;
    if(!gps)return RIDE_GPS_WEAK;
    if(rerouting)return RIDE_REROUTING;
    return paused?RIDE_PAUSED:RIDE_LIVE;
}
static inline int ride_turn_side(uint8_t direction) {
    return direction==2 || direction==4 || direction==6 ? -1 :
           direction==3 || direction==5 || direction==7 ? 1 : 0;
}
static inline bool ride_turn_alert(uint8_t direction,uint8_t maneuver_state,
                                   uint32_t distance,uint8_t recovery) {
    return recovery==RIDE_LIVE && maneuver_state!=5 && ride_turn_side(direction) && distance<=15;
}
static inline bool ride_blink_on(int64_t now_us){return (now_us/500000)%2==0;}
static inline bool ride_sharp_warning(uint8_t direction,uint8_t state,uint32_t distance,
                                      unsigned speed,uint8_t recovery) {
    unsigned threshold=speed*6/3.6f;
    if(threshold<50)threshold=50;
    if(threshold>200)threshold=200;
    return recovery==RIDE_LIVE && state!=5 && (direction==6 || direction==7) && distance<=threshold;
}
static inline uint8_t ride_scheduled_brightness(unsigned hour){return hour>=7 && hour<19?85:35;}
static inline uint8_t ride_call_action(bool glove,int x,int y) {
    if(y<(glove?190:201) || y>(glove?280:269))return 0;
    if(x>=(glove?67:77) && x<=(glove?150:145))return 2;
    if(x>=(glove?170:175) && x<=(glove?253:243))return 1;
    return 0;
}
