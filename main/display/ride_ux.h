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
static inline uint8_t ride_music_command(uint8_t state,uint8_t actions,bool glove,int x,int y) {
    if(state<3 || state>5)return y>=190 && y<=263?(state==1?8:7):0;
    // Test volume first: its visible labels overlap the old playback hit row.
    if(y>=277 && y<=310) {
        if((actions&16) && x>=(glove?80:92) && x<=(glove?150:144))return 6;
        if((actions&16) && x>=(glove?170:176) && x<=(glove?240:228))return 5;
    }
    if(y<(glove?199:205) || y>285)return 0;
    if((actions&8) && x>=(glove?38:48) && x<=112)return 4;
    if(x>=(glove?120:125) && x<=(glove?200:195)) {
        uint8_t command=state==4?2:1;
        return (actions&command)?command:0;
    }
    return (actions&4) && x>=208 && x<=(glove?282:272)?3:0;
}
