#pragma once
#include <stdbool.h>
#include <stdint.h>

// Display-only standby; BLE, touch polling and route state remain running.
typedef struct {int64_t last_activity_us;bool asleep;} display_idle_t;
static inline bool display_sleep_minutes_valid(unsigned minutes) {
    return minutes==0 || minutes==1 || minutes==5 || minutes==10;
}
// -1 enter standby, +1 wake, 0 unchanged. Active use always refreshes the timer.
static inline int display_idle_update(display_idle_t *s,int64_t now,bool busy,bool touch,unsigned minutes) {
    if(busy || touch)s->last_activity_us=now;
    bool sleep=minutes && !busy && !touch && now-s->last_activity_us>=(int64_t)minutes*60000000;
    if(sleep==s->asleep)return 0;
    s->asleep=sleep;
    return sleep?-1:1;
}
