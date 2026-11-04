#pragma once
#include <stdbool.h>
#include <stdint.h>

// Display-only standby; BLE, touch polling and route state remain running.
typedef struct {int64_t last_activity_us;bool asleep;} display_idle_t;
// -1 enter standby, +1 wake, 0 unchanged. Active use always refreshes the timer.
static inline int display_idle_update(display_idle_t *s,int64_t now,bool busy,bool touch) {
    if(busy || touch)s->last_activity_us=now;
    bool sleep=!busy && !touch && now-s->last_activity_us>=15000000;
    if(sleep==s->asleep)return 0;
    s->asleep=sleep;
    return sleep?-1:1;
}
