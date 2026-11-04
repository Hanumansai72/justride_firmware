#pragma once
#include "hud_ui.h"
#include "ride_ux.h"
typedef struct { uint16_t x,y; } round_route_point_t;
typedef struct {
    hud_model_t nav;
    const round_route_point_t *points;
    unsigned count, completed;
    bool overview;
    float tilt;
    int zoom_percent;
    uint8_t recovery, maneuver_state, battery_percent;
    bool blink_on, battery_low, trip_valid;
    char arrival_time[8];
    char destination[32];
    uint8_t next_direction;
} round_nav_t;
void round_ui_navigation(const round_nav_t *view);
void round_ui_set_glove_mode(bool enabled);
void round_ui_arrival(void);
void round_ui_splash(unsigned progress);
void round_ui_splash_progress(unsigned progress);
void round_ui_clock(uint8_t hours,uint8_t minutes,const char *date,float remaining_km);
void round_ui_music(const char *title,const char *artist,const char *status,
                    bool playing,uint8_t actions,bool active);
void round_ui_call(const char *caller,uint8_t actions,bool connected);
void round_ui_notification(const char *app,const char *sender,const char *message);
