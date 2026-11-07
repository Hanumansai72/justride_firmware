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
    bool battery_low, trip_valid;
    char arrival_time[8];
    char destination[32];
    uint8_t next_direction;
} round_nav_t;
void round_ui_navigation(const round_nav_t *view);
void round_ui_set_glove_mode(bool enabled);
void round_ui_arrival(void);
void round_ui_splash(unsigned progress);
void round_ui_splash_progress(unsigned progress);
void round_ui_home(const char *clock,const char *date,uint32_t distance_m,
                    uint8_t battery_percent,bool valid);
void round_ui_music(const char *title,const char *artist,const char *status,
                    bool playing,uint8_t actions,bool active,uint32_t elapsed,uint32_t duration);
void round_ui_call(const char *caller,const char *number,uint8_t actions,bool connected);
void round_ui_notification(const char *app,const char *sender,const char *message);
void round_ui_settings(bool connected,unsigned brightness);
void round_ui_brightness(unsigned brightness);
void round_ui_update(unsigned progress);
void round_ui_reset(void);
void round_ui_pairing(uint32_t passkey);
// Layout and hit areas share these coordinates in the 320-square canvas.
enum { ROUND_SETTINGS_X=55, ROUND_SETTINGS_W=210, ROUND_SETTINGS_Y=90,
       ROUND_SETTINGS_STEP=52, ROUND_ROW_H=40, ROUND_ACTION_Y=220 };
