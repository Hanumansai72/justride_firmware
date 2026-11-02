#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "display_theme.h"

// Native 320x240 coordinates, scaled from the rectified reference screen.
#define HUD_MAP_X 94
#define HUD_MAP_Y 0
#define HUD_MAP_W 226
#define HUD_MAP_H 177
#define HUD_BG JR_BG
#define HUD_PANEL JR_PANEL
#define HUD_BORDER JR_BORDER
#define HUD_ACCENT JR_PRIMARY
#define HUD_ACCENT_DARK JR_PRIMARY_DARK
#define HUD_MUTED JR_MUTED
#define HUD_TEXT JR_TEXT


typedef struct {
  uint16_t speed, speed_limit, heading;
  bool speed_valid;
  uint8_t direction;
  uint32_t distance_m;
  float remaining_km;
  char duration[24], instruction[32];
  bool poi_valid[2];
  uint16_t poi_distance[2];
} hud_model_t;

void hud_speed(const hud_model_t *m);
void hud_poi(const hud_model_t *m, unsigned type);
void hud_trip(const hud_model_t *m);
void hud_maneuver(const hud_model_t *m);
void hud_compass(uint16_t heading);
void hud_notification(const char *app, const char *sender, const char *message,
                      unsigned remaining_ms);
void hud_call(const char *caller, uint8_t actions, bool connected);
void hud_text(int x, int y, const char *text, unsigned size, uint16_t fg,
              uint16_t bg, int max_width);
int hud_text_width(const char *text, unsigned size);
// Use inside a buffered region to preserve the map underneath glyph edges.
void hud_text_overlay(int x,int y,const char *text,unsigned size,uint16_t fg,int max_width);
