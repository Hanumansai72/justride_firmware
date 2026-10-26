#include "hud_ui.h"
#ifdef HUD_CAPTURE
#include "hud_capture_driver.h"
#else
#include "display_tft.h"
#endif
#include "assets/hud_fonts.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const hud_font_t *font(unsigned size) {
  switch (size) {
  case 8:
    return &hud_font_8;
  case 10:
    return &hud_font_10;
  case 12:
    return &hud_font_12;
  case 16:
    return &hud_font_16;
  case 22:
    return &hud_font_22;
  case 36:
    return &hud_font_36;
  default:
    return &hud_font_28;
  }
}
static uint16_t blend(uint16_t a, uint16_t b, unsigned n) {
  unsigned r = (((a >> 11) & 31) * (15 - n) + ((b >> 11) & 31) * n) / 15;
  unsigned g = (((a >> 5) & 63) * (15 - n) + ((b >> 5) & 63) * n) / 15;
  unsigned bl = ((a & 31) * (15 - n) + (b & 31) * n) / 15;
  return (uint16_t)((r << 11) | (g << 5) | bl);
}
int hud_text_width(const char *s, unsigned size) {
  int w = 0;
  const hud_font_t *f = font(size);
  for (; *s; s++) {
    unsigned c = (unsigned char)*s;
    if (c < 32 || c > 126)
      c = '?';
    w += f->glyphs[c - 32].advance;
  }
  return w;
}
void hud_text(int x, int y, const char *s, unsigned size, uint16_t fg,
              uint16_t bg, int max_width) {
  // Fixed row scratch: no framebuffer, heap allocation, or per-pixel SPI
  // writes.
  static uint8_t row[64];
  int end = x + max_width;
  const hud_font_t *f = font(size);
  for (; *s; s++) {
    unsigned c = (unsigned char)*s;
    if (c < 32 || c > 126)
      c = '?';
    const hud_glyph_t *g = &f->glyphs[c - 32];
    if (x + g->advance > end)
      break;
    for (unsigned yy = 0; yy < g->height; yy++) {
      for (unsigned xx = 0; xx < g->width; xx++) {
        unsigned i = yy * g->width + xx;
        uint8_t packed = f->pixels[g->offset + i / 2];
        uint16_t color = blend(bg, fg, (i & 1) ? packed & 15 : packed >> 4);
        row[xx * 2] = color >> 8;
        row[xx * 2 + 1] = color & 255;
      }
      if (g->width)
        tft_draw_raw_pixels(x + g->left, y + g->top + (int)yy, g->width, 1,
                            row);
    }
    x += g->advance;
  }
}
static void centered(int cx, int y, const char *s, unsigned size, uint16_t fg,
                     uint16_t bg) {
  hud_text(cx - hud_text_width(s, size) / 2, y, s, size, fg, bg, 320);
}
static void card(int x, int y, int w, int h, int radius, uint16_t border) {
  tft_fill_round_rect(x, y, w, h, radius, HUD_PANEL);
  tft_draw_round_rect(x, y, w, h, radius, border);
}
static void chevron(int x, int y, int size, uint16_t color) {
  tft_draw_line(x, y - size, x + size, y, color);
  tft_draw_line(x + size, y, x, y + size, color);
}
static void pump(int x, int y, uint16_t color) {
  tft_draw_round_rect(x, y, 8, 13, 1, color);
  tft_draw_rect(x + 2, y + 2, 4, 4, color);
  tft_draw_fast_h_line(x - 1, y + 13, 11, color);
  tft_draw_line(x + 9, y + 3, x + 12, y + 6, color);
  tft_draw_line(x + 12, y + 6, x + 12, y + 12, color);
  tft_draw_line(x + 12, y + 12, x + 9, y + 12, color);
}
static void wrench(int x, int y, uint16_t color) {
  tft_fill_circle(x + 10, y + 3, 4, color);
  tft_fill_triangle(x + 8, y - 2, x + 13, y - 2, x + 12, y + 3, HUD_PANEL);
  tft_draw_thick_line(x + 8, y + 6, x + 1, y + 13, 4, color);
  tft_fill_circle(x + 1, y + 13, 2, color);
}
static void flag(int x, int y) {
  tft_draw_fast_v_line(x, y, 14, HUD_TEXT);
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 4; c++)
      tft_fill_rect(x + 2 + c * 2, y + r * 2, 2, 2,
                    ((r + c) & 1) ? HUD_TEXT : HUD_BORDER);
}
static void arc(int cx, int cy, int radius, float start, float finish,
                int width, uint16_t color) {
  for (float a = start; a < finish; a += 4.0f) {
    float b = fminf(a + 4.0f, finish), r = 0.01745329252f;
    tft_draw_thick_line(cx + (int)lroundf(radius * cosf(a * r)),
                        cy + (int)lroundf(radius * sinf(a * r)),
                        cx + (int)lroundf(radius * cosf(b * r)),
                        cy + (int)lroundf(radius * sinf(b * r)), width, color);
  }
}
static void draw_speed(void *context) {
  const hud_model_t *m = context;
  card(3, 5, 88, 82, 6, HUD_BORDER);
  arc(41, 51, 30, 150, 390, 5, JR_BORDER);
  if (m->speed_valid)
    arc(41, 51, 30, 150, 150 + 240 * fminf(m->speed / 120.0f, 1), 5, HUD_ACCENT);
  char value[10];
  if (m->speed_valid)
    snprintf(value, sizeof(value), "%u", m->speed);
  else
    strcpy(value, "--");
  centered(41, 23, value, strlen(value) > 3 ? 22 : 36, HUD_TEXT, HUD_PANEL);
  centered(41, 60, "km/h", 10, HUD_MUTED, HUD_PANEL);
  tft_fill_circle(77, 24, 10, 0xF800);
  tft_fill_circle(77, 24, 8, 0xFFFF);
  if (m->speed_limit)
    snprintf(value, sizeof(value), "%u", m->speed_limit);
  else
    strcpy(value, "--");
  centered(77, 18, value, 10, 0x0000, 0xFFFF);
}
void hud_speed(const hud_model_t *m) {
  tft_draw_buffered_region(3, 5, 88, 82, HUD_BG, draw_speed, (void *)m);
}
void hud_poi(const hud_model_t *m, unsigned type) {
  if (type > 1)
    return;
  int y = type ? 132 : 93;
  card(3, y, 90, 35, 5, HUD_BORDER);
  if (type)
    wrench(10, y + 10, HUD_ACCENT);
  else
    pump(11, y + 9, HUD_ACCENT);
  hud_text(31, y + 4, type ? "Bike Repair" : "Petrol Bunk", 10, HUD_TEXT,
           HUD_PANEL, 56);
  char distance[18];
  if (!m->poi_valid[type])
    strcpy(distance, "-- km");
  else if (m->poi_distance[type] < 1000)
    snprintf(distance, sizeof(distance), "%u m", m->poi_distance[type]);
  else
    snprintf(distance, sizeof(distance), "%.1f km",
             m->poi_distance[type] / 1000.0f);
  hud_text(31, y + 17, distance, 12, HUD_MUTED, HUD_PANEL, 52);
  chevron(85, y + 18, 3, HUD_MUTED);
}
void hud_trip(const hud_model_t *m) {
  card(245, 4, 73, 34, 6, HUD_BORDER);
  flag(252, 14);
  hud_text(270, 7, m->duration[0] ? m->duration : "--", 10, HUD_TEXT, HUD_PANEL,
           40);
  char distance[20];
  snprintf(distance, sizeof(distance), "%.1f km", m->remaining_km);
  hud_text(270, 20, distance, 10, HUD_MUTED, HUD_PANEL, 42);
  chevron(311, 18, 3, HUD_MUTED);
}
static const char *instruction(const hud_model_t *m) {
  if (m->instruction[0])
    return m->instruction;
  switch (m->direction) {
  case 4:
    return "TURN LEFT";
  case 5:
    return "TURN RIGHT";
  case 2:
    return "BEAR LEFT";
  case 3:
    return "BEAR RIGHT";
  case 6:
    return "SHARP LEFT";
  case 7:
    return "SHARP RIGHT";
  case 8:
    return "MAKE U-TURN";
  case 9:
    return "ARRIVED";
  case 10:
    return "ROUNDABOUT";
  case 11:
    return "TAKE EXIT";
  case 12:
    return "MERGE";
  case 13:
    return "KEEP ROUTE";
  default:
    return "CONTINUE STRAIGHT";
  }
}
static void draw_maneuver(void *context) {
  const hud_model_t *m = context;
  card(3, 177, 314, 59, 12, HUD_ACCENT);
  // Reference's subtle geometric arrows stay behind the live instruction.
  for (int x = 243; x < 310; x += 20) {
    tft_fill_triangle(x, 180, x + 17, 206, x, 233, JR_ACCENT_DIM);
    tft_fill_triangle(x + 9, 180, x + 26, 206, x + 9, 233, HUD_PANEL);
  }
  card(7, 183, 67, 49, 9, HUD_ACCENT_DARK);
  tft_draw_round_rect(8, 184, 65, 47, 8, HUD_ACCENT);
  tft_draw_turn_arrow(39, 207, 31, (nav_direction_t)m->direction, HUD_TEXT);
  tft_draw_fast_v_line(82, 188, 37, HUD_ACCENT);
  char text[32];
  snprintf(text, sizeof(text), "%s", instruction(m));
  for (unsigned i = 0; text[i]; i++)
    text[i] = (char)toupper((unsigned char)text[i]);
  hud_text(96, 184, text, 10, HUD_MUTED, HUD_PANEL, 202);
  char value[20];
  const char *unit = "m";
  if (m->distance_m >= 1000) {
    snprintf(value, sizeof(value), "%.1f", m->distance_m / 1000.0f);
    unit = "km";
  } else
    snprintf(value, sizeof(value), "%lu", (unsigned long)m->distance_m);
  unsigned size = hud_text_width(value, 36) > 144 ? 28 : 36;
  hud_text(96, 195, value, size, HUD_ACCENT, HUD_PANEL, 156);
  hud_text(100 + hud_text_width(value, size), size == 36 ? 213 : 207, unit, 16,
           HUD_TEXT, HUD_PANEL, 34);
}
void hud_maneuver(const hud_model_t *m) {
  tft_draw_buffered_region(3, 177, 314, 59, HUD_BG, draw_maneuver, (void *)m);
}
void hud_compass(uint16_t heading) {
  // Keep the map provider's bottom attribution strip unobstructed.
  tft_fill_circle(303, 136, 12, HUD_BG);
  tft_draw_circle(303, 136, 12, HUD_ACCENT_DARK);
  centered(303, 124, "N", 8, HUD_TEXT, HUD_BG);
  float a = (heading % 360) * 0.01745329252f;
  int x = 303, y = 138;
  tft_fill_triangle(
      x + (int)(8 * sinf(a)), y - (int)(8 * cosf(a)),
      x + (int)(6 * sinf(a + 2.4f)), y - (int)(6 * cosf(a + 2.4f)),
      x + (int)(6 * sinf(a - 2.4f)), y - (int)(6 * cosf(a - 2.4f)), HUD_ACCENT);
}
void hud_notification(const char *app, const char *sender, const char *message,
                      unsigned remaining_ms) {
  card(100, 4, 135, 27, 6, HUD_BORDER);
  bool whatsapp = strstr(app, "WhatsApp") || strstr(app, "whatsapp");
  tft_fill_round_rect(107, 9, 14, 14, 3, whatsapp ? 0x07E8 : HUD_ACCENT);
  tft_draw_circle(114, 16, 5, HUD_TEXT);
  tft_draw_line(110, 20, 109, 22, HUD_TEXT);
  tft_draw_line(112, 14, 112, 16, HUD_TEXT);
  tft_draw_line(112, 16, 115, 18, HUD_TEXT);
  tft_draw_line(115, 18, 117, 17, HUD_TEXT);
  hud_text(124, 6, app[0] ? app : "Alert", 8, HUD_TEXT, HUD_PANEL, 92);
  char preview[96];
  snprintf(preview, sizeof(preview), "%s%s%s", sender, sender[0] ? ": " : "",
           message);
  hud_text(124, 16, preview, 8, HUD_MUTED, HUD_PANEL, 105);
  hud_text(224, 7, "2s", 8, HUD_MUTED, HUD_PANEL, 10);
  tft_fill_rect(104, 29,
                127 * (remaining_ms > 2000 ? 2000 : remaining_ms) / 2000, 1,
                HUD_ACCENT);
}
void hud_call(const char *caller, uint8_t actions, bool connected) {
  tft_fill_screen(HUD_BG);
  centered(160, 20, "INCOMING CALL", 16, HUD_TEXT, HUD_BG);
  tft_fill_circle(160, 81, 28, JR_PANEL_ALT);
  tft_fill_circle(160, 74, 10, HUD_MUTED);
  tft_fill_round_rect(142, 86, 36, 17, 8, HUD_MUTED);
  hud_text(20, 120, caller[0] ? caller : "Unknown caller", 16, HUD_TEXT, HUD_BG,
           280);
  centered(160, 148, connected ? "Phone call" : "Phone disconnected", 10,
           HUD_MUTED, HUD_BG);
  uint16_t accept = connected && (actions & 1) ? HUD_ACCENT : HUD_BORDER;
  uint16_t decline = connected && (actions & 2) ? JR_DANGER : HUD_BORDER;
  tft_fill_round_rect(18, 178, 133, 43, 10, accept);
  tft_fill_round_rect(169, 178, 133, 43, 10, decline);
  centered(84, 189, "ACCEPT", 12, HUD_TEXT, accept);
  centered(235, 189, "DECLINE", 12, HUD_TEXT, decline);
}
