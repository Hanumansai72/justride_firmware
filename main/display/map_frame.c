#include "map_frame.h"
#include "display_tft.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "hud_ui.h"
#include <math.h>
#include <string.h>

#define FRAME_BYTES (MAP_WIDTH * MAP_HEIGHT * 2)
static uint8_t *pixels;
static bool receiving, valid, dirty, on_screen, marker_dirty;
static uint16_t frame_id, sequence, marker_x = 99, marker_y = 97, heading;

static size_t offset;
static uint32_t expected_crc;
static int64_t started;
static uint16_t source_w = MAP_WIDTH, source_h = MAP_HEIGHT;
static uint32_t paint_serial;
static int screen_x, screen_y;
static bool marker_visible;
static int16_t tilt_degrees;
static uint32_t marker_serial;
static int previous_x, previous_y;
uint32_t map_frame_marker_serial(void) { return marker_serial; }
bool map_frame_marker_overlaps(int x, int y, int w, int h) {
  return (screen_x + 10 >= x && screen_x - 10 < x + w && screen_y + 10 >= y &&
          screen_y - 10 < y + h) ||
         (previous_x + 10 >= x && previous_x - 10 < x + w &&
          previous_y + 10 >= y && previous_y - 10 < y + h);
}
uint32_t map_frame_paint_serial(void) { return paint_serial; }
void map_frame_set_tilt(float roll_deg) {
  if (!isfinite(roll_deg))
    return;
  int16_t next = (int16_t)lroundf(roll_deg / 2.0f) * 2;
  if (next != tilt_degrees) {
    tilt_degrees = next;
    marker_dirty = true;
  }
}
static uint8_t status;
static uint32_t last_chunk_crc;
static uint16_t u16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }

void map_frame_invalidate(void) {
  receiving = false;
  valid = false;
  dirty = false;
  status = 0;
}
bool map_frame_active(void) { return receiving || valid || on_screen; }
const uint8_t *map_frame_snapshot(unsigned *w,unsigned *h,unsigned *x,unsigned *y) {
  if(!valid || receiving)return NULL;
  *w=source_w;*h=source_h;*x=marker_x;*y=marker_y;return pixels;
}
const char *map_frame_status(void) {
  return status == 0   ? "MAP LOAD"
         : status == 2 ? "MAP ERROR"
         : status == 3 ? "MAP QUOTA"
         : status == 4 ? "MAP EDGE"
                       : "N";
}

bool map_frame_packet(const uint8_t *p, size_t len) {
  if (len == 2 && p[0] == 0x19 && p[1] <= 4) {
    status = p[1];
    return true;
  }
  if (len < 3)
    return false;
  uint16_t id = u16(p + 1);
  switch (p[0]) {
  case 0x14: // START, id, width/height, CRC32 of decoded RGB565 bytes
    if (len != 11)
      return false;
    uint16_t w = u16(p + 3), h = u16(p + 5);
    if (!((w == MAP_WIDTH && h == MAP_HEIGHT) || (w == 198 && h == 194)))
      return false;
    source_w = w;
    source_h = h;
    if (!pixels) {
      if (heap_caps_get_free_size(MALLOC_CAP_8BIT) < FRAME_BYTES + 32768)
        return false;
      pixels = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_8BIT);
      if (!pixels)
        return false;
    }
    frame_id = id;
    sequence = 0;
    offset = 0;
    expected_crc = p[7] | ((uint32_t)p[8] << 8) | ((uint32_t)p[9] << 16) |
                   ((uint32_t)p[10] << 24);
    receiving = true;
    valid = false;
    dirty = false;
    status = 0;
    started = esp_timer_get_time();
    return true;
  case 0x15: { // DATA: id, sequence, (run length, RGB565 high, low)...
    if (!receiving || id != frame_id || len < 8 || (len - 5) % 3)
      return false;
    uint16_t seq = u16(p + 3);
    // App retries identical ATT writes only. Never advance twice on retry.
    uint32_t chunk_crc = esp_rom_crc32_le(0, p, len);
    if (sequence && seq == (uint16_t)(sequence - 1))
      return chunk_crc == last_chunk_crc;
    if (seq != sequence)
      return false;
    size_t expanded = 0;
    for (size_t i = 5; i < len; i += 3) {
      if (!p[i])
        return false;
      expanded += p[i] * 2;
    }
    if (offset + expanded > (size_t)source_w * source_h * 2)
      return false;
    for (size_t i = 5; i < len; i += 3)
      for (unsigned n = 0; n < p[i]; n++) {
        pixels[offset++] = p[i + 1];
        pixels[offset++] = p[i + 2];
      }
    sequence++;
    last_chunk_crc = chunk_crc;
    return true;
  }
  case 0x16: // COMMIT: incomplete/corrupt images never reach the display
    if (len != 3 || !receiving || id != frame_id)
      return false;
    receiving = false;
    if (offset != (size_t)source_w * source_h * 2 ||
        esp_rom_crc32_le(0, pixels, offset) != expected_crc) {
      status = 2;
      return false;
    }
    valid = true;
    paint_serial++;
    dirty = true;
    status = 1;
    marker_x = source_w / 2;
    marker_y = source_h / 2;
    marker_dirty = true;
    ESP_LOGI("map", "frame accepted: raw=%u transfer_us=%lld heap=%u",
             FRAME_BYTES, esp_timer_get_time() - started,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    return true;
  case 0x17:
    if (len != 3 || id != frame_id)
      return false;
    receiving = false;
    return true;
  case 0x18: // marker in the exact downloaded north-up Web Mercator viewport
    if (len != 9 || !valid || receiving || id != frame_id)
      return false;
    if (u16(p + 3) < 11 || u16(p + 3) >= source_w - 11 || u16(p + 5) < 11 ||
        u16(p + 5) >= source_h - 11 || u16(p + 7) >= 360)
      return false;
    if (marker_x == u16(p + 3) && marker_y == u16(p + 5) &&
        heading == u16(p + 7))
      return true;
    marker_x = u16(p + 3);
    marker_y = u16(p + 5);
    heading = u16(p + 7);
    marker_dirty = true;
    marker_serial++;
    if (status == 4)
      status = 1;
    return true;
  default:
    return false;
  }
}

// Retain the small marker background independently of an incoming frame.
// The last GPS marker can remain visible while the next frame is received.
static uint8_t marker_patch[21 * 21 * 2];
static void restore_marker_patch(void) {
  if (marker_visible)
    tft_draw_raw_pixels(screen_x - 10, screen_y - 10, 21, 21, marker_patch);
  marker_visible = false;
  previous_x = screen_x;
  previous_y = screen_y;
  marker_serial++;
}
static void draw_marker(void) {
  float a = (heading + tilt_degrees) * 0.01745329252f;
  int x = screen_x, y = screen_y;
  tft_fill_triangle(
      x + (int)(9 * sinf(a)), y - (int)(9 * cosf(a)),
      x + (int)(8 * sinf(a + 2.4f)), y - (int)(8 * cosf(a + 2.4f)),
      x + (int)(8 * sinf(a - 2.4f)), y - (int)(8 * cosf(a - 2.4f)), HUD_ACCENT);
  tft_fill_triangle(
      x + (int)(7 * sinf(a)), y - (int)(7 * cosf(a)),
      x + (int)(5 * sinf(a + 2.4f)), y - (int)(5 * cosf(a + 2.4f)),
      x + (int)(5 * sinf(a - 2.4f)), y - (int)(5 * cosf(a - 2.4f)), HUD_TEXT);
  marker_visible = true;
  marker_dirty = false;
  marker_serial++;
}
bool map_frame_draw(bool force) {
  if (force)
    on_screen = false;
  if (receiving || !valid) {
    if (on_screen && marker_visible && marker_dirty) {
      restore_marker_patch();
      draw_marker();
    }
    return on_screen;
  }
  if (dirty || !on_screen) {
    int64_t t = esp_timer_get_time();
    // The incoming rows already cover this viewport. Never blank it first.
    if (source_w < MAP_WIDTH) {
      int margin = (MAP_WIDTH - source_w) / 2;
      tft_fill_rect(HUD_MAP_X, HUD_MAP_Y, margin, HUD_MAP_H, HUD_BG);
      tft_fill_rect(HUD_MAP_X + margin + source_w, HUD_MAP_Y,
                    MAP_WIDTH - margin - source_w, HUD_MAP_H, HUD_BG);
    }
    int crop = source_h > MAP_HEIGHT ? (source_h - MAP_HEIGHT) / 2 : 0;
    int left = HUD_MAP_X + (MAP_WIDTH - source_w) / 2;
    for (int y = 0; y < MAP_HEIGHT; y++)
      tft_draw_raw_pixels(
          left, HUD_MAP_Y + y, source_w, 1,
          pixels +
              ((y >= MAP_HEIGHT - 24 ? source_h - MAP_HEIGHT + y : y + crop) *
               source_w) *
                  2);
    paint_serial++;
    ESP_LOGI("map", "blit_us=%lld", esp_timer_get_time() - t);
    on_screen = true;
    dirty = false;
    marker_dirty = true;
    marker_visible = false;
  } else if (marker_dirty)
    restore_marker_patch();
  if (marker_dirty) {
    uint16_t drawn_x = marker_x, drawn_y = marker_y;

    screen_x = HUD_MAP_X + (MAP_WIDTH - source_w) / 2 + drawn_x;
    screen_y = HUD_MAP_Y + drawn_y -
               (source_h > MAP_HEIGHT ? (source_h - MAP_HEIGHT) / 2 : 0);
    if (screen_y < HUD_MAP_Y + 10 ||
        screen_y >= HUD_MAP_Y + HUD_MAP_H - 24 - 10) {
      marker_dirty = false;
      status = 4;
      return true;
    }
    for (int y = 0; y < 21; y++)
      memcpy(marker_patch + y * 42,
             pixels + ((drawn_y - 10 + y) * source_w + drawn_x - 10) * 2, 42);
    draw_marker();
  }
  return true;
}

void map_frame_restore_region(int x, int y, int w, int h) {
  int left = HUD_MAP_X + (MAP_WIDTH - source_w) / 2;
  int crop = source_h > MAP_HEIGHT ? (source_h - MAP_HEIGHT) / 2 : 0;
  tft_fill_rect(x, y, w, h, HUD_BG);
  if (!valid || receiving)
    return;
  for (int yy = y; yy < y + h; yy++) {
    int local_y = yy - HUD_MAP_Y;
    int sx = x - left, sy = local_y >= MAP_HEIGHT - 24
                                ? source_h - MAP_HEIGHT + local_y
                                : local_y + crop;
    int width = w;
    if (sx < 0) {
      width += sx;
      sx = 0;
    }
    if (sx + width > source_w)
      width = source_w - sx;
    if (width > 0 && sy >= 0 && sy < source_h)
      tft_draw_raw_pixels(left + sx, yy, width, 1,
                          pixels + (sy * source_w + sx) * 2);
  }
  paint_serial++;
}
