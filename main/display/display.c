#include "mpu6500.h"
#include "map_frame.h"
#include "hud_ui.h"
#include "display.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "ble.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "rtc_time.h"
#include "splash_assets.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "DISPLAY";

static bool s_is_initialized = false;
static screen_id_t s_current_screen =
    SCREEN_COUNT; // Initialize to SCREEN_COUNT during boot/splash
static SemaphoreHandle_t s_display_mutex = NULL;

// Dynamic brightness state (0-100%)
static uint8_t s_brightness_pct = 80;

// Dynamic system state
static uint8_t s_time_hours = 14;
static uint8_t s_time_minutes = 5;
static bool s_ble_connected = false;
static bool s_phone_state_ready = false;
typedef struct { uint8_t state,actions; char caller[48]; } call_info_t;
static call_info_t s_call;
static bool s_call_painted;
static QueueHandle_t s_call_queue;
static StaticQueue_t s_call_queue_storage;
static uint8_t s_call_queue_bytes[sizeof(call_info_t)];
void display_update_call(uint8_t state,uint8_t actions,const char *caller) {
  if(!s_call_queue)return;
  call_info_t call={.state=state,.actions=actions};
  snprintf(call.caller,sizeof(call.caller),"%s",caller?caller:"");
  xQueueOverwrite(s_call_queue,&call);
}
static uint16_t s_speed_kmh =
    0; // Live vehicle speed (km/h) — 0 when stationary

// =============================================================================
// IMU HEADING OVERLAY (MPU6500 Gyro Integration)
// Updated by display_set_imu_heading() called from mpu6500.c imu_task.
// This heading rotates the nav turn arrow to match the bike's physical turn.
// It does NOT modify s_nav (BLE/GPS data) — purely a visual overlay.
// =============================================================================
static float          s_imu_ref_heading    = -1.0f;  // -1 = not yet set
static bool           s_imu_heading_valid  = false;  // True once first IMU packet
static nav_direction_t s_imu_arrow_override = NAV_DIR_NONE; // Active IMU arrow dir

// Reset Device confirmation modal state
static bool s_show_reset_confirm = false;
// Dedicated Brightness popup modal state (Master Prompt §28)
static bool s_show_brightness_popup = false;
// On-Screen Route Resume modal state (when power/BLE/GPS reconnects during
// active route)
static bool s_show_route_resume_modal = false;
static bool s_nav_was_active = false;
static bool s_nav_interrupted = false;

// Latest Notification State (Strictly holds ONLY the most recent notification)
typedef struct {
  char title[32];
  char message[64];
  char app[24];
  bool active;
} notification_info_t;

static notification_info_t s_latest_notif = {
    .title = "",
    .message = "",
    .active = false,
};
static bool s_notif_dirty = true;
static int64_t s_notif_started_us;
static TimerHandle_t s_notif_timer = NULL;
static volatile bool s_notif_dismiss_requested;

// Gallery / User Image state (Strict SINGLE image support)
static bool s_user_image_valid = false;

// Forward route animation step

// Dynamic Color Palette for Day & Night Themes
typedef struct {
  uint16_t bg;        // Canvas background
  uint16_t card_bg;   // Overlay card background
  uint16_t border;    // Card and road borders
  uint16_t route;     // Active navigation road corridor (ALWAYS BLUE)
  uint16_t text_pri;  // Primary high-contrast text
  uint16_t text_sec;  // Subtitles and distance units
  uint16_t speed_num; // Speedometer numeric readout
  uint16_t pointer;   // Vehicle GPS arrowhead (GREEN/WHITE)
} theme_palette_t;

// Day mode & Night mode (Uniform Black & White High Contrast)
static const theme_palette_t s_theme_day = {
    .bg = COLOR_BLACK,           // Pure Black background
    .card_bg = COLOR_BLACK,      // Pure Black card background
    .border = COLOR_DARKGREY,    // Dark grey border
    .route = COLOR_WHITE,        // Pure White route corridor
    .text_pri = COLOR_WHITE,     // White text
    .text_sec = COLOR_LIGHTGREY, // Light grey secondary
    .speed_num = COLOR_WHITE,    // White speed
    .pointer = COLOR_WHITE,      // White navigation chevron
};

static const theme_palette_t s_theme_night = {
    .bg = COLOR_BLACK,           // Pure Black background
    .card_bg = COLOR_BLACK,      // Pure Black card background
    .border = COLOR_DARKGREY,    // Dark grey border
    .route = COLOR_WHITE,        // Pure White route corridor
    .text_pri = COLOR_WHITE,     // White primary text
    .text_sec = COLOR_LIGHTGREY, // Light grey secondary
    .speed_num = COLOR_WHITE,    // Crisp white speed readout
    .pointer = COLOR_WHITE,      // White GPS arrowhead
};

static inline bool is_night_mode(void) {
  return (s_time_hours >= 18 || s_time_hours < 6);
}

static inline const theme_palette_t *get_active_theme(void) {
  return is_night_mode() ? &s_theme_night : &s_theme_day;
}

// Active Navigation Guidance State
static navigation_data_t s_nav = {
    .direction = NAV_DIR_STRAIGHT,
    .distance_meters = 0,
    .current_street = "Ready to Ride",
    .next_direction = NAV_DIR_STRAIGHT,
    .next_distance_meters = 0,
    .next_street = "Continue Straight",
    .eta_time_str = "--",
    .remaining_dist_km = 0.0f,
    .speed_kmh = 0,
    .heading_deg = 0,
    .is_navigating = false,
    .gps_available = true,
    .rerouting = false,
};

// Active Sliding 500m Route Window
static route_segment_t s_route_seg = {
    .count = 0,
    .is_destination_visible = false,
    .dest_x = 0,
    .dest_y = 0,
};
static nav_context_t s_nav_context = {0};
static bool s_route_seg_dirty = true;
static bool s_map_drawn = false;

// Navigation Camera & Zoom Engine
static nav_camera_mode_t s_camera_mode = NAV_CAMERA_FOLLOW;
static nav_zoom_level_t s_zoom_level = NAV_ZOOM_LEVEL_2; // Closer live follow view
static int16_t s_pan_offset_x = 0;
static int16_t s_pan_offset_y = 0;

// Wrong Turn / Deviation State Tracker
typedef enum {
  WRONG_TURN_NONE = 0,
  WRONG_TURN_DETECTED,
  WRONG_TURN_REROUTING,
} wrong_turn_state_t;

// Live Navigation Persistent HUD Cache & Smoothing State
typedef struct {
  bool is_frame_active;           // Static HUD frame has been drawn
  uint32_t last_dist_m;           // Distance in meters
  uint16_t last_speed;            // Vehicle speed km/h
  nav_direction_t last_arrow_dir; // Maneuver arrow enum
  uint16_t last_arrow_color;      // Maneuver arrow color
  char last_turn_instruction[16]; // Text instruction beside the maneuver arrow
  char last_street[32];           // Street name
  char last_dist_rem[24];         // Remaining distance text
  char last_eta[24];              // ETA text
  float last_remaining_dist_km;   // Last rendered remaining distance
  uint8_t last_route_progress_pct;
  bool last_gps_available;        // Satellite lock status
  bool last_rerouting;            // Rerouting status
  wrong_turn_state_t last_wrong_turn_state; // Last wrong-turn warning state
  float current_heading_rad; // Smoothly interpolated heading in radians
  bool destination_reached;  // Arrival screen active
  int16_t last_px;           // Last rendered pointer X
  int16_t last_py;           // Last rendered pointer Y
  float current_px;          // Continuous animated pointer X coordinate
  float current_py;          // Continuous animated pointer Y coordinate
  float current_angle;       // Continuous animated pointer angle in radians
  int16_t last_box_x0;       // Bounding box of previous chevron (X0)
  int16_t last_box_y0;       // Bounding box of previous chevron (Y0)
  int16_t last_box_x1;       // Bounding box of previous chevron (X1)
  int16_t last_box_y1;       // Bounding box of previous chevron (Y1)
  nav_direction_t last_drawn_junction_dir; // Direction of currently rendered
                                           // junction corridor
  bool
      last_show_maneuver_junction; // Whether maneuver junction was drawn (<30m)
  uint32_t last_corridor_dist_m;   // Distance when road corridor was last drawn
} nav_hud_cache_t;

static nav_hud_cache_t s_hud = {
    .is_frame_active = false,
    .last_dist_m = 0xFFFFFFFF,
    .last_speed = 0xFFFF,
    .last_arrow_dir = (nav_direction_t)0xFF,
    .last_arrow_color = 0,
    .last_turn_instruction = {0},
    .last_street = {0},
    .last_dist_rem = {0},
    .last_eta = {0},
    .last_remaining_dist_km = -1.0f,
    .last_gps_available = true,
    .last_rerouting = false,
    .last_wrong_turn_state = WRONG_TURN_NONE,
    .current_heading_rad = 0.0f,
    .destination_reached = false,
    .last_px = 160,
    .last_py = -1,
    .current_px = 160.0f,
    .current_py = 145.0f,
    .current_angle = 0.0f,
    .last_box_x0 = -1,
    .last_box_y0 = -1,
    .last_box_x1 = -1,
    .last_box_y1 = -1,
    .last_drawn_junction_dir = NAV_DIR_NONE,
    .last_show_maneuver_junction = false,
    .last_corridor_dist_m = 0xFFFFFFFF,
};

// Forward declaration of internal refreshers
static QueueHandle_t s_nav_queue, s_media_queue, s_route_queue, s_context_queue;
static QueueHandle_t s_connection_queue, s_notification_queue;
static StaticQueue_t s_connection_queue_storage, s_notification_queue_storage;
static uint8_t s_connection_queue_bytes[sizeof(bool)];
static uint8_t s_notification_queue_bytes[sizeof(notification_info_t)];
static void apply_notification(const notification_info_t *notification);
static StaticQueue_t s_route_queue_storage, s_context_queue_storage;
static uint8_t s_route_queue_bytes[sizeof(route_segment_t)];
static uint8_t s_context_queue_bytes[4 * sizeof(nav_context_t)];
static StaticQueue_t s_nav_queue_storage, s_media_queue_storage;
static uint8_t s_nav_queue_bytes[sizeof(navigation_data_t)];
static uint8_t s_media_queue_bytes[sizeof(media_state_t)];
static media_state_t s_media = {.state = 6};
static bool s_show_motion_settings;
static bool s_media_dirty = true, s_media_frame = false, s_screen_refresh = false;
static int64_t s_last_media_command_us;
static void apply_navigation(const navigation_data_t *nav_data);

void display_update_media(const media_state_t *state) {
  if (s_media_queue && state) xQueueOverwrite(s_media_queue, state);
}

static void render_music(void) {
  if (!s_media_frame) {
    tft_fill_screen(COLOR_BLACK);
    tft_draw_string(14, 12, "MUSIC", COLOR_JUST_GREEN, COLOR_BLACK, 2);
    tft_draw_string(184, 16, "SWIPE FOR NAV", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    s_media_frame = true;
    s_media_dirty = true;
  }
  if (!s_media_dirty) return;
  s_media_dirty = false;
  tft_fill_rect(8, 40, 304, 190, COLOR_BLACK);
  const char *status = !s_ble_connected ? "PHONE DISCONNECTED" :
      s_media.state == 0 ? "ANDROID APP REQUIRED" :
      s_media.state == 6 ? "WAITING FOR PHONE MEDIA" :
      s_media.state == 7 ? "RECONNECT PHONE MEDIA" :
      s_media.state == 1 ? "ENABLE NOTIFICATION ACCESS" :
      s_media.state == 2 ? "OPEN A MUSIC APP ON PHONE" :
      s_media.state == 4 ? "PLAYING" : s_media.state == 3 ? "PAUSED" : "PLAYER READY";
  tft_draw_string(14, 43, status, COLOR_JUST_GREEN, COLOR_BLACK, 1);
  char title[25] = {0};
  snprintf(title, sizeof(title), "%.24s", s_ble_connected ? s_media.title : "");
  tft_draw_string(14, 65, title, COLOR_WHITE, COLOR_BLACK, 2);
  tft_draw_string(14, 91, s_ble_connected ? s_media.artist : "", COLOR_LIGHTGREY, COLOR_BLACK, 1);
  bool active = s_ble_connected && s_media.state >= 3 && s_media.state <= 5;
  uint16_t prev_fg = active && (s_media.actions & 8) ? COLOR_JUST_GREEN : COLOR_DARKGREY;
  uint16_t play_fg = active && (s_media.actions & (s_media.state == 4 ? 2 : 1)) ? COLOR_JUST_GREEN : COLOR_DARKGREY;
  uint16_t next_fg = active && (s_media.actions & 4) ? COLOR_JUST_GREEN : COLOR_DARKGREY;
  uint16_t volume_fg = active && (s_media.actions & 16) ? COLOR_JUST_GREEN : COLOR_DARKGREY;
  tft_draw_round_rect(10,118,94,60,8,prev_fg);
  tft_draw_round_rect(113,118,94,60,8,play_fg);
  tft_draw_round_rect(216,118,94,60,8,next_fg);
  tft_draw_string_centered(57, 140, "PREV", prev_fg, COLOR_BLACK, 2);
  tft_draw_string_centered(160, 140, s_media.state == 4 ? "PAUSE" : "PLAY", play_fg, COLOR_BLACK, 2);
  tft_draw_string_centered(263, 140, "NEXT", next_fg, COLOR_BLACK, 2);
  tft_draw_round_rect(10, 187, 94, 43, 6, volume_fg);
  tft_draw_round_rect(216, 187, 94, 43, 6, volume_fg);
  tft_draw_string_centered(57, 200, "VOL -", volume_fg, COLOR_BLACK, 2);
  tft_draw_string_centered(263, 200, "VOL +", volume_fg, COLOR_BLACK, 2);
  if (s_ble_connected && !active) {
    tft_fill_rect(10, 118, 300, 60, COLOR_BLACK);
    tft_draw_round_rect(10,118,300,60,8,COLOR_JUST_GREEN);
    tft_draw_string_centered(160,140,s_media.state == 1 ? "ENABLE ON PHONE" : "RETRY MEDIA",COLOR_JUST_GREEN,COLOR_BLACK,1);
  }
  char volume[12];
  snprintf(volume, sizeof(volume), "%u%%", s_media.volume);
  tft_draw_string_centered(160, 200, volume, COLOR_WHITE, COLOR_BLACK, 2);
}

static void display_nav_render_partial(void);
static void update_brightness_widgets(void);
static void render_notification_banner(void);
void display_show_brightness_screen(void);

// Keep the maneuver explicit as well as graphical.  The arrow alone is easy
// to miss on a vibrating motorcycle display, especially at the turn itself.
static __attribute__((unused)) const char *
turn_instruction_label(nav_direction_t direction, uint8_t maneuver_state) {
  const bool turn_now = (maneuver_state == 4) ||
                        (s_nav.distance_meters <= 14 &&
                         s_nav.distance_meters > 0); // TURN_ACTIVE / TURNING

  switch (direction) {
  case NAV_DIR_LEFT:
    return turn_now ? "TAKING LEFT" : "LEFT TURN";
  case NAV_DIR_RIGHT:
    return turn_now ? "TAKING RIGHT" : "RIGHT TURN";
  case NAV_DIR_SLIGHT_LEFT:
    return turn_now ? "BEAR LEFT" : "SLIGHT LEFT";
  case NAV_DIR_SLIGHT_RIGHT:
    return turn_now ? "BEAR RIGHT" : "SLIGHT RIGHT";
  case NAV_DIR_SHARP_LEFT:
    return turn_now ? "SHARP LEFT" : "SHARP LEFT";
  case NAV_DIR_SHARP_RIGHT:
    return turn_now ? "SHARP RIGHT" : "SHARP RIGHT";
  case NAV_DIR_UTURN:
    return turn_now ? "TAKING U-TURN" : "U-TURN";
  default:
    return "";
  }
}

// The phone deliberately reports STRAIGHT until a maneuver enters its 100m
// approach zone.  Keep the road drawing on the same rule, even when the next
// 500m polyline already contains a later corner.
__attribute__((unused)) static bool is_turn_approach_visible(void) {
  return s_nav.direction != NAV_DIR_NONE &&
         s_nav.direction != NAV_DIR_STRAIGHT &&
         s_nav.direction != NAV_DIR_REACHED && s_nav.distance_meters <= 100;
}

// =============================================================================
// BOTTOM PAGINATION DOTS: < . . * . . > (Y: 226 to 238)
// =============================================================================
static void render_pagination(screen_id_t active_id) {
  int y_center = 232;

  // Clear the pagination strip
  tft_fill_rect(80, 224, 160, 16, COLOR_BLACK);

  // Left chevron '<'
  tft_draw_char(96, 226, '<', COLOR_LIGHTGREY, COLOR_BLACK, 1);

  // 5 dots centered around X=160
  int dot_centers[5] = {136, 148, 160, 172, 184};
  for (int i = 0; i < 5; i++) {
    if (i == (int)active_id) {
      tft_fill_circle(dot_centers[i], y_center, 4, COLOR_WHITE);
    } else {
      tft_fill_circle(dot_centers[i], y_center, 2, COLOR_DARKGREY);
    }
  }

  // Right chevron '>'
  tft_draw_char(218, 226, '>', COLOR_LIGHTGREY, COLOR_BLACK, 1);
}

// Helper: Glowing Winding Route Icon (from official brand logo asset)
static void draw_splash_route_icon(int cx, int cy, int size, uint16_t color) {
  int spacing = size / 2;
  int x1 = cx - spacing;
  int x2 = cx;
  int x3 = cx + spacing;
  int r = spacing / 2;
  int y_top = cy - size / 2;
  int y_bot = cy + size / 2;
  int th = 2;

  // 3 vertical legs
  tft_fill_rect(x1 - th / 2, y_top + r, th, y_bot - y_top - 2 * r, color);
  tft_fill_rect(x2 - th / 2, y_top + r, th, y_bot - y_top - 2 * r, color);
  tft_fill_rect(x3 - th / 2, y_top + r, th, y_bot - y_top - 2 * r, color);

  // Bottom bend (between x1 and x2)
  int cx_bend1 = (x1 + x2) / 2;
  int cy_bend1 = y_bot - r;
  for (int deg = 0; deg <= 180; deg += 10) {
    float rad = deg * 3.14159f / 180.0f;
    int px = cx_bend1 - (int)(cosf(rad) * r);
    int py = cy_bend1 + (int)(sinf(rad) * r);
    tft_fill_circle(px, py, th, color);
  }

  // Top bend (between x2 and x3)
  int cx_bend2 = (x2 + x3) / 2;
  int cy_bend2 = y_top + r;
  for (int deg = 0; deg <= 180; deg += 10) {
    float rad = deg * 3.14159f / 180.0f;
    int px = cx_bend2 - (int)(cosf(rad) * r);
    int py = cy_bend2 - (int)(sinf(rad) * r);
    tft_fill_circle(px, py, th, color);
  }

  // Terminal waypoint dots
  tft_fill_circle(x1, y_top + r, th + 1, color);
  tft_fill_circle(x3, y_bot - r, th + 1, color);
}



// =============================================================================
// SCREEN: ANIMATED BOOT SPLASH SCREEN (320x240) - OFFICIAL BRAND IDENTITY
// =============================================================================

// Helper: Fast horizontal span blitter for 1-bit bitmap masks
static void draw_splash_bitmap_1bit(int x, int y, int w, int h,
                                    const uint8_t *bitmap, uint16_t color) {
  if (!bitmap || w <= 0 || h <= 0)
    return;
  int stride = (w + 7) / 8;
  for (int r = 0; r < h; r++) {
    int span_start = -1;
    for (int c = 0; c < w; c++) {
      uint8_t byte = bitmap[r * stride + (c / 8)];
      bool is_set = (byte & (0x80 >> (c % 8))) != 0;
      if (is_set) {
        if (span_start < 0) {
          span_start = c;
        }
      } else {
        if (span_start >= 0) {
          tft_draw_fast_h_line(x + span_start, y + r, c - span_start, color);
          span_start = -1;
        }
      }
    }
    if (span_start >= 0) {
      tft_draw_fast_h_line(x + span_start, y + r, w - span_start, color);
    }
  }
}

// Helper: Topographic contour lines across lower portion of screen
static void draw_splash_topographic_lines(void) {
  const struct {
    int offset;
    float freq1;
    float freq2;
    int amp1;
    int amp2;
    float phase;
  } topo_waves[] = {
      {234, 0.024f, 0.048f, 7, 3, 0.5f},  {225, 0.022f, 0.038f, 10, 5, 1.2f},
      {214, 0.019f, 0.045f, 13, 6, 2.1f}, {204, 0.021f, 0.035f, 9, 4, 0.8f},
      {194, 0.016f, 0.032f, 6, 3, 3.0f},  {185, 0.025f, 0.042f, 5, 2, 1.7f}};

  for (int w = 0; w < 6; w++) {
    int prev_x = 0;
    int prev_y = topo_waves[w].offset +
                 (int)(topo_waves[w].amp1 * sinf(topo_waves[w].phase) +
                       topo_waves[w].amp2 * cosf(topo_waves[w].phase));
    for (int x = 2; x < DISPLAY_WIDTH; x += 2) {
      float rad1 = x * topo_waves[w].freq1 + topo_waves[w].phase;
      float rad2 = x * topo_waves[w].freq2 + topo_waves[w].phase;
      int y = topo_waves[w].offset + (int)(topo_waves[w].amp1 * sinf(rad1) +
                                           topo_waves[w].amp2 * cosf(rad2));
      if (y >= 0 && y < DISPLAY_HEIGHT && prev_y >= 0 &&
          prev_y < DISPLAY_HEIGHT) {
        tft_draw_line(prev_x, prev_y, x, y, COLOR_SPLASH_TOPO);
      }
      prev_x = x;
      prev_y = y;
    }
  }
}

void display_show_splash(void) {
  if (!s_is_initialized)
    display_init();

  ESP_LOGI(TAG,
           "Rendering JustRide Motorcycle Splash Screen (MADE FOR BIKERS)...");

  // Acquire display mutex to lock out all background tasks during splash
  // animation
  if (s_display_mutex) {
    xSemaphoreTakeRecursive(s_display_mutex, portMAX_DELAY);
  }

  // Hold background rendering tasks at bay
  s_current_screen = SCREEN_COUNT;

  // 1. Clean screen canvas to pure deep black
  tft_fill_screen(COLOR_BLACK);

  // 2. Subtle topographic contour curves across bottom
  draw_splash_topographic_lines();

  // 3. Logo & Brand Wordmark ("just" in white, "ride" in neon green with
  // diamond dot)
  const int icon_w = SPLASH_LOGO_ICON_WIDTH;
  const int icon_gap = 10;
  const int just_w = SPLASH_LOGO_JUST_WIDTH;
  const int ride_w = SPLASH_LOGO_RIDE_WIDTH;
  const int logo_y = 62;

#if SPLASH_SHOW_ROUTE_ICON
  const int total_logo_w = icon_w + icon_gap + just_w + ride_w;
  const int start_x = (DISPLAY_WIDTH - total_logo_w) / 2;
  draw_splash_route_icon(start_x + icon_w / 2,
                         logo_y + SPLASH_LOGO_JUST_HEIGHT / 2, 22,
                         COLOR_NEON_GREEN);
  const int x_just = start_x + icon_w + icon_gap;
#else
  const int total_logo_w = just_w + ride_w;
  const int x_just = (DISPLAY_WIDTH - total_logo_w) / 2;
#endif

  draw_splash_bitmap_1bit(x_just, logo_y, just_w, SPLASH_LOGO_JUST_HEIGHT,
                          splash_logo_just_bits, COLOR_WHITE);
  draw_splash_bitmap_1bit(x_just + just_w, logo_y, ride_w,
                          SPLASH_LOGO_RIDE_HEIGHT, splash_logo_ride_bits,
                          COLOR_NEON_GREEN);

  // 4. Tagline "MADE FOR BIKERS"
  tft_draw_string_centered(DISPLAY_WIDTH / 2, 106,
                           "M A D E   F O R   B I K E R S", COLOR_WHITE,
                           COLOR_BLACK, 1);

  // 5. Sleek Progress Bar Architecture (Pre-rendered before turning on
  // backlight)
  const int bar_w = 190;
  const int bar_h = 4;
  const int bar_x = (DISPLAY_WIDTH - bar_w) / 2;
  const int bar_y = 142;
  tft_fill_rect(bar_x, bar_y, bar_w, bar_h, COLOR_SPLASH_BAR_BG);

  // 6. Slogan "GEAR UP. RIDE ON."
  tft_draw_string_centered(DISPLAY_WIDTH / 2, 166,
                           "G E A R   U P .   R I D E   O N .", COLOR_LIGHTGREY,
                           COLOR_BLACK, 1);

  // Raise backlight cleanly now that full splash frame is rendered (zero
  // glitch/noise)
  tft_set_backlight(s_brightness_pct);

  // 7. Smooth neon-green progress bar sweep animation (0% to 100%)
  for (int p = 5; p <= 100; p += 5) {
    int fill = (p * bar_w) / 100;
    if (fill > 0) {
      tft_fill_rect(bar_x, bar_y, fill, bar_h, COLOR_NEON_GREEN);
    }
    vTaskDelay(pdMS_TO_TICKS(35));
  }

  vTaskDelay(pdMS_TO_TICKS(180));

  // Release mutex before switching screen
  if (s_display_mutex) {
    xSemaphoreGiveRecursive(s_display_mutex);
  }

  display_set_screen(SCREEN_HOME);
}

// =============================================================================
// SCREEN 0: HOME SCREEN (TIME SCREEN)
// Pure black background, massive centered clock (HH:MM where Hours are White,
// Colon is White, Minutes are JustRide Green #22E875), Date centered below
// (WED, 09 SEP 2026), zero clutter (no top status bar, no battery/BLE, no
// buttons).
// =============================================================================
static void draw_home_modern_digit(int x, int y, char digit, uint16_t color) {
  const int w = 42;
  const int h = 66;
  const int stroke = 10;
  const int mid_y = y + h / 2;
  const int half_s = stroke / 2;

  if (digit == '1') {
    tft_fill_rect(x + w - stroke, y, stroke, h, color);
    tft_fill_rect(x + w - stroke - 10, y + 4, 10, stroke, color);
    return;
  }

  // 7-segment bitmask: a(0), b(1), c(2), d(3), e(4), f(5), g(6)
  static const uint8_t seg_table[10] = {
      0x3F, // 0: a b c d e f
      0x06, // 1: b c
      0x5B, // 2: a b d e g
      0x4F, // 3: a b c d g
      0x66, // 4: b c f g
      0x6D, // 5: a c d f g
      0x7D, // 6: a c d e f g
      0x07, // 7: a b c
      0x7F, // 8: a b c d e f g
      0x6F, // 9: a b c d f g
  };

  if (digit < '0' || digit > '9')
    return;
  uint8_t mask = seg_table[digit - '0'];

  if (mask & (1 << 0))
    tft_fill_rect(x, y, w, stroke, color); // a: top
  if (mask & (1 << 1))
    tft_fill_rect(x + w - stroke, y, stroke, (mid_y + half_s) - y,
                  color); // b: top-right
  if (mask & (1 << 2))
    tft_fill_rect(x + w - stroke, mid_y - half_s, stroke,
                  (y + h) - (mid_y - half_s), color); // c: bot-right
  if (mask & (1 << 3))
    tft_fill_rect(x, y + h - stroke, w, stroke, color); // d: bot
  if (mask & (1 << 4))
    tft_fill_rect(x, mid_y - half_s, stroke, (y + h) - (mid_y - half_s),
                  color); // e: bot-left
  if (mask & (1 << 5))
    tft_fill_rect(x, y, stroke, (mid_y + half_s) - y, color); // f: top-left
  if (mask & (1 << 6))
    tft_fill_rect(x, mid_y - half_s, w, stroke, color); // g: mid
}

static void draw_home_screen_clock(void) {
  // Clear central clock and date area cleanly: X: 0..319, Y: 50..220
  tft_fill_rect(0, 50, DISPLAY_WIDTH, 170, COLOR_BLACK);

  const int x0 = 43;
  const int y0 = 68;

  char h_tens = '0' + (s_time_hours / 10);
  char h_units = '0' + (s_time_hours % 10);
  char m_tens = '0' + (s_time_minutes / 10);
  char m_units = '0' + (s_time_minutes % 10);

  // Hours in Pure White
  draw_home_modern_digit(x0, y0, h_tens, COLOR_WHITE);
  draw_home_modern_digit(x0 + 42 + 12, y0, h_units, COLOR_WHITE);

  // Colon in Pure White (two square dots)
  int cx = x0 + 42 * 2 + 12 + 14;
  tft_fill_rect(cx, y0 + 16, 10, 10, COLOR_WHITE);
  tft_fill_rect(cx, y0 + 40, 10, 10, COLOR_WHITE);

  // Minutes in Vivid JustRide Neon Green (#22E875)
  int x_min = cx + 10 + 14;
  draw_home_modern_digit(x_min, y0, m_tens, COLOR_NEON_GREEN);
  draw_home_modern_digit(x_min + 42 + 12, y0, m_units, COLOR_NEON_GREEN);

  // Centered Date string below in White (scale 2)
  char date_str[32] = {0};
  time_get_date_str(date_str, sizeof(date_str));
  if (date_str[0] == '\0') {
    strncpy(date_str, "WED, 09 SEP 2026", sizeof(date_str) - 1);
  }
  tft_draw_string_centered(DISPLAY_WIDTH / 2, 166, date_str, COLOR_WHITE,
                           COLOR_BLACK, 2);
}

void display_show_home_screen(void) {
  if (!s_is_initialized)
    display_init();
  s_current_screen = SCREEN_HOME;

  // Pure black background with zero clutter (no top status bar, no battery, no
  // BLE, no buttons)
  tft_fill_screen(COLOR_BLACK);
  draw_home_screen_clock();

  // Latest Notification Overlay (if active)
  render_notification_banner();
}

// Native reference composition: 94px left rail, 226x177 map, 59px maneuver panel.


// Medium maneuver: centered arrow, bounded distance, one instruction line.




// Trip footer: fixed columns keep distance and duration from pushing each other.


// Navigation-map colors are local so the surrounding HUD keeps its theme.
#define COLOR_NAV_MAP_BG    HUD_BG
#define COLOR_NAV_ROUTE_HALO JR_PRIMARY_DARK
#define COLOR_NAV_ROUTE      JR_PRIMARY

// ── 4. ROTATED RIDER CHEVRON (Large Map Area: cx=220, cy=146)
// ─────────────
static void draw_hud_rider_chevron(float angle_rad) {
  const float cx = 207.0f;
  const float cy = 148.0f;
  const float cos_a = cosf(angle_rad);
  const float sin_a = sinf(angle_rad);

  // Prominent 3D Delta-wing arrowhead
  // Tip: (0, -18), Left wing: (-15, +11), Notch: (0, +3), Right wing: (+15,
  // +11)
  int16_t x0 = (int16_t)roundf(cx + (18.0f * sin_a));
  int16_t y0 = (int16_t)roundf(cy - (18.0f * cos_a));

  int16_t x1 = (int16_t)roundf(cx - (15.0f * cos_a) - (11.0f * sin_a));
  int16_t y1 = (int16_t)roundf(cy - (15.0f * sin_a) + (11.0f * cos_a));

  int16_t x2 = (int16_t)roundf(cx - (3.0f * sin_a));
  int16_t y2 = (int16_t)roundf(cy + (3.0f * cos_a));

  int16_t x3 = (int16_t)roundf(cx + (15.0f * cos_a) - (11.0f * sin_a));
  int16_t y3 = (int16_t)roundf(cy + (15.0f * sin_a) + (11.0f * cos_a));

  // Emerald outline and off-white body match the website palette.
  tft_draw_thick_line(x0, y0, x1, y1, 3, COLOR_NAV_ROUTE);
  tft_draw_thick_line(x1, y1, x2, y2, 3, COLOR_NAV_ROUTE);
  tft_draw_thick_line(x2, y2, x3, y3, 3, COLOR_NAV_ROUTE);
  tft_draw_thick_line(x3, y3, x0, y0, 3, COLOR_NAV_ROUTE);

  // High-contrast off-white core
  tft_fill_triangle(x0, y0, x1, y1, x2, y2, COLOR_WHITE);
  tft_fill_triangle(x0, y0, x2, y2, x3, y3, COLOR_WHITE);

  tft_draw_line(x0, y0, x2, y2, COLOR_NAV_ROUTE);
}

// ── 5. LARGE HEADING-UP ROUTE MAP (X: 122..319, Y: 4..198)
// Visual motion affects only the rider pointer, never the route instruction,
// GPS speed or map/compass heading. Sampling remains on the 50 Hz sensor task.
static float rider_motion_degrees(void) {
  if(!mpu6500_is_available() || !mpu6500_is_calibration_complete())return 0;
  if(mpu6500_is_gyro_available()) {
    float angle=mpu6500_get_heading_deg()-s_imu_ref_heading;
    if(angle>180)angle-=360;
    if(angle< -180)angle+=360;
    return angle;
  }
  if(s_imu_arrow_override==NAV_DIR_UTURN)return 180;
  if(s_imu_arrow_override==NAV_DIR_NONE || s_imu_arrow_override==NAV_DIR_STRAIGHT)return 0;
  return fmaxf(-90,fminf(90,mpu6500_get_roll_deg()));
}
static int s_drawn_motion_degrees=1000;
typedef struct { int x, y, w, h; } route_region_t;
static void draw_route_region(void *context);
static void paint_route_region(int x, int y, int w, int h) {
  for (int row = y; row < y + h; row += 8) {
    route_region_t region = {x, row, w, y + h - row > 8 ? 8 : y + h - row};
    tft_draw_buffered_region(x, row, w, region.h, HUD_BG, draw_route_region, &region);
  }
}
static void draw_hud_rider_marker(nav_direction_t dir) {
  (void)dir;
  s_drawn_motion_degrees=(int)lroundf(rider_motion_degrees()/2)*2;
  // Restore the actual road beneath the marker; never punch a blank hole in it.
  paint_route_region(184, 125, 47, 47);
}
static int route_zoom_percent(void) {
  switch (s_zoom_level) {
  case NAV_ZOOM_LEVEL_1: return 150;
  case NAV_ZOOM_LEVEL_2: return 100;
  case NAV_ZOOM_LEVEL_3: return 75;
  case NAV_ZOOM_LEVEL_4: return 50;
  default: return 35;
  }
}
static int16_t route_map_x(uint16_t x) {
  return 207 + ((int)x - 160) * 9 * route_zoom_percent() / 1000;
}
static int16_t route_map_y(uint16_t y) {
  return 148 + ((int)y - 160) * 9 * route_zoom_percent() / 1000;
}
// Clip whole segments; clamping each endpoint creates false boundary roads.
static bool clip_route_line(float *x, float *y, float *xx, float *yy,
                            const route_region_t *r) {
  float dx=*xx-*x, dy=*yy-*y, lo=0, hi=1;
  float p[4]={-dx,dx,-dy,dy};
  float q[4]={*x-r->x+5,r->x+r->w+4-*x,*y-r->y+5,r->y+r->h+4-*y};
  for(int i=0;i<4;i++) {
    if(p[i]==0){if(q[i]<0)return false;continue;}
    float t=q[i]/p[i];
    if(p[i]<0){if(t>lo)lo=t;}else if(t<hi)hi=t;
    if(lo>hi)return false;
  }
  *xx=*x+hi*dx;*yy=*y+hi*dy;*x+=lo*dx;*y+=lo*dy;
  return true;
}
static void draw_route_region(void *context) {
  const route_region_t *region=context;
  for(uint8_t i=1;i<s_route_seg.count;i++) {
    float x=route_map_x(s_route_seg.points[i-1].x), y=route_map_y(s_route_seg.points[i-1].y);
    float xx=route_map_x(s_route_seg.points[i].x), yy=route_map_y(s_route_seg.points[i].y);
    if(!clip_route_line(&x,&y,&xx,&yy,region))continue;
    bool past=i<=s_route_seg.completed_segment_count;
    tft_draw_thick_line(lroundf(x),lroundf(y),lroundf(xx),lroundf(yy),past?5:7,COLOR_NAV_ROUTE_HALO);
    tft_draw_thick_line(lroundf(x),lroundf(y),lroundf(xx),lroundf(yy),past?2:4,past?HUD_MUTED:COLOR_NAV_ROUTE);
    if(!past)tft_draw_thick_line(lroundf(x),lroundf(y),lroundf(xx),lroundf(yy),1,COLOR_WHITE);
  }
  if(s_route_seg.count>=2 && s_route_seg.is_destination_visible)
    tft_fill_circle(route_map_x(s_route_seg.dest_x),route_map_y(s_route_seg.dest_y),4,COLOR_NAV_ROUTE);
  if(s_route_seg.count<2)hud_text(145,70,"WAITING FOR ROUTE",10,HUD_MUTED,HUD_BG,170);
  draw_hud_rider_chevron(s_drawn_motion_degrees*0.01745329252f);
}
static void draw_hud_road_corridor(nav_direction_t dir, uint32_t dist_m) {
  (void)dir; (void)dist_m;
  if (map_frame_draw(!s_map_drawn)) {
    s_map_drawn=true;s_route_seg_dirty=false;return;
  }
  s_drawn_motion_degrees=(int)lroundf(rider_motion_degrees()/2)*2;
  // Compose strips before SPI transfer, without clearing the visible map first.
  paint_route_region(HUD_MAP_X,HUD_MAP_Y,HUD_MAP_W,HUD_MAP_H);
  if (!s_ble_connected || !s_phone_state_ready || !s_nav.gps_available || s_nav.rerouting) {
    const char *warning = !s_ble_connected ? "PHONE OFFLINE" : !s_phone_state_ready ?
        "SYNCING" : s_nav.gps_available ? "REROUTING" : "GPS LOST";
    hud_text(132,7,warning,10,HUD_MUTED,HUD_BG,150);
  }
  s_map_drawn=true;s_route_seg_dirty=false;
}

// =============================================================================
// HELPER: BRIGHTNESS HUD OVERLAY GAUGE
// =============================================================================
// =============================================================================
// HELPER: LATEST NOTIFICATION HUD BANNER OVERLAY (Y: 34 to 88, X: 12 to 308)
// Displays strictly ONLY the latest incoming notification with auto/tap dismiss
// =============================================================================
static void render_notification_banner(void) {
  if (!s_latest_notif.active || s_current_screen == SCREEN_OTA || s_call.state==1) return;
  unsigned elapsed=(unsigned)((esp_timer_get_time()-s_notif_started_us)/1000);
  unsigned remaining=elapsed>=2000?0:2000-elapsed;
  if (s_notif_dirty) {
    hud_notification(s_latest_notif.app,s_latest_notif.title,s_latest_notif.message,remaining);
    s_notif_dirty=false;
  } else {
    tft_fill_rect(104,29,127,1,HUD_PANEL);
    tft_fill_rect(104,29,127*remaining/2000,1,HUD_ACCENT);
  }
}

// =============================================================================
// HELPER: ON-SCREEN ROUTE RESUME MODAL
// Shown when BLE reconnects after an interrupted route, or on GPS recovery
// =============================================================================
static void render_route_resume_modal(void) {
  if (!s_show_route_resume_modal)
    return;

  // Center Modal window
  tft_fill_rect(20, 35, 280, 170, COLOR_BLACK);
  tft_draw_rect(20, 35, 280, 170, COLOR_WHITE);
  tft_draw_rect(21, 36, 278, 168, COLOR_DARKGREY);

  tft_draw_string_centered(160, 50, "RESUME ROUTE?", COLOR_WHITE, COLOR_BLACK,
                           2);
  tft_draw_string_centered(160, 80, "Connection restored.", COLOR_LIGHTGREY,
                           COLOR_BLACK, 1);
  tft_draw_string_centered(160, 98, "Continue to destination?", COLOR_LIGHTGREY,
                           COLOR_BLACK, 1);

  // [ CONTINUE ] Button: X: 35..150 (W: 115)
  tft_fill_rect(35, 130, 115, 40, COLOR_WHITE);
  tft_draw_string_centered(92, 142, "CONTINUE", COLOR_BLACK, COLOR_WHITE, 1);

  // [ CANCEL ] Button: X: 170..285 (W: 115)
  tft_fill_rect(170, 130, 115, 40, COLOR_BLACK);
  tft_draw_rect(170, 130, 115, 40, COLOR_DARKGREY);
  tft_draw_string_centered(227, 142, "CANCEL", COLOR_WHITE, COLOR_BLACK, 1);
}

void display_show_resume_route_modal(void) {
  if (s_display_mutex &&
      xSemaphoreTakeRecursive(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    s_show_route_resume_modal = true;
    screen_id_t screen = s_current_screen;
    xSemaphoreGiveRecursive(s_display_mutex);
    s_screen_refresh = true;
    display_set_screen(screen);
  }
}

void display_dismiss_resume_route_modal(void) {
  if (s_display_mutex &&
      xSemaphoreTakeRecursive(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    s_show_route_resume_modal = false;
    screen_id_t screen = s_current_screen;
    xSemaphoreGiveRecursive(s_display_mutex);
    s_screen_refresh = true;
    display_set_screen(screen);
  }
}

bool display_is_route_interrupted(void) { return s_nav_interrupted; }

// =============================================================================
// SCREEN 1: PERSISTENT LIVE NAVIGATION HUD SCREEN (MONOCHROME MOTORCYCLE HUD)
// Master Enhancement: Live, Non-Refreshing TFT Navigation (Google Maps Style)
//
// The screen is initialized ONCE on entering navigation. Full-screen clear
// (tft_fill_screen) is STRICTLY PROHIBITED during navigation. Dynamic elements
// (distance, speed, maneuver arrow, street name, road corridor, chevron)
// are updated via dirty-region partial redraws without screen flicker.
// =============================================================================


static hud_model_t s_last_hud;
#define ARRIVAL_DISPLAY_US (10LL * 1000 * 1000)
static int64_t s_arrival_started_us;
static nav_context_t s_pois[2];
static uint32_t s_last_map_paint,s_last_marker_paint;
static void display_nav_render_partial(void) {
  if (s_current_screen != SCREEN_NAV) return;
  if (s_arrival_started_us && esp_timer_get_time() - s_arrival_started_us >= ARRIVAL_DISPLAY_US) {
    s_arrival_started_us = 0;
    display_show_home_screen();
    return;
  }
  if (!s_nav.is_navigating && s_nav.direction != NAV_DIR_REACHED) {
    display_show_home_screen(); return;
  }
  hud_model_t model = {0};
  model.speed=s_speed_kmh; model.speed_valid=s_ble_connected && s_phone_state_ready && s_nav.gps_available;
  model.heading=s_nav.heading_deg; model.direction=s_nav.direction; model.distance_m=s_nav.distance_meters;
  model.remaining_km=s_nav.remaining_dist_km;
  snprintf(model.duration,sizeof(model.duration),"%s",s_nav.eta_time_str);
  const char *instruction=s_nav.current_street;
  if(s_nav.is_navigating) {
    if(!s_ble_connected)instruction="PHONE OFFLINE";
    else if(!s_phone_state_ready)instruction="SYNCING";
    else if(!s_nav.gps_available)instruction="GPS LOST";
    else if(s_nav.rerouting)instruction="REROUTING";
  }
  snprintf(model.instruction,sizeof(model.instruction),"%s",instruction);
  for(int i=0;i<2;i++){model.poi_valid[i]=s_pois[i].poi_type==i+1 && s_pois[i].poi_name[0];model.poi_distance[i]=s_pois[i].poi_distance_m;}
  bool first=!s_hud.is_frame_active;
  if(first){tft_fill_screen(HUD_BG);s_map_drawn=false;s_route_seg_dirty=true;}
  map_frame_set_tilt(rider_motion_degrees());
  bool route_painted=s_route_seg_dirty && !map_frame_active();
  if(s_route_seg_dirty)draw_hud_road_corridor(NAV_DIR_STRAIGHT,s_nav.distance_meters);
  else {
    map_frame_draw(false);
    if(!map_frame_active() && s_drawn_motion_degrees!=(int)lroundf(rider_motion_degrees()/2)*2)
      draw_hud_rider_marker(NAV_DIR_STRAIGHT);
  }
  bool map_changed=first || route_painted || map_frame_paint_serial()!=s_last_map_paint;
  s_last_map_paint=map_frame_paint_serial();
  bool marker_changed=map_frame_marker_serial()!=s_last_marker_paint;
  s_last_marker_paint=map_frame_marker_serial();
  if(first || model.speed!=s_last_hud.speed || model.speed_valid!=s_last_hud.speed_valid)hud_speed(&model);
  for(int i=0;i<2;i++)if(first || model.poi_valid[i]!=s_last_hud.poi_valid[i] || model.poi_distance[i]!=s_last_hud.poi_distance[i])hud_poi(&model,i);
  if(first || map_changed || (marker_changed && map_frame_marker_overlaps(245,4,73,34)) || model.remaining_km!=s_last_hud.remaining_km || strcmp(model.duration,s_last_hud.duration))hud_trip(&model);
  if(first || model.direction!=s_last_hud.direction || model.distance_m!=s_last_hud.distance_m || strcmp(model.instruction,s_last_hud.instruction))hud_maneuver(&model);
  if(first || map_changed || (marker_changed && map_frame_marker_overlaps(290,123,26,26)) || model.heading!=s_last_hud.heading)hud_compass(model.heading);
  if(map_changed || (marker_changed && map_frame_marker_overlaps(100,4,135,27)))s_notif_dirty=true;
  s_last_hud=model; s_hud.last_dist_m=model.distance_m; s_hud.is_frame_active=true;
  render_notification_banner(); render_route_resume_modal();
}

void display_show_main_screen(void) {
  if (!s_is_initialized)
    display_init();
  s_current_screen = SCREEN_NAV;
  s_hud.is_frame_active = false;
  s_hud.destination_reached = false;
  display_nav_render_partial();
}

// =============================================================================
// SCREEN 3: PICTURE / GALLERY SCREEN
// Photo viewer, Counter "3 / 24", Navigation chevrons, Trash can icon
// =============================================================================
#define USER_IMG_MAGIC 0x4A52494D                                  // "JRIM"
#define USER_IMG_TOTAL_BYTES (DISPLAY_WIDTH * DISPLAY_HEIGHT * 2u) // 153600

static const esp_partition_t *s_user_img_part = NULL;
static size_t s_stream_flash_offset = 0;
static volatile bool s_image_refresh_requested;

static const esp_partition_t *get_user_img_partition(void) {
  if (!s_user_img_part) {
    s_user_img_part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x99, "user_img");
    if (!s_user_img_part) {
      s_user_img_part = esp_partition_find_first(
          ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "user_img");
    }
  }
  return s_user_img_part;
}

bool display_has_user_image(void) {
  if (s_user_image_valid)
    return true;
  const esp_partition_t *part = get_user_img_partition();
  if (!part)
    return false;
  uint32_t header[2] = {0};
  if (part->size >= 4096u + USER_IMG_TOTAL_BYTES &&
      esp_partition_read(part, 0, header, sizeof(header)) == ESP_OK) {
    if (header[0] == USER_IMG_MAGIC && header[1] == USER_IMG_TOTAL_BYTES) {
      s_user_image_valid = true;
      return true;
    }
  }
  return false;
}

bool display_start_image_stream(void) {
  if (!s_is_initialized) display_init();
  xSemaphoreTakeRecursive(s_display_mutex,portMAX_DELAY);
  // Flash erase must not interrupt an active ride or OTA rendering.
  if (s_nav.is_navigating || s_current_screen == SCREEN_OTA) {
    xSemaphoreGiveRecursive(s_display_mutex);
    return false;
  }
  const esp_partition_t *part = get_user_img_partition();
  const size_t image_end = 4096u + USER_IMG_TOTAL_BYTES;
  const size_t erase_size = (image_end + 4095u) & ~(size_t)4095u;
  if (!part || erase_size > part->size ||
      esp_partition_erase_range(part, 0, erase_size) != ESP_OK) {
    xSemaphoreGiveRecursive(s_display_mutex);
    return false;
  }
  s_user_image_valid = false;
  s_stream_flash_offset = 4096;
  s_screen_refresh = true;
  display_set_screen(SCREEN_PICTURE);
  xSemaphoreGiveRecursive(s_display_mutex);
  return true;
}

bool display_write_image_chunk(const uint8_t *chunk, size_t len) {
  if (!chunk || !len || (len & 1) || s_stream_flash_offset < 4096 ||
      len > 4096 + USER_IMG_TOTAL_BYTES - s_stream_flash_offset) return false;
  const esp_partition_t *part = get_user_img_partition();
  if (!part || esp_partition_write(part, s_stream_flash_offset, chunk, len) != ESP_OK) return false;
  s_stream_flash_offset += len;
  // Store only. Painting these chunks here races swipes and can overwrite
  // Music/Home/Settings after the user leaves the photo page.
  return true;
}

bool display_finish_image_stream(void) {
  if (s_stream_flash_offset != 4096 + USER_IMG_TOTAL_BYTES) return false;
  const esp_partition_t *part = get_user_img_partition();
  uint32_t header[2] = {USER_IMG_MAGIC, USER_IMG_TOTAL_BYTES};
  if (!part || esp_partition_write(part, 0, header, sizeof(header)) != ESP_OK) return false;
  s_user_image_valid = true;
  s_image_refresh_requested = true;
  return true;
}

void display_show_user_image(void) {
  if (!s_is_initialized)
    display_init();
  s_current_screen = SCREEN_PICTURE;

  tft_fill_screen(COLOR_BLACK);

  // 1. Picture Frame Card (X=8, Y=30, W=304, H=192)
  tft_fill_rect(8, 30, 304, 192, COLOR_BLACK);
  tft_draw_rect(8, 30, 304, 192, COLOR_DARKGREY);

  if (display_has_user_image()) {
    // Stream raw RGB565 from flash
    const esp_partition_t *part = get_user_img_partition();
    if (part) {
      tft_stream_image_start();
      uint8_t read_buf[512];
      size_t offset = 4096;
      size_t remaining = USER_IMG_TOTAL_BYTES;
      while (remaining > 0) {
        size_t chunk =
            (remaining > sizeof(read_buf)) ? sizeof(read_buf) : remaining;
        if (esp_partition_read(part, offset, read_buf, chunk) != ESP_OK) {
          ESP_LOGE(TAG,"Photo read failed at offset %u",(unsigned)offset);
          tft_fill_screen(COLOR_BLACK);
          tft_draw_string_centered(160,110,"IMAGE UNAVAILABLE",COLOR_WHITE,COLOR_BLACK,2);
          break;
        }
        tft_stream_image_chunk(read_buf, chunk);
        offset += chunk;
        remaining -= chunk;
      }
    }
  } else {
    // Clean empty state when no image exists
    tft_draw_string_centered(160, 110, "NO IMAGE UPLOADED", COLOR_WHITE,
                             COLOR_BLACK, 2);
    tft_draw_string_centered(160, 140, "Upload an image from the app",
                             COLOR_LIGHTGREY, COLOR_BLACK, 1);
  }

  // Draw page identity and navigation AFTER the full-screen image. Even an
  // entirely black uploaded photo must not look like an unresponsive display.
  tft_fill_rect(0,0,DISPLAY_WIDTH,27,COLOR_BLACK);
  tft_draw_string(12,9,"PHOTO",COLOR_JUST_GREEN,COLOR_BLACK,1);
  tft_draw_string(176,9,"SWIPE TO CHANGE",COLOR_LIGHTGREY,COLOR_BLACK,1);
  render_pagination(SCREEN_PICTURE);

  // Latest Notification Overlay (if active)
  render_notification_banner();
}

// =============================================================================
// SCREEN 4: SETTINGS SCREEN
// List items: Bluetooth, Display, System, Restart Device, Reset Device
// =============================================================================
static void update_brightness_widgets(void) {
  char bright_val_str[8];
  snprintf(bright_val_str, sizeof(bright_val_str), "%d%%", s_brightness_pct);
  tft_fill_rect(240, 80, 50, 18, COLOR_BLACK);
  tft_draw_string(240, 80, bright_val_str, COLOR_WHITE, COLOR_BLACK, 1);
}

static void render_motion_settings(bool force) {
  static int last_roll = 10000, last_pitch = 10000, last_motion = -1;
  static char last_status[32];
  const char *status = mpu6500_get_status();
  int roll = (int)mpu6500_get_roll_deg(), pitch = (int)mpu6500_get_pitch_deg();
  int motion = mpu6500_get_motion();
  if (!force && roll == last_roll && pitch == last_pitch && motion == last_motion && !strcmp(status,last_status)) return;
  if (force) {
    tft_fill_rect(24,35,272,181,COLOR_BLACK);
    tft_draw_rect(24,35,272,181,COLOR_JUST_GREEN);
    tft_draw_string_centered(160,47,"ACCELEROMETER",COLOR_WHITE,COLOR_BLACK,2);
    tft_draw_round_rect(36,158,158,42,5,COLOR_JUST_GREEN);
    tft_draw_string_centered(115,174,"CALIBRATE",COLOR_WHITE,COLOR_BLACK,1);
    tft_draw_round_rect(204,158,80,42,5,COLOR_LIGHTGREY);
    tft_draw_string_centered(244,174,"DONE",COLOR_WHITE,COLOR_BLACK,1);
  }
  tft_fill_rect(32,75,256,76,COLOR_BLACK);
  tft_draw_string_centered(160,78,status,COLOR_JUST_GREEN,COLOR_BLACK,1);
  char angles[34]; snprintf(angles,sizeof(angles),"Roll %+d  Pitch %+d",roll,pitch);
  tft_draw_string_centered(160,100,angles,COLOR_WHITE,COLOR_BLACK,1);
  const char *motions[] = {"LEVEL","FORWARD","BACKWARD","LEFT","RIGHT"};
  tft_draw_string_centered(160,120,mpu6500_is_calibration_complete() && !strcmp(status,"READY") ? motions[motion >=0 && motion <=4 ? motion:0] : "Keep device still, then calibrate",COLOR_WHITE,COLOR_BLACK,1);
  tft_draw_string_centered(160,138,"Tilt is not GPS speed or a route turn",COLOR_LIGHTGREY,COLOR_BLACK,1);
  last_roll=roll; last_pitch=pitch; last_motion=motion;
  snprintf(last_status,sizeof(last_status),"%s",status);
}

void display_show_settings_screen(void) {
  if (!s_is_initialized)
    display_init();
  s_current_screen = SCREEN_SETTINGS;

  tft_fill_screen(COLOR_BLACK);

  // 1. Bluetooth List Card (Y = 32 to 68)
  tft_fill_rect(10, 32, DISPLAY_WIDTH - 20, 36, COLOR_BLACK);
  tft_draw_rect(10, 32, DISPLAY_WIDTH - 20, 36, COLOR_DARKGREY);
  // BLE Icon
  tft_draw_char(24, 44, 'B', COLOR_WHITE, COLOR_BLACK, 2);
  tft_draw_string(46, 44, "Bluetooth", COLOR_WHITE, COLOR_BLACK, 1);
  tft_draw_string(216, 44, s_ble_connected ? "Connected" : "Searching",
                  s_ble_connected ? COLOR_WHITE : COLOR_LIGHTGREY, COLOR_BLACK,
                  1);
  tft_draw_char(292, 44, '>', COLOR_LIGHTGREY, COLOR_BLACK, 1);

  // 2. Display / Brightness List Card (Y = 72 to 108)
  tft_fill_rect(10, 72, DISPLAY_WIDTH - 20, 36, COLOR_BLACK);
  tft_draw_rect(10, 72, DISPLAY_WIDTH - 20, 36, COLOR_DARKGREY);
  // Sun / Brightness Icon
  tft_draw_circle(28, 90, 6, COLOR_WHITE);
  tft_draw_string(46, 84, "Display", COLOR_WHITE, COLOR_BLACK, 1);
  char bright_val_str[8];
  snprintf(bright_val_str, sizeof(bright_val_str), "%d%%", s_brightness_pct);
  tft_draw_string(240, 84, bright_val_str, COLOR_WHITE, COLOR_BLACK, 1);
  tft_draw_char(292, 84, '>', COLOR_LIGHTGREY, COLOR_BLACK, 1);

  // 3. System Info List Card (Y = 112 to 148)
  tft_fill_rect(10, 112, DISPLAY_WIDTH - 20, 36, COLOR_BLACK);
  tft_draw_rect(10, 112, DISPLAY_WIDTH - 20, 36, COLOR_DARKGREY);
  // Gear Icon
  tft_draw_circle(28, 130, 6, COLOR_WHITE);
  tft_draw_circle(28, 130, 2, COLOR_BLACK);
  tft_draw_string(46, 124, "Motion / Calibrate", COLOR_WHITE, COLOR_BLACK, 1);
  tft_draw_string(224, 124, mpu6500_is_calibration_complete() ? "Ready" : "Setup", COLOR_LIGHTGREY, COLOR_BLACK, 1);
  tft_draw_char(292, 124, '>', COLOR_LIGHTGREY, COLOR_BLACK, 1);

  // 4. Two Bottom Action Buttons (Y = 154 to 216)
  // Left: "Restart Device" (X=10, W=145)
  tft_fill_rect(10, 158, 145, 54, COLOR_BLACK);
  tft_draw_rect(10, 158, 145, 54, COLOR_DARKGREY);
  // Refresh / reload circle arrow
  tft_draw_circle(32, 185, 8, COLOR_WHITE);
  tft_draw_string(46, 180, "Restart Device", COLOR_WHITE, COLOR_BLACK, 1);

  // Right: "Reset Device" (X=165, W=145)
  tft_fill_rect(165, 158, 145, 54, COLOR_BLACK);
  tft_draw_rect(165, 158, 145, 54, COLOR_DARKGREY);
  // Trash bin icon
  tft_draw_rect(182, 180, 12, 12, COLOR_LIGHTGREY);
  tft_draw_fast_h_line(180, 178, 16, COLOR_LIGHTGREY);
  tft_draw_string(200, 180, "Reset Device", COLOR_WHITE, COLOR_BLACK, 1);

  // Confirmation Modal for "Reset Device" (Monochrome)
  if (s_show_reset_confirm) {
    // Modal Backdrop Card
    tft_fill_rect(30, 60, 260, 120, COLOR_BLACK);
    tft_draw_rect(30, 60, 260, 120, COLOR_WHITE);
    tft_draw_rect(31, 61, 258, 118, COLOR_DARKGREY);

    tft_draw_string_centered(160, 75, "RESET DEVICE?", COLOR_WHITE, COLOR_BLACK,
                             2);
    tft_draw_string_centered(160, 100, "Reset settings to defaults?",
                             COLOR_LIGHTGREY, COLOR_BLACK, 1);

    // Cancel Button: X: 45..140, Y: 125..160
    tft_fill_rect(45, 125, 95, 36, COLOR_BLACK);
    tft_draw_rect(45, 125, 95, 36, COLOR_DARKGREY);
    tft_draw_string_centered(92, 137, "CANCEL", COLOR_WHITE, COLOR_BLACK, 1);

    // Confirm Button: X: 175..270, Y: 125..160
    tft_fill_rect(175, 125, 95, 36, COLOR_WHITE);
    tft_draw_rect(175, 125, 95, 36, COLOR_WHITE);
    tft_draw_string_centered(222, 137, "RESET", COLOR_BLACK, COLOR_WHITE, 1);
  }

  // Dedicated Centered Brightness Popup Modal (Monochrome)
  if (s_show_brightness_popup) {
    // Centered Modal Window: X: 24, Y: 36, W: 272, H: 172
    tft_fill_rect(24, 36, 272, 172, COLOR_BLACK);
    tft_draw_rect(24, 36, 272, 172, COLOR_WHITE);
    tft_draw_rect(25, 37, 270, 170, COLOR_DARKGREY);

    tft_draw_string_centered(160, 50, "BRIGHTNESS", COLOR_WHITE, COLOR_BLACK,
                             2);

    // Large Readout: e.g. "70%"
    char b_str[16];
    snprintf(b_str, sizeof(b_str), "%d%%", s_brightness_pct);
    tft_draw_string_centered(160, 80, b_str, COLOR_WHITE, COLOR_BLACK, 3);

    // [ - ] Button: X: 44..96, Y: 114..152 (W=52, H=38)
    tft_fill_rect(44, 114, 52, 38, COLOR_BLACK);
    tft_draw_rect(44, 114, 52, 38, COLOR_DARKGREY);
    tft_draw_fast_h_line(60, 133, 20, COLOR_WHITE);
    tft_draw_fast_h_line(60, 134, 20, COLOR_WHITE);

    // [ + ] Button: X: 224..276, Y: 114..152 (W=52, H=38)
    tft_fill_rect(224, 114, 52, 38, COLOR_BLACK);
    tft_draw_rect(224, 114, 52, 38, COLOR_DARKGREY);
    tft_draw_fast_h_line(240, 133, 20, COLOR_WHITE);
    tft_draw_fast_h_line(240, 134, 20, COLOR_WHITE);
    tft_draw_fast_v_line(250, 123, 20, COLOR_WHITE);
    tft_draw_fast_v_line(251, 123, 20, COLOR_WHITE);

    // Interactive Track/Slider in center: X: 110..210, Y: 132
    tft_draw_fast_h_line(110, 132, 100, COLOR_DARKGREY);
    tft_draw_fast_h_line(110, 133, 100, COLOR_DARKGREY);
    int thumb_x = 110 + (s_brightness_pct * 100) / 100;
    if (thumb_x > 210)
      thumb_x = 210;
    tft_fill_circle(thumb_x, 133, 8, COLOR_WHITE);
    tft_fill_circle(thumb_x, 133, 3, COLOR_BLACK);

    // [ DONE ] Button: X: 106..214, Y: 164..196 (W=108, H=32)
    tft_fill_rect(106, 164, 108, 32, COLOR_BLACK);
    tft_draw_rect(106, 164, 108, 32, COLOR_WHITE);
    tft_draw_string_centered(160, 173, "DONE", COLOR_WHITE, COLOR_BLACK, 1);
  }

  // 5. Pagination Dots
  render_pagination(SCREEN_SETTINGS);
  if (s_show_motion_settings) render_motion_settings(true);

  // 6. Latest Notification Overlay (if active)
  render_notification_banner();
}

// =============================================================================
// SCREEN: DEDICATED BRIGHTNESS SLIDER SCREEN (Accessed via Swipe Down from Top)
// =============================================================================
void display_show_brightness_screen(void) {
  if (!s_is_initialized)
    display_init();
  s_current_screen = SCREEN_BRIGHTNESS;

  tft_fill_screen(COLOR_BLACK);

  // Large Title
  tft_draw_string_centered(160, 42, "BRIGHTNESS", COLOR_WHITE, COLOR_BLACK, 2);

  // Large Percentage Readout: e.g. "70%"
  char b_str[16];
  snprintf(b_str, sizeof(b_str), "%d%%", s_brightness_pct);
  tft_draw_string_centered(160, 72, b_str, COLOR_WHITE, COLOR_BLACK, 3);

  // Decrement [ - ] Button: X: 24..76, Y: 110..154 (W=52, H=44)
  tft_fill_rect(24, 110, 52, 44, COLOR_BLACK);
  tft_draw_rect(24, 110, 52, 44, COLOR_DARKGREY);
  tft_draw_fast_h_line(38, 132, 24, COLOR_WHITE);
  tft_draw_fast_h_line(38, 133, 24, COLOR_WHITE);

  // Increment [ + ] Button: X: 244..296, Y: 110..154 (W=52, H=44)
  tft_fill_rect(244, 110, 52, 44, COLOR_BLACK);
  tft_draw_rect(244, 110, 52, 44, COLOR_DARKGREY);
  tft_draw_fast_h_line(258, 132, 24, COLOR_WHITE);
  tft_draw_fast_h_line(258, 133, 24, COLOR_WHITE);
  tft_draw_fast_v_line(270, 120, 24, COLOR_WHITE);
  tft_draw_fast_v_line(271, 120, 24, COLOR_WHITE);

  // Wide Interactive Slider Track in center: X: 88..232 (W=144), Y: 130..134
  // Outer groove
  tft_fill_rect(88, 130, 144, 6, COLOR_DARKGREY);
  // Filled active level on slider
  int fill_w = (s_brightness_pct * 144) / 100;
  if (fill_w > 144)
    fill_w = 144;
  if (fill_w > 0) {
    tft_fill_rect(88, 130, fill_w, 6, COLOR_WHITE);
  }

  // Slider Knob / Thumb
  int thumb_x = 88 + (s_brightness_pct * 144) / 100;
  if (thumb_x < 88)
    thumb_x = 88;
  if (thumb_x > 232)
    thumb_x = 232;
  tft_fill_circle(thumb_x, 133, 11, COLOR_WHITE);
  tft_fill_circle(thumb_x, 133, 5, COLOR_BLACK);

  // [ DONE ] Button: X: 80..240 (W=160, H=36), Y: 174..210
  tft_fill_rect(80, 174, 160, 36, COLOR_BLACK);
  tft_draw_rect(80, 174, 160, 36, COLOR_WHITE);
  tft_draw_string_centered(160, 184, "DONE", COLOR_WHITE, COLOR_BLACK, 2);

  // Bottom hint
  tft_draw_string_centered(160, 222, "^  SWIPE UP OR TAP DONE TO CLOSE  ^",
                           COLOR_DARKGREY, COLOR_BLACK, 1);
}

// =============================================================================
// SCREEN 5: OTA UPDATING SCREEN (Preserved)
// =============================================================================
void display_show_ota_screen(uint8_t progress_pct) {
  if (!s_is_initialized)
    display_init();
  if (progress_pct > 100)
    progress_pct = 100;

  // Ensure backlight is ON and visible (at least 85%) during firmware update
  if (s_brightness_pct < 80) {
    s_brightness_pct = 85;
  }
  tft_set_backlight(s_brightness_pct);

  if (s_display_mutex) {
    xSemaphoreTakeRecursive(s_display_mutex, portMAX_DELAY);
  }

  bool first_draw = (s_current_screen != SCREEN_OTA);
  s_current_screen = SCREEN_OTA;

  if (first_draw) {
    // Full screen background & static card layout (drawn ONLY ONCE upon
    // starting update)
    tft_fill_screen(COLOR_BLACK);
    tft_draw_rect(6, 6, DISPLAY_WIDTH - 12, DISPLAY_HEIGHT - 12, COLOR_WHITE);
    tft_draw_rect(8, 8, DISPLAY_WIDTH - 16, DISPLAY_HEIGHT - 16,
                  COLOR_DARKGREY);

    // Header Title & Warning
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 32, "FIRMWARE UPDATE",
                             COLOR_WHITE, COLOR_BLACK, 2);
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 66,
                             "DO NOT TURN OFF OR DISCONNECT", COLOR_LIGHTGREY,
                             COLOR_BLACK, 1);
    tft_draw_fast_h_line(24, 88, DISPLAY_WIDTH - 48, COLOR_DARKGREY);

    // Progress Bar Frame: X=28, Y=110, W=264, H=28
    tft_draw_rect(28, 110, 264, 28, COLOR_WHITE);
    tft_draw_rect(29, 111, 262, 26, COLOR_DARKGREY);
  }

  // Flicker-free Partial Refresh: Only redraw the interior progress bar
  const int bar_x = 32;
  const int bar_y = 114;
  const int bar_max_w = 256;
  const int bar_h = 20;

  int fill_width = (progress_pct * bar_max_w) / 100;
  if (fill_width > bar_max_w)
    fill_width = bar_max_w;

  // Clear empty portion
  if (fill_width < bar_max_w) {
    tft_fill_rect(bar_x + fill_width, bar_y, bar_max_w - fill_width, bar_h,
                  COLOR_BLACK);
  }
  // Fill active progress
  if (fill_width > 0) {
    tft_fill_rect(bar_x, bar_y, fill_width, bar_h, COLOR_WHITE);
  }

  // Update Percentage and Status Text (Clear only text bounding boxes)
  tft_fill_rect(20, 150, DISPLAY_WIDTH - 40, 56, COLOR_BLACK);

  char pct_str[32];
  snprintf(pct_str, sizeof(pct_str), "%d%% TRANSFERRED", progress_pct);
  tft_draw_string_centered(DISPLAY_WIDTH / 2, 154, pct_str, COLOR_WHITE,
                           COLOR_BLACK, 2);

  if (progress_pct >= 100) {
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 184,
                             "VERIFYING FIRMWARE...", COLOR_WHITE,
                             COLOR_BLACK, 1);
  } else {
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 184, "WRITING FLASH MEMORY...",
                             COLOR_LIGHTGREY, COLOR_BLACK, 1);
  }

  if (s_display_mutex) {
    xSemaphoreGiveRecursive(s_display_mutex);
  }
}

// =============================================================================
// SCREEN TRANSITION APIS & SWIPE ROUTING
// =============================================================================
void display_set_screen(uint8_t screen_id) {
  if(s_call.state==1 && screen_id!=SCREEN_OTA)return;
  if (screen_id >= SCREEN_COUNT)
    screen_id = SCREEN_HOME;
  if (screen_id == SCREEN_OTA) {
    display_show_ota_screen(0);
    return;
  }
  ESP_LOGI(TAG, "display_set_screen: switching from %d to %d",
           (int)s_current_screen, (int)screen_id);

  if (s_display_mutex) {
    xSemaphoreTakeRecursive(s_display_mutex, portMAX_DELAY);
  }

  if (s_current_screen == screen_id && !s_screen_refresh) {
    if (s_display_mutex) xSemaphoreGiveRecursive(s_display_mutex);
    return;
  }
  s_screen_refresh = false;
  s_media_frame = false;
  s_current_screen = (screen_id_t)screen_id;
  s_hud.is_frame_active = false; // Reset HUD static frame flag so entering
                                 // SCREEN_NAV initialises cleanly
  s_hud.destination_reached = false;
  s_map_drawn = false;
  s_show_route_resume_modal = false;
  s_show_motion_settings = false;

  switch (s_current_screen) {
  case SCREEN_HOME:
    display_show_home_screen();
    break;
  case SCREEN_NAV:
    display_show_main_screen();
    break;
  case SCREEN_MUSIC:
    render_music();
    break;
  case SCREEN_PICTURE:
    display_show_user_image();
    break;
  case SCREEN_SETTINGS:
    display_show_settings_screen();
    break;
  case SCREEN_BRIGHTNESS:
    display_show_brightness_screen();
    break;
  default:
    break;
  }

  ESP_LOGI(TAG,"Screen %d rendered; brightness=%u%%",(int)s_current_screen,(unsigned)s_brightness_pct);
  if (s_display_mutex) {
    xSemaphoreGiveRecursive(s_display_mutex);
  }
}

uint8_t display_get_current_screen(void) { return (uint8_t)s_current_screen; }

void display_next_screen(void) {
  if (s_current_screen == SCREEN_OTA)
    return;

  // Screen cycling order:
  // When navigating: HOME (0) -> NAV (1) -> PICTURE (3) -> SETTINGS (4) -> HOME
  // (0) When ride NOT started: HOME (0) -> PICTURE (3) -> SETTINGS (4) -> HOME
  // (0)
  uint8_t next;
  if (s_current_screen == SCREEN_HOME) {
    if (s_nav.is_navigating) {
      next = SCREEN_NAV;
    } else {
      next = SCREEN_MUSIC; // Show image directly on swipe left from HOME when
                             // no ride is started!
    }
  } else if (s_current_screen == SCREEN_NAV) {
    next = SCREEN_MUSIC;
  } else if (s_current_screen == SCREEN_MUSIC) {
    next = SCREEN_PICTURE;
  } else if (s_current_screen == SCREEN_PICTURE) {
    next = SCREEN_SETTINGS;
  } else if (s_current_screen == SCREEN_SETTINGS) {
    next = SCREEN_HOME;
  } else {
    next = SCREEN_HOME;
  }

  display_set_screen(next);
}

void display_prev_screen(void) {
  if (s_current_screen == SCREEN_OTA)
    return;

  // Reverse screen cycling order (Swipe Right):
  uint8_t prev;
  if (s_current_screen == SCREEN_HOME) {
    prev = SCREEN_SETTINGS;
  } else if (s_current_screen == SCREEN_SETTINGS) {
    prev = SCREEN_PICTURE;
  } else if (s_current_screen == SCREEN_PICTURE) {
    prev = SCREEN_MUSIC;
  } else if (s_current_screen == SCREEN_MUSIC) {
    if (s_nav.is_navigating) {
      prev = SCREEN_NAV;
    } else {
      prev = SCREEN_HOME;
    }
  } else if (s_current_screen == SCREEN_NAV) {
    prev = SCREEN_HOME;
  } else {
    prev = SCREEN_HOME;
  }

  display_set_screen(prev);
}

// =============================================================================
// GESTURE & TOUCH EVENT HANDLER
// =============================================================================
static void handle_touch_event(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, touch_gesture_t gesture);
void display_handle_touch_event(int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, touch_gesture_t gesture) {
  if (!s_display_mutex) return;
  xSemaphoreTakeRecursive(s_display_mutex, portMAX_DELAY);
  handle_touch_event(start_x, start_y, end_x, end_y, gesture);
  xSemaphoreGiveRecursive(s_display_mutex);
}

static void handle_touch_event(int16_t start_x, int16_t start_y, int16_t end_x,
                                int16_t end_y, touch_gesture_t gesture) {
  // Completely lock touch input while firmware update is in progress
  if (s_current_screen == SCREEN_OTA) {
    ESP_LOGD(TAG, "Touch ignored: OTA update in progress");
    return;
  }

  if(s_call.state==1) {
    if(gesture==TOUCH_GESTURE_TAP && s_ble_connected && start_y>=178 && start_y<=221) {
      uint8_t action=start_x>=18 && start_x<=151?1:start_x>=169 && start_x<=302?2:0;
      if(action && (s_call.actions & (1<<(action-1)))) {
        uint8_t packet[2]={0x1C,action};ble_nav_notify_diagnostic_packet(packet,sizeof(packet));
      }
    }
    return;
  }
  switch (gesture) {
  case TOUCH_GESTURE_SWIPE_LEFT:
    // Dismiss confirmation modal on swipe
    s_show_reset_confirm = false;
    // Swipe Left -> Next screen in order Home -> Nav -> Picture -> Settings
    ESP_LOGI(TAG, "SWIPE LEFT -> Next Screen");
    display_next_screen();
    break;

  case TOUCH_GESTURE_SWIPE_RIGHT:
    // Dismiss confirmation modal on swipe
    s_show_reset_confirm = false;
    // Swipe Right -> Previous screen
    ESP_LOGI(TAG, "SWIPE RIGHT -> Previous Screen");
    display_prev_screen();
    break;

  case TOUCH_GESTURE_SWIPE_DOWN:
    s_show_reset_confirm = false;
    // Swiping down from top of screen (start_y <= 130) -> open Brightness
    // screen with interactive slider
    if (start_y <= 130) {
      ESP_LOGI(TAG,
               "SWIPE DOWN (top gesture) -> Open Brightness Slider Screen");
      display_set_screen(SCREEN_BRIGHTNESS);
    } else {
      // Normal swipe down from bottom half -> go to Settings
      ESP_LOGI(TAG, "SWIPE DOWN -> Settings");
      display_set_screen(SCREEN_SETTINGS);
    }
    break;

  case TOUCH_GESTURE_SWIPE_UP:
    s_show_reset_confirm = false;
    // If on brightness screen, swipe up returns to HOME
    if (s_current_screen == SCREEN_BRIGHTNESS) {
      ESP_LOGI(TAG, "SWIPE UP -> Exit Brightness -> HOME");
      display_set_screen(SCREEN_HOME);
      break;
    }
    // Swiping up from top half (start_y <= 130) on other screens -> increase
    // brightness
    if (start_y <= 130) {
      if (s_brightness_pct <= 90)
        s_brightness_pct += 10;
      else
        s_brightness_pct = 100;
      display_set_brightness(s_brightness_pct);
      ESP_LOGI(TAG, "SWIPE UP (top gesture) -> Brightness INC -> %d%%",
               s_brightness_pct);
      // Show high-visibility graphical brightness gauge
      display_set_screen(SCREEN_BRIGHTNESS);
    }
    break;

  case TOUCH_GESTURE_TAP: {
    int16_t tx = start_x;
    int16_t ty = start_y;
    ESP_LOGI(TAG, "TOUCH_GESTURE_TAP at (%d, %d) on screen %d", tx, ty,
             (int)s_current_screen);

    // N: Tap on active Notification Banner (Y: 30..90, X: 10..310) immediately
    // dismisses it
    if (s_latest_notif.active && ty >= 4 && ty <= 31 && tx >= 100 &&
        tx <= 235) {
      ESP_LOGI(TAG, "Touch: Notification Banner tapped -> Dismiss");
      display_dismiss_notification();
      return;
    }



    // Route Resume Modal
    if (s_show_route_resume_modal) {
      // CONTINUE button: X:35..150, Y:130..170
      if (tx >= 35 && tx <= 150 && ty >= 130 && ty <= 170) {
        ESP_LOGI(TAG, "Touch: Resume Route [CONTINUE] tapped");
        s_show_route_resume_modal = false;
        s_nav_interrupted = false;
        ble_nav_notify_status(0x05); // NAV_STATUS_RESUME_ROUTE
        s_nav.rerouting = true;
        strncpy(s_nav.current_street, "Recalculating route...",
                sizeof(s_nav.current_street) - 1);
        display_set_screen(SCREEN_NAV);
        return;
      }
      // CANCEL button: X:170..285, Y:130..170
      if (tx >= 170 && tx <= 285 && ty >= 130 && ty <= 170) {
        ESP_LOGI(TAG, "Touch: Resume Route [CANCEL] tapped");
        s_show_route_resume_modal = false;
        s_nav_interrupted = false;
        s_nav_was_active = false;
        s_nav.is_navigating = false;
        mpu6500_end_ride_calibration();
        ble_nav_notify_status(0x06); // NAV_STATUS_CANCEL_ROUTE
        display_set_screen(SCREEN_HOME);
        return;
      }
      // Ignore touches outside modal when it's open
      return;
    }

    if (s_current_screen == SCREEN_MUSIC) {
      ESP_LOGI(TAG, "MEDIA TAP x=%d y=%d connected=%d state=%u actions=0x%02x",
               tx, ty, s_ble_connected, s_media.state, s_media.actions);
      if (!s_ble_connected) return;
      if (s_media.state < 3 || s_media.state > 5) {
        int64_t now = esp_timer_get_time();
        if (ty >= 118 && ty <= 178 && now - s_last_media_command_us >= 1000000) {
          s_last_media_command_us = now;
          ble_media_command(s_media.state == 1 ? 8 : 7);
        }
        return;
      }
      uint8_t command = 0, action = 0;
      if (ty >= 118 && ty <= 178) {
        if (tx >= 10 && tx <= 104) { command = 4; action = 8; }
        else if (tx >= 113 && tx <= 207) {
          command = s_media.state == 4 ? 2 : 1;
          action = s_media.state == 4 ? 2 : 1;
        } else if (tx >= 216 && tx <= 310) { command = 3; action = 4; }
      } else if (ty >= 187 && ty <= 230) {
        if (tx >= 10 && tx <= 104) { command = 6; action = 16; }
        else if (tx >= 216 && tx <= 310) { command = 5; action = 16; }
      }
      if (!command) ESP_LOGW(TAG, "MEDIA TAP outside button bounds; check touch calibration");
      else if (!(s_media.actions & action)) ESP_LOGW(TAG, "MEDIA command %u unavailable in phone session", command);
      int64_t now = esp_timer_get_time();
      if (command && (s_media.actions & action) && now - s_last_media_command_us >= 250000) {
        s_last_media_command_us = now;
        ble_media_command(command);
      }
      return;
    }

    // Compass returns the actual map camera to the rider; trip card fits route.
    if(s_current_screen==SCREEN_NAV) {
      if(tx>=288 && ty>=142 && ty<=171){display_nav_recenter();return;}
      if(tx>=245 && ty<=38){s_camera_mode=NAV_CAMERA_OVERVIEW;s_zoom_level=NAV_ZOOM_LEVEL_5;
        uint8_t packet[2]={0x14,255};ble_nav_notify_diagnostic_packet(packet,sizeof(packet));return;}
      return;
    }

    // 1. Check SCREEN_BRIGHTNESS touch controls (Slider, [-], [+], [DONE])
    if (s_current_screen == SCREEN_BRIGHTNESS) {
      // [ - ] Button: X: 15..85, Y: 100..165 (Generous hitbox)
      if (tx >= 15 && tx <= 85 && ty >= 100 && ty <= 165) {
        if (s_brightness_pct >= 5)
          s_brightness_pct -= 5;
        else
          s_brightness_pct = 0;
        display_set_brightness(s_brightness_pct);
        ESP_LOGI(TAG, "Brightness screen: [-] pressed -> %d%%",
                 s_brightness_pct);
        display_show_brightness_screen();
        return;
      }
      // [ + ] Button: X: 235..305, Y: 100..165 (Generous hitbox)
      if (tx >= 235 && tx <= 305 && ty >= 100 && ty <= 165) {
        if (s_brightness_pct <= 95)
          s_brightness_pct += 5;
        else
          s_brightness_pct = 100;
        display_set_brightness(s_brightness_pct);
        ESP_LOGI(TAG, "Brightness screen: [+] pressed -> %d%%",
                 s_brightness_pct);
        display_show_brightness_screen();
        return;
      }
      // Interactive Slider Track: X: 86..234, Y: 110..160
      if (tx >= 86 && tx <= 234 && ty >= 110 && ty <= 160) {
        int b_val = ((tx - 88) * 100) / 144;
        if (b_val < 0)
          b_val = 0;
        if (b_val > 100)
          b_val = 100;
        display_set_brightness((uint8_t)b_val);
        ESP_LOGI(TAG, "Brightness screen: slider adjusted -> %d%%",
                 s_brightness_pct);
        display_show_brightness_screen();
        return;
      }
      // [ DONE ] Button: X: 70..250, Y: 165..215 (Generous hitbox)
      if (tx >= 70 && tx <= 250 && ty >= 165 && ty <= 215) {
        ESP_LOGI(TAG, "Brightness screen: [DONE] tapped -> return to HOME");
        display_set_screen(SCREEN_HOME);
        return;
      }
      // Tapping anywhere outside the slider/controls exits back to HOME
      if (ty > 32) {
        ESP_LOGI(TAG, "Brightness screen: background tapped -> return to HOME");
        display_set_screen(SCREEN_HOME);
        return;
      }
    }

    // 2. Check HOME screen touch controls (tap bottom prompt to open Brightness
    // screen)
    if (s_current_screen == SCREEN_HOME) {
      if (ty >= 180 && ty <= 240) {
        ESP_LOGI(TAG, "Home bottom prompt tapped -> open Brightness screen");
        display_set_screen(SCREEN_BRIGHTNESS);
        return;
      }
    }

    // 3. Check SETTINGS screen controls
    if (s_current_screen == SCREEN_SETTINGS) {
      if (s_show_motion_settings) {
        if (ty >= 158 && ty <= 200 && tx >=36 && tx <=194) {
          mpu6500_request_calibration();
          render_motion_settings(true);
        } else if ((ty >=158 && ty <=200 && tx >=204 && tx <=284) || ty <35 || ty >216) {
          s_show_motion_settings=false;
          display_show_settings_screen();
        }
        return;
      }
      // If Brightness Popup Modal is open, handle [-], [+], slider, and [DONE]
      if (s_show_brightness_popup) {
        // [ - ] Button: X: 35..105, Y: 105..160 (Generous 70x55 hitbox)
        if (tx >= 35 && tx <= 105 && ty >= 105 && ty <= 160) {
          if (s_brightness_pct >= 5)
            s_brightness_pct -= 5;
          else
            s_brightness_pct = 0;
          display_set_brightness(s_brightness_pct);
          ESP_LOGI(TAG, "Brightness popup: [-] pressed -> %d%%",
                   s_brightness_pct);
          display_show_settings_screen();
          return;
        }
        // [ + ] Button: X: 215..285, Y: 105..160 (Generous 70x55 hitbox)
        if (tx >= 215 && tx <= 285 && ty >= 105 && ty <= 160) {
          if (s_brightness_pct <= 95)
            s_brightness_pct += 5;
          else
            s_brightness_pct = 100;
          display_set_brightness(s_brightness_pct);
          ESP_LOGI(TAG, "Brightness popup: [+] pressed -> %d%%",
                   s_brightness_pct);
          display_show_settings_screen();
          return;
        }
        // Interactive Slider Track: X: 106..214, Y: 115..150
        if (tx >= 106 && tx <= 214 && ty >= 115 && ty <= 150) {
          int b_val = ((tx - 110) * 100) / 100;
          if (b_val < 0)
            b_val = 0;
          if (b_val > 100)
            b_val = 100;
          display_set_brightness((uint8_t)b_val);
          ESP_LOGI(TAG, "Brightness popup: slider dragged -> %d%%",
                   s_brightness_pct);
          display_show_settings_screen();
          return;
        }
        // [ DONE ] Button: X: 90..230, Y: 155..205 (Generous 140x50 hitbox)
        if (tx >= 90 && tx <= 230 && ty >= 155 && ty <= 205) {
          ESP_LOGI(TAG, "Brightness popup: [DONE] tapped -> closed");
          s_show_brightness_popup = false;
          display_show_settings_screen();
          return;
        }
        // Tap outside modal closes the popup
        s_show_brightness_popup = false;
        display_show_settings_screen();
        return;
      }

      // If Confirmation Modal is open, handle Cancel vs Confirm
      if (s_show_reset_confirm) {
        // Cancel Button: X: 30..150, Y: 110..180
        if (tx >= 30 && tx <= 150 && ty >= 110 && ty <= 180) {
          ESP_LOGI(TAG, "Settings: Reset CANCELLED by user");
          s_show_reset_confirm = false;
          display_show_settings_screen();
          return;
        }
        // Confirm Reset Button: X: 160..280, Y: 110..180
        if (tx >= 160 && tx <= 280 && ty >= 110 && ty <= 180) {
          ESP_LOGW(TAG, "Settings: Reset CONFIRMED by user! Resetting state to "
                        "factory defaults...");
          s_show_reset_confirm = false;
          s_brightness_pct = 70;
          tft_set_backlight(s_brightness_pct);
          display_nav_recenter();
          display_set_screen(SCREEN_HOME);
          return;
        }
        // Any other tap outside modal cancels the modal
        s_show_reset_confirm = false;
        display_show_settings_screen();
        return;
      }

      if (ty >=112 && ty <=148) {
        s_show_motion_settings=true;
        render_motion_settings(true);
        return;
      }
      // Display / Brightness row (Y: 65..115): opens dedicated Brightness Popup
      // Modal!
      if (ty >= 65 && ty <= 115) {
        ESP_LOGI(TAG, "Display row tapped -> Opening Brightness Popup Modal");
        s_show_brightness_popup = true;
        display_show_settings_screen();
        return;
      }
      // Restart Device button (X: 0..158, Y: 150..225)
      if (tx >= 0 && tx <= 158 && ty >= 150 && ty <= 225) {
        ESP_LOGW(TAG, "Restart Device requested via touch");
        ble_disconnect_and_restart();
        return;
      }
      // Reset Device button (X: 162..320, Y: 150..225) -> triggers
      // double-confirmation modal!
      if (tx >= 162 && tx <= 320 && ty >= 150 && ty <= 225) {
        ESP_LOGI(TAG, "Reset Device tapped -> Opening Confirmation Modal");
        s_show_reset_confirm = true;
        display_show_settings_screen();
        return;
      }
      // Bottom dots / pagination bar tap switches screens (Y: 226..240)
      if (ty >= 226 && ty <= 240) {
        if (tx < DISPLAY_WIDTH / 2) {
          display_prev_screen();
        } else {
          display_next_screen();
        }
        return;
      }
    }

    // 4. Check PICTURE screen touch interaction
    if (s_current_screen == SCREEN_PICTURE) {
      // Bottom dots / pagination bar tap switches screens
      if (ty >= 215 && ty <= 240) {
        if (tx < DISPLAY_WIDTH / 2) {
          display_prev_screen();
        } else {
          display_next_screen();
        }
        return;
      }
    }

    ESP_LOGD(TAG, "Tap at (%d,%d) on screen %d", tx, ty, (int)s_current_screen);
    break;
  }
  }
}

// =============================================================================
// HARDWARE DRIVER WRAPPERS & CALLBACKS
// =============================================================================
void display_set_brightness(uint8_t brightness_pct) {
  if (brightness_pct > 100)
    brightness_pct = 100;
  s_brightness_pct = brightness_pct;
  tft_set_backlight(s_brightness_pct);
  if (s_is_initialized && s_current_screen == SCREEN_SETTINGS) {
    update_brightness_widgets();
  }
}

uint8_t display_get_brightness(void) { return s_brightness_pct; }

static TaskHandle_t s_nav_render_task_handle = NULL;

static void nav_render_task(void *pvParam) {
  ESP_LOGI(TAG, "Navigation live HUD render task started (20 FPS)");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 FPS (50ms cadence)

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (!s_is_initialized) {
      continue;
    }

    if (s_display_mutex &&
        xSemaphoreTakeRecursive(s_display_mutex, pdMS_TO_TICKS(35)) == pdTRUE) {
      if (s_notif_dismiss_requested) {
        s_notif_dismiss_requested = false;
        display_dismiss_notification();
      }
      int64_t started = esp_timer_get_time();
      if (s_image_refresh_requested && s_call.state!=1) {
        s_image_refresh_requested = false;
        if (s_current_screen == SCREEN_PICTURE) display_show_user_image();
      }
      bool connected;
      if (xQueueReceive(s_connection_queue, &connected, 0) == pdTRUE) {
        s_ble_connected = connected;
        s_call_painted=false;
        s_phone_state_ready = false;
        s_media = (media_state_t){.state = 6};
        s_media_dirty = true;
        s_route_seg_dirty = true;
        s_nav_interrupted = !connected && s_nav.is_navigating;
      }
      notification_info_t notification;
      if (xQueueReceive(s_notification_queue, &notification, 0) == pdTRUE)
        apply_notification(&notification);
      navigation_data_t nav;
      if (xQueueReceive(s_nav_queue, &nav, 0) == pdTRUE) apply_navigation(&nav);
      route_segment_t segment;
      if (xQueueReceive(s_route_queue, &segment, 0) == pdTRUE && s_nav.is_navigating &&
          memcmp(&segment, &s_route_seg, sizeof(segment)) != 0) {
        bool geometry_changed=segment.count!=s_route_seg.count ||
          segment.completed_segment_count!=s_route_seg.completed_segment_count ||
          segment.is_destination_visible!=s_route_seg.is_destination_visible ||
          segment.dest_x!=s_route_seg.dest_x || segment.dest_y!=s_route_seg.dest_y ||
          memcmp(segment.points,s_route_seg.points,sizeof(segment.points))!=0;
        s_route_seg = segment;
        if(geometry_changed)s_route_seg_dirty = true;
      }
      nav_context_t context;
      while (xQueueReceive(s_context_queue,&context,0)==pdTRUE) {
        s_nav_context=context;
        if(context.poi_type>=1 && context.poi_type<=2)s_pois[context.poi_type-1]=context;
        else memset(s_pois,0,sizeof(s_pois));
      }
      media_state_t media;
      if (xQueueReceive(s_media_queue, &media, 0) == pdTRUE && memcmp(&media, &s_media, sizeof(media)) != 0) {
        s_media = media;
        s_media_dirty = true;
      }
      call_info_t call;
      bool restore_call=false;
      if(xQueueReceive(s_call_queue,&call,0)==pdTRUE) {
        restore_call=s_call.state==1 && call.state!=1;
        if(memcmp(&s_call,&call,sizeof(call)))s_call_painted=false;
        s_call=call;
      }
      if(s_call.state==1 && s_current_screen!=SCREEN_OTA) {
        if(!s_call_painted){hud_call(s_call.caller,s_call.actions,s_ble_connected);s_call_painted=true;}
      } else {
      if(restore_call && s_current_screen!=SCREEN_OTA){s_screen_refresh=true;display_set_screen(s_current_screen);}
      if (s_current_screen == SCREEN_NAV) display_nav_render_partial();
      else if (s_current_screen == SCREEN_MUSIC && !s_latest_notif.active) render_music();
      else if (s_current_screen == SCREEN_SETTINGS && s_show_motion_settings) render_motion_settings(false);
      }
      int64_t elapsed = esp_timer_get_time() - started;
      static int64_t last_metrics, max_render_us,total_render_us;
      static uint32_t render_ticks;
      total_render_us+=elapsed;render_ticks++;
      if (elapsed > max_render_us) max_render_us = elapsed;
      if (esp_timer_get_time() - last_metrics >= 30000000) {
        ESP_LOGI(TAG, "metrics: max_render_us=%lld avg_render_us=%lld ticks=%lu window_ms=%lld notifications=%d heap=%u largest=%u stack_free=%u", max_render_us,total_render_us/render_ticks,(unsigned long)render_ticks,(esp_timer_get_time()-last_metrics)/1000,s_latest_notif.active,
          (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
          (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
          (unsigned)uxTaskGetStackHighWaterMark(NULL));
        max_render_us = 0;total_render_us=0;render_ticks=0;last_metrics=esp_timer_get_time();
      }
      xSemaphoreGiveRecursive(s_display_mutex);
    }
  }
}

void display_init(void) {
  if (s_is_initialized)
    return;

  if (!s_display_mutex) {
    s_display_mutex = xSemaphoreCreateRecursiveMutex();
  }

  s_call_queue=xQueueCreateStatic(1,sizeof(call_info_t),s_call_queue_bytes,&s_call_queue_storage);
  s_nav_queue = xQueueCreateStatic(1, sizeof(navigation_data_t), s_nav_queue_bytes, &s_nav_queue_storage);
  s_media_queue = xQueueCreateStatic(1, sizeof(media_state_t), s_media_queue_bytes, &s_media_queue_storage);
  s_route_queue = xQueueCreateStatic(1, sizeof(route_segment_t), s_route_queue_bytes, &s_route_queue_storage);
  s_context_queue = xQueueCreateStatic(4, sizeof(nav_context_t), s_context_queue_bytes, &s_context_queue_storage);
  s_connection_queue = xQueueCreateStatic(1, sizeof(bool), s_connection_queue_bytes, &s_connection_queue_storage);
  s_notification_queue = xQueueCreateStatic(1, sizeof(notification_info_t), s_notification_queue_bytes, &s_notification_queue_storage);
  configASSERT(s_display_mutex && s_nav_queue && s_media_queue && s_route_queue &&
      s_context_queue && s_connection_queue && s_notification_queue && s_call_queue);
  ESP_ERROR_CHECK(tft_init());
  ESP_ERROR_CHECK(touch_init());

  if (!s_nav_render_task_handle) {
    BaseType_t created = xTaskCreate(nav_render_task, "nav_render", 4096, NULL, 3,
                &s_nav_render_task_handle);
    configASSERT(created == pdPASS);
  }

  s_is_initialized = true;
}

void display_set_time(uint8_t hours, uint8_t minutes) {
  if (hours > 23)
    hours = 23;
  if (minutes > 59)
    minutes = 59;
  s_time_hours = hours;
  s_time_minutes = minutes;
  if (s_is_initialized && s_current_screen == SCREEN_HOME && s_call.state!=1) {
    if (s_display_mutex &&
        xSemaphoreTakeRecursive(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if(s_call.state!=1)draw_home_screen_clock();
      xSemaphoreGiveRecursive(s_display_mutex);
    }
  }
}

void display_set_ble_status(bool connected) {
  if (s_nav_queue) xQueueReset(s_nav_queue);
  if (s_connection_queue) xQueueOverwrite(s_connection_queue, &connected);
}

void display_set_speed(uint16_t speed_kmh) {
  if (speed_kmh < 3)
    speed_kmh = 0; // Stationary deadband: eliminate GPS Doppler noise
  s_speed_kmh = speed_kmh;
  s_nav.speed_kmh = speed_kmh;
  // 20 FPS nav_render_task smoothly renders the updated speed badge without
  // whole-screen refresh
}

// =============================================================================
// IMU HEADING: Called by mpu6500.c imu_task at 50 Hz
// Converts absolute heading into a relative turn delta vs. the reference
// heading, then maps that delta to a nav_direction_t arrow rotation.
// This visual-only heading must not overwrite phone-provided route heading.
// =============================================================================
void display_set_imu_override_direction(nav_direction_t dir) {
  if (dir == s_imu_arrow_override)
    return;
  s_imu_heading_valid = true;
  s_imu_arrow_override = dir;
  ESP_LOGD(TAG, "IMU arrow override: dir=%d", (int)dir);
}

void display_set_imu_heading(float heading_deg) {
  s_imu_heading_valid = true;

  // Lock in reference on first call (s_imu_ref_heading initialised to -1.0)
  if (s_imu_ref_heading < 0.0f) {
    s_imu_ref_heading = heading_deg;
    ESP_LOGI(TAG, "IMU reference heading locked at %.1f deg", heading_deg);
  }

  // Signed delta from the reference heading [-180..+180]
  float delta = heading_deg - s_imu_ref_heading;
  if (delta >  180.0f) delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;

  // Map delta angle to a nav_direction_t arrow:
  //   |delta| <  8 deg  -> no override (dead-zone: straight road noise)
  //   |delta| <  30 deg -> SLIGHT_LEFT / SLIGHT_RIGHT
  //   |delta| <  75 deg -> LEFT / RIGHT
  //   |delta| >= 75 deg -> SHARP_LEFT / SHARP_RIGHT
  nav_direction_t imu_dir;
  if      (delta < -75.0f) imu_dir = NAV_DIR_SHARP_LEFT;
  else if (delta < -30.0f) imu_dir = NAV_DIR_LEFT;
  else if (delta <  -8.0f) imu_dir = NAV_DIR_SLIGHT_LEFT;
  else if (delta >  75.0f) imu_dir = NAV_DIR_SHARP_RIGHT;
  else if (delta >  30.0f) imu_dir = NAV_DIR_RIGHT;
  else if (delta >   8.0f) imu_dir = NAV_DIR_SLIGHT_RIGHT;
  else                     imu_dir = NAV_DIR_NONE; // Inside dead-zone: no override

  display_set_imu_override_direction(imu_dir);
}


void display_update_navigation(const navigation_data_t *nav_data) {
  if (s_nav_queue && nav_data) xQueueOverwrite(s_nav_queue, nav_data);
}

static void apply_navigation(const navigation_data_t *nav_data) {
  if (!nav_data)
    return;

  if (!s_phone_state_ready) s_route_seg_dirty = true;
  s_phone_state_ready = true;
  navigation_data_t filtered = *nav_data;
  if (filtered.speed_kmh < 3)
    filtered.speed_kmh = 0;

  // Stationary GPS jitter filter: When stopped (speed == 0), suppress small ±4m
  // drifts
  if (filtered.speed_kmh == 0 && s_hud.last_dist_m != 0xFFFFFFFF &&
      s_hud.last_dist_m > 0) {
    int diff = (int)filtered.distance_meters - (int)s_hud.last_dist_m;
    if (diff >= -4 && diff <= 4) {
      filtered.distance_meters = s_hud.last_dist_m;
    }
  }

  bool arriving = filtered.direction == NAV_DIR_REACHED && s_nav.direction != NAV_DIR_REACHED;
  if (arriving) {
    s_arrival_started_us = esp_timer_get_time();
    tft_set_backlight(s_brightness_pct);
  }
  if (filtered.is_navigating) s_arrival_started_us = 0;
  bool entering = !s_nav.is_navigating && filtered.is_navigating;
  if (entering) mpu6500_begin_ride_calibration();
  if (!filtered.is_navigating && s_nav.is_navigating) {
    mpu6500_end_ride_calibration();
  }
  bool critical_turn = filtered.is_navigating && filtered.gps_available &&
      filtered.maneuver_state == 4 && s_nav.maneuver_state != 4 && s_current_screen == SCREEN_MUSIC;
  {
    if (!s_nav.rerouting && filtered.rerouting) {
      s_nav_context.poi_type = 0;
      s_nav_context.poi_name[0] = '\0';
      s_route_seg_dirty = true;
    }
    if (!filtered.is_navigating ||
        (!s_nav.is_navigating && filtered.is_navigating)) {
      memset(&s_route_seg, 0, sizeof(s_route_seg));
      memset(&s_nav_context, 0, sizeof(s_nav_context));
      memset(s_pois,0,sizeof(s_pois));
      s_route_seg_dirty = true;
      if (!s_nav.is_navigating && filtered.is_navigating) {
        s_camera_mode = NAV_CAMERA_FOLLOW;
        s_zoom_level = NAV_ZOOM_LEVEL_2;
        s_pan_offset_x = 0;
        s_pan_offset_y = 0;
      }
    }
    memcpy(&s_nav, &filtered, sizeof(navigation_data_t));
    s_speed_kmh = filtered.speed_kmh;
  }
  if ((entering || critical_turn || arriving) && s_current_screen != SCREEN_OTA) {
    s_current_screen = SCREEN_NAV;
    s_hud.is_frame_active = false;
  }

  if (nav_data->is_navigating) {
    s_nav_was_active = true;
    s_nav_interrupted = false;
  } else if (nav_data->direction == NAV_DIR_REACHED) {
    s_nav_was_active = false;
    s_nav_interrupted = false;
  }

  // Decoupled architecture: The incoming BLE packet updates target navigation
  // state in memory. The FreeRTOS 20 FPS nav_render_task smoothly interpolates
  // heading and renders dirty regions.
}

void display_update_route_segment(const route_segment_t *seg) {
  if (seg && s_route_queue) xQueueOverwrite(s_route_queue, seg);
}

void display_update_nav_context(const nav_context_t *context) {
  if (context && s_context_queue) {
    if(xQueueSend(s_context_queue,context,0)!=pdTRUE){nav_context_t old;xQueueReceive(s_context_queue,&old,0);xQueueSend(s_context_queue,context,0);}
  }
}

bool display_map_packet(const uint8_t *packet, size_t len) {
  if (!s_display_mutex || xSemaphoreTakeRecursive(s_display_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  bool ok = map_frame_packet(packet, len);
  if (ok && packet[0]==0x16) s_route_seg_dirty = true;
  xSemaphoreGiveRecursive(s_display_mutex);
  return ok;
}

void display_begin_route_revision(void) {
  if (s_display_mutex) xSemaphoreTakeRecursive(s_display_mutex, portMAX_DELAY);
  map_frame_invalidate();
  memset(s_pois,0,sizeof(s_pois));
  if (s_display_mutex) xSemaphoreGiveRecursive(s_display_mutex);
  if (s_route_queue) xQueueReset(s_route_queue);
  if (s_context_queue) xQueueReset(s_context_queue);
}

void display_clear(void) {
  if (s_display_mutex)
    xSemaphoreTakeRecursive(s_display_mutex, portMAX_DELAY);
  tft_fill_screen(COLOR_SURFACE_DARK);
  if (s_display_mutex)
    xSemaphoreGiveRecursive(s_display_mutex);
}

// =============================================================================
// NAVIGATION CAMERA & ZOOM CONTROLS IMPLEMENTATION
// =============================================================================

void display_nav_zoom_in(void) {
  if (s_camera_mode == NAV_CAMERA_OVERVIEW) {
    // Exit overview back to wide context (Level 4)
    s_camera_mode = NAV_CAMERA_FOLLOW;
    s_zoom_level = NAV_ZOOM_LEVEL_4;
  } else if (s_zoom_level > NAV_ZOOM_LEVEL_1) {
    s_zoom_level = (nav_zoom_level_t)((int)s_zoom_level - 1);
  }
  if (map_frame_active()) {
    uint8_t zoom_packet[2] = {0x14, s_camera_mode == NAV_CAMERA_OVERVIEW ? 255 : (uint8_t)(18 - (int)s_zoom_level)};
    ble_nav_notify_diagnostic_packet(zoom_packet, sizeof(zoom_packet));
  }
  ESP_LOGI(TAG, "Nav Zoom In: Level %d", (int)s_zoom_level);
  if (s_is_initialized && s_current_screen == SCREEN_NAV) {
    s_route_seg_dirty = true;
  }
}

void display_nav_zoom_out(void) {
  if (s_zoom_level < NAV_ZOOM_LEVEL_4) {
    s_zoom_level = (nav_zoom_level_t)((int)s_zoom_level + 1);
  } else {
    // Reached Level 4 -> next zoom out triggers full Route Overview (Level 5)
    s_camera_mode = NAV_CAMERA_OVERVIEW;
    s_zoom_level = NAV_ZOOM_LEVEL_5;
  }
  if (map_frame_active()) {
    uint8_t zoom_packet[2] = {0x14, s_camera_mode == NAV_CAMERA_OVERVIEW ? 255 : (uint8_t)(18 - (int)s_zoom_level)};
    ble_nav_notify_diagnostic_packet(zoom_packet, sizeof(zoom_packet));
  }
  ESP_LOGI(TAG, "Nav Zoom Out: Level %d, Mode %d", (int)s_zoom_level,
           (int)s_camera_mode);
  if (s_is_initialized && s_current_screen == SCREEN_NAV) {
    s_route_seg_dirty = true;
  }
}

void display_nav_recenter(void) {
  s_camera_mode = NAV_CAMERA_FOLLOW;
  s_zoom_level = NAV_ZOOM_LEVEL_2; // Close heading-up follow view
  s_pan_offset_x = 0;
  s_pan_offset_y = 0;
  if (map_frame_active()) {
    uint8_t zoom_packet[2] = {0x14, 254};
    ble_nav_notify_diagnostic_packet(zoom_packet, sizeof(zoom_packet));
  }
  ESP_LOGI(TAG, "Nav Recenter -> Follow Mode at Zoom Level 2");
  if (s_is_initialized && s_current_screen == SCREEN_NAV) {
    s_route_seg_dirty = true;
  }
}

void display_nav_toggle_overview(void) {
  if (s_camera_mode == NAV_CAMERA_OVERVIEW) {
    display_nav_recenter();
  } else {
    s_camera_mode = NAV_CAMERA_OVERVIEW;
    s_zoom_level = NAV_ZOOM_LEVEL_5;
    if (map_frame_active()) {
    uint8_t zoom_packet[2] = {0x14, s_camera_mode == NAV_CAMERA_OVERVIEW ? 255 : (uint8_t)(18 - (int)s_zoom_level)};
    ble_nav_notify_diagnostic_packet(zoom_packet, sizeof(zoom_packet));
  }
  ESP_LOGI(TAG, "Nav Toggle -> Route Overview");
    if (s_is_initialized && s_current_screen == SCREEN_NAV) {
      s_route_seg_dirty = true;
    }
  }
}

void display_nav_pan(int16_t dx, int16_t dy) {
  s_camera_mode = NAV_CAMERA_MANUAL;
  s_pan_offset_x += dx;
  s_pan_offset_y += dy;
  // Clamp panning within sensible bounds
  if (s_pan_offset_x > 120)
    s_pan_offset_x = 120;
  if (s_pan_offset_x < -120)
    s_pan_offset_x = -120;
  if (s_pan_offset_y > 80)
    s_pan_offset_y = 80;
  if (s_pan_offset_y < -80)
    s_pan_offset_y = -80;
  if (s_is_initialized && s_current_screen == SCREEN_NAV) s_route_seg_dirty = true;
}

nav_zoom_level_t display_nav_get_zoom(void) { return s_zoom_level; }

nav_camera_mode_t display_nav_get_camera_mode(void) { return s_camera_mode; }

// =============================================================================
// LATEST NOTIFICATION APIS & AUTO-DISMISS TIMER (Strictly Latest Only)
// =============================================================================

static void notif_auto_dismiss_timer_cb(TimerHandle_t xTimer) {
  ESP_LOGI(TAG, "Notification auto-dismiss timer fired (2s)");
  s_notif_dismiss_requested = true;
}

void display_show_notification(const char *title, const char *message) {
  display_show_app_notification("Alert",title,message);
}

void display_show_app_notification(const char *app,const char *title,const char *message) {
  if (!s_notification_queue) return;
  notification_info_t notification = {0};
  snprintf(notification.app,sizeof(notification.app),"%s",app?app:"");
  snprintf(notification.title, sizeof(notification.title), "%s", title ? title : "");
  snprintf(notification.message, sizeof(notification.message), "%s", message ? message : "");
  xQueueOverwrite(s_notification_queue, &notification);
}

static void apply_notification(const notification_info_t *notification) {
  if (!s_is_initialized) return;
  s_latest_notif=*notification;
  s_latest_notif.active=true;
  s_notif_dirty=true;
  s_notif_started_us=esp_timer_get_time();
  s_notif_dismiss_requested=false;
  render_notification_banner();

  // Start or reset 2-second auto-dismiss timer
  if (!s_notif_timer) {
    s_notif_timer = xTimerCreate("notif_timer", pdMS_TO_TICKS(2000), pdFALSE,
                                 NULL, notif_auto_dismiss_timer_cb);
  }
  if (s_notif_timer) {
    xTimerStop(s_notif_timer, 0);
    xTimerStart(s_notif_timer, 0);
  }
}

void display_dismiss_notification(void) {
  if (!s_latest_notif.active)
    return;

  if (s_notif_timer) {
    xTimerStop(s_notif_timer, 0);
  }

  if (s_display_mutex &&
      xSemaphoreTakeRecursive(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    s_latest_notif.active = false;
    memset(&s_latest_notif, 0, sizeof(s_latest_notif));

    if(s_call.state==1){xSemaphoreGiveRecursive(s_display_mutex);return;}
    // Restore only the covered region during navigation.
    switch (s_current_screen) {
    case SCREEN_HOME:
      display_show_home_screen();
      break;
    case SCREEN_NAV:
      if(map_frame_active()) map_frame_restore_region(100,4,135,27);
      else s_route_seg_dirty=true;
      break;
    case SCREEN_MUSIC:
      s_media_frame = false;
      render_music();
      break;
    case SCREEN_PICTURE:
      display_show_user_image();
      break;
    case SCREEN_SETTINGS:
      display_show_settings_screen();
      break;
    default:
      break;
    }

    xSemaphoreGiveRecursive(s_display_mutex);
  }
}

bool display_is_notification_active(void) { return s_latest_notif.active; }
