#include "display.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_partition.h"
#include "ble.h"

static const char *TAG = "DISPLAY";

static bool s_is_initialized = false;
static screen_id_t s_current_screen = SCREEN_COUNT; // Initialize to SCREEN_COUNT during boot/splash
static SemaphoreHandle_t s_display_mutex = NULL;

// Dynamic brightness state (0-100%)
static uint8_t s_brightness_pct = 80;

// Dynamic status bar state
static uint8_t s_time_hours = 10;
static uint8_t s_time_minutes = 24;
static bool s_ble_connected = false;
static uint16_t s_speed_kmh = 0; // Live vehicle speed (km/h) — 0 when stationary


// Reset Device confirmation modal state
static bool s_show_reset_confirm = false;
// Dedicated Brightness popup modal state (Master Prompt §28)
static bool s_show_brightness_popup = false;
// On-Screen Route Resume modal state (when power/BLE/GPS reconnects during active route)
static bool s_show_route_resume_modal = false;
static bool s_nav_was_active = false;
static bool s_nav_interrupted = false;

// Latest Notification State (Strictly holds ONLY the most recent notification)
typedef struct {
    char title[32];
    char message[64];
    bool active;
} notification_info_t;

static notification_info_t s_latest_notif = {
    .title = "",
    .message = "",
    .active = false,
};
static TimerHandle_t s_notif_timer = NULL;

// Gallery / User Image state (Strict SINGLE image support)
static bool s_user_image_valid = false;

// Forward route animation step
static uint8_t s_anim_step = 0;

// Dynamic Color Palette for Day & Night Themes
typedef struct {
    uint16_t bg;          // Canvas background
    uint16_t card_bg;     // Overlay card background
    uint16_t border;      // Card and road borders
    uint16_t route;       // Active navigation road corridor (ALWAYS BLUE)
    uint16_t text_pri;    // Primary high-contrast text
    uint16_t text_sec;    // Subtitles and distance units
    uint16_t speed_num;   // Speedometer numeric readout
    uint16_t pointer;     // Vehicle GPS arrowhead (GREEN/WHITE)
} theme_palette_t;

// Day mode & Night mode (Uniform Black & White High Contrast)
static const theme_palette_t s_theme_day = {
    .bg        = COLOR_BLACK,       // Pure Black background
    .card_bg   = COLOR_BLACK,       // Pure Black card background
    .border    = COLOR_DARKGREY,    // Dark grey border
    .route     = COLOR_WHITE,       // Pure White route corridor
    .text_pri  = COLOR_WHITE,       // White text
    .text_sec  = COLOR_LIGHTGREY,   // Light grey secondary
    .speed_num = COLOR_WHITE,       // White speed
    .pointer   = COLOR_WHITE,       // White navigation chevron
};

static const theme_palette_t s_theme_night = {
    .bg        = COLOR_BLACK,       // Pure Black background
    .card_bg   = COLOR_BLACK,       // Pure Black card background
    .border    = COLOR_DARKGREY,    // Dark grey border
    .route     = COLOR_WHITE,       // Pure White route corridor
    .text_pri  = COLOR_WHITE,       // White primary text
    .text_sec  = COLOR_LIGHTGREY,   // Light grey secondary
    .speed_num = COLOR_WHITE,       // Crisp white speed readout
    .pointer   = COLOR_WHITE,       // White GPS arrowhead
};

static inline bool is_night_mode(void) {
    return (s_time_hours >= 18 || s_time_hours < 6);
}

static inline const theme_palette_t* get_active_theme(void) {
    return is_night_mode() ? &s_theme_night : &s_theme_day;
}

// Active Navigation Guidance State
static navigation_data_t s_nav = {
    .direction = NAV_DIR_STRAIGHT,
    .distance_meters = 0,
    .current_street = "Ready to Ride",
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

// Navigation Camera & Zoom Engine
static nav_camera_mode_t s_camera_mode = NAV_CAMERA_FOLLOW;
static nav_zoom_level_t s_zoom_level = NAV_ZOOM_LEVEL_3; // Default 500m
static int16_t s_pan_offset_x = 0;
static int16_t s_pan_offset_y = 0;

// Live Navigation Persistent HUD Cache & Smoothing State
typedef struct {
    bool is_frame_active;          // Static HUD frame has been drawn
    uint32_t last_dist_m;          // Distance in meters
    uint16_t last_speed;           // Vehicle speed km/h
    nav_direction_t last_arrow_dir;// Maneuver arrow enum
    uint16_t last_arrow_color;     // Maneuver arrow color
    char last_turn_instruction[16]; // Text instruction beside the maneuver arrow
    char last_street[32];          // Street name
    char last_dist_rem[24];        // Remaining distance text
    char last_eta[24];             // ETA text
    bool last_gps_available;       // Satellite lock status
    bool last_rerouting;           // Rerouting status
    float current_heading_rad;     // Smoothly interpolated heading in radians
    bool destination_reached;      // Arrival screen active
    int16_t last_px;               // Last rendered pointer X
    int16_t last_py;               // Last rendered pointer Y
    float current_px;              // Continuous animated pointer X coordinate
    float current_py;              // Continuous animated pointer Y coordinate
    float current_angle;           // Continuous animated pointer angle in radians
    int16_t last_box_x0;           // Bounding box of previous chevron (X0)
    int16_t last_box_y0;           // Bounding box of previous chevron (Y0)
    int16_t last_box_x1;           // Bounding box of previous chevron (X1)
    int16_t last_box_y1;           // Bounding box of previous chevron (Y1)
    nav_direction_t last_drawn_junction_dir; // Direction of currently rendered junction corridor
    bool last_show_maneuver_junction;        // Whether maneuver junction was drawn (<30m)
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
    .last_gps_available = true,
    .last_rerouting = false,
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
};

// Forward declaration of internal refreshers
static void display_nav_render_partial(void);
static void update_brightness_widgets(void);
static void render_notification_banner(void);
void display_show_brightness_screen(void);

// Keep the maneuver explicit as well as graphical.  The arrow alone is easy
// to miss on a vibrating motorcycle display, especially at the turn itself.
static const char *turn_instruction_label(nav_direction_t direction,
                                          uint8_t maneuver_state) {
    const bool turn_now = (maneuver_state == 4) || (s_nav.distance_meters <= 14 && s_nav.distance_meters > 0); // TURN_ACTIVE / TURNING

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
           s_nav.direction != NAV_DIR_REACHED &&
           s_nav.distance_meters <= 100;
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
    int dot_centers[5] = { 136, 148, 160, 172, 184 };
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

// =============================================================================
// TOP STATUS BAR (Y: 0 to 26) — Common to all screens (Monochrome)
// JustRide brand (left), BLE + Signal + GPS Pin + Clock (right)
// =============================================================================
static void render_status_bar(void) {
    if (s_current_screen == SCREEN_OTA) return;

    tft_fill_rect(0, 0, DISPLAY_WIDTH, 26, COLOR_BLACK);

    // Brand "JustRide" on top-left
    tft_draw_string(10, 8, "Just", COLOR_WHITE, COLOR_BLACK, 1);
    tft_draw_string(34, 8, "Ride", COLOR_WHITE, COLOR_BLACK, 1);

    // Right side indicators: BLE symbol, cellular signal bars, GPS pin, Time
    // 1. BLE indicator
    if (s_ble_connected) {
        tft_draw_string(216, 8, "B", COLOR_WHITE, COLOR_BLACK, 1);
    } else {
        tft_draw_string(216, 8, "B", COLOR_DARKGREY, COLOR_BLACK, 1);
    }

    // 2. Cellular signal bars (4 ascending bars at X=232)
    int sx = 232;
    tft_fill_rect(sx,     16, 2, 4, COLOR_WHITE);
    tft_fill_rect(sx + 3, 14, 2, 6, COLOR_WHITE);
    tft_fill_rect(sx + 6, 12, 2, 8, COLOR_WHITE);
    tft_fill_rect(sx + 9, 10, 2, 10, COLOR_WHITE);

    // 3. Location pin symbol at X=252 (White when locked, Dark Grey when lost)
    uint16_t gps_col = s_nav.gps_available ? COLOR_WHITE : COLOR_DARKGREY;
    tft_fill_circle(254, 11, 3, gps_col);
    tft_fill_triangle(252, 12, 256, 12, 254, 17, gps_col);

    // 4. Digital Time (e.g. "10:24 AM")
    char time_buf[16];
    snprintf(time_buf, sizeof(time_buf), "%02d:%02d", s_time_hours, s_time_minutes);
    tft_draw_string(268, 8, time_buf, COLOR_WHITE, COLOR_BLACK, 1);

    // Subtle divider
    tft_draw_fast_h_line(0, 26, DISPLAY_WIDTH, COLOR_DARKGREY);
}

// =============================================================================
// SCREEN: ANIMATED BOOT SPLASH SCREEN (320x240)
// =============================================================================
void display_show_splash(void) {
    if (!s_is_initialized) display_init();

    ESP_LOGI(TAG, "Rendering JustRide Motorcycle Splash Screen (MADE FOR BIKES)...");

    // Acquire display mutex to lock out all background tasks during splash animation
    if (s_display_mutex) {
        xSemaphoreTake(s_display_mutex, portMAX_DELAY);
    }

    // Hold background rendering tasks at bay
    s_current_screen = SCREEN_COUNT;

    // 1. Clean screen canvas
    tft_fill_screen(COLOR_BLACK);

    // 2. High-contrast double border frame
    tft_draw_rect(6, 6, DISPLAY_WIDTH - 12, DISPLAY_HEIGHT - 12, COLOR_WHITE);
    tft_draw_rect(8, 8, DISPLAY_WIDTH - 16, DISPLAY_HEIGHT - 16, COLOR_DARKGREY);

    // 3. JustRide Helmet / HUD Chevron Icon
    tft_fill_circle(DISPLAY_WIDTH / 2, 40, 14, COLOR_WHITE);
    tft_fill_circle(DISPLAY_WIDTH / 2, 40, 11, COLOR_BLACK);
    tft_fill_triangle(DISPLAY_WIDTH / 2, 33, DISPLAY_WIDTH / 2 - 6, 45, DISPLAY_WIDTH / 2 + 6, 45, COLOR_WHITE);

    // 4. Main Brand Header
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 64, "JUST RIDE", COLOR_WHITE, COLOR_BLACK, 3);
    tft_draw_fast_h_line(50, 96, 220, COLOR_WHITE);

    // 5. Official Motorcycle Tagline
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 106, "MADE FOR BIKES", COLOR_WHITE, COLOR_BLACK, 2);
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 128, "CONNECTED COCKPIT HUD", COLOR_LIGHTGREY, COLOR_BLACK, 1);

    // 6. Progress Bar Architecture (Pre-rendered before turning on backlight)
    const int bar_x = 45;
    const int bar_y = 158;
    const int bar_w = 230;
    const int bar_h = 10;
    tft_draw_rect(bar_x - 2, bar_y - 2, bar_w + 4, bar_h + 4, COLOR_WHITE);
    tft_fill_rect(bar_x, bar_y, bar_w, bar_h, COLOR_BLACK);

    // Pre-draw initial 0% status string
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 180, "LOADING SYSTEM   0%", COLOR_LIGHTGREY, COLOR_BLACK, 1);

    // Raise backlight cleanly now that full splash frame is rendered (zero glitch/noise)
    tft_set_backlight(s_brightness_pct);

    // 7. Smooth, glitch-free progress bar animation
    for (int p = 5; p <= 100; p += 5) {
        int fill = (p * bar_w) / 100;
        if (fill > 0) {
            tft_fill_rect(bar_x, bar_y, fill, bar_h, COLOR_WHITE);
        }
        char prog_str[24];
        snprintf(prog_str, sizeof(prog_str), "LOADING SYSTEM %3d%%", p);
        // Clear text box first to prevent character ghosting / tearing
        tft_fill_rect(DISPLAY_WIDTH / 2 - 80, 178, 160, 12, COLOR_BLACK);
        tft_draw_string_centered(DISPLAY_WIDTH / 2, 180, prog_str, COLOR_LIGHTGREY, COLOR_BLACK, 1);
        vTaskDelay(pdMS_TO_TICKS(35));
    }

    vTaskDelay(pdMS_TO_TICKS(150));

    // Release mutex before switching screen
    if (s_display_mutex) {
        xSemaphoreGive(s_display_mutex);
    }

    display_set_screen(SCREEN_HOME);
}

// =============================================================================
// SCREEN 0: HOME SCREEN
// Large clock, Date, "RIDE MORE EXPLORE MORE", Rider Helmet Art, App Bar
// =============================================================================
void display_show_home_screen(void) {
    if (!s_is_initialized) display_init();
    s_current_screen = SCREEN_HOME;

    tft_fill_screen(COLOR_BLACK);
    render_status_bar();

    // Digital Clock displayed centrally and cleanly (scale 5: ~144x35 px, centered at X=160, Y=95)
    char time_buf[16];
    snprintf(time_buf, sizeof(time_buf), "%02d:%02d", s_time_hours, s_time_minutes);
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 95, time_buf, COLOR_WHITE, COLOR_BLACK, 5);

    // Subtle down-arrow swipe indicator at bottom
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 218, "v  SWIPE DOWN FOR BRIGHTNESS  v", COLOR_DARKGREY, COLOR_BLACK, 1);

    // Latest Notification Overlay (if active)
    render_notification_banner();
}


// =============================================================================
// HELPER: PROCEDURAL DOUBLE-LINE ROAD NETWORK WITH WHITE BORDERS & SIDE STREETS
// Replicates Google Maps road styling with left/right branching streets (double lines)
// and high-contrast white borders. Supports clipped rendering for zero-flicker HUD.
// =============================================================================

static inline void draw_clipped_h_line(int xa, int xb, int y, int cx0, int cy0, int cx1, int cy1, uint16_t color) {
    if (y < cy0 || y > cy1) return;
    int x_start = (xa < xb) ? xa : xb;
    int x_end   = (xa < xb) ? xb : xa;
    if (x_end < cx0 || x_start > cx1) return;
    if (x_start < cx0) x_start = cx0;
    if (x_end > cx1) x_end = cx1;
    tft_draw_fast_h_line(x_start, y, x_end - x_start + 1, color);
}

static inline void draw_clipped_v_line(int x, int ya, int yb, int cx0, int cy0, int cx1, int cy1, uint16_t color) {
    if (x < cx0 || x > cx1) return;
    int y_start = (ya < yb) ? ya : yb;
    int y_end   = (ya < yb) ? yb : ya;
    if (y_end < cy0 || y_start > cy1) return;
    if (y_start < cy0) y_start = cy0;
    if (y_end > cy1) y_end = cy1;
    tft_draw_fast_v_line(x, y_start, y_end - y_start + 1, color);
}

static inline void draw_clipped_line(int x0, int y0, int x1, int y1, int cx0, int cy0, int cx1, int cy1, uint16_t color) {
    int min_x = (x0 < x1) ? x0 : x1;
    int max_x = (x0 > x1) ? x0 : x1;
    int min_y = (y0 < y1) ? y0 : y1;
    int max_y = (y0 > y1) ? y0 : y1;
    if (max_x < cx0 || min_x > cx1 || max_y < cy0 || min_y > cy1) return;
    tft_draw_line(x0, y0, x1, y1, color);
}

static void render_road_scene_clipped(int cx0, int cy0, int cx1, int cy1, nav_direction_t direction, uint32_t distance_meters) {
    // Maneuver turn junction appears ONLY when approaching within 30m of the maneuver!
    // Beyond 30m (e.g. 124m, 500m), the road is the continuous straight route corridor being traveled.
    bool show_maneuver = (distance_meters <= 30 && distance_meters > 0 && s_nav.is_navigating) || (s_nav.maneuver_state == 4);

    bool is_right_turn = show_maneuver && (direction == NAV_DIR_RIGHT || direction == NAV_DIR_SLIGHT_RIGHT || direction == NAV_DIR_SHARP_RIGHT);
    bool is_left_turn  = show_maneuver && (direction == NAV_DIR_LEFT  || direction == NAV_DIR_SLIGHT_LEFT  || direction == NAV_DIR_SHARP_LEFT);
    bool is_uturn      = show_maneuver && (direction == NAV_DIR_UTURN);

    // ── 1. MAIN ROAD CORRIDOR BORDERS (2px Solid White Borders, 38px Wide Road) ────
    // Left border: X = 141..142
    if (is_left_turn) {
        // Open junction mouth at Y in [52..78] for the left turn street
        draw_clipped_v_line(141, 28, 51, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(142, 28, 51, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(141, 79, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(142, 79, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
    } else {
        // Unbroken continuous straight road corridor while traveling along the route
        draw_clipped_v_line(141, 28, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(142, 28, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
    }

    // Right border: X = 178..179
    if (is_right_turn) {
        // Open junction mouth at Y in [52..78] for the right turn street
        draw_clipped_v_line(178, 28, 51, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(179, 28, 51, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(178, 79, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(179, 79, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
    } else {
        // Unbroken continuous straight road corridor while traveling along the route
        draw_clipped_v_line(178, 28, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(179, 28, 214, cx0, cy0, cx1, cy1, COLOR_WHITE);
    }

    // ── 2. DYNAMIC MANEUVER JUNCTION (Appears dynamically at <= 30m!) ────────
    if (is_right_turn) {
        // Double-line turn road branching right with smooth corner chamfers
        draw_clipped_h_line(186, 250, 52, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(186, 250, 78, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(179, 44, 187, 52, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(178, 44, 186, 52, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(179, 86, 187, 78, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(178, 86, 186, 78, cx0, cy0, cx1, cy1, COLOR_WHITE);
        // Curved dashed center line into the turn street
        draw_clipped_line(160, 84, 163, 77, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(169, 71, 178, 66, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(186, 198, 65, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(206, 220, 65, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(228, 245, 65, cx0, cy0, cx1, cy1, COLOR_WHITE);
    } else if (is_left_turn) {
        // Double-line turn road branching left with smooth corner chamfers
        draw_clipped_h_line(70, 134, 52, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(70, 134, 78, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(141, 44, 133, 52, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(142, 44, 134, 52, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(141, 86, 133, 78, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(142, 86, 134, 78, cx0, cy0, cx1, cy1, COLOR_WHITE);
        // Curved dashed center line into the turn street
        draw_clipped_line(160, 84, 157, 77, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(151, 71, 142, 66, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(122, 134, 65, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(100, 114, 65, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_h_line(75, 92, 65, cx0, cy0, cx1, cy1, COLOR_WHITE);
    } else if (is_uturn) {
        // Smooth curved U-turn loop
        draw_clipped_h_line(115, 141, 52, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(115, 52, 105, 65, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_line(105, 65, 115, 80, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(115, 80, 105, cx0, cy0, cx1, cy1, COLOR_WHITE);
    }

    // ── 3. MAIN ROAD CENTER DASHED LANE DIVIDER (X=159..160, 2px wide) ────────
    // Dashed line (8px dash, 8px gap)
    int dash_limit_y = (is_right_turn || is_left_turn || is_uturn) ? 85 : 30;
    for (int y = 210; y >= dash_limit_y; y -= 16) {
        draw_clipped_v_line(159, y - 7, y, cx0, cy0, cx1, cy1, COLOR_WHITE);
        draw_clipped_v_line(160, y - 7, y, cx0, cy0, cx1, cy1, COLOR_WHITE);
    }
}

// =============================================================================
// HELPER: CONTINUOUS VECTOR RIDER CHEVRON WITH ARBITRARY ANGLE ROTATION (ZOOMED IN)
// Draws an aerodynamic navigation chevron at (cx, cy) with heading angle_rad.
// Restores only the dirty bounding box patch from previous frame for ZERO FLICKER.
// =============================================================================
static void draw_smooth_route_pointer(float cx, float cy, float angle_rad) {
    float sin_a = sinf(angle_rad);
    float cos_a = cosf(angle_rad);

    // Forward and Right unit vectors
    float fx = sin_a;
    float fy = -cos_a;
    float rx = cos_a;
    float ry = sin_a;

    // Zoomed-in aerodynamic dimensions (prominent, high-visibility motorcycle HUD pointer)
    const float l_tip   = 20.0f; // Tip forward (zoomed in)
    const float l_wing  = 16.0f; // Wings backward
    const float w_wing  = 12.0f; // Wing half-width
    const float l_notch = 6.5f;  // Inset notch

    // Floating-point vertices
    float p0_x = cx + l_tip * fx;
    float p0_y = cy + l_tip * fy;
    float p1_x = cx - l_wing * fx - w_wing * rx;
    float p1_y = cy - l_wing * fy - w_wing * ry;
    float p2_x = cx - l_wing * fx + w_wing * rx;
    float p2_y = cy - l_wing * fy + w_wing * ry;
    float p3_x = cx - l_notch * fx;
    float p3_y = cy - l_notch * fy;

    // Rounded integer coordinates
    int ip0_x = (int)(p0_x + 0.5f);
    int ip0_y = (int)(p0_y + 0.5f);
    int ip1_x = (int)(p1_x + 0.5f);
    int ip1_y = (int)(p1_y + 0.5f);
    int ip2_x = (int)(p2_x + 0.5f);
    int ip2_y = (int)(p2_y + 0.5f);
    int ip3_x = (int)(p3_x + 0.5f);
    int ip3_y = (int)(p3_y + 0.5f);

    // Bounding box of the new chevron
    int min_x = ip0_x;
    if (ip1_x < min_x) min_x = ip1_x;
    if (ip2_x < min_x) min_x = ip2_x;
    if (ip3_x < min_x) min_x = ip3_x;

    int max_x = ip0_x;
    if (ip1_x > max_x) max_x = ip1_x;
    if (ip2_x > max_x) max_x = ip2_x;
    if (ip3_x > max_x) max_x = ip3_x;

    int min_y = ip0_y;
    if (ip1_y < min_y) min_y = ip1_y;
    if (ip2_y < min_y) min_y = ip2_y;
    if (ip3_y < min_y) min_y = ip3_y;

    int max_y = ip0_y;
    if (ip1_y > max_y) max_y = ip1_y;
    if (ip2_y > max_y) max_y = ip2_y;
    if (ip3_y > max_y) max_y = ip3_y;

    // 1. Restore the dirty background patch from previous frame
    if (s_hud.last_box_x0 >= 0) {
        int r_x0 = s_hud.last_box_x0;
        int r_y0 = s_hud.last_box_y0;
        int r_x1 = s_hud.last_box_x1;
        int r_y1 = s_hud.last_box_y1;

        if (r_x0 < 0) r_x0 = 0;
        if (r_y0 < 28) r_y0 = 28;
        if (r_x1 >= DISPLAY_WIDTH) r_x1 = DISPLAY_WIDTH - 1;
        if (r_y1 > 214) r_y1 = 214;

        if (r_x1 >= r_x0 && r_y1 >= r_y0) {
            tft_fill_rect(r_x0, r_y0, r_x1 - r_x0 + 1, r_y1 - r_y0 + 1, COLOR_BLACK);
            render_road_scene_clipped(r_x0, r_y0, r_x1, r_y1, s_hud.last_drawn_junction_dir, s_nav.distance_meters);
        }
    }

    // 2. Draw crisp 3D aerodynamic vector chevron (zoomed in)
    tft_fill_triangle(ip0_x, ip0_y, ip1_x, ip1_y, ip3_x, ip3_y, COLOR_WHITE);
    tft_fill_triangle(ip0_x, ip0_y, ip3_x, ip3_y, ip2_x, ip2_y, COLOR_WHITE);

    // Dark facet spine for 3D depth
    tft_draw_line(ip0_x, ip0_y, ip3_x, ip3_y, COLOR_DARKGREY);

    // High-contrast outer boundary outline
    tft_draw_line(ip0_x, ip0_y, ip1_x, ip1_y, COLOR_BLACK);
    tft_draw_line(ip1_x, ip1_y, ip3_x, ip3_y, COLOR_BLACK);
    tft_draw_line(ip3_x, ip3_y, ip2_x, ip2_y, COLOR_BLACK);
    tft_draw_line(ip2_x, ip2_y, ip0_x, ip0_y, COLOR_BLACK);

    // 3. Save new bounding box with 3px safety margin for next dirty rect restore
    s_hud.last_box_x0 = min_x - 3;
    s_hud.last_box_y0 = min_y - 3;
    s_hud.last_box_x1 = max_x + 3;
    s_hud.last_box_y1 = max_y + 3;
    s_hud.last_px = (int16_t)(cx + 0.5f);
    s_hud.last_py = (int16_t)(cy + 0.5f);
}

// =============================================================================
// HELPER: BRIGHTNESS HUD OVERLAY GAUGE
// =============================================================================
static void show_brightness_hud(uint8_t pct) {
    const int box_x = 50;
    const int box_y = 80;
    const int box_w = 220;
    const int box_h = 75;

    // Background Card
    tft_fill_rect(box_x, box_y, box_w, box_h, COLOR_BLACK);
    tft_draw_rect(box_x, box_y, box_w, box_h, COLOR_WHITE);
    tft_draw_rect(box_x + 2, box_y + 2, box_w - 4, box_h - 4, COLOR_DARKGREY);

    // Title / Icon
    char title_buf[32];
    snprintf(title_buf, sizeof(title_buf), "BRIGHTNESS  %d%%", pct);
    tft_draw_string_centered(160, box_y + 16, title_buf, COLOR_WHITE, COLOR_BLACK, 2);

    // Visual Progress Bar Frame
    const int bar_x = box_x + 20;
    const int bar_y = box_y + 44;
    const int bar_w = box_w - 40; // 180px
    const int bar_h = 14;

    tft_draw_rect(bar_x, bar_y, bar_w, bar_h, COLOR_WHITE);
    tft_fill_rect(bar_x + 2, bar_y + 2, bar_w - 4, bar_h - 4, COLOR_BLACK);

    int fill_w = (pct * (bar_w - 4)) / 100;
    if (fill_w > bar_w - 4) fill_w = bar_w - 4;
    if (fill_w > 0) {
        tft_fill_rect(bar_x + 2, bar_y + 2, fill_w, bar_h - 4, COLOR_WHITE);
    }
}

// =============================================================================
// HELPER: LATEST NOTIFICATION HUD BANNER OVERLAY (Y: 34 to 88, X: 12 to 308)
// Displays strictly ONLY the latest incoming notification with auto/tap dismiss
// =============================================================================
static void render_notification_banner(void) {
    if (!s_latest_notif.active) return;

    const int nx = 12;
    const int ny = 34;
    const int nw = DISPLAY_WIDTH - 24; // 296 px
    const int nh = 52;

    // Solid dark card background with bright white automotive border
    tft_fill_rect(nx, ny, nw, nh, COLOR_BLACK);
    tft_draw_rect(nx, ny, nw, nh, COLOR_WHITE);
    tft_draw_rect(nx + 1, ny + 1, nw - 2, nh - 2, COLOR_WHITE);

    // Bell / Alert Icon on the left (at X=28, Y=60)
    // Small bell outline & clapper
    tft_fill_triangle(28, 44, 22, 56, 34, 56, COLOR_WHITE);
    tft_fill_rect(20, 56, 16, 3, COLOR_WHITE);
    tft_fill_circle(28, 61, 2, COLOR_WHITE);

    // Title (Sender or App) in bold white (Scale 2 or 1)
    char title_buf[24] = {0};
    strncpy(title_buf, s_latest_notif.title[0] ? s_latest_notif.title : "Notification", sizeof(title_buf) - 1);
    tft_draw_string(44, 42, title_buf, COLOR_WHITE, COLOR_BLACK, 1);

    // Dismiss hint 'X' on top right
    tft_draw_string(nx + nw - 18, 42, "x", COLOR_LIGHTGREY, COLOR_BLACK, 1);

    // Subtitle / Body text (Light grey, Scale 1, truncated)
    char msg_buf[38] = {0};
    strncpy(msg_buf, s_latest_notif.message[0] ? s_latest_notif.message : "New message received", sizeof(msg_buf) - 1);
    tft_draw_string(44, 62, msg_buf, COLOR_LIGHTGREY, COLOR_BLACK, 1);
}

// =============================================================================
// HELPER: ON-SCREEN ROUTE RESUME MODAL
// Shown when BLE reconnects after an interrupted route, or on GPS recovery
// =============================================================================
static void render_route_resume_modal(void) {
    if (!s_show_route_resume_modal) return;

    // Center Modal window
    tft_fill_rect(20, 35, 280, 170, COLOR_BLACK);
    tft_draw_rect(20, 35, 280, 170, COLOR_WHITE);
    tft_draw_rect(21, 36, 278, 168, COLOR_DARKGREY);

    tft_draw_string_centered(160, 50, "RESUME ROUTE?", COLOR_WHITE, COLOR_BLACK, 2);
    tft_draw_string_centered(160, 80, "Connection restored.", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    tft_draw_string_centered(160, 98, "Continue to destination?", COLOR_LIGHTGREY, COLOR_BLACK, 1);

    // [ CONTINUE ] Button: X: 35..150 (W: 115)
    tft_fill_rect(35, 130, 115, 40, COLOR_WHITE);
    tft_draw_string_centered(92, 142, "CONTINUE", COLOR_BLACK, COLOR_WHITE, 1);

    // [ CANCEL ] Button: X: 170..285 (W: 115)
    tft_fill_rect(170, 130, 115, 40, COLOR_BLACK);
    tft_draw_rect(170, 130, 115, 40, COLOR_DARKGREY);
    tft_draw_string_centered(227, 142, "CANCEL", COLOR_WHITE, COLOR_BLACK, 1);
}

void display_show_resume_route_modal(void) {
    if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_show_route_resume_modal = true;
        if (s_current_screen == SCREEN_NAV) display_show_main_screen();
        else if (s_current_screen == SCREEN_HOME) display_show_home_screen();
        else display_set_screen(s_current_screen);
        xSemaphoreGive(s_display_mutex);
    }
}

void display_dismiss_resume_route_modal(void) {
    if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_show_route_resume_modal = false;
        if (s_current_screen == SCREEN_NAV) display_show_main_screen();
        else if (s_current_screen == SCREEN_HOME) display_show_home_screen();
        else display_set_screen(s_current_screen);
        xSemaphoreGive(s_display_mutex);
    }
}

bool display_is_route_interrupted(void) {
    return s_nav_interrupted;
}

// =============================================================================
// SCREEN 1: PERSISTENT LIVE NAVIGATION HUD SCREEN (MONOCHROME MOTORCYCLE HUD)
// Master Enhancement: Live, Non-Refreshing TFT Navigation (Google Maps Style)
//
// The screen is initialized ONCE on entering navigation. Full-screen clear
// (tft_fill_screen) is STRICTLY PROHIBITED during navigation. Dynamic elements
// (distance, speed, maneuver arrow, street name, road corridor, chevron)
// are updated via dirty-region partial redraws without screen flicker.
// =============================================================================

static void display_nav_render_partial(void) {
    if (s_current_screen != SCREEN_NAV) return;

    // When there is no active ride started, do not display the active navigation
    // corridor or HUD gauges. Keep the display on SCREEN_HOME (clock & status).
    if (!s_nav.is_navigating && s_nav.direction != NAV_DIR_REACHED) {
        s_current_screen = SCREEN_HOME;
        s_hud.is_frame_active = false;
        display_show_home_screen();
        return;
    }

    // -------------------------------------------------------------------------
    // 0. ARRIVAL SCREEN (NAV_DIR_REACHED) — Drawn ONCE on arrival
    // -------------------------------------------------------------------------
    if (s_nav.direction == NAV_DIR_REACHED) {
        if (!s_hud.destination_reached) {
            s_hud.destination_reached = true;
            s_hud.is_frame_active = false;
            tft_fill_screen(COLOR_BLACK);

            // Large white checkmark in circle
            tft_draw_circle(160, 75, 30, COLOR_WHITE);
            tft_draw_thick_line(146, 75, 156, 87, 4, COLOR_WHITE);
            tft_draw_thick_line(156, 87, 176, 63, 4, COLOR_WHITE);

            tft_draw_string_centered(160, 120, "DESTINATION REACHED", COLOR_WHITE, COLOR_BLACK, 2);

            // Destination marker (◎)
            tft_draw_circle(160, 150, 10, COLOR_WHITE);
            tft_fill_circle(160, 150, 4, COLOR_WHITE);

            // END RIDE button prompt
            tft_draw_rect(80, 178, 160, 36, COLOR_WHITE);
            tft_draw_string_centered(160, 188, "END RIDE", COLOR_WHITE, COLOR_BLACK, 2);
        }
        return;
    } else {
        s_hud.destination_reached = false;
    }

    // -------------------------------------------------------------------------
    // 1. PERSISTENT STATIC FRAME INITIALIZATION (Drawn ONCE on entry)
    // -------------------------------------------------------------------------
    if (!s_hud.is_frame_active) {
        tft_fill_screen(COLOR_BLACK);

        // Speedometer circular outer double-ring outline (persistent)
        const int spd_cx = 268;
        const int spd_cy = 182;
        const int spd_r = 30;
        tft_draw_circle(spd_cx, spd_cy, spd_r, COLOR_WHITE);
        tft_draw_circle(spd_cx, spd_cy, spd_r - 2, COLOR_WHITE);

        // Bottom floating bar divider & center dot (persistent)
        tft_draw_fast_h_line(10, 216, 300, COLOR_DARKGREY);
        tft_fill_circle(160, 225, 2, COLOR_WHITE);

        // Double-line road corridor with white borders
        s_hud.last_drawn_junction_dir = s_nav.direction;
        s_hud.last_show_maneuver_junction = (s_nav.distance_meters <= 30 && s_nav.distance_meters > 0 && s_nav.is_navigating) || (s_nav.maneuver_state == 4);
        render_road_scene_clipped(0, 28, DISPLAY_WIDTH - 1, 215, s_hud.last_drawn_junction_dir, s_nav.distance_meters);

        // Initial vector rider pointer (zoomed in, seated in active road corridor)
        s_hud.current_px = 160.0f;
        s_hud.current_py = 145.0f;
        s_hud.current_angle = 0.0f;
        s_hud.last_box_x0 = -1;
        draw_smooth_route_pointer(s_hud.current_px, s_hud.current_py, s_hud.current_angle);

        // Invalidate all dynamic cache entries to force immediate draw
        s_hud.is_frame_active = true;
        s_hud.last_dist_m = 0xFFFFFFFF;
        s_hud.last_speed = 0xFFFF;
        s_hud.last_arrow_dir = (nav_direction_t)0xFF;
        s_hud.last_arrow_color = 0;
        s_hud.last_turn_instruction[0] = '\0';
        s_hud.last_street[0] = '\0';
        s_hud.last_dist_rem[0] = '\0';
        s_hud.last_eta[0] = '\0';
        s_hud.last_gps_available = !s_nav.gps_available;
        s_hud.last_rerouting = !s_nav.rerouting;
    }

    // -------------------------------------------------------------------------
    // 2. WARNING BANNERS (GPS LOST / REROUTING) — Dedicated Region (Y: 6..28)
    // -------------------------------------------------------------------------
    if (s_nav.gps_available != s_hud.last_gps_available || s_nav.rerouting != s_hud.last_rerouting) {
        tft_fill_rect(20, 6, 280, 22, COLOR_BLACK);
        if (!s_nav.gps_available) {
            tft_draw_rect(30, 8, 260, 18, COLOR_WHITE);
            tft_draw_string_centered(160, 13, "! GPS SIGNAL LOST", COLOR_WHITE, COLOR_BLACK, 1);
        } else if (s_nav.rerouting) {
            tft_draw_rect(40, 8, 240, 18, COLOR_LIGHTGREY);
            tft_draw_string_centered(160, 13, "REROUTING...", COLOR_LIGHTGREY, COLOR_BLACK, 1);
        }
        s_hud.last_gps_available = s_nav.gps_available;
        s_hud.last_rerouting = s_nav.rerouting;
    }

    // -------------------------------------------------------------------------
    // 3. LOWER-LEFT: TURN DIRECTION ARROW & COUNTDOWN DISTANCE
    // -------------------------------------------------------------------------
    nav_direction_t arrow_dir = s_nav.direction;
    if (arrow_dir == NAV_DIR_NONE) {
        arrow_dir = NAV_DIR_STRAIGHT;
    }
    uint16_t arrow_color = COLOR_WHITE;

    if (s_nav.maneuver_state == 2 && s_nav.distance_meters > 80) {
        arrow_color = COLOR_LIGHTGREY;
    } else {
        arrow_color = COLOR_WHITE;
    }

    // Keep the street text clear of the maneuver region.  Previously this
    // redraw overlapped the upper part of a left/right arrow, which could make
    // the turn cue appear clipped or disappear during live updates.
    char current_street_trunc[20] = {0};
    if (s_nav.current_street[0] != '\0' && strcmp(s_nav.current_street, "Ready to Ride") != 0) {
        strncpy(current_street_trunc, s_nav.current_street, sizeof(current_street_trunc) - 1);
    }
    if (strcmp(current_street_trunc, s_hud.last_street) != 0) {
        tft_fill_rect(8, 34, 128, 14, COLOR_BLACK);
        if (current_street_trunc[0] != '\0') {
            tft_draw_string(10, 37, current_street_trunc, COLOR_LIGHTGREY, COLOR_BLACK, 1);
        }
        strncpy(s_hud.last_street, current_street_trunc, sizeof(s_hud.last_street) - 1);
    }

    const char *turn_instruction = turn_instruction_label(
        arrow_dir, s_nav.maneuver_state);
    if (strcmp(turn_instruction, s_hud.last_turn_instruction) != 0) {
        tft_fill_rect(8, 132, 128, 14, COLOR_BLACK);
        if (turn_instruction[0] != '\0') {
            tft_draw_string(8, 135, turn_instruction, COLOR_WHITE, COLOR_BLACK, 1);
        }
        strncpy(s_hud.last_turn_instruction, turn_instruction,
                sizeof(s_hud.last_turn_instruction) - 1);
        s_hud.last_turn_instruction[sizeof(s_hud.last_turn_instruction) - 1] = '\0';
    }

    // Turn Arrow: Partial redraw ONLY when maneuver or urgency color changes.
    // It is paired with the explicit instruction above and stays visible for
    // the complete APPROACHING/PREVIEW/ACTIVE maneuver lifecycle.
    if (arrow_dir != s_hud.last_arrow_dir || arrow_color != s_hud.last_arrow_color) {
        tft_fill_rect(8, 150, 52, 58, COLOR_BLACK);
        tft_draw_turn_arrow(36, 178, 28, arrow_dir, arrow_color);
        s_hud.last_arrow_dir = arrow_dir;
        s_hud.last_arrow_color = arrow_color;
    }

    // Distance Box: Partial redraw ONLY when countdown distance changes
    if (s_nav.distance_meters != s_hud.last_dist_m || arrow_dir != s_hud.last_arrow_dir) {
        char dist_val[12];
        char dist_unit[8];
        // Direction prefix label (fits in 1–2 chars at scale 2)
        const char *dir_prefix = "";
        switch (arrow_dir) {
            case NAV_DIR_LEFT:         dir_prefix = "L"; break;
            case NAV_DIR_SLIGHT_LEFT:  dir_prefix = "L"; break;
            case NAV_DIR_SHARP_LEFT:   dir_prefix = "L"; break;
            case NAV_DIR_RIGHT:        dir_prefix = "R"; break;
            case NAV_DIR_SLIGHT_RIGHT: dir_prefix = "R"; break;
            case NAV_DIR_SHARP_RIGHT:  dir_prefix = "R"; break;
            case NAV_DIR_UTURN:        dir_prefix = "U"; break;
            case NAV_DIR_ROUNDABOUT:   dir_prefix = "O"; break;
            default:                   dir_prefix = ""; break;
        }
        if (s_nav.maneuver_state == 4 && s_nav.distance_meters == 0) {
            snprintf(dist_val, sizeof(dist_val), "NOW");
            dist_unit[0] = '\0';
            dir_prefix = "";
        } else if (s_nav.distance_meters >= 1000) {
            snprintf(dist_val, sizeof(dist_val), "%.1f", (float)s_nav.distance_meters / 1000.0f);
            snprintf(dist_unit, sizeof(dist_unit), "km");
        } else {
            snprintf(dist_val, sizeof(dist_val), "%lu", (unsigned long)s_nav.distance_meters);
            snprintf(dist_unit, sizeof(dist_unit), "m");
        }

        tft_fill_rect(60, 148, 74, 64, COLOR_BLACK);
        if (dir_prefix[0] != '\0') {
            // Direction label at scale 2 (12 px wide, 16 px tall) — top of box
            tft_draw_string(64, 150, dir_prefix, COLOR_WHITE, COLOR_BLACK, 2);
        }
        tft_draw_string(62, 167, dist_val, COLOR_WHITE, COLOR_BLACK, 3);
        if (dist_unit[0] != '\0') {
            tft_draw_string(64, 195, dist_unit, COLOR_WHITE, COLOR_BLACK, 1);
        }
        s_hud.last_dist_m = s_nav.distance_meters;
    }

    // -------------------------------------------------------------------------
    // 4. LOWER-RIGHT: CIRCULAR SPEEDOMETER GAUGE (Speed Dial Inner Readout)
    // -------------------------------------------------------------------------
    if (s_speed_kmh != s_hud.last_speed || s_nav.gps_available != s_hud.last_gps_available) {
        const int spd_cx = 268;
        const int spd_cy = 182;
        char spd_num[8];
        if (!s_nav.gps_available) {
            snprintf(spd_num, sizeof(spd_num), "--");
        } else {
            snprintf(spd_num, sizeof(spd_num), "%u", (unsigned)s_speed_kmh);
        }

        // Clear inner circular dial (r=26) without disturbing white outer double-ring
        tft_fill_circle(spd_cx, spd_cy, 26, COLOR_BLACK);
        tft_draw_string_centered(spd_cx, spd_cy - 10, spd_num, COLOR_WHITE, COLOR_BLACK, 3);
        tft_draw_string_centered(spd_cx, spd_cy + 13, "km/h", COLOR_LIGHTGREY, COLOR_BLACK, 1);
        s_hud.last_speed = s_speed_kmh;
    }

    // -------------------------------------------------------------------------
    // 5. BOTTOM FLOATING BAR: REMAINING DISTANCE & ETA
    // -------------------------------------------------------------------------
    char rem_str[24];
    if (s_nav.remaining_dist_km >= 1.0f) {
        snprintf(rem_str, sizeof(rem_str), "DIST %.1f km", s_nav.remaining_dist_km);
    } else {
        snprintf(rem_str, sizeof(rem_str), "DIST %.0f m", s_nav.remaining_dist_km * 1000.0f);
    }
    if (strcmp(rem_str, s_hud.last_dist_rem) != 0) {
        tft_fill_rect(12, 220, 140, 14, COLOR_BLACK);
        tft_draw_string(24, 222, rem_str, COLOR_WHITE, COLOR_BLACK, 1);
        strncpy(s_hud.last_dist_rem, rem_str, sizeof(s_hud.last_dist_rem) - 1);
    }

    char eta_str[24];
    snprintf(eta_str, sizeof(eta_str), "ETA %s", s_nav.eta_time_str[0] ? s_nav.eta_time_str : "--");
    if (strcmp(eta_str, s_hud.last_eta) != 0) {
        tft_fill_rect(170, 220, 140, 14, COLOR_BLACK);
        tft_draw_string_centered(236, 222, eta_str, COLOR_WHITE, COLOR_BLACK, 1);
        strncpy(s_hud.last_eta, eta_str, sizeof(s_hud.last_eta) - 1);
    }

    // -------------------------------------------------------------------------
    // 6. CENTRAL ROAD POV CORRIDOR & LIVE MOVING RIDER POINTER (ZERO SCREEN REFRESH)
    // Double-line road with white borders and left/right side streets.
    // As vehicle enters maneuver junction (< 25m), vehicle pointer follows a
    // smooth curved Bezier turn arc into the side street with continuous angle rotation.
    // -------------------------------------------------------------------------
    static uint32_t s_leg_initial_dist_m = 0;
    static nav_direction_t s_last_leg_dir = NAV_DIR_NONE;
    static uint16_t s_last_maneuver_id = 0;

    if (s_nav.maneuver_id != s_last_maneuver_id || s_nav.direction != s_last_leg_dir || s_nav.distance_meters > s_leg_initial_dist_m) {
        s_leg_initial_dist_m = s_nav.distance_meters > 0 ? s_nav.distance_meters : 100;
        s_last_leg_dir = s_nav.direction;
        s_last_maneuver_id = s_nav.maneuver_id;
        s_hud.current_px = 160.0f;
        s_hud.current_py = 145.0f;
        s_hud.current_angle = 0.0f;
    }

    // Dynamic Road Network Update:
    // The turn junction ONLY appears when approaching within 30m of the maneuver.
    // Beyond 30m (e.g. 124m), the corridor is unbroken straight road without fake side streets.
    bool show_maneuver = (s_nav.distance_meters <= 30 && s_nav.distance_meters > 0 && s_nav.is_navigating) || (s_nav.maneuver_state == 4);

    if (show_maneuver != s_hud.last_show_maneuver_junction || (show_maneuver && s_nav.direction != s_hud.last_drawn_junction_dir)) {
        tft_fill_rect(70, 28, 185, 188, COLOR_BLACK);
        s_hud.last_drawn_junction_dir = s_nav.direction;
        s_hud.last_show_maneuver_junction = show_maneuver;
        render_road_scene_clipped(70, 28, 255, 215, s_hud.last_drawn_junction_dir, s_nav.distance_meters);
        s_hud.last_box_x0 = -1; // Reset previous box so pointer redraws clean
    }

    // ── Google Maps Style Curved Turn Trajectory & Smooth Continuous Rotation ─
    float target_x = 160.0f;
    float target_y = 145.0f;
    float target_angle = 0.0f; // Radians: 0.0f is straight UP, +pi/2 is right, -pi/2 is left

    bool is_right = show_maneuver && (s_nav.direction == NAV_DIR_RIGHT || s_nav.direction == NAV_DIR_SLIGHT_RIGHT || s_nav.direction == NAV_DIR_SHARP_RIGHT);
    bool is_left  = show_maneuver && (s_nav.direction == NAV_DIR_LEFT  || s_nav.direction == NAV_DIR_SLIGHT_LEFT  || s_nav.direction == NAV_DIR_SHARP_LEFT);
    bool is_uturn = show_maneuver && (s_nav.direction == NAV_DIR_UTURN);

    if (is_right) {
        float t = 1.0f - ((float)s_nav.distance_meters / 30.0f);
        if (s_nav.maneuver_state == 4 && t < 0.45f) t = 0.45f; // Stay active in turn arc
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        if (t < 0.30f) {
            // Stage 1 (30m -> 21m): straight approach up road corridor towards junction mouth
            float u = t / 0.30f;
            target_x = 160.0f;
            target_y = 145.0f - (u * 45.0f); // 145 down to 100
            target_angle = 0.0f;
        } else {
            // Stage 2 (21m -> 0m): Quadratic Bezier turn arc into right street
            // P0=(160, 100), P1=(160, 65), P2=(215, 65)
            float s = (t - 0.30f) / 0.70f;
            if (s > 1.0f) s = 1.0f;
            float inv_s = 1.0f - s;
            target_x = (inv_s * inv_s * 160.0f) + (2.0f * inv_s * s * 160.0f) + (s * s * 215.0f);
            target_y = (inv_s * inv_s * 100.0f) + (2.0f * inv_s * s * 65.0f)  + (s * s * 65.0f);

            // Tangent derivative for continuous smooth heading rotation (0 to +pi/2)
            float dx = 110.0f * s;
            float dy = -70.0f * inv_s;
            target_angle = atan2f(dx, -dy);
        }
    } else if (is_left) {
        float t = 1.0f - ((float)s_nav.distance_meters / 30.0f);
        if (s_nav.maneuver_state == 4 && t < 0.45f) t = 0.45f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        if (t < 0.30f) {
            // Stage 1 (30m -> 21m): straight approach up road corridor towards junction mouth
            float u = t / 0.30f;
            target_x = 160.0f;
            target_y = 145.0f - (u * 45.0f); // 145 down to 100
            target_angle = 0.0f;
        } else {
            // Stage 2 (21m -> 0m): Quadratic Bezier turn arc into left street
            // P0=(160, 100), P1=(160, 65), P2=(105, 65)
            float s = (t - 0.30f) / 0.70f;
            if (s > 1.0f) s = 1.0f;
            float inv_s = 1.0f - s;
            target_x = (inv_s * inv_s * 160.0f) + (2.0f * inv_s * s * 160.0f) + (s * s * 105.0f);
            target_y = (inv_s * inv_s * 100.0f) + (2.0f * inv_s * s * 65.0f)  + (s * s * 65.0f);

            // Tangent derivative for continuous smooth heading rotation (0 to -pi/2)
            float dx = -110.0f * s;
            float dy = -70.0f * inv_s;
            target_angle = atan2f(dx, -dy);
        }
    } else if (is_uturn) {
        float t = 1.0f - ((float)s_nav.distance_meters / 30.0f);
        if (s_nav.maneuver_state == 4 && t < 0.45f) t = 0.45f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        if (t < 0.30f) {
            float u = t / 0.30f;
            target_x = 160.0f;
            target_y = 145.0f - (u * 45.0f);
            target_angle = 0.0f;
        } else {
            float s = (t - 0.30f) / 0.70f;
            if (s > 1.0f) s = 1.0f;
            float u_angle = s * -(float)M_PI;
            target_x = 145.0f + 15.0f * cosf(u_angle);
            target_y = 80.0f - 15.0f * sinf(-u_angle);
            target_angle = u_angle;
        }
    } else {
        // Straight road cruising: pointer is seated centered in the corridor
        target_x = 160.0f;
        target_y = 145.0f;
        target_angle = 0.0f;
    }

    // 20 Hz Exponential visual smoothing (eliminates jitter and ensures cinematic smoothness)
    s_hud.current_px += (target_x - s_hud.current_px) * 0.25f;
    s_hud.current_py += (target_y - s_hud.current_py) * 0.25f;

    // Shortest angular distance interpolation
    float diff_a = target_angle - s_hud.current_angle;
    while (diff_a > (float)M_PI) diff_a -= 2.0f * (float)M_PI;
    while (diff_a < -(float)M_PI) diff_a += 2.0f * (float)M_PI;
    s_hud.current_angle += diff_a * 0.25f;

    draw_smooth_route_pointer(s_hud.current_px, s_hud.current_py, s_hud.current_angle);

    // -------------------------------------------------------------------------
    // 7. OVERLAYS (Latest Notification & Route Resume Modal if active)
    // -------------------------------------------------------------------------
    render_notification_banner();
    render_route_resume_modal();
}

void display_show_main_screen(void) {
    if (!s_is_initialized) display_init();
    s_current_screen = SCREEN_NAV;
    s_hud.is_frame_active = false;
    display_nav_render_partial();
}


// =============================================================================
// SCREEN 3: PICTURE / GALLERY SCREEN
// Photo viewer, Counter "3 / 24", Navigation chevrons, Trash can icon
// =============================================================================
#define USER_IMG_MAGIC 0x4A52494D // "JRIM"
#define USER_IMG_TOTAL_BYTES (DISPLAY_WIDTH * DISPLAY_HEIGHT * 2u) // 153600

static const esp_partition_t *s_user_img_part = NULL;
static size_t s_stream_flash_offset = 0;

static const esp_partition_t *get_user_img_partition(void) {
    if (!s_user_img_part) {
        s_user_img_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x99, "user_img");
        if (!s_user_img_part) {
            s_user_img_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "user_img");
        }
    }
    return s_user_img_part;
}

bool display_has_user_image(void) {
    if (s_user_image_valid) return true;
    const esp_partition_t *part = get_user_img_partition();
    if (!part) return false;
    uint32_t magic = 0;
    if (esp_partition_read(part, 0, &magic, sizeof(magic)) == ESP_OK) {
        if (magic == USER_IMG_MAGIC) {
            s_user_image_valid = true;
            return true;
        }
    }
    return false;
}

void display_start_image_stream(void) {
    if (!s_is_initialized) display_init();
    s_current_screen = SCREEN_PICTURE;
    const esp_partition_t *part = get_user_img_partition();
    if (part) {
        esp_partition_erase_range(part, 0, part->size);
    }
    s_stream_flash_offset = 4096;
    tft_stream_image_start();
}

void display_write_image_chunk(const uint8_t *chunk, size_t len) {
    if (!chunk || len == 0) return;
    tft_stream_image_chunk(chunk, len);
    const esp_partition_t *part = get_user_img_partition();
    if (part && s_stream_flash_offset + len <= part->size) {
        esp_partition_write(part, s_stream_flash_offset, chunk, len);
        s_stream_flash_offset += len;
    }
}

void display_finish_image_stream(void) {
    const esp_partition_t *part = get_user_img_partition();
    if (part) {
        uint32_t header[2] = { USER_IMG_MAGIC, USER_IMG_TOTAL_BYTES };
        esp_partition_write(part, 0, header, sizeof(header));
    }
    s_user_image_valid = true;
    s_current_screen = SCREEN_PICTURE;
    render_pagination(SCREEN_PICTURE);
}

void display_show_user_image(void) {
    if (!s_is_initialized) display_init();
    s_current_screen = SCREEN_PICTURE;

    tft_fill_screen(COLOR_BLACK);
    render_status_bar();

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
                size_t chunk = (remaining > sizeof(read_buf)) ? sizeof(read_buf) : remaining;
                if (esp_partition_read(part, offset, read_buf, chunk) == ESP_OK) {
                    tft_stream_image_chunk(read_buf, chunk);
                }
                offset += chunk;
                remaining -= chunk;
            }
        }
    } else {
        // Clean empty state when no image exists
        tft_draw_string_centered(160, 110, "NO IMAGE UPLOADED", COLOR_WHITE, COLOR_BLACK, 2);
        tft_draw_string_centered(160, 140, "Upload an image from the app", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    }

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

void display_show_settings_screen(void) {
    if (!s_is_initialized) display_init();
    s_current_screen = SCREEN_SETTINGS;

    tft_fill_screen(COLOR_BLACK);
    render_status_bar();

    // 1. Bluetooth List Card (Y = 32 to 68)
    tft_fill_rect(10, 32, DISPLAY_WIDTH - 20, 36, COLOR_BLACK);
    tft_draw_rect(10, 32, DISPLAY_WIDTH - 20, 36, COLOR_DARKGREY);
    // BLE Icon
    tft_draw_char(24, 44, 'B', COLOR_WHITE, COLOR_BLACK, 2);
    tft_draw_string(46, 44, "Bluetooth", COLOR_WHITE, COLOR_BLACK, 1);
    tft_draw_string(216, 44, s_ble_connected ? "Connected" : "Searching",
                    s_ble_connected ? COLOR_WHITE : COLOR_LIGHTGREY, COLOR_BLACK, 1);
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
    tft_draw_string(46, 124, "System", COLOR_WHITE, COLOR_BLACK, 1);
    tft_draw_string(224, 124, "v2.1 OTA", COLOR_LIGHTGREY, COLOR_BLACK, 1);
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

        tft_draw_string_centered(160, 75, "RESET DEVICE?", COLOR_WHITE, COLOR_BLACK, 2);
        tft_draw_string_centered(160, 100, "Reset settings to defaults?", COLOR_LIGHTGREY, COLOR_BLACK, 1);

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

        tft_draw_string_centered(160, 50, "BRIGHTNESS", COLOR_WHITE, COLOR_BLACK, 2);

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
        if (thumb_x > 210) thumb_x = 210;
        tft_fill_circle(thumb_x, 133, 8, COLOR_WHITE);
        tft_fill_circle(thumb_x, 133, 3, COLOR_BLACK);

        // [ DONE ] Button: X: 106..214, Y: 164..196 (W=108, H=32)
        tft_fill_rect(106, 164, 108, 32, COLOR_BLACK);
        tft_draw_rect(106, 164, 108, 32, COLOR_WHITE);
        tft_draw_string_centered(160, 173, "DONE", COLOR_WHITE, COLOR_BLACK, 1);
    }

    // 5. Pagination Dots
    render_pagination(SCREEN_SETTINGS);

    // 6. Latest Notification Overlay (if active)
    render_notification_banner();
}

// =============================================================================
// SCREEN: DEDICATED BRIGHTNESS SLIDER SCREEN (Accessed via Swipe Down from Top)
// =============================================================================
void display_show_brightness_screen(void) {
    if (!s_is_initialized) display_init();
    s_current_screen = SCREEN_BRIGHTNESS;

    tft_fill_screen(COLOR_BLACK);
    render_status_bar();

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
    if (fill_w > 144) fill_w = 144;
    if (fill_w > 0) {
        tft_fill_rect(88, 130, fill_w, 6, COLOR_WHITE);
    }

    // Slider Knob / Thumb
    int thumb_x = 88 + (s_brightness_pct * 144) / 100;
    if (thumb_x < 88) thumb_x = 88;
    if (thumb_x > 232) thumb_x = 232;
    tft_fill_circle(thumb_x, 133, 11, COLOR_WHITE);
    tft_fill_circle(thumb_x, 133, 5, COLOR_BLACK);

    // [ DONE ] Button: X: 80..240 (W=160, H=36), Y: 174..210
    tft_fill_rect(80, 174, 160, 36, COLOR_BLACK);
    tft_draw_rect(80, 174, 160, 36, COLOR_WHITE);
    tft_draw_string_centered(160, 184, "DONE", COLOR_WHITE, COLOR_BLACK, 2);

    // Bottom hint
    tft_draw_string_centered(160, 222, "^  SWIPE UP OR TAP DONE TO CLOSE  ^", COLOR_DARKGREY, COLOR_BLACK, 1);
}


// =============================================================================
// SCREEN 5: OTA UPDATING SCREEN (Preserved)
// =============================================================================
void display_show_ota_screen(uint8_t progress_pct) {
    if (!s_is_initialized) display_init();
    if (progress_pct > 100) progress_pct = 100;

    // Ensure backlight is ON and visible (at least 85%) during firmware update
    if (s_brightness_pct < 80) {
        s_brightness_pct = 85;
    }
    tft_set_backlight(s_brightness_pct);

    if (s_display_mutex) {
        xSemaphoreTake(s_display_mutex, portMAX_DELAY);
    }

    bool first_draw = (s_current_screen != SCREEN_OTA);
    s_current_screen = SCREEN_OTA;

    if (first_draw) {
        // Full screen background & static card layout (drawn ONLY ONCE upon starting update)
        tft_fill_screen(COLOR_BLACK);
        tft_draw_rect(6, 6, DISPLAY_WIDTH - 12, DISPLAY_HEIGHT - 12, COLOR_WHITE);
        tft_draw_rect(8, 8, DISPLAY_WIDTH - 16, DISPLAY_HEIGHT - 16, COLOR_DARKGREY);

        // Header Title & Warning
        tft_draw_string_centered(DISPLAY_WIDTH / 2, 32, "FIRMWARE UPDATE", COLOR_WHITE, COLOR_BLACK, 2);
        tft_draw_string_centered(DISPLAY_WIDTH / 2, 66, "DO NOT TURN OFF OR DISCONNECT", COLOR_LIGHTGREY, COLOR_BLACK, 1);
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
    if (fill_width > bar_max_w) fill_width = bar_max_w;

    // Clear empty portion
    if (fill_width < bar_max_w) {
        tft_fill_rect(bar_x + fill_width, bar_y, bar_max_w - fill_width, bar_h, COLOR_BLACK);
    }
    // Fill active progress
    if (fill_width > 0) {
        tft_fill_rect(bar_x, bar_y, fill_width, bar_h, COLOR_WHITE);
    }

    // Update Percentage and Status Text (Clear only text bounding boxes)
    tft_fill_rect(20, 150, DISPLAY_WIDTH - 40, 56, COLOR_BLACK);

    char pct_str[32];
    snprintf(pct_str, sizeof(pct_str), "%d%% COMPLETED", progress_pct);
    tft_draw_string_centered(DISPLAY_WIDTH / 2, 154, pct_str, COLOR_WHITE, COLOR_BLACK, 2);

    if (progress_pct >= 100) {
        tft_draw_string_centered(DISPLAY_WIDTH / 2, 184, "VERIFYING & RESTARTING...", COLOR_WHITE, COLOR_BLACK, 1);
    } else {
        tft_draw_string_centered(DISPLAY_WIDTH / 2, 184, "WRITING FLASH MEMORY...", COLOR_LIGHTGREY, COLOR_BLACK, 1);
    }

    if (s_display_mutex) {
        xSemaphoreGive(s_display_mutex);
    }
}

// =============================================================================
// SCREEN TRANSITION APIS & SWIPE ROUTING
// =============================================================================
void display_set_screen(uint8_t screen_id) {
    if (screen_id >= SCREEN_COUNT) screen_id = SCREEN_HOME;
    ESP_LOGI(TAG, "display_set_screen: switching from %d to %d", (int)s_current_screen, (int)screen_id);

    if (s_display_mutex) {
        xSemaphoreTake(s_display_mutex, portMAX_DELAY);
    }

    s_current_screen = (screen_id_t)screen_id;
    s_hud.is_frame_active = false; // Reset HUD static frame flag so entering SCREEN_NAV initialises cleanly

    switch (s_current_screen) {
        case SCREEN_HOME:       display_show_home_screen(); break;
        case SCREEN_NAV:        display_show_main_screen(); break;
        case SCREEN_MUSIC:      display_set_screen(SCREEN_NAV); break; // Music removed — redirect to Nav
        case SCREEN_PICTURE:    display_show_user_image(); break;
        case SCREEN_SETTINGS:   display_show_settings_screen(); break;
        case SCREEN_BRIGHTNESS: display_show_brightness_screen(); break;
        case SCREEN_OTA:        display_show_ota_screen(0); break;
        default: break;
    }

    if (s_display_mutex) {
        xSemaphoreGive(s_display_mutex);
    }
}

uint8_t display_get_current_screen(void) {
    return (uint8_t)s_current_screen;
}

void display_next_screen(void) {
    if (s_current_screen == SCREEN_OTA) return;

    // Screen cycling order:
    // When navigating: HOME (0) -> NAV (1) -> PICTURE (3) -> SETTINGS (4) -> HOME (0)
    // When ride NOT started: HOME (0) -> PICTURE (3) -> SETTINGS (4) -> HOME (0)
    uint8_t next;
    if (s_current_screen == SCREEN_HOME) {
        if (s_nav.is_navigating) {
            next = SCREEN_NAV;
        } else {
            next = SCREEN_PICTURE; // Show image directly on swipe left from HOME when no ride is started!
        }
    } else if (s_current_screen == SCREEN_NAV) {
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
    if (s_current_screen == SCREEN_OTA) return;

    // Reverse screen cycling order (Swipe Right):
    uint8_t prev;
    if (s_current_screen == SCREEN_HOME) {
        prev = SCREEN_SETTINGS;
    } else if (s_current_screen == SCREEN_SETTINGS) {
        prev = SCREEN_PICTURE;
    } else if (s_current_screen == SCREEN_PICTURE) {
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
void display_handle_touch_event(int16_t start_x, int16_t start_y,
                                int16_t end_x,   int16_t end_y,
                                touch_gesture_t gesture) {
    // Completely lock touch input while firmware update is in progress
    if (s_current_screen == SCREEN_OTA) {
        ESP_LOGD(TAG, "Touch ignored: OTA update in progress");
        return;
    }

    switch (gesture) {
        case TOUCH_GESTURE_SWIPE_LEFT:
            // Dismiss confirmation modal on swipe
            s_show_reset_confirm = false;
            // Swipe Left -> Next screen in order Home -> Nav -> Music -> Picture -> Settings
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
            // Swiping down from top of screen (start_y <= 130) -> open Brightness screen with interactive slider
            if (start_y <= 130) {
                ESP_LOGI(TAG, "SWIPE DOWN (top gesture) -> Open Brightness Slider Screen");
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
            // Swiping up from top half (start_y <= 130) on other screens -> increase brightness
            if (start_y <= 130) {
                if (s_brightness_pct <= 90) s_brightness_pct += 10;
                else s_brightness_pct = 100;
                display_set_brightness(s_brightness_pct);
                ESP_LOGI(TAG, "SWIPE UP (top gesture) -> Brightness INC -> %d%%", s_brightness_pct);
                // Show high-visibility graphical brightness gauge
                show_brightness_hud(s_brightness_pct);
                vTaskDelay(pdMS_TO_TICKS(600));
                // Redraw current screen to cleanly dismiss HUD
                display_set_screen((uint8_t)s_current_screen);
            }
            break;

        case TOUCH_GESTURE_TAP: {
            int16_t tx = start_x;
            int16_t ty = start_y;
            ESP_LOGI(TAG, "TOUCH_GESTURE_TAP at (%d, %d) on screen %d", tx, ty, (int)s_current_screen);

            // N: Tap on active Notification Banner (Y: 30..90, X: 10..310) immediately dismisses it
            if (s_latest_notif.active && ty >= 30 && ty <= 90 && tx >= 10 && tx <= 310) {
                ESP_LOGI(TAG, "Touch: Notification Banner tapped -> Dismiss");
                display_dismiss_notification();
                return;
            }

            // G: Global Status Bar Tap Shortcuts (Y: 0..32) on any screen
            // Top-left area (X: 0..90): Return to HOME screen
            if (ty <= 32 && tx <= 90 && s_current_screen != SCREEN_HOME) {
                ESP_LOGI(TAG, "Touch: Top-left header tapped -> HOME");
                display_set_screen(SCREEN_HOME);
                return;
            }
            // Top-right area (X: 240..320): Quick jump to SETTINGS
            if (ty <= 32 && tx >= 240 && s_current_screen != SCREEN_SETTINGS) {
                ESP_LOGI(TAG, "Touch: Top-right header tapped -> SETTINGS");
                display_set_screen(SCREEN_SETTINGS);
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
                    strncpy(s_nav.current_street, "Recalculating route...", sizeof(s_nav.current_street) - 1);
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
                    ble_nav_notify_status(0x06); // NAV_STATUS_CANCEL_ROUTE
                    display_set_screen(SCREEN_HOME);
                    return;
                }
                // Ignore touches outside modal when it's open
                return;
            }

            // 0. Check NAVIGATION screen touch buttons
            if (s_current_screen == SCREEN_NAV) {
                // A: Recenter button (X: 10..46, Y: 138..172)
                if (tx >= 8 && tx <= 50 && ty >= 135 && ty <= 175) {
                    ESP_LOGI(TAG, "Touch: NAV RECENTER");
                    display_nav_recenter();
                    return;
                }
                // B: Zoom In button (X: 270..314, Y: 100..136)
                if (tx >= 268 && tx <= 316 && ty >= 98 && ty <= 138) {
                    ESP_LOGI(TAG, "Touch: NAV ZOOM IN");
                    display_nav_zoom_in();
                    return;
                }
                // C: Zoom Out / Overview button (X: 270..314, Y: 138..174)
                if (tx >= 268 && tx <= 316 && ty >= 138 && ty <= 176) {
                    ESP_LOGI(TAG, "Touch: NAV ZOOM OUT / OVERVIEW");
                    display_nav_zoom_out();
                    return;
                }
                // D: Compass tap -> toggle overview mode
                if (tx >= 280 && tx <= 316 && ty >= 30 && ty <= 60) {
                    ESP_LOGI(TAG, "Touch: NAV COMPASS / OVERVIEW TOGGLE");
                    display_nav_toggle_overview();
                    return;
                }
                // E: Bottom floating bar tap -> Return to HOME
                if (ty >= 210 && ty <= 240) {
                    ESP_LOGI(TAG, "Touch: Nav bottom bar tapped -> HOME");
                    display_set_screen(SCREEN_HOME);
                    return;
                }
            }

            // 1. Check SCREEN_BRIGHTNESS touch controls (Slider, [-], [+], [DONE])
            if (s_current_screen == SCREEN_BRIGHTNESS) {
                // [ - ] Button: X: 15..85, Y: 100..165 (Generous hitbox)
                if (tx >= 15 && tx <= 85 && ty >= 100 && ty <= 165) {
                    if (s_brightness_pct >= 5) s_brightness_pct -= 5;
                    else s_brightness_pct = 0;
                    display_set_brightness(s_brightness_pct);
                    ESP_LOGI(TAG, "Brightness screen: [-] pressed -> %d%%", s_brightness_pct);
                    display_show_brightness_screen();
                    return;
                }
                // [ + ] Button: X: 235..305, Y: 100..165 (Generous hitbox)
                if (tx >= 235 && tx <= 305 && ty >= 100 && ty <= 165) {
                    if (s_brightness_pct <= 95) s_brightness_pct += 5;
                    else s_brightness_pct = 100;
                    display_set_brightness(s_brightness_pct);
                    ESP_LOGI(TAG, "Brightness screen: [+] pressed -> %d%%", s_brightness_pct);
                    display_show_brightness_screen();
                    return;
                }
                // Interactive Slider Track: X: 86..234, Y: 110..160
                if (tx >= 86 && tx <= 234 && ty >= 110 && ty <= 160) {
                    int b_val = ((tx - 88) * 100) / 144;
                    if (b_val < 0) b_val = 0;
                    if (b_val > 100) b_val = 100;
                    display_set_brightness((uint8_t)b_val);
                    ESP_LOGI(TAG, "Brightness screen: slider adjusted -> %d%%", s_brightness_pct);
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

            // 2. Check HOME screen touch controls (tap bottom prompt to open Brightness screen)
            if (s_current_screen == SCREEN_HOME) {
                if (ty >= 180 && ty <= 240) {
                    ESP_LOGI(TAG, "Home bottom prompt tapped -> open Brightness screen");
                    display_set_screen(SCREEN_BRIGHTNESS);
                    return;
                }
            }

            // 3. Check SETTINGS screen controls
            if (s_current_screen == SCREEN_SETTINGS) {
                // If Brightness Popup Modal is open, handle [-], [+], slider, and [DONE]
                if (s_show_brightness_popup) {
                    // [ - ] Button: X: 35..105, Y: 105..160 (Generous 70x55 hitbox)
                    if (tx >= 35 && tx <= 105 && ty >= 105 && ty <= 160) {
                        if (s_brightness_pct >= 5) s_brightness_pct -= 5;
                        else s_brightness_pct = 0;
                        display_set_brightness(s_brightness_pct);
                        ESP_LOGI(TAG, "Brightness popup: [-] pressed -> %d%%", s_brightness_pct);
                        display_show_settings_screen();
                        return;
                    }
                    // [ + ] Button: X: 215..285, Y: 105..160 (Generous 70x55 hitbox)
                    if (tx >= 215 && tx <= 285 && ty >= 105 && ty <= 160) {
                        if (s_brightness_pct <= 95) s_brightness_pct += 5;
                        else s_brightness_pct = 100;
                        display_set_brightness(s_brightness_pct);
                        ESP_LOGI(TAG, "Brightness popup: [+] pressed -> %d%%", s_brightness_pct);
                        display_show_settings_screen();
                        return;
                    }
                    // Interactive Slider Track: X: 106..214, Y: 115..150
                    if (tx >= 106 && tx <= 214 && ty >= 115 && ty <= 150) {
                        int b_val = ((tx - 110) * 100) / 100;
                        if (b_val < 0) b_val = 0;
                        if (b_val > 100) b_val = 100;
                        display_set_brightness((uint8_t)b_val);
                        ESP_LOGI(TAG, "Brightness popup: slider dragged -> %d%%", s_brightness_pct);
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
                        ESP_LOGW(TAG, "Settings: Reset CONFIRMED by user! Resetting state to factory defaults...");
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

                // Display / Brightness row (Y: 65..115): opens dedicated Brightness Popup Modal!
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
                // Reset Device button (X: 162..320, Y: 150..225) -> triggers double-confirmation modal!
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
    if (brightness_pct > 100) brightness_pct = 100;
    s_brightness_pct = brightness_pct;
    tft_set_backlight(s_brightness_pct);
    if (s_is_initialized && s_current_screen == SCREEN_SETTINGS) {
        update_brightness_widgets();
    }
}

uint8_t display_get_brightness(void) {
    return s_brightness_pct;
}

static TaskHandle_t s_nav_render_task_handle = NULL;

static void nav_render_task(void *pvParam) {
    ESP_LOGI(TAG, "Navigation live HUD render task started (20 FPS)");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 FPS (50ms cadence)

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!s_is_initialized || s_current_screen != SCREEN_NAV) {
            continue;
        }

        if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(35)) == pdTRUE) {
            display_nav_render_partial();
            xSemaphoreGive(s_display_mutex);
        }
    }
}

void display_init(void) {
    if (s_is_initialized) return;

    if (!s_display_mutex) {
        s_display_mutex = xSemaphoreCreateMutex();
    }

    ESP_ERROR_CHECK(tft_init());
    touch_init();

    if (!s_nav_render_task_handle) {
        xTaskCreate(nav_render_task, "nav_render", 4096, NULL, 3, &s_nav_render_task_handle);
    }

    s_is_initialized = true;
}

void display_set_battery(uint8_t percentage) {
    // Battery logic removed
    (void)percentage;
}

void display_set_time(uint8_t hours, uint8_t minutes) {
    if (hours > 23) hours = 23;
    if (minutes > 59) minutes = 59;
    s_time_hours = hours;
    s_time_minutes = minutes;
    if (s_is_initialized && s_current_screen != SCREEN_COUNT) {
        if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            render_status_bar();
            if (s_current_screen == SCREEN_HOME) {
                // Redraw central clock cleanly
                char time_buf[16];
                snprintf(time_buf, sizeof(time_buf), "%02d:%02d", s_time_hours, s_time_minutes);
                tft_draw_string_centered(DISPLAY_WIDTH / 2, 95, time_buf, COLOR_WHITE, COLOR_BLACK, 5);
            }
            xSemaphoreGive(s_display_mutex);
        }
    }
}


void display_set_ble_status(bool connected) {
    if (s_ble_connected == connected) return;
    s_ble_connected = connected;
    ESP_LOGI(TAG, "BLE status changed: %s", connected ? "LINKED" : "SEARCHING");
    
    if (connected && s_nav_interrupted) {
        display_show_resume_route_modal();
    }
    
    if (s_is_initialized && s_current_screen != SCREEN_COUNT) {
        if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            render_status_bar();
            xSemaphoreGive(s_display_mutex);
        }
    }
}

void display_set_speed(uint16_t speed_kmh) {
    s_speed_kmh = speed_kmh;
    s_nav.speed_kmh = speed_kmh;
    // 20 FPS nav_render_task smoothly renders the updated speed badge without whole-screen refresh
}

void display_update_navigation(const navigation_data_t *nav_data) {
    if (!nav_data) return;
    if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(&s_nav, nav_data, sizeof(navigation_data_t));
        s_speed_kmh = nav_data->speed_kmh;
        xSemaphoreGive(s_display_mutex);
    } else {
        memcpy(&s_nav, nav_data, sizeof(navigation_data_t));
        s_speed_kmh = nav_data->speed_kmh;
    }
    
    if (nav_data->is_navigating) {
        s_nav_was_active = true;
        s_nav_interrupted = false;
    } else if (nav_data->direction == NAV_DIR_REACHED) {
        s_nav_was_active = false;
        s_nav_interrupted = false;
    }

    // Decoupled architecture: The incoming BLE packet updates target navigation state in memory.
    // The FreeRTOS 20 FPS nav_render_task smoothly interpolates heading and renders dirty regions.
}

void display_update_route_segment(const route_segment_t *seg) {
    if (!seg) return;
    if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(&s_route_seg, seg, sizeof(route_segment_t));
        s_anim_step = 0;
        xSemaphoreGive(s_display_mutex);
    } else {
        memcpy(&s_route_seg, seg, sizeof(route_segment_t));
        s_anim_step = 0;
    }
}

void display_clear(void) {
    if (s_display_mutex) xSemaphoreTake(s_display_mutex, portMAX_DELAY);
    tft_fill_screen(COLOR_SURFACE_DARK);
    if (s_display_mutex) xSemaphoreGive(s_display_mutex);
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
    ESP_LOGI(TAG, "Nav Zoom In: Level %d", (int)s_zoom_level);
    if (s_is_initialized && s_current_screen == SCREEN_NAV) {
        display_show_main_screen();
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
    ESP_LOGI(TAG, "Nav Zoom Out: Level %d, Mode %d", (int)s_zoom_level, (int)s_camera_mode);
    if (s_is_initialized && s_current_screen == SCREEN_NAV) {
        display_show_main_screen();
    }
}

void display_nav_recenter(void) {
    s_camera_mode = NAV_CAMERA_FOLLOW;
    s_zoom_level = NAV_ZOOM_LEVEL_3; // Reset to standard 500m follow view
    s_pan_offset_x = 0;
    s_pan_offset_y = 0;
    ESP_LOGI(TAG, "Nav Recenter -> Follow Mode at Zoom Level 3 (500m)");
    if (s_is_initialized && s_current_screen == SCREEN_NAV) {
        display_show_main_screen();
    }
}

void display_nav_toggle_overview(void) {
    if (s_camera_mode == NAV_CAMERA_OVERVIEW) {
        display_nav_recenter();
    } else {
        s_camera_mode = NAV_CAMERA_OVERVIEW;
        s_zoom_level = NAV_ZOOM_LEVEL_5;
        ESP_LOGI(TAG, "Nav Toggle -> Route Overview");
        if (s_is_initialized && s_current_screen == SCREEN_NAV) {
            display_show_main_screen();
        }
    }
}

void display_nav_pan(int16_t dx, int16_t dy) {
    s_camera_mode = NAV_CAMERA_MANUAL;
    s_pan_offset_x += dx;
    s_pan_offset_y += dy;
    // Clamp panning within sensible bounds
    if (s_pan_offset_x > 120) s_pan_offset_x = 120;
    if (s_pan_offset_x < -120) s_pan_offset_x = -120;
    if (s_pan_offset_y > 80) s_pan_offset_y = 80;
    if (s_pan_offset_y < -80) s_pan_offset_y = -80;
    if (s_is_initialized && s_current_screen == SCREEN_NAV) {
        display_show_main_screen();
    }
}

nav_zoom_level_t display_nav_get_zoom(void) {
    return s_zoom_level;
}

nav_camera_mode_t display_nav_get_camera_mode(void) {
    return s_camera_mode;
}


// =============================================================================
// LATEST NOTIFICATION APIS & AUTO-DISMISS TIMER (Strictly Latest Only)
// =============================================================================

static void notif_auto_dismiss_timer_cb(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Notification auto-dismiss timer fired (2s)");
    display_dismiss_notification();
}

void display_show_notification(const char *title, const char *message) {
    if (!s_is_initialized) return;

    if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Strictly replace previous notification with the latest one
        memset(&s_latest_notif, 0, sizeof(s_latest_notif));
        if (title && title[0]) {
            strncpy(s_latest_notif.title, title, sizeof(s_latest_notif.title) - 1);
        } else {
            strncpy(s_latest_notif.title, "Notification", sizeof(s_latest_notif.title) - 1);
        }

        if (message && message[0]) {
            strncpy(s_latest_notif.message, message, sizeof(s_latest_notif.message) - 1);
        } else {
            strncpy(s_latest_notif.message, "New alert", sizeof(s_latest_notif.message) - 1);
        }
        s_latest_notif.active = true;

        // Render banner directly over current screen
        render_notification_banner();

        xSemaphoreGive(s_display_mutex);
    }

    // Start or reset 2-second auto-dismiss timer
    if (!s_notif_timer) {
        s_notif_timer = xTimerCreate("notif_timer", pdMS_TO_TICKS(2000), pdFALSE, NULL, notif_auto_dismiss_timer_cb);
    }
    if (s_notif_timer) {
        xTimerStop(s_notif_timer, 0);
        xTimerStart(s_notif_timer, 0);
    }
}

void display_dismiss_notification(void) {
    if (!s_latest_notif.active) return;

    if (s_notif_timer) {
        xTimerStop(s_notif_timer, 0);
    }

    if (s_display_mutex && xSemaphoreTake(s_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_latest_notif.active = false;
        memset(&s_latest_notif, 0, sizeof(s_latest_notif));

        // Redraw current screen to cleanly clear the notification banner overlay
        switch (s_current_screen) {
            case SCREEN_HOME:     display_show_home_screen(); break;
            case SCREEN_NAV:      display_show_main_screen(); break;
            case SCREEN_MUSIC:    display_show_main_screen(); break; // Music removed — same as Nav
            case SCREEN_PICTURE:  display_show_user_image(); break;
            case SCREEN_SETTINGS: display_show_settings_screen(); break;
            default: break;
        }

        xSemaphoreGive(s_display_mutex);
    }
}

bool display_is_notification_active(void) {
    return s_latest_notif.active;
}
