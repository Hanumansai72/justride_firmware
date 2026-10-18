#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "display_tft.h"
#include "touch.h"

// =============================================================================
// GESTURE EVENT TYPE
// Produced by the FSM in main.c, consumed by display_handle_touch_event()
// =============================================================================
typedef enum {
    TOUCH_GESTURE_TAP          = 0,  ///< Short stationary press (no significant drag)
    TOUCH_GESTURE_SWIPE_RIGHT  = 1,  ///< Finger moved left-to-right > SWIPE_MIN_PX
    TOUCH_GESTURE_SWIPE_LEFT   = 2,  ///< Finger moved right-to-left > SWIPE_MIN_PX
    TOUCH_GESTURE_SWIPE_DOWN   = 3,  ///< Finger moved top-to-bottom > SWIPE_MIN_PX
    TOUCH_GESTURE_SWIPE_UP     = 4,  ///< Finger moved bottom-to-top > SWIPE_MIN_PX
} touch_gesture_t;


#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// MOTORCYCLE HUD NAVIGATION DATA STRUCTURES
// =============================================================================
#define ROUTE_MAX_POINTS 16

typedef struct {
    uint16_t x;
    uint16_t y;
} route_point_t;

typedef struct {
    uint8_t count;
    uint8_t completed_segment_count;
    uint8_t progress_percent;
    bool is_destination_visible;
    uint16_t dest_x;
    uint16_t dest_y;
    route_point_t points[ROUTE_MAX_POINTS];
} route_segment_t;

typedef struct {
    char city[19];
    uint8_t poi_type;
    uint16_t poi_distance_m;
    uint16_t poi_x;
    uint16_t poi_y;
    char poi_name[19];
} nav_context_t;

typedef struct {
    nav_direction_t direction;    // Turn direction enum
    uint32_t distance_meters;     // Distance to next turn
    char current_street[32];      // Primary navigation maneuver description
    char eta_time_str[16];        // Estimated arrival (e.g. "12:35 PM" or "999 min")
    float remaining_dist_km;      // Total route remaining distance (e.g. 4.2 km)
    uint16_t speed_kmh;           // Live vehicle speed (km/h)
    uint16_t heading_deg;         // Vehicle heading in degrees (0..359)
    bool is_navigating;           // Active turn-by-turn guidance flag
    bool gps_available;           // Satellite lock validity
    bool rerouting;               // Route recalculation flag
    uint16_t maneuver_id;         // Unique identifier for current maneuver
    uint8_t maneuver_state;       // Maneuver state (0:NONE, 1:STRAIGHT, 2:APPROACHING, 3:PREVIEW, 4:ACTIVE, 5:COMPLETED)
    nav_direction_t next_direction; // Lookahead / Secondary upcoming maneuver direction enum
    uint32_t next_distance_meters;  // Distance for next maneuver (meters)
    char next_street[32];           // Next maneuver description / street name (e.g. "Continue Straight", "MG Road")
} navigation_data_t;


// =============================================================================
// NAVIGATION CAMERA & ZOOM MODES
// =============================================================================
typedef enum {
    NAV_CAMERA_FOLLOW = 0,   ///< Normal riding mode: camera tracks rider at origin (160, 145)
    NAV_CAMERA_MANUAL,       ///< User dragged map or manually inspected route
    NAV_CAMERA_OVERVIEW,     ///< Full route auto-fit on 320x240 canvas
} nav_camera_mode_t;

typedef enum {
    NAV_ZOOM_LEVEL_1 = 1,    ///< ~100m - High detail junction view (2.0x)
    NAV_ZOOM_LEVEL_2 = 2,    ///< ~250m - Close approach view (1.5x)
    NAV_ZOOM_LEVEL_3 = 3,    ///< ~500m - Standard Navigation Follow (1.0x - default)
    NAV_ZOOM_LEVEL_4 = 4,    ///< ~1500m - Wide spatial context (0.5x)
    NAV_ZOOM_LEVEL_5 = 5,    ///< Full route overview / bounding-box fit
} nav_zoom_level_t;

// =============================================================================
// Screen Identifiers
typedef enum {
    SCREEN_HOME = 0,
    SCREEN_NAV,
    SCREEN_PICTURE,
    SCREEN_SETTINGS,
    SCREEN_BRIGHTNESS,
    SCREEN_OTA,
    SCREEN_COUNT
} screen_id_t;

// =============================================================================
// PUBLIC DISPLAY APIS (Preserved & Enhanced)
// =============================================================================

/** Initialize 2.8" SPI TFT LCD, DMA buffer, and Touch Controller */
void display_init(void);

/** Show the JustRide motorcycle boot splash screen */
void display_show_splash(void);

/** Screen Show Functions */
void display_show_home_screen(void);
void display_show_main_screen(void);       // Screen 1: Navigation
void display_show_user_image(void);        // Screen 2: Picture / Gallery
void display_show_settings_screen(void);   // Screen 3: Settings
void display_show_brightness_screen(void); // Dedicated Brightness Control Screen

/** Show Firmware Flash / OTA Updating screen with live percentage */
void display_show_ota_screen(uint8_t progress_pct);

/** Switch screen state: 0=Home, 1=Nav, 2=Picture, 3=Settings */
void display_set_screen(uint8_t screen_id);
uint8_t display_get_current_screen(void);

/** Cycle screen left / right via swipe gestures */
void display_next_screen(void);
void display_prev_screen(void);

/** Set display brightness level (0 to 100%) */
void display_set_brightness(uint8_t brightness_pct);
uint8_t display_get_brightness(void);

/** Display power management: turn screen backlight and rendering on/off */
void display_set_power(bool on);
bool display_get_power(void);

/** Update live vehicle speed (km/h) */
void display_set_speed(uint16_t speed_kmh);

/**
 * @brief Update the IMU-derived heading from MPU6500 gyro integration.
 *        When the bike physically turns left or right, this heading causes
 *        the navigation turn arrow on the HUD to rotate accordingly.
 *        ONLY modifies the IMU overlay — does NOT change BLE/GPS nav state.
 * @param heading_deg  Integrated yaw heading in degrees [0..360)
 */
void display_set_imu_heading(float heading_deg);

/**
 * @brief Directly set the IMU arrow override direction on the HUD.
 *        Supports NAV_DIR_STRAIGHT (Forward), NAV_DIR_UTURN (Backward),
 *        NAV_DIR_LEFT, NAV_DIR_RIGHT, or NAV_DIR_NONE (Level/Route).
 * @param dir  Navigation direction override
 */
void display_set_imu_override_direction(nav_direction_t dir);

/**
 * @brief Process a completed gesture event from the FSM touch engine.
 * @param start_x  X pixel at touch-down (used for tap hit-testing)
 * @param start_y  Y pixel at touch-down
 * @param end_x    X pixel at finger lift (used for swipe delta)
 * @param end_y    Y pixel at finger lift
 * @param gesture  Classified gesture: TAP, SWIPE_RIGHT, or SWIPE_LEFT
 */
void display_handle_touch_event(int16_t start_x, int16_t start_y,
                                int16_t end_x,   int16_t end_y,
                                touch_gesture_t gesture);

/** Update real-time clock (called by rtc_time.c) */
void display_set_time(uint8_t hours, uint8_t minutes);

/** Update live BLE connection status indicator */
void display_set_ble_status(bool connected);

/** Push navigation packet from BLE or internal GPS to HUD screen */
void display_update_navigation(const navigation_data_t *nav_data);

/** Push dynamic 500m sliding route window geometry */
void display_update_route_segment(const route_segment_t *seg);
void display_update_nav_context(const nav_context_t *context);


/** Show the latest notification popup on the display (replaces any previous notification) */
void display_show_notification(const char *title, const char *message);

/** Dismiss the currently active notification popup immediately */
void display_dismiss_notification(void);

/** Check whether a notification banner is currently active */
bool display_is_notification_active(void);

/** Navigation Map Camera & Zoom Controls */
void display_nav_zoom_in(void);
void display_nav_zoom_out(void);
void display_nav_recenter(void);
void display_nav_toggle_overview(void);
void display_nav_pan(int16_t dx, int16_t dy);
nav_zoom_level_t display_nav_get_zoom(void);
nav_camera_mode_t display_nav_get_camera_mode(void);

/** On-screen Route Resume Modal (displayed when power / BLE link / GPS recovers) */
void display_show_resume_route_modal(void);
void display_dismiss_resume_route_modal(void);
bool display_is_route_interrupted(void);

/** Clear active display */
void display_clear(void);

// =============================================================================
// USER IMAGE APIs — Send a 320×240 RGB565 image from the mobile app via BLE
// =============================================================================

/**
 * @brief Begin direct image streaming. Switches screen to Image View and opens full-screen window.
 */
void display_start_image_stream(void);

/**
 * @brief Push a stream chunk of RGB565 pixels directly into the display's GRAM.
 */
void display_write_image_chunk(const uint8_t *chunk, size_t len);

/**
 * @brief Finalize image stream and draw UI controls over the image.
 */
void display_finish_image_stream(void);

/** Show the stored user image screen (called on SWIPE RIGHT gesture). */
void display_show_user_image(void);

/** Returns true if a valid user image has been received and stored. */
bool display_has_user_image(void);


#ifdef __cplusplus
}
#endif

#endif // DISPLAY_H
