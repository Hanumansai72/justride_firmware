#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// JustRide firmware modules
#include "display.h"
#include "rtc_time.h"
#include "ble.h"
#include "ota.h"
#include "buttons.h"
#include "mpu6500.h"

static const char *TAG = "JUSTRIDE_MAIN";

// =============================================================================
// TOUCH GESTURE FSM ENGINE
// =============================================================================
//
// State machine:
//   IDLE -> PRESSED (on first valid touch)
//   PRESSED -> IDLE (on finger lift)
//
// On lift: computes displacement (dx, dy) across the full drag and fires
// the correct gesture event to display_handle_touch_event().
// The display handler is ONLY called on clean gesture boundaries —
// never spammed on every poll tick like the old implementation.
//
// Gesture thresholds:
//   Swipe Right: dx > SWIPE_MIN_PX and abs(dy) < SWIPE_MAX_SKEW
//   Swipe Left:  dx < -SWIPE_MIN_PX and abs(dy) < SWIPE_MAX_SKEW
//   Tap:         total travel < TAP_MAX_PX
// =============================================================================

#define SWIPE_MIN_PX       40   // 40 px minimum horizontal displacement for swipes
#define TAP_MAX_PX         28   // Max movement (px) still classified as a tap (forgiving for finger rolls)
#define DEBOUNCE_TICKS      1   // 1 tick (20ms) = immediate touch response

typedef enum {
    GESTURE_IDLE = 0,
    GESTURE_PRESSED,
} gesture_state_t;

static void touch_task(void *pvParam) {
    touch_point_t pt;

    gesture_state_t state   = GESTURE_IDLE;
    int16_t  start_x        = 0;
    int16_t  start_y        = 0;
    int16_t  last_x         = 0;
    int16_t  last_y         = 0;
    uint32_t press_ms       = 0;
    int      confirm_count  = 0;   // Debounce counter

    while (1) {
        bool is_down = touch_get_point(&pt);

        switch (state) {

            // -----------------------------------------------------------------
            case GESTURE_IDLE:
                if (is_down) {
                    confirm_count++;
                    if (confirm_count >= DEBOUNCE_TICKS) {
                        // Confirmed touch start
                        start_x      = pt.x;
                        start_y      = pt.y;
                        last_x       = pt.x;
                        last_y       = pt.y;
                        press_ms     = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        state        = GESTURE_PRESSED;
                        confirm_count = 0;
                        ESP_LOGD(TAG, "TOUCH DOWN: (%d, %d)", start_x, start_y);
                    }
                } else {
                    confirm_count = 0;
                }
                break;

            // -----------------------------------------------------------------
            case GESTURE_PRESSED:
                if (is_down) {
                    last_x = pt.x;
                    last_y = pt.y;
                } else {
                    // Finger lifted — compute gesture
                    int16_t dx = last_x - start_x;
                    int16_t dy = last_y - start_y;
                    uint32_t held_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - press_ms;

                    ESP_LOGD(TAG, "TOUCH UP: start(%d,%d) -> end(%d,%d) | dx=%d dy=%d held=%lums",
                             start_x, start_y, last_x, last_y, dx, dy, (unsigned long)held_ms);

                    // Master Prompt §12: Require abs(dx) > abs(dy) * 1.5 to prevent diagonal drifts
                    if (dx >= SWIPE_MIN_PX && (abs(dx) * 2 > abs(dy) * 3)) {
                        // ✅ SWIPE RIGHT (Prev App / Prev Image)
                        ESP_LOGI(TAG, "GESTURE: SWIPE RIGHT (dx=+%d)", dx);
                        display_handle_touch_event(start_x, start_y, last_x, last_y, TOUCH_GESTURE_SWIPE_RIGHT);

                    } else if (dx <= -SWIPE_MIN_PX && (abs(dx) * 2 > abs(dy) * 3)) {
                        // ✅ SWIPE LEFT (Next App / Next Image)
                        ESP_LOGI(TAG, "GESTURE: SWIPE LEFT (dx=%d)", dx);
                        display_handle_touch_event(start_x, start_y, last_x, last_y, TOUCH_GESTURE_SWIPE_LEFT);

                    } else if (dy >= SWIPE_MIN_PX && (abs(dy) * 2 > abs(dx) * 3)) {
                        // ✅ SWIPE DOWN -> brightness decrease (top zone) or Settings (elsewhere)
                        ESP_LOGI(TAG, "GESTURE: SWIPE DOWN (dy=+%d, start_y=%d)", dy, start_y);
                        display_handle_touch_event(start_x, start_y, last_x, last_y, TOUCH_GESTURE_SWIPE_DOWN);

                    } else if (dy <= -SWIPE_MIN_PX && (abs(dy) * 2 > abs(dx) * 3)) {
                        // ✅ SWIPE UP -> brightness increase (when originating from top of screen)
                        ESP_LOGI(TAG, "GESTURE: SWIPE UP (dy=%d, start_y=%d)", dy, start_y);
                        display_handle_touch_event(start_x, start_y, last_x, last_y, TOUCH_GESTURE_SWIPE_UP);

                    } else if (abs(dx) <= TAP_MAX_PX && abs(dy) <= TAP_MAX_PX) {
                        // ✅ TAP
                        ESP_LOGI(TAG, "GESTURE: TAP at (%d, %d)", start_x, start_y);
                        display_handle_touch_event(start_x, start_y, start_x, start_y, TOUCH_GESTURE_TAP);

                    } else if (abs(dx) < SWIPE_MIN_PX && abs(dy) < SWIPE_MIN_PX) {
                        // Small movement that didn't meet full swipe threshold -> treat as tap at start position
                        ESP_LOGI(TAG, "GESTURE: Short Drag fallback to TAP at (%d, %d)", start_x, start_y);
                        display_handle_touch_event(start_x, start_y, start_x, start_y, TOUCH_GESTURE_TAP);

                    } else {
                        ESP_LOGD(TAG, "GESTURE: unclassified drag (dx=%d, dy=%d)", dx, dy);
                    }

                    state         = GESTURE_IDLE;
                    confirm_count = 0;
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Master Prompt §8: 20 ms periodic touch polling (50 Hz)
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Boot reset reason: %d", (int)esp_reset_reason());
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "       JustRide Motorcycle Device       ");
    ESP_LOGI(TAG, "     2.8\" TFT SPI HUD & Dual OTA       ");
    ESP_LOGI(TAG, "========================================");

    // --------------------------------------------------
    // STEP 1: OTA Partition Inspector
    // --------------------------------------------------
    ota_init();

    // --------------------------------------------------
    // STEP 2: 2.8" SPI TFT & Touch Initialization
    // --------------------------------------------------
    display_init();

    // Show high-contrast boot splash screen (MADE FOR BIKES) and transition to HUD
    display_show_splash();

    // --------------------------------------------------
    // STEP 3: Real-Time Clock Engine
    // --------------------------------------------------
    time_init();

    // --------------------------------------------------
    // STEP 4: Physical Push-buttons
    // --------------------------------------------------
    buttons_init();

    // --------------------------------------------------
    // STEP 5: BLE Stack & Real-Time Sync
    // --------------------------------------------------
    ble_init();

    // --------------------------------------------------
    // STEP 6: MPU6500 IMU — Gyro-based turn arrow rotation
    // GPIO 21 = SDA, GPIO 22 = SCL, NCS pulled to 3.3V
    // --------------------------------------------------
    esp_err_t imu_ret = mpu6500_init();
    if (imu_ret == ESP_OK) {
        mpu6500_start_task(); // Launches 50 Hz yaw integration -> display arrow
    } else {
        ESP_LOGW(TAG, "MPU6500 init failed (%s) - IMU arrow rotation disabled",
                 esp_err_to_name(imu_ret));
    }

    // Launch background touch input monitoring task
    xTaskCreate(touch_task, "touch_task", 3072, NULL, 5, NULL);

    // Initial HUD state: Ready for Ride / Awaiting Google Navigation over BLE
    navigation_data_t initial_nav = {
        .direction = NAV_DIR_STRAIGHT,
        .distance_meters = 0,
        .current_street = "Ready for Ride",
        .eta_time_str = "--:--",
        .remaining_dist_km = 0.0f,
        .speed_kmh = 0,
        .is_navigating = false,
    };
    display_update_navigation(&initial_nav);

    ESP_LOGI(TAG, "All subsystems running. 2.8\" HUD operational.");

    // Keep rollback available if the new image fails during initial startup.
    vTaskDelay(pdMS_TO_TICKS(10000));
    ota_confirm_boot_valid();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
