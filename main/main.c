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
#define RELEASE_CONFIRM_TICKS 3 // Ignore up to two noisy/edge samples during a swipe

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
    int      release_count  = 0;
    int      max_dx = 0, max_dy = 0;
    bool tap_pending=false, second_tap=false, zoom_drag=false;
    int16_t tap_x=0,tap_y=0,zoom_y=0;
    uint32_t tap_ms=0,zoom_ms=0;

    while (1) {
        bool is_down = touch_get_point(&pt);
        uint32_t now=xTaskGetTickCount()*portTICK_PERIOD_MS;

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
                        max_dx = 0;
                        max_dy = 0;
                        press_ms     = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        second_tap=tap_pending && now-tap_ms<=320 &&
                            abs(pt.x-tap_x)<=20 && abs(pt.y-tap_y)<=20 &&
                            display_nav_touch_zoom_allowed(pt.x,pt.y);
                        tap_pending=false;
                        zoom_drag=false;zoom_y=pt.y;zoom_ms=now;
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
                    release_count = 0;
                    last_x = pt.x;
                    last_y = pt.y;
                    if (abs(pt.x - start_x) > max_dx) max_dx = abs(pt.x - start_x);
                    if (abs(pt.y - start_y) > max_dy) max_dy = abs(pt.y - start_y);
                    if(second_tap) {
                        // Only the second press of a short map tap can claim a
                        // vertical drag. Ordinary swipes retain their meaning.
                        if(!display_nav_touch_zoom_allowed(start_x,start_y)) {
                            second_tap=false;
                            zoom_drag=true; // Consume until release, never tap a new overlay.
                        } else if(!zoom_drag && abs(pt.x-start_x)>28 &&
                                  abs(pt.x-start_x)>=abs(pt.y-start_y)) {
                            second_tap=false;
                        } else if((zoom_drag || abs(pt.y-start_y)*2>abs(pt.x-start_x)*3) &&
                                  abs(pt.y-zoom_y)>=32 && now-zoom_ms>=120) {
                            zoom_drag=true;
                            display_handle_touch_event(start_x,start_y,pt.x,pt.y,
                                pt.y<zoom_y?TOUCH_GESTURE_ZOOM_IN:TOUCH_GESTURE_ZOOM_OUT);
                            zoom_y=pt.y;zoom_ms=now;
                        }
                    }
                } else {
                    // One noisy/edge sample is not a finger release. Splitting
                    // a swipe into taps can accidentally activate controls.
                    if (++release_count < RELEASE_CONFIRM_TICKS) break;
                    release_count = 0;
                    // Finger lifted — compute gesture
                    int16_t dx = last_x - start_x;
                    int16_t dy = last_y - start_y;
                    uint32_t held_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - press_ms;

                    ESP_LOGD(TAG, "TOUCH UP: start(%d,%d) -> end(%d,%d) | dx=%d dy=%d held=%lums",
                             start_x, start_y, last_x, last_y, dx, dy, (unsigned long)held_ms);

                    // Master Prompt §12: Require abs(dx) > abs(dy) * 1.5 to prevent diagonal drifts
                    if(zoom_drag) {
                        // All zoom steps already emitted; no release swipe/tap.
                    } else if (dx >= SWIPE_MIN_PX && (abs(dx) * 2 > abs(dy) * 3)) {
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

                    } else if (max_dx <= TAP_MAX_PX && max_dy <= TAP_MAX_PX) {
                        // ✅ TAP
                        ESP_LOGI(TAG, "GESTURE: TAP at (%d, %d)", start_x, start_y);
                        bool short_tap=held_ms<=280 && max_dx<=12 && max_dy<=12;
                        display_handle_touch_event(start_x, start_y, start_x, start_y,
                            second_tap && short_tap?TOUCH_GESTURE_DOUBLE_TAP:TOUCH_GESTURE_TAP);
                        tap_pending=!second_tap && short_tap &&
                            display_nav_touch_zoom_allowed(start_x,start_y);
                        tap_x=start_x;tap_y=start_y;tap_ms=now;

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
#if CONFIG_JUSTRIDE_ROUND_AMOLED
    ESP_LOGI(TAG, "  ESP32-S3 1.43 AMOLED / BLE / Dual OTA ");
#else
    ESP_LOGI(TAG, "     2.8\" TFT SPI HUD & Dual OTA       ");
#endif
    ESP_LOGI(TAG, "========================================");

    // --------------------------------------------------
    // STEP 1: OTA Partition Inspector
    // --------------------------------------------------
    ota_init();

    // Restore the saved calendar before any home-screen frame is drawn.
    ESP_ERROR_CHECK(time_init());

    // --------------------------------------------------
    // STEP 2: 2.8" SPI TFT & Touch Initialization
    // --------------------------------------------------
    display_init();

    // Show high-contrast boot splash screen (MADE FOR BIKES) and transition to HUD
    display_show_splash();

    // --------------------------------------------------
    // STEP 4: BLE Stack & Real-Time Sync
    // --------------------------------------------------
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


    ble_init();

    // --------------------------------------------------
    // STEP 5: MPU6500 IMU — Gyro-based turn arrow rotation
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
    BaseType_t touch_created = xTaskCreate(touch_task, "touch_task", 3072, NULL, 5, NULL);
    configASSERT(touch_created == pdPASS);

#if CONFIG_JUSTRIDE_ROUND_AMOLED
    ESP_LOGI(TAG,"1.43-inch Waveshare AMOLED running. NimBLE logs its BLE address.");
#else
    ESP_LOGI(TAG, "All subsystems running. 2.8\" HUD operational.");
#endif

    // Keep rollback available if the new image fails during initial startup.
    vTaskDelay(pdMS_TO_TICKS(10000));
    ota_confirm_boot_valid();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
