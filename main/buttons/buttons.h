#ifndef BUTTONS_H
#define BUTTONS_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// PHYSICAL BUTTON PIN CONFIGURATION
// =============================================================================
// Connect a momentary push-button between GPIO 14 and GND.
// Internal pull-up is active (no external resistor required).
// GPIO 14 is an RTC GPIO on ESP32 that supports wake from Deep Sleep.
#define BUTTON_PIN_POWER        14

/**
 * @brief Initialize button GPIO, internal pull-up, and background debounced polling task.
 * @return ESP_OK on success.
 */
esp_err_t buttons_init(void);

/**
 * @brief Check if the display is currently turned on.
 */
bool buttons_is_display_on(void);

/**
 * @brief Toggle display power state (turn backlight on/off).
 */
void buttons_toggle_display_power(void);

#ifdef __cplusplus
}
#endif

#endif // BUTTONS_H
