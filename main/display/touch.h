/**
 * @file touch.h
 * @brief Professional XPT2046 resistive touchscreen driver for JustRide HUD.
 *
 * Wiring (2.8" display module, shared SPI bus with ILI9341):
 *   T_CS  -> GPIO 27  (unique chip select)
 *   T_CLK -> GPIO 18  (shared SPI clock)
 *   T_DIN -> GPIO 23  (shared MOSI)
 *   T_DO  -> GPIO 19  (shared MISO — MUST be connected for touch to work!)
 *   T_IRQ -> leave disconnected (pure SPI pressure detection used)
 */

#ifndef TOUCH_H
#define TOUCH_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// PIN CONFIGURATION
// =============================================================================
#define TOUCH_PIN_CS    27   ///< XPT2046 Chip Select (active LOW, GPIO 27)
#define TOUCH_PIN_IRQ   -1   ///< -1 = pure SPI pressure polling (no IRQ needed)

// =============================================================================
// RESISTIVE CALIBRATION CONSTANTS
// If touch coordinates are offset, adjust MIN/MAX to match your panel edges.
// =============================================================================
#define TOUCH_RAW_MIN_X     300
#define TOUCH_RAW_MAX_X     3800
#define TOUCH_RAW_MIN_Y     250
#define TOUCH_RAW_MAX_Y     3850

/// Minimum Z1 pressure ADC count
/// Lowered from 400 to 180 for highly sensitive light taps
#define TOUCH_PRESSURE_THRESHOLD    180

/// Touch edge margin (pixels) to reject accidental bezel touches
#define TOUCH_EDGE_MARGIN           4

// =============================================================================
// TYPES
// =============================================================================

/** Touch point in landscape screen coordinates (320x240) */
typedef struct {
    int16_t  x;          ///< Pixel X (0 = left edge, 319 = right edge)
    int16_t  y;          ///< Pixel Y (0 = top edge, 239 = bottom edge)
    uint16_t pressure;   ///< Raw pressure ADC (higher = harder press)
    bool     is_pressed; ///< true when screen is genuinely touched
} touch_point_t;

// =============================================================================
// PUBLIC API
// =============================================================================

/** Initialize XPT2046 on the shared SPI bus. Call after tft_init(). */
esp_err_t touch_init(void);

/**
 * @brief  Read the instantaneous touch state.
 * @param  point  Output: populated with x, y, pressure, is_pressed.
 * @return true   Screen is being touched and coordinates are valid.
 * @return false  Not touched, or reading failed.
 */
bool touch_get_point(touch_point_t *point);

#ifdef __cplusplus
}
#endif

#endif // TOUCH_H
