#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ADC Oneshot Unit and ADC Calibration for battery voltage monitoring.
 *        Starts the background FreeRTOS task that dynamically updates the display.
 */
void battery_init(void);

/**
 * @brief Measure the current battery voltage in Volts.
 *
 * @return float Battery voltage in Volts (e.g. 4.12f). Returns -1.0f on error.
 */
float battery_get_voltage(void);

/**
 * @brief Estimate remaining battery capacity percentage (0 to 100%).
 *
 * @return int Battery percentage from 0 to 100.
 *         Returns 0 when in USB-only mode (no battery).
 */
int battery_get_percentage(void);

/**
 * @brief Returns true when no LiPo battery is detected.
 *        ADC voltage < 2.5V means the divider pin is floating (USB only).
 */
bool battery_is_usb_only(void);

/**
 * @brief Returns true when battery is actively charging (Vbat > 4.22V).
 */
bool battery_is_charging(void);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_H
