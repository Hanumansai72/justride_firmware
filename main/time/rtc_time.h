#ifndef RTC_TIME_H
#define RTC_TIME_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Real-Time Clock engine and start the background FreeRTOS
 *        task that automatically updates the display clock every minute.
 */
esp_err_t time_init(void);

/**
 * @brief Set the RTC time directly (e.g. from phone sync or manual input).
 *        Immediately updates the display with the new real-time value.
 *
 * @param hours Hours (0-23)
 * @param minutes Minutes (0-59)
 * @param seconds Seconds (0-59)
 */
void time_set_current_time(uint8_t hours, uint8_t minutes, uint8_t seconds);

/**
 * @brief Set the RTC time using a standard 32-bit Unix epoch timestamp.
 *
 * @param epoch_seconds Seconds since 1970-01-01 00:00:00 UTC
 */
void time_set_epoch(uint32_t epoch_seconds);

/**
 * @brief Check whether the time has been synchronized with the phone.
 *
 * @return true if synchronized, false if using default boot time
 */
bool time_is_synchronized(void);

#ifdef __cplusplus
}
#endif

#endif // RTC_TIME_H
