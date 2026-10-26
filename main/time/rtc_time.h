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
 * @brief Restore the last saved calendar time from NVS before display startup,
 *        then update the clock and checkpoint to flash each minute.
 */
esp_err_t time_init(void);

/** Save the current full calendar time to flash for the next boot. */
void time_save_checkpoint(void);

/** Set a complete local calendar date/time from Bluetooth CTS. Reject invalid dates. */
bool time_set_datetime(uint16_t year, uint8_t month, uint8_t day,
                       uint8_t hours, uint8_t minutes, uint8_t seconds);

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
 * @return true if synchronized or restored from a valid saved clock
 */
bool time_is_synchronized(void);

/**
 * @brief Get the current system date string formatted as e.g. "WED, 09 SEP 2026".
 */
void time_get_date_str(char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif // RTC_TIME_H
