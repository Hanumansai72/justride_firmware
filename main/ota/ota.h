#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_ota_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OTA_STATUS_IDLE = 0,
    OTA_STATUS_READY,
    OTA_STATUS_RECEIVING,
    OTA_STATUS_VERIFYING,
    OTA_STATUS_SUCCESS,
    OTA_STATUS_ERROR_BATTERY_LOW,
    OTA_STATUS_ERROR_FLASH_WRITE,
    OTA_STATUS_ERROR_VALIDATION_FAIL,
    OTA_STATUS_ERROR_ABORTED
} ota_status_t;

/**
 * @brief Initialize the OTA subsystem and inspect boot partition state.
 *        Validates if the currently running app booted properly from a fresh update.
 */
void ota_init(void);

/**
 * @brief Prepare the inactive OTA partition for receiving a new firmware update.
 *        Erases starting sectors.
 *
 * @param image_size Total size of the incoming binary in bytes
 * @return esp_err_t ESP_OK on success, or error code
 */
esp_err_t ota_start(uint32_t image_size);

/**
 * @brief Write an incoming chunk of binary firmware data to the inactive OTA partition.
 *
 * @param offset Byte offset from start of image
 * @param data Pointer to binary chunk
 * @param length Length of chunk in bytes
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ota_write_chunk(uint32_t offset, const uint8_t *data, uint16_t length);

/**
 * @brief Finalize and validate the written firmware image, mark the new partition
 *        as bootable, and prepare for reboot.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ota_finish(void);

/**
 * @brief Abort an active OTA update session and clean up state.
 */
void ota_abort(void);

/**
 * @brief Get the current progress percentage (0 - 100).
 */
uint8_t ota_get_progress_percent(void);

/**
 * @brief Get current OTA state machine status.
 */
ota_status_t ota_get_status(void);

/**
 * @brief Confirm successful startup after reboot. Cancels automatic rollback.
 *        Call this after hardware self-tests (Display, Battery, BLE) pass.
 */
void ota_confirm_boot_valid(void);

#ifdef __cplusplus
}
#endif

#endif // OTA_H
