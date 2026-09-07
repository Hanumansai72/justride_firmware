#ifndef BLE_H
#define BLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize NVS and the NimBLE stack for the JustRide motorcycle device.
 *        Starts the NimBLE FreeRTOS host task and registers the sync callback.
 */
void ble_init(void);

/**
 * @brief Start fast BLE advertising with the device name "JUSTRIDE".
 *        Enables general discoverable and connectable undirected advertising.
 */
void ble_start_advertising(void);

/**
 * @brief Check if a phone is currently connected over BLE.
 *
 * @return true if connected, false otherwise
 */
bool ble_is_connected(void);

/**
 * @brief Cleanly terminate the BLE connection (notifying the mobile app of disconnection)
 *        and restart the ESP32 microcontroller.
 */
void ble_disconnect_and_restart(void);

/**
 * @brief Navigation Status Codes for BLE 0xFF32 feedback
 */
#define NAV_STATUS_DEVICE_READY   0x01
#define NAV_STATUS_RESUME_ROUTE   0x05
#define NAV_STATUS_CANCEL_ROUTE   0x06
#define NAV_STATUS_DIAGNOSTICS    0x10

/**
 * @brief Notify phone of device status or button trigger over BLE 0xFF32
 */
void ble_nav_notify_status(uint8_t status_code);

/**
 * @brief Notify phone of diagnostic payload over BLE 0xFF32
 */
void ble_nav_notify_diagnostic_packet(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_H
