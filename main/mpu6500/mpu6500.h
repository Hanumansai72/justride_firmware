/**
 * @file mpu6500.h
 * @brief MPU6500 IMU driver - I2C interface for JustRide motorcycle HUD.
 *
 * Reads raw gyroscope Z-axis data to derive relative yaw heading.
 * The computed heading is fed into the display layer to visually rotate
 * the navigation turn arrow when the rider physically turns left/right.
 *
 * Wiring (I2C mode, NCS pulled HIGH to 3.3V):
 *   MPU6500 VCC   -> ESP32 3.3V
 *   MPU6500 GND   -> ESP32 GND
 *   MPU6500 SCL   -> ESP32 GPIO 22
 *   MPU6500 SDA   -> ESP32 GPIO 21
 *   MPU6500 NCS   -> ESP32 3.3V   (MUST be HIGH to select I2C mode)
 *   MPU6500 AD0   -> GND          (I2C address = 0x68)
 *   MPU6500 INT   -> GPIO 34      (optional interrupt, not used here)
 *   MPU6500 FSYNC -> GND
 *   MPU6500 EDA   -> GND          (no auxiliary sensor)
 *   MPU6500 ECL   -> GND          (no auxiliary sensor)
 */

#ifndef MPU6500_H
#define MPU6500_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// I2C BUS CONFIGURATION
// =============================================================================
#define MPU6500_I2C_PORT        I2C_NUM_0   ///< ESP32 I2C peripheral number
#define MPU6500_I2C_SDA_PIN     21          ///< GPIO 21 -> MPU6500 SDA
#define MPU6500_I2C_SCL_PIN     22          ///< GPIO 22 -> MPU6500 SCL
#define MPU6500_I2C_FREQ_HZ     100000      ///< 100 kHz Standard Mode (safe for bare chip)
#define MPU6500_I2C_ADDR        0x68        ///< AD0 = GND -> 0x68

// =============================================================================
// MPU6500 REGISTER MAP (relevant subset)
// =============================================================================
#define MPU6500_REG_SMPLRT_DIV      0x19
#define MPU6500_REG_CONFIG          0x1A
#define MPU6500_REG_GYRO_CONFIG     0x1B
#define MPU6500_REG_ACCEL_CONFIG    0x1C
#define MPU6500_REG_ACCEL_CONFIG_2  0x1D
#define MPU6500_REG_ACCEL_XOUT_H    0x3B
#define MPU6500_REG_GYRO_XOUT_H     0x43
#define MPU6500_REG_PWR_MGMT_1      0x6B
#define MPU6500_REG_PWR_MGMT_2      0x6C
#define MPU6500_REG_WHO_AM_I        0x75

#define MPU6500_WHO_AM_I_VAL        0x70    ///< Expected WHO_AM_I response for MPU6500

/**
 * 4-Direction Physical Motion / Tilt States
 */
typedef enum {
    IMU_MOTION_LEVEL = 0,   ///< Level / Stationary (within dead-zone)
    IMU_MOTION_FORWARD,     ///< Tilted Forward (Pitch down/forward) -> Straight Up Arrow
    IMU_MOTION_BACKWARD,    ///< Tilted Backward (Pitch up/backward) -> U-Turn Down Arrow
    IMU_MOTION_LEFT,        ///< Tilted Left (Roll left) -> Left Turn Arrow
    IMU_MOTION_RIGHT,       ///< Tilted Right (Roll right) -> Right Turn Arrow
} imu_motion_t;

/**
 * Inversion controls:
 * If tilting left turns right, set MPU6500_INVERT_ROLL to 1.
 * If tilting forward triggers backward, set MPU6500_INVERT_PITCH to 1.
 */
#define MPU6500_INVERT_ROLL         0
#define MPU6500_INVERT_PITCH        0

// =============================================================================
// PUBLIC API
// =============================================================================

/**
 * @brief Initialize MPU6500 over I2C.
 *        Configures accelerometer and gyroscope, wakes from sleep.
 *        Automatically detects whether gyroscope or accelerometer mode is used.
 * @return ESP_OK on success.
 */
esp_err_t mpu6500_init(void);

/**
 * @brief Launch the FreeRTOS task at 50 Hz that samples the sensor,
 *        computes Forward/Backward/Left/Right motion, and updates the HUD display.
 *        Call ONCE after mpu6500_init() succeeds.
 */
void mpu6500_start_task(void);

/**
 * @brief Read raw 16-bit accelerometer values (blocking, single-shot).
 * @param[out] ax  Raw X axis acceleration
 * @param[out] ay  Raw Y axis acceleration
 * @param[out] az  Raw Z axis acceleration
 * @return ESP_OK on success.
 */
esp_err_t mpu6500_read_accel(int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief Read raw 16-bit gyro Z value (blocking, single-shot).
 * @param[out] gz_raw  Raw gyroscope Z value
 * @return ESP_OK on success.
 */
esp_err_t mpu6500_read_gyro_z(int16_t *gz_raw);

/**
 * @brief Returns true if gyro hardware is functional, false if operating
 *        in accelerometer tilt/lean fallback mode.
 */
bool mpu6500_is_gyro_available(void);

/** Returns true after the stationary startup baseline has completed. */
bool mpu6500_is_calibration_complete(void);
bool mpu6500_is_available(void);
const char *mpu6500_get_status(void);
/** Queue calibration on the sensor task, never block the touch/render task. */
bool mpu6500_request_calibration(void);
void mpu6500_begin_ride_calibration(void);
void mpu6500_end_ride_calibration(void);
void mpu6500_publish_status(void);

/**
 * @brief Get the active classified physical motion direction.
 */
imu_motion_t mpu6500_get_motion(void);

/**
 * @brief Get the current filtered roll angle in degrees [-180..+180].
 * @return Roll angle (positive = right, negative = left).
 */
float mpu6500_get_roll_deg(void);

/**
 * @brief Get the current filtered pitch angle in degrees [-90..+90].
 * @return Pitch angle (positive = forward, negative = backward).
 */
float mpu6500_get_pitch_deg(void);

/**
 * @brief Get the current integrated heading in degrees [0..360).
 * @return Heading in degrees.
 */
float mpu6500_get_heading_deg(void);

/**
 * @brief Reset the heading / re-zero the level baseline orientation.
 */
void mpu6500_reset_heading(void);

#ifdef __cplusplus
}
#endif

#endif // MPU6500_H
