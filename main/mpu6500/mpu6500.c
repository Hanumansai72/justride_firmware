/**
 * @file mpu6500.c
 * @brief MPU6500 IMU driver implementation for JustRide motorcycle HUD.
 *
 * Accurate 4-Direction Motion Sensing:
 *  - FORWARD:  Screen pitched forward  -> Displays UP Arrow & "Forward"
 *  - BACKWARD: Screen pitched backward -> Displays U-TURN / Down Arrow & "Backward"
 *  - LEFT:     Screen rolled left      -> Displays LEFT Arrow & "Turn Left"
 *  - RIGHT:    Screen rolled right     -> Displays RIGHT Arrow & "Turn Right"
 *  - LEVEL:    Screen within dead-zone -> Follows regular GPS/BLE navigation
 *
 * Dual-Mode Operation:
 *  - Automatically falls back to Accelerometer 4-direction sensing if Gyro
 *    MEMS core is non-functional (e.g. bare chip missing REGOUT capacitor).
 *  - If a working Gyro chip is installed, supports Gyro yaw integration.
 *  - Filtered at 50 Hz with hardware DLPF (41 Hz) + software exponential moving
 *    average to eliminate motorcycle engine and road vibration spikes.
 */

#include "mpu6500.h"
#include "ble.h"
#include "display.h"   // display_set_imu_override_direction(), display_set_imu_heading()

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "MPU6500";

// =============================================================================
// SCALE FACTORS & CONSTANTS
// =============================================================================
#define GYRO_SENSITIVITY_DPS    65.5f       // LSB per deg/s at +/-500 deg/s range
#define ACCEL_SENSITIVITY_LSB_G 16384.0f    // LSB per 1g at +/-2g range

#define IMU_TASK_PERIOD_MS      20          // 50 Hz sampling (20ms period)
#define ACCEL_FILTER_TAU_SEC    0.020f      // Short noise filter for live local tilt
#define MOTION_ENTER_DEG        10.0f
#define MOTION_EXIT_DEG         8.0f
#define MOTION_DOMINANCE_MARGIN_DEG 2.0f

// Temporary stationary noise characterization. Enable only for a hardware
// measurement run; production firmware keeps this fully compiled out.
#ifndef MPU6500_DIAGNOSTIC_MODE
#define MPU6500_DIAGNOSTIC_MODE 0
#endif

// Heading & Motion states
static volatile float        s_heading_deg     = 0.0f;
static volatile float        s_filtered_roll   = 0.0f;
static volatile float        s_filtered_pitch  = 0.0f;
static volatile imu_motion_t s_current_motion  = IMU_MOTION_LEVEL;

// Gyro bias collected during warm-up
static float s_gyro_z_bias = 0.0f;

// Hardware status & Accelerometer baseline
static bool  s_gyro_available    = false;
static float s_base_ax           = 0.0f;
static float s_base_ay           = 0.0f;
static float s_base_az           = 0.0f;
static bool  s_is_vertical_mount = true;
static uint8_t s_mpu_addr        = MPU6500_I2C_ADDR;
static volatile bool s_calibration_complete = false;
static volatile bool s_sensor_available, s_calibration_requested, s_calibrating, s_read_error;
static TaskHandle_t s_imu_task;
static volatile bool s_auto_calibration;
static int64_t s_next_auto_calibration_us;

void mpu6500_begin_ride_calibration(void) {
    if (!s_sensor_available) return;
    s_auto_calibration = true;
    s_calibration_requested = true;
}
void mpu6500_end_ride_calibration(void) {
    s_auto_calibration = false;
    s_calibration_requested = false;
    s_calibrating = false;
}
void mpu6500_publish_status(void) {
    if (!ble_is_connected()) return;
    uint8_t state = !s_sensor_available ? 0 : (s_calibrating || s_calibration_requested) ? 2 :
        s_read_error ? 4 : s_auto_calibration ? 5 : s_calibration_complete ? 3 : 1;
    int16_t roll = (int16_t)lroundf(s_filtered_roll * 10), pitch = (int16_t)lroundf(s_filtered_pitch * 10);
    uint8_t packet[8] = {0x1A,1,state,(uint8_t)s_current_motion,
        (uint8_t)roll,(uint8_t)((uint16_t)roll >> 8),(uint8_t)pitch,(uint8_t)((uint16_t)pitch >> 8)};
    ble_nav_notify_diagnostic_packet(packet,sizeof(packet));
}
static int64_t s_last_sample_us;

bool mpu6500_is_available(void) { return s_sensor_available; }
const char *mpu6500_get_status(void) {
    if (!s_sensor_available) return "SENSOR NOT DETECTED";
    if (s_calibrating || s_calibration_requested) return "KEEP STILL...";
    if (s_read_error) return "SENSOR READ ERROR";
    if (s_auto_calibration) return "WAITING FOR STILLNESS";
    if (!s_calibration_complete) return s_auto_calibration ? "WAITING FOR STILLNESS" : "CALIBRATION NEEDED";
    return "READY";
}
bool mpu6500_request_calibration(void) {
    if (!s_sensor_available || !s_imu_task || s_calibrating || s_calibration_requested) return false;
    s_calibration_requested = true;
    return true;
}

#if MPU6500_DIAGNOSTIC_MODE
typedef struct {
    uint32_t count;
    double sum[3];
    double sum_sq[3];
    int16_t min[3];
    int16_t max[3];
    int64_t last_report_us;
} accel_noise_stats_t;

static accel_noise_stats_t s_noise_stats = {
    .min = {INT16_MAX, INT16_MAX, INT16_MAX},
    .max = {INT16_MIN, INT16_MIN, INT16_MIN},
};

static void diagnostic_record_accel(int16_t ax, int16_t ay, int16_t az) {
    const int16_t sample[3] = {ax, ay, az};
    for (int i = 0; i < 3; i++) {
        if (sample[i] < s_noise_stats.min[i]) s_noise_stats.min[i] = sample[i];
        if (sample[i] > s_noise_stats.max[i]) s_noise_stats.max[i] = sample[i];
        s_noise_stats.sum[i] += (double)sample[i];
        s_noise_stats.sum_sq[i] += (double)sample[i] * (double)sample[i];
    }
    s_noise_stats.count++;

    const int64_t now_us = esp_timer_get_time();
    if (s_noise_stats.count >= 250 &&
        (s_noise_stats.last_report_us == 0 ||
         now_us - s_noise_stats.last_report_us >= 5000000)) {
        float mean[3];
        float stddev[3];
        for (int i = 0; i < 3; i++) {
            mean[i] = (float)(s_noise_stats.sum[i] / s_noise_stats.count);
            double variance = (s_noise_stats.sum_sq[i] / s_noise_stats.count) -
                              ((double)mean[i] * mean[i]);
            stddev[i] = (float)sqrt(fmax(variance, 0.0));
        }
        const float magnitude_g = sqrtf(mean[0] * mean[0] +
                                        mean[1] * mean[1] +
                                        mean[2] * mean[2]) /
                                  ACCEL_SENSITIVITY_LSB_G;
        ESP_LOGI(TAG,
                 "NOISE n=%" PRIu32 " mean=(%.1f,%.1f,%.1f) std=(%.2f,%.2f,%.2f) "
                 "pkpk=(%d,%d,%d) |g|=%.4f",
                 s_noise_stats.count, mean[0], mean[1], mean[2],
                 stddev[0], stddev[1], stddev[2],
                 (int)(s_noise_stats.max[0] - s_noise_stats.min[0]),
                 (int)(s_noise_stats.max[1] - s_noise_stats.min[1]),
                 (int)(s_noise_stats.max[2] - s_noise_stats.min[2]),
                 magnitude_g);
        memset(&s_noise_stats, 0, sizeof(s_noise_stats));
        s_noise_stats.min[0] = s_noise_stats.min[1] = s_noise_stats.min[2] = INT16_MAX;
        s_noise_stats.max[0] = s_noise_stats.max[1] = s_noise_stats.max[2] = INT16_MIN;
        s_noise_stats.last_report_us = now_us;
    }
}
#endif

static bool accel_sample_valid(int16_t ax, int16_t ay, int16_t az) {
    if (ax == INT16_MIN || ay == INT16_MIN || az == INT16_MIN) return false;
    const float magnitude = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az);
    // Reject disconnected/zero samples and physically impossible values while
    // allowing short motorcycle vibration spikes within the configured ±2g range.
    return magnitude >= (0.20f * ACCEL_SENSITIVITY_LSB_G) &&
           magnitude <= (3.0f * ACCEL_SENSITIVITY_LSB_G);
}

// =============================================================================
// LOW-LEVEL I2C HELPERS
// =============================================================================

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_mpu_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MPU6500_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_mpu_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_mpu_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MPU6500_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// =============================================================================
// PUBLIC: READ SENSORS
// =============================================================================

esp_err_t mpu6500_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    esp_err_t ret = i2c_read_reg(MPU6500_REG_ACCEL_XOUT_H, buf, 6);
    if (ret != ESP_OK) return ret;
    if (ax) *ax = (int16_t)((buf[0] << 8) | buf[1]);
    if (ay) *ay = (int16_t)((buf[2] << 8) | buf[3]);
    if (az) *az = (int16_t)((buf[4] << 8) | buf[5]);
    return ESP_OK;
}

esp_err_t mpu6500_read_gyro_z(int16_t *gz_raw) {
    uint8_t buf[2];
    esp_err_t ret = i2c_read_reg(MPU6500_REG_GYRO_XOUT_H + 4, buf, 2);
    if (ret != ESP_OK) return ret;
    if (gz_raw) *gz_raw = (int16_t)((buf[0] << 8) | buf[1]);
    return ESP_OK;
}

// =============================================================================
// PUBLIC: GETTERS & RESET
// =============================================================================

bool mpu6500_is_gyro_available(void) {
    return s_gyro_available;
}

bool mpu6500_is_calibration_complete(void) {
    return s_calibration_complete;
}

imu_motion_t mpu6500_get_motion(void) {
    return s_current_motion;
}

float mpu6500_get_roll_deg(void) {
    return s_filtered_roll;
}

float mpu6500_get_pitch_deg(void) {
    return s_filtered_pitch;
}

float mpu6500_get_heading_deg(void) {
    return s_heading_deg;
}

void mpu6500_reset_heading(void) {
    mpu6500_request_calibration();
}

// =============================================================================
// PUBLIC: INIT
// =============================================================================

esp_err_t mpu6500_init(void) {
    s_sensor_available = false;
    s_calibration_complete = false;
    ESP_LOGI(TAG, "Initialising MPU6500 (I2C SDA=GPIO%d SCL=GPIO%d addr=0x%02X)",
             MPU6500_I2C_SDA_PIN, MPU6500_I2C_SCL_PIN, MPU6500_I2C_ADDR);

    // -------------------------------------------------------------------------
    // Hardware Line Level Diagnostic & Bus Clear
    // -------------------------------------------------------------------------
    gpio_config_t test_conf = {
        .pin_bit_mask = (1ULL << MPU6500_I2C_SDA_PIN) | (1ULL << MPU6500_I2C_SCL_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&test_conf);
    vTaskDelay(pdMS_TO_TICKS(10));

    int sda_idle = gpio_get_level(MPU6500_I2C_SDA_PIN);
    int scl_idle = gpio_get_level(MPU6500_I2C_SCL_PIN);
    ESP_LOGI(TAG, "Hardware Line State: SDA(GPIO%d)=%s, SCL(GPIO%d)=%s",
             MPU6500_I2C_SDA_PIN, sda_idle ? "HIGH (OK)" : "LOW (STUCK/SHORTED)",
             MPU6500_I2C_SCL_PIN, scl_idle ? "HIGH (OK)" : "LOW (STUCK/SHORTED)");

    if (sda_idle == 0 || scl_idle == 0) {
        ESP_LOGW(TAG, "Bus line is LOW! Toggling SCL clock 9 times to un-stick slave...");
        gpio_set_direction(MPU6500_I2C_SCL_PIN, GPIO_MODE_OUTPUT);
        for (int i = 0; i < 9; i++) {
            gpio_set_level(MPU6500_I2C_SCL_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(MPU6500_I2C_SCL_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        gpio_set_direction(MPU6500_I2C_SCL_PIN, GPIO_MODE_INPUT);
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "Post-recovery Line State: SDA(GPIO%d)=%s, SCL(GPIO%d)=%s",
                 MPU6500_I2C_SDA_PIN, gpio_get_level(MPU6500_I2C_SDA_PIN) ? "HIGH" : "LOW",
                 MPU6500_I2C_SCL_PIN, gpio_get_level(MPU6500_I2C_SCL_PIN) ? "HIGH" : "LOW");
    }

    // Configure I2C master bus
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = MPU6500_I2C_SDA_PIN,
        .scl_io_num       = MPU6500_I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6500_I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(MPU6500_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(MPU6500_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // -------------------------------------------------------------------------
    // I2C Bus Scanner with SDA/SCL AUTO-DETECTION
    // Tries BOTH GPIO21/22 AND GPIO22/21 to find the MPU6500.
    // This definitively rules out a pin swap vs a power/connection issue.
    // -------------------------------------------------------------------------
    bool    mpu_found       = false;
    uint8_t mpu_actual_addr = MPU6500_I2C_ADDR;
    int     sda_used        = MPU6500_I2C_SDA_PIN;
    int     scl_used        = MPU6500_I2C_SCL_PIN;

    // Two GPIO configurations to try: [SDA, SCL]
    int gpio_combos[2][2] = {
        { MPU6500_I2C_SDA_PIN, MPU6500_I2C_SCL_PIN }, // normal:  SDA=21, SCL=22
        { MPU6500_I2C_SCL_PIN, MPU6500_I2C_SDA_PIN }, // swapped: SDA=22, SCL=21
    };

    for (int combo = 0; combo < 2 && !mpu_found; combo++) {
        int try_sda = gpio_combos[combo][0];
        int try_scl = gpio_combos[combo][1];

        // Re-configure I2C with this pin combination
        i2c_driver_delete(MPU6500_I2C_PORT);
        i2c_config_t try_conf = {
            .mode             = I2C_MODE_MASTER,
            .sda_io_num       = try_sda,
            .scl_io_num       = try_scl,
            .sda_pullup_en    = GPIO_PULLUP_ENABLE,
            .scl_pullup_en    = GPIO_PULLUP_ENABLE,
            .master.clk_speed = MPU6500_I2C_FREQ_HZ,
        };
        i2c_param_config(MPU6500_I2C_PORT, &try_conf);
        i2c_driver_install(MPU6500_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

        ESP_LOGI(TAG, "--- Probing with SDA=GPIO%d SCL=GPIO%d ---", try_sda, try_scl);

        for (int attempt = 0; attempt < 2 && !mpu_found; attempt++) {
            vTaskDelay(pdMS_TO_TICKS(200 + attempt * 300));

            // Direct WHO_AM_I probe at 0x68 and 0x69 (reads register 0x75 with 50ms timeout)
            for (uint8_t addr = 0x68; addr <= 0x69 && !mpu_found; addr++) {
                uint8_t who = 0;
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
                i2c_master_write_byte(cmd, MPU6500_REG_WHO_AM_I, true);
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
                i2c_master_read_byte(cmd, &who, I2C_MASTER_NACK);
                i2c_master_stop(cmd);
                esp_err_t probe_ret = i2c_master_cmd_begin(MPU6500_I2C_PORT, cmd, pdMS_TO_TICKS(50));
                i2c_cmd_link_delete(cmd);

                ESP_LOGI(TAG, "  Probe addr 0x%02X (SDA=%d SCL=%d): ret=%s, WHO_AM_I=0x%02X",
                         addr, try_sda, try_scl, esp_err_to_name(probe_ret), who);

                if (probe_ret == ESP_OK && who != 0x00 && who != 0xFF) {
                    ESP_LOGI(TAG, "  *** FOUND MPU6500 at 0x%02X (SDA=GPIO%d SCL=GPIO%d, WHO_AM_I=0x%02X) ***",
                             addr, try_sda, try_scl, who);
                    mpu_found       = true;
                    mpu_actual_addr = addr;
                    sda_used        = try_sda;
                    scl_used        = try_scl;
                }
            }

            if (!mpu_found) {
                // Full scan so all other devices are also visible in log
                ESP_LOGI(TAG, "  Full scan (SDA=GPIO%d SCL=GPIO%d attempt %d/2)...",
                         try_sda, try_scl, attempt + 1);
                for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
                    i2c_cmd_handle_t probe = i2c_cmd_link_create();
                    i2c_master_start(probe);
                    i2c_master_write_byte(probe, (addr << 1) | I2C_MASTER_WRITE, true);
                    i2c_master_stop(probe);
                    esp_err_t probe_ret = i2c_master_cmd_begin(MPU6500_I2C_PORT, probe,
                                                                pdMS_TO_TICKS(10));
                    i2c_cmd_link_delete(probe);
                    if (probe_ret == ESP_OK) {
                        ESP_LOGI(TAG, "  Found 0x%02X%s", addr,
                                 (addr == 0x68) ? " <- MPU6500 (AD0=GND)" :
                                 (addr == 0x69) ? " <- MPU6500 (AD0=3.3V)" : " (other device)");
                        if (addr == 0x68 || addr == 0x69) {
                            mpu_found = true;
                            mpu_actual_addr = addr;
                            sda_used = try_sda;
                            scl_used = try_scl;
                        }
                    }
                }
            }
        }

        if (!mpu_found) {
            ESP_LOGW(TAG, "  MPU6500 not found with SDA=GPIO%d SCL=GPIO%d", try_sda, try_scl);
        }
    }

    if (!mpu_found) {
        ESP_LOGE(TAG, "============================================");
        ESP_LOGE(TAG, "=== MPU6500 NOT DETECTED (BOTH GPIO COMBOS TRIED) ===");
        ESP_LOGE(TAG, "============================================");
        ESP_LOGE(TAG, "Tried: SDA=GPIO21/SCL=GPIO22  AND  SDA=GPIO22/SCL=GPIO21");
        ESP_LOGE(TAG, "Neither combination found the device.");
        ESP_LOGE(TAG, "This means it is a POWER or HARDWARE issue, NOT a pin swap:");
        ESP_LOGE(TAG, "  >> Check VCC -> 3.3V with a multimeter");
        ESP_LOGE(TAG, "  >> Check NCS (CS) is wired to 3.3V (NOT GND)");
        ESP_LOGE(TAG, "  >> Check GND is connected");
        ESP_LOGE(TAG, "  >> Check AD0 -> GND (not floating)");
        ESP_LOGE(TAG, "  >> The chip may be damaged or dead");
        i2c_driver_delete(MPU6500_I2C_PORT);
        return ESP_FAIL;
    }

    if (sda_used != MPU6500_I2C_SDA_PIN) {
        ESP_LOGW(TAG, "NOTE: MPU6500 found with SWAPPED pins! SDA=GPIO%d SCL=GPIO%d", sda_used, scl_used);
        ESP_LOGW(TAG, "Either swap your physical wires, or change mpu6500.h:");
        ESP_LOGW(TAG, "  #define MPU6500_I2C_SDA_PIN  %d", sda_used);
        ESP_LOGW(TAG, "  #define MPU6500_I2C_SCL_PIN  %d", scl_used);
    }


    s_mpu_addr = mpu_actual_addr;

    // Read WHO_AM_I from the confirmed MPU address
    // Accept: 0x70 = MPU6500, 0x71 = MPU6500 variant, 0x68 = MPU6050 clone
    uint8_t who_am_i = 0;
    ret = i2c_read_reg(MPU6500_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s (addr=0x%02X)", esp_err_to_name(ret), mpu_actual_addr);
        i2c_driver_delete(MPU6500_I2C_PORT);
        return ret;
    }
    bool who_ok = (who_am_i == 0x70 || who_am_i == 0x71 || who_am_i == 0x68 || who_am_i == 0x98);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (%s)", who_am_i,
             who_am_i == 0x70 ? "MPU6500" :
             who_am_i == 0x71 ? "MPU6500 variant" :
             who_am_i == 0x68 ? "MPU6050 clone" : "Unknown — continuing anyway");
    if (!who_ok) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I=0x%02X — chip may be clone or damaged, continuing anyway", who_am_i);
    }

    // Step 1: Hardware reset
    ESP_LOGI(TAG, "Step 1: Hardware reset...");
    ret = i2c_write_reg(MPU6500_REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) return ret;

    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint8_t pwr1 = 0xFF;
        if (i2c_read_reg(MPU6500_REG_PWR_MGMT_1, &pwr1, 1) == ESP_OK) {
            if (!(pwr1 & 0x80)) break;
        }
    }

    // Step 2: Wake from sleep
    ESP_LOGI(TAG, "Step 2: Waking from sleep...");
    ret = i2c_write_reg(MPU6500_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = i2c_write_reg(MPU6500_REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(30));

    // Step 3: Enable ALL axes
    ESP_LOGI(TAG, "Step 3: Enabling all axes...");
    ret = i2c_write_reg(MPU6500_REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 3.5: Signal path reset & test gyro response
    ret = i2c_write_reg(0x68, 0x07);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = i2c_write_reg(0x6A, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = i2c_write_reg(MPU6500_REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_reg(MPU6500_REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t gyro_test[6];
    memset(gyro_test, 0, 6);
    ret = i2c_read_reg(0x43, gyro_test, 6);
    int16_t gx_test = (int16_t)((gyro_test[0] << 8) | gyro_test[1]);
    int16_t gy_test = (int16_t)((gyro_test[2] << 8) | gyro_test[3]);
    int16_t gz_test = (int16_t)((gyro_test[4] << 8) | gyro_test[5]);

    if (ret == ESP_OK && (gx_test != 0 || gy_test != 0 || gz_test != 0)) {
        s_gyro_available = true;
        ESP_LOGI(TAG, "Gyroscope is LIVE (X=%d Y=%d Z=%d)", gx_test, gy_test, gz_test);
    } else {
        s_gyro_available = false;
        ESP_LOGW(TAG, "Gyroscope core inactive (MEMS output 0).");
        ESP_LOGI(TAG, "============================================================");
        ESP_LOGI(TAG, ">>> ACCELEROMETER 4-DIRECTION MOTION SENSING ACTIVE <<<");
        ESP_LOGI(TAG, "HUD: Forward / Backward / Left / Right fully supported!");
        ESP_LOGI(TAG, "============================================================");
    }

    // Step 4: Configure Accelerometer & Sample Rate
    ESP_LOGI(TAG, "Step 4: Configuring sensors...");
    ret = i2c_write_reg(MPU6500_REG_SMPLRT_DIV, 0x09);
    if (ret != ESP_OK) return ret;      // 100 Hz internal sample rate
    ret = i2c_write_reg(MPU6500_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;    // +/- 2g range (16384 LSB/g)
    ret = i2c_write_reg(MPU6500_REG_ACCEL_CONFIG_2, 0x03);
    if (ret != ESP_OK) return ret;  // 41 Hz DLPF (filters road vibration)

    if (s_gyro_available) {
        ret = i2c_write_reg(MPU6500_REG_CONFIG, 0x03);
        if (ret != ESP_OK) return ret;       // DLPF 42 Hz
        ret = i2c_write_reg(MPU6500_REG_GYRO_CONFIG, 0x08);
        if (ret != ESP_OK) return ret;  // +/-500 deg/s
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 5: Verify live accelerometer data
    int16_t ax = 0, ay = 0, az = 0;
    ret = mpu6500_read_accel(&ax, &ay, &az);
    ESP_LOGI(TAG, "Step 5: Live sensor test -> Accel: X=%6d  Y=%6d  Z=%6d (ret=%s)",
             ax, ay, az, esp_err_to_name(ret));

    if (ret != ESP_OK || !accel_sample_valid(ax, ay, az)) return ret != ESP_OK ? ret : ESP_ERR_INVALID_RESPONSE;
    s_sensor_available = true;
    s_heading_deg    = 0.0f;
    s_filtered_roll  = 0.0f;
    s_filtered_pitch = 0.0f;
    s_current_motion = IMU_MOTION_LEVEL;

    ESP_LOGI(TAG, "MPU6500 init complete.");
    return ESP_OK;
}

// =============================================================================
// MOTION STRING HELPERS (for serial logs)
// =============================================================================

static const char *motion_to_str(imu_motion_t m) {
    switch (m) {
        case IMU_MOTION_FORWARD:  return "FORWARD";
        case IMU_MOTION_BACKWARD: return "BACKWARD";
        case IMU_MOTION_LEFT:     return "LEFT";
        case IMU_MOTION_RIGHT:    return "RIGHT";
        case IMU_MOTION_LEVEL:
        default:                  return "LEVEL";
    }
}

static const char *dir_to_str(nav_direction_t d) {
    switch (d) {
        case NAV_DIR_STRAIGHT:     return "STRAIGHT (Up Arrow)";
        case NAV_DIR_UTURN:        return "UTURN (Down Arrow)";
        case NAV_DIR_LEFT:
        case NAV_DIR_SLIGHT_LEFT:  return "LEFT (Left Arrow)";
        case NAV_DIR_RIGHT:
        case NAV_DIR_SLIGHT_RIGHT: return "RIGHT (Right Arrow)";
        default:                   return "NONE (Route Active)";
    }
}

// =============================================================================
// IMU TASK — 50 Hz sensing -> display_set_imu_override_direction()
// =============================================================================

// Collect one calibration sample per task tick. Live sensing continues using
// the last accepted baseline while a new stationary window is collected.
static void calibration_step(bool begin) {
    static float sums[4];
    static int16_t lo[4], hi[4];
    static int count, gyro_count, attempts;
    if (begin) {
        s_calibrating = true;
        count=0; gyro_count=0; attempts=0;
        memset(sums,0,sizeof(sums));
        mpu6500_publish_status();
    }
    // Keep the last accepted baseline usable if this attempt is interrupted by
    // movement. Only replace it after a complete stationary sample window.
    int16_t sample[4] = {0};
    bool ok = mpu6500_read_accel(&sample[0], &sample[1], &sample[2]) == ESP_OK &&
              accel_sample_valid(sample[0], sample[1], sample[2]);
    bool gyro_ok = s_gyro_available && mpu6500_read_gyro_z(&sample[3]) == ESP_OK && abs(sample[3]) < 655;
    if (!ok) { count = 0; gyro_count=0; memset(sums,0,sizeof(sums)); }
    else {
        if (!count) for (int i=0;i<4;i++) { lo[i]=sample[i]; hi[i]=sample[i]; }
        bool steady = true;
        for (int i=0;i<4;i++) {
            if (sample[i] < lo[i]) lo[i]=sample[i];
            if (sample[i] > hi[i]) hi[i]=sample[i];
            if (i < 3 && (int)hi[i] - lo[i] > 1800) steady=false;
        }
        if (!steady) { count=0; gyro_count=0; memset(sums,0,sizeof(sums)); }
        else {
            for (int i=0;i<3;i++) sums[i]+=sample[i];
            if (gyro_ok) { sums[3]+=sample[3]; gyro_count++; }
            count++;
        }
    }
    if (++attempts < 120 && count < 40) return;
    if (count == 40) {
        s_base_ax=sums[0]/40; s_base_ay=sums[1]/40; s_base_az=sums[2]/40;
        if (s_gyro_available && (gyro_count != 40 || (int)hi[3]-lo[3] > 160)) {
            s_gyro_available=false;
            ESP_LOGW(TAG,"Gyro calibration unavailable; accelerometer remains active");
        }
        s_gyro_z_bias=s_gyro_available ? sums[3]/40 : 0;
        s_auto_calibration=false;
        s_is_vertical_mount=hypotf(s_base_ax,s_base_ay)>4000;
        s_filtered_roll=0; s_filtered_pitch=0; s_heading_deg=0;
        s_last_sample_us=esp_timer_get_time(); s_read_error=false;
        s_calibration_complete=true;
        display_set_imu_override_direction(NAV_DIR_NONE);
        ESP_LOGI(TAG,"Calibration accepted: ax=%.0f ay=%.0f az=%.0f",s_base_ax,s_base_ay,s_base_az);
    } else ESP_LOGW(TAG,"Calibration rejected: keep still and retry; valid stationary samples=%d",count);
    s_calibrating=false;
    s_next_auto_calibration_us=esp_timer_get_time()+5000000;
    mpu6500_publish_status();

}

static void imu_task(void *pvParam) {
    s_auto_calibration = true;
    s_calibration_requested = true;
    uint32_t log_counter = 0;
    int64_t previous_sample_us = esp_timer_get_time();

    int64_t last_publish_us=0;
    TickType_t sample_wake = xTaskGetTickCount();
    while (1) {
        if (esp_timer_get_time()-last_publish_us >= 200000) {
            mpu6500_publish_status();
            last_publish_us=esp_timer_get_time();
        }
        if (s_auto_calibration && !s_calibrating && esp_timer_get_time()>=s_next_auto_calibration_us)
            s_calibration_requested=true;
        if (s_calibration_requested) {
            s_calibration_requested = false;
            calibration_step(true);
        } else if (s_calibrating) {
            calibration_step(false);
        }
        if (!s_calibration_complete) { vTaskDelayUntil(&sample_wake,pdMS_TO_TICKS(IMU_TASK_PERIOD_MS)); continue; }
        const int64_t sample_now_us = esp_timer_get_time();
        float measured_dt = (float)(sample_now_us - previous_sample_us) / 1000000.0f;
        previous_sample_us = sample_now_us;
        if (!(measured_dt > 0.0f) || measured_dt > 0.25f) {
            measured_dt = IMU_TASK_PERIOD_MS / 1000.0f;
        }

        if (s_gyro_available) {
            // Gyroscope mode: read gz and integrate yaw
            int16_t gz_raw = 0;
            esp_err_t ret = mpu6500_read_gyro_z(&gz_raw);
            if (ret == ESP_OK) {
                float gz_dps = ((float)gz_raw - s_gyro_z_bias) / GYRO_SENSITIVITY_DPS;
                if (gz_dps > 0.5f || gz_dps < -0.5f) {
                    s_heading_deg += gz_dps * measured_dt;
                    if (s_heading_deg >= 360.0f) s_heading_deg -= 360.0f;
                    if (s_heading_deg < 0.0f)    s_heading_deg += 360.0f;
                }
                display_set_imu_heading(s_heading_deg);

                log_counter++;
                if (log_counter >= 50) {
                    log_counter = 0;
                    ESP_LOGI(TAG, "Gyro Z raw=%6d  dps=%7.2f  heading=%.1f deg",
                             gz_raw, gz_dps, s_heading_deg);
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        {
            // Always sample tilt, including on boards with a functional gyroscope.
            int16_t ax = 0, ay = 0, az = 0;
            esp_err_t ret = mpu6500_read_accel(&ax, &ay, &az);
            if (ret == ESP_OK && accel_sample_valid(ax, ay, az)) {
                s_last_sample_us = esp_timer_get_time();
                s_read_error = false;
#if MPU6500_DIAGNOSTIC_MODE
                diagnostic_record_accel(ax, ay, az);
#endif
                float roll_raw = 0.0f;
                float pitch_raw = 0.0f;

                if (s_is_vertical_mount) {
                    // Vertical / angled screen mount (Z axis is normal to screen)
                    // Roll: in-plane rotation
                    float cross_z = s_base_ax * (float)ay - s_base_ay * (float)ax;
                    float dot_z   = s_base_ax * (float)ax + s_base_ay * (float)ay;
                    roll_raw = atan2f(cross_z, dot_z) * (180.0f / (float)M_PI);

                    // Pitch: tilt forward/backward projects gravity onto normal Z axis
                    float delta_z = ((float)az - s_base_az) / ACCEL_SENSITIVITY_LSB_G;
                    if (delta_z > 1.0f) delta_z = 1.0f;
                    if (delta_z < -1.0f) delta_z = -1.0f;
                    pitch_raw = asinf(delta_z) * (180.0f / (float)M_PI);
                } else {
                    // Horizontal / flat mount
                    float delta_x = ((float)ax - s_base_ax) / ACCEL_SENSITIVITY_LSB_G;
                    if (delta_x > 1.0f) delta_x = 1.0f;
                    if (delta_x < -1.0f) delta_x = -1.0f;
                    roll_raw = asinf(delta_x) * (180.0f / (float)M_PI);

                    float delta_y = ((float)ay - s_base_ay) / ACCEL_SENSITIVITY_LSB_G;
                    if (delta_y > 1.0f) delta_y = 1.0f;
                    if (delta_y < -1.0f) delta_y = -1.0f;
                    pitch_raw = asinf(delta_y) * (180.0f / (float)M_PI);
                }

#if MPU6500_INVERT_ROLL
                roll_raw = -roll_raw;
#endif
#if MPU6500_INVERT_PITCH
                pitch_raw = -pitch_raw;
#endif

                // Time-normalized exponential low-pass filter. At the normal
                // 20ms cadence alpha is 0.5 for a quick local response,
                // but timing jitter no longer changes the effective cutoff.
                float filter_alpha = measured_dt /
                                     (ACCEL_FILTER_TAU_SEC + measured_dt);
                if (filter_alpha < 0.05f) filter_alpha = 0.05f;
                if (filter_alpha > 0.60f) filter_alpha = 0.60f;
                s_filtered_roll += filter_alpha * (roll_raw - s_filtered_roll);
                s_filtered_pitch += filter_alpha * (pitch_raw - s_filtered_pitch);

                // 4-Direction Classification with Hysteresis:
                // Threshold = 10.0°, Hysteresis = 2.0°
                float abs_r = fabsf(s_filtered_roll);
                float abs_p = fabsf(s_filtered_pitch);
                float active_thresh = (s_current_motion == IMU_MOTION_LEVEL)
                    ? MOTION_ENTER_DEG : MOTION_EXIT_DEG;

                if (abs_r < active_thresh && abs_p < active_thresh) {
                    s_current_motion = IMU_MOTION_LEVEL;
                } else if (fabsf(abs_r - abs_p) < MOTION_DOMINANCE_MARGIN_DEG &&
                           s_current_motion != IMU_MOTION_LEVEL) {
                    // Keep the previous axis while the two tilt components are
                    // nearly tied; this prevents left/right or forward/back
                    // chatter at the diagonal boundary.
                } else if (abs_r >= abs_p) {
                    // Roll (Left/Right) dominant
                    s_current_motion = (s_filtered_roll > 0.0f) ? IMU_MOTION_RIGHT : IMU_MOTION_LEFT;
                } else {
                    // Pitch (Forward/Backward) dominant
                    s_current_motion = (s_filtered_pitch > 0.0f) ? IMU_MOTION_FORWARD : IMU_MOTION_BACKWARD;
                }

                // Map classified motion directly to HUD navigation direction
                nav_direction_t target_dir = NAV_DIR_NONE;
                switch (s_current_motion) {
                    case IMU_MOTION_FORWARD:
                        target_dir = NAV_DIR_STRAIGHT; // Up arrow
                        break;
                    case IMU_MOTION_BACKWARD:
                        target_dir = NAV_DIR_UTURN;    // Down / U-turn arrow
                        break;
                    case IMU_MOTION_LEFT:
                        target_dir = (abs_r > 35.0f) ? NAV_DIR_LEFT : NAV_DIR_SLIGHT_LEFT;
                        break;
                    case IMU_MOTION_RIGHT:
                        target_dir = (abs_r > 35.0f) ? NAV_DIR_RIGHT : NAV_DIR_SLIGHT_RIGHT;
                        break;
                    case IMU_MOTION_LEVEL:
                    default:
                        target_dir = NAV_DIR_NONE;    // Route active
                        break;
                }

                display_set_imu_override_direction(target_dir);

                // Update heading field for compass badge
                float heading = s_filtered_roll;
                if (heading < 0.0f)   heading += 360.0f;
                if (heading >= 360.0f) heading -= 360.0f;
                if (!s_gyro_available) s_heading_deg = heading;

                log_counter++;
                if (log_counter >= 50) {
                    log_counter = 0;
                    ESP_LOGI(TAG, "Roll=%+5.1f° Pitch=%+5.1f° | Motion: %-8s -> HUD: %s",
                             s_filtered_roll, s_filtered_pitch,
                             motion_to_str(s_current_motion),
                             dir_to_str(target_dir));
                }
            } else {
                if (esp_timer_get_time() - s_last_sample_us > 500000) {
                    s_read_error = true;
                    s_current_motion = IMU_MOTION_LEVEL;
                    display_set_imu_override_direction(NAV_DIR_NONE);
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        vTaskDelayUntil(&sample_wake,pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

void mpu6500_start_task(void) {
    if (!s_sensor_available || s_imu_task) return;
    BaseType_t result = xTaskCreate(imu_task, "imu_task", 4096, NULL, 4, &s_imu_task);
    if (result != pdPASS) { s_sensor_available=false; ESP_LOGE(TAG,"IMU task creation failed"); }
    else ESP_LOGI(TAG, "IMU task started");
}
