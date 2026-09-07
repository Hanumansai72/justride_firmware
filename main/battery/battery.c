#include "battery.h"

#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Modern ESP-IDF v5.x ADC Oneshot Driver and Calibration APIs
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

// JustRide modules
#include "display.h"

static const char *TAG = "BATTERY";

// =============================================================================
// HARDWARE ADC CONFIGURATION
// Configure BATTERY_ADC_CHANNEL to match your voltage divider pin.
// On ESP32 / ESP32-S3 ADC1:
//   ADC_CHANNEL_3 = GPIO 34 on ESP32 (or GPIO 4 on ESP32-S3)
// =============================================================================
#define BATTERY_ADC_UNIT        ADC_UNIT_1
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_3  

// 12dB attenuation allows measuring up to ~3.1V safely
#define BATTERY_ADC_ATTEN       ADC_ATTEN_DB_12

// Voltage Divider: R1 = 100k, R2 = 100k -> Ratio = (100+100)/100 = 2.0
#define BATTERY_DIVIDER_R1_KOHMS    100.0f
#define BATTERY_DIVIDER_R2_KOHMS    100.0f
#define BATTERY_VOLTAGE_DIVIDER_RATIO ((BATTERY_DIVIDER_R1_KOHMS + BATTERY_DIVIDER_R2_KOHMS) / BATTERY_DIVIDER_R2_KOHMS)

#define BATTERY_LOW_PERCENTAGE_THRESHOLD 20
#define ADC_OVERSAMPLE_COUNT    16

// =============================================================================
// TP4056 DIGITAL INDICATOR PINS (No resistor needed!)
// Connect TP4056 CHRG  pin -> GPIO 26  (LOW = actively charging)
// Connect TP4056 STDBY pin -> GPIO 25  (LOW = charge complete / 100%)
// Set to -1 to disable and use ADC voltage divider mode instead.
// =============================================================================
#define TP4056_PIN_CHRG     26   // LOW = charging in progress
#define TP4056_PIN_STDBY    25   // LOW = battery fully charged

// =============================================================================
// PRIVATE DRIVER STATE
// =============================================================================

static adc_oneshot_unit_handle_t s_adc_unit_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static bool s_cali_enabled = false;
static bool s_is_initialized = false;
static bool s_usb_only_mode = false;    // true when no real battery is detected
static bool s_charging = false;         // true when battery voltage > 4.20V (charging)

// LiPo voltage-to-percentage lookup table
typedef struct {
    float voltage;
    int   percentage;
} battery_lut_point_t;

static const battery_lut_point_t s_lipo_lut[] = {
    {4.20f, 100},
    {4.10f, 90},
    {4.00f, 80},
    {3.90f, 65},
    {3.80f, 50},
    {3.70f, 35},
    {3.60f, 20},
    {3.50f, 10},
    {3.30f, 0}
};
#define LIPO_LUT_SIZE (sizeof(s_lipo_lut) / sizeof(s_lipo_lut[0]))

// =============================================================================
// ADC CALIBRATION HELPER
// =============================================================================

static bool battery_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) calibrated = true;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) calibrated = true;
#endif

    *out_handle = handle;
    if (calibrated) {
        ESP_LOGI(TAG, "ADC Calibration curve initialized.");
    } else {
        ESP_LOGW(TAG, "ADC Calibration scheme not available; using uncalibrated conversion.");
    }

    return calibrated;
}

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

float battery_get_voltage(void)
{
    if (!s_is_initialized) {
        return -1.0f;
    }

    // 1. Oversample ADC to filter electrical noise
    int raw_sum = 0;
    for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        int raw_val = 0;
        esp_err_t ret = adc_oneshot_read(s_adc_unit_handle, BATTERY_ADC_CHANNEL, &raw_val);
        if (ret != ESP_OK) {
            return -1.0f;
        }
        raw_sum += raw_val;
    }
    int raw_avg = raw_sum / ADC_OVERSAMPLE_COUNT;

    // 2. Convert to millivolts
    int voltage_mv = 0;
    if (s_cali_enabled) {
        adc_cali_raw_to_voltage(s_adc_cali_handle, raw_avg, &voltage_mv);
    } else {
        voltage_mv = (raw_avg * 3100) / 4095;
    }

    // 3. Apply voltage divider ratio
    float vadc_volts = (float)voltage_mv / 1000.0f;
    float vbattery = vadc_volts * BATTERY_VOLTAGE_DIVIDER_RATIO;

    return vbattery;
}

int battery_get_percentage(void)
{
    float voltage = battery_get_voltage();
    if (voltage < 0.0f) {
        return 0;
    }

    // ----------------------------------------------------------------
    // USB-only mode: ADC reads near 0V because the voltage divider pin
    // is floating when no battery is connected.
    // We detect this as voltage < 2.5V and flag it as USB mode.
    // ----------------------------------------------------------------
    if (voltage < 2.5f) {
        s_usb_only_mode = true;
        s_charging = false;
        ESP_LOGD(TAG, "USB power detected (no battery). ADC=%.2fV", voltage);
        return 0; // Return 0: display will show "USB" not a fake 100%
    }

    s_usb_only_mode = false;

    // Charging: battery voltage above full charge level means charger is active
    if (voltage > 4.22f) {
        s_charging = true;
        return 100;
    }
    s_charging = false;

    if (voltage >= s_lipo_lut[0].voltage) {
        return 100;
    }
    if (voltage <= s_lipo_lut[LIPO_LUT_SIZE - 1].voltage) {
        ESP_LOGW(TAG, "CRITICAL: Battery depleted! (%.2f V)", voltage);
        return 0;
    }

    // Linear interpolation between LUT entries
    int percentage = 0;
    for (size_t i = 0; i < LIPO_LUT_SIZE - 1; i++) {
        float v_high = s_lipo_lut[i].voltage;
        float v_low  = s_lipo_lut[i + 1].voltage;

        if (voltage <= v_high && voltage >= v_low) {
            int p_high = s_lipo_lut[i].percentage;
            int p_low  = s_lipo_lut[i + 1].percentage;
            percentage = p_low + (int)(((voltage - v_low) / (v_high - v_low)) * (float)(p_high - p_low));
            break;
        }
    }

    if (percentage <= BATTERY_LOW_PERCENTAGE_THRESHOLD) {
        ESP_LOGW(TAG, "Low Battery: %d%% (%.2f V)", percentage, voltage);
    }

    return percentage;
}

/** Returns true when no LiPo battery detected — powered by USB only */
bool battery_is_usb_only(void) {
    return s_usb_only_mode;
}

/** Returns true when battery is actively charging (Vbat > 4.22V) */
bool battery_is_charging(void) {
    return s_charging;
}

// Background FreeRTOS task — reads TP4056 digital pins + ADC every 5 seconds
static void battery_monitor_task(void *param)
{
    while (1) {
#if (TP4056_PIN_CHRG >= 0 && TP4056_PIN_STDBY >= 0)
        // ── TP4056 Digital Mode (NO resistor needed) ──────────────────────
        int chrg  = gpio_get_level(TP4056_PIN_CHRG);   // 0 = charging
        int stdby = gpio_get_level(TP4056_PIN_STDBY);  // 0 = fully charged

        if (chrg == 0 && stdby == 1) {
            // Actively charging
            s_charging = true;
            s_usb_only_mode = false;
            ESP_LOGI(TAG, "TP4056: CHARGING in progress");
            display_set_battery(102);
        } else if (chrg == 1 && stdby == 0) {
            // Charge complete (100%)
            s_charging = false;
            s_usb_only_mode = false;
            ESP_LOGI(TAG, "TP4056: Battery FULL (100%%)");
            display_set_battery(100);
        } else if (chrg == 1 && stdby == 1) {
            // Discharging (no USB connected) — use ADC for real percentage
            s_charging = false;
            s_usb_only_mode = false;
            float voltage = battery_get_voltage();
            int pct = battery_get_percentage();
            ESP_LOGI(TAG, "Battery: %.2fV | %d%%", voltage, pct);
            display_set_battery((uint8_t)pct);
        } else {
            // Both LOW = no battery inserted
            s_usb_only_mode = true;
            ESP_LOGI(TAG, "TP4056: No battery detected");
            display_set_battery(101); // USB icon
        }
#else
        // ── ADC Voltage Divider Mode (requires resistors) ─────────────────
        float voltage = battery_get_voltage();
        if (voltage >= 0.0f) {
            int pct = battery_get_percentage();
            if (battery_is_usb_only()) {
                ESP_LOGI(TAG, "Power: USB only (no battery). ADC=%.2fV", voltage);
                display_set_battery(101);
            } else if (battery_is_charging()) {
                ESP_LOGI(TAG, "Battery: CHARGING (%.2fV)", voltage);
                display_set_battery(102);
            } else {
                ESP_LOGI(TAG, "Battery: %.2fV | %d%%", voltage, pct);
                display_set_battery((uint8_t)pct);
            }
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void battery_init(void)
{
    if (s_is_initialized) return;

    ESP_LOGI(TAG, "Initializing Battery ADC Driver...");

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_unit_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit_handle, BATTERY_ADC_CHANNEL, &chan_config));

    s_cali_enabled = battery_adc_calibration_init(BATTERY_ADC_UNIT, BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN, &s_adc_cali_handle);

#if (TP4056_PIN_CHRG >= 0 && TP4056_PIN_STDBY >= 0)
    gpio_config_t tp_conf = {
        .pin_bit_mask = (1ULL << TP4056_PIN_CHRG) | (1ULL << TP4056_PIN_STDBY),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&tp_conf);
    ESP_LOGI(TAG, "TP4056 Digital inputs configured on GPIO %d (CHRG) & GPIO %d (STDBY)",
             TP4056_PIN_CHRG, TP4056_PIN_STDBY);
#endif

    s_is_initialized = true;

    // Read immediately once to initialize the display before splash transitions
    int initial_pct = battery_get_percentage();
    display_set_battery((uint8_t)initial_pct);

    // Create background FreeRTOS task to keep battery percentage continuously updated
    xTaskCreate(battery_monitor_task, "battery_task", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Battery subsystem initialized. Initial capacity: %d%%", initial_pct);
}
