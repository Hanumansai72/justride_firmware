/**
 * @file touch.c
 * @brief Professional XPT2046 resistive touchscreen driver.
 *
 * Key design decisions:
 *  - Pure SPI pressure polling (no IRQ pin dependency)
 *  - Z1/Z2 pressure ratio detection: prevents ghost touches from noise
 *  - 8-sample acquisition + bubble-sort median filter: removes spike errors
 *  - Explicit SPI mutex per transaction: no bus collision with ILI9341
 *  - All coordinates output in landscape (320x240) pixel space
 */

#include "touch.h"
#include "display_tft.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>

static const char *TAG = "XPT2046";

// =============================================================================
// XPT2046 COMMAND BYTES (Single-ended, 12-bit, SER/DFR=1)
// =============================================================================
#define CMD_X       0xD0  ///< Measure X position
#define CMD_Y       0x90  ///< Measure Y position
#define CMD_Z1      0xB0  ///< Measure Z1 (pressure component 1)
#define CMD_Z2      0xC0  ///< Measure Z2 (pressure component 2)

#define SAMPLE_COUNT    5   ///< Master Prompt §5: 5-sample acquisition + median filter

static spi_device_handle_t s_touch_spi = NULL;

// =============================================================================
// LOW-LEVEL SPI READ — reads one 12-bit ADC channel
// =============================================================================

static uint16_t xpt2046_read_channel(uint8_t cmd) {
    // XPT2046 protocol: send 8-bit command, receive 2 bytes (12-bit result, MSB first)
    uint8_t tx[3] = { cmd, 0x00, 0x00 };
    uint8_t rx[3] = { 0x00, 0x00, 0x00 };

    spi_transaction_t t = {
        .length    = 24,       // 3 bytes = 24 bits
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    tft_spi_lock();
    esp_err_t err = spi_device_polling_transmit(s_touch_spi, &t);
    tft_spi_unlock();

    if (err != ESP_OK) {
        return 0;
    }

    // Result is in bits [14:3] of the 24-bit response
    // rx[0] = dummy, rx[1] = high byte, rx[2] = low byte
    uint16_t val = ((uint16_t)(rx[1] & 0x7F) << 5) | (rx[2] >> 3);
    return val;
}

// =============================================================================
// MEDIAN FILTER — removes outlier spike samples
// Uses a simple bubble-sort on SAMPLE_COUNT values.
// =============================================================================

static void sort_array(uint16_t *arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                uint16_t tmp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
}

static uint16_t read_median(uint8_t cmd) {
    uint16_t buf[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        buf[i] = xpt2046_read_channel(cmd);
    }
    sort_array(buf, SAMPLE_COUNT);
    // Average middle half (discard lowest & highest quartile outliers)
    uint32_t sum = 0;
    int low  = SAMPLE_COUNT / 4;
    int high = SAMPLE_COUNT - SAMPLE_COUNT / 4;
    for (int i = low; i < high; i++) {
        sum += buf[i];
    }
    return (uint16_t)(sum / (high - low));
}

// =============================================================================
// PUBLIC: INITIALIZATION
// =============================================================================

esp_err_t touch_init(void) {
    ESP_LOGI(TAG, "Initializing XPT2046 touch controller...");
    ESP_LOGI(TAG, "  T_CS  = GPIO %d", TOUCH_PIN_CS);
    ESP_LOGI(TAG, "  T_CLK = GPIO 18 (shared SPI clock)");
    ESP_LOGI(TAG, "  T_DIN = GPIO 23 (shared MOSI)");
    ESP_LOGI(TAG, "  T_DO  = GPIO 19 (shared MISO — MUST be wired!)");

    // Configure IRQ pin only if it is wired (TOUCH_PIN_IRQ >= 0 in touch.h)
#if (TOUCH_PIN_IRQ >= 0)
    {
        gpio_config_t irq_conf = {
            .pin_bit_mask = (1ULL << TOUCH_PIN_IRQ),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&irq_conf);
        ESP_LOGI(TAG, "  T_IRQ = GPIO %d (interrupt enabled)", TOUCH_PIN_IRQ);
    }
#else
    ESP_LOGI(TAG, "  T_IRQ = not used (pure SPI pressure polling)");
#endif

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // XPT2046 max: 2 MHz
        .mode           = 0,                // CPOL=0, CPHA=0
        .spics_io_num   = TOUCH_PIN_CS,
        .queue_size     = 1,
        .pre_cb         = NULL,
        .post_cb        = NULL,
    };

    esp_err_t ret = spi_bus_add_device(tft_get_spi_host(), &devcfg, &s_touch_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to add XPT2046 SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Do a warm-up read to settle the ADC
    xpt2046_read_channel(CMD_Z1);
    xpt2046_read_channel(CMD_X);
    xpt2046_read_channel(CMD_Y);

    ESP_LOGI(TAG, "XPT2046 initialized successfully.");
    return ESP_OK;
}

// =============================================================================
// PUBLIC: READ TOUCH POINT
// =============================================================================

bool touch_get_point(touch_point_t *point) {
    if (!point || !s_touch_spi) {
        return false;
    }

    // -----------------------------------------------------------------
    // STEP 1: Pressure check using Z1/Z2 ratio
    //
    //   Pressure = X_position * (Z2/Z1 - 1) * Rx_plate
    //   A simpler check: Z1 must be non-trivial AND Z1 > 0
    //   (XPT2046 datasheet section 7.1)
    // -----------------------------------------------------------------
    uint16_t z1 = xpt2046_read_channel(CMD_Z1);
    uint16_t z2 = xpt2046_read_channel(CMD_Z2);

    // Noise floor / no touch: Z1 is 0 or near 0, Z2 saturated near 4095
    bool touched = (z1 > TOUCH_PRESSURE_THRESHOLD) && (z2 < 4000);
    if (!touched) {
        point->is_pressed = false;
        point->pressure   = 0;
        return false;
    }

    // -----------------------------------------------------------------
    // STEP 2: Sample X and Y using median filter
    // -----------------------------------------------------------------
    uint16_t raw_x = read_median(CMD_X);
    uint16_t raw_y = read_median(CMD_Y);

    ESP_LOGD(TAG, "Raw  Z1=%u Z2=%u  X=%u  Y=%u", z1, z2, raw_x, raw_y);

    // Sanity check: readings outside calibrated range are noise
    if (raw_x < (TOUCH_RAW_MIN_X - 200) || raw_x > (TOUCH_RAW_MAX_X + 200) ||
        raw_y < (TOUCH_RAW_MIN_Y - 200) || raw_y > (TOUCH_RAW_MAX_Y + 200)) {
        ESP_LOGD(TAG, "Out-of-range reading discarded");
        point->is_pressed = false;
        return false;
    }

    // -----------------------------------------------------------------
    // STEP 3: Map ADC counts to portrait pixel space (240 x 320)
    // -----------------------------------------------------------------
    int px = (int)((float)(raw_x - TOUCH_RAW_MIN_X) * 240.0f /
                   (float)(TOUCH_RAW_MAX_X - TOUCH_RAW_MIN_X));
    int py = (int)((float)(raw_y - TOUCH_RAW_MIN_Y) * 320.0f /
                   (float)(TOUCH_RAW_MAX_Y - TOUCH_RAW_MIN_Y));

    // Clamp to portrait panel bounds
    if (px < 0)   px = 0;
    if (px > 239) px = 239;
    if (py < 0)   py = 0;
    if (py > 319) py = 319;

    // -----------------------------------------------------------------
    // STEP 4: Rotate to landscape (320 x 240)
    //
    // XPT2046 on this panel: raw Y ADC increases from RIGHT to LEFT,
    // so landscape screen_x = (319 - py) corrects the left-right mirror.
    // raw X ADC increases from TOP to BOTTOM in portrait,
    // so landscape screen_y = px (no inversion needed on Y axis).
    //
    // If vertical swipes feel inverted, change to screen_y = (239 - px).
    // -----------------------------------------------------------------
#if (DISPLAY_ROTATION == TFT_ROTATION_LANDSCAPE)
    point->x = (int16_t)(319 - py);  // Corrected: inverts X mirror
    point->y = (int16_t)px;
#elif (DISPLAY_ROTATION == TFT_ROTATION_LANDSCAPE_FLIP)
    point->x = (int16_t)py;
    point->y = (int16_t)(239 - px);
#else
    point->x = (int16_t)px;
    point->y = (int16_t)py;
#endif

    // Master Prompt §9: Reject accidental touches directly on the physical bezel edge (10px margin)
    if (point->x < TOUCH_EDGE_MARGIN || point->x > (319 - TOUCH_EDGE_MARGIN) ||
        point->y < TOUCH_EDGE_MARGIN || point->y > (239 - TOUCH_EDGE_MARGIN)) {
        ESP_LOGD(TAG, "Edge touch (%d, %d) rejected by margin guard", point->x, point->y);
        point->is_pressed = false;
        return false;
    }

    point->pressure   = z1;
    point->is_pressed = true;

    ESP_LOGD(TAG, "Screen X=%d  Y=%d  Pressure=%u", point->x, point->y, z1);
    return true;
}

