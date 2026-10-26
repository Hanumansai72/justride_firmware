/**
 * @file touch.c
 * @brief XPT2046 resistive touchscreen driver.
 *
 * Key design decisions:
 *  - Pure SPI pressure polling (no IRQ pin dependency)
 *  - Existing Z1/Z2 threshold checks (not a calculated pressure ratio)
 *  - Five-sample trimmed mean, discarding min/max samples
 *  - Explicit SPI mutex per transaction: no bus collision with ILI9341
 *  - Coordinates follow the configured display rotation
 */

#include "touch.h"
#include "display_tft.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "XPT2046";

// =============================================================================
// XPT2046: original 12-bit differential commands, SER/DFR=0, PD1:PD0=00.
// ADC powers down and PENIRQ is enabled between conversions (XPTEK Tables 6-8).
// =============================================================================
#define CMD_X       0xD0  ///< Measure X position
#define CMD_Y       0x90  ///< Measure Y position
#define CMD_Z1      0xB0  ///< Measure Z1 (pressure component 1)
#define CMD_Z2      0xC0  ///< Measure Z2 (pressure component 2)

#define SAMPLE_COUNT    5   ///< Preserve the existing five-sample trimmed mean

static spi_device_handle_t s_touch_spi = NULL;

// =============================================================================
// LOW-LEVEL SPI READ — reads one 12-bit ADC channel
// =============================================================================

static bool xpt2046_read_channel(uint8_t cmd, uint16_t *value) {
    // XPT2046 protocol: send 8-bit command, receive 2 bytes (12-bit result, MSB first)
    // Inline storage supports <=32 bits and avoids DMA bounce buffers for
    // the original 3-byte stack arrays. Command, length and clock are unchanged.
    spi_transaction_t t = {
        .length    = 24,       // 3 bytes = 24 bits
        .flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data   = { cmd, 0x00, 0x00, 0x00 },
    };

    tft_spi_lock();
    esp_err_t err = spi_device_polling_transmit(s_touch_spi, &t);
    tft_spi_unlock();

    if (err != ESP_OK) {
        return false;
    }

    // Result is in bits [14:3] of the 24-bit response
    // rx[0] = dummy, rx[1] = high byte, rx[2] = low byte
    *value = ((uint16_t)(t.rx_data[1] & 0x7F) << 5) | (t.rx_data[2] >> 3);
    return true;
}

// =============================================================================
// TRIMMED MEAN — retain existing filtering, discarding min/max samples
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

static bool read_trimmed_mean(uint8_t cmd, uint16_t *value) {
    uint16_t buf[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (!xpt2046_read_channel(cmd, &buf[i])) return false;
    }
    sort_array(buf, SAMPLE_COUNT);
    // Average middle half (discard lowest & highest quartile outliers)
    uint32_t sum = 0;
    int low  = SAMPLE_COUNT / 4;
    int high = SAMPLE_COUNT - SAMPLE_COUNT / 4;
    for (int i = low; i < high; i++) {
        sum += buf[i];
    }
    *value = (uint16_t)(sum / (high - low));
    return true;
}

// =============================================================================
// PUBLIC: INITIALIZATION
// =============================================================================

esp_err_t touch_init(void) {
    ESP_LOGI(TAG, "Initializing XPT2046 touch controller...");
    ESP_LOGI(TAG, "  Touch mapping: rotation=%d raw-X-inverted=%d", DISPLAY_ROTATION, TOUCH_INVERT_RAW_X);
    ESP_LOGI(TAG, "  T_CS  = GPIO %d", TOUCH_PIN_CS);
    ESP_LOGI(TAG, "  T_CLK = GPIO 18 (shared SPI clock)");
    ESP_LOGI(TAG, "  T_DIN = GPIO 23 (shared MOSI)");
    ESP_LOGI(TAG, "  T_DO  = GPIO 19 (shared MISO — MUST be wired!)");

    // Reserve the wired IRQ as an input; pressure is still polled over SPI.
#if (TOUCH_PIN_IRQ >= 0)
    {
        gpio_config_t irq_conf = {
            .pin_bit_mask = (1ULL << TOUCH_PIN_IRQ),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
        };
        esp_err_t gpio_ret = gpio_config(&irq_conf);
        if (gpio_ret != ESP_OK) return gpio_ret;
        ESP_LOGI(TAG, "  T_IRQ = GPIO %d (touch input; SPI pressure polling)", TOUCH_PIN_IRQ);
    }
#else
    ESP_LOGI(TAG, "  T_IRQ = not used (pure SPI pressure polling)");
#endif

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // Preserve original 2 MHz DCLK
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
    uint16_t warmup;
    if (!xpt2046_read_channel(CMD_Z1, &warmup) ||
        !xpt2046_read_channel(CMD_X, &warmup) ||
        !xpt2046_read_channel(CMD_Y, &warmup)) {
        spi_bus_remove_device(s_touch_spi);
        s_touch_spi = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "XPT2046 initialized successfully.");
    return ESP_OK;
}

// =============================================================================
// PUBLIC: READ TOUCH POINT
// =============================================================================

bool touch_get_point(touch_point_t *point) {
    if (!point) return false;
    point->is_pressed = false;
    point->pressure = 0;
    if (!s_touch_spi) return false;

    // -----------------------------------------------------------------
    // STEP 1: Preserve existing Z1/Z2 host thresholds, not a pressure ratio.
    // -----------------------------------------------------------------
    uint16_t z1, z2;
    if (!xpt2046_read_channel(CMD_Z1, &z1) ||
        !xpt2046_read_channel(CMD_Z2, &z2)) return false;

    // Noise floor / no touch: Z1 is 0 or near 0, Z2 saturated near 4095
    bool touched = (z1 > TOUCH_PRESSURE_THRESHOLD) && (z2 < 4000);
    if (!touched) {
        point->is_pressed = false;
        point->pressure   = 0;
        return false;
    }

    // -----------------------------------------------------------------
    // STEP 2: Sample X and Y using the existing trimmed mean
    // -----------------------------------------------------------------
    uint16_t raw_x, raw_y;
    if (!read_trimmed_mean(CMD_X, &raw_x) ||
        !read_trimmed_mean(CMD_Y, &raw_y)) return false;

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

    // Apply the panel electrode orientation once, before display rotation.
    // Reported PLAY/PAUSE Y=70..76 must become Y=169..163 on this panel.
#if TOUCH_INVERT_RAW_X
    px = 239 - px;
#endif

    // -----------------------------------------------------------------
    // STEP 4: Rotate to landscape (320 x 240)
    //
    // XPT2046 on this panel: raw Y ADC increases from RIGHT to LEFT,
    // so landscape screen_x = (319 - py) corrects the left-right mirror.
    // px already includes the mounted panel's raw-X inversion above.
    // Use the same rotated coordinates for drawing, taps and swipes.
    // -----------------------------------------------------------------
#if (DISPLAY_ROTATION == TFT_ROTATION_LANDSCAPE)
    point->x = (int16_t)(319 - py);  // Corrected: inverts X mirror
    point->y = (int16_t)px;
#elif (DISPLAY_ROTATION == TFT_ROTATION_LANDSCAPE_FLIP)
    point->x = (int16_t)py;
    point->y = (int16_t)(239 - px);
#elif (DISPLAY_ROTATION == TFT_ROTATION_PORTRAIT_FLIP)
    point->x = (int16_t)(239 - px);
    point->y = (int16_t)(319 - py);
#else
    point->x = (int16_t)px;
    point->y = (int16_t)py;
#endif

    // Preserve the existing margin using bounds for the configured rotation.
    if (point->x < TOUCH_EDGE_MARGIN || point->x > (DISPLAY_WIDTH - 1 - TOUCH_EDGE_MARGIN) ||
        point->y < TOUCH_EDGE_MARGIN || point->y > (DISPLAY_HEIGHT - 1 - TOUCH_EDGE_MARGIN)) {
        ESP_LOGD(TAG, "Edge touch (%d, %d) rejected by margin guard", point->x, point->y);
        point->is_pressed = false;
        return false;
    }

    point->pressure   = z1;
    point->is_pressed = true;

    ESP_LOGD(TAG, "Screen X=%d  Y=%d  Pressure=%u", point->x, point->y, z1);
    return true;
}

