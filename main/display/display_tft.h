#ifndef DISPLAY_TFT_H
#define DISPLAY_TFT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// HARDWARE SPI PIN CONFIGURATION (ESP32 DevKit to 2.8" SPI TFT)
// =============================================================================
#define TFT_SPI_HOST        SPI2_HOST // HSPI
#define TFT_PIN_MOSI        23        // SPI Master Data Out
#define TFT_PIN_MISO        19        // SPI Master Data In (Shared with Touch MISO)
#define TFT_PIN_SCLK        18        // SPI Clock (Shared with Touch CLK)
#define TFT_PIN_CS          5         // TFT Chip Select
#define TFT_PIN_DC          2         // Data / Command control
#define TFT_PIN_RST         4         // Hardware Reset (-1 if tied to EN/3.3V)
#define TFT_PIN_BCKL        32        // Backlight PWM control (Connect display LED/BL pin to GPIO 32)

#define TFT_SPI_CLOCK_HZ    (40 * 1000 * 1000) // 40 MHz SPI clock

// =============================================================================
// PHYSICAL DIMENSIONS & ORIENTATION
// =============================================================================
#define PHYSICAL_WIDTH      240
#define PHYSICAL_HEIGHT     320

// Display Orientations
#define TFT_ROTATION_PORTRAIT            0
#define TFT_ROTATION_LANDSCAPE           1
#define TFT_ROTATION_PORTRAIT_FLIP       2
#define TFT_ROTATION_LANDSCAPE_FLIP      3

// Active landscape configuration (Usable 320 width x 240 height)
#define DISPLAY_ROTATION    TFT_ROTATION_LANDSCAPE

#if (DISPLAY_ROTATION == TFT_ROTATION_LANDSCAPE || DISPLAY_ROTATION == TFT_ROTATION_LANDSCAPE_FLIP)
    #define DISPLAY_WIDTH   320
    #define DISPLAY_HEIGHT  240
#else
    #define DISPLAY_WIDTH   240
    #define DISPLAY_HEIGHT  320
#endif

// =============================================================================
// 16-BIT RGB565 COLOR DEFINITIONS
// =============================================================================
#define COLOR_BLACK         0x0000  // Pure Black
#define COLOR_WHITE         0xFFFF  // Pure White
#define COLOR_NAVY          0x0000  // Black
#define COLOR_DARKCYAN      0x39E7  // Dark Grey
#define COLOR_DARKGREY      0x39E7  // Dark Grey #383838
#define COLOR_LIGHTGREY     0xC618  // Light Grey #C6C6C6
#define COLOR_RED           0xFFFF  // White (Monochrome High Contrast)
#define COLOR_GREEN         0xFFFF  // White (Monochrome High Contrast)
#define COLOR_PRIMARY_GREEN 0xFFFF  // Pure White in Black & White mode
#define COLOR_AMBER         0xC618  // Light Grey (Monochrome Warning)
#define COLOR_BLUE          0xFFFF  // White
#define COLOR_CYAN          0xFFFF  // White
#define COLOR_SURFACE_DARK  0x0000  // Pure Black background
#define COLOR_CARD_BG       0x0000  // Pure Black card background
#define COLOR_CARD_BORDER   0x39E7  // Subtle dark grey card border

// Turn Direction Enum for Navigation
typedef enum {
    NAV_DIR_NONE = 0,
    NAV_DIR_STRAIGHT,
    NAV_DIR_SLIGHT_LEFT,
    NAV_DIR_SLIGHT_RIGHT,
    NAV_DIR_LEFT,
    NAV_DIR_RIGHT,
    NAV_DIR_SHARP_LEFT,
    NAV_DIR_SHARP_RIGHT,
    NAV_DIR_UTURN,
    NAV_DIR_REACHED,
    NAV_DIR_ROUNDABOUT,
    NAV_DIR_EXIT,
    NAV_DIR_MERGE,
    NAV_DIR_FORK,
} nav_direction_t;

// =============================================================================
// DRIVER APIS
// =============================================================================
esp_err_t tft_init(void);
void tft_set_backlight(uint8_t brightness_pct);
void tft_set_rotation(uint8_t rotation);
void tft_fill_screen(uint16_t color);
void tft_draw_pixel(int x, int y, uint16_t color);
void tft_draw_fast_h_line(int x, int y, int w, uint16_t color);
void tft_draw_fast_v_line(int x, int y, int h, uint16_t color);
void tft_fill_rect(int x, int y, int w, int h, uint16_t color);
void tft_draw_rect(int x, int y, int w, int h, uint16_t color);
void tft_draw_line(int x0, int y0, int x1, int y1, uint16_t color);
void tft_draw_thick_line(int x0, int y0, int x1, int y1, int thickness, uint16_t color);
void tft_draw_circle(int x0, int y0, int r, uint16_t color);
void tft_fill_circle(int x0, int y0, int r, uint16_t color);
void tft_draw_triangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color);
void tft_fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color);

// Text & Glyph functions
void tft_draw_char(int x, int y, char c, uint16_t fg_color, uint16_t bg_color, uint8_t scale);
void tft_draw_string(int x, int y, const char *str, uint16_t fg_color, uint16_t bg_color, uint8_t scale);
void tft_draw_string_centered(int x_center, int y, const char *str, uint16_t fg_color, uint16_t bg_color, uint8_t scale);

// Navigation Scalable Vector Turn Arrow
void tft_draw_turn_arrow(int cx, int cy, int size, nav_direction_t direction, uint16_t color);

/**
 * @brief Bulk-blit raw RGB565 pixels into a display window.
 *        Data must be big-endian RGB565 (high byte first), row-major, 2 bytes/pixel.
 *        Exactly (w * h * 2) bytes must be provided.
 *        This is the fastest way to push an image to the ILI9341 — one SPI transfer per row.
 *
 * @param x      Top-left X of window
 * @param y      Top-left Y of window
 * @param w      Width in pixels
 * @param h      Height in pixels
 * @param data   Pointer to RGB565 big-endian pixel data (w*h*2 bytes)
 */
void tft_draw_raw_pixels(int x, int y, int w, int h, const uint8_t *data);

/**
 * @brief Open the entire 320x240 screen window for sequential streaming of RGB565 chunks.
 */
void tft_stream_image_start(void);

/**
 * @brief Stream a chunk of RGB565 pixels directly into the opened window.
 */
void tft_stream_image_chunk(const uint8_t *data, size_t len);

spi_host_device_t tft_get_spi_host(void);
void tft_spi_lock(void);
void tft_spi_unlock(void);

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_TFT_H
