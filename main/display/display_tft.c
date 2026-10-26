#include "display_tft.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DISPLAY_TFT";

static spi_device_handle_t s_spi = NULL;
static esp_err_t s_spi_error = ESP_OK;
static uint16_t s_width = DISPLAY_WIDTH;
static uint16_t s_height = DISPLAY_HEIGHT;
static SemaphoreHandle_t s_spi_mutex = NULL;

void tft_spi_lock(void) {
    if (s_spi_mutex) {
        xSemaphoreTakeRecursive(s_spi_mutex, portMAX_DELAY);
    }
}

void tft_spi_unlock(void) {
    if (s_spi_mutex) {
        xSemaphoreGiveRecursive(s_spi_mutex);
    }
}

// =============================================================================
// 5x8 MONOSPACE ASCII FONT TABLE (Characters 32 ' ' to 126 '~')
// =============================================================================
static const uint8_t font5x8[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' ' (Space)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x41, 0x22, 0x14, 0x08, 0x00}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x01, 0x01}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x32}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x08, 0x08, 0x2A, 0x1C, 0x08}  // ~
};

// =============================================================================
// LOW-LEVEL SPI COMMAND & DATA TRANSMISSION
// =============================================================================

static void tft_send_cmd(uint8_t cmd) {
    tft_spi_lock();
    gpio_set_level(TFT_PIN_DC, 0); // Command mode
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    if (err != ESP_OK && s_spi_error == ESP_OK) {
        s_spi_error = err;
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(err));
    }
    tft_spi_unlock();
}

static void tft_send_data(const uint8_t *data, int len) {
    if (len <= 0) return;
    tft_spi_lock();
    gpio_set_level(TFT_PIN_DC, 1); // Data mode
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    if (err != ESP_OK && s_spi_error == ESP_OK) {
        s_spi_error = err;
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(err));
    }
    tft_spi_unlock();
}

static void tft_send_byte(uint8_t val) {
    tft_send_data(&val, 1);
}

static void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    tft_send_cmd(0x2A); // Column Address Set (CASET)
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    tft_send_data(data, 4);

    tft_send_cmd(0x2B); // Page Address Set (PASET)
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    tft_send_data(data, 4);

    tft_send_cmd(0x2C); // Memory Write (RAMWR)
}

// Configure Backlight with LEDC hardware PWM
#if (TFT_PIN_BCKL >= 0)
static void tft_backlight_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT, // 0 to 1023
        .freq_hz          = 5000,              // 5 kHz flicker-free PWM
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = TFT_PIN_BCKL,
        .duty           = 0,                   // Start at 0 to avoid current surge during boot
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "LEDC Backlight PWM configured on GPIO %d (5 kHz)", TFT_PIN_BCKL);
}
#endif

void tft_set_backlight(uint8_t brightness_pct) {
#if (TFT_PIN_BCKL >= 0)
    if (brightness_pct > 100) brightness_pct = 100;
    // Map 0-100% to 10-bit duty cycle (0-1023)
    uint32_t duty = (brightness_pct * 1023) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "Hardware Backlight set to %d%% (Duty: %lu/1023)", brightness_pct, duty);
#endif
}

// =============================================================================
// ILI9341 CONTROLLER INITIALIZATION SEQUENCE
// =============================================================================

esp_err_t tft_init(void) {
    ESP_LOGI(TAG, "Initializing 2.8\" SPI TFT (ILI9341 240x320)...");

    if (!s_spi_mutex) {
        s_spi_mutex = xSemaphoreCreateRecursiveMutex();
        if (!s_spi_mutex) return ESP_ERR_NO_MEM;
    }

    // Configure DC and RST output GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TFT_PIN_DC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (TFT_PIN_RST >= 0) {
        io_conf.pin_bit_mask |= (1ULL << (uint32_t)TFT_PIN_RST);
    }
    gpio_config(&io_conf);

#if (TFT_PIN_BCKL >= 0)
    tft_backlight_init();
    // Keep panel dark during hardware reset: eliminates visible
    // controller-noise flash on soft-restart (esp_restart after BLE update).
    tft_set_backlight(0);
#endif

    // Hardware Reset
    if (TFT_PIN_RST >= 0) {
        gpio_set_level(TFT_PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(TFT_PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    // Configure SPI Bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = TFT_PIN_MOSI,
        .miso_io_num = TFT_PIN_MISO,
        .sclk_io_num = TFT_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 320 * 32 * 2, // DMA buffer size
    };

    esp_err_t ret = spi_bus_initialize(TFT_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Attach TFT device to SPI bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = TFT_PIN_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    ret = spi_bus_add_device(TFT_SPI_HOST, &devcfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TFT SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Standard ILI9341 Initialization Commands
    tft_send_cmd(0x01); // Software Reset
    vTaskDelay(pdMS_TO_TICKS(100));

    tft_send_cmd(0x28); // Display OFF
    tft_send_cmd(0xCB); // Power Control A
    { uint8_t d[] = {0x39, 0x2C, 0x00, 0x34, 0x02}; tft_send_data(d, 5); }
    tft_send_cmd(0xCF); // Power Control B
    { uint8_t d[] = {0x00, 0xC1, 0x30}; tft_send_data(d, 3); }
    tft_send_cmd(0xE8); // Driver timing control A
    { uint8_t d[] = {0x85, 0x00, 0x78}; tft_send_data(d, 3); }
    tft_send_cmd(0xEA); // Driver timing control B
    { uint8_t d[] = {0x00, 0x00}; tft_send_data(d, 2); }
    tft_send_cmd(0xED); // Power on sequence control
    { uint8_t d[] = {0x64, 0x03, 0x12, 0x81}; tft_send_data(d, 4); }
    tft_send_cmd(0xF7); // Pump ratio control
    { uint8_t d[] = {0x20}; tft_send_data(d, 1); }
    tft_send_cmd(0xC0); // Power Control 1
    { uint8_t d[] = {0x23}; tft_send_data(d, 1); }
    tft_send_cmd(0xC1); // Power Control 2
    { uint8_t d[] = {0x10}; tft_send_data(d, 1); }
    tft_send_cmd(0xC5); // VCOM Control 1
    { uint8_t d[] = {0x3E, 0x28}; tft_send_data(d, 2); }
    tft_send_cmd(0xC7); // VCOM Control 2
    { uint8_t d[] = {0x86}; tft_send_data(d, 1); }
    tft_send_cmd(0x3A); // Pixel Format Set: 16 bits/pixel (RGB565)
    { uint8_t d[] = {0x55}; tft_send_data(d, 1); }
    tft_send_cmd(0xB1); // Frame Rate Control
    { uint8_t d[] = {0x00, 0x18}; tft_send_data(d, 2); }

    tft_set_rotation(DISPLAY_ROTATION);

    tft_send_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));
    tft_send_cmd(0x29); // Display ON

    // Pre-clear to black while backlight is still off.
    // display_show_splash() will raise the backlight after the first frame is ready.
    tft_fill_screen(COLOR_BLACK);
    if (s_spi_error != ESP_OK) return s_spi_error;
    ESP_LOGI(TAG, "TFT initialization complete. Mode: Landscape %dx%d.", s_width, s_height);
    return ESP_OK;
}

spi_host_device_t tft_get_spi_host(void) {
    return TFT_SPI_HOST;
}

void tft_set_rotation(uint8_t rotation) {
    tft_send_cmd(0x36); // Memory Access Control (MADCTL)
    uint8_t madctl = 0;
    switch (rotation % 4) {
        case TFT_ROTATION_PORTRAIT:
            madctl = 0x48;
            s_width = PHYSICAL_WIDTH;
            s_height = PHYSICAL_HEIGHT;
            break;
        case TFT_ROTATION_LANDSCAPE:
            madctl = 0x28; // MX, BGR
            s_width = PHYSICAL_HEIGHT;
            s_height = PHYSICAL_WIDTH;
            break;
        case TFT_ROTATION_PORTRAIT_FLIP:
            madctl = 0x88; // MY, BGR
            s_width = PHYSICAL_WIDTH;
            s_height = PHYSICAL_HEIGHT;
            break;
        case TFT_ROTATION_LANDSCAPE_FLIP:
            madctl = 0xE8; // MV, MX, MY, BGR
            s_width = PHYSICAL_HEIGHT;
            s_height = PHYSICAL_WIDTH;
            break;
    }
    tft_send_byte(madctl);
}

// =============================================================================
// PRIMITIVE DRAWING IMPLEMENTATIONS
// =============================================================================

// One reusable 5 KiB strip, not a screen framebuffer. The owning task draws
// complete pixels into RAM; other SPI clients remain behind the recursive lock.
static DMA_ATTR uint8_t s_hud_strip[320*8*2];
static struct { int x,y,w,h; TaskHandle_t owner; } s_tile;
static bool tile_active(void) {
    return s_tile.owner && s_tile.owner==xTaskGetCurrentTaskHandle();
}
static void tile_fill(int x,int y,int w,int h,uint16_t color) {
    int right=x+w,bottom=y+h;
    if(x<s_tile.x)x=s_tile.x;
    if(y<s_tile.y)y=s_tile.y;
    if(right>s_tile.x+s_tile.w)right=s_tile.x+s_tile.w;
    if(bottom>s_tile.y+s_tile.h)bottom=s_tile.y+s_tile.h;
    for(int yy=y;yy<bottom;yy++)for(int xx=x;xx<right;xx++) {
        unsigned at=((yy-s_tile.y)*s_tile.w+xx-s_tile.x)*2;
        s_hud_strip[at]=color>>8;s_hud_strip[at+1]=color&255;
    }
}
void tft_draw_buffered_region(int x,int y,int w,int h,uint16_t background,
                             void (*draw)(void *),void *context) {
    if(!draw || w<=0 || w>320 || h<=0 || x<0 || y<0 || x+w>s_width || y+h>s_height)return;
    tft_spi_lock();
    for(int offset=0;offset<h;offset+=8) {
        int rows=h-offset>8?8:h-offset;
        s_tile.x=x;s_tile.y=y+offset;s_tile.w=w;s_tile.h=rows;
        s_tile.owner=xTaskGetCurrentTaskHandle();
        tile_fill(x,y+offset,w,rows,background);
        draw(context);
        s_tile.owner=NULL;
        tft_draw_raw_pixels(x,y+offset,w,rows,s_hud_strip);
    }
    tft_spi_unlock();
}

void tft_draw_pixel(int x, int y, uint16_t color) {
    if(tile_active()){tile_fill(x,y,1,1,color);return;}
    if (x < 0 || x >= s_width || y < 0 || y >= s_height) return;
    tft_spi_lock();
    tft_set_addr_window(x, y, x, y);
    uint8_t data[2] = { (color >> 8) & 0xFF, color & 0xFF };
    tft_send_data(data, 2);
    tft_spi_unlock();
}


void tft_draw_fast_h_line(int x, int y, int w, uint16_t color) {
    if(tile_active()){tile_fill(x,y,w,1,color);return;}
    if (y < 0 || y >= s_height || x >= s_width) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > s_width) w = s_width - x;
    if (w <= 0) return;

    tft_spi_lock();
    tft_set_addr_window(x, y, x + w - 1, y);
    uint8_t byte_h = (color >> 8) & 0xFF;
    uint8_t byte_l = color & 0xFF;

    uint8_t line_buf[64];
    for (int i = 0; i < 64; i += 2) {
        line_buf[i] = byte_h;
        line_buf[i + 1] = byte_l;
    }

    int remaining = w;
    while (remaining > 0) {
        int chunk = (remaining > 32) ? 32 : remaining;
        tft_send_data(line_buf, chunk * 2);
        remaining -= chunk;
    }    tft_spi_unlock();
}


void tft_draw_fast_v_line(int x, int y, int h, uint16_t color) {
    if(tile_active()){tile_fill(x,y,1,h,color);return;}
    if (x < 0 || x >= s_width || y >= s_height) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > s_height) h = s_height - y;
    if (h <= 0) return;

    tft_spi_lock();
    tft_set_addr_window(x, y, x, y + h - 1);
    uint8_t byte_h = (color >> 8) & 0xFF;
    uint8_t byte_l = color & 0xFF;

    uint8_t line_buf[64];
    for (int i = 0; i < 64; i += 2) {
        line_buf[i] = byte_h;
        line_buf[i + 1] = byte_l;
    }

    int remaining = h;
    while (remaining > 0) {
        int chunk = (remaining > 32) ? 32 : remaining;
        tft_send_data(line_buf, chunk * 2);
        remaining -= chunk;
    }    tft_spi_unlock();
}


void tft_fill_rect(int x, int y, int w, int h, uint16_t color) {
    if(tile_active()){tile_fill(x,y,w,h,color);return;}
    if (x >= s_width || y >= s_height) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > s_width) w = s_width - x;
    if (y + h > s_height) h = s_height - y;
    if (w <= 0 || h <= 0) return;

    tft_spi_lock();
    tft_set_addr_window(x, y, x + w - 1, y + h - 1);
    uint8_t byte_h = (color >> 8) & 0xFF;
    uint8_t byte_l = color & 0xFF;

    // Fast block fill buffer (256 pixels / 512 bytes)
    static DMA_ATTR uint8_t block[512];
    for (int i = 0; i < 512; i += 2) {
        block[i] = byte_h;
        block[i + 1] = byte_l;
    }

    int total_pixels = w * h;
    while (total_pixels > 0) {
        int count = (total_pixels > 256) ? 256 : total_pixels;
        tft_send_data(block, count * 2);
        total_pixels -= count;
    }    tft_spi_unlock();
}


void tft_fill_screen(uint16_t color) {
    tft_fill_rect(0, 0, s_width, s_height, color);
}

// =============================================================================
// BULK IMAGE BLIT — Streams raw RGB565 data to a display window in one SPI pass.
// Data layout: big-endian RGB565, row-major, 2 bytes per pixel.
// Flutter sends little-endian so image_converter.dart swaps bytes before send.
// One row at a time to keep stack usage bounded (max 640 bytes / row).
// =============================================================================
void tft_draw_raw_pixels(int x, int y, int w, int h, const uint8_t *data) {
    if (!data || w <= 0 || h <= 0) return;
    if(tile_active()) {
        int left=x>s_tile.x?x:s_tile.x,top=y>s_tile.y?y:s_tile.y;
        int right=x+w<s_tile.x+s_tile.w?x+w:s_tile.x+s_tile.w;
        int bottom=y+h<s_tile.y+s_tile.h?y+h:s_tile.y+s_tile.h;
        if(right>left)for(int yy=top;yy<bottom;yy++)
            memcpy(s_hud_strip+((yy-s_tile.y)*s_tile.w+left-s_tile.x)*2,
                   data+((yy-y)*w+left-x)*2,(right-left)*2);
        return;
    }
    if (x < 0 || y < 0 || x + w > s_width || y + h > s_height) return;

    tft_spi_lock();
    tft_set_addr_window(x, y, x + w - 1, y + h - 1);

    // Send all pixel data in one large SPI transfer (no byte-swap needed here —
    // the Flutter side already converts to big-endian RGB565 before sending).
    // Split into 512-byte chunks to keep SPI transaction size safe.
    const int total_bytes = w * h * 2;
    int sent = 0;
    while (sent < total_bytes) {
        int chunk = total_bytes - sent;
        if (chunk > 512) chunk = 512;
        tft_send_data(data + sent, chunk);
        sent += chunk;
    }    tft_spi_unlock();
}


static size_t s_stream_pixel = 0;
void tft_stream_image_start(void) {
    tft_spi_lock();
    s_stream_pixel = 0;
    tft_spi_unlock();
}

void tft_stream_image_chunk(const uint8_t *data, size_t len) {
    if (!data || len == 0 || (len & 1)) return;
    tft_spi_lock();
    if (s_stream_pixel + len / 2 > (size_t)s_width * s_height) {
        tft_spi_unlock();
        return;
    }
    while (len > 0) {
        int x = s_stream_pixel % s_width;
        int y = s_stream_pixel / s_width;
        size_t pixels = len / 2;
        if (pixels > (size_t)(s_width - x)) pixels = s_width - x;
        tft_set_addr_window(x, y, x + pixels - 1, y);
        tft_send_data(data, pixels * 2);
        data += pixels * 2;
        len -= pixels * 2;
        s_stream_pixel += pixels;
    }
    tft_spi_unlock();
}

void tft_draw_rect(int x, int y, int w, int h, uint16_t color) {
    tft_draw_fast_h_line(x, y, w, color);
    tft_draw_fast_h_line(x, y + h - 1, w, color);
    tft_draw_fast_v_line(x, y, h, color);
    tft_draw_fast_v_line(x + w - 1, y, h, color);
}

void tft_draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (1) {
        tft_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void tft_draw_thick_line(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
    int half = thickness / 2;
    for (int off = -half; off <= half; off++) {
        if (abs(x1 - x0) > abs(y1 - y0)) {
            tft_draw_line(x0, y0 + off, x1, y1 + off, color);
        } else {
            tft_draw_line(x0 + off, y0, x1 + off, y1, color);
        }
    }
}

void tft_draw_circle(int x0, int y0, int r, uint16_t color) {
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    tft_draw_pixel(x0, y0 + r, color);
    tft_draw_pixel(x0, y0 - r, color);
    tft_draw_pixel(x0 + r, y0, color);
    tft_draw_pixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        tft_draw_pixel(x0 + x, y0 + y, color);
        tft_draw_pixel(x0 - x, y0 + y, color);
        tft_draw_pixel(x0 + x, y0 - y, color);
        tft_draw_pixel(x0 - x, y0 - y, color);
        tft_draw_pixel(x0 + y, y0 + x, color);
        tft_draw_pixel(x0 - y, y0 + x, color);
        tft_draw_pixel(x0 + y, y0 - x, color);
        tft_draw_pixel(x0 - y, y0 - x, color);
    }
}

void tft_fill_circle(int x0, int y0, int r, uint16_t color) {
    tft_draw_fast_v_line(x0, y0 - r, 2 * r + 1, color);
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        tft_draw_fast_v_line(x0 + x, y0 - y, 2 * y + 1, color);
        tft_draw_fast_v_line(x0 - x, y0 - y, 2 * y + 1, color);
        tft_draw_fast_v_line(x0 + y, y0 - x, 2 * x + 1, color);
        tft_draw_fast_v_line(x0 - y, y0 - x, 2 * x + 1, color);
    }
}

void tft_draw_triangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color) {
    tft_draw_line(x0, y0, x1, y1, color);
    tft_draw_line(x1, y1, x2, y2, color);
    tft_draw_line(x2, y2, x0, y0, color);
}

void tft_fill_triangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color) {
    int a, b, y, last;
    if (y0 > y1) { int t=y0; y0=y1; y1=t; t=x0; x0=x1; x1=t; }
    if (y1 > y2) { int t=y2; y2=y1; y1=t; t=x2; x2=x1; x1=t; }
    if (y0 > y1) { int t=y0; y0=y1; y1=t; t=x0; x0=x1; x1=t; }

    if (y0 == y2) {
        a = b = x0;
        if (x1 < a) a = x1; else if (x1 > b) b = x1;
        if (x2 < a) a = x2; else if (x2 > b) b = x2;
        tft_draw_fast_h_line(a, y0, b - a + 1, color);
        return;
    }

    int dx01 = x1 - x0, dy01 = y1 - y0;
    int dx02 = x2 - x0, dy02 = y2 - y0;
    int dx12 = x2 - x1, dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;

    last = (y1 == y2) ? y1 : y1 - 1;

    for (y = y0; y <= last; y++) {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if (a > b) { int t = a; a = b; b = t; }
        tft_draw_fast_h_line(a, y, b - a + 1, color);
    }

    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for (; y <= y2; y++) {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b) { int t = a; a = b; b = t; }
        tft_draw_fast_h_line(a, y, b - a + 1, color);
    }
}

// =============================================================================
// ROUNDED RECTANGLE & ARC PRIMITIVES
// =============================================================================

static void draw_circle_helper(int x0, int y0, int r, uint8_t cornername, uint16_t color) {
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        if (cornername & 0x4) { // Top-Left
            tft_draw_pixel(x0 - x, y0 - y, color);
            tft_draw_pixel(x0 - y, y0 - x, color);
        }
        if (cornername & 0x2) { // Top-Right
            tft_draw_pixel(x0 + x, y0 - y, color);
            tft_draw_pixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) { // Bottom-Left
            tft_draw_pixel(x0 - x, y0 + y, color);
            tft_draw_pixel(x0 - y, y0 + x, color);
        }
        if (cornername & 0x1) { // Bottom-Right
            tft_draw_pixel(x0 + x, y0 + y, color);
            tft_draw_pixel(x0 + y, y0 + x, color);
        }
    }
}

static void fill_circle_helper(int x0, int y0, int r, uint8_t cornername, int delta, uint16_t color) {
    int f = 1 - r;
    int ddF_x = 1;
    int ddF_y = -2 * r;
    int x = 0;
    int y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x1) {
            tft_draw_fast_v_line(x0 + x, y0 - y, 2 * y + 1 + delta, color);
            tft_draw_fast_v_line(x0 + y, y0 - x, 2 * x + 1 + delta, color);
        }
        if (cornername & 0x2) {
            tft_draw_fast_v_line(x0 - x, y0 - y, 2 * y + 1 + delta, color);
            tft_draw_fast_v_line(x0 - y, y0 - x, 2 * x + 1 + delta, color);
        }
    }
}

void tft_draw_round_rect(int x, int y, int w, int h, int r, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    if (r <= 0) {
        tft_draw_rect(x, y, w, h, color);
        return;
    }
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;

    tft_draw_fast_h_line(x + r, y, w - 2 * r, color);
    tft_draw_fast_h_line(x + r, y + h - 1, w - 2 * r, color);
    tft_draw_fast_v_line(x, y + r, h - 2 * r, color);
    tft_draw_fast_v_line(x + w - 1, y + r, h - 2 * r, color);

    draw_circle_helper(x + r, y + r, r, 0x4, color);
    draw_circle_helper(x + w - r - 1, y + r, r, 0x2, color);
    draw_circle_helper(x + w - r - 1, y + h - r - 1, r, 0x1, color);
    draw_circle_helper(x + r, y + h - r - 1, r, 0x8, color);
}

void tft_fill_round_rect(int x, int y, int w, int h, int r, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    if (r <= 0) {
        tft_fill_rect(x, y, w, h, color);
        return;
    }
    if (r > w / 2) r = w / 2;
    if (r > h / 2) r = h / 2;

    tft_fill_rect(x + r, y, w - 2 * r, h, color);
    fill_circle_helper(x + w - r - 1, y + r, r, 0x1, h - 2 * r - 1, color);
    fill_circle_helper(x + r, y + r, r, 0x2, h - 2 * r - 1, color);
}

void tft_draw_arc(int cx, int cy, int r_out, int r_in, float start_deg, float end_deg, uint16_t color) {
    if (r_out <= r_in || r_in <= 0) return;
    float step = 1.0f;
    if (start_deg > end_deg) {
        float tmp = start_deg;
        start_deg = end_deg;
        end_deg = tmp;
    }

    for (float deg = start_deg; deg <= end_deg; deg += step) {
        float rad = deg * (3.14159265f / 180.0f);
        float cos_v = cosf(rad);
        float sin_v = sinf(rad);
        int x_in  = cx + (int)roundf((float)r_in * cos_v);
        int y_in  = cy + (int)roundf((float)r_in * sin_v);
        int x_out = cx + (int)roundf((float)r_out * cos_v);
        int y_out = cy + (int)roundf((float)r_out * sin_v);
        tft_draw_line(x_in, y_in, x_out, y_out, color);
    }
}

// =============================================================================
// SCALABLE VECTOR TURN ARROW RENDERING
// =============================================================================

void tft_draw_turn_arrow(int cx, int cy, int size, nav_direction_t direction, uint16_t color) {
    int half = size / 2;
    int stem_w = size / 3;

    switch (direction) {
        case NAV_DIR_STRAIGHT:
            // Up Arrow
            tft_fill_triangle(cx, cy - half, cx - half, cy, cx + half, cy, color);
            tft_fill_rect(cx - stem_w / 2, cy, stem_w, half, color);
            break;

        case NAV_DIR_LEFT:
            // Smooth Curved 90-degree Left Arrow (Automotive Cockpit HUD)
            tft_fill_rect(cx + 2, cy - 2, 6, half + 2, color);
            tft_fill_circle(cx + 2, cy - 2, 3, color);
            tft_draw_thick_line(cx + 2, cy - 2, cx - half + 9, cy - 2, 6, color);
            tft_fill_triangle(cx - half - 2, cy - 2, cx - half + 9, cy - 10, cx - half + 9, cy + 6, color);
            break;

        case NAV_DIR_RIGHT:
            // Smooth Curved 90-degree Right Arrow (Automotive Cockpit HUD)
            tft_fill_rect(cx - 7, cy - 2, 6, half + 2, color);
            tft_fill_circle(cx - 2, cy - 2, 3, color);
            tft_draw_thick_line(cx - 2, cy - 2, cx + half - 9, cy - 2, 6, color);
            tft_fill_triangle(cx + half + 2, cy - 2, cx + half - 9, cy - 10, cx + half - 9, cy + 6, color);
            break;

        case NAV_DIR_SLIGHT_LEFT:
            tft_fill_triangle(cx - half + 5, cy - half, cx - half + 5, cy + 5, cx + 5, cy - half, color);
            tft_fill_rect(cx - stem_w / 3, cy, stem_w, half, color);
            break;

        case NAV_DIR_SLIGHT_RIGHT:
            tft_fill_triangle(cx + half - 5, cy - half, cx + half - 5, cy + 5, cx - 5, cy - half, color);
            tft_fill_rect(cx - stem_w / 3, cy, stem_w, half, color);
            break;

        case NAV_DIR_SHARP_LEFT:
            tft_fill_triangle(cx - half, cy + half - 4, cx - half + 14, cy + half, cx - half + 2, cy + half - 14, color);
            tft_draw_thick_line(cx + 6, cy - half / 2, cx - half + 6, cy + half - 6, stem_w, color);
            break;

        case NAV_DIR_SHARP_RIGHT:
            tft_fill_triangle(cx + half, cy + half - 4, cx + half - 14, cy + half, cx + half - 2, cy + half - 14, color);
            tft_draw_thick_line(cx - 6, cy - half / 2, cx + half - 6, cy + half - 6, stem_w, color);
            break;

        case NAV_DIR_UTURN:
            tft_fill_circle(cx, cy - size / 4, size / 3, color);
            tft_fill_circle(cx, cy - size / 4, size / 5, COLOR_SURFACE_DARK);
            tft_fill_triangle(cx - size / 3 - 6, cy + size / 4, cx - size / 3 + 6, cy, cx - size / 6, cy + size / 4, color);
            tft_fill_rect(cx + size / 5, cy - size / 4, stem_w / 2, half, color);
            break;

        case NAV_DIR_REACHED:
            tft_fill_circle(cx, cy, half, color);
            tft_fill_circle(cx, cy, half - 5, COLOR_SURFACE_DARK);
            tft_fill_circle(cx, cy, half / 2, color);
            break;

        case NAV_DIR_ROUNDABOUT:
            // Circular roundabout ring with exit chevron
            tft_draw_circle(cx, cy, half - 3, color);
            tft_draw_circle(cx, cy, half - 4, color);
            tft_draw_circle(cx, cy, half - 5, color);
            // Entry stem from bottom
            tft_fill_rect(cx - 2, cy + half - 5, 4, 8, color);
            // Exit arrow top-right
            tft_fill_triangle(cx + half, cy - 4, cx + half - 7, cy - 10, cx + half - 7, cy + 2, color);
            break;

        case NAV_DIR_EXIT:
            // Off-ramp exit right arrow
            tft_draw_thick_line(cx - half / 2, cy + half, cx + half - 6, cy - half + 6, stem_w, color);
            tft_fill_triangle(cx + half, cy - half, cx + half - 12, cy - half, cx + half, cy - half + 12, color);
            break;

        case NAV_DIR_MERGE:
            // Two paths converging
            tft_draw_thick_line(cx - half + 4, cy + half, cx, cy - 2, 3, color);
            tft_draw_thick_line(cx + half - 4, cy + half, cx, cy - 2, 3, color);
            tft_fill_triangle(cx, cy - half, cx - half / 2, cy - 2, cx + half / 2, cy - 2, color);
            break;

        case NAV_DIR_FORK:
            // Fork branching left and right
            tft_fill_rect(cx - 2, cy, 4, half, color);
            tft_draw_thick_line(cx, cy, cx - half + 4, cy - half + 6, 3, color);
            tft_draw_thick_line(cx, cy, cx + half - 4, cy - half + 6, 3, color);
            tft_fill_triangle(cx - half + 2, cy - half + 2, cx - half + 9, cy - half + 2, cx - half + 2, cy - half + 9, color);
            tft_fill_triangle(cx + half - 2, cy - half + 2, cx + half - 9, cy - half + 2, cx + half - 2, cy - half + 9, color);
            break;

        default:
            // Default Compass Dot
            tft_fill_circle(cx, cy, 6, color);
            break;
    }
}

// =============================================================================
// TYPOGRAPHY / TEXT HELPERS
// =============================================================================

void tft_draw_char(int x, int y, char c, uint16_t fg_color, uint16_t bg_color, uint8_t scale) {
    if (c < 32 || c > 126) c = ' ';
    int font_idx = c - 32;

    // Send a complete glyph instead of 40 separate address-window writes.
    // A screen cleared during a swipe must not sit black while text is drawn.
    if (scale >= 1 && scale <= 4 && bg_color != fg_color &&
        x >= 0 && y >= 0 && x + 5 * scale <= s_width && y + 8 * scale <= s_height) {
        static DMA_ATTR uint8_t glyph[5 * 8 * 4 * 4 * 2];
        tft_spi_lock();
        int offset = 0;
        for (int row = 0; row < 8 * scale; row++) {
            for (int col = 0; col < 5 * scale; col++) {
                uint16_t color = font5x8[font_idx][col / scale] & (1 << (row / scale))
                    ? fg_color : bg_color;
                glyph[offset++] = color >> 8;
                glyph[offset++] = color & 0xff;
            }
        }
        tft_draw_raw_pixels(x, y, 5 * scale, 8 * scale, glyph);
        tft_spi_unlock();
        return;
    }

    for (int col = 0; col < 5; col++) {
        uint8_t line = font5x8[font_idx][col];
        for (int row = 0; row < 8; row++) {
            if (line & (1 << row)) {
                tft_fill_rect(x + (col * scale), y + (row * scale), scale, scale, fg_color);
            } else if (bg_color != fg_color) {
                tft_fill_rect(x + (col * scale), y + (row * scale), scale, scale, bg_color);
            }
        }
    }
}

void tft_draw_string(int x, int y, const char *str, uint16_t fg_color, uint16_t bg_color, uint8_t scale) {
    while (*str) {
        tft_draw_char(x, y, *str, fg_color, bg_color, scale);
        x += 6 * scale; // 5 px font + 1 px spacing
        str++;
    }
}

void tft_draw_string_centered(int x_center, int y, const char *str, uint16_t fg_color, uint16_t bg_color, uint8_t scale) {
    int len = strlen(str);
    int total_w = len * 6 * scale;
    int x0 = x_center - (total_w / 2);
    tft_draw_string(x0, y, str, fg_color, bg_color, scale);
}
