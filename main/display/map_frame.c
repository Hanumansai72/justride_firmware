#include "map_frame.h"
#include "display_tft.h"
#include "esp_heap_caps.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#define FRAME_BYTES (MAP_WIDTH * MAP_HEIGHT * 2)
static uint8_t *pixels;
static bool receiving, valid, dirty, on_screen, marker_dirty;
static uint16_t frame_id, sequence, marker_x = 99, marker_y = 97, heading;
static uint16_t drawn_x = 99, drawn_y = 97;
static size_t offset;
static uint32_t expected_crc;
static int64_t started;
static uint8_t status;
static uint32_t last_chunk_crc;
static uint16_t u16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }

void map_frame_invalidate(void) { receiving = false; valid = false; dirty = false; status = 0; }
bool map_frame_active(void) { return receiving || valid || on_screen; }
const char *map_frame_status(void) {
    return status == 0 ? "MAP LOAD" : status == 2 ? "MAP ERROR" : status == 3 ? "MAP QUOTA" : status == 4 ? "MAP EDGE" : "N";
}

bool map_frame_packet(const uint8_t *p, size_t len) {
    if (len == 2 && p[0] == 0x19 && p[1] <= 4) { status = p[1]; return true; }
    if (len < 3) return false;
    uint16_t id = u16(p + 1);
    switch (p[0]) {
    case 0x14: // START, id, width/height, CRC32 of decoded RGB565 bytes
        if (len != 11 || u16(p + 3) != MAP_WIDTH || u16(p + 5) != MAP_HEIGHT) return false;
        if (!pixels) {
            if (heap_caps_get_free_size(MALLOC_CAP_8BIT) < FRAME_BYTES + 32768) return false;
            pixels = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_8BIT);
            if (!pixels) return false;
        }
        frame_id = id; sequence = 0; offset = 0;
        expected_crc = p[7] | ((uint32_t)p[8] << 8) | ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 24);
        receiving = true; valid = false; dirty = false; status = 0; started = esp_timer_get_time();
        return true;
    case 0x15: { // DATA: id, sequence, (run length, RGB565 high, low)...
        if (!receiving || id != frame_id || len < 8 || (len - 5) % 3) return false;
        uint16_t seq = u16(p + 3);
        // App retries identical ATT writes only. Never advance twice on retry.
        uint32_t chunk_crc = esp_rom_crc32_le(0, p, len);
        if (sequence && seq == (uint16_t)(sequence - 1)) return chunk_crc == last_chunk_crc;
        if (seq != sequence) return false;
        size_t expanded = 0;
        for (size_t i = 5; i < len; i += 3) {
            if (!p[i]) return false;
            expanded += p[i] * 2;
        }
        if (offset + expanded > FRAME_BYTES) return false;
        for (size_t i = 5; i < len; i += 3)
            for (unsigned n = 0; n < p[i]; n++) { pixels[offset++] = p[i+1]; pixels[offset++] = p[i+2]; }
        sequence++; last_chunk_crc = chunk_crc; return true;
    }
    case 0x16: // COMMIT: incomplete/corrupt images never reach the display
        if (len != 3 || !receiving || id != frame_id) return false;
        receiving = false;
        if (offset != FRAME_BYTES || esp_rom_crc32_le(0, pixels, FRAME_BYTES) != expected_crc) { status = 2; return false; }
        valid = true; dirty = true; status = 1; marker_x = 99; marker_y = 97; marker_dirty = true;
        ESP_LOGI("map", "frame accepted: raw=%u transfer_us=%lld heap=%u", FRAME_BYTES,
            esp_timer_get_time() - started, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
        return true;
    case 0x17:
        if (len != 3 || id != frame_id) return false;
        receiving = false; return true;
    case 0x18: // marker in the exact downloaded north-up Web Mercator viewport
        if (len != 9 || !valid || receiving || id != frame_id) return false;
        if (u16(p + 3) < 11 || u16(p + 3) >= MAP_WIDTH - 11 ||
            u16(p + 5) < 11 || u16(p + 5) >= MAP_HEIGHT - 28 || u16(p + 7) >= 360) return false;
        marker_x = u16(p + 3); marker_y = u16(p + 5); heading = u16(p + 7); marker_dirty = true;
        if (status == 4) status = 1;
        return true;
    default: return false;
    }
}

bool map_frame_draw(bool force) {
    if (force) on_screen = false;
    if (receiving || !valid) return on_screen;
    if (dirty || !on_screen) {
        int64_t t = esp_timer_get_time();
        tft_draw_raw_pixels(122, 4, MAP_WIDTH, MAP_HEIGHT, pixels);
        ESP_LOGI("map", "blit_us=%lld", esp_timer_get_time() - t);
        on_screen = true; dirty = false; marker_dirty = true;
    } else if (marker_dirty) {
        // Restore just the previous marker patch from the backing image.
        for (int y = drawn_y - 10; y <= drawn_y + 10; y++)
            tft_draw_raw_pixels(122 + drawn_x - 10, 4 + y, 21, 1,
                pixels + (y * MAP_WIDTH + drawn_x - 10) * 2);
    }
    if (marker_dirty) {
        float a = heading * 0.01745329252f;
        int x = 122 + marker_x, y = 4 + marker_y;
        tft_fill_circle(x, y, 9, 0xFFFF);
        tft_fill_triangle(x + (int)(8 * sinf(a)), y - (int)(8 * cosf(a)),
            x + (int)(6 * sinf(a + 2.4f)), y - (int)(6 * cosf(a + 2.4f)),
            x + (int)(6 * sinf(a - 2.4f)), y - (int)(6 * cosf(a - 2.4f)), 0x07D0);
        drawn_x = marker_x; drawn_y = marker_y; marker_dirty = false;
    }
    return true;
}
