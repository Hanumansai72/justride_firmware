#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// The UI uses a 320-square canvas; the transport expands it to 466-square.
// Existing auxiliary screens keep their 320x240 hitboxes in an inscribed pane.
#define ROUND_UI_SIZE 320
#define ROUND_PANEL_SIZE 466
esp_err_t round_panel_init(void);
void round_panel_brightness(uint8_t percent);
void round_panel_canvas(bool round);
bool round_panel_is_round(void);
void round_panel_window(int x, int y, int w, int h);
void round_panel_pixels(const uint8_t *data, size_t length);
bool round_panel_touch(int raw_x, int raw_y, int16_t *x, int16_t *y);
