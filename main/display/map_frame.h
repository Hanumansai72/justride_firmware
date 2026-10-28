#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAP_WIDTH 226
#define MAP_HEIGHT 177
// All functions run under display.c's recursive UI lock.
bool map_frame_packet(const uint8_t *p, size_t len);
void map_frame_invalidate(void);
bool map_frame_draw(bool force);
bool map_frame_active(void);
// Read-only snapshot for the round compositor, under the existing UI lock.
const uint8_t *map_frame_snapshot(unsigned *w,unsigned *h,unsigned *x,unsigned *y);
const char *map_frame_status(void);

// Visual device tilt; does not change GPS heading or route data.
void map_frame_set_tilt(float roll_deg);

uint32_t map_frame_paint_serial(void);
void map_frame_restore_region(int x, int y, int w, int h);

uint32_t map_frame_marker_serial(void);
bool map_frame_marker_overlaps(int x, int y, int w, int h);
