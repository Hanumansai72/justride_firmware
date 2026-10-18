#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAP_WIDTH 198
#define MAP_HEIGHT 194
// All functions run under display.c's recursive UI lock.
bool map_frame_packet(const uint8_t *p, size_t len);
void map_frame_invalidate(void);
bool map_frame_draw(bool force);
bool map_frame_active(void);
const char *map_frame_status(void);
