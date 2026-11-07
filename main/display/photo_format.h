#pragma once
#include <stdint.h>
#include <stdbool.h>
enum { PHOTO_LEGACY_BYTES=320*240*2 };
static inline bool photo_size_valid(uint32_t bytes) {
    return bytes==PHOTO_LEGACY_BYTES;
}
