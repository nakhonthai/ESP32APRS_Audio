#pragma once
typedef struct { //bitmap_t
  const void *pixels;
  const uint8_t bytesPerPixel;
  const uint16_t width;
  const uint16_t height;
  const uint32_t colorKeyLow;
  const uint32_t colorKeyHigh;
  const uint8_t alpha;
} bitmap_t;
