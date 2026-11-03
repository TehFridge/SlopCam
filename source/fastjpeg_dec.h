#ifndef FASTJPEG_DEC_H
#define FASTJPEG_DEC_H

// Fast baseline JPEG decoder for playback on the 3DS (the gallery's video player).
// - baseline / extended sequential Huffman, 8-bit, 3 components (YCbCr), luma sampling
//   1x1, 2x1 (4:2:2) or 2x2 (4:2:0), chroma 1x1; restart markers supported
// - pictures bigger than the top screen are decoded at half size straight from the
//   coefficients (only the low 4x4 of each block is used: 4-point IDCT, ~1/4 of the work)
// - small pictures are zoomed 2x
// - output is RGB565 written directly in the top framebuffer's layout (400x240, column
//   order, bottom to top), centred
// Anything else (progressive, grayscale, odd sampling) returns false: use another decoder.

#include <stdint.h>
#include <stdbool.h>

typedef struct FJDec FJDec;

FJDec* fjd_create(void);
void   fjd_destroy(FJDec* d);

// Decode into fb (400*240 u16, framebuffer layout). Only the picture area is written;
// *out_w / *out_h get the shown size (to clear borders when it changes).
bool fjd_decode_fb(FJDec* d, const uint8_t* jpg, uint32_t size, uint16_t* fb, int* out_w, int* out_h);

#endif
