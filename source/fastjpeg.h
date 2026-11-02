#ifndef FASTJPEG_H
#define FASTJPEG_H

// Baseline JPEG encoder tuned for the 3DS ARM11 (ARMv6K).
// - reads the camera's packed YUYV directly (no planar conversion pass)
// - 2-way 16-bit SIMD AAN DCT (sadd16/ssub16/smulw), Q16 constants
// - quantisation by reciprocal multiply, standard Huffman tables
// - 4:2:0 or 4:2:2 output, any size (edges are replicated)
// Same code runs on a PC (portable fallbacks for the SIMD ops) for testing.

#include <stdint.h>

typedef struct {
    int quality;
    uint16_t recip[2][64];   // per zigzag index, Q16 reciprocal of the folded divisor
    uint8_t  qt[2][64];      // quant tables in zigzag order (written to DQT)
    // Exact zero range per coefficient: c quantises to 0  <=>  (u32)(c + lo) <= span,
    // packed as lo (low 16 bits, signed) | span << 16. Lets the scan skip zeros with an
    // add + compare instead of a multiply (whose result latency stalls the ARM11).
    uint32_t dz[2][64];
} FJQuant;

// Prepare tables for a quality (1..100). Cheap; call when quality changes.
void fj_set_quality(FJQuant* q, int quality);

// Encode a packed YUYV image (Y0 U Y1 V per 32-bit word, little endian).
// stride_bytes: bytes per source row (usually w*2). sub422: 1 = 4:2:2, 0 = 4:2:0.
// Returns JPEG size in bytes, or 0 if out_cap was too small.
uint32_t fj_encode_yuyv(const FJQuant* q, const uint8_t* yuyv, int w, int h, int stride_bytes,
                        int sub422, uint8_t* out, uint32_t out_cap);

// ---- Static-block skip (for video) -------------------------------------------------
// A cache remembers, per 16-pixel macroblock, the camera pixels it last encoded and the
// finished Huffman bits of each 8x8 block. On the next frame, a macroblock whose pixels
// are still within `thresh` of that reference (sum of |difference| over every 8-pixel
// row segment, Y+U+V bytes) is not transformed or coded again: its bits are replayed
// (only the DC difference is recomputed). The error is always measured against the
// pixels the bits came from, so it can never creep. thresh 0 = exact repeats only.
// One cache per encoder thread / per stream; it re-sizes itself when w, h, sub422
// or the quality change. If memory runs out it just encodes normally.
typedef struct FJCache FJCache;
FJCache* fj_cache_create(void);
void fj_cache_destroy(FJCache* c);
uint32_t fj_encode_yuyv_cached(const FJQuant* q, const uint8_t* yuyv, int w, int h, int stride_bytes,
                               int sub422, uint8_t* out, uint32_t out_cap, FJCache* c, int thresh);
// Macroblocks skipped / total in the last encode with this cache.
void fj_cache_stats(const FJCache* c, int* skipped, int* total);

// Worst-case output size for a w x h image.
uint32_t fj_max_size(int w, int h);

// Downscale YUYV w x h -> YUYV (w/2) x (h/2) with 2x2 averaging (for half-size streaming).
void fj_half_yuyv(const uint8_t* src, int w, int h, uint8_t* dst);

#endif
