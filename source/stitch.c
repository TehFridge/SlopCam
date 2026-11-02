#include "stitch.h"
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint32_t u32;

#define Q 4                  // coarse search works on 1/4 size luma
#define MAX_COARSE_DX 40     // +-160 px at full size
#define MAX_COARSE_DY 4      // +-16 px at full size

// mean of the luma samples used for matching (every Q-th pixel)
static int mean_luma(const u8* img, int w, int h) {
    unsigned long sum = 0, n = 0;
    for (int y = 0; y < h; y += Q)
        for (int x = 0; x < w; x += Q) { sum += img[(y * w + x) * 2]; n++; }
    return n ? (int)(sum / n) : 128;
}

// Mean absolute difference between A(x, y) and B(x - dx, y - dy) over the overlap,
// sampling every `step` pixels, with B's brightness shifted by `off`.
static unsigned cost(const u8* a, const u8* b, int w, int h, int dx, int dy, int step, int off) {
    int x0 = dx > 0 ? dx : 0, x1 = dx > 0 ? w : w + dx;
    int y0 = dy > 0 ? dy : 0, y1 = dy > 0 ? h : h + dy;
    // keep away from the borders a little (lens vignetting / edge artefacts)
    x0 += 8; x1 -= 8; y0 += 8; y1 -= 8;
    if (x1 - x0 < w / 3 || y1 - y0 < h / 3) return ~0u;
    unsigned long sum = 0, n = 0;
    for (int y = y0; y < y1; y += step) {
        const u8* ra = a + (size_t)y * w * 2;
        const u8* rb = b + (size_t)(y - dy) * w * 2;
        for (int x = x0; x < x1; x += step) {
            int d = ra[x * 2] - (rb[(x - dx) * 2] + off);
            sum += d < 0 ? -d : d;
            n++;
        }
    }
    return n ? (unsigned)(sum * 16 / n) : ~0u;
}

StitchInfo stitch_estimate(const u8* a, const u8* b, int w, int h) {
    StitchInfo s;
    memset(&s, 0, sizeof(s));
    int off = mean_luma(a, w, h) - mean_luma(b, w, h);

    // coarse: full search on a 1/4 grid
    int best_dx = 0, best_dy = 0;
    unsigned best = ~0u;
    for (int dy = -MAX_COARSE_DY; dy <= MAX_COARSE_DY; dy++)
        for (int dx = -MAX_COARSE_DX; dx <= MAX_COARSE_DX; dx++) {
            unsigned c = cost(a, b, w, h, dx * Q, dy * Q, Q * 2, off);
            if (c < best) { best = c; best_dx = dx * Q; best_dy = dy * Q; }
        }
    // fine: refine dx then dy (+-Q each) at full resolution, every 2nd pixel
    int cx = best_dx, cy = best_dy;
    best = ~0u;
    for (int dx = cx - Q; dx <= cx + Q; dx++) {
        unsigned c = cost(a, b, w, h, dx, cy, 2, off);
        if (c < best) { best = c; best_dx = dx; }
    }
    best = ~0u;
    for (int dy = cy - Q; dy <= cy + Q; dy++) {
        unsigned c = cost(a, b, w, h, best_dx, dy, 2, off);
        if (c < best) { best = c; best_dy = dy; }
    }

    // A(x) ~ B(x - dx). If A is the left view, B's column 0 shows what A shows at
    // column dx, so dx > 0  <=>  A is the left camera.
    s.a_is_left = best_dx >= 0;
    int d = best_dx < 0 ? -best_dx : best_dx;
    d &= ~1;                                   // keep YUYV words aligned
    if (d > w / 4) d = w / 4 & ~1;
    s.dx = d;
    // vertical offset expressed for the right view: right(y) ~ left(y + dy)
    s.dy = s.a_is_left ? best_dy : -best_dy;
    s.out_w = w + d;
    s.y_offset = s.a_is_left ? off : -off;     // brings the right view's luma to the left's level
    return s;
}

static inline u8 clamp8(int v) { return v < 0 ? 0 : v > 255 ? 255 : (u8)v; }

void stitch_compose(const u8* a, const u8* b, int w, int h, const StitchInfo* s, u8* out) {
    const u8* L = s->a_is_left ? a : b;
    const u8* R = s->a_is_left ? b : a;
    const int d = s->dx, ow = s->out_w;
    const int blend = d ? (w / 8 < 64 ? w / 8 : 64) & ~1 : 0;  // soft seam width inside the overlap
    const int seam_end = w;                                       // left view used up to here
    const int seam_start = seam_end - blend;

    for (int y = 0; y < h; y++) {
        const u8* lrow = L + (size_t)y * w * 2;
        int ry = y - s->dy;                     // matching row of the right view
        if (ry < 0) ry = 0;
        if (ry >= h) ry = h - 1;
        const u8* rrow = R + (size_t)ry * w * 2;
        u8* orow = out + (size_t)y * ow * 2;

        // left view up to the seam
        memcpy(orow, lrow, (size_t)seam_start * 2);
        // blended seam (in the overlap): left fades out, right fades in
        for (int x = seam_start; x < seam_end; x += 2) {
            int t = ((x - seam_start) * 256) / (blend ? blend : 1);
            const u8* lp = lrow + x * 2;
            const u8* rp = rrow + (x - d) * 2;
            orow[x * 2 + 0] = (u8)((lp[0] * (256 - t) + clamp8(rp[0] + s->y_offset) * t) >> 8);
            orow[x * 2 + 1] = (u8)((lp[1] * (256 - t) + rp[1] * t) >> 8);
            orow[x * 2 + 2] = (u8)((lp[2] * (256 - t) + clamp8(rp[2] + s->y_offset) * t) >> 8);
            orow[x * 2 + 3] = (u8)((lp[3] * (256 - t) + rp[3] * t) >> 8);
        }
        // strip only the right camera sees
        for (int x = seam_end; x < ow; x += 2) {
            const u8* rp = rrow + (x - d) * 2;
            orow[x * 2 + 0] = clamp8(rp[0] + s->y_offset);
            orow[x * 2 + 1] = rp[1];
            orow[x * 2 + 2] = clamp8(rp[2] + s->y_offset);
            orow[x * 2 + 3] = rp[3];
        }
    }
}
