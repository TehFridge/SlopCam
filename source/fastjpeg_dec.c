// fastjpeg_dec.c - see fastjpeg_dec.h
#include "fastjpeg_dec.h"
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef int16_t s16; typedef int32_t s32;
// The coefficient buffer is s32[64] for full size, but read/written as packed s16 / u32
// for half size: these types tell the optimiser the accesses may overlap (strict aliasing).
typedef int16_t  __attribute__((may_alias)) s16a;
typedef uint32_t __attribute__((may_alias)) u32a;

// ---------------------------------------------------------------- tables
static const u8 zz_nat[64] = {   // zigzag index -> natural (row*8 + col)
     0, 1, 8,16, 9, 2, 3,10,17,24,32,25,18,11, 4, 5,12,19,26,33,40,48,41,34,27,20,13, 6, 7,14,21,
    28,35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63 };
static s16 zz_half[64];          // zigzag index -> 4x4 natural index, or -1 (coefficient not needed)
// Half-size path keeps coefficients as s16 pairs per row, (f0, f2) then (f1, f3), so the
// 4-point IDCT is four dual multiplies. Unused coefficients go to slot 16 (a sink).
static u8 zz_pack[64];

// colour: R = Y + 1.402 Cr, G = Y - 0.344 Cb - 0.714 Cr, B = Y + 1.772 Cb (full range)
static s16 t_rv[256], t_gu[256], t_gv[256], t_bu[256];
static u16 c_r[768], c_g[768], c_b[768];   // index v + 256, v in -256..511 -> RGB565 part
static u32 t_v2[256], t_u2[256];           // (t_rv << 16 | t_gv), (t_bu << 16 | t_gu): packed for SIMD
// Dequantized coefficients of a valid 8-bit JPEG fit in 12 bits; clamping there (one ssat)
// keeps corrupt data from overflowing the IDCT arithmetic.
#if defined(__arm__) && !defined(__thumb__)
static inline s32 DQ(s32 v) { s32 r; __asm__("ssat %0, #12, %1" : "=r"(r) : "r"(v)); return r; }
#else
static inline s32 DQ(s32 v) { return v < -2048 ? -2048 : v > 2047 ? 2047 : v; }
#endif
// pixel clamp to 0..255 for any int (corrupt data can overshoot a lot): one ARMv6 usat
#if defined(__arm__) && !defined(__thumb__)
static inline u8 CLAMP(s32 v) { s32 r; __asm__("usat %0, #8, %1" : "=r"(r) : "r"(v)); return (u8)r; }
#else
static inline u8 CLAMP(s32 v) { return (u8)(v < 0 ? 0 : v > 255 ? 255 : v); }
#endif
static bool tables_ready = false;

static void init_tables(void) {
    if (tables_ready) return;
    for (int k = 0; k < 64; k++) {
        int n = zz_nat[k], r = n >> 3, c = n & 7;
        zz_half[k] = (r < 4 && c < 4) ? (s16)(r * 4 + c) : -1;
        static const u8 perm[4] = { 0, 2, 1, 3 };
        zz_pack[k] = (r < 4 && c < 4) ? (u8)(r * 4 + perm[c]) : 16;
    }
    for (int i = 0; i < 256; i++) {
        int c = i - 128;
        t_rv[i] = (s16)((359 * c + 128) >> 8);
        t_gu[i] = (s16)((88 * c + 128) >> 8);
        t_gv[i] = (s16)((183 * c + 128) >> 8);
        t_bu[i] = (s16)((454 * c + 128) >> 8);
        t_v2[i] = (u32)t_rv[i] << 16 | (u16)t_gv[i];
        t_u2[i] = (u32)t_bu[i] << 16 | (u16)t_gu[i];
    }
    for (int i = 0; i < 768; i++) {
        int v = i - 256; v = v < 0 ? 0 : v > 255 ? 255 : v;
        c_r[i] = (u16)((v >> 3) << 11);
        c_g[i] = (u16)((v >> 2) << 5);
        c_b[i] = (u16)(v >> 3);
    }
    tables_ready = true;
}

// ---------------------------------------------------------------- Huffman
typedef struct {
    u16 lut[512];                // first 9 bits -> (length << 8) | symbol, 0 = longer code
    s32 fast[512];               // AC: first 9 bits -> whole coefficient when code + value fit:
                                 //   bits 0-4 bits used, 5-6 kind (1 coef, 2 end of block),
                                 //   8-11 zero run, 16-31 value. 0 = use the normal path
    s32 maxcode[18];             // largest code of each length (-1 = none)
    s32 delta[17];               // vals index = code + delta[len]
    u16 skip[512];               // AC: first 9 bits -> bits used (code + value, bits 0-4), zigzag
                                 //   steps (run + 1, bits 5-9), 1 << 10 = end of block, 1 << 11 =
                                 //   ZRL; 0 = slow path. For the coefficients half size doesn't keep.
    u8 vals[256];
    u8 raw[17 + 256]; u16 rawn;  // the DHT bytes this was built from (skip rebuilding)
    bool ok;
} Huff;

static bool build_huff(Huff* h, const u8* bits16, const u8* vals, int nvals) {
    u16 rawn = (u16)(16 + nvals);
    if (h->ok && h->rawn == rawn && !memcmp(h->raw, bits16, 16) && !memcmp(h->raw + 16, vals, nvals)) return true;
    memcpy(h->raw, bits16, 16); memcpy(h->raw + 16, vals, nvals); h->rawn = rawn;
    memcpy(h->vals, vals, nvals);
    memset(h->lut, 0, sizeof(h->lut));
    int code = 0, k = 0;
    for (int len = 1; len <= 16; len++) {
        int n = bits16[len - 1];
        if (code + n > (1 << len) || k + n > nvals) { h->ok = false; return false; }   // corrupt table
        h->delta[len] = k - code;
        for (int i = 0; i < n; i++, k++, code++) {
            if (len <= 9) {
                int fill = 1 << (9 - len), base = code << (9 - len);
                for (int j = 0; j < fill; j++) h->lut[base + j] = (u16)((len << 8) | vals[k]);
            }
        }
        h->maxcode[len] = n ? code - 1 : -1;
        code <<= 1;
    }
    h->maxcode[17] = 0x7FFFFFFF;
    // fast table: for every 9-bit lookahead whose code (and the value bits after it) fit
    for (int i = 0; i < 512; i++) {
        h->fast[i] = 0;
        h->skip[i] = 0;
        u16 e = h->lut[i];
        if (!e) continue;
        {
            int len = e >> 8, rs = e & 0xFF, run = rs >> 4, sz = rs & 15;
            if (rs == 0x00) h->skip[i] = (u16)(len | 1 << 10);
            else if (!sz && run == 15) h->skip[i] = (u16)(len | 16 << 5 | 1 << 11);   // ZRL: 16 zeros
            else if (sz) h->skip[i] = (u16)((len + sz) | (run + 1) << 5);
            // (other symbols with size 0 are invalid: the slow path handles them)
        }
        int len = e >> 8, rs = e & 0xFF, run = rs >> 4, sz = rs & 15;
        if (rs == 0x00) { h->fast[i] = len | (2 << 5); continue; }      // end of block
        if (!sz || len + sz > 9) continue;                               // ZRL / too long
        int v = (i >> (9 - len - sz)) & ((1 << sz) - 1);
        if (v < (1 << (sz - 1))) v -= (1 << sz) - 1;
        h->fast[i] = (s32)((u32)v << 16) | (run << 8) | (1 << 5) | (len + sz);
    }
    h->ok = true;
    return true;
}

// ---------------------------------------------------------------- bit reader
// The scan is first copied with the FF 00 stuffing removed (and RSTn markers noted), with
// zero padding after it, so refilling is branch-free: one 32-bit load per refill.
#define PAD 2048                 // zero bytes after the data (more than an MCU can consume, ~1.3 KB)
typedef struct {
    const u8* p; const u8* end;  // end = first padding byte
    u32 acc;                     // next bits at the top; bits below `n` are zero or stream bits
    int n;                       // valid bits in acc
} Bits;

static inline u32 be32(const u8* p) { u32 v; memcpy(&v, p, 4); return __builtin_bswap32(v); }

static inline void fill(Bits* b) {                    // afterwards n >= 24
    if (b->n <= 24) {
        b->acc |= be32(b->p) >> b->n;
        b->p += (31 - b->n) >> 3;
        b->n |= 24;
    }
}

static inline int get_bits(Bits* b, int s) {        // 1 <= s <= 16, needs n >= s
    int v = (int)(b->acc >> (32 - s));
    b->acc <<= s; b->n -= s;
    return v;
}

static inline int extend(int v, int s) { return v < (1 << (s - 1)) ? v - (1 << s) + 1 : v; }

static inline int huff_decode(Bits* b, const Huff* h) {
    fill(b);
    u16 e = h->lut[b->acc >> 23];
    if (e) { int l = e >> 8; b->acc <<= l; b->n -= l; return e & 0xFF; }
    int l = 10;
    s32 code = (s32)(b->acc >> 22);
    while (code > h->maxcode[l]) { l++; code = (s32)(b->acc >> (32 - l)); }
    if (l > 16) return -1;
    b->acc <<= l; b->n -= l;
    return h->vals[code + h->delta[l]];
}

// ---------------------------------------------------------------- IDCT
// ARMv6 SIMD for the colour conversion (two 16-bit lanes)
#if defined(__arm__) && !defined(__thumb__)
static inline u32 sadd16(u32 a, u32 b) { u32 r; __asm__("sadd16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline u32 ssub16(u32 a, u32 b) { u32 r; __asm__("ssub16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline u32 usat8x2(u32 a) { u32 r; __asm__("usat16 %0, #8, %1" : "=r"(r) : "r"(a)); return r; }
#else
static inline u32 sadd16(u32 a, u32 b) { return (u16)((s16)a + (s16)b) | ((u32)(u16)((s16)(a >> 16) + (s16)(b >> 16)) << 16); }
static inline u32 ssub16(u32 a, u32 b) { return (u16)((s16)a - (s16)b) | ((u32)(u16)((s16)(a >> 16) - (s16)(b >> 16)) << 16); }
static inline u32 usat8x2(u32 a) { s16 l = (s16)a, h = (s16)(a >> 16); l = l < 0 ? 0 : l > 255 ? 255 : l; h = h < 0 ? 0 : h > 255 ? 255 : h; return (u16)l | ((u32)(u16)h << 16); }
#endif
static inline u32 pack16(int hi, int lo) { return (u32)(u16)lo | (u32)hi << 16; }
#if defined(__arm__) && !defined(__thumb__)
static inline u32 pk_tops(u32 a, u32 b) { u32 r; __asm__("pkhtb %0, %1, %2, asr #16" : "=r"(r) : "r"(a), "r"(b)); return r; }   // a.hi : b.hi
static inline u32 pk_bots(u32 a, u32 b) { u32 r; __asm__("pkhbt %0, %2, %1, lsl #16" : "=r"(r) : "r"(a), "r"(b)); return r; }   // a.lo : b.lo
static inline u32 uxtb16(u32 a) { u32 r; __asm__("uxtb16 %0, %1" : "=r"(r) : "r"(a)); return r; }
static inline u32 uxtb16_8(u32 a) { u32 r; __asm__("uxtb16 %0, %1, ror #8" : "=r"(r) : "r"(a)); return r; }
#else
static inline u32 pk_tops(u32 a, u32 b) { return (a & 0xFFFF0000u) | b >> 16; }
static inline u32 pk_bots(u32 a, u32 b) { return a << 16 | (b & 0xFFFF); }
static inline u32 uxtb16(u32 a) { return a & 0x00FF00FFu; }
static inline u32 uxtb16_8(u32 a) { return (a >> 8) & 0x00FF00FFu; }
#endif
#if defined(__arm__) && !defined(__thumb__)
static inline s32 smuad(u32 a, u32 b) { s32 r; __asm__("smuad %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline s32 smultb(u32 a, u32 b) { s32 r; __asm__("smultb %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }   // a.hi * b.lo
static inline s32 smlad(u32 a, u32 b, s32 c) { s32 r; __asm__("smlad %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(c)); return r; }
static inline s32 smlsd(u32 a, u32 b, s32 c) { s32 r; __asm__("smlsd %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(c)); return r; }
static inline u8 usat_sh14(s32 v) { s32 r; __asm__("usat %0, #8, %1, asr #14" : "=r"(r) : "r"(v)); return (u8)r; }
static inline s32 smusd(u32 a, u32 b) { s32 r; __asm__("smusd %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
#else
static inline s32 smuad(u32 a, u32 b) { return (s16)a * (s16)b + (s16)(a >> 16) * (s16)(b >> 16); }
static inline s32 smlad(u32 a, u32 b, s32 c) { return smuad(a, b) + c; }
static inline s32 smultb(u32 a, u32 b) { return (s16)(a >> 16) * (s16)b; }
static inline s32 smlsd(u32 a, u32 b, s32 c) { return (s16)a * (s16)b - (s16)(a >> 16) * (s16)(b >> 16) + c; }
static inline u8 usat_sh14(s32 v) { v >>= 14; return (u8)(v < 0 ? 0 : v > 255 ? 255 : v); }
static inline s32 smusd(u32 a, u32 b) { return (s16)a * (s16)b - (s16)(a >> 16) * (s16)(b >> 16); }
#endif

// One real multiply: left alone, gcc turns constant multiplies into long shift/add chains
// (and runs out of registers).
#if defined(__arm__) && !defined(__thumb__)
static inline s32 M(s32 a, s32 k) { s32 r; __asm__("mul %0, %1, %2" : "=&r"(r) : "r"(a), "r"(k)); return r; }
#else
static inline s32 M(s32 a, s32 k) { return a * k; }
#endif
// 1-D even/odd forms, constants in Q12. Output of the 2-D transforms: pixel - 128, in Q3.
#define FIX(x) ((s32)((x) * 4096 + 0.5))
// 4-point (half size): y[x] = 1/2 sum C(u) F(u) cos((2x+1)u pi/8)
#define W0 FIX(0.353553391)      // 0.5 / sqrt2 (= 0.5 cos(pi/4))
#define W1 FIX(0.461939766)      // 0.5 cos(pi/8)
#define W3 FIX(0.191341716)      // 0.5 cos(3pi/8)

// in: per row r, words in[2r] = (f0 | f2 << 16), in[2r+1] = (f1 | f3 << 16), dequantized s16.
// 1-D 4-point: e0 = W0 (f0 + f2), e1 = W0 (f0 - f2), o0 = W1 f1 + W3 f3, o1 = W3 f1 - W1 f3
static void idct4x4(const u32a* in, u8* out, int stride) {
    const u32 kee = (u32)W0 | (u32)W0 << 16;        // (W0, W0)
    const u32 ko0 = (u32)W1 | (u32)W3 << 16;        // (W1, W3)
    const u32 ko1 = (u32)W3 | (u32)W1 << 16;        // (W3, W1)
    // rows -> Q2, kept in registers as packed pairs: R[r][0] = (y1 : y0), R[r][1] = (y3 : y2)
    u32 R[4][2];
    for (int r = 0; r < 4; r++) {
        const u32 ev = in[2 * r], od = in[2 * r + 1];
        if (!od && !(ev >> 16)) {                   // only f0: flat row (or all zero)
            s32 y = smlad(ev, kee, 512) >> 10;
            R[r][0] = R[r][1] = pk_bots(y, y);
        } else {                                    // (the rounding is folded into the even part)
            s32 e0 = smlad(ev, kee, 512), e1 = smlsd(ev, kee, 512);
            s32 o0 = smuad(od, ko0), o1 = smusd(od, ko1);
            R[r][0] = pk_bots((e1 + o1) >> 10, (e0 + o0) >> 10);
            R[r][1] = pk_bots((e0 - o0) >> 10, (e1 - o1) >> 10);
        }
    }
    // columns -> pixels (Q2 * Q12 = Q14); column c needs (r0, r2) and (r1, r3) packed (lo, hi)
    // + 128 and the rounding folded into the even part (exact: 128 << 14 is a whole step)
    const s32 bias = (128 << 14) + (1 << 13);
    for (int c = 0; c < 4; c++) {
        const int h = c >> 1;
        u32 ev, od;
        if (c & 1) { ev = pk_tops(R[2][h], R[0][h]); od = pk_tops(R[3][h], R[1][h]); }
        else       { ev = pk_bots(R[2][h], R[0][h]); od = pk_bots(R[3][h], R[1][h]); }
        s32 e0 = smlad(ev, kee, bias), e1 = smlsd(ev, kee, bias);
        s32 o0 = smuad(od, ko0), o1 = smusd(od, ko1);
        out[c]              = usat_sh14(e0 + o0);
        out[stride + c]     = usat_sh14(e1 + o1);
        out[2 * stride + c] = usat_sh14(e1 - o1);
        out[3 * stride + c] = usat_sh14(e0 - o0);
    }
}

// 8-point (full size): y[x] = 1/2 sum C(u) F(u) cos((2x+1)u pi/16)
static const s32 c8[8][8] = {   // c8[x][u] = 0.5 C(u) cos((2x+1) u pi / 16), Q12
    { 1448,  2009,  1892,  1703,  1448,  1138,   784,   400},
    { 1448,  1703,   784,  -400, -1448, -2009, -1892, -1138},
    { 1448,  1138,  -784, -2009, -1448,   400,  1892,  1703},
    { 1448,   400, -1892, -1138,  1448,  1703,  -784, -2009},
    { 1448,  -400, -1892,  1138,  1448, -1703,  -784,  2009},
    { 1448, -1138,  -784,  2009, -1448,  -400,  1892, -1703},
    { 1448, -1703,   784,   400, -1448,  2009, -1892,  1138},
    { 1448, -2009,  1892, -1703,  1448, -1138,   784,  -400} };

static void idct8x8(const s32* in, u8* out, int stride) {
    s32 t[64];
    int rows = 0;                                // rows >= this are all zero
    for (int r = 0; r < 8; r++) {
        const s32* f = in + r * 8;
        s32* o = t + r * 8;
        if (!(f[1] | f[2] | f[3] | f[4] | f[5] | f[6] | f[7])) {
            s32 dc = (M(f[0], c8[0][0]) + 256) >> 9;
            for (int x = 0; x < 8; x++) o[x] = dc;
            if (f[0]) rows = r + 1;
            continue;
        }
        rows = r + 1;
        for (int x = 0; x < 4; x++) {
            s32 e = M(f[0], c8[x][0]) + M(f[2], c8[x][2]) + M(f[4], c8[x][4]) + M(f[6], c8[x][6]);
            s32 od = M(f[1], c8[x][1]) + M(f[3], c8[x][3]) + M(f[5], c8[x][5]) + M(f[7], c8[x][7]);
            o[x] = (e + od + 256) >> 9;
            o[7 - x] = (e - od + 256) >> 9;
        }
    }
    for (int c = 0; c < 8; c++) {
        const s32* f = t + c;
        for (int y = 0; y < 4; y++) {
            s32 e, od;
            if (rows <= 2)      { e = M(f[0], c8[y][0]);                          od = M(f[8], c8[y][1]); }
            else if (rows <= 4) { e = M(f[0], c8[y][0]) + M(f[16], c8[y][2]);     od = M(f[8], c8[y][1]) + M(f[24], c8[y][3]); }
            else {
                e = M(f[0], c8[y][0]) + M(f[16], c8[y][2]) + M(f[32], c8[y][4]) + M(f[48], c8[y][6]);
                od = M(f[8], c8[y][1]) + M(f[24], c8[y][3]) + M(f[40], c8[y][5]) + M(f[56], c8[y][7]);
            }
            out[y * stride + c]       = CLAMP(((e + od + (1 << 14)) >> 15) + 128);
            out[(7 - y) * stride + c] = CLAMP(((e - od + (1 << 14)) >> 15) + 128);
        }
    }
}

// ---------------------------------------------------------------- colour conversion
// A whole MCU into framebuffer columns. Each chroma sample is computed once for the HS x VS
// pixels sharing it. Called with constants, so the loops unroll.
static inline __attribute__((always_inline))
void cc_mcu(const u8* yb, const u8* cb, const u8* cr, u16* base, const int BS, const int HS, const int VS) {
    const int MW = BS * HS;
    for (int cx = 0; cx < BS; cx++) {
        u16* col0 = base + (cx * HS) * 240;
        for (int cy = 0; cy < BS; cy++) {
            const int U = cb[cy * BS + cx], V = cr[cy * BS + cx];
            const int rv = t_rv[V] + 256, gu = 256 - t_gu[U] - t_gv[V], bu = t_bu[U] + 256;
            for (int dy = 0; dy < VS; dy++) {
                const int y = cy * VS + dy;
                const u8* py = yb + y * MW + cx * HS;
                for (int dx = 0; dx < HS; dx++) {
                    const int Y = py[dx];
                    col0[dx * 240 - y] = (u16)(c_r[Y + rv] | c_g[Y + gu] | c_b[Y + bu]);
                }
            }
        }
    }
}

// Same, two rows per 32-bit store (rows y and y+1 are neighbours in a framebuffer column).
// base + (-y - 1) must be 4-byte aligned for even y.
static inline __attribute__((always_inline))
void cc_mcu2(const u8* yb, const u8* cb, const u8* cr, u16* base, const int BS, const int HS, const int VS) {
    const int MW = BS * HS;
    for (int cx = 0; cx < BS; cx++) {
        u16* col0 = base + (cx * HS) * 240;
        for (int y = 0; y < BS * VS; y += 2) {
            const int ca = (y / VS) * BS + cx, cb2 = ((y + 1) / VS) * BS + cx;
            int U = cb[ca], V = cr[ca];
            const int rva = t_rv[V] + 256, gua = 256 - t_gu[U] - t_gv[V], bua = t_bu[U] + 256;
            int rvb = rva, gub = gua, bub = bua;
            if (VS == 1) { U = cb[cb2]; V = cr[cb2]; rvb = t_rv[V] + 256; gub = 256 - t_gu[U] - t_gv[V]; bub = t_bu[U] + 256; }
            const u8* p0 = yb + y * MW + cx * HS;
            const u8* p1 = p0 + MW;
            // two pixels per register: hi lane = row y, lo lane = row y + 1 (the lower address)
            const u32 rv2 = pack16(rva - 256, rvb - 256), gu2 = pack16(256 - gua, 256 - gub), bu2 = pack16(bua - 256, bub - 256);
            for (int dx = 0; dx < HS; dx++) {
                const u32 yy = pack16(p0[dx], p1[dx]);
                const u32 r = usat8x2(sadd16(yy, rv2)), g = usat8x2(ssub16(yy, gu2)), b = usat8x2(sadd16(yy, bu2));
                *(u32a*)(col0 + dx * 240 - y - 1) = ((b >> 3) & 0x001F001Fu) | ((r & 0x00F800F8u) << 8) | ((g & 0x00FC00FCu) << 3);
            }
        }
    }
}

// Half-size VGA MCU (8x4 luma pixels for 4:2:2, 8x8 for 4:2:0, 4x4 chroma), written as a plain
// loop so the constants stay in registers (the fully unrolled version spilled them).
// Same results as cc_mcu2(yb, cb, cr, base, 4, 2, VS).
static inline __attribute__((always_inline, optimize("no-unroll-loops"))) void cc_half(const u8* yb, const u8* cb, const u8* cr, u16* base, const int VS) {
    const u32 mb = 0x001F001Fu, mr = 0x00F800F8u, mg = 0x00FC00FCu;
    const int rows = 4 * VS;
    for (int cx = 0; cx < 4; cx++) {
        u16* col = base + cx * 2 * 240;
        const u8* py = yb + cx * 2;
        const u8* pu = cb + cx; const u8* pv = cr + cx;
        for (int y = 0; y < rows; y += 2, py += 16) {
            u32 va = t_v2[*pv], ua = t_u2[*pu];
            u32 vb = va, ub = ua;
            if (VS == 1) { vb = t_v2[pv[4]]; ub = t_u2[pu[4]]; pu += 8; pv += 8; }
            else { pu += 4; pv += 4; }
            const u32 rv2 = pk_tops(va, vb), bu2 = pk_tops(ua, ub);
            const u32 gu2 = pk_bots(sadd16(va, ua), sadd16(vb, ub));
            // rows y, y + 1: two luma pixels each; hi lane = row y
            const u32 w = (u32)*(const u16*)py << 16 | *(const u16*)(py + 8);
            u32 yy = uxtb16(w);
            u32 r = usat8x2(sadd16(yy, rv2)), g = usat8x2(ssub16(yy, gu2)), b = usat8x2(sadd16(yy, bu2));
            *(u32a*)(col - y - 1) = ((b >> 3) & mb) | ((r & mr) << 8) | ((g & mg) << 3);
            yy = uxtb16_8(w);
            r = usat8x2(sadd16(yy, rv2)); g = usat8x2(ssub16(yy, gu2)); b = usat8x2(sadd16(yy, bu2));
            *(u32a*)(col + 240 - y - 1) = ((b >> 3) & mb) | ((r & mr) << 8) | ((g & mg) << 3);
        }
    }
}

static void __attribute__((noinline)) cc_half1(const u8* yb, const u8* cb, const u8* cr, u16* base) { cc_half(yb, cb, cr, base, 1); }
static void __attribute__((noinline)) cc_half2(const u8* yb, const u8* cb, const u8* cr, u16* base) { cc_half(yb, cb, cr, base, 2); }

// ---------------------------------------------------------------- decoder
typedef struct { int id, h, v, tq, td, ta; s32 dc; } Comp;

#define MAX_RST 4096
struct FJDec {
    u16 qt[4][64];               // zigzag order
    u32 qh[4][80];               // half size: q[k] | (byte offset of its packed slot << 16); k >= 64: q 0, the sink
    Huff dc[4], ac[4];
    u8* sbuf; u32 scap;          // the scan without byte stuffing, + PAD zero bytes
    u32 rst[MAX_RST]; int nrst;  // offsets in sbuf where each RSTn marker was
};

FJDec* fjd_create(void) { init_tables(); return (FJDec*)calloc(1, sizeof(FJDec)); }
void fjd_destroy(FJDec* d) { if (d) free(d->sbuf); free(d); }

// Copy the entropy-coded data from p, dropping the 00 after each FF and noting RSTn markers;
// stops at any other marker. Returns the data length (padding follows), or -1 (no memory,
// or more than MAX_RST restart markers).
static long unstuff(FJDec* d, const u8* p, const u8* end) {
    u32 need = (u32)(end - p) + PAD + 4;
    if (d->scap < need) {
        u8* nb = realloc(d->sbuf, need);
        if (!nb) return -1;
        d->sbuf = nb; d->scap = need;
    }
    u8* o = d->sbuf;
    d->nrst = 0;
    while (p < end) {
        // fast: copy 4 bytes at a time while none of them is FF
        while (end - p >= 4) {
            u32 w; memcpy(&w, p, 4);
            u32 x = ~w;                                    // an FF byte -> a zero byte in x
            if ((x - 0x01010101u) & ~x & 0x80808080u) break;
            memcpy(o, &w, 4); o += 4; p += 4;
        }
        if (p >= end) break;
        u8 c = *p++;
        if (c != 0xFF) { *o++ = c; continue; }
        u8 c2 = p < end ? *p : 0xD9;
        if (c2 == 0x00) { *o++ = 0xFF; p++; continue; }
        if (c2 == 0xFF) continue;                         // fill bytes before a marker
        if (c2 >= 0xD0 && c2 <= 0xD7) {
            if (d->nrst >= MAX_RST) return -1;            // absurdly many: let libjpeg-turbo have it
            d->rst[d->nrst++] = (u32)(o - d->sbuf); p++; continue;
        }
        break;                                             // EOI or another marker: end of scan
    }
    long len = o - d->sbuf;
    memset(o, 0, PAD + 4);
    return len;
}

static inline u16 rd16(const u8* p) { return (u16)(p[0] << 8 | p[1]); }

// One block: Huffman-decode, dequantize the coefficients we use, return the last index.
static inline __attribute__((always_inline)) int decode_block_in(Bits* b, Comp* c, const Huff* dct, const Huff* act, const u16* q, s32* coef, bool half) {
    s16a* ch = (s16a*)coef;                       // half size: packed s16 layout (see zz_pack)
    int s = huff_decode(b, dct);
    if (s < 0 || s > 11) return -2;
    int diff = 0;
    if (s) { fill(b); diff = extend(get_bits(b, s), s); }
    c->dc += diff;
    if (c->dc > 32767) c->dc = 32767; else if (c->dc < -32768) c->dc = -32768;   // (corrupt data)
    if (half) ch[0] = (s16)DQ(c->dc * q[0]); else coef[0] = DQ(c->dc * q[0]);
    int last = 0;
    for (int k = 1; k < 64; ) {
        fill(b);
        s32 f = act->fast[b->acc >> 23];
        if (f & (3 << 5)) {                              // fast path: one lookup per coefficient
            int used = f & 31;
            b->acc <<= used; b->n -= used;
            if (f & (2 << 5)) break;                     // end of block
            k += (f >> 8) & 15;
            if (k > 63) return -2;                       // corrupt data: run past the block
            int v = f >> 16;
            if (half) ch[zz_pack[k]] = (s16)DQ(v * q[k]); else coef[zz_nat[k]] = DQ(v * q[k]);
            last = k;
            k++;
            continue;
        }
        int rs = huff_decode(b, act);
        if (rs < 0) return -2;
        int r = rs >> 4; s = rs & 15;
        if (!s) {
            if (r != 15) break;          // end of block
            k += 16; continue;
        }
        k += r;
        if (k > 63) return -2;
        fill(b);
        int v = extend(get_bits(b, s), s);
        if (half) ch[zz_pack[k]] = (s16)DQ(v * q[k]); else coef[zz_nat[k]] = DQ(v * q[k]);
        last = k;
        k++;
    }
    return last;
}

static int decode_block(Bits* bp, Comp* c, const Huff* dct, const Huff* act, const u16* q, s32* coef, bool half) {
    Bits bl = *bp;                               // work on a local copy: stays in registers
    Bits* b = &bl;
    int last = decode_block_in(b, c, dct, act, q, coef, half);
    *bp = bl;
    return last;
}

// Half size (the VGA path), written out on its own: coefficients 1..24 (all of the 4x4 corner
// we keep) are decoded and stored, the rest are only skipped over. Same results as
// decode_block(..., half = true). Clears the packed coefficients itself.
static inline __attribute__((always_inline)) int decode_block_half(Bits* b, Comp* c, const Huff* dct, const Huff* act, const u32* qh, u32* cw) {
    cw[0] = cw[1] = cw[2] = cw[3] = cw[4] = cw[5] = cw[6] = cw[7] = cw[8] = 0;
    s16a* ch = (s16a*)cw;
    int last = -2;
    int s = huff_decode(b, dct);
    if (s < 0 || s > 11) goto out;
    {
        int diff = 0;
        if (s) { fill(b); diff = extend(get_bits(b, s), s); }
        c->dc += diff;
        if (c->dc > 32767) c->dc = 32767; else if (c->dc < -32768) c->dc = -32768;
        ch[0] = (s16)DQ(c->dc * (s32)(qh[0] & 0xFFFF));
    }
    last = 0;
    int k = 1;
    const s32* fast = act->fast;
    u8* const cb8 = (u8*)cw;
    while (k <= 24) {
        fill(b);
        s32 f = fast[b->acc >> 23];
        if (f & (3 << 5)) {
            int used = f & 31;
            b->acc <<= used; b->n -= used;
            if (f & (2 << 5)) goto out;                  // end of block
            k += (f >> 8) & 15;                          // (at most 39 here)
            u32 e = qh[k];                               // value (top of f) * q (bottom of e)
            *(s16a*)(cb8 + (e >> 16)) = (s16)DQ(smultb((u32)f, e));
            last = k++;
            continue;
        }
        int rs = huff_decode(b, act);
        if (rs < 0) { last = -2; goto out; }
        int r = rs >> 4; s = rs & 15;
        if (!s) { if (r != 15) goto out; k += 16; continue; }
        k += r;
        if (k > 63) { last = -2; goto out; }
        fill(b);
        int v = extend(get_bits(b, s), s);
        u32 e = qh[k];
        *(s16a*)(cb8 + (e >> 16)) = (s16)DQ(v * (s32)(e & 0xFFFF));
        last = k++;
    }
    {                                                    // the rest: only skip over it
        const int k0 = k;
        const u16* skip = act->skip;
        while (k < 64) {
            fill(b);
            u32 e = skip[b->acc >> 23];
            if (e) {
                int used = e & 31;
                b->acc <<= used; b->n -= used;
                if (e & (1 << 10)) break;                // end of block
                k += (e >> 5) & 31;
            } else {
                int rs = huff_decode(b, act);
                if (rs < 0) { last = -2; goto out; }
                int r = rs >> 4; s = rs & 15;
                if (!s) { if (r != 15) break; k += 16; }
                else { fill(b); b->acc <<= s; b->n -= s; k += r + 1; }
            }
        }
        if (k > 64) last = -2;                           // corrupt: ran past the block
        else if (k > k0) last = 63;                      // (something there: not a flat block)
    }
out:
    return last;
}

typedef struct { Comp* c; const Huff* dct; const Huff* act; const u32* qh; } BlkSpec;

// All blocks of one MCU, the bit reader kept in registers throughout. 0, or -1 = corrupt.
static int __attribute__((noinline)) decode_mcu_half(Bits* bp, const BlkSpec* spec, int nb, u32 (*cw)[9], int* lasts) {
    Bits bl = *bp;
    int ret = 0;
    for (int i = 0; i < nb; i++) {
        int last = decode_block_half(&bl, spec[i].c, spec[i].dct, spec[i].act, spec[i].qh, cw[i]);
        if (last < -1) { ret = -1; break; }
        lasts[i] = last;
    }
    *bp = bl;
    return ret;
}

bool fjd_decode_fb(FJDec* d, const uint8_t* jpg, uint32_t size, uint16_t* fb, int* out_w, int* out_h) {
    const u8* p = jpg; const u8* end = jpg + size;
    if (size < 4 || p[0] != 0xFF || p[1] != 0xD8) return false;
    p += 2;
    int W = 0, H = 0, ncomp = 0, restart = 0;
    Comp comp[3];
    bool sof = false;
    // ---- headers up to SOS
    for (;;) {
        while (p < end && *p != 0xFF) p++;               // (tolerate junk between segments)
        while (p < end && *p == 0xFF) p++;
        if (p >= end) return false;
        u8 m = *p++;
        if (m == 0xD8 || (m >= 0xD0 && m <= 0xD7) || m == 0x01) continue;
        if (m == 0xD9) return false;
        if (end - p < 2) return false;
        u16 len = rd16(p);
        if (len < 2 || p + len > end) return false;
        const u8* s = p + 2; const u8* se = p + len;
        switch (m) {
        case 0xC0: case 0xC1: {                           // baseline / extended sequential, Huffman
            if (se - s < 6 || s[0] != 8) return false;
            H = rd16(s + 1); W = rd16(s + 3); ncomp = s[5];
            if (ncomp != 3 || se - s < 6 + 9 || W <= 0 || H <= 0) return false;
            for (int i = 0; i < 3; i++) {
                comp[i].id = s[6 + i * 3]; comp[i].h = s[7 + i * 3] >> 4; comp[i].v = s[7 + i * 3] & 15;
                comp[i].tq = s[8 + i * 3] & 3; comp[i].dc = 0;
            }
            if (comp[1].h != 1 || comp[1].v != 1 || comp[2].h != 1 || comp[2].v != 1) return false;
            if (comp[0].h < 1 || comp[0].h > 2 || comp[0].v < 1 || comp[0].v > 2) return false;
            sof = true;
            break;
        }
        case 0xC2: case 0xC3: case 0xC5: case 0xC6: case 0xC7: case 0xC9: case 0xCA: case 0xCB:
        case 0xCD: case 0xCE: case 0xCF:
            return false;                                 // progressive / lossless / arithmetic
        case 0xC4:                                        // DHT
            while (se - s >= 17) {
                int cls = s[0] >> 4, id = s[0] & 3;
                int n = 0;
                for (int i = 1; i <= 16; i++) n += s[i];
                if (n > 256 || se - s < 17 + n || cls > 1) return false;
                if (!build_huff(cls ? &d->ac[id] : &d->dc[id], s + 1, s + 17, n)) return false;
                s += 17 + n;
            }
            break;
        case 0xDB:                                        // DQT
            while (se - s >= 65) {
                if (s[0] >> 4) return false;              // 16-bit tables: not for us
                for (int i = 0; i < 64; i++) d->qt[s[0] & 3][i] = s[1 + i];
                for (int i = 0; i < 80; i++) d->qh[s[0] & 3][i] = i < 64 ? (u32)s[1 + i] | (u32)zz_pack[i] << 17 : 32u << 16;
                s += 65;
            }
            break;
        case 0xDD:                                        // DRI
            if (se - s >= 2) restart = rd16(s);
            break;
        case 0xDA: {                                      // SOS
            if (!sof || se - s < 1 || s[0] != 3 || se - s < 1 + 6 + 3) return false;
            for (int i = 0; i < 3; i++) {
                int cid = s[1 + i * 2], t = s[2 + i * 2];
                int j = 0;
                while (j < 3 && comp[j].id != cid) j++;
                if (j != i) return false;                 // components in frame order only
                comp[i].td = t >> 4 & 3; comp[i].ta = t & 3;
            }
            const u8* ss = s + 7;
            if (ss[0] != 0 || ss[1] != 63 || ss[2] != 0) return false;
            p = se;
            goto scan;
        }
        default: break;                                   // APPn, COM, ...
        }
        p = se;
    }
scan:;
    for (int i = 0; i < 3; i++)
        if (!d->dc[comp[i].td].ok || !d->ac[comp[i].ta].ok) return false;

    // ---- output geometry
    const int hs = comp[0].h, vs = comp[0].v;
    bool half = false; int zoom = 1;
    if (W <= 400 && H <= 240) { if (W * 2 <= 400 && H * 2 <= 240) zoom = 2; }
    else if ((W + 1) / 2 <= 400 && (H + 1) / 2 <= 240) half = true;
    else return false;
    const int bs = half ? 4 : 8;                          // output pixels per block side
    const int ow = half ? (W + 1) / 2 : W, oh = half ? (H + 1) / 2 : H;
    const int sw = ow * zoom, sh = oh * zoom;             // on screen
    const int x0 = (400 - sw) / 2, y0 = (240 - sh) / 2;
    const int mw = bs * hs, mh = bs * vs;                 // MCU in output pixels
    const int mcux = (W + 8 * hs - 1) / (8 * hs), mcuy = (H + 8 * vs - 1) / (8 * vs);
    *out_w = sw; *out_h = sh;

    u8 ybuf[16 * 16] __attribute__((aligned(4))), cb[8 * 8] __attribute__((aligned(4))), cr[8 * 8] __attribute__((aligned(4)));
    s32 coef[64];
    u32 hcw[6][9];                                        // half size: packed coefficients per block
    int lasts[6];
    BlkSpec spec[6];
    const int nl = hs * vs, nblk = nl + 2;
    u8* bout[6]; int bstride[6];                          // half size: where each block's pixels go
    for (int i = 0; i < nblk; i++) {
        const int ci = i < nl ? 0 : i - nl + 1;
        spec[i] = (BlkSpec){ &comp[ci], &d->dc[comp[ci].td], &d->ac[comp[ci].ta], d->qh[comp[ci].tq] };
        if (!ci) { bout[i] = ybuf + (i / hs) * 4 * (4 * hs) + (i % hs) * 4; bstride[i] = 4 * hs; }
        else { bout[i] = ci == 1 ? cb : cr; bstride[i] = 4; }
    }
    long slen = unstuff(d, p, end);
    if (slen < 0) return false;
    const u8* const sdata = d->sbuf;
    Bits b = { sdata, sdata + slen, 0, 0 };
    int todo = restart, rsti = 0;
    const u16* q0 = d->qt[comp[0].tq]; const u16* q1 = d->qt[comp[1].tq]; const u16* q2 = d->qt[comp[2].tq];

    for (int my = 0; my < mcuy; my++) {
        for (int mx = 0; mx < mcux; mx++) {
            if (restart) {
                if (todo == 0) {                          // RSTn: continue at the next marker's data, reset predictions
                    b.acc = 0; b.n = 0;
                    if (rsti < d->nrst) b.p = sdata + d->rst[rsti++];
                    else b.p = b.end;                     // (missing marker: zeros from here)
                    comp[0].dc = comp[1].dc = comp[2].dc = 0;
                    todo = restart;
                }
                todo--;
            }
            // past the data everything reads as zeros, so any position there is the same:
            // pull the pointer back so a corrupt scan can't walk off the padding
            if (b.p > b.end) b.p = b.end;
            if (half) {
                if (decode_mcu_half(&b, spec, nblk, hcw, lasts)) return false;
                for (int i = 0; i < nblk; i++) {
                    u8* o = bout[i]; const int st = bstride[i];
                    if (lasts[i] == 0) {                  // flat block: no IDCT needed
                        s32 v = ((((const s16a*)hcw[i])[0] + 4) >> 3) + 128;
                        u32 w = CLAMP(v) * 0x01010101u;
                        *(u32a*)o = w; *(u32a*)(o + st) = w; *(u32a*)(o + 2 * st) = w; *(u32a*)(o + 3 * st) = w;
                    } else idct4x4((const u32a*)hcw[i], o, st);
                }
                goto colour;
            }
            // luma blocks
            for (int by = 0; by < vs; by++)
                for (int bx = 0; bx < hs; bx++) {
                    memset(coef, 0, 64 * sizeof(s32));
                    int last = decode_block(&b, &comp[0], &d->dc[comp[0].td], &d->ac[comp[0].ta], q0, coef, false);
                    if (last < -1) return false;
                    u8* o = ybuf + by * bs * mw + bx * bs;
                    if (last == 0) {                      // flat block: no IDCT needed
                        s32 dcv = half ? ((const s16a*)coef)[0] : coef[0];
                        s32 v = ((dcv + 4) >> 3) + 128; u8 px = (u8)(v < 0 ? 0 : v > 255 ? 255 : v);
                        for (int r = 0; r < bs; r++) memset(o + r * mw, px, bs);
                    } else if (half) idct4x4((const u32a*)coef, o, mw);
                    else idct8x8(coef, o, mw);
                }
            // chroma
            for (int ci = 1; ci <= 2; ci++) {
                memset(coef, 0, 64 * sizeof(s32));
                int last = decode_block(&b, &comp[ci], &d->dc[comp[ci].td], &d->ac[comp[ci].ta], ci == 1 ? q1 : q2, coef, false);
                if (last < -1) return false;
                u8* o = ci == 1 ? cb : cr;
                if (last == 0) {
                    s32 dcv = half ? ((const s16a*)coef)[0] : coef[0];
                    s32 v = ((dcv + 4) >> 3) + 128; memset(o, (u8)(v < 0 ? 0 : v > 255 ? 255 : v), bs * bs);
                } else if (half) idct4x4((const u32a*)coef, o, bs);
                else idct8x8(coef, o, bs);
            }
        colour:
            // colour convert straight into the framebuffer layout
            const int px0 = mx * mw, py0 = my * mh;
            const int xe = px0 + mw <= ow ? mw : ow - px0, ye = py0 + mh <= oh ? mh : oh - py0;
            if (zoom == 1 && xe == mw && ye == mh) {
                u16* base = fb + (x0 + px0) * 240 + 239 - y0 - py0;
                if (!(y0 & 1)) switch ((half ? 8 : 0) | (hs - 1) << 1 | (vs - 1)) {   // paired stores
                case 8 | 2 | 0: cc_half1(ybuf, cb, cr, base); continue;
                case 8 | 2 | 1: cc_half2(ybuf, cb, cr, base); continue;
                case 8 | 0 | 0: cc_mcu2(ybuf, cb, cr, base, 4, 1, 1); continue;
                case 0 | 2 | 0: cc_mcu2(ybuf, cb, cr, base, 8, 2, 1); continue;
                case 0 | 2 | 1: cc_mcu2(ybuf, cb, cr, base, 8, 2, 2); continue;
                default:        cc_mcu2(ybuf, cb, cr, base, 8, 1, 1); continue;
                }
                switch ((half ? 8 : 0) | (hs - 1) << 1 | (vs - 1)) {   // sizes known at compile time
                case 8 | 2 | 0: cc_mcu(ybuf, cb, cr, base, 4, 2, 1); break;   // VGA 4:2:2 at half size
                case 8 | 2 | 1: cc_mcu(ybuf, cb, cr, base, 4, 2, 2); break;   // 4:2:0 at half size
                case 8 | 0 | 0: cc_mcu(ybuf, cb, cr, base, 4, 1, 1); break;
                case 0 | 2 | 0: cc_mcu(ybuf, cb, cr, base, 8, 2, 1); break;   // 4:2:2 full size
                case 0 | 2 | 1: cc_mcu(ybuf, cb, cr, base, 8, 2, 2); break;
                default:        cc_mcu(ybuf, cb, cr, base, 8, 1, 1); break;
                }
                continue;
            }
            for (int x = 0; x < xe; x++) {
                const int cx = x / hs;
                for (int z = 0; z < zoom; z++) {
                    u16* col = fb + (x0 + (px0 + x) * zoom + z) * 240 + 239 - y0 - py0 * zoom;
                    for (int y = 0; y < ye; y++) {
                        const int cyi = (y / vs) * bs + cx;
                        int Y = ybuf[y * mw + x], U = cb[cyi], V = cr[cyi];
                        u16 px = (u16)(c_r[Y + t_rv[V] + 256] | c_g[Y - t_gu[U] - t_gv[V] + 256] | c_b[Y + t_bu[U] + 256]);
                        for (int zz = 0; zz < zoom; zz++) *col-- = px;
                    }
                }
            }
        }
    }
    return true;
}
