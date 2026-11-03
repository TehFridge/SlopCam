// fastjpeg.c - baseline JPEG encoder tuned for the 3DS ARM11. See fastjpeg.h.
#include "fastjpeg.h"
#include <string.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int16_t  s16;
typedef int32_t  s32;
// 16-bit view of the packed 32-bit DCT blocks; may_alias stops the optimiser from
// reordering these reads around the u32 writes (strict aliasing).
typedef int16_t  __attribute__((may_alias)) s16a;

#if defined(__arm__) && !defined(__thumb__) && defined(__ARM_ARCH) && __ARM_ARCH >= 6
// ---------------------------------------------------------------- ARMv6 SIMD
#define FJ_ARMV6 1
static inline u32 sadd16(u32 a, u32 b) { u32 r; __asm__("sadd16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline u32 ssub16(u32 a, u32 b) { u32 r; __asm__("ssub16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline u32 uhadd8(u32 a, u32 b) { u32 r; __asm__("uhadd8 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline s32 smulwb(s32 c, u32 p) { s32 r; __asm__("smulwb %0, %1, %2" : "=r"(r) : "r"(c), "r"(p)); return r; }
static inline s32 smulwt(s32 c, u32 p) { s32 r; __asm__("smulwt %0, %1, %2" : "=r"(r) : "r"(c), "r"(p)); return r; }
static inline u32 pkhbt(u32 lo, u32 hi) { u32 r; __asm__("pkhbt %0, %1, %2, lsl #16" : "=r"(r) : "r"(lo), "r"(hi)); return r; }
static inline u32 pkhtb(u32 hi, u32 lo) { u32 r; __asm__("pkhtb %0, %1, %2, asr #16" : "=r"(r) : "r"(hi), "r"(lo)); return r; }
static inline int clz32(u32 x) { return __builtin_clz(x); }
static inline u32 uxtb16_r8(u32 a) { u32 r; __asm__("uxtb16 %0, %1, ror #8" : "=r"(r) : "r"(a)); return r; }
static inline u32 usad8(u32 a, u32 b) { u32 r; __asm__("usad8 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline u32 usada8(u32 a, u32 b, u32 acc) { u32 r; __asm__("usada8 %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc)); return r; }
#else
// ---------------------------------------------------------------- portable equivalents (PC tests)
static inline u32 sadd16(u32 a, u32 b) { return (u16)((s16)a + (s16)b) | ((u32)(u16)((s16)(a >> 16) + (s16)(b >> 16)) << 16); }
static inline u32 ssub16(u32 a, u32 b) { return (u16)((s16)a - (s16)b) | ((u32)(u16)((s16)(a >> 16) - (s16)(b >> 16)) << 16); }
static inline u32 uhadd8(u32 a, u32 b) {
    u32 r = 0;
    for (int i = 0; i < 32; i += 8) r |= ((((a >> i) & 0xFF) + ((b >> i) & 0xFF)) >> 1) << i;
    return r;
}
static inline s32 smulwb(s32 c, u32 p) { return (s32)(((int64_t)c * (s16)p) >> 16); }
static inline s32 smulwt(s32 c, u32 p) { return (s32)(((int64_t)c * (s16)(p >> 16)) >> 16); }
static inline u32 pkhbt(u32 lo, u32 hi) { return (lo & 0xFFFF) | (hi << 16); }
static inline u32 pkhtb(u32 hi, u32 lo) { return (hi & 0xFFFF0000u) | (((s32)lo >> 16) & 0xFFFF); }
static inline int clz32(u32 x) { return x ? __builtin_clz(x) : 32; }
static inline u32 uxtb16_r8(u32 a) { return ((a >> 8) & 0xFF) | (((a >> 24) & 0xFF) << 16); }
static inline u32 usad8(u32 a, u32 b) {
    u32 s = 0;
    for (int i = 0; i < 32; i += 8) { int d = (int)((a >> i) & 0xFF) - (int)((b >> i) & 0xFF); s += d < 0 ? -d : d; }
    return s;
}
static inline u32 usada8(u32 a, u32 b, u32 acc) { return acc + usad8(a, b); }
#endif

// per-lane (x * c) >> 16 for both 16-bit lanes, c in Q16
static inline u32 mulq16(u32 p, s32 c) { return pkhbt((u32)smulwb(c, p), (u32)smulwt(c, p)); }

// ---------------------------------------------------------------- tables
static const u8 zigzag[64] = {   // zigzag index -> natural index (row*8+col)
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63 };

// The DCT leaves coefficients transposed: natural (r,c) lives at c*8+r.
// Compile-time constant so the unrolled scan below uses immediate offsets.
static const u8 zz_mem[64] = {   // zigzag k -> index in the (transposed) DCT output
     0,  8,  1,  2,  9, 16, 24, 17, 10,  3,  4, 11, 18, 25, 32, 40,
    33, 26, 19, 12,  5,  6, 13, 20, 27, 34, 41, 48, 56, 49, 42, 35,
    28, 21, 14,  7, 15, 22, 29, 36, 43, 50, 57, 58, 51, 44, 37, 30,
    23, 31, 38, 45, 52, 59, 60, 53, 46, 39, 47, 54, 61, 62, 55, 63 };

static const u8 std_lum_q[64] = {
    16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68,109,103, 77, 24, 35, 55, 64, 81,104,113, 92,
    49, 64, 78, 87,103,121,120,101, 72, 92, 95, 98,112,100,103, 99 };
static const u8 std_chr_q[64] = {
    17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99 };

// AAN scale factors: DCT output for (u,v) = true coef * 16 * a[u] * a[v] (16 = 8 * input<<1)
static const double aan[8] = { 1.0, 1.387039845, 1.306562965, 1.175875602,
                               1.0, 0.785694958, 0.541196100, 0.275899379 };

// Standard Huffman tables (JPEG Annex K)
static const u8 dc_lum_bits[17] = {0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static const u8 dc_lum_val[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const u8 dc_chr_bits[17] = {0,0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static const u8 dc_chr_val[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const u8 ac_lum_bits[17] = {0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static const u8 ac_lum_val[162] = {
    0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
    0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
    0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
    0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
    0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
    0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
    0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
    0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
    0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa };
static const u8 ac_chr_bits[17] = {0,0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static const u8 ac_chr_val[162] = {
    0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
    0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
    0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
    0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
    0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
    0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
    0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa };

typedef struct { u16 code[256]; u8 size[256]; u32 cs[256]; } HuffTable;   // cs = code << 16 | size
static HuffTable h_dc[2], h_ac[2];
// Full bit pattern for (zero run 0..15, value -31..31): (huffman code << nbits | value bits) << 5 | length
#define RV_MAX 31
static u32 h_rv[2][16 * 64];
static int tables_ready = 0;

static void build_huff(HuffTable* t, const u8* bits, const u8* val) {
    u32 code = 0; int k = 0;
    memset(t, 0, sizeof(*t));
    for (int len = 1; len <= 16; len++) {
        for (int i = 0; i < bits[len]; i++) {
            t->code[val[k]] = (u16)code; t->size[val[k]] = (u8)len;
            t->cs[val[k]] = (code << 16) | (u32)len;
            k++; code++;
        }
        code <<= 1;
    }
}

static void init_tables(void) {
    if (tables_ready) return;
    build_huff(&h_dc[0], dc_lum_bits, dc_lum_val);
    build_huff(&h_dc[1], dc_chr_bits, dc_chr_val);
    build_huff(&h_ac[0], ac_lum_bits, ac_lum_val);
    build_huff(&h_ac[1], ac_chr_bits, ac_chr_val);
    for (int t = 0; t < 2; t++)
        for (int run = 0; run < 16; run++)
            for (int v = -RV_MAX; v <= RV_MAX; v++) {
                if (!v) continue;
                u32 a = v < 0 ? -v : v;
                int nb = 32 - __builtin_clz(a);
                u32 vb = v < 0 ? (u32)(v - 1) & ((1u << nb) - 1) : (u32)v;
                u32 cs = h_ac[t].cs[(run << 4) | nb];
                u32 bits = ((cs >> 16) << nb) | vb, len = (cs & 0xFFFF) + nb;
                h_rv[t][(run << 6) + v + 32] = (bits << 5) | len;
            }
    tables_ready = 1;
}

void fj_set_quality(FJQuant* q, int quality) {
    init_tables();
    if (quality < 1) quality = 1;
    if (quality > 100) quality = 100;
    q->quality = quality;
    int scale = quality < 50 ? 5000 / quality : 200 - quality * 2;     // IJG scaling
    for (int t = 0; t < 2; t++) {
        const u8* base = t ? std_chr_q : std_lum_q;
        for (int k = 0; k < 64; k++) {
            int n = zigzag[k];
            int v = (base[n] * scale + 50) / 100;
            if (v < 1) v = 1;
            if (v > 255) v = 255;
            q->qt[t][k] = (u8)v;
            double div = 16.0 * aan[n >> 3] * aan[n & 7] * v;          // >= 1.21, so recip < 2^16
            int r = (int)(65536.0 / div + 0.5);
            q->recip[t][k] = (u16)(r > 65535 ? 65535 : r);
            // zero iff -32768 <= c*r <= 32767  <=>  -floor(32768/r) <= c <= floor(32767/r)
            u32 rr = q->recip[t][k];
            u32 neglo = 32768u / rr, hi = 32767u / rr;
            q->dz[t][k] = (neglo & 0xFFFF) | ((neglo + hi) << 16);
        }
    }
}

// ---------------------------------------------------------------- DCT
#define C_0_382683433  25080
#define C_0_541196100  35468
#define C_0_707106781  46341
#define C_1_306562965  85627

// 1-D AAN DCT on 8 packed values (two lanes = two columns at once). Outputs o0..o7.
#define FDCT1D(d0, d1, d2, d3, d4, d5, d6, d7, o) do { \
    u32 t0 = sadd16(d0, d7), t7 = ssub16(d0, d7); \
    u32 t1 = sadd16(d1, d6), t6 = ssub16(d1, d6); \
    u32 t2 = sadd16(d2, d5), t5 = ssub16(d2, d5); \
    u32 t3 = sadd16(d3, d4), t4 = ssub16(d3, d4); \
    u32 t10 = sadd16(t0, t3), t13 = ssub16(t0, t3); \
    u32 t11 = sadd16(t1, t2), t12 = ssub16(t1, t2); \
    o##0 = sadd16(t10, t11); \
    o##4 = ssub16(t10, t11); \
    u32 z1 = mulq16(sadd16(t12, t13), C_0_707106781); \
    o##2 = sadd16(t13, z1); \
    o##6 = ssub16(t13, z1); \
    t10 = sadd16(t4, t5); t11 = sadd16(t5, t6); t12 = sadd16(t6, t7); \
    u32 z5 = mulq16(ssub16(t10, t12), C_0_382683433); \
    u32 z2 = sadd16(mulq16(t10, C_0_541196100), z5); \
    u32 z4 = sadd16(mulq16(t12, C_1_306562965), z5); \
    u32 z3 = mulq16(t11, C_0_707106781); \
    u32 z11 = sadd16(t7, z3), z13 = ssub16(t7, z3); \
    o##5 = sadd16(z13, z2); \
    o##3 = ssub16(z13, z2); \
    o##1 = sadd16(z11, z4); \
    o##7 = ssub16(z11, z4); \
} while (0)

// Column DCT in place (stride 4 words)
static inline void fdct_cols(u32* d) {
    u32 o0, o1, o2, o3, o4, o5, o6, o7;
    FDCT1D(d[0], d[4], d[8], d[12], d[16], d[20], d[24], d[28], o);
    d[0] = o0; d[4] = o1; d[8] = o2; d[12] = o3; d[16] = o4; d[20] = o5; d[24] = o6; d[28] = o7;
}

// Column DCT on loaded values, written straight into rows 2k, 2k+1 of t (transposed),
// which replaces a separate transpose step.
static inline void fdct_col_vals_T(u32 d0, u32 d1, u32 d2, u32 d3, u32 d4, u32 d5, u32 d6, u32 d7, u32* t, int k) {
    u32 o0, o1, o2, o3, o4, o5, o6, o7;
    FDCT1D(d0, d1, d2, d3, d4, d5, d6, d7, o);
    u32* r0 = t + (2 * k) * 4;
    u32* r1 = r0 + 4;
    r0[0] = pkhbt(o0, o1); r1[0] = pkhtb(o1, o0);
    r0[1] = pkhbt(o2, o3); r1[1] = pkhtb(o3, o2);
    r0[2] = pkhbt(o4, o5); r1[2] = pkhtb(o5, o4);
    r0[3] = pkhbt(o6, o7); r1[3] = pkhtb(o7, o6);
}

// (Y0,Y1) lanes from a YUYV word, << 1: (2*Y) per lane. The JPEG level shift (-128, i.e.
// -256 here) is NOT applied per pixel: a constant added to all 64 inputs only changes the
// DC coefficient (by exactly 64 * 256; every other output is a difference in which it
// cancels), so it's taken off the DC once after the transform (DC_SHIFT). Bit-identical
// output, one instruction less per two pixels.
static inline u32 prep_y(u32 p) { return (p << 1) & 0x01FE01FEu; }
#define DC_SHIFT(b) ((b)[0] = ssub16((b)[0], 64u * 256u))   // low lane = DC

// Luma block straight from YUYV rows into the first DCT pass (no staging copy).
static void __attribute__((noinline)) fdct_y(u32* b, const u32* const* rows, int wx) {
    const u32 *r0 = rows[0] + wx, *r1 = rows[1] + wx, *r2 = rows[2] + wx, *r3 = rows[3] + wx;
    const u32 *r4 = rows[4] + wx, *r5 = rows[5] + wx, *r6 = rows[6] + wx, *r7 = rows[7] + wx;
    for (int k = 0; k < 4; k++)
        fdct_col_vals_T(prep_y(r0[k]), prep_y(r1[k]), prep_y(r2[k]), prep_y(r3[k]),
                        prep_y(r4[k]), prep_y(r5[k]), prep_y(r6[k]), prep_y(r7[k]), b, k);
    fdct_cols(b + 0); fdct_cols(b + 1); fdct_cols(b + 2); fdct_cols(b + 3);
    DC_SHIFT(b);
}

// U and V lanes for chroma column pair k from two YUYV words (p0 = even sample, p1 = odd):
// uxtb16 pulls (U, V) out of each word, two pack instructions regroup them as (U0, U1)
// and (V0, V1), then 2x per lane like the luma path.
static inline void prep_uv(u32 p0, u32 p1, u32* u, u32* v) {
    u32 a = uxtb16_r8(p0), b = uxtb16_r8(p1);     // (U0, V0), (U1, V1)
    *u = pkhbt(a, b) << 1;                        // (U0, U1), level shift: DC_SHIFT
    *v = pkhtb(b, a) << 1;                        // (V0, V1)
}

// Both chroma blocks of an MCU straight from YUYV (rows2 != NULL -> average row pairs, 4:2:0)
static void __attribute__((noinline)) fdct_uv(u32* bu, u32* bv, const u32* const* rows, const u32* const* rows2, int wx) {
    for (int k = 0; k < 4; k++) {
        u32 a[8], c[8];
        for (int r = 0; r < 8; r++) {
            u32 p0 = rows[r][wx + 2 * k], p1 = rows[r][wx + 2 * k + 1];
            if (rows2) { p0 = uhadd8(p0, rows2[r][wx + 2 * k]); p1 = uhadd8(p1, rows2[r][wx + 2 * k + 1]); }
            prep_uv(p0, p1, &a[r], &c[r]);
        }
        fdct_col_vals_T(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], bu, k);
        fdct_col_vals_T(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], bv, k);
    }
    fdct_cols(bu + 0); fdct_cols(bu + 1); fdct_cols(bu + 2); fdct_cols(bu + 3);
    fdct_cols(bv + 0); fdct_cols(bv + 1); fdct_cols(bv + 2); fdct_cols(bv + 3);
    DC_SHIFT(bu); DC_SHIFT(bv);
}

// ---------------------------------------------------------------- bit writer
typedef struct {
    u8* p;
    u8* end;
    u32 acc;     // pending bits, right aligned
    int used;    // number of pending bits (0..31)
    int overflow;
    int raw;     // 1 = capture buffer: whole native words, no 0xFF stuffing
} BitW;

static void __attribute__((noinline)) emit_word(BitW* w, u32 v) {
    if (w->raw) {
        if (w->p + 4 > w->end) { w->overflow = 1; return; }
        memcpy(w->p, &v, 4); w->p += 4;
        return;
    }
    if (w->p + 8 > w->end) { w->overflow = 1; return; }
    // fast path: no 0xFF byte, no stuffing needed
    u32 nv = ~v;
    if (!((nv - 0x01010101u) & ~nv & 0x80808080u)) {
        w->p[0] = v >> 24; w->p[1] = v >> 16; w->p[2] = v >> 8; w->p[3] = v;
        w->p += 4;
        return;
    }
    for (int s = 24; s >= 0; s -= 8) {
        u8 c = v >> s;
        *w->p++ = c;
        if (c == 0xFF) *w->p++ = 0;
    }
}

// Rare path: the 32-bit accumulator is full -> write a word out.
static void __attribute__((noinline)) put_bits_slow(BitW* w, u32 v, int n) {
    int free = 32 - w->used;
    int rest = n - free;
    u32 word = (free == 32 ? 0 : (w->acc << free)) | (v >> rest);
    emit_word(w, word);
    w->acc = rest ? (v & ((1u << rest) - 1)) : 0;
    w->used = rest;
}

static inline void put_bits(BitW* w, u32 v, int n) {   // n <= 27, v < 2^n
    int used = w->used + n;
    if (__builtin_expect(used < 32, 1)) {
        w->acc = (w->acc << n) | v;
        w->used = used;
    } else {
        put_bits_slow(w, v, n);
    }
}

static void flush_bits(BitW* w) {
    // pad with 1 bits to a byte boundary, then emit whole bytes
    int pad = (8 - (w->used & 7)) & 7;
    if (pad) put_bits(w, (1u << pad) - 1, pad);
    while (w->used >= 8) {
        w->used -= 8;
        u8 c = (u8)(w->acc >> w->used);
        if (w->p + 2 > w->end) { w->overflow = 1; return; }
        *w->p++ = c;
        if (c == 0xFF) *w->p++ = 0;
    }
}

// ---------------------------------------------------------------- block coding
static inline int nbits(s32 v) { u32 a = v < 0 ? -v : v; return 32 - clz32(a); }


static void __attribute__((noinline, noclone)) emit_run(BitW* w, const HuffTable* ac, int run, s32 v) {
    while (run > 15) { u32 z = ac->cs[0xF0]; put_bits(w, z >> 16, z & 0xFFFF); run -= 16; }
    int nb = nbits(v);
    u32 vb = v < 0 ? (u32)(v - 1) & ((1u << nb) - 1) : (u32)v;
    u32 cs = ac->cs[(run << 4) | nb];
    put_bits(w, ((cs >> 16) << nb) | vb, (cs & 0xFFFF) + nb);
}

// Quantise + entropy-code one block. Fully unrolled zigzag scan with constant offsets.
// - zero coefficient: load, add, compare against its exact zero range (no multiply)
// - non-zero, |v| <= 31 and run <= 15 (nearly all of them): one load from a table that
//   holds the finished bit pattern, then a shift/or into the bit accumulator
// - anything else: shared out-of-line path
// The bit accumulator lives in registers for the whole block.
static void __attribute__((noinline, noclone)) encode_block(BitW* w, const u32* blk, const u16* recip,
                                                    const u32* dz, const HuffTable* dct, const HuffTable* ac,
                                                    const u32* rv, int* dc_prev) {
    const s16a* c = (const s16a*)blk;
    u32 acc = w->acc;
    int used = w->used;
    u8* op = w->p;
    // full words go out inline when they need no FF stuffing and there's room (never in
    // capture mode: lim = NULL); anything else takes the out-of-line path
    u8* const lim = w->raw ? NULL : w->end - 8;
#define FLUSH_STATE() do { w->acc = acc; w->used = used; w->p = op; } while (0)
#define LOAD_STATE()  do { acc = w->acc; used = w->used; op = w->p; } while (0)
#define PUT(bits, n) do { u32 b_ = (bits); int n_ = (n); int u_ = used + n_; \
        if (__builtin_expect(u_ < 32, 1)) { acc = (acc << n_) | b_; used = u_; } \
        else { const int rest_ = u_ - 32; \
            u32 wd_ = (acc << (32 - used)) | (b_ >> rest_); u32 nw_ = ~wd_; \
            if (__builtin_expect(op <= lim && !((nw_ - 0x01010101u) & ~nw_ & 0x80808080u), 1)) { \
                wd_ = __builtin_bswap32(wd_); memcpy(op, &wd_, 4); op += 4; acc = b_; used = rest_; \
            } else { FLUSH_STATE(); put_bits_slow(w, b_, n_); LOAD_STATE(); } } } while (0)

    s32 dc = (c[0] * (s32)recip[0] + 0x8000) >> 16;
    s32 diff = dc - *dc_prev; *dc_prev = dc;
    int nb = nbits(diff);
    u32 vb = diff < 0 ? (u32)(diff - 1) & ((1u << nb) - 1) : (u32)diff;
    u32 dcs = dct->cs[nb];
    PUT(((dcs >> 16) << nb) | vb, (dcs & 0xFFFF) + nb);

    int last = 0;
#define NONZERO(k, cv) ((u32)((cv) + (s32)(s16)dz[k]) > (dz[k] >> 16))
// Past k = 15 most coefficients are zero: hint the symbol code cold so gcc moves it out of
// line and the zero scan stays a compact straight run (I-cache friendly).
#define HOT(k) ((k) < 16 ? NONZERO(k, cv) : __builtin_expect(NONZERO(k, cv), 0))
#define AC(k) { s32 cv = c[zz_mem[k]]; if (HOT(k)) { \
        s32 v = (cv * (s32)recip[k] + 0x8000) >> 16; int run = k - last - 1; last = k; \
        __asm__("" : "+r"(last));   /* opaque: stops gcc from cloning the rest of the scan per value of last */ \
        if (__builtin_expect(run <= 15 && (u32)(v + RV_MAX) <= 2 * RV_MAX, 1)) { \
            u32 e = rv[(run << 6) + v + 32]; PUT(e >> 5, e & 31); \
        } else { FLUSH_STATE(); emit_run(w, ac, run, v); LOAD_STATE(); } } }
    AC(1) AC(2) AC(3) AC(4) AC(5) AC(6) AC(7) AC(8) AC(9) AC(10) AC(11) AC(12) AC(13) AC(14) AC(15)
    AC(16) AC(17) AC(18) AC(19) AC(20) AC(21) AC(22) AC(23) AC(24) AC(25) AC(26) AC(27) AC(28) AC(29) AC(30) AC(31)
    AC(32) AC(33) AC(34) AC(35) AC(36) AC(37) AC(38) AC(39) AC(40) AC(41) AC(42) AC(43) AC(44) AC(45) AC(46) AC(47)
    AC(48) AC(49) AC(50) AC(51) AC(52) AC(53) AC(54) AC(55) AC(56) AC(57) AC(58) AC(59) AC(60) AC(61) AC(62) AC(63)
#undef AC
#undef NONZERO
#undef HOT
    if (last != 63) { u32 e = ac->cs[0x00]; PUT(e >> 16, e & 0xFFFF); }
    FLUSH_STATE();
#undef PUT
#undef FLUSH_STATE
#undef LOAD_STATE
}

// ---------------------------------------------------------------- block loaders (from YUYV)

// ---------------------------------------------------------------- headers
static u8* put16(u8* p, int v) { p[0] = v >> 8; p[1] = v; return p + 2; }

static u8* write_dht(u8* p, int cls_id, const u8* bits, const u8* val) {
    int n = 0;
    for (int i = 1; i <= 16; i++) n += bits[i];
    *p++ = 0xFF; *p++ = 0xC4;
    p = put16(p, 2 + 1 + 16 + n);
    *p++ = (u8)cls_id;
    memcpy(p, bits + 1, 16); p += 16;
    memcpy(p, val, n); p += n;
    return p;
}

static u8* write_headers(u8* p, const FJQuant* q, int w, int h, int sub422) {
    static const u8 app0[] = { 0xFF,0xE0, 0,16, 'J','F','I','F',0, 1,1, 0, 0,1, 0,1, 0,0 };
    *p++ = 0xFF; *p++ = 0xD8;
    memcpy(p, app0, sizeof(app0)); p += sizeof(app0);
    for (int t = 0; t < 2; t++) {
        *p++ = 0xFF; *p++ = 0xDB; p = put16(p, 67); *p++ = (u8)t;
        memcpy(p, q->qt[t], 64); p += 64;            // DQT is stored in zigzag order
    }
    *p++ = 0xFF; *p++ = 0xC0; p = put16(p, 17); *p++ = 8;
    p = put16(p, h); p = put16(p, w); *p++ = 3;
    *p++ = 1; *p++ = sub422 ? 0x21 : 0x22; *p++ = 0;
    *p++ = 2; *p++ = 0x11; *p++ = 1;
    *p++ = 3; *p++ = 0x11; *p++ = 1;
    p = write_dht(p, 0x00, dc_lum_bits, dc_lum_val);
    p = write_dht(p, 0x10, ac_lum_bits, ac_lum_val);
    p = write_dht(p, 0x01, dc_chr_bits, dc_chr_val);
    p = write_dht(p, 0x11, ac_chr_bits, ac_chr_val);
    *p++ = 0xFF; *p++ = 0xDA; p = put16(p, 12); *p++ = 3;
    *p++ = 1; *p++ = 0x00;
    *p++ = 2; *p++ = 0x11;
    *p++ = 3; *p++ = 0x11;
    *p++ = 0; *p++ = 63; *p++ = 0;
    return p;
}

uint32_t fj_max_size(int w, int h) {
    // absolute worst case (pure noise at q100 with 0xFF stuffing) ~ 6.5 bytes/pixel for 4:2:2
    return (uint32_t)w * h * 7 + 4096;
}

// ---------------------------------------------------------------- static-block cache
#include <stdlib.h>

#define FJ_SLOT_W   16                 // captured bits per block: up to 16 words (512 bits)
#define FJ_INVALID  0xFFFF             // slot not captured yet
#define FJ_UNCACHED 0xFFFE             // block codes too long for a slot: re-code from the reference

typedef struct {
    u16 nbits;                         // captured bit count (DC category-0 prefix included)
    s16 dc;                            // quantised DC of the block
    u32 bits[FJ_SLOT_W + 1];           // big-endian bit string in native words
} FJSlot;

struct FJCache {
    int w, h, sub422, quality;
    int mcus_x, mcus, nblk, mcu_h;
    u32* ref;                          // per macroblock: mcu_h rows x 8 YUYV words
    u8* valid;                         // per macroblock: ref holds real pixels
    FJSlot* slot;                      // per block
    int skipped, total;
    int checked;                       // macroblocks compared against a valid reference (last frame)
    int cooldown;                      // frames left with the cache switched off (moving scene)
    int probes;                        // consecutive probe frames that found nothing static
};

FJCache* fj_cache_create(void) { return (FJCache*)calloc(1, sizeof(FJCache)); }

static void cache_release(FJCache* c) {
    free(c->ref); free(c->valid); free(c->slot);
    c->ref = NULL; c->valid = NULL; c->slot = NULL; c->w = c->h = 0;
}

void fj_cache_destroy(FJCache* c) { if (c) { cache_release(c); free(c); } }

void fj_cache_stats(const FJCache* c, int* skipped, int* total) {
    *skipped = c ? c->skipped : 0;
    *total = c ? c->total : 0;
}

// (Re)shape the cache for this frame format. Returns 0 if it can't be used.
static int cache_prepare(FJCache* c, int w, int h, int sub422, int quality) {
    if (c->ref && c->w == w && c->h == h && c->sub422 == sub422) {
        if (c->quality != quality) {               // pixels stay valid, coded bits don't
            for (int i = 0; i < c->mcus * c->nblk; i++) c->slot[i].nbits = FJ_INVALID;
            c->quality = quality;
        }
        return 1;
    }
    cache_release(c);
    int mcu_h = sub422 ? 8 : 16;
    int mx = (w + 15) / 16, my = (h + mcu_h - 1) / mcu_h;
    c->mcus_x = mx; c->mcus = mx * my; c->mcu_h = mcu_h; c->nblk = sub422 ? 4 : 6;
    c->ref = (u32*)malloc((size_t)c->mcus * mcu_h * 8 * 4);
    c->valid = (u8*)calloc((size_t)c->mcus, 1);
    c->slot = (FJSlot*)malloc((size_t)c->mcus * c->nblk * sizeof(FJSlot));
    if (!c->ref || !c->valid || !c->slot) { cache_release(c); return 0; }
    c->w = w; c->h = h; c->sub422 = sub422; c->quality = quality;
    return 1;
}

// Is the macroblock still within `t` of its reference, per 8-pixel row segment?
// usad8 = |a-b| summed over 4 bytes in one instruction. Exits on the first change.
static int mcu_same(const u32* const* R, int wx, int mh, const u32* ref, u32 t) {
    for (int r = 0; r < mh; r++, ref += 8) {
        const u32* s = R[r] + wx;
        u32 a = usad8(s[0], ref[0]); a = usada8(s[1], ref[1], a);
        a = usada8(s[2], ref[2], a); a = usada8(s[3], ref[3], a);
        u32 b = usad8(s[4], ref[4]); b = usada8(s[5], ref[5], b);
        b = usada8(s[6], ref[6], b); b = usada8(s[7], ref[7], b);
        if (a > t || b > t) return 0;
    }
    return 1;
}

static void mcu_store(const u32* const* R, int wx, int mh, u32* ref) {
    for (int r = 0; r < mh; r++, ref += 8) memcpy(ref, R[r] + wx, 32);
}

// Append bits [pos, end) of a captured bit string.
static void replay_bits(BitW* w, const u32* b, int pos, int end) {
    while (pos < end) {
        int k = end - pos; if (k > 24) k = 24;
        int i = pos >> 5, off = pos & 31;
        u32 v = b[i] << off;
        if (off + k > 32) v |= b[i + 1] >> (32 - off);
        put_bits(w, v >> (32 - k), k);
        pos += k;
    }
}

static inline void put_dc(BitW* w, const HuffTable* dct, s32 dc, int* dc_prev) {
    s32 diff = dc - *dc_prev; *dc_prev = dc;
    int nb = nbits(diff);
    u32 vb = diff < 0 ? (u32)(diff - 1) & ((1u << nb) - 1) : (u32)diff;
    u32 dcs = dct->cs[nb];
    put_bits(w, ((dcs >> 16) << nb) | vb, (dcs & 0xFFFF) + nb);
}

// All DCT blocks of a macroblock from row pointers (Y0 Y1 [Y2 Y3] U V).
static void dct_mcu(u32 (*b)[32], const u32* const* R, int wx, int sub422) {
    if (sub422) {
        fdct_y(b[0], R, wx); fdct_y(b[1], R, wx + 4);
        fdct_uv(b[2], b[3], R, NULL, wx);
    } else {
        const u32* t[8]; const u32* u[8];
        for (int r = 0; r < 8; r++) { t[r] = R[2 * r]; u[r] = R[2 * r + 1]; }
        fdct_y(b[0], R, wx); fdct_y(b[1], R, wx + 4);
        fdct_y(b[2], R + 8, wx); fdct_y(b[3], R + 8, wx + 4);
        fdct_uv(b[4], b[5], t, u, wx);
    }
}

// Unchanged macroblock: replay captured bits; capture first if needed (from the reference
// pixels, so the bits always match the reference the comparison uses).
static void __attribute__((noinline)) encode_mcu_cached(BitW* bw, const FJQuant* q, FJCache* c, int m,
                                                        int* dcy, int* dcu, int* dcv) {
    const int nb = c->nblk, ny = nb - 2, mh = c->mcu_h;
    FJSlot* sl = c->slot + (size_t)m * nb;
    u32 blocks[6][32] __attribute__((aligned(8)));
    int have = 0;
    for (int j = 0; j < nb; j++) {
        const int t = j < ny ? 0 : 1;
        int* dcp = j < ny ? dcy : (j == ny ? dcu : dcv);
        FJSlot* s = &sl[j];
        if (s->nbits < FJ_UNCACHED) {
            put_dc(bw, &h_dc[t], s->dc, dcp);
            replay_bits(bw, s->bits, h_dc[t].cs[0] & 0xFFFF, s->nbits);
            continue;
        }
        if (!have) {
            const u32* R[16];
            const u32* ref = c->ref + (size_t)m * mh * 8;
            for (int r = 0; r < mh; r++) R[r] = ref + r * 8;
            dct_mcu(blocks, R, 0, c->sub422);
            have = 1;
        }
        if (s->nbits == FJ_INVALID) {
            // capture: DC coded as "no change" (a fixed prefix the replay skips)
            const s16a* cf = (const s16a*)blocks[j];
            s32 dc = (cf[0] * (s32)q->recip[t][0] + 0x8000) >> 16;
            int tmp = dc;
            BitW cw = { (u8*)s->bits, (u8*)(s->bits + FJ_SLOT_W), 0, 0, 0, 1 };
            encode_block(&cw, blocks[j], q->recip[t], q->dz[t], &h_dc[t], &h_ac[t], h_rv[t], &tmp);
            if (!cw.overflow) {
                int nw = (int)((u32*)cw.p - s->bits);
                s->bits[nw] = cw.used ? cw.acc << (32 - cw.used) : 0;
                s->nbits = (u16)(nw * 32 + cw.used);
                s->dc = (s16)dc;
                put_dc(bw, &h_dc[t], dc, dcp);
                replay_bits(bw, s->bits, h_dc[t].cs[0] & 0xFFFF, s->nbits);
                continue;
            }
            s->nbits = FJ_UNCACHED;
        }
        encode_block(bw, blocks[j], q->recip[t], q->dz[t], &h_dc[t], &h_ac[t], h_rv[t], dcp);
    }
}

// ---------------------------------------------------------------- frame
static uint32_t encode_frame(const FJQuant* q, const uint8_t* yuyv, int w, int h, int stride_bytes,
                             int sub422, uint8_t* out, uint32_t out_cap, FJCache* cache, int thresh) {
    init_tables();
    if (out_cap < 1024) return 0;
    u8* p = write_headers(out, q, w, h, sub422);

    BitW bw = { p, out + out_cap - 2, 0, 0, 0, 0 };
    int dcy = 0, dcu = 0, dcv = 0;
    u32 blk[32] __attribute__((aligned(8)));
    u32 bu[32] __attribute__((aligned(8)));
    u32 bv[32] __attribute__((aligned(8)));

    const int mcu_h = sub422 ? 8 : 16;
    const int words = w / 2;                       // YUYV words per row
    const u32* rows[16];
    const u32* top[8];
    const u32* bot[8];

    if (cache && !cache_prepare(cache, w, h, sub422, q->quality)) cache = NULL;
    // Moving scene: comparing and copying reference pixels costs memory bandwidth for
    // nothing, so after a frame that reused < 5% of the blocks, encode plainly for a while
    // and probe again later (refs go stale meanwhile, so they're invalidated).
    if (cache && cache->cooldown > 0) {
        cache->cooldown--;
        cache->skipped = 0; cache->total = cache->mcus;
        cache = NULL;
    }
    if (cache) { cache->skipped = 0; cache->total = cache->mcus; cache->checked = 0; }
    FJCache* const cache_used = cache;
    const u32 t = thresh < 0 ? 0 : (u32)thresh;
    int m = 0;                                     // macroblock index (cache)

    for (int my = 0; my < h; my += mcu_h) {
        for (int r = 0; r < mcu_h; r++) {          // replicate the last row past the bottom edge
            int y = my + r < h ? my + r : h - 1;
            rows[r] = (const u32*)(yuyv + (size_t)y * stride_bytes);
        }
        if (!sub422) for (int r = 0; r < 8; r++) { top[r] = rows[2 * r]; bot[r] = rows[2 * r + 1]; }

        for (int mx = 0; mx < w; mx += 16, m++) {
            int wx = mx / 2;
            // Prefetch the next MCU's 32-byte row segments while this one is encoded:
            // the camera frame lives in slow main memory and every miss stalls the CPU.
            if (wx + 16 <= words)
                for (int r = 0; r < mcu_h; r++) __builtin_prefetch(rows[r] + wx + 8);
            // right edge: if the MCU would read past the row, use a padded copy
            if (wx + 8 > words) {
                u32 pad[16][8];                    // on the stack: several encoder threads
                const u32* prow[16];
                for (int r = 0; r < mcu_h; r++) {
                    for (int k = 0; k < 8; k++) pad[r][k] = rows[r][wx + k < words ? wx + k : words - 1];
                    prow[r] = pad[r];
                }
                if (cache) {
                    u32* ref = cache->ref + (size_t)m * mcu_h * 8;
                    if (cache->valid[m] && (cache->checked++, 1) && mcu_same(prow, 0, mcu_h, ref, t)) {
                        cache->skipped++;
                        encode_mcu_cached(&bw, q, cache, m, &dcy, &dcu, &dcv);
                        continue;
                    }
                    mcu_store(prow, 0, mcu_h, ref);
                    cache->valid[m] = 1;
                    for (int j = 0; j < cache->nblk; j++) cache->slot[(size_t)m * cache->nblk + j].nbits = FJ_INVALID;
                }
                const u32* pt[8]; const u32* pb[8];
                if (!sub422) for (int r = 0; r < 8; r++) { pt[r] = prow[2 * r]; pb[r] = prow[2 * r + 1]; }
                for (int by = 0; by < mcu_h; by += 8) for (int bx = 0; bx < 2; bx++) {
                    fdct_y(blk, prow + by, bx * 4); encode_block(&bw, blk, q->recip[0], q->dz[0], &h_dc[0], &h_ac[0], h_rv[0], &dcy);
                }
                if (sub422) fdct_uv(bu, bv, prow, NULL, 0); else fdct_uv(bu, bv, pt, pb, 0);
                encode_block(&bw, bu, q->recip[1], q->dz[1], &h_dc[1], &h_ac[1], h_rv[1], &dcu);
                encode_block(&bw, bv, q->recip[1], q->dz[1], &h_dc[1], &h_ac[1], h_rv[1], &dcv);
                continue;
            }
            if (cache) {
                u32* ref = cache->ref + (size_t)m * mcu_h * 8;
                if (cache->valid[m] && (cache->checked++, 1) && mcu_same(rows, wx, mcu_h, ref, t)) {
                    cache->skipped++;
                    encode_mcu_cached(&bw, q, cache, m, &dcy, &dcu, &dcv);
                    if (bw.overflow) return 0;
                    continue;
                }
                mcu_store(rows, wx, mcu_h, ref);
                cache->valid[m] = 1;
                for (int j = 0; j < cache->nblk; j++) cache->slot[(size_t)m * cache->nblk + j].nbits = FJ_INVALID;
            }
            for (int by = 0; by < mcu_h; by += 8) {
                fdct_y(blk, rows + by, wx);     encode_block(&bw, blk, q->recip[0], q->dz[0], &h_dc[0], &h_ac[0], h_rv[0], &dcy);
                fdct_y(blk, rows + by, wx + 4); encode_block(&bw, blk, q->recip[0], q->dz[0], &h_dc[0], &h_ac[0], h_rv[0], &dcy);
            }
            if (sub422) fdct_uv(bu, bv, rows, NULL, wx); else fdct_uv(bu, bv, top, bot, wx);
            encode_block(&bw, bu, q->recip[1], q->dz[1], &h_dc[1], &h_ac[1], h_rv[1], &dcu);
            encode_block(&bw, bv, q->recip[1], q->dz[1], &h_dc[1], &h_ac[1], h_rv[1], &dcv);
            if (bw.overflow) return 0;
        }
    }
    flush_bits(&bw);
    if (cache_used) {
        // judge only frames that had references to compare with (not the first one)
        if (cache_used->checked * 2 >= cache_used->total && cache_used->skipped * 20 < cache_used->checked) {
            // first miss: back off 8 frames, then 16, then 30 while the scene keeps moving
            int n = cache_used->probes < 1 ? 8 : cache_used->probes < 2 ? 16 : 30;
            cache_used->probes++;
            cache_used->cooldown = n;
            memset(cache_used->valid, 0, (size_t)cache_used->mcus);
        } else if (cache_used->checked * 2 >= cache_used->total) {
            cache_used->probes = 0;
        }
    }
    if (bw.overflow) return 0;
    p = bw.p;
    *p++ = 0xFF; *p++ = 0xD9;
    return (uint32_t)(p - out);
}

uint32_t fj_encode_yuyv(const FJQuant* q, const uint8_t* yuyv, int w, int h, int stride_bytes,
                        int sub422, uint8_t* out, uint32_t out_cap) {
    return encode_frame(q, yuyv, w, h, stride_bytes, sub422, out, out_cap, NULL, 0);
}

uint32_t fj_encode_yuyv_cached(const FJQuant* q, const uint8_t* yuyv, int w, int h, int stride_bytes,
                               int sub422, uint8_t* out, uint32_t out_cap, FJCache* c, int thresh) {
    return encode_frame(q, yuyv, w, h, stride_bytes, sub422, out, out_cap, c, thresh);
}

void fj_half_yuyv(const uint8_t* src, int w, int h, uint8_t* dst) {
    const u32* s = (const u32*)src;
    u32* d = (u32*)dst;
    const int sw = w / 2;              // words per source row
    const int ow = w / 4;              // words per dest row (w/2 pixels)
    for (int y = 0; y < h / 2; y++) {
        const u32* r0 = s + (2 * y) * sw;
        const u32* r1 = r0 + sw;
        for (int x = 0; x < ow; x++) {
            u32 a = uhadd8(r0[2 * x], r1[2 * x]);           // vertical average, bytewise
            u32 b = uhadd8(r0[2 * x + 1], r1[2 * x + 1]);
            u32 ya = (((a & 0xFF) + ((a >> 16) & 0xFF) + 1) >> 1);
            u32 yb = (((b & 0xFF) + ((b >> 16) & 0xFF) + 1) >> 1);
            u32 uv = uhadd8(a, b) & 0xFF00FF00u;            // average U and V of both words
            d[y * ow + x] = ya | (yb << 16) | uv;
        }
    }
}
