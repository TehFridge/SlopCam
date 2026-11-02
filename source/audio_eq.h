#ifndef AUDIO_EQ_H
#define AUDIO_EQ_H

#include <stdint.h>
#include <stdbool.h>

// Microphone EQ for recorded audio (mono 16-bit PCM).
// Chain: low cut (2nd order high-pass) -> 5 bands (100 Hz low shelf, 300 Hz / 1 kHz /
// 3 kHz peaks, 8 kHz high shelf) -> boost -> optional soft limiter.
// Same band layout as FridgeMic's EQ; frequencies are clamped below Nyquist so every
// mic sample rate works.

#define MICEQ_BANDS 5

typedef struct {
    bool  enabled;
    int   gain_db[MICEQ_BANDS];   // -12..+12
    int   lowcut;                 // 0 off, 1 = 80 Hz, 2 = 120 Hz, 3 = 200 Hz
    int   boost_db;               // 0..24
    bool  limiter;
} MicEqParams;

typedef struct { float b0, b1, b2, a1, a2, z1, z2; } Biquad;

typedef struct {
    Biquad st[MICEQ_BANDS + 1];
    int    nst;                   // active stages (0 dB bands are skipped)
    float  gain;                  // linear boost
    bool   limiter;
    bool   active;                // false = bypass (nothing to do)
} MicEq;

void miceq_design(MicEq* eq, float sample_rate, const MicEqParams* p);
void miceq_reset(MicEq* eq);                         // clear filter memory
void miceq_process(MicEq* eq, int16_t* pcm, int n);  // in place

#endif
