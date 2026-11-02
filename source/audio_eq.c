#include "audio_eq.h"
#include <math.h>
#include <string.h>

static const float band_hz[MICEQ_BANDS] = { 100.0f, 300.0f, 1000.0f, 3000.0f, 8000.0f };
static const float lowcut_hz[4] = { 0.0f, 80.0f, 120.0f, 200.0f };

// RBJ audio-EQ-cookbook biquads, normalised by a0.
static void set(Biquad* q, double b0, double b1, double b2, double a0, double a1, double a2) {
    q->b0 = (float)(b0 / a0); q->b1 = (float)(b1 / a0); q->b2 = (float)(b2 / a0);
    q->a1 = (float)(a1 / a0); q->a2 = (float)(a2 / a0);
    q->z1 = q->z2 = 0.0f;
}

static void design_peak(Biquad* q, double fs, double f, double Q, double db) {
    double A = pow(10.0, db / 40.0), w = 2.0 * M_PI * f / fs, al = sin(w) / (2.0 * Q), c = cos(w);
    set(q, 1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A);
}

static void design_shelf(Biquad* q, double fs, double f, double db, int high) {
    double A = pow(10.0, db / 40.0), w = 2.0 * M_PI * f / fs, c = cos(w);
    double al = sin(w) / 2.0 * sqrt(2.0), sA = 2.0 * sqrt(A) * al;   // S = 1 (Q = 0.707)
    if (high)
        set(q, A * ((A + 1) + (A - 1) * c + sA), -2 * A * ((A - 1) + (A + 1) * c), A * ((A + 1) + (A - 1) * c - sA),
               (A + 1) - (A - 1) * c + sA, 2 * ((A - 1) - (A + 1) * c), (A + 1) - (A - 1) * c - sA);
    else
        set(q, A * ((A + 1) - (A - 1) * c + sA), 2 * A * ((A - 1) - (A + 1) * c), A * ((A + 1) - (A - 1) * c - sA),
               (A + 1) + (A - 1) * c + sA, -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - sA);
}

static void design_highpass(Biquad* q, double fs, double f) {
    double w = 2.0 * M_PI * f / fs, al = sin(w) / (2.0 * 0.7071), c = cos(w);
    set(q, (1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
}

void miceq_design(MicEq* eq, float fs, const MicEqParams* p) {
    memset(eq, 0, sizeof(*eq));
    eq->gain = 1.0f;
    if (!p->enabled || fs < 4000.0f) return;
    double nyq_safe = fs * 0.45;
    if (p->lowcut > 0 && p->lowcut < 4) design_highpass(&eq->st[eq->nst++], fs, lowcut_hz[p->lowcut]);
    for (int i = 0; i < MICEQ_BANDS; i++) {
        if (p->gain_db[i] == 0) continue;                     // 0 dB = identity, skip the work
        double f = band_hz[i] < nyq_safe ? band_hz[i] : nyq_safe;
        if (i == 0)                    design_shelf(&eq->st[eq->nst++], fs, f, p->gain_db[i], 0);
        else if (i == MICEQ_BANDS - 1) design_shelf(&eq->st[eq->nst++], fs, f, p->gain_db[i], 1);
        else                           design_peak(&eq->st[eq->nst++], fs, f, 1.0, p->gain_db[i]);
    }
    eq->gain = (float)pow(10.0, p->boost_db / 20.0);
    eq->limiter = p->limiter;
    eq->active = eq->nst > 0 || p->boost_db != 0 || p->limiter;
}

void miceq_reset(MicEq* eq) {
    for (int i = 0; i < eq->nst; i++) eq->st[i].z1 = eq->st[i].z2 = 0.0f;
}

void miceq_process(MicEq* eq, int16_t* pcm, int n) {
    if (!eq->active) return;
    const float g = eq->gain;
    const float knee = 0.7f * 32767.0f, room = 32767.0f - knee;
    for (int k = 0; k < n; k++) {
        float x = (float)pcm[k];
        for (int i = 0; i < eq->nst; i++) {                   // transposed direct form II
            Biquad* q = &eq->st[i];
            float y = q->b0 * x + q->z1;
            q->z1 = q->b1 * x - q->a1 * y + q->z2;
            q->z2 = q->b2 * x - q->a2 * y;
            x = y;
        }
        x *= g;
        if (eq->limiter) {                                    // soft knee: approaches full scale, never hard-clips
            float a = x < 0 ? -x : x;
            if (a > knee) {
                float e = a - knee;
                a = knee + e / (1.0f + e / room);
                x = x < 0 ? -a : a;
            }
        }
        if (x > 32767.0f) x = 32767.0f;
        else if (x < -32768.0f) x = -32768.0f;
        pcm[k] = (int16_t)(x < 0 ? x - 0.5f : x + 0.5f);
    }
    // flush tiny filter state so silence never drops into slow denormal arithmetic
    for (int i = 0; i < eq->nst; i++) {
        if (fabsf(eq->st[i].z1) < 1e-15f) eq->st[i].z1 = 0.0f;
        if (fabsf(eq->st[i].z2) < 1e-15f) eq->st[i].z2 = 0.0f;
    }
}
