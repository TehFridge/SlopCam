// gallery.c - photo viewer + MJPEG/PCM AVI player for SlopCam's own files.
//
// Top screen: the picture. Bottom screen: file list, info, controls.
// Video: frames are decoded with libjpeg-turbo straight at screen size (its DCT scaling,
// e.g. VGA -> 320x240), which is ~4x less work than a full decode. The wall clock decides
// which frame is due; if decoding falls behind, frames in between are skipped (MJPEG frames
// are independent), so the picture never lags the sound. Sound plays through NDSP
// (needs the DSP firmware dump, sdmc:/3ds/dspfirm.cdc; without it videos play silently).
#include "gallery.h"
#include "avi_read.h"
#include "fastjpeg_dec.h"
#include <3ds.h>
#include <turbojpeg.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#define MAX_FILES   2000
#define LIST_ROWS   24
#define AB_COUNT    4
#define AB_SAMPLES  4096            // per wave buffer (~125 ms at 32 kHz)

typedef struct { char name[64]; u32 size; u8 video; } Entry;

static Entry* files = NULL;
static int nfiles = 0;
static char g_folder[128];

static tjhandle tj = NULL;
static u8* jbuf = NULL; static u32 jcap = 0;      // compressed data
static u8* rgb = NULL;  static u32 rgbcap = 0;    // decoded picture
static int clear_left = 2;                        // framebuffers to clear (both of the double buffer)
static bool drew = false;                         // a new picture is in the back buffer: swap it in
static int photo_w = 0, photo_h = 0;              // real size of the last decoded JPEG

// ---------------------------------------------------------------- files
static int cmp_desc(const void* a, const void* b) { return strcmp(((const Entry*)b)->name, ((const Entry*)a)->name); }

static void scan_files(void) {
    nfiles = 0;
    DIR* d = opendir(g_folder);
    if (!d) return;
    struct dirent* e;
    while ((e = readdir(d)) && nfiles < MAX_FILES) {
        const char* n = e->d_name;
        size_t l = strlen(n);
        if (l < 5 || l >= sizeof(files[0].name)) continue;
        bool jpg = !strcasecmp(n + l - 4, ".jpg"), avi = !strcasecmp(n + l - 4, ".avi");
        if (!jpg && !avi) continue;
        Entry* f = &files[nfiles];
        snprintf(f->name, sizeof(f->name), "%s", n);
        f->video = avi;
        char path[200];
        snprintf(path, sizeof(path), "%s/%s", g_folder, n);
        struct stat st;
        f->size = stat(path, &st) == 0 ? (u32)st.st_size : 0;   // (no need to open the file)
        nfiles++;
    }
    closedir(d);
    qsort(files, nfiles, sizeof(Entry), cmp_desc);    // names carry the date: newest first
}

static bool grow(u8** p, u32* cap, u32 need) {
    if (need <= *cap) return true;
    u8* n = realloc(*p, need);
    if (!n) return false;
    *p = n; *cap = need;
    return true;
}

// ---------------------------------------------------------------- drawing
static void top_clear_now(void) {
    for (int i = 0; i < 2; i++) {
        u16* fb = (u16*)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
        memset(fb, 0, 400 * 240 * 2);
        gfxFlushBuffers(); gfxSwapBuffers(); gspWaitForVBlank();
    }
}

// RGB w x h -> top framebuffer, centred, integer upscale s. Column-ordered framebuffer:
// screen (x, y) lives at fb[x * 240 + 239 - y]. 16x16 tiles keep both sides in cache.
static void blit_rgb(const u8* src, int w, int h, int s) {
    u16* fb = (u16*)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
    if (clear_left > 0) { clear_left--; memset(fb, 0, 400 * 240 * 2); }
    const int ow = w * s, oh = h * s;
    const int x0 = (400 - ow) / 2, y0 = (240 - oh) / 2;
    if (s == 1) {                                     // the normal case: no zoom, no divisions
        for (int ty = 0; ty < h; ty += 16) {
            const int ye = ty + 16 < h ? ty + 16 : h;
            for (int x = 0; x < w; x++) {
                u16* col = fb + (x0 + x) * 240 + 239 - y0 - ty;
                const u8* p = src + ((size_t)ty * w + x) * 3;
                const size_t stride = (size_t)w * 3;
                for (int y = ty; y < ye; y++, p += stride)
                    *col-- = (u16)(((p[0] & 0xF8) << 8) | ((p[1] & 0xFC) << 3) | (p[2] >> 3));
            }
        }
        return;
    }
    for (int ty = 0; ty < oh; ty += 16)
        for (int x = 0; x < ow; x++) {
            u16* col = fb + (x0 + x) * 240 + 239 - y0;
            const u8* row = src + (x / s) * 3;
            const int ye = ty + 16 < oh ? ty + 16 : oh;
            for (int y = ty; y < ye; y++) {
                const u8* p = row + (size_t)(y / s) * w * 3;
                col[-y] = (u16)(((p[0] & 0xF8) << 8) | ((p[1] & 0xFC) << 3) | (p[2] >> 3));
            }
        }
}

// Decode a JPEG at the largest libjpeg-turbo scale that fits the top screen
// (into *out, w/h/zoom returned). Thread-safe with its own tjhandle.
static bool decode_fit(tjhandle t, const u8* data, u32 size, bool fast, u8** out, u32* cap,
                       int* ow, int* oh, int* oz, int* realw, int* realh) {
    int w, h, sub, cs;
    if (tjDecompressHeader3(t, data, size, &w, &h, &sub, &cs)) return false;
    *realw = w; *realh = h;
    int n = 0;
    tjscalingfactor* sf = tjGetScalingFactors(&n);
    int bw = 0, bh = 0;
    for (int i = 0; i < n; i++) {
        int sw = TJSCALED(w, sf[i]), sh = TJSCALED(h, sf[i]);
        if (sw <= 400 && sh <= 240 && sw * sh > bw * bh) { bw = sw; bh = sh; }
    }
    if (!bw) {                                        // bigger than 8x the screen: smallest scale
        for (int i = 0; i < n; i++) {
            int sw = TJSCALED(w, sf[i]), sh = TJSCALED(h, sf[i]);
            if (!bw || sw * sh < bw * bh) { bw = sw; bh = sh; }
        }
    }
    if (bw > 400 || bh > 240) return false;
    if (!grow(out, cap, (u32)bw * bh * 3)) return false;
    int flags = fast ? (TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) : 0;
    if (tjDecompress2(t, data, size, *out, bw, 0, bh, TJPF_RGB, flags)) return false;
    int s = 1;
    while ((s + 1) * bw <= 400 && (s + 1) * bh <= 240) s++;  // small pictures: integer zoom
    *ow = bw; *oh = bh; *oz = s;
    return true;
}

static int last_w = 0, last_h = 0;
static void show_rgb(const u8* p, int w, int h, int s) {
    if (w * s != last_w || h * s != last_h) { clear_left = 2; last_w = w * s; last_h = h * s; }
    blit_rgb(p, w, h, s);
    drew = true;
}

// Decode on the main thread and show (photos, stills, scrubbing).
static bool show_jpeg(const u8* data, u32 size, bool fast) {
    int w, h, s;
    if (!decode_fit(tj, data, size, fast, &rgb, &rgbcap, &w, &h, &s, &photo_w, &photo_h)) return false;
    show_rgb(rgb, w, h, s);
    return true;
}

// Whole file into jbuf with one read straight from the file service (stdio on the 3DS SD
// card is very slow). Returns the size, 0 on failure.
static u32 read_whole(const char* path) {
#ifdef __3DS__
    const char* p = strncmp(path, "sdmc:", 5) == 0 ? path + 5 : path;
    Handle fh = 0;
    if (R_SUCCEEDED(FSUSER_OpenFileDirectly(&fh, ARCHIVE_SDMC, fsMakePath(PATH_EMPTY, ""),
                                            fsMakePath(PATH_ASCII, p), FS_OPEN_READ, 0))) {
        u64 sz = 0; u32 got = 0;
        bool ok = R_SUCCEEDED(FSFILE_GetSize(fh, &sz)) && sz > 0 && sz < 32u * 1024 * 1024 &&
                  grow(&jbuf, &jcap, (u32)sz) && R_SUCCEEDED(FSFILE_Read(fh, &got, 0, jbuf, (u32)sz)) && got == sz;
        FSFILE_Close(fh);
        if (ok) return (u32)sz;
    }
#endif
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    bool ok = n > 0 && n < 32 * 1024 * 1024 && grow(&jbuf, &jcap, (u32)n) && fread(jbuf, 1, (size_t)n, f) == (size_t)n;
    fclose(f);
    return ok ? (u32)n : 0;
}

static bool show_photo(const char* path) {
    u32 n = read_whole(path);
    return n && show_jpeg(jbuf, n, false);
}

// ---------------------------------------------------------------- video decoder threads
// Two decoders work on alternate frames: one on core 1, one in core 0's spare time.
// The main thread reads each frame's JPEG from the file, hands it over, and shows the newest
// finished frame whose time has come. A generation number discards work after a seek/stop.
enum { D_IDLE, D_BUSY, D_DONE };
typedef struct {
    Thread th;
    LightEvent go;
    tjhandle t;
    volatile int state;
    u32 frame, gen;
    u8* jpg; u32 jcap, jsize;
    u8* out; u32 ocap;
    int w, h, s, rw, rh;
    bool ok;
    u32 ms;                 // how long the last decode took
    FJDec* fj;              // our decoder (fastjpeg_dec.c): straight to a screen image
    u16* screen;            // 400x240 in framebuffer layout, borders kept black
    int sw, sh;             // picture size in `screen` (0 = borders not cleared yet)
    bool is_screen;         // result is in `screen` (else RGB in `out`, from libjpeg-turbo)
    bool c1;                // runs on core 1 (the system core)
    volatile bool running;  // decoding right now
} Dec;
static Dec dec[2];
static int ndec = 0;
static volatile bool dec_quit = false;
static u32 dec_gen = 1;

// Core 1 is the Old 3DS's system core: the system services, including the file service
// that reads the SD card, run there too. A decoder that keeps core 1 busy can starve it
// (on a real console the reads measured 0.1 MB/s). So the reader can ask for core 1 while it
// reads: the core-1 decoder then starts no new picture until the read is done. That costs
// decode time, so the reader measures block reads both ways and keeps the faster mode
// (re-checking the other one every few blocks).
static volatile bool io_quiet = false;
static double io_probe[2];         // ms for a 64 KB probe read: [0] = core 1 decoding, [1] = core 1 left free
static double io_ms_mb = 0;        // ms per MB of the normal 1 MB block reads
static u32 io_blocks = 0, io_probes = 0;
#define PROBE (32 * 1024)

static void dec_thread(void* arg) {
    Dec* d = (Dec*)arg;
    while (!dec_quit) {
        LightEvent_Wait(&d->go);
        if (dec_quit) break;
        if (d->state != D_BUSY) continue;
        while (d->c1 && io_quiet && !dec_quit) svcSleepThread(2000000);   // SD read running
        d->running = true;
        __dmb();
        u64 t = osGetTime();
        d->is_screen = false;
        int w = 0, h = 0;
        if (d->fj && d->screen && fjd_decode_fb(d->fj, d->jpg, d->jsize, d->screen, &w, &h)) {
            if (w != d->sw || h != d->sh) {          // new picture size: clear the borders once
                memset(d->screen, 0, 400 * 240 * 2);
                d->sw = w; d->sh = h;
                fjd_decode_fb(d->fj, d->jpg, d->jsize, d->screen, &w, &h);
            }
            d->ok = d->is_screen = true;
        } else {
            // something our decoder doesn't handle: libjpeg-turbo
            d->ok = decode_fit(d->t, d->jpg, d->jsize, true, &d->out, &d->ocap, &d->w, &d->h, &d->s, &d->rw, &d->rh);
        }
        d->ms = (u32)(osGetTime() - t);
        d->running = false;
        __dmb();
        d->state = D_DONE;
    }
}

static void dec_start(void) {
    dec_quit = false;
    ndec = 0;
    s32 prio = 0x30;
    svcGetThreadPriority(&prio, CUR_THREAD_HANDLE);
    const int cores[2] = { 1, 0 };
    for (int i = 0; i < 2; i++) {
        Dec* d = &dec[ndec];
        memset(d, 0, sizeof(*d));
        d->t = tjInitDecompress();
        if (!d->t) break;
        d->fj = fjd_create();
        d->screen = malloc(400 * 240 * 2);
        LightEvent_Init(&d->go, RESET_ONESHOT);
        // core 1: the lowest priority, so the system services there (SD card!) always go first;
        // core 0: just below the main thread, so input/UI always win
        d->c1 = cores[i] == 1;
        d->th = threadCreate(dec_thread, d, 0x8000, i == 0 ? 0x3F : prio + 1, cores[i], false);
        if (!d->th && i == 0) { d->c1 = false; d->th = threadCreate(dec_thread, d, 0x8000, prio + 1, -2, false); }
        if (!d->th) { tjDestroy(d->t); fjd_destroy(d->fj); free(d->screen); break; }
        ndec++;
    }
}

static void dec_stop(void) {
    dec_quit = true;
    for (int i = 0; i < ndec; i++) LightEvent_Signal(&dec[i].go);
    for (int i = 0; i < ndec; i++) {
        threadJoin(dec[i].th, U64_MAX); threadFree(dec[i].th);
        tjDestroy(dec[i].t);
        fjd_destroy(dec[i].fj); free(dec[i].screen);
        free(dec[i].jpg); free(dec[i].out);
    }
    ndec = 0;
}

// Forget queued/finished work (seek, stop, other file). Busy ones finish and are dropped.
static void dec_flush(void) {
    dec_gen++;
    for (int i = 0; i < ndec; i++) if (dec[i].state == D_DONE) dec[i].state = D_IDLE;
}

// ---------------------------------------------------------------- reader thread
// The only code that touches the SD card while a video plays. It walks the file in order
// (1 MB blocks = few, large SD reads) and splits it: audio into an 8 s ring, video
// frames into a small queue for the decoders. Sound has priority: if the video queue is
// full while the sound is running low, video frames are dropped instead of waiting.
// Frames that can no longer be shown in time aren't even read.
#define VQ_MAX    16                  // ~0.5 s of video read ahead (so the sound can be too)
#define VQ_BYTES  (4u << 20)
// The ring is big on purpose: in a VGA recording the SD writer can be a few seconds behind
// the camera (frames wait in the record queue while the mic audio is written right away),
// so in the file the sound for a moment comes up to ~3-4 s BEFORE its pictures. Walking the
// file in order, the reader has to hold all that sound before it reaches the pictures; with
// a 2 s ring it stalled, every picture arrived late and was skipped: frozen picture,
// sound still playing. 8 s covers the worst the recorder can do (64 queued frames).
#define AR_CAP    (262144u)           // ring samples (8 s at 32 kHz, 512 KB)
typedef struct { u32 frame, size, cap; u8* data; } VFrame;
static VFrame vq[VQ_MAX];
static int vq_r = 0, vq_n = 0;
static u32 vq_bytes = 0;
static LightLock vq_lock;
static s16* aring = NULL;
static volatile u64 ar_w = 0, ar_r = 0;           // samples written / taken, ever (ring index = % AR_CAP)
static Thread rd_th = NULL;
static LightEvent rd_go;
static volatile bool rd_quit = false, rd_busy = false, rd_eof = false;
static volatile u32 rd_gen = 0;
static AviReader* rd_avi = NULL;                  // the reader's own handle (own FILE, big buffer)
static u32 rd_from_frame = 0;
static u64 rd_from_sample = 0;
static bool rd_audio = false;                     // read the sound too (only if it's being played)
static u32 rd_rate = 1;
static volatile u32 rd_late_before = 0;           // set by the player: earlier frames would be late
static volatile u32 st_rd_bytes = 0, st_rd_ms = 0, st_vdrop = 0;   // statistics
static volatile u32 a_queued = 0;                 // samples sitting in DSP buffers (set by the player)
static volatile u32 a_low = 0;                    // sound reserve below which video gets dropped
static s16 rd_tmp[8192];

// Every SD read on the 3DS costs a lot no matter how small (measured: ~0.1 MB/s with one
// read per chunk). So the reader pulls the file in 1 MB blocks - one file-service call
// each - and takes the sound and frames out of memory. A block starts where the next needed
// chunk starts, so a skipped frame that isn't inside the current block is never read.
#define BLK (1024 * 1024)
static u8* blk = NULL;
static long blk_pos = -1;
static u32 blk_len = 0;

static bool io_has_c1(void) {
    for (int k = 0; k < ndec; k++) if (dec[k].c1) return true;
    return false;
}

// Normal block reads: leave core 1 free unless reading with core 1 busy measured about as fast.
static int io_mode(void) {
    if (!io_has_c1()) return 0;
    if (io_probe[0] <= 0 || io_probe[1] <= 0) return 1;  // not measured yet: the safe mode
    return io_probe[1] < io_probe[0] * 0.7 ? 1 : 0;
}

static void io_quiet_on(void) {    // a running core-1 decode finishes (<~60 ms), no new one starts
    io_quiet = true;
    __dmb();
    for (int i = 0; i < 40; i++) {
        bool busy = false;
        for (int k = 0; k < ndec; k++) if (dec[k].c1 && dec[k].running && !dec_quit) busy = true;
        if (!busy) break;
        svcSleepThread(2000000);
    }
}

static bool blk_get(AviReader* r, u32 off, void* dst, u32 n) {
    if (blk && blk_pos >= 0 && (long)off >= blk_pos && (long)off + n <= blk_pos + (long)blk_len) {
        memcpy(dst, blk + (off - blk_pos), n);
        return true;
    }
    // A probe: a small read (32 KB) in one mode, to compare the two modes at the same size.
    // Only with some sound buffered, since with a starved file service even 32 KB can take a
    // moment. Then every 16th read, alternating modes, to follow changes (a mode that was
    // much slower is re-checked only every 64th read).
    int mode = io_mode();
    u32 len = BLK;
    if (blk && n <= PROBE && io_has_c1()) {
        double reserve_ms = rd_audio ? ((ar_w - ar_r) + a_queued) * 1000.0 / rd_rate : 1e9;
        int pm = io_probe[1] <= 0 ? 1 : io_probe[0] <= 0 ? 0 : (int)(io_probes & 1);
        bool due = io_probe[pm] <= 0 || io_blocks % 16 == 15;
        if (due && pm == 0 && io_probe[0] > 4 * io_probe[1] && io_blocks % 64 != 63) { due = false; io_probes++; }
        if (due && reserve_ms > 350) { mode = pm; len = PROBE; }
    }
    if (mode) io_quiet_on();
    u64 t = osGetTime();
    bool ok;
    if (!blk || n > BLK) {                                // bigger than a block: read it directly
        ok = avir_read_raw(r, off, dst, n) == n;
        st_rd_bytes += n;
    } else {
        blk_len = avir_read_raw(r, off, blk, len);
        blk_pos = off;
        st_rd_bytes += blk_len;
        ok = blk_len >= n;
        if (ok) memcpy(dst, blk, n);
    }
    u32 ms = (u32)(osGetTime() - t);
    st_rd_ms += ms;
    io_quiet = false;
    if (blk && n <= BLK) {
        io_blocks++;
        if (len == PROBE && blk_len == PROBE) {
            double v = ms + 0.5;
            io_probe[mode] = io_probe[mode] > 0 ? io_probe[mode] * 0.5 + v * 0.5 : v;
            io_probes++;
        } else if (blk_len >= BLK / 2) {
            double v = ms * (double)(1 << 20) / blk_len;   // ms per MB of normal block reads
            io_ms_mb = io_ms_mb > 0 ? io_ms_mb * 0.5 + v * 0.5 : v;
        }
    }
    return ok;
}

// Would reading this frame need a new block (i.e. real SD time)?
static bool blk_has(u32 off, u32 n) { return blk_pos >= 0 && (long)off >= blk_pos && (long)off + n <= blk_pos + (long)blk_len; }

static void rd_thread(void* arg) {
    (void)arg;
    while (!rd_quit) {
        LightEvent_Wait(&rd_go);
        if (rd_quit) break;
        const u32 gen = rd_gen;
        AviReader* r = rd_avi;
        if (!r) { rd_busy = false; continue; }
        const u32 nv = avir_frames(r), na = (rd_audio && avir_audio_rate(r)) ? avir_audio_chunks(r) : 0;
        u32 iv = rd_from_frame, ia = na ? avir_audio_chunk_at(r, rd_from_sample) : 0;
        const u64 s0 = rd_from_sample;
        const u32 rate = avir_audio_rate(r) ? avir_audio_rate(r) : 1;
        double kb_per_ms = 2.0;                          // SD speed (block reads), learned
        bool first = true;
        blk_pos = -1; blk_len = 0;
        #define STOPPED (rd_quit || gen != rd_gen)
        while (!STOPPED) {
            bool more_v = iv < nv, more_a = ia < na;
            if (!more_v && !more_a) { rd_eof = true; break; }
            bool audio_next = more_a && (!more_v || avir_audio_chunk_offset(r, ia) < avir_frame_offset(r, iv));
            if (audio_next) {
                u32 ns = avir_audio_chunk_samples(r, ia);
                u64 st = avir_audio_chunk_start(r, ia);
                u32 skip = st < s0 ? (u32)(s0 - st) : 0;
                if (ns > 8192) ns = 8192;
                if (skip >= ns) { ia++; continue; }
                while (!STOPPED && (ar_w - ar_r) + (ns - skip) > AR_CAP) svcSleepThread(4000000);   // room?
                if (STOPPED) break;
                u32 got = blk_get(r, avir_audio_chunk_offset(r, ia), rd_tmp, ns * 2) ? ns : 0;
                u64 w = ar_w;
                for (u32 k = skip; k < got; k++, w++) aring[w % AR_CAP] = rd_tmp[k];
                __dmb();
                ar_w = w;
                ia++;
            } else {
                u32 sz = avir_frame_size(r, iv);
                if (!sz) { iv++; continue; }                          // repeat frame: nothing to read
                if (iv < rd_late_before && iv + 1 < nv) { iv++; st_vdrop++; continue; }   // would be late
                // Sound first: if what's buffered wouldn't outlast reading this frame (plus a
                // margin), skip the frame - it's never read, so the SD time goes to the sound.
                const u32 foff = avir_frame_offset(r, iv);
                if (na && !first && !blk_has(foff, sz)) {      // (frames already in memory are free)
                    double reserve_ms = ((ar_w - ar_r) + a_queued) * 1000.0 / rate;
                    double read_ms = (sz > BLK ? sz : BLK) / 1024.0 / kb_per_ms;
                    if (reserve_ms < read_ms + 150.0) { iv++; st_vdrop++; continue; }
                }
                bool drop = false;
                for (;;) {
                    LightLock_Lock(&vq_lock);
                    bool room = vq_n < VQ_MAX && vq_bytes + sz <= VQ_BYTES;
                    LightLock_Unlock(&vq_lock);
                    if (room || STOPPED) break;
                    // sound first: everything not yet heard (ring + DSP buffers) under ~0.25 s
                    if (na && (ar_w - ar_r) + a_queued < a_low) { drop = true; break; }
                    svcSleepThread(3000000);
                }
                if (STOPPED) break;
                if (drop) { iv++; st_vdrop++; continue; }
                LightLock_Lock(&vq_lock);
                VFrame* f = &vq[(vq_r + vq_n) % VQ_MAX];          // free slot: only we touch it
                LightLock_Unlock(&vq_lock);
                if (f->cap < sz) { u8* p = realloc(f->data, sz); if (!p) { iv++; continue; } f->data = p; f->cap = sz; }
                bool fresh = !blk_has(foff, sz);
                u64 t = osGetTime();
                u32 got = blk_get(r, foff, f->data, sz) ? sz : 0;
                u32 ms = (u32)(osGetTime() - t);
                if (fresh && ms > 0) {                         // learn the card's speed from block reads
                    double k = (sz > BLK ? sz : BLK) / 1024.0 / ms;
                    kb_per_ms = kb_per_ms * 0.7 + k * 0.3;
                }
                first = false;
                if (got) {
                    f->frame = iv; f->size = got;
                    LightLock_Lock(&vq_lock);
                    vq_n++; vq_bytes += got;
                    LightLock_Unlock(&vq_lock);
                }
                iv++;
            }
        }
        #undef STOPPED
        rd_busy = false;
    }
}

// Stop the reader and empty its queues (before a seek / another file / leaving).
static void rd_stop(void) {
    rd_gen++;
    for (int i = 0; i < 400 && rd_busy; i++) svcSleepThread(5000000);   // (finishes its current read)
    LightLock_Lock(&vq_lock);
    vq_r = vq_n = 0; vq_bytes = 0;
    LightLock_Unlock(&vq_lock);
    ar_w = ar_r = 0;
    rd_eof = false;
}

static void rd_start(u32 frame, u64 sample, u32 rate, bool audio) {
    rd_stop();
    a_low = rate / 4;
    rd_audio = audio;
    rd_rate = rate ? rate : 1;
    rd_from_frame = frame; rd_from_sample = sample;
    rd_late_before = 0;
    rd_busy = true;
    __dmb();
    rd_gen++;
    LightEvent_Signal(&rd_go);
}

// ---------------------------------------------------------------- audio (NDSP)
static bool audio_ok = false;
static s16* abuf = NULL;
static ndspWaveBuf wb[AB_COUNT];
static bool wb_used[AB_COUNT];
static u64 a_played = 0;                          // samples of finished buffers since the start
static u64 a_base = 0;                            // sample number playback started at
static bool a_active = false;

static void audio_stop(void) {
    a_queued = 0;
    if (!audio_ok) return;
    ndspChnWaveBufClear(0);
    memset(wb, 0, sizeof(wb));
    memset(wb_used, 0, sizeof(wb_used));
    a_active = false;
}

static void audio_start(AviReader* r, u64 sample) {
    a_base = sample; a_played = 0;
    if (!audio_ok || !avir_audio_rate(r)) return;
    audio_stop();
    ndspChnReset(0);
    ndspChnSetInterp(0, NDSP_INTERP_LINEAR);
    ndspChnSetRate(0, (float)avir_audio_rate(r));
    ndspChnSetFormat(0, NDSP_FORMAT_MONO_PCM16);
    float mix[12] = { 0 };
    mix[0] = mix[1] = 1.0f;
    ndspChnSetMix(0, mix);
    a_active = true;
}

// Move sound from the reader's ring into free DSP buffers (memory only, never the SD card).
static void audio_feed(void) {
    if (!a_active) return;
    for (int i = 0; i < AB_COUNT; i++) {
        if (wb_used[i]) {
            if (wb[i].status != NDSP_WBUF_DONE) continue;
            a_played += wb[i].nsamples;                // finished: counts towards the clock
            wb_used[i] = false;
        }
        u64 avail = ar_w - ar_r;
        if (!avail) continue;
        u32 n = avail < AB_SAMPLES ? (u32)avail : AB_SAMPLES;
        s16* b = abuf + i * AB_SAMPLES;
        for (u32 k = 0; k < n; k++) b[k] = aring[(ar_r + k) % AR_CAP];
        __dmb();
        ar_r += n;
        memset(&wb[i], 0, sizeof(wb[i]));
        wb[i].data_vaddr = b;
        wb[i].nsamples = n;
        DSP_FlushDataCache(b, n * 2);
        ndspChnWaveBufAdd(0, &wb[i]);
        wb_used[i] = true;
    }
    u32 q = 0;
    for (int i = 0; i < AB_COUNT; i++) if (wb_used[i] && wb[i].status != NDSP_WBUF_DONE) q += wb[i].nsamples;
    a_queued = q;
}

// The sound is the clock: samples actually played (it stops if the sound runs dry, so the
// picture can never run away from it).
static u64 audio_clock(void) {
    u64 done = a_played;
    for (int i = 0; i < AB_COUNT; i++)
        if (wb_used[i] && wb[i].status == NDSP_WBUF_DONE) done += wb[i].nsamples;
    bool playing_any = false;
    for (int i = 0; i < AB_COUNT; i++) if (wb_used[i] && wb[i].status == NDSP_WBUF_PLAYING) playing_any = true;
    return a_base + done + (playing_any ? ndspChnGetSamplePos(0) : 0);
}

// ---------------------------------------------------------------- UI
static void fmt_size(char* out, int n, u32 b) {
    if (b >= 100u << 20)     snprintf(out, n, "%luM", (unsigned long)(b >> 20));
    else if (b >= 1u << 20)  snprintf(out, n, "%lu.%luM", (unsigned long)(b >> 20), (unsigned long)(((u64)b * 10 >> 20) % 10));
    else                     snprintf(out, n, "%luK", (unsigned long)((b + 1023) >> 10));
}

static void draw_list(int cur, int top, const char* info, const char* status) {
    printf("\x1b[0;0H\x1b[36m%s\x1b[0m  %d %s  \x1b[33m[" SLOPCAM_BUILD "]\x1b[0m\x1b[K\n", tr("GALLERY"), nfiles, tr("files"));
    printf("%.39s\x1b[K\n", info);
    for (int r = 0; r < LIST_ROWS; r++) {
        int i = top + r;
        if (i >= nfiles) { printf("\x1b[K\n"); continue; }
        char sz[16];
        fmt_size(sz, sizeof(sz), files[i].size);
        bool sel = i == cur;
        printf("%s%c %c %-26.26s %6s\x1b[0m\x1b[K\n", sel ? "\x1b[47;30m" : "", sel ? '>' : ' ',
               files[i].video ? 'V' : 'P', files[i].name, sz);
    }
    printf("\x1b[33m%.39s\x1b[0m\x1b[K\n", status);
    printf("%s\x1b[K", tr("A play/pause  B back  </> seek  ^v pick"));
}

static void draw_status(const char* status) {   // just the status row (cheap, while playing)
    printf("\x1b[%d;0H\x1b[33m%.39s\x1b[0m\x1b[K", 2 + LIST_ROWS, status);
}

// Progress while a long recording's index is read (or a cut-off one is scanned).
static void open_progress(int pct) {
    printf("\x1b[1;0H%s %d%%\x1b[K", tr("Reading index..."), pct);
    gfxFlushBuffers();
}

// ---------------------------------------------------------------- main loop
bool gallery_run(const char* folder) {
    snprintf(g_folder, sizeof(g_folder), "%s", folder);
    if (!files) files = malloc(sizeof(Entry) * MAX_FILES);
    if (!tj) tj = tjInitDecompress();
    consoleClear();
    gfxSet3D(false);
    top_clear_now();
    if (!files || !tj) return true;
    scan_files();

    dec_start();
    LightLock_Init(&vq_lock);
    LightEvent_Init(&rd_go, RESET_ONESHOT);
    aring = malloc(AR_CAP * sizeof(s16));
    blk = malloc(BLK);
    io_probe[0] = io_probe[1] = 0; io_ms_mb = 0; io_blocks = io_probes = 0;
    rd_quit = false;
    s32 mprio = 0x30;
    svcGetThreadPriority(&mprio, CUR_THREAD_HANDLE);
    rd_th = aring ? threadCreate(rd_thread, NULL, 0x4000, mprio - 2, 0, false) : NULL;   // mostly waits on the SD
    if (!rd_th) rd_th = aring ? threadCreate(rd_thread, NULL, 0x4000, mprio - 2, -2, false) : NULL;
    audio_ok = R_SUCCEEDED(ndspInit());
    if (audio_ok) {
        abuf = linearAlloc(AB_COUNT * AB_SAMPLES * 2);
        if (!abuf) { ndspExit(); audio_ok = false; }
        else ndspSetOutputMode(NDSP_OUTPUT_STEREO);
    }

    int cur = 0, top = 0, shown = -1;
    AviReader* vid = NULL;
    bool playing = false, paused = false;
    double base_t = 0;                 // video time at t0_ms (seconds)
    u64 t0_ms = 0;
    int64_t pic_shown = -1;
    u32 next_dispatch = 0;             // next frame number to hand to a decoder
    double dec_ms = 40;                // average decode time: decoders work this far ahead
    bool use_audio = false;            // this video's sound drives the clock
    u32 st_shown = 0; u64 st_t0 = osGetTime(); char stats[48] = "";
    char info[80] = "", status[80] = "";
    u64 sel_ms = osGetTime();          // last cursor move (open after a short delay)
    bool keep = true;
    u64 last_ui = 0;
    bool ui = true;

    while (true) {
        if (!aptMainLoop()) { keep = false; break; }
        hidScanInput();
        u32 kd = hidKeysDown(), kr = hidKeysDownRepeat();

        if (kd & (KEY_B | KEY_START)) {
            if (playing && !(kd & KEY_START)) { playing = false; paused = false; audio_stop(); snprintf(status, sizeof(status), "%s", ""); ui = true; }
            else break;
        }
        if (nfiles && (kr & (KEY_UP | KEY_DOWN))) {
            cur += (kr & KEY_UP) ? -1 : 1;
            if (cur < 0) cur = nfiles - 1;
            if (cur >= nfiles) cur = 0;
            if (playing) { playing = false; paused = false; audio_stop(); }
            sel_ms = osGetTime();
            status[0] = 0;
            ui = true;
        }
        if (cur < top) top = cur;
        if (cur >= top + LIST_ROWS) top = cur - LIST_ROWS + 1;

        // open the selection (after the cursor rested 150 ms, so scrolling stays quick)
        if (nfiles && shown != cur && osGetTime() - sel_ms >= 150) {
            dec_flush();                                 // (decoders never touch the file)
            if (rd_th) rd_stop();
            audio_stop();
            if (rd_avi) { avir_close(rd_avi); rd_avi = NULL; }
            if (vid) { avir_close(vid); vid = NULL; }
            playing = paused = false;
            pic_shown = -1;
            char path[200];
            snprintf(path, sizeof(path), "%s/%s", g_folder, files[cur].name);
            if (files[cur].video) {
                avir_progress = open_progress;               // big files: show that something happens
                vid = avir_open(path);
                avir_progress = NULL;
                if (vid && rd_th) {
                    rd_avi = avir_dup(vid);                  // own file handle, same index
                    if (rd_avi) avir_set_buffer(rd_avi, 0);   // exact reads: frames we skip are never read
                }
                if (vid) {
                    u32 secs = (u32)(avir_frames(vid) / avir_fps(vid));
                    snprintf(info, sizeof(info), "%dx%d %.0ffps %02lu:%02lu%s%s", avir_width(vid), avir_height(vid),
                             avir_fps(vid), (unsigned long)(secs / 60), (unsigned long)(secs % 60),
                             avir_audio_rate(vid) ? "" : tr(" no sound"), avir_was_cut(vid) ? tr(" (cut off)") : "");
                    int64_t p = -1;
                    for (u32 k = 0; p < 0 && k < avir_frames(vid) && k < 600; k++) if (avir_frame_size(vid, k)) p = k;
                    u32 n = p >= 0 ? avir_frame_size(vid, (u32)p) : 0;
                    if (n && grow(&jbuf, &jcap, n) && avir_read_frame(vid, (u32)p, jbuf, jcap)) show_jpeg(jbuf, n, true);
                    base_t = 0;
                } else snprintf(info, sizeof(info), "%s", tr("Can't open this file"));
            } else {
                if (!show_photo(path)) snprintf(info, sizeof(info), "%s", tr("Can't open this file"));
                else snprintf(info, sizeof(info), "%s %dx%d", tr("photo"), photo_w, photo_h);
                status[0] = 0;
            }
            shown = cur;
            ui = true;
        }

        // video controls
        if (vid && shown == cur) {
            double dur = avir_frames(vid) / avir_fps(vid);
            const u32 arate = avir_audio_rate(vid);
            #define NOW_T() (!playing ? base_t : use_audio ? (double)audio_clock() / arate \
                             : paused ? base_t : base_t + (osGetTime() - t0_ms) / 1000.0)
            double now_t = NOW_T();
            if (kd & KEY_A) {
                if (!playing) {
                    if (now_t >= dur - 0.05) now_t = 0;
                    dec_flush(); next_dispatch = 0; pic_shown = -1;
                    playing = true; paused = false; base_t = now_t; t0_ms = osGetTime();
                    audio_start(vid, (u64)(now_t * arate));
                    use_audio = a_active;
                    if (rd_th && rd_avi) rd_start((u32)(now_t * avir_fps(vid)), (u64)(now_t * arate), arate, a_active);
                    st_shown = 0; st_t0 = osGetTime(); st_rd_bytes = st_rd_ms = st_vdrop = 0;
                } else if (!paused) {
                    paused = true; base_t = now_t;
                    if (a_active) ndspChnSetPaused(0, true);
                } else {
                    paused = false; t0_ms = osGetTime();
                    if (a_active) ndspChnSetPaused(0, false);
                }
                ui = true;
            }
            if (kr & (KEY_LEFT | KEY_RIGHT)) {
                double t = now_t + ((kr & KEY_RIGHT) ? 5.0 : -5.0);
                if (t < 0) t = 0;
                if (t > dur - 0.05) t = dur > 0.05 ? dur - 0.05 : 0;
                base_t = t; t0_ms = osGetTime();
                dec_flush(); next_dispatch = 0;
                if (playing) {
                    audio_start(vid, (u64)(t * arate));
                    use_audio = a_active;
                    if (paused && a_active) ndspChnSetPaused(0, true);
                    if (rd_th && rd_avi) rd_start((u32)(t * avir_fps(vid)), (u64)(t * arate), arate, a_active);
                }
                pic_shown = -1;
                if (!playing) {                          // scrub the still picture too
                    int64_t p = avir_picture_at(vid, (u32)(t * avir_fps(vid)));
                    u32 n = p >= 0 ? avir_frame_size(vid, (u32)p) : 0;
                    if (n && grow(&jbuf, &jcap, n) && avir_read_frame(vid, (u32)p, jbuf, jcap)) { show_jpeg(jbuf, n, true); pic_shown = p; }
                }
                ui = true;
            }
            if (playing && !paused) audio_feed();
            now_t = NOW_T();
            if (playing) {
                // end: clock past the end, or (sound clock) nothing left to play
                bool ended = now_t >= dur - 0.001 ||
                             (use_audio && rd_eof && ar_w == ar_r && !(wb_used[0] | wb_used[1] | wb_used[2] | wb_used[3]));
                if (ended) {
                    playing = paused = false; base_t = now_t < dur ? now_t : dur; audio_stop();
                    if (rd_th) rd_stop();
                    ui = true;
                } else {
                    const u32 nfr = avir_frames(vid);
                    u32 due = (u32)(now_t * avir_fps(vid));
                    // 1) show the newest finished picture whose time has come
                    int best = -1;
                    for (int i = 0; i < ndec; i++) {
                        Dec* d = &dec[i];
                        if (d->state != D_DONE) continue;
                        if (d->gen != dec_gen || !d->ok || (int64_t)d->frame <= pic_shown) { d->state = D_IDLE; continue; }
                        if (d->frame <= due && (best < 0 || d->frame > dec[best].frame)) best = i;
                    }
                    for (int i = 0; i < ndec; i++)           // learn how long decoding takes
                        if (dec[i].state == D_DONE && dec[i].gen == dec_gen) dec_ms = dec_ms * 0.8 + dec[i].ms * 0.2;
                    if (best >= 0) {
                        Dec* d = &dec[best];
                        if (d->is_screen) {                    // ready-made screen image: one copy
                            memcpy(gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL), d->screen, 400 * 240 * 2);
                            drew = true;
                            last_w = d->sw; last_h = d->sh;
                            clear_left = 2;                    // (the RGB path must clear if it's used next)
                        } else {
                            show_rgb(d->out, d->w, d->h, d->s);
                        }
                        pic_shown = d->frame;
                        st_shown++;
                        d->state = D_IDLE;
                        for (int i = 0; i < ndec; i++)       // anything older is now useless
                            if (dec[i].state == D_DONE && (int64_t)dec[i].frame <= pic_shown) dec[i].state = D_IDLE;
                    }
                    // 2) give idle decoders the next pictures. Aim at the frame that will be due
                    //    when the decode finishes, so pictures are ready on time (lip-sync) and
                    //    frames we can't make in time are skipped instead of shown late.
                    u32 ahead = (u32)((now_t + (dec_ms * 1.25 + 17) / 1000.0) * avir_fps(vid));
                    if (rd_th && rd_avi) {
                        rd_late_before = pic_shown < 0 ? 0 : ahead;     // the reader skips what would be late
                        for (int i = 0; i < ndec; i++) {
                            Dec* d = &dec[i];
                            if (d->state != D_IDLE) continue;
                            if (d->c1 && io_quiet) continue;          // core 1 belongs to the SD read now
                            VFrame* f = NULL;
                            LightLock_Lock(&vq_lock);
                            while (vq_n > 0) {                     // drop queued frames that are already late
                                VFrame* q = &vq[vq_r];
                                if (pic_shown >= 0 && q->frame < ahead && vq_n > 1) {
                                    vq_r = (vq_r + 1) % VQ_MAX; vq_n--; vq_bytes -= q->size; st_vdrop++;
                                    continue;
                                }
                                f = q; break;
                            }
                            LightLock_Unlock(&vq_lock);
                            if (!f) break;
                            bool ok = grow(&d->jpg, &d->jcap, f->size);
                            if (ok) memcpy(d->jpg, f->data, f->size);   // (front entry: the reader never touches it)
                            u32 fr = f->frame, sz = f->size;
                            LightLock_Lock(&vq_lock);
                            vq_r = (vq_r + 1) % VQ_MAX; vq_n--; vq_bytes -= sz;
                            LightLock_Unlock(&vq_lock);
                            if (!ok) continue;
                            d->jsize = sz; d->frame = fr; d->gen = dec_gen;
                            __dmb();
                            d->state = D_BUSY;
                            LightEvent_Signal(&d->go);
                        }
                    } else for (int i = 0; i < ndec; i++) {
                        Dec* d = &dec[i];
                        if (d->state != D_IDLE) continue;
                        u32 from = next_dispatch > ahead ? next_dispatch : ahead;
                        if (next_dispatch == 0 && pic_shown < 0) from = due;   // first picture asap
                        u32 k = from;
                        while (k < nfr && !avir_frame_size(vid, k)) k++;   // repeats need no decode
                        if (k >= nfr) break;
                        u32 n = avir_frame_size(vid, k);
                        if (!grow(&d->jpg, &d->jcap, n) || !avir_read_frame(vid, k, d->jpg, d->jcap)) { next_dispatch = k + 1; continue; }
                        d->jsize = n; d->frame = k; d->gen = dec_gen;
                        next_dispatch = k + 1;
                        __dmb();
                        d->state = D_BUSY;
                        LightEvent_Signal(&d->go);
                    }
                    if (!ndec) {                             // no threads: decode here
                        int64_t p = avir_picture_at(vid, due);
                        if (p >= 0 && p != pic_shown) {
                            u32 n = avir_frame_size(vid, (u32)p);
                            if (grow(&jbuf, &jcap, n) && avir_read_frame(vid, (u32)p, jbuf, jcap)) show_jpeg(jbuf, n, true);
                            pic_shown = p;
                        }
                    }
                }
            }
            if (playing && osGetTime() - st_t0 >= 1000) {   // what the player is doing, once a second
                u32 dt = (u32)(osGetTime() - st_t0);
                // SD speed of the 1 MB block reads (MB/s), "*" = core 1 is left free while reading,
                // then the two 32 KB probe times (ms): core 1 free / core 1 decoding
                u32 sp = io_ms_mb > 0 ? (u32)(10000.0 / io_ms_mb) : 0;   // tenths of MB/s
                snprintf(stats, sizeof(stats), "%2lufps d%2.0f sd%lu.%lu%s %.0f/%.0f dr%lu s%lu.%lu",
                         (unsigned long)(st_shown * 1000 / dt), dec_ms, (unsigned long)(sp / 10), (unsigned long)(sp % 10),
                         io_mode() ? "*" : "", io_probe[1], io_probe[0],
                         (unsigned long)st_vdrop, (unsigned long)(((ar_w - ar_r) + a_queued) / (arate ? arate : 1)),
                         (unsigned long)(((ar_w - ar_r) + a_queued) * 10 / (arate ? arate : 1) % 10));
                printf("\x1b[1;0H%.39s\x1b[K", stats);
                st_shown = 0; st_t0 = osGetTime(); st_rd_bytes = st_rd_ms = 0;
            }
            if (osGetTime() - last_ui > 250 && !ui) {
                u32 a = (u32)now_t, b = (u32)dur;
                snprintf(status, sizeof(status), "%s %02lu:%02lu / %02lu:%02lu%s",
                         playing ? (paused ? tr("PAUSED") : tr("PLAY")) : tr("STOP"),
                         (unsigned long)(a / 60), (unsigned long)(a % 60), (unsigned long)(b / 60), (unsigned long)(b % 60),
                         (playing && !audio_ok && avir_audio_rate(vid)) ? tr(" (no DSP firm.)") : "");
                draw_status(status);
                last_ui = osGetTime();
            }
        }
        if (!nfiles) snprintf(info, sizeof(info), "%s", tr("No photos or videos yet"));

        if (ui) { draw_list(cur, top, info, status); ui = false; last_ui = osGetTime(); }
        gfxFlushBuffers();
        if (drew) { gfxSwapBuffers(); drew = false; }   // only a new picture: no flicker between buffers
        gspWaitForVBlank();
    }

    audio_stop();
    dec_stop();
    if (rd_th) {
        rd_stop();
        rd_quit = true; LightEvent_Signal(&rd_go);
        threadJoin(rd_th, U64_MAX); threadFree(rd_th); rd_th = NULL;
    }
    if (rd_avi) { avir_close(rd_avi); rd_avi = NULL; }
    for (int i = 0; i < VQ_MAX; i++) { free(vq[i].data); vq[i].data = NULL; vq[i].cap = 0; }
    free(aring); aring = NULL;
    free(blk); blk = NULL; blk_pos = -1;
    if (vid) avir_close(vid);
    if (audio_ok) { ndspExit(); linearFree(abuf); abuf = NULL; audio_ok = false; }
    free(jbuf); jbuf = NULL; jcap = 0;
    free(rgb); rgb = NULL; rgbcap = 0;
    last_w = last_h = 0;
    consoleClear();
    return keep;
}
