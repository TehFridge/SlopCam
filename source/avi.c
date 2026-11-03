#include "avi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AVI_SPLIT_BYTES   (1900ull * 1024 * 1024)  // stay under 2 GB
#define AVI_MAX_INDEX     (400 * 1024)             // 6.4 MB of index max
// Every file-service call on the 3DS SD card has a large fixed cost (tens of ms, measured on
// a console: a 32 KB read took ~40-90 ms, a 1 MB read ~200-370 ms). The writer used stdio
// with a 256 KB buffer, so a quarter of every write was overhead: at VGA in a bright scene
// (big frames) the card couldn't keep up, the record queue filled and frames were dropped
// ("late"). Now data goes out in 1 MB writes straight to the file service.
#define AVI_IO_BUFFER     (1024 * 1024)

#define AVIF_HASINDEX       0x10
#define AVIF_ISINTERLEAVED  0x100
#define AVIIF_KEYFRAME      0x10

typedef struct { u32 id, flags, offset, size; } AviIdx;

struct AviWriter {
    FILE* f;
#ifdef __3DS__
    Handle fh;          // direct file-service handle (0 = stdio)
#endif
    char* iobuf;
    u32 buf_n;          // bytes waiting in iobuf (direct mode)
    u64 flushed;        // file offset of iobuf[0]
    u64 wr_bytes, wr_ms;   // write statistics (direct mode)
    u16 w, h;
    u32 fps, arate;
    u64 pos;            // bytes written so far
    u32 movi_fcc_pos;   // file offset of the 'movi' fourcc (idx1 offsets are relative to it)
    AviIdx* idx;
    u32 idx_count, idx_cap;
    u32 video_frames, audio_bytes;
    u32 max_video_chunk, max_audio_chunk;
    bool error;
};

// ---- little byte builder for the header ----
typedef struct { u8* p; u32 n; } Buf;
static void b32(Buf* b, u32 v) { memcpy(b->p + b->n, &v, 4); b->n += 4; }
static void b16(Buf* b, u16 v) { memcpy(b->p + b->n, &v, 2); b->n += 2; }
static void bcc(Buf* b, const char* c) { memcpy(b->p + b->n, c, 4); b->n += 4; }
static u32  bopen(Buf* b, const char* c) { bcc(b, c); u32 at = b->n; b32(b, 0); return at; }
static void bclose(Buf* b, u32 at) { u32 sz = b->n - at - 4; memcpy(b->p + at, &sz, 4); }

#define HDR_MAX 512

// Builds the whole header (RIFF..'movi'). Same size every call, so it can be
// rewritten in place on close with the final counts.
static u32 build_header(const AviWriter* a, u8* out, u32 riff_size, u32 movi_size) {
    Buf b = { out, 0 };
    bool audio = a->arate > 0;
    u32 usec = a->fps ? 1000000u / a->fps : 33333u;
    u32 audio_samples = a->audio_bytes / 2;

    bcc(&b, "RIFF"); b32(&b, riff_size); bcc(&b, "AVI ");

    u32 hdrl = bopen(&b, "LIST"); bcc(&b, "hdrl");
      bcc(&b, "avih"); b32(&b, 56);
      b32(&b, usec);
      b32(&b, a->max_video_chunk * a->fps + a->arate * 2);   // max bytes/sec (approx)
      b32(&b, 0);                                            // padding granularity
      b32(&b, AVIF_HASINDEX | AVIF_ISINTERLEAVED);
      b32(&b, a->video_frames);
      b32(&b, 0);                                            // initial frames
      b32(&b, audio ? 2 : 1);
      b32(&b, a->max_video_chunk + 8);
      b32(&b, a->w); b32(&b, a->h);
      b32(&b, 0); b32(&b, 0); b32(&b, 0); b32(&b, 0);

      u32 strl = bopen(&b, "LIST"); bcc(&b, "strl");
        bcc(&b, "strh"); b32(&b, 56);
        bcc(&b, "vids"); bcc(&b, "MJPG");
        b32(&b, 0); b16(&b, 0); b16(&b, 0); b32(&b, 0);
        b32(&b, 1); b32(&b, a->fps);                         // scale / rate
        b32(&b, 0); b32(&b, a->video_frames);                // start / length
        b32(&b, a->max_video_chunk);
        b32(&b, 0xFFFFFFFF); b32(&b, 0);                     // quality / sample size
        b16(&b, 0); b16(&b, 0); b16(&b, a->w); b16(&b, a->h);
        bcc(&b, "strf"); b32(&b, 40);
        b32(&b, 40); b32(&b, a->w); b32(&b, a->h);
        b16(&b, 1); b16(&b, 24); bcc(&b, "MJPG");
        b32(&b, (u32)a->w * a->h * 3);
        b32(&b, 0); b32(&b, 0); b32(&b, 0); b32(&b, 0);
      bclose(&b, strl);

      if (audio) {
        strl = bopen(&b, "LIST"); bcc(&b, "strl");
          bcc(&b, "strh"); b32(&b, 56);
          bcc(&b, "auds"); b32(&b, 0);
          b32(&b, 0); b16(&b, 0); b16(&b, 0); b32(&b, 0);
          b32(&b, 1); b32(&b, a->arate);                     // scale / rate (samples)
          b32(&b, 0); b32(&b, audio_samples);
          b32(&b, a->max_audio_chunk);
          b32(&b, 0xFFFFFFFF); b32(&b, 2);                   // quality / sample size
          b16(&b, 0); b16(&b, 0); b16(&b, 0); b16(&b, 0);
          bcc(&b, "strf"); b32(&b, 16);
          b16(&b, 1); b16(&b, 1);                            // PCM, mono
          b32(&b, a->arate); b32(&b, a->arate * 2);
          b16(&b, 2); b16(&b, 16);
        bclose(&b, strl);
      }
    bclose(&b, hdrl);

    bcc(&b, "LIST"); b32(&b, movi_size); bcc(&b, "movi");
    return b.n;
}

#ifdef __3DS__
static bool fs_write_at(AviWriter* a, u64 off, const void* data, u32 len) {
    u32 done = 0;
    u64 t = osGetTime();
    bool ok = R_SUCCEEDED(FSFILE_Write(a->fh, &done, off, data, len, 0)) && done == len;
    a->wr_ms += osGetTime() - t;
    a->wr_bytes += len;
    return ok;
}
#endif

static void flush_buf(AviWriter* a) {
#ifdef __3DS__
    if (a->fh && a->buf_n) {
        if (!fs_write_at(a, a->flushed, a->iobuf, a->buf_n)) a->error = true;
        a->flushed += a->buf_n;
        a->buf_n = 0;
    }
#else
    (void)a;
#endif
}

static void put(AviWriter* a, const void* data, u32 len) {
    if (a->error || len == 0) return;
#ifdef __3DS__
    if (a->fh) {
        const u8* d = (const u8*)data;
        a->pos += len;
        while (len) {
            u32 n = AVI_IO_BUFFER - a->buf_n;
            if (n > len) n = len;
            memcpy(a->iobuf + a->buf_n, d, n);
            a->buf_n += n; d += n; len -= n;
            if (a->buf_n == AVI_IO_BUFFER) { flush_buf(a); if (a->error) return; }
        }
        return;
    }
#endif
    if (fwrite(data, 1, len, a->f) != len) a->error = true;
    a->pos += len;
}

static bool add_index(AviWriter* a, u32 id, u32 flags, u32 offset, u32 size) {
    if (a->idx_count == a->idx_cap) {
        u32 ncap = a->idx_cap ? a->idx_cap * 2 : 8192;
        if (ncap > AVI_MAX_INDEX) ncap = AVI_MAX_INDEX;
        if (ncap <= a->idx_count) return false;
        AviIdx* n = realloc(a->idx, ncap * sizeof(AviIdx));
        if (!n) return false;
        a->idx = n; a->idx_cap = ncap;
    }
    a->idx[a->idx_count++] = (AviIdx){ id, flags, offset, size };
    return true;
}

static bool write_chunk(AviWriter* a, const char* fcc, u32 id, u32 flags, const void* data, u32 size) {
    if (a->error) return false;
    u32 offset = (u32)(a->pos - a->movi_fcc_pos);
    if (!add_index(a, id, flags, offset, size)) { a->error = true; return false; }
    u8 hdr[8];
    memcpy(hdr, fcc, 4); memcpy(hdr + 4, &size, 4);
    put(a, hdr, 8);
    put(a, data, size);
    if (size & 1) { u8 z = 0; put(a, &z, 1); }
    return !a->error;
}

AviWriter* avi_open(const char* path, u16 width, u16 height, u32 fps, u32 audio_rate) {
    AviWriter* a = calloc(1, sizeof(AviWriter));
    if (!a) return NULL;
    a->iobuf = malloc(AVI_IO_BUFFER);
#ifdef __3DS__
    if (a->iobuf) {     // "sdmc:/dir/file" -> "/dir/file" on the SD archive
        const char* p = strncmp(path, "sdmc:", 5) == 0 ? path + 5 : path;
        Handle fh = 0;
        if (R_SUCCEEDED(FSUSER_OpenFileDirectly(&fh, ARCHIVE_SDMC, fsMakePath(PATH_EMPTY, ""),
                                                fsMakePath(PATH_ASCII, p), FS_OPEN_WRITE | FS_OPEN_CREATE, 0))) {
            if (R_SUCCEEDED(FSFILE_SetSize(fh, 0))) a->fh = fh;    // (an old file of that name: start empty)
            else FSFILE_Close(fh);
        }
    }
    if (!a->fh)
#endif
    {
        a->f = fopen(path, "wb");
        if (!a->f) { free(a->iobuf); free(a); return NULL; }
        if (a->iobuf) setvbuf(a->f, a->iobuf, _IOFBF, AVI_IO_BUFFER);
    }

    a->w = width; a->h = height; a->fps = fps ? fps : 30; a->arate = audio_rate;

    u8 hdr[HDR_MAX];
    u32 n = build_header(a, hdr, 0, 0);
    put(a, hdr, n);
    a->movi_fcc_pos = n - 4;
    if (a->error) { avi_close(a); return NULL; }
    return a;
}

bool avi_write_video(AviWriter* a, const void* jpeg, u32 size) {
    if (!a) return false;
    if (size > a->max_video_chunk) a->max_video_chunk = size;
    bool ok = write_chunk(a, "00dc", 0x63643030, size ? AVIIF_KEYFRAME : 0, jpeg, size);
    if (ok) a->video_frames++;
    return ok;
}

bool avi_write_audio(AviWriter* a, const void* pcm16, u32 bytes) {
    if (!a || !a->arate || bytes == 0) return false;
    bytes &= ~1u;
    if (bytes > a->max_audio_chunk) a->max_audio_chunk = bytes;
    bool ok = write_chunk(a, "01wb", 0x62773130, AVIIF_KEYFRAME, pcm16, bytes);
    if (ok) a->audio_bytes += bytes;
    return ok;
}

u32 avi_video_frames(const AviWriter* a) { return a ? a->video_frames : 0; }
u64 avi_audio_samples(const AviWriter* a) { return a ? a->audio_bytes / 2 : 0; }
bool avi_failed(const AviWriter* a) { return !a || a->error; }

// How full this file is before the writer starts the next one (0..1000):
// whichever runs out first, the 1.9 GB size cap or the index.
u32 avi_fill_permille(const AviWriter* a) {
    if (!a) return 0;
    u64 bytes = a->pos * 1000 / AVI_SPLIT_BYTES;
    u64 idx = (u64)a->idx_count * 1000 / (AVI_MAX_INDEX - 64);
    u64 m = bytes > idx ? bytes : idx;
    return m > 1000 ? 1000 : (u32)m;
}
u64 avi_file_bytes(const AviWriter* a) { return a ? a->pos : 0; }

bool avi_should_split(const AviWriter* a) {
    return a && (a->pos >= AVI_SPLIT_BYTES || a->idx_count + 64 >= AVI_MAX_INDEX);
}

// SD write speed of the 1 MB writes so far, in KB/s (0 = not known yet / stdio).
u32 avi_write_kbs(const AviWriter* a) {
    if (!a || !a->wr_ms) return 0;
    return (u32)(a->wr_bytes * 1000 / 1024 / a->wr_ms);
}

void avi_close(AviWriter* a) {
    if (!a) return;
#ifdef __3DS__
    if (a->fh) {
        u32 movi_size = (u32)(a->pos - (a->movi_fcc_pos - 4) - 4);
        u8 hdr[8];
        memcpy(hdr, "idx1", 4);
        u32 isz = a->idx_count * sizeof(AviIdx);
        memcpy(hdr + 4, &isz, 4);
        bool was_error = a->error;
        a->error = false;               // still try to finalize a partial file
        put(a, hdr, 8);
        put(a, a->idx, isz);
        flush_buf(a);
        u32 riff_size = (u32)(a->pos - 8);
        u8 h[HDR_MAX];
        u32 n = build_header(a, h, riff_size, movi_size);
        fs_write_at(a, 0, h, n);        // final counts into the header
        FSFILE_Flush(a->fh);
        FSFILE_Close(a->fh);
        a->error = was_error;
        free(a->idx);
        free(a->iobuf);
        free(a);
        return;
    }
#endif
    if (a->f) {
        u32 movi_size = (u32)(a->pos - (a->movi_fcc_pos - 4) - 4);
        u8 hdr[8];
        memcpy(hdr, "idx1", 4);
        u32 isz = a->idx_count * sizeof(AviIdx);
        memcpy(hdr + 4, &isz, 4);
        bool was_error = a->error;
        a->error = false;               // still try to finalize a partial file
        put(a, hdr, 8);
        put(a, a->idx, isz);
        u32 riff_size = (u32)(a->pos - 8);

        u8 h[HDR_MAX];
        u32 n = build_header(a, h, riff_size, movi_size);
        fflush(a->f);
        if (fseek(a->f, 0, SEEK_SET) == 0) fwrite(h, 1, n, a->f);
        fclose(a->f);
        a->error = was_error;
    }
    free(a->idx);
    free(a->iobuf);
    free(a);
}
