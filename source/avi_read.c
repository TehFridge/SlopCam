#include "avi_read.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __3DS__
#include <3ds.h>
#endif

// All file access goes through rd_at(): on the 3DS a positional FSFILE_Read straight to the
// file service (one call per chunk, no seeking). stdio on the 3DS SD card turned out to be
// painfully slow for this access pattern (0.2 MB/s measured on a new card).

typedef struct { uint32_t off, size; } VChunk;               // off = file offset of the data
typedef struct { uint32_t off, size; uint64_t start; } AChunk; // start = first sample number

struct AviReader {
    FILE* f;
    long file_size;
    int w, h;
    uint32_t rate, scale;               // video frames per second = rate / scale
    uint32_t arate;
    VChunk* v; uint32_t nv, capv;
    AChunk* a; uint32_t na, capa;
    uint64_t asamples;
    bool cut;
    uint32_t acur;                      // audio chunk cursor (sequential reads are the norm)
    char* iobuf;
    bool shared;                        // index arrays belong to another reader (avir_dup)
    char path[256];
#ifdef __3DS__
    Handle fh;                          // direct FS handle (0 = use stdio)
#endif
};

static bool rd_at(AviReader* r, long off, void* dst, uint32_t n) {
    if (off < 0) return false;
#ifdef __3DS__
    if (r->fh) {
        u32 got = 0;
        return R_SUCCEEDED(FSFILE_Read(r->fh, &got, (u64)off, dst, n)) && got == n;
    }
#endif
    return fseek(r->f, off, SEEK_SET) == 0 && fread(dst, 1, n, r->f) == n;
}

// Bytes actually available from `off` (for the torn tail of a cut-off file).
static uint32_t rd_some(AviReader* r, long off, void* dst, uint32_t n) {
    if (off < 0 || off >= r->file_size) return 0;
    if ((long)n > r->file_size - off) n = (uint32_t)(r->file_size - off);
    return rd_at(r, off, dst, n) ? n : 0;
}

static uint32_t rd32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | p[1] << 8); }

static bool add_v(AviReader* r, uint32_t off, uint32_t size) {
    if (r->nv == r->capv) {
        uint32_t n = r->capv ? r->capv * 2 : 4096;
        VChunk* p = realloc(r->v, n * sizeof(VChunk));
        if (!p) return false;
        r->v = p; r->capv = n;
    }
    r->v[r->nv++] = (VChunk){ off, size };
    return true;
}

static bool add_a(AviReader* r, uint32_t off, uint32_t size) {
    if (r->na == r->capa) {
        uint32_t n = r->capa ? r->capa * 2 : 4096;
        AChunk* p = realloc(r->a, n * sizeof(AChunk));
        if (!p) return false;
        r->a = p; r->capa = n;
    }
    r->a[r->na++] = (AChunk){ off, size, r->asamples };
    r->asamples += size / 2;
    return true;
}

// Stream headers inside LIST hdrl (already read into memory).
static void parse_hdrl(AviReader* r, const uint8_t* p, uint32_t n) {
    char type[5] = "";
    for (uint32_t i = 0; i + 8 <= n; ) {
        uint32_t sz = rd32(p + i + 4);
        if (sz > n - i - 8) break;
        const uint8_t* d = p + i + 8;
        if (!memcmp(p + i, "LIST", 4)) { if (sz >= 4) parse_hdrl(r, d + 4, sz - 4); }
        else if (!memcmp(p + i, "avih", 4) && sz >= 40) {
            r->w = (int)rd32(d + 32); r->h = (int)rd32(d + 36);
            uint32_t usec = rd32(d);
            if (usec && !r->rate) { r->rate = 1000000; r->scale = usec; }
        } else if (!memcmp(p + i, "strh", 4) && sz >= 32) {
            memcpy(type, d, 4); type[4] = 0;
            if (!strcmp(type, "vids")) {
                uint32_t sc = rd32(d + 20), ra = rd32(d + 24);
                if (sc && ra) { r->scale = sc; r->rate = ra; }
            }
        } else if (!memcmp(p + i, "strf", 4)) {
            if (!strcmp(type, "auds") && sz >= 16 && rd16(d) == 1 && rd16(d + 2) == 1 && rd16(d + 14) == 16)
                r->arate = rd32(d + 4);                       // PCM, mono, 16-bit only
            else if (!strcmp(type, "vids") && sz >= 12 && !r->w) { r->w = (int)rd32(d + 4); r->h = (int)rd32(d + 8); }
        }
        i += 8 + sz + (sz & 1);
    }
}

void (*avir_progress)(int percent) = NULL;

// Walk chunks [pos, end) of 'movi' (no index available). Big recordings have tens of
// thousands of chunks, so headers are read through a 4 KB window: one SD read usually covers
// a frame's header and the small audio chunk after it.
static void scan_movi(AviReader* r, long pos, long end) {
    static uint8_t win[4096];
    long wpos = -1; uint32_t wlen = 0;
    uint32_t want = 64;                                // read size: grows to cover a small chunk + next header
    const long start = pos;
    int last_pct = -1;
    uint32_t n = 0;
    uint8_t h[8];
    while (pos + 8 <= end) {
        if (!(wpos >= 0 && pos >= wpos && pos + 8 <= wpos + (long)wlen)) {
            uint32_t n = want < end - pos ? want : (uint32_t)(end - pos);
            wlen = rd_some(r, pos, win, n);
            wpos = pos;
            if (wlen < 8) break;
        }
        memcpy(h, win + (pos - wpos), 8);
        if (avir_progress && (++n & 255) == 0) {
            int pct = (int)((pos - start) * 100 / (end - start > 0 ? end - start : 1));
            if (pct != last_pct) { avir_progress(pct); last_pct = pct; }
        }
        uint32_t sz = rd32(h + 4);
        if ((long)sz > end - pos - 8) break;                  // torn last chunk
        // small chunk (audio): next time read enough for one like it plus the header after it;
        // big chunk (a frame): its data isn't needed, only the next header
        if (sz + 24 <= sizeof(win)) want = sz + 24 > want ? sz + 24 : want;
        if (!memcmp(h, "00dc", 4) || !memcmp(h, "00db", 4)) { if (!add_v(r, (uint32_t)(pos + 8), sz)) break; }
        else if (!memcmp(h, "01wb", 4)) { if (r->arate && !add_a(r, (uint32_t)(pos + 8), sz)) break; }
        else if (!memcmp(h, "LIST", 4)) { pos += 12; continue; }          // rec lists: step inside
        else if (memcmp(h, "JUNK", 4) && memcmp(h, "idx1", 4)) {
            if (h[0] < 0x20 || h[0] > 0x7E) break;            // not a chunk: garbage tail
        }
        pos += 8 + sz + (sz & 1);
    }
}

static bool open_file(AviReader* r, const char* path) {
    snprintf(r->path, sizeof(r->path), "%s", path);
#ifdef __3DS__
    {   // "sdmc:/dir/file" -> FS path "/dir/file" on the SD archive
        const char* p = strncmp(path, "sdmc:", 5) == 0 ? path + 5 : path;
        Handle fh = 0;
        if (R_SUCCEEDED(FSUSER_OpenFileDirectly(&fh, ARCHIVE_SDMC, fsMakePath(PATH_EMPTY, ""),
                                                fsMakePath(PATH_ASCII, p), FS_OPEN_READ, 0))) {
            u64 sz = 0;
            if (R_SUCCEEDED(FSFILE_GetSize(fh, &sz)) && sz < 0x7FFFFFFF) { r->fh = fh; r->file_size = (long)sz; return true; }
            FSFILE_Close(fh);
        }
    }
#endif
    r->f = fopen(path, "rb");
    if (!r->f) return false;
    fseek(r->f, 0, SEEK_END);
    r->file_size = ftell(r->f);
    fseek(r->f, 0, SEEK_SET);
    return true;
}

AviReader* avir_dup(const AviReader* src) {
    AviReader* r = calloc(1, sizeof(AviReader));
    if (!r) return NULL;
    *r = *src;                                         // same index (shared), own file handle
    r->shared = true; r->iobuf = NULL; r->f = NULL; r->acur = 0;
#ifdef __3DS__
    r->fh = 0;
#endif
    if (!open_file(r, src->path)) { free(r); return NULL; }
    return r;
}

AviReader* avir_open(const char* path) {
    AviReader* r = calloc(1, sizeof(AviReader));
    if (!r) return NULL;
    if (!open_file(r, path)) { free(r); return NULL; }

    uint8_t h[12];
    if (!rd_at(r, 0, h, 12) || memcmp(h, "RIFF", 4) || memcmp(h + 8, "AVI ", 4)) { avir_close(r); return NULL; }

    long movi = -1, movi_end = r->file_size, idx_pos = -1;
    uint32_t idx_size = 0;
    long pos = 12;
    while (pos + 8 <= r->file_size) {
        uint8_t c[12];
        if (rd_some(r, pos, c, 12) < 8) break;
        uint32_t sz = rd32(c + 4);
        if (!memcmp(c, "LIST", 4) && !memcmp(c + 8, "hdrl", 4) && sz < (1u << 20)) {
            uint8_t* buf = malloc(sz);
            if (buf && rd_at(r, pos + 12, buf, sz - 4))
                parse_hdrl(r, buf, sz - 4);
            free(buf);
        } else if (!memcmp(c, "LIST", 4) && !memcmp(c + 8, "movi", 4)) {
            movi = pos + 8;                                   // offset of the 'movi' fourcc
            // An unfinished recording still has size 0 here: the data runs to the end.
            if (sz < 4 || (long)sz > r->file_size - pos - 8) { movi_end = r->file_size; break; }
            movi_end = pos + 8 + sz;
        } else if (!memcmp(c, "idx1", 4)) {
            idx_pos = pos + 8; idx_size = sz;
        }
        if (movi >= 0 && sz == 0) break;
        pos += 8 + sz + (sz & 1);
    }
    if (movi < 0 || r->w <= 0 || r->h <= 0) { avir_close(r); return NULL; }
    if (!r->rate || !r->scale) { r->rate = 30; r->scale = 1; }

    bool indexed = false;
    if (idx_pos > 0 && idx_size >= 16 && (long)idx_size <= r->file_size - idx_pos) {
        // read in 64 KB pieces: a long recording's index is several MB
        enum { PIECE = 65536 };
        uint8_t* ix = malloc(PIECE);
        long base = movi;
        indexed = ix != NULL;
        for (uint32_t done = 0; indexed && done + 16 <= idx_size; ) {
            uint32_t n = idx_size - done; if (n > PIECE) n = PIECE; n &= ~15u;
            if (!rd_at(r, idx_pos + (long)done, ix, n)) { indexed = false; break; }
            if (done == 0) {
                // offsets are normally relative to the 'movi' fourcc; some writers use file offsets
                uint32_t first = rd32(ix + 8);
                base = (first >= (uint32_t)movi) ? 0 : movi;
            }
            for (uint32_t i = 0; i < n; i += 16) {
                const uint8_t* e = ix + i;
                long off = base + (long)rd32(e + 8) + 8;      // entry points at the chunk header
                uint32_t sz = rd32(e + 12);
                if (off + (long)sz > r->file_size) { indexed = false; break; }
                bool ok = true;
                if (!memcmp(e, "00dc", 4) || !memcmp(e, "00db", 4)) ok = add_v(r, (uint32_t)off, sz);
                else if (!memcmp(e, "01wb", 4) && r->arate)    ok = add_a(r, (uint32_t)off, sz);
                if (!ok) { indexed = false; break; }
            }
            done += n;
            if (avir_progress && idx_size > (1u << 20)) avir_progress((int)((uint64_t)done * 100 / idx_size));
        }
        free(ix);
        if (!indexed) { r->nv = r->na = 0; r->asamples = 0; }
    }
    if (!indexed) { r->cut = true; scan_movi(r, movi + 4, movi_end); }
    if (!r->nv) { avir_close(r); return NULL; }
    return r;
}

void avir_close(AviReader* r) {
    if (!r) return;
    if (r->f) fclose(r->f);
#ifdef __3DS__
    if (r->fh) FSFILE_Close(r->fh);
#endif
    free(r->iobuf);
    if (!r->shared) { free(r->v); free(r->a); }
    free(r);
}

int      avir_width(const AviReader* r)  { return r->w; }
int      avir_height(const AviReader* r) { return r->h; }
double   avir_fps(const AviReader* r)    { return (double)r->rate / r->scale; }
uint32_t avir_frames(const AviReader* r) { return r->nv; }
uint32_t avir_audio_rate(const AviReader* r) { return r->na ? r->arate : 0; }
uint64_t avir_audio_samples(const AviReader* r) { return r->asamples; }
bool     avir_was_cut(const AviReader* r) { return r->cut; }
uint32_t avir_frame_size(const AviReader* r, uint32_t i) { return i < r->nv ? r->v[i].size : 0; }

int64_t avir_picture_at(const AviReader* r, uint32_t i) {
    if (!r->nv) return -1;
    if (i >= r->nv) i = r->nv - 1;
    for (int64_t k = i; k >= 0; k--) if (r->v[k].size) return k;
    return -1;
}

uint32_t avir_read_frame(AviReader* r, uint32_t i, uint8_t* buf, uint32_t cap) {
    if (i >= r->nv || !r->v[i].size || r->v[i].size > cap) return 0;
    return rd_at(r, (long)r->v[i].off, buf, r->v[i].size) ? r->v[i].size : 0;
}

uint32_t avir_read_audio(AviReader* r, uint64_t pos, int16_t* dst, uint32_t n) {
    if (!r->na || pos >= r->asamples) return 0;
    // find the chunk holding `pos`: usually the cursor or the next one, else binary search
    uint32_t c = r->acur < r->na ? r->acur : 0;
    if (!(r->a[c].start <= pos && pos < r->a[c].start + r->a[c].size / 2)) {
        uint32_t lo = 0, hi = r->na;
        while (hi - lo > 1) { uint32_t m = (lo + hi) / 2; if (r->a[m].start <= pos) lo = m; else hi = m; }
        c = lo;
    }
    uint32_t got = 0;
    while (got < n && c < r->na) {
        const AChunk* a = &r->a[c];
        uint64_t in = pos + got - a->start;                    // sample offset inside the chunk
        uint32_t left = a->size / 2 - (uint32_t)in;
        uint32_t take = n - got < left ? n - got : left;
        if (take && !rd_at(r, (long)(a->off + in * 2), dst + got, take * 2)) break;
        got += take;
        if (got < n) c++;
    }
    r->acur = c;
    return got;
}

uint32_t avir_audio_chunks(const AviReader* r) { return r->na; }
uint64_t avir_audio_chunk_start(const AviReader* r, uint32_t j) { return j < r->na ? r->a[j].start : r->asamples; }
uint32_t avir_audio_chunk_samples(const AviReader* r, uint32_t j) { return j < r->na ? r->a[j].size / 2 : 0; }
uint32_t avir_frame_offset(const AviReader* r, uint32_t i) { return i < r->nv ? r->v[i].off : 0xFFFFFFFFu; }
uint32_t avir_audio_chunk_offset(const AviReader* r, uint32_t j) { return j < r->na ? r->a[j].off : 0xFFFFFFFFu; }

uint32_t avir_audio_chunk_at(const AviReader* r, uint64_t sample) {
    if (!r->na) return 0;
    uint32_t lo = 0, hi = r->na;
    while (hi - lo > 1) { uint32_t m = (lo + hi) / 2; if (r->a[m].start <= sample) lo = m; else hi = m; }
    return lo;
}

uint32_t avir_read_audio_chunk(AviReader* r, uint32_t j, int16_t* dst, uint32_t cap) {
    if (j >= r->na) return 0;
    uint32_t n = r->a[j].size / 2;
    if (n > cap) n = cap;
    return rd_at(r, (long)r->a[j].off, dst, n * 2) ? n : 0;
}

void avir_set_buffer(AviReader* r, uint32_t bytes) {
    if (!r->f) return;                                        // direct FS reads: nothing to buffer
    if (!bytes) { setvbuf(r->f, NULL, _IONBF, 0); return; }   // unbuffered: every read is exactly one SD read
    char* b = malloc(bytes);
    if (b && setvbuf(r->f, b, _IOFBF, bytes) == 0) { free(r->iobuf); r->iobuf = b; }
    else free(b);
}

uint32_t avir_read_raw(AviReader* r, uint32_t off, void* dst, uint32_t n) { return rd_some(r, (long)off, dst, n); }
uint32_t avir_file_size(const AviReader* r) { return (uint32_t)r->file_size; }
