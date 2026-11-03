#ifndef AVI_READ_H
#define AVI_READ_H

// Reader for SlopCam's own AVI files (MJPEG video + mono 16-bit PCM audio), for the
// gallery player. Uses the idx1 index when it's there; files whose recording was cut off
// (power loss, SD pulled) have no index, so the 'movi' list is scanned instead.

#include <stdint.h>
#include <stdbool.h>

typedef struct AviReader AviReader;

AviReader* avir_open(const char* path);
// Second handle on the same file sharing r's index (for another thread). Close it before r.
AviReader* avir_dup(const AviReader* r);
// Optional: called while a big index is read / a cut-off file is scanned (0..100).
extern void (*avir_progress)(int percent);
void       avir_close(AviReader* r);

int      avir_width(const AviReader* r);
int      avir_height(const AviReader* r);
double   avir_fps(const AviReader* r);
uint32_t avir_frames(const AviReader* r);          // video frames (incl. empty "repeat" frames)
uint32_t avir_audio_rate(const AviReader* r);      // 0 = no audio track
uint64_t avir_audio_samples(const AviReader* r);
bool     avir_was_cut(const AviReader* r);         // no index: recording didn't finish cleanly

// Size of frame i (0 = repeat of the previous frame).
uint32_t avir_frame_size(const AviReader* r, uint32_t i);
// Last frame at or before i that has a picture, or -1.
int64_t  avir_picture_at(const AviReader* r, uint32_t i);
// Reads frame i's JPEG into buf. Returns its size, 0 on error / too big.
uint32_t avir_read_frame(AviReader* r, uint32_t i, uint8_t* buf, uint32_t cap);
// Reads up to n mono samples starting at sample `pos`. Returns how many were read.
uint32_t avir_read_audio(AviReader* r, uint64_t pos, int16_t* dst, uint32_t n);

// ---- sequential access (the player's reader thread): chunks in file order
uint32_t avir_audio_chunks(const AviReader* r);
uint64_t avir_audio_chunk_start(const AviReader* r, uint32_t j);    // first sample number
uint32_t avir_audio_chunk_samples(const AviReader* r, uint32_t j);
uint32_t avir_audio_chunk_at(const AviReader* r, uint64_t sample);  // chunk holding `sample`
uint32_t avir_frame_offset(const AviReader* r, uint32_t i);         // file offset (for ordering)
uint32_t avir_audio_chunk_offset(const AviReader* r, uint32_t j);
uint32_t avir_read_audio_chunk(AviReader* r, uint32_t j, int16_t* dst, uint32_t cap);  // samples read
// stdio buffer size; 0 = unbuffered (each chunk read is one exact SD read, skipped data costs nothing).
void     avir_set_buffer(AviReader* r, uint32_t bytes);

// Raw bytes at a file offset (returns how many were read; less at the end of the file).
uint32_t avir_read_raw(AviReader* r, uint32_t off, void* dst, uint32_t n);
uint32_t avir_file_size(const AviReader* r);

#endif
