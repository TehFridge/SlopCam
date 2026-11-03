#ifndef AVI_H
#define AVI_H

#include <3ds.h>
#include <stdbool.h>

// Minimal AVI 1.0 writer: one MJPEG video stream + optional mono PCM16 audio.
// Header and index are patched on close; files are split by the caller
// (see avi_should_split) before AVI 1.0 offsets/players run into trouble.

typedef struct AviWriter AviWriter;

// audio_rate = 0 -> video-only file
AviWriter* avi_open(const char* path, u16 width, u16 height, u32 fps, u32 audio_rate);
// size 0 writes an empty "repeat previous frame" chunk (keeps timing constant)
bool avi_write_video(AviWriter* a, const void* jpeg, u32 size);
bool avi_write_audio(AviWriter* a, const void* pcm16, u32 bytes);
u32  avi_video_frames(const AviWriter* a);
u64  avi_audio_samples(const AviWriter* a);
bool avi_should_split(const AviWriter* a);
u32  avi_fill_permille(const AviWriter* a);   // progress to the next file split, 0..1000
u64  avi_file_bytes(const AviWriter* a);
bool avi_failed(const AviWriter* a);
u32  avi_write_kbs(const AviWriter* a);      // SD write speed so far (KB/s), 0 = unknown
void avi_close(AviWriter* a);

#endif
