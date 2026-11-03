#ifndef GALLERY_H
#define GALLERY_H

#include <stdbool.h>

// Build tag, shown in the gallery header (which version is on the console).
#define SLOPCAM_BUILD "b12"

// Gallery: browse photos and videos in `folder`, view photos, play videos with sound.
// Blocks until the user leaves (B / START). Returns false if the app should quit
// (HOME menu "close" / aptMainLoop ended).
bool gallery_run(const char* folder);

// UI translation (main.c)
const char* tr(const char* s);

#endif
