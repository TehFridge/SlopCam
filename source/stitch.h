#ifndef STITCH_H
#define STITCH_H

#include <stdint.h>

// Joins the two outer-camera images (same size, packed YUYV) into one slightly wider
// image. The cameras sit ~3.5 cm apart, so each sees a thin strip the other doesn't:
// the overlap is found automatically (horizontal + small vertical offset, which camera
// is on the left, brightness difference), and the extra strip is appended with a soft
// seam. Typical gain: ~10-60 px at 640 wide (more for close subjects).

typedef struct {
    int dx;          // horizontal offset between the views in pixels (>= 0, even)
    int dy;          // vertical offset of the right view relative to the left one
    int a_is_left;   // 1 if image A is the left view
    int out_w;       // width of the stitched image (w + dx)
    int y_offset;    // brightness correction added to the right view's luma
} StitchInfo;

// Find how image B lines up with image A. Pure computation, no allocation.
StitchInfo stitch_estimate(const uint8_t* a, const uint8_t* b, int w, int h);

// Compose the stitched image into out (out_w * h * 2 bytes, packed YUYV).
void stitch_compose(const uint8_t* a, const uint8_t* b, int w, int h, const StitchInfo* s, uint8_t* out);

#endif
