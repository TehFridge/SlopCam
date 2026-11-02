SlopCam
=======
# DISCLAIMER: (I MADE THIS VIBECODED SLOP FOR FUN, WANTED TO SEE IF AI CAN MAX OUT THE 3DS RECORDING BETTER, PLZ DONT KILL ME)

3DS camera app: live viewfinder, photos, and AVI recording with sound.

Controls
- A: switch between VIDEO and PHOTO mode.
- Y: VIDEO = start/stop recording to `sdmc:/3ds_cam/VID_<date>_<time>.avi` (MJPEG + mono PCM, splits
  into `_part2.avi`... every ~1.9 GB, so no file ever gets near FAT32's 4 GB limit). While
  recording, a line under [REC] shows: File N [bar] % = how full the current file is before the
  next part starts (turns yellow at 90%), then SD free space and recording time left at the
  current data rate (h:mm, red when under 5 minutes or 200 MB). PHOTO = take a photo `sdmc:/3ds_cam/IMG_<date>_<time>.jpg`
  (with the self-timer set, Y starts / cancels the countdown).
- Up/Down: pick a setting. Left/Right: change it (hold to repeat). L/R: change in steps of 10.
- START: exit (finishes saving any recording first).

Photo mode
- Camera: Outer, Inner, or "Both: wide+3D". Both uses the two outer cameras together: the top screen
  shows a live 3D preview, and the photo is the two views stitched into one slightly wider image.
  The cameras are 3D cameras ~3.5 cm apart, so each only sees a thin extra strip: the result is
  roughly 650-740 px wide at VGA (wider for close subjects). Alignment, which camera is left and
  the brightness difference between the cameras are detected automatically for every shot.
- "Save L+R" also saves each camera's view (_L.jpg / _R.jpg) next to the stitched photo.
- Resolution, JPEG quality, chroma, self-timer (3/5/10 s) and shutter sound are separate from the
  video settings; the image controls in CAMERA (exposure, white balance, etc.) apply to both modes.
- Photos always use the fast encoder.

Settings (bottom screen, scrolls; saved automatically to `sdmc:/3ds_cam/settings.ini`, which you can also edit by hand)
- Resolution (video and photo separately): every camera size the 3DS has - 640x480 VGA, 512x384 DS x4,
  400x240 3DS top screen, 352x288 CIF, 320x240 QVGA (= 3DS bottom screen), 256x192 DS, 176x144 QCIF,
  160x120 QQVGA. The preview scales any of them to fit the top screen.
- Camera: video = Outer / Inner / Outer 3D (live 3D preview, records the left view);
  photo = Outer / Inner / Both (3D preview, stitched wider photo). Push the 3D slider up to see 3D.
  If the second camera can't keep up at the chosen size, the app first caps both cameras at 15 fps,
  then falls back to one camera and says so on the bottom screen.
- Camera image (done by the camera hardware, free): scene preset (Normal, Portrait, Landscape, Night,
  Letter), effect (None, Mono, Sepia, Negative, Negative film, Warm sepia), auto exposure on/off,
  exposure -5..+5, white balance, contrast 1..11, sharpness, noise filter, lens corner correction,
  flip (mirror / upside down / rotate 180). A scene preset sets exposure/sharpness/contrast itself.
- Software colour (costs CPU, only active when not neutral): saturation 0..200 %, brightness -64..+64.
- Recording: JPEG quality 30-100, chroma 4:2:0/4:2:2, DCT fast/accurate, file FPS 10-30,
  audio Off/8/11/16/32 kHz, mic gain 0-63.
- Rec preview: 15 fps halves the preview's CPU use while recording; Off blanks the top screen
  while recording and gives that CPU time to the encoder (best for Old 3DS + VGA).
- Preview (CAMERA): Hardware uses the 3DS's Y2R colour-conversion unit, so the CPU does no
  colour maths for the preview. At start-up a self-test converts a test card and compares it with
  the software preview, trying two ways of getting Y2R's output onto the screen:
    (T) GPU untiles/halves, CPU transposes finished pixels: small CPU cost
    (L) plain lines, CPU scaled copy: some CPU cost, works for every camera size
  The menu shows which one passed and the DMA unit sizes that worked ("Hardware (T1/8)" etc.), or why
  none did ("HW n/a: timeout" = Y2R never answered, "HW n/a: blank N" = it answered but wrote
  nothing, "HW n/a: dN bX/Y" = it wrote something that didn't match, "HW failed" =
  it stopped answering later). In every failure case the software preview is used.
  The conversion runs in the background: the menu and buttons never wait for it. If the
  hardware preview ever looks wrong, pick Software.

Encoder (RECORDING menu):
- Fast (3DS-tuned), default: fastjpeg.c, a JPEG encoder written for the ARM11. Reads the camera's
  YUYV directly, uses the ARMv6 SIMD instructions for the DCT and a branch-light coefficient scan.
  Measured on an ARM11 emulator per VGA frame: ~2.2x fewer instructions than libjpeg-turbo at
  4:2:0 q50, ~2.6x at 4:2:2 q50, ~2.1x at 4:2:2 q95, with the same image quality (within 0.2 dB).
  Zero coefficients are skipped with an exact add+compare test (no multiply stalls); each AC
  symbol (Huffman code + value bits) is a single table load with the bit buffer kept in
  registers; chroma is split out with uxtb16/pkhbt; rarely-taken code sits out of line so the hot
  loop fits the 16 KB instruction cache; the next block's camera rows are prefetched.
- libjpeg-turbo: the original library, kept as a fallback.

Static skip (RECORDING menu; Fast encoder only):
Every frame is still a complete JPEG, but 16x16 blocks that haven't changed since the
encoder last coded them are not transformed or Huffman-coded again: the encoder keeps the
pixels and the finished bits of each block, compares the new pixels with a single-instruction
SAD (usad8), and replays the old bits. On the emulator a fully static VGA frame costs about
1/5 of a normal one. A tripod / fixed shot gains the most; a handheld moving shot gains
nothing: after a frame where under 5% of blocks could be reused, it switches itself off for
8-30 frames and then checks again, so a moving shot costs only about 1% extra.
- Off: every block coded every frame.
- Low / Medium / High: how much camera noise still counts as "unchanged". Higher = more blocks
  reused = faster, but tiny slow changes (a few pixels moving slightly) may show a frame or
  so late. The difference is always measured against the pixels the reused bits came from,
  so the picture can never drift away from what the camera sees by more than that threshold.
- The top line shows "skip NN%" while recording = share of blocks reused in the last frame.
  If it stays near 0% on a still scene, the camera is noisy (dark room): try High, or turn on
  Noise filt. in CAMERA.
Uses ~1.1-1.3 MB of RAM per encoder core at VGA (the libjpeg-turbo scratch buffers, 1.2 MB per
core, are now only allocated if that encoder is actually selected).
Biggest speed knobs, in order: Encoder, Resolution, Chroma 4:2:0, File FPS. JPEG quality changes speed a bit
at high values (more non-zero coefficients to code).
The [REC] line shows real frames, late (dropped) frames and MB per minute, so you can see the effect of each tweak.

Start-up camera check
- At start-up SlopCam grabs one small frame from the left and the right outer camera. If one of
  them doesn't deliver, the app starts with the one that works: "Outer side" marks the broken
  one, 3D video and the stitched "Both" photo are shown as n/a, and the status line says
  "L cam only" / "R cam only". If neither outer camera works it uses the inner one.
- Hold R while the app starts to skip the check completely (no camera is switched on) and pick
  one by hand: L = left outer, R = right outer, X = inner, START = exit. Only that camera is
  used for the session.
- Outer side (CAMERA): which outer camera to use when not recording in 3D.

Language (CAMERA): English / Polski. The 3DS console font has no Polish letters, so Polish is
shown without diacritics.

Build: needs devkitARM + libctru + libjpeg-turbo portlib (`make`).

## Credits:
Logo and Icon: TehFridge
Everything else: Claude
