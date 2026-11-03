// SlopCam - 3DS camera viewfinder, photos and AVI (MJPEG + PCM) recording.
//
// Thread layout
//   capture  (core 0, prio 0x18) : owns the camera, fills a pool of frame slots
//   main     (core 0, prio 0x30) : input, UI, software preview synced to vblank
//   encoder  (core 1/2 if possible): one JPEG per recorded frame
//   writer   (core 0, prio 0x31) : SD card I/O, interleaves mic audio, only while recording
//
// Nothing slow ever runs in the capture thread, and every consumer always takes
// the *latest* frame, so a slow encoder or SD card drops frames instead of
// stalling the camera or building up latency.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <3ds.h>
#include <turbojpeg.h>
#include "avi.h"
#include "fastjpeg.h"
#include "gallery.h"
#include "stitch.h"
#include "audio_eq.h"

// ---------------------------------------------------------------- config
#define SAVE_FOLDER      "sdmc:/3ds_cam"

#define MAX_ENCODERS     3                    // one per usable CPU core
#define NUM_SLOTS        (4 + MAX_ENCODERS)   // recv + latest + preview + spare + one per encoder
#define SLOT_BYTES       (640 * 480 * 2)

#define REC_QUEUE        64                   // max frames waiting for the SD card
#define REC_POOL_BYTES   (6 * 1024 * 1024)    // max JPEG bytes waiting (absorbs SD latency spikes)
#define TICKET_WINDOW    4096                 // dropped-frame table (indexed by ticket, can't overflow)

#define MIC_BUF_SIZE     0x60000              // ~6 s of 32 kHz audio of slack for the SD writer
#define SETTINGS_PATH    SAVE_FOLDER "/settings.ini"


#define WAIT_TIMEOUT_RC  0x09401BFE

// ---------------------------------------------------------------- settings
// Every tweakable value lives in this table: it drives the menu, the help line,
// what gets locked while recording, and settings.ini on the SD card.

// Every camera resolution the 3DS supports, in CAMU_Size order (so settings.ini indices
// stay valid). SIZE_CTR_BOTTOM_LCD is the same as QVGA (320x240).
static const char* const L_RES[]    = {"640x480 VGA", "320x240 QVGA", "160x120 QQVGA", "352x288 CIF",
                                       "176x144 QCIF", "256x192 DS", "512x384 DS x4", "400x240 3DS top"};
static const u16  val_res_w[]       = {640, 320, 160, 352, 176, 256, 512, 400};
static const u16  val_res_h[]       = {480, 240, 120, 288, 144, 192, 384, 240};
static const CAMU_Size val_cam_size[] = {SIZE_VGA, SIZE_QVGA, SIZE_QQVGA, SIZE_CIF, SIZE_QCIF,
                                         SIZE_DS_LCD, SIZE_DS_LCDx4, SIZE_CTR_TOP_LCD};
#define NUM_RES 8

static const char* L_CAM[]          = {"Outer", "Inner", "Outer 3D (rec L)"};   // [2] changes if 3D is unavailable
static const char* L_SIDE[]         = {"Left", "Right"};                         // marked if that camera is broken
static const char* const L_LANG[]   = {"English", "Polski"};
static const char* const L_CAMFPS[] = {"30", "30-10 auto", "20", "15", "15-5 auto", "10"};
static const CAMU_FrameRate val_camfps[] = {FRAME_RATE_30, FRAME_RATE_30_TO_10, FRAME_RATE_20,
                                            FRAME_RATE_15, FRAME_RATE_15_TO_5, FRAME_RATE_10};
static const char* const L_EFFECT[] = {"None", "Mono", "Sepia", "Negative", "Negative film", "Sepia (warm)"};
static const CAMU_Effect val_effects[] = {EFFECT_NONE, EFFECT_MONO, EFFECT_SEPIA, EFFECT_NEGATIVE,
                                          EFFECT_NEGAFILM, EFFECT_SEPIA01};
static const char* const L_SCENE[]  = {"Normal", "Portrait", "Landscape", "Night", "Letter (text)"};
static const CAMU_PhotoMode val_scene[] = {PHOTO_MODE_NORMAL, PHOTO_MODE_PORTRAIT, PHOTO_MODE_LANDSCAPE,
                                           PHOTO_MODE_NIGHTVIEW, PHOTO_MODE_LETTER};
static const char* const L_CONTRAST[] = {"1 (flattest)", "2", "3", "4", "5 (low)", "6 (normal)", "7 (high)",
                                         "8", "9", "10", "11 (punchiest)"};
static const char* const L_LENS[]   = {"Off (dark edges)", "70% (normal)", "90% (bright)"};
static const CAMU_LensCorrection val_lens[] = {LENS_CORRECTION_OFF, LENS_CORRECTION_ON_70, LENS_CORRECTION_ON_90};
static const char* const L_FLIP[]   = {"None", "Mirror", "Upside down", "Rotate 180"};
static const CAMU_Flip val_flip[]   = {FLIP_NONE, FLIP_HORIZONTAL, FLIP_VERTICAL, FLIP_REVERSE};
static const char* const L_WB[]     = {"Auto", "3200K tungsten", "4150K fluor.", "5200K daylight", "6000K cloudy", "7000K shade"};
static const char* const L_ONOFF[]  = {"Off", "On"};
static const char* const L_LOWCUT[] = {"Off", "80 Hz", "120 Hz", "200 Hz"};
static const char* const L_CHROMA[] = {"4:2:0 (smaller)", "4:2:2 (full)"};
static const char* const L_DCT[]    = {"Fast", "Accurate"};
static const char* const L_ENC[]    = {"Fast (3DS-tuned)", "libjpeg-turbo"};
static const char* const L_SKIP[]   = {"Off", "Low", "Medium", "High"};
// Static skip thresholds: max sum of |pixel change| per 8-pixel row segment (16 YUYV bytes)
// for a macroblock to count as unchanged. ~2.5 / 4 / 6 levels per byte on average.
static const int SKIP_THRESH[4] = { 0, 40, 64, 96 };
static const char* L_PCAM[]         = {"Outer", "Inner", "Both: wide+3D"};
static const char* const L_TIMER[]  = {"Off", "3 s", "5 s", "10 s"};
static const int val_timer_ms[]     = {0, 3000, 5000, 10000};
static const char* const L_RPREV[]  = {"30 fps", "15 fps (faster)", "Off (fastest)"};
static const char* L_PVHW[]         = {"Hardware", "Software"};   // [0] is changed if the self-test fails
static const char* const L_RECFPS[] = {"30", "25", "24", "20", "15", "10"};
static const u32 val_recfps[]       = {30, 25, 24, 20, 15, 10};
static const char* const L_AUDIO[]  = {"Off", "8 kHz", "11 kHz", "16 kHz", "32 kHz (max)"};
static const MICU_SampleRate val_audio_rate[] = {0, MICU_SAMPLE_RATE_8180, MICU_SAMPLE_RATE_10910,
                                                 MICU_SAMPLE_RATE_16360, MICU_SAMPLE_RATE_32730};
static const u32 val_audio_hz[]     = {0, 8182, 10909, 16364, 32728};   // real hardware rates

// current values (defaults = max quality recording)
static int s_res = 0, s_cam = 0, s_camfps = 0, s_effect = 0, s_exposure = 0;
static int s_wb = 0, s_sharp = 0, s_contrast = 6, s_noise = 1;
static int s_scene = 0, s_autoexp = 1, s_lens = 1, s_flip = 0;
static int s_sat = 100, s_bright = 0;  // software colour: saturation %, brightness offset
static int s_eq_on = 0, s_eq[MICEQ_BANDS] = { 0, 0, 0, 0, 0 }, s_eq_lowcut = 0, s_eq_boost = 0, s_eq_limit = 1;
static int s_rec_q = 95, s_rec_chroma = 1, s_rec_dct = 1, s_rec_fps = 0, s_audio = 4, s_mic_gain = 40;
static int s_encoder = 0;              // 0 = fastjpeg (ARM11 SIMD), 1 = libjpeg-turbo
static int s_skip = 2;                 // static-block skip (fast encoder, video)
static int s_mode = 0;                 // 0 = video, 1 = photo
static int s_rec_prev = 1;             // preview while recording: 0 = 30 fps, 1 = 15 fps, 2 = off
static int s_pv_hw = 0;
static int s_outer = 0;                // single outer camera: 0 = left (OUT1), 1 = right (OUT2)
static int s_lang = 0;                 // 0 = English, 1 = Polski                // preview conversion: 0 = Y2R+GPU hardware when it works, 1 = software
static int s_photo_res = 0, s_photo_cam = 0, s_photo_q = 95, s_photo_chroma = 1;
static int s_photo_timer = 0, s_shutter = 1, s_photo_pair = 0;

enum { SF_LOCK = 1, SF_RECONF = 2, SF_IMAGE = 4, SF_MIC_RATE = 8, SF_MIC_GAIN = 16,
       SF_VIDEO = 32,       // only shown in video mode
       SF_HIDDEN = 64,      // saved in settings.ini but not in the menu
       SF_DUAL = 128,       // may switch the dual-camera setup
       SF_LUT = 256,        // software colour tables need rebuilding
       SF_EQ = 512,         // mic EQ changed
       SF_DB = 1024,        // show the number as "+N dB"
       SF_PAUSE = 2048 };   // locked while recording, but may change while paused
enum { G_CAMERA, G_RECORD, G_PHOTO, G_MICEQ };

typedef struct {
    const char* name;
    const char* key;
    int* v;
    int min, max;
    const char* const* labels;   // NULL = show number
    int flags, group;
    const char* help;
} Setting;

// ---- Polish UI (ASCII only: the 3DS console font has no Polish letters)
typedef struct { const char* en; const char* pl; } Tr;
static const Tr tr_tab[] = {
    {"Mode", "Tryb"},
    {"Resolution", "Rozdzielcz."},
    {"Camera", "Kamera"},
    {"Outer side", "Kam. zewn."},
    {"Cam FPS", "FPS kamery"},
    {"Scene", "Scena"},
    {"Effect", "Efekt"},
    {"Auto expo.", "Auto ekspo."},
    {"Exposure", "Ekspozycja"},
    {"White bal.", "Balans biel"},
    {"Contrast", "Kontrast"},
    {"Sharpness", "Ostrosc"},
    {"Noise filt.", "Filtr szumu"},
    {"Lens corr.", "Kor. obiekt"},
    {"Flip", "Odbicie"},
    {"Saturation", "Nasycenie"},
    {"Brightness", "Jasnosc"},
    {"Preview", "Podglad"},
    {"Language", "Jezyk"},
    {"Encoder", "Koder"},
    {"Static skip", "Pomin stat."},
    {"JPEG qual.", "Jakosc JPEG"},
    {"File FPS", "FPS pliku"},
    {"Audio", "Dzwiek"},
    {"Mic gain", "Wzmoc. mikr"},
    {"Rec preview", "Podgl. nagr"},
    {"Low cut", "Filtr niski"},
    {"Boost", "Wzmocnienie"},
    {"Timer", "Samowyzw."},
    {"Shutter snd", "Dzw. migaw."},
    {"Save L+R", "Zapisz L+P"},
    {"CAMERA", "KAMERA"},
    {"RECORDING", "NAGRYWANIE"},
    {"PHOTO", "ZDJECIA"},
    {"MIC EQ", "EQ MIKROFONU"},
    {"Outer", "Zewnetrzna"},
    {"Inner", "Wewnetrzna"},
    {"Outer 3D (rec L)", "Zewn. 3D (nagr L)"},
    {"3D (n/a: 1 cam)", "3D (brak: 1 kam)"},
    {"Both: wide+3D", "Obie: szer.+3D"},
    {"Both (n/a: 1 cam)", "Obie (brak 1 kam)"},
    {"Left", "Lewa"},
    {"Right", "Prawa"},
    {"Left (broken)", "Lewa (zepsuta)"},
    {"Right (broken)", "Prawa (zepsuta)"},
    {"None", "Brak"},
    {"Negative", "Negatyw"},
    {"Negative film", "Negatyw filmowy"},
    {"Sepia (warm)", "Sepia (ciepla)"},
    {"Normal", "Normalna"},
    {"Portrait", "Portret"},
    {"Landscape", "Krajobraz"},
    {"Night", "Noc"},
    {"Letter (text)", "Tekst (dokument)"},
    {"1 (flattest)", "1 (najlagodniej)"},
    {"5 (low)", "5 (niski)"},
    {"6 (normal)", "6 (normalny)"},
    {"7 (high)", "7 (wysoki)"},
    {"11 (punchiest)", "11 (najmocniej)"},
    {"Off (dark edges)", "Wyl (ciemne rogi)"},
    {"70% (normal)", "70% (normalna)"},
    {"90% (bright)", "90% (jasna)"},
    {"Mirror", "Lustro"},
    {"Upside down", "Do gory nogami"},
    {"Rotate 180", "Obrot 180"},
    {"3200K tungsten", "3200K zarowka"},
    {"4150K fluor.", "4150K swietlow."},
    {"5200K daylight", "5200K dzien"},
    {"6000K cloudy", "6000K pochmurno"},
    {"7000K shade", "7000K cien"},
    {"Off", "Wyl"},
    {"On", "Wl"},
    {"4:2:0 (smaller)", "4:2:0 (mniejsze)"},
    {"4:2:2 (full)", "4:2:2 (pelne)"},
    {"Fast", "Szybka"},
    {"Accurate", "Dokladna"},
    {"Fast (3DS-tuned)", "Szybki (pod 3DS)"},
    {"Low", "Niski"},
    {"Medium", "Sredni"},
    {"High", "Wysoki"},
    {"15 fps (faster)", "15 fps (szybciej)"},
    {"Off (fastest)", "Wyl (najszybciej)"},
    {"Hardware", "Sprzetowy"},
    {"Software", "Programowy"},
    {"32 kHz (max)", "32 kHz (maks)"},
    {"400x240 3DS top", "400x240 gorny 3DS"},
    {"Any 3DS camera size; bigger = slower", "Kazdy rozmiar kamery; wiekszy = wolniej"},
    {"3D = both outer cams, 3D preview", "3D = obie kamery zewn., podglad 3D"},
    {"Which outer camera (when not 3D)", "Ktora kamera zewnetrzna (bez 3D)"},
    {"auto = slower in the dark, less noise", "auto = wolniej w ciemnosci, mniej szumu"},
    {"Preset; overrides expo/sharp/contrast", "Preset; zastepuje ekspo/ostrosc/kontr."},
    {"Color effect applied by the camera", "Efekt koloru nakladany przez kamere"},
    {"Off = exposure stays where it is", "Wyl = ekspozycja zostaje jak jest"},
    {"Brightness bias for auto exposure", "Korekta jasnosci auto ekspozycji"},
    {"Auto, or fix it for steady colors", "Auto, albo stale dla rownych kolorow"},
    {"Camera tone curve, 11 steps", "Krzywa tonalna kamery, 11 krokow"},
    {"Higher = crisper but noisier / bigger", "Wyzej = ostrzej, ale wiecej szumu"},
    {"On = cleaner image, smaller files", "Wl = czystszy obraz, mniejsze pliki"},
    {"Brightens the darker image corners", "Rozjasnia ciemniejsze rogi obrazu"},
    {"Mirror / upside down (done by camera)", "Lustro / do gory nogami (w kamerze)"},
    {"% color; 0 = B&W, 100 = normal (CPU)", "% koloru; 0 = cz-b, 100 = normalnie"},
    {"Lift/darken image; 0 = off (CPU)", "Rozjasnij/przyciemnij; 0 = wyl (CPU)"},
    {"Hardware = Y2R+GPU, frees the CPU", "Sprzetowy = Y2R+GPU, odciaza CPU"},
    {"English / Polski", "English / Polski"},
    {"Fast = ~2x quicker, same image quality", "Szybki = ~2x szybciej, ta sama jakosc"},
    {"Reuse unchanged blocks: big win on tripod", "Reuzywa stale bloki: zysk na statywie"},
    {"Higher = better image, bigger files", "Wyzej = lepszy obraz, wieksze pliki"},
    {"4:2:2 keeps all color the camera gives", "4:2:2 zachowuje caly kolor z kamery"},
    {"libjpeg only (Fast enc. is always precise)", "Tylko libjpeg (Szybki zawsze dokladny)"},
    {"AVI frame rate; lower = less encoding", "Klatki AVI; mniej = mniej kodowania"},
    {"Mic sample rate (16-bit mono PCM)", "Probkowanie mikr. (16-bit mono PCM)"},
    {"Raise if quiet, lower if it clips", "Podnies gdy cicho, obniz gdy przester"},
    {"Lower/off frees CPU for encoding (O3DS)", "Mniej/wyl = wiecej CPU na kodowanie"},
    {"Tone shaping for the recorded audio", "Barwa nagrywanego dzwieku"},
    {"Bass (low shelf)", "Bas (polka niska)"},
    {"Warmth; cut if boomy/muddy", "Cieplo; obetnij gdy dudni/mula"},
    {"Body; cut if nasal/honky", "Pelnia; obetnij gdy nosowo"},
    {"Voice clarity / presence", "Wyraznosc glosu / obecnosc"},
    {"Air; cut if hissy (needs 32 kHz)", "Powietrze; obetnij gdy syczy (32 kHz)"},
    {"Removes rumble / handling noise", "Usuwa dudnienie / szum trzymania"},
    {"Extra loudness after the EQ", "Dodatkowa glosnosc po EQ"},
    {"Soft-limits peaks instead of clipping", "Lagodnie ogranicza szczyty"},
    {"Any 3DS camera size; VGA = largest", "Kazdy rozmiar kamery; VGA = najwiekszy"},
    {"Both = 2 outer cams stitched wider", "Obie = 2 kamery zewn. sklejone szerzej"},
    {"Higher = better image, bigger file", "Wyzej = lepszy obraz, wiekszy plik"},
    {"Self-timer before the shot", "Samowyzwalacz przed zdjeciem"},
    {"Play the system shutter sound", "Systemowy dzwiek migawki"},
    {"Both cams: also save each view", "Obie kamery: zapisz tez kazdy widok"},
    {"\x1b[33m[SAVING...]\x1b[0m\x1b[K\n", "\x1b[33m[ZAPISYWANIE...]\x1b[0m\x1b[K\n"},
    {"\x1b[31m[SD WRITE ERROR - recording stopped]\x1b[0m\x1b[K\n", "\x1b[31m[BLAD ZAPISU SD - nagrywanie stop]\x1b[0m\x1b[K\n"},
    {"not enough memory", "za malo pamieci"},
    {"2nd camera not delivering", "2. kamera nie daje obrazu"},
    {"\x1b[31m3D off:\x1b[0m %s\x1b[K\n", "\x1b[31m3D wyl:\x1b[0m %s\x1b[K\n"},
    {"\x1b[33m[PHOTO]\x1b[0m timer: %lu   (Y = cancel)\x1b[K\n", "\x1b[33m[FOTO]\x1b[0m timer: %lu   (Y = anuluj)\x1b[K\n"},
    {"\x1b[32m[PHOTO]\x1b[0m %s\x1b[K\n", "\x1b[32m[FOTO]\x1b[0m %s\x1b[K\n"},
    {"ready - Y to shoot", "gotowe - Y = zdjecie"},
    {"\x1b[32m[VIDEO]\x1b[0m ready%s%s%s\x1b[K\n", "\x1b[32m[WIDEO]\x1b[0m gotowe%s%s%s\x1b[K\n"},
    {" (audio off)", " (bez dzwieku)"},
    {" (no mic)", " (brak mikr.)"},
    {" 3D 15fps", " 3D 15fps"},
    {" 3D on", " 3D wl"},
    {" L cam only", " tylko kam. L"},
    {" R cam only", " tylko kam. P"},
    {" inner only", " tylko wewn."},
    {"A video  Y shoot  SEL gallery", "A wideo  Y foto  SEL galeria"},
    {"A photo  Y record  SEL gallery", "A foto  Y nagrywaj  SEL galeria"},
    {"GALLERY", "GALERIA"}, {"Reading index...", "Czytanie indeksu..."}, {"files", "plikow"}, {"photo", "zdjecie"},
    {" no sound", " bez dzwieku"}, {" (cut off)", " (urwane)"},
    {"Can't open this file", "Nie mozna otworzyc pliku"},
    {"No photos or videos yet", "Brak zdjec i filmow"},
    {"A play/pause  B back  </> seek  ^v pick", "A graj/pauza  B wroc  </> 5s  ^v wybor"},
    {"PAUSED", "PAUZA"}, {"PLAY", "GRA"}, {"STOP", "STOP"}, {" (no DSP firm.)", " (brak DSP firm.)"},
    {"Y stop  X pause/resume  L/R x10", "Y stop  X pauza/wznow  L/R x10"},
    {"\x1b[33m[PAUSED]\x1b[0m %02lu:%02lu  X = resume, Y = stop\x1b[K\n", "\x1b[33m[PAUZA]\x1b[0m %02lu:%02lu  X = wznow, Y = stop\x1b[K\n"},
    {"\x1b[33m[PAUSED]\x1b[0m %02lu:%02lu  camera starting...\x1b[K\n", "\x1b[33m[PAUZA]\x1b[0m %02lu:%02lu  start kamery...\x1b[K\n"},
    {"\x1b[33m[PAUSING...]\x1b[0m %02lu:%02lu\x1b[K\n", "\x1b[33m[PAUZOWANIE...]\x1b[0m %02lu:%02lu\x1b[K\n"},
    {"File%2d [%s%s\x1b[0m]%3lu%%", "Plik%2d [%s%s\x1b[0m]%3lu%%"},
    {"No camera frame yet", "Brak klatki z kamery"},
    {"Out of memory", "Brak pamieci"},
    {"Saved %dx%d (+%d px wider)", "Zapisano %dx%d (+%d px szerzej)"},
    {"Saved %dx%d", "Zapisano %dx%d"},
    {"Save failed (SD full?)", "Blad zapisu (pelna karta SD?)"},
    {"Not enough memory for 2 cams", "Za malo pamieci na 2 kamery"},
    {"\x1b[1;0H\x1b[33m[PHOTO] saving...\x1b[0m\x1b[K", "\x1b[1;0H\x1b[33m[FOTO] zapisywanie...\x1b[0m\x1b[K"},
    {"\x1b[2J\x1b[0;0HSaving video...\n", "\x1b[2J\x1b[0;0HZapisywanie wideo...\n"},
};
const char* tr(const char* s) {
    if (!s_lang || !s) return s;
    for (unsigned i = 0; i < sizeof(tr_tab) / sizeof(tr_tab[0]); i++)
        if (!strcmp(s, tr_tab[i].en)) return tr_tab[i].pl;
    return s;
}

static const Setting settings[] = {
    {"Mode",       "mode",       &s_mode,      0, 1,   NULL,     SF_HIDDEN, G_CAMERA, ""},
    {"Resolution", "res",        &s_res,       0, NUM_RES - 1, L_RES, SF_LOCK | SF_RECONF | SF_VIDEO, G_CAMERA, "Any 3DS camera size; bigger = slower"},
    {"Camera",     "camera",     &s_cam,       0, 2,   L_CAM,    SF_LOCK | SF_PAUSE | SF_RECONF | SF_VIDEO | SF_DUAL, G_CAMERA, "3D = both outer cams, 3D preview"},
    {"Outer side", "outer_side", &s_outer,     0, 1,   L_SIDE,   SF_LOCK | SF_PAUSE | SF_RECONF, G_CAMERA, "Which outer camera (when not 3D)"},
    {"Cam FPS",    "cam_fps",    &s_camfps,    0, 5,   L_CAMFPS, SF_LOCK | SF_PAUSE | SF_RECONF, G_CAMERA, "auto = slower in the dark, less noise"},
    {"Scene",      "scene",      &s_scene,     0, 4,   L_SCENE,  SF_IMAGE, G_CAMERA, "Preset; overrides expo/sharp/contrast"},
    {"Effect",     "effect",     &s_effect,    0, 5,   L_EFFECT, SF_IMAGE, G_CAMERA, "Color effect applied by the camera"},
    {"Auto expo.", "auto_exposure",&s_autoexp, 0, 1,   L_ONOFF,  SF_IMAGE, G_CAMERA, "Off = exposure stays where it is"},
    {"Exposure",   "exposure",   &s_exposure, -5, 5,   NULL,     SF_IMAGE, G_CAMERA, "Brightness bias for auto exposure"},
    {"White bal.", "wb",         &s_wb,        0, 5,   L_WB,     SF_IMAGE, G_CAMERA, "Auto, or fix it for steady colors"},
    {"Contrast",   "contrast",   &s_contrast,  1, 11,  L_CONTRAST, SF_IMAGE, G_CAMERA, "Camera tone curve, 11 steps"},
    {"Sharpness",  "sharpness",  &s_sharp,    -4, 5,   NULL,     SF_IMAGE, G_CAMERA, "Higher = crisper but noisier / bigger"},
    {"Noise filt.","noise",      &s_noise,     0, 1,   L_ONOFF,  SF_IMAGE, G_CAMERA, "On = cleaner image, smaller files"},
    {"Lens corr.", "lens",       &s_lens,      0, 2,   L_LENS,   SF_IMAGE, G_CAMERA, "Brightens the darker image corners"},
    {"Flip",       "flip",       &s_flip,      0, 3,   L_FLIP,   SF_IMAGE, G_CAMERA, "Mirror / upside down (done by camera)"},
    {"Saturation", "saturation", &s_sat,       0, 200, NULL,     SF_LUT,   G_CAMERA, "% color; 0 = B&W, 100 = normal (CPU)"},
    {"Brightness", "brightness", &s_bright,  -64, 64,  NULL,     SF_LUT,   G_CAMERA, "Lift/darken image; 0 = off (CPU)"},
    {"Preview",    "preview_hw", &s_pv_hw,     0, 1,   L_PVHW,   0,       G_CAMERA, "Hardware = Y2R+GPU, frees the CPU"},
    {"Language",   "language",   &s_lang,      0, 1,   L_LANG,   0,       G_CAMERA, "English / Polski"},
    {"Encoder",    "encoder",    &s_encoder,   0, 1,   L_ENC,    0,       G_RECORD, "Fast = ~2x quicker, same image quality"},
    {"Static skip","static_skip",&s_skip,      0, 3,   L_SKIP,   0,       G_RECORD, "Reuse unchanged blocks: big win on tripod"},
    {"JPEG qual.", "rec_q",      &s_rec_q,    30, 100, NULL,     0,       G_RECORD, "Higher = better image, bigger files"},
    {"Chroma",     "rec_chroma", &s_rec_chroma,0, 1,   L_CHROMA, 0,       G_RECORD, "4:2:2 keeps all color the camera gives"},
    {"DCT",        "rec_dct",    &s_rec_dct,   0, 1,   L_DCT,    0,       G_RECORD, "libjpeg only (Fast enc. is always precise)"},
    {"File FPS",   "rec_fps",    &s_rec_fps,   0, 5,   L_RECFPS, SF_LOCK,  G_RECORD, "AVI frame rate; lower = less encoding"},
    {"Audio",      "audio",      &s_audio,     0, 4,   L_AUDIO,  SF_LOCK | SF_MIC_RATE, G_RECORD, "Mic sample rate (16-bit mono PCM)"},
    {"Mic gain",   "mic_gain",   &s_mic_gain,  0, 63,  NULL,     SF_MIC_GAIN, G_RECORD, "Raise if quiet, lower if it clips"},
    {"Rec preview","rec_preview",&s_rec_prev,  0, 2,   L_RPREV,  0,       G_RECORD, "Lower/off frees CPU for encoding (O3DS)"},
    {"EQ",         "eq_on",      &s_eq_on,     0, 1,   L_ONOFF,  SF_EQ,   G_MICEQ, "Tone shaping for the recorded audio"},
    {"100 Hz",     "eq_100",     &s_eq[0],   -12, 12,  NULL,     SF_EQ | SF_DB, G_MICEQ, "Bass (low shelf)"},
    {"300 Hz",     "eq_300",     &s_eq[1],   -12, 12,  NULL,     SF_EQ | SF_DB, G_MICEQ, "Warmth; cut if boomy/muddy"},
    {"1 kHz",      "eq_1k",      &s_eq[2],   -12, 12,  NULL,     SF_EQ | SF_DB, G_MICEQ, "Body; cut if nasal/honky"},
    {"3 kHz",      "eq_3k",      &s_eq[3],   -12, 12,  NULL,     SF_EQ | SF_DB, G_MICEQ, "Voice clarity / presence"},
    {"8 kHz",      "eq_8k",      &s_eq[4],   -12, 12,  NULL,     SF_EQ | SF_DB, G_MICEQ, "Air; cut if hissy (needs 32 kHz)"},
    {"Low cut",    "eq_lowcut",  &s_eq_lowcut, 0, 3,   L_LOWCUT, SF_EQ,   G_MICEQ, "Removes rumble / handling noise"},
    {"Boost",      "eq_boost",   &s_eq_boost,  0, 24,  NULL,     SF_EQ | SF_DB, G_MICEQ, "Extra loudness after the EQ"},
    {"Limiter",    "eq_limit",   &s_eq_limit,  0, 1,   L_ONOFF,  SF_EQ,   G_MICEQ, "Soft-limits peaks instead of clipping"},
    {"Resolution", "photo_res",  &s_photo_res, 0, NUM_RES - 1, L_RES, SF_RECONF, G_PHOTO, "Any 3DS camera size; VGA = largest"},
    {"Camera",     "photo_cam",  &s_photo_cam, 0, 2,   L_PCAM,   SF_RECONF | SF_DUAL, G_PHOTO, "Both = 2 outer cams stitched wider"},
    {"JPEG qual.", "photo_q",    &s_photo_q,  30, 100, NULL,     0,       G_PHOTO, "Higher = better image, bigger file"},
    {"Chroma",     "photo_chroma",&s_photo_chroma,0,1, L_CHROMA, 0,       G_PHOTO, "4:2:2 keeps all color the camera gives"},
    {"Timer",      "photo_timer",&s_photo_timer,0, 3,  L_TIMER,  0,       G_PHOTO, "Self-timer before the shot"},
    {"Shutter snd","shutter",    &s_shutter,   0, 1,   L_ONOFF,  0,       G_PHOTO, "Play the system shutter sound"},
    {"Save L+R",   "photo_pair", &s_photo_pair,0, 1,   L_ONOFF,  0,       G_PHOTO, "Both cams: also save each view"},
};
#define NUM_SETTINGS ((int)(sizeof(settings) / sizeof(settings[0])))
static int menu_cursor = 1;

static bool setting_visible(int i) {
    const Setting* st = &settings[i];
    if (st->flags & SF_HIDDEN) return false;
    if (s_mode == 1 && ((st->flags & SF_VIDEO) || st->group == G_RECORD)) return false;
    if (s_mode == 0 && st->group == G_PHOTO) return false;
    if (s_mode == 1 && st->group == G_MICEQ) return false;          // EQ is for recorded audio
    return true;
}

static int menu_step(int from, int dir) {
    int i = from;
    for (int n = 0; n < NUM_SETTINGS; n++) {
        i = (i + dir + NUM_SETTINGS) % NUM_SETTINGS;
        if (setting_visible(i)) return i;
    }
    return from;
}

static void settings_clamp(void) {
    for (int i = 0; i < NUM_SETTINGS; i++) {
        const Setting* st = &settings[i];
        if (*st->v < st->min) *st->v = st->min;
        if (*st->v > st->max) *st->v = st->max;
    }
}

static void settings_load(void) {
    FILE* f = fopen(SETTINGS_PATH, "r");
    if (!f) return;
    char line[96], key[48];
    int val;
    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, " %47[^= ] = %d", key, &val) != 2) continue;
        for (int i = 0; i < NUM_SETTINGS; i++)
            if (!strcmp(key, settings[i].key)) *settings[i].v = val;
    }
    fclose(f);
    settings_clamp();
}

static void settings_save(void) {
    FILE* f = fopen(SETTINGS_PATH, "w");
    if (!f) return;
    fprintf(f, "# SlopCam settings (edit here or in the app)\n");
    for (int i = 0; i < NUM_SETTINGS; i++) fprintf(f, "%s=%d\n", settings[i].key, *settings[i].v);
    fclose(f);
}

// ---------------------------------------------------------------- frame pool
typedef struct {
    u8* data;
    u8* data2;      // second outer camera (dual mode only, allocated on first use)
    u16 w, h;
    u8 dual;        // data2 holds the matching frame from the other camera
    u32 seq;
    u64 tick;
} FrameSlot;

static FrameSlot slots[NUM_SLOTS];
static int slot_users[NUM_SLOTS];
static int slot_latest = -1;
static int slot_recv = -1;
static u32 frame_seq = 0;
static LightLock slot_lock;
static LightEvent enc_event;

static volatile bool running = true;
static volatile bool cam_reconfig = true;
static volatile bool cam_suspend = false;   // gallery open: camera off, capture thread idle
static volatile u32 stat_cap = 0, stat_enc = 0, stat_prev = 0;
static volatile int stat_skip_pct = -1;  // static skip: % of macroblocks reused in the last recorded frame

static int pick_free_slot(void) {
    for (int i = 0; i < NUM_SLOTS; i++)
        if (i != slot_latest && i != slot_recv && slot_users[i] == 0) return i;
    return -1;
}

// Returns a slot index holding a frame newer than *last_seq, or -1.
static int acquire_latest(u32* last_seq) {
    int idx = -1;
    LightLock_Lock(&slot_lock);
    if (slot_latest >= 0 && slots[slot_latest].seq != *last_seq) {
        idx = slot_latest;
        slot_users[idx]++;
        *last_seq = slots[idx].seq;
    }
    LightLock_Unlock(&slot_lock);
    return idx;
}

static void release_slot(int idx) {
    LightLock_Lock(&slot_lock);
    slot_users[idx]--;
    LightLock_Unlock(&slot_lock);
}

// ---------------------------------------------------------------- camera
static int  eff_res(void)  { return s_mode ? s_photo_res : s_res; }
// Which cameras work, from the start-up check (or the hold-R chooser).
static bool cam_ok_l = true, cam_ok_r = true, cam_ok_in = true;
static bool dual_avail(void)  { return cam_ok_l && cam_ok_r; }
static bool outer_avail(void) { return cam_ok_l || cam_ok_r; }
// 0 outer, 1 inner, 2 both outer - the setting, adjusted to the cameras that work
static int  eff_cam(void) {
    int c = s_mode ? s_photo_cam : s_cam;
    if (c == 2 && !dual_avail()) c = 0;
    if (c == 0 && !outer_avail()) c = 1;
    if (c == 1 && !cam_ok_in && outer_avail()) c = 0;
    return c;
}
static int  outer_side(void) { return (s_outer == 1 && cam_ok_r) || !cam_ok_l ? 1 : 0; }   // 1 = right (OUT2)
static bool want_dual(void) { return eff_cam() == 2; }
// Short note for the status line when not every camera is available.
static const char* cam_note(void) {
    if (dual_avail() && cam_ok_in) return "";
    if (!outer_avail()) return " inner only";
    if (!dual_avail()) return cam_ok_l ? " L cam only" : " R cam only";
    return "";
}
static volatile bool dual_ok = false;   // second buffers allocated (main thread sets this)
// 0 = dual as configured, 1 = dual capped at 15 fps, 2 = gave up -> single camera.
// Raised by the capture thread when the second camera keeps failing; reset by apply_mode().
static volatile int dual_level = 0;
static u32 cam_select(void) {
    int c = eff_cam();
    return c == 2 ? SELECT_OUT1_OUT2 : c == 1 ? SELECT_IN1 : (outer_side() ? SELECT_OUT2 : SELECT_OUT1);
}
// Port of a single camera: the right outer camera (OUT2) is wired to port 2.
static u32 cam_port_single(void) { return eff_cam() == 0 && outer_side() ? PORT_CAM2 : PORT_CAM1; }

static void apply_camera_settings(void) {
    u32 sel = cam_select();
    // Scene preset first; manual exposure/sharpness/contrast only in Normal so the
    // preset isn't immediately overwritten.
    CAMU_SetPhotoMode(sel, val_scene[s_scene]);
    CAMU_SetEffect(sel, val_effects[s_effect], CONTEXT_A);
    CAMU_FlipImage(sel, val_flip[s_flip], CONTEXT_A);
    CAMU_SetLensCorrection(sel, val_lens[s_lens]);
    CAMU_SetWhiteBalance(sel, (CAMU_WhiteBalance)s_wb);
    CAMU_SetAutoWhiteBalance(sel, s_wb == 0);
    CAMU_SetNoiseFilter(sel, s_noise != 0);
    if (s_scene == 0) {
        CAMU_SetAutoExposure(sel, s_autoexp != 0);
        CAMU_SetExposure(sel, (s8)s_exposure);
        CAMU_SetSharpness(sel, (s8)s_sharp);
        CAMU_SetContrast(sel, (CAMU_Contrast)(s_contrast - 1));
    }
    if (sel == SELECT_OUT1_OUT2) CAMU_SetBrightnessSynchronization(true);   // matching L/R exposure
}

// ---- software colour (saturation / brightness), applied to each frame in place when
// not neutral. Two table sets; the capture thread reads the active one.
static u8 lut_y[2][256], lut_uv[2][256];
static volatile int lut_active = -1;            // -1 = neutral (skip), else table set index

static void rebuild_luts(void) {
    if (s_sat == 100 && s_bright == 0) { lut_active = -1; return; }
    int t = lut_active == 0 ? 1 : 0;            // build into the idle set, then publish
    for (int i = 0; i < 256; i++) {
        int y = i + s_bright;
        lut_y[t][i] = y < 0 ? 0 : y > 255 ? 255 : (u8)y;
        int c = 128 + ((i - 128) * s_sat) / 100;
        lut_uv[t][i] = c < 0 ? 0 : c > 255 ? 255 : (u8)c;
    }
    __dmb();
    lut_active = t;
}

static void apply_luts(u8* data, u32 bytes) {
    int t = lut_active;
    if (t < 0) return;
    const u8* ly = lut_y[t];
    const u8* lc = lut_uv[t];
    u32* p = (u32*)data;
    for (u32 i = 0, n = bytes / 4; i < n; i++) {
        u32 v = p[i];
        p[i] = ly[v & 0xFF] | ((u32)lc[(v >> 8) & 0xFF] << 8) | ((u32)ly[(v >> 16) & 0xFF] << 16) | ((u32)lc[v >> 24] << 24);
    }
    GSPGPU_FlushDataCache(data, bytes);         // no dirty lines left over the next DMA into this slot
}

static u16 cam_w = 320, cam_h = 240;
static u32 cam_transfer = 0;
static bool cam_dual = false;           // current capture setup (capture thread)
static u32 cam_port1 = PORT_CAM1;       // port used when capturing from one camera

static void configure_camera(void) {
    CAMU_Activate(SELECT_NONE);
    cam_dual = want_dual() && dual_ok && dual_level < 2;
    int r = eff_res();
    cam_w = val_res_w[r];
    cam_h = val_res_h[r];
    u32 sel = cam_dual ? SELECT_OUT1_OUT2 : cam_select();
    cam_port1 = cam_dual ? PORT_CAM1 : cam_port_single();

    CAMU_SetSize(sel, val_cam_size[r], CONTEXT_A);
    CAMU_SetOutputFormat(sel, OUTPUT_YUV_422, CONTEXT_A);
    CAMU_FrameRate fr = val_camfps[s_camfps];
    if (cam_dual && dual_level >= 1) fr = FRAME_RATE_15;          // two cameras need more bus time
    CAMU_SetFrameRate(sel, fr);
    CAMU_SetAutoExposure(sel, true);
    CAMU_SetTrimming(PORT_CAM1, false);
    CAMU_SetTrimming(PORT_CAM2, false);
    apply_camera_settings();

    CAMU_GetMaxBytes(&cam_transfer, cam_w, cam_h);
    CAMU_SetTransferBytes(cam_dual ? PORT_BOTH : cam_port1, cam_transfer, cam_w, cam_h);
    CAMU_Activate(sel);
    if (cam_dual) CAMU_SynchronizeVsyncTiming(SELECT_OUT1, SELECT_OUT2);   // both views at the same instant
}

// ---- start-up camera check: one small frame from each outer camera (and the inner one
// if neither outer works). Broken cameras are hidden from the menu for this session.
static bool probe_camera(u32 sel, u32 port, u8* buf) {
    CAMU_Activate(SELECT_NONE);
    CAMU_SetSize(sel, SIZE_QQVGA, CONTEXT_A);
    CAMU_SetOutputFormat(sel, OUTPUT_YUV_422, CONTEXT_A);
    CAMU_SetFrameRate(sel, FRAME_RATE_30);
    CAMU_SetTrimming(port, false);
    u32 tb = 0;
    CAMU_GetMaxBytes(&tb, 160, 120);
    CAMU_SetTransferBytes(port, tb, 160, 120);
    if (R_FAILED(CAMU_Activate(sel))) { CAMU_Activate(SELECT_NONE); return false; }
    CAMU_ClearBuffer(port);
    bool ok = false;
    Handle ev = 0;
    if (R_SUCCEEDED(CAMU_SetReceiving(&ev, buf, port, 160 * 120 * 2, (s16)tb))) {
        CAMU_StartCapture(port);
        ok = svcWaitSynchronization(ev, 1500000000LL) == 0;   // first frame after power-up is slow
        CAMU_StopCapture(port);
        svcCloseHandle(ev);
    }
    CAMU_Activate(SELECT_NONE);
    return ok;
}

static void update_cam_labels(void) {
    L_SIDE[0] = cam_ok_l ? "Left" : "Left (broken)";
    L_SIDE[1] = cam_ok_r ? "Right" : "Right (broken)";
    L_CAM[2]  = dual_avail() ? "Outer 3D (rec L)" : "3D (n/a: 1 cam)";
    L_PCAM[2] = dual_avail() ? "Both: wide+3D" : "Both (n/a: 1 cam)";
}

static void check_cameras(u8* buf) {
    printf("%s", s_lang ? "Sprawdzanie kamer...\n" : "Checking cameras...\n");
    gfxFlushBuffers(); gfxSwapBuffers();
    cam_ok_l = probe_camera(SELECT_OUT1, PORT_CAM1, buf);
    cam_ok_r = probe_camera(SELECT_OUT2, PORT_CAM2, buf);
    cam_ok_in = outer_avail() ? true : probe_camera(SELECT_IN1, PORT_CAM1, buf);
    update_cam_labels();
}

// Hold R while starting: no camera is touched; pick one by hand.
static bool choose_camera(void) {
    printf("\x1b[2J\x1b[0;0H");
    printf("%s", s_lang
        ? "\x1b[33mSlopCam\x1b[0m - pominieto test kamer (R)\n\nWybierz kamere:\n\n  L     = lewa zewnetrzna\n  R     = prawa zewnetrzna\n  X     = wewnetrzna\n  START = wyjscie\n"
        : "\x1b[33mSlopCam\x1b[0m - camera check skipped (R held)\n\nChoose a camera:\n\n  L     = left outer\n  R     = right outer\n  X     = inner\n  START = exit\n");
    gfxFlushBuffers(); gfxSwapBuffers();
    // wait for R to be released first, so the held R doesn't pick "right"
    while (aptMainLoop()) { hidScanInput(); if (!(hidKeysHeld() & KEY_R)) break; gspWaitForVBlank(); }
    while (aptMainLoop()) {
        hidScanInput();
        u32 k = hidKeysDown();
        if (k & KEY_START) return false;
        if (k & (KEY_L | KEY_R | KEY_X)) {
            cam_ok_l = (k & KEY_L) != 0;
            cam_ok_r = !cam_ok_l && (k & KEY_R) != 0;
            cam_ok_in = !cam_ok_l && !cam_ok_r;
            if (cam_ok_l || cam_ok_r) { s_cam = s_photo_cam = 0; s_outer = cam_ok_r; }
            else                      { s_cam = s_photo_cam = 1; }
            update_cam_labels();
            printf("\x1b[2J\x1b[0;0H");
            return true;
        }
        gspWaitForVBlank();
    }
    return false;
}

// Capture thread: one camera (PORT_CAM1), or both outer cameras (PORT_CAM1 + PORT_CAM2)
// filling the two halves of a slot. A slot is published once all ports have delivered.
static void capture_thread(void* arg) {
    (void)arg;
    Handle ev_err[2] = { 0, 0 }, ev_recv[2] = { 0, 0 };
    bool got[2] = { false, false };
    int nports = 1;
    int dual_errors = 0;                       // failures since the last complete L+R pair
    u64 half_since = 0;                        // one camera delivered, waiting for the other
    bool active = false;
    u64 last_frame_ms = osGetTime();
    u32 ports[2] = { PORT_CAM1, PORT_CAM2 };

    #define SLOT_BUF(slot, p) ((p) ? slots[slot].data2 : slots[slot].data)
    #define ARM(p, slot) CAMU_SetReceiving(&ev_recv[p], SLOT_BUF(slot, p), ports[p], cam_w * cam_h * 2, (s16)cam_transfer)
    #define CLOSE(h) do { if (h) { svcCloseHandle(h); h = 0; } } while (0)

    while (running) {
        if (cam_suspend) {                      // gallery: switch the camera off and wait
            if (active) {
                CAMU_StopCapture(PORT_BOTH);
                for (int p = 0; p < 2; p++) { CLOSE(ev_recv[p]); CLOSE(ev_err[p]); got[p] = false; }
                CAMU_Activate(SELECT_NONE);
                active = false;
            }
            svcSleepThread(50000000LL);
            continue;                           // (!active restarts it when the gallery closes)
        }
        // (Re)start the camera: on startup, on settings change, or if frames stopped
        // arriving (e.g. after sleep mode / HOME menu).
        if (cam_reconfig || !active || osGetTime() - last_frame_ms > 3000) {
            if (active) CAMU_StopCapture(PORT_BOTH);
            for (int p = 0; p < 2; p++) { CLOSE(ev_recv[p]); CLOSE(ev_err[p]); got[p] = false; }
            cam_reconfig = false;

            configure_camera();
            nports = cam_dual ? 2 : 1;
            ports[0] = cam_dual ? PORT_CAM1 : cam_port1;
            for (int p = 0; p < nports; p++) CAMU_GetBufferErrorInterruptEvent(&ev_err[p], ports[p]);
            CAMU_ClearBuffer(PORT_BOTH);

            LightLock_Lock(&slot_lock);
            slot_latest = -1;                  // old frames have the old size
            slot_recv = -1;
            slot_recv = pick_free_slot();
            LightLock_Unlock(&slot_lock);
            if (slot_recv < 0) { svcSleepThread(10000000LL); continue; }

            for (int p = 0; p < nports; p++) ARM(p, slot_recv);
            CAMU_StartCapture(nports == 2 ? PORT_BOTH : ports[0]);
            active = true;
            last_frame_ms = osGetTime();
        }

        // Wait on whatever handles are live: error events + pending receives.
        Handle evs[4]; int what[4]; int n = 0;
        for (int p = 0; p < nports; p++) {
            evs[n] = ev_err[p]; what[n++] = p;              // 0/1: error on port p
            if (ev_recv[p]) { evs[n] = ev_recv[p]; what[n++] = 2 + p; }   // 2/3: frame on port p
        }
        s32 which = -1;
        Result rc = svcWaitSynchronizationN(&which, evs, n, false, 100000000LL);
        if (!running) break;
        bool timed_out = rc == (Result)WAIT_TIMEOUT_RC || R_FAILED(rc) || which < 0 || which >= n;
        // Dual: one camera delivered but the other hasn't for 500 ms -> treat as an error.
        bool stuck_half = nports == 2 && (got[0] != got[1]) && osGetTime() - half_since > 500;
        if (timed_out && !stuck_half) continue;
        int ev = timed_out ? 0 : what[which];

        if (ev < 2) {
            if (nports == 2 && ++dual_errors >= 4) {
                // The second camera keeps failing at this setting: first cap the frame rate,
                // then fall back to one camera (the main thread shows why).
                dual_errors = 0;
                if (dual_level < 2) { dual_level++; cam_reconfig = true; continue; }
            }
            // Buffer error (we were too late re-arming): restart the transfer(s).
            CAMU_StopCapture(PORT_BOTH);
            CAMU_ClearBuffer(PORT_BOTH);
            for (int p = 0; p < nports; p++) { CLOSE(ev_recv[p]); got[p] = false; ARM(p, slot_recv); }
            CAMU_StartCapture(nports == 2 ? PORT_BOTH : ports[0]);
            continue;
        }

        int p = ev - 2;
        CLOSE(ev_recv[p]);
        if (!got[0] && !got[1]) half_since = osGetTime();
        got[p] = true;
        if (nports == 2 && !(got[0] && got[1])) continue;   // wait for the other camera
        got[0] = got[1] = false;
        dual_errors = 0;

        // Frame complete. Re-arm into a free slot first so the camera never waits.
        int done = slot_recv;
        LightLock_Lock(&slot_lock);
        int next = pick_free_slot();
        if (next < 0) next = done;             // everyone busy: overwrite (drop) this frame
        slot_recv = next;
        LightLock_Unlock(&slot_lock);
        for (int q = 0; q < nports; q++) ARM(q, next);

        last_frame_ms = osGetTime();
        if (next == done) continue;

        GSPGPU_InvalidateDataCache(slots[done].data, cam_w * cam_h * 2);
        if (nports == 2) GSPGPU_InvalidateDataCache(slots[done].data2, cam_w * cam_h * 2);
        apply_luts(slots[done].data, cam_w * cam_h * 2);             // no-op when neutral
        if (nports == 2) apply_luts(slots[done].data2, cam_w * cam_h * 2);
        slots[done].w = cam_w;
        slots[done].h = cam_h;
        slots[done].dual = nports == 2;
        slots[done].tick = svcGetSystemTick();
        LightLock_Lock(&slot_lock);
        slots[done].seq = ++frame_seq;
        slot_latest = done;
        LightLock_Unlock(&slot_lock);
        stat_cap++;
        LightEvent_Signal(&enc_event);
    }

    CAMU_StopCapture(PORT_BOTH);
    for (int q = 0; q < 2; q++) { CLOSE(ev_recv[q]); CLOSE(ev_err[q]); }
    #undef SLOT_BUF
    #undef ARM
    #undef CLOSE
}

// ---------------------------------------------------------------- preview (main thread)
// Software YUYV -> RGB565, scaled to 320x240 and written rotated straight into a top
// framebuffer. Works on 2x2 pixel blocks in framebuffer (column) order: chroma once per
// block, R/G/B for two pixels at a time with ARMv6 SIMD (sadd16 + usat16), and one
// 32-bit store per two pixels.
static s16 t_rv[256], t_guv_u[256], t_guv_v[256], t_bu[256];

static void preview_init(void) {
    for (int i = 0; i < 256; i++) {
        int c = i - 128;
        t_rv[i]    = (s16)((359 * c) >> 8);   // 1.402 V
        t_guv_u[i] = (s16)((88  * c) >> 8);   // 0.344 U
        t_guv_v[i] = (s16)((183 * c) >> 8);   // 0.714 V
        t_bu[i]    = (s16)((454 * c) >> 8);   // 1.772 U
    }
}

#if defined(__arm__) && !defined(__thumb__)
static inline u32 p_sadd16(u32 a, u32 b) { u32 r; __asm__("sadd16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline u32 p_ssub16(u32 a, u32 b) { u32 r; __asm__("ssub16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b)); return r; }
static inline u32 p_usat8(u32 a) { u32 r; __asm__("usat16 %0, #8, %1" : "=r"(r) : "r"(a)); return r; }
#else
static inline u32 p_sadd16(u32 a, u32 b) { return (u16)((s16)a + (s16)b) | ((u32)(u16)((s16)(a >> 16) + (s16)(b >> 16)) << 16); }
static inline u32 p_ssub16(u32 a, u32 b) { return (u16)((s16)a - (s16)b) | ((u32)(u16)((s16)(a >> 16) - (s16)(b >> 16)) << 16); }
static inline u32 p_usat8(u32 a) { s16 l = (s16)a, h = (s16)(a >> 16); l = l < 0 ? 0 : l > 255 ? 255 : l; h = h < 0 ? 0 : h > 255 ? 255 : h; return (u16)l | ((u32)(u16)h << 16); }
#endif

// Two RGB565 pixels from a packed Y pair (lanes) and chroma terms replicated in both lanes.
static inline u32 rgb565x2(u32 yy, u32 rv2, u32 guv2, u32 bu2) {
    u32 r = p_usat8(p_sadd16(yy, rv2));
    u32 g = p_usat8(p_ssub16(yy, guv2));
    u32 b = p_usat8(p_sadd16(yy, bu2));
    return ((b >> 3) & 0x001F001Fu) | ((r & 0x00F800F8u) << 8) | ((g & 0x00FC00FCu) << 3);
}

static inline void chroma2(u32 p, u32* rv2, u32* guv2, u32* bu2) {
    int U = (p >> 8) & 0xFF, V = p >> 24;
    u32 rv = (u16)t_rv[V], guv = (u16)(t_guv_u[U] + t_guv_v[V]), bu = (u16)t_bu[U];
    *rv2 = rv | (rv << 16); *guv2 = guv | (guv << 16); *bu2 = bu | (bu << 16);
}

// Draw one YUYV image of any camera size into fb, scaled to fit 400x240 (keeping the
// aspect ratio) and centred. Column x of the screen is fb[x*240 .. x*240+239], bottom to
// top, so screen rows y+1 and y share one 32-bit word (lo = row y+1, hi = row y).
static u16 pv_sw, pv_sh, pv_ow, pv_oh, pv_x0, pv_y0;
static u16 pv_colw[400];                // source word index for each screen column
static u8  pv_colsh[400];               // 0 = Y0, 16 = Y1 of that word
static u32 pv_rowoff[240];              // source word offset of each screen row

static void preview_setup(int sw, int sh) {
    // fit: scale = min(400/sw, 240/sh), even output size
    int ow, oh;
    if (sw * 240 > sh * 400) { ow = 400; oh = (sh * 400 / sw) & ~1; }
    else                     { oh = 240; ow = (sw * 240 / sh) & ~1; }
    pv_sw = sw; pv_sh = sh; pv_ow = ow; pv_oh = oh;
    pv_x0 = (400 - ow) / 2; pv_y0 = ((240 - oh) / 2) & ~1;   // even: rows are written in pairs
    for (int x = 0; x < ow; x++) { int sx = x * sw / ow; pv_colw[x] = sx >> 1; pv_colsh[x] = (sx & 1) * 16; }
    for (int y = 0; y < oh; y++) pv_rowoff[y] = (u32)(y * sh / oh) * (sw / 2);
}

static void preview_image(const u8* data, int sw, int sh, u16* fb) {
    if (sw != pv_sw || sh != pv_sh) preview_setup(sw, sh);
    const u32* src = (const u32*)data;
    u32 rv2, guv2, bu2;
    const int oh = pv_oh;
    // y0 is even, so row pairs stay word-aligned in the framebuffer
    // 16x16 screen tiles keep the source rows in cache while the columns are written
    for (int ty = 0; ty < oh; ty += 16)
    for (int x = 0; x < pv_ow; x += 2) {
        u32* c0 = (u32*)(fb + (pv_x0 + x) * 240);
        u32* c1 = (u32*)(fb + (pv_x0 + x + 1) * 240);
        const u32* s0 = src + pv_colw[x];
        const u32* s1 = src + pv_colw[x + 1];
        const int sh0 = pv_colsh[x], sh1 = pv_colsh[x + 1];
        const int ye = ty + 16 < oh ? ty + 16 : oh;
        for (int y = ty; y < ye; y += 2) {
            u32 r0 = pv_rowoff[y], r1 = pv_rowoff[y + 1];
            u32 a = s0[r0], b = s1[r0], e = s0[r1], f = s1[r1];
            chroma2(a, &rv2, &guv2, &bu2);
            int w = (238 - (pv_y0 + y)) >> 1;
            c0[w] = rgb565x2(((e >> sh0) & 0xFF) | (((a >> sh0) & 0xFF) << 16), rv2, guv2, bu2);
            c1[w] = rgb565x2(((f >> sh1) & 0xFF) | (((b >> sh1) & 0xFF) << 16), rv2, guv2, bu2);
        }
    }
}

// ---------------------------------------------------------------- hardware preview
// Y2R (the 3DS's YUV->RGB hardware) does the colour conversion so the CPU doesn't.
// Three ways to get its output onto the (column-ordered) top framebuffer, tried in order
// by a self-test at start-up:
//   R: Y2R rotates 90 degrees, a GX display transfer writes straight into the framebuffer
//      (2x2-averaged for VGA). No CPU work. Camera heights 240 and 480.
//   T: no rotation; GX halves/untiles into a small buffer, the CPU transposes finished RGB
//      pixels into the framebuffer (cheap, cache-friendly). Heights 240 and 480.
//   L: Y2R writes plain lines, the CPU does a scaled copy. Any size.
// The test converts a known image each way (with every plausible rotation, colour range,
// transfer layout and flip) and keeps the first way that matches the software preview.
// If none does, or Y2R stops answering later, the software preview is used.
enum { HW_NONE = 0, HW_R, HW_T, HW_L };
static int    hw_mode = HW_NONE;
static Handle y2r_end = 0;
static u16*   hw_buf = NULL;           // Y2R output, up to 640x480 RGB565
static u16*   hw_lin = NULL;           // mode T: untiled (halved) image, up to 400x240
static int    hw_rotation = ROTATION_NONE, hw_coef = COEFFICIENT_ITU_R_BT_601, hw_unit = 0, hw_flip = 0;
static int    hw_fail = 0;
static int    hw_cfg_key = -1;
static char   hw_label[24] = "Hardware";

static bool hw_pv_fits(int w, int h) {
    if (hw_mode == HW_L) return w <= 640 && h <= 480;
    return (h == 240 && w <= 400) || (h == 480 && w == 640);
}

static int hw_send_l = 1, hw_recv_l = 8;  // DMA transfer units in lines (found by the self-test)

// Configure (if needed) and start one conversion. Returns at once.
static bool y2r_start(const void* src, int w, int h, int rot, int coef, int block) {
    int key = (w << 16) ^ (h << 5) ^ (rot << 3) ^ (block << 2) ^ coef;
    if (key != hw_cfg_key) {
        Y2RU_StopConversion();                           // never configure a running unit
        Y2RU_ConversionParams p;
        memset(&p, 0, sizeof(p));
        p.input_format = INPUT_YUV422_BATCH;
        p.output_format = OUTPUT_RGB_16_565;
        p.rotation = (Y2RU_Rotation)rot;
        p.block_alignment = block ? BLOCK_8_BY_8 : BLOCK_LINE;
        p.input_line_width = (s16)w;
        p.input_lines = (s16)h;
        p.standard_coefficient = (Y2RU_StandardCoefficient)coef;
        p.alpha = 0xFF;
        if (R_FAILED(Y2RU_SetConversionParams(&p))) { hw_cfg_key = -1; return false; }
        hw_cfg_key = key;
    }
    u32 size = (u32)w * h * 2;
    if (R_FAILED(Y2RU_SetSendingYUYV(src, size, (s16)(w * 2 * hw_send_l), 0))) return false;
    int rl = block ? 8 : hw_recv_l;                      // 8x8 tile output comes in 8-line batches
    if (R_FAILED(Y2RU_SetReceiving(hw_buf, size, (s16)(w * 2 * rl), 0))) return false;
    svcClearEvent(y2r_end);
    return R_SUCCEEDED(Y2RU_StartConversion());
}

// Finished? Polls the output side (the end event can fire early, or not at all).
static bool y2r_done(void) {
    bool done = false, busy = true;
    if (R_FAILED(Y2RU_IsDoneReceiving(&done)) || !done) return false;
    if (R_FAILED(Y2RU_IsBusyConversion(&busy))) return false;
    return !busy;
}

// Blocking version (start-up self-test only).
static bool y2r_convert(const void* src, int w, int h, int rot, int coef, int block, int unit_mode, void* dst) {
    (void)unit_mode; (void)dst;
    if (!y2r_start(src, w, h, rot, coef, block)) return false;
    for (int i = 0; i < 150; i++) {                      // up to ~150 ms
        if (y2r_done()) return true;
        svcSleepThread(1000000);
    }
    Y2RU_StopConversion();
    return false;
}

// Tiled iw x ih -> linear, halved if `half`; waits for completion.
static void gx_untile(const u16* in, int iw, int ih, bool half, u16* dst, int flip) {
    int ow = half ? iw / 2 : iw, oh = half ? ih / 2 : ih;
    GX_DisplayTransfer((u32*)in, GX_BUFFER_DIM(iw, ih), (u32*)dst, GX_BUFFER_DIM(ow, oh),
                       GX_TRANSFER_FLIP_VERT(flip) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) |
                       GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGB565) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB565) |
                       GX_TRANSFER_SCALING(half ? GX_TRANSFER_SCALE_XY : GX_TRANSFER_SCALE_NO));
    gspWaitForPPF();
}

// Row-major lw x lh image -> framebuffer columns starting at column x0 (rows centred).
// 8x8 tiles keep both sides in cache; two pixels per 32-bit store.
static void transpose_to_fb(const u16* lin, int lw, int lh, u16* fb, int x0) {
    const int y0 = ((240 - lh) / 2) & ~1;
    for (int ty = 0; ty < lh; ty += 8)
        for (int tx = 0; tx < lw; tx += 8)
            for (int x = tx; x < tx + 8 && x < lw; x++) {
                u16* col = fb + (x0 + x) * 240;
                for (int y = ty; y < ty + 8 && y + 1 < lh + 1; y += 2) {
                    // screen row y -> index 239 - (y0 + y); rows y, y+1 share one word
                    u32 hi = lin[y * lw + x], lo = y + 1 < lh ? lin[(y + 1) * lw + x] : 0;
                    *(u32*)(col + 238 - (y0 + y)) = lo | (hi << 16);
                }
            }
}

// Linear RGB565 sw x sh -> framebuffer, scaled with the software preview's layout tables.
// Works in 16x16 tiles of screen pixels: the framebuffer is column-ordered, the source is
// row-ordered, so walking whole columns would miss the cache on nearly every read.
static void scaled_to_fb(const u16* src, int sw, int sh, u16* fb) {
    if (sw != pv_sw || sh != pv_sh) preview_setup(sw, sh);
    for (int tx = 0; tx < pv_ow; tx += 16) {
        const int xe = tx + 16 < pv_ow ? tx + 16 : pv_ow;
        for (int ty = 0; ty < pv_oh; ty += 16) {
            const int ye = ty + 16 < pv_oh ? ty + 16 : pv_oh;
            for (int x = tx; x < xe; x++) {
                u32* col = (u32*)(fb + (pv_x0 + x) * 240);
                const u16* s = src + pv_colw[x] * 2 + (pv_colsh[x] ? 1 : 0);
                for (int y = ty; y < ye; y += 2) {
                    u32 hi = s[pv_rowoff[y] * 2], lo = s[pv_rowoff[y + 1] * 2];
                    col[(238 - (pv_y0 + y)) >> 1] = lo | (hi << 16);
                }
            }
        }
    }
}

// Mean per-channel difference (0..255) between two RGB565 column images (sampled).
static int rgb565_diff(const u16* a, const u16* b, int cols) {
    u32 sum = 0, n = 0;
    for (int c = 0; c < cols; c += 3)
        for (int r = 0; r < 240; r += 3) {
            u16 p = a[c * 240 + r], q = b[c * 240 + r];
            int dr = (int)((p >> 11) & 31) - (int)((q >> 11) & 31);
            int dg = (int)((p >> 5) & 63) - (int)((q >> 5) & 63);
            int db = (int)(p & 31) - (int)(q & 31);
            sum += (u32)(abs(dr) * 8 + abs(dg) * 4 + abs(db) * 8);
            n += 3;
        }
    return n ? (int)(sum / n) : 255;
}

// Puts a finished conversion (in hw_buf) on screen.
static void hw_finish(int w, int h, u16* fb) {
    if (hw_mode == HW_T) {
        bool half = h == 480;
        int lw = half ? w / 2 : w, lh = half ? h / 2 : h;
        gx_untile(hw_buf, w, h, half, hw_lin, hw_flip);
        GSPGPU_InvalidateDataCache(hw_lin, (u32)lw * lh * 2);
        transpose_to_fb(hw_lin, lw, lh, fb, (400 - lw) / 2);
    } else {
        GSPGPU_InvalidateDataCache(hw_buf, (u32)w * h * 2);
        scaled_to_fb(hw_buf, w, h, fb);
    }
}

static void hw_preview_init(void) {
    snprintf(hw_label, sizeof(hw_label), "HW n/a: no Y2R");
    L_PVHW[0] = hw_label;
    if (R_FAILED(y2rInit())) return;
    bool ok = R_SUCCEEDED(Y2RU_SetTransferEndInterrupt(true)) && R_SUCCEEDED(Y2RU_GetTransferEndEvent(&y2r_end));
    hw_buf = linearAlloc(640 * 480 * 2);
    hw_lin = linearAlloc(400 * 240 * 2);
    u8* test = linearAlloc(640 * 480 * 2);
    u16* fbt = linearAlloc(400 * 240 * 2);              // test framebuffer
    u16* ref = malloc(400 * 240 * 2);
    int best = 1 << 30, successes = 0, failures = 0;
    (void)failures;
    static const int coefs[2] = { COEFFICIENT_ITU_R_BT_601, COEFFICIENT_ITU_R_BT_601_SCALING };
    if (!ok || !hw_buf || !hw_lin || !test || !fbt || !ref) { snprintf(hw_label, sizeof(hw_label), "HW n/a: no memory"); goto done; }

    // test card: brightness rises left->right, blue-difference rises top->bottom,
    // so any wrong rotation or mirror gives a large error
    for (int y = 0; y < 480; y++)
        for (int x = 0; x < 640; x += 2) {
            u8* p = test + (y * 640 + x) * 2;
            p[0] = (u8)(30 + x * 200 / 639); p[2] = (u8)(30 + (x + 1) * 200 / 639);
            p[1] = (u8)(60 + y * 136 / 479); p[3] = 128;
        }
    memset(ref, 0, 400 * 240 * 2);
    preview_image(test, 640, 480, ref);                  // software reference
    pv_sw = pv_sh = 0;
    GSPGPU_FlushDataCache(test, 640 * 480 * 2);

    #define SCORE(mode_, ci_, fl_) do { \
        int d_ = rgb565_diff(fbt + 40 * 240, ref + 40 * 240, 320); \
        if (d_ < best) { best = d_; hw_mode = mode_; hw_coef = coefs[ci_]; hw_flip = fl_; \
                         best_send = hw_send_l; best_recv = hw_recv_l; } } while (0)
    static const int sends[2] = { 1, 8 }, recvs[3] = { 8, 4, 2 };
    int best_send = 1, best_recv = 8, blank = 0;
    for (int si = 0; si < 2 && best > 20; si++)
        for (int ri = 0; ri < 3 && best > 20; ri++) {
            hw_send_l = sends[si]; hw_recv_l = recvs[ri];
            for (int ci = 0; ci < 2 && best > 20; ci++) {
                // L: plain lines + CPU scaled copy (also tells us whether Y2R writes anything)
                memset(hw_buf, 0, 640 * 480 * 2);
                GSPGPU_FlushDataCache(hw_buf, 640 * 480 * 2);
                if (!y2r_convert(test, 640, 480, ROTATION_NONE, coefs[ci], 0, 0, hw_buf)) { failures++; continue; }
                successes++;
                GSPGPU_InvalidateDataCache(hw_buf, 640 * 480 * 2);
                u32 nz = 0;
                for (int i = 0; i < 640 * 480; i += 97) nz += hw_buf[i] != 0;
                if (nz < 16) { blank++; continue; }
                memset(fbt, 0, 400 * 240 * 2);
                scaled_to_fb(hw_buf, 640, 480, fbt);
                pv_sw = pv_sh = 0;
                SCORE(HW_L, ci, 0);
                if (best > 20) continue;
                // T: 8x8 tiles, untiled + halved by GX, transposed by the CPU (cheaper than L)
                int keep_best = best, keep_mode = hw_mode;
                best = 1 << 30;
                if (y2r_convert(test, 640, 480, ROTATION_NONE, coefs[ci], 1, 0, hw_buf)) {
                    for (int fl = 0; fl < 2; fl++) {
                        gx_untile(hw_buf, 640, 480, true, hw_lin, fl);
                        GSPGPU_InvalidateDataCache(hw_lin, 320 * 240 * 2);
                        memset(fbt, 0, 400 * 240 * 2);
                        transpose_to_fb(hw_lin, 320, 240, fbt, 40);
                        SCORE(HW_T, ci, fl);
                    }
                }
                if (best > 20) { best = keep_best; hw_mode = keep_mode; hw_coef = coefs[ci]; hw_flip = 0;
                                 best_send = sends[si]; best_recv = recvs[ri]; }
            }
        }
    hw_send_l = best_send; hw_recv_l = best_recv;
    hw_cfg_key = -1;
    #undef SCORE
    if (best <= 20) {
        static const char mode_ch[4] = { '-', 'R', 'T', 'L' };
        snprintf(hw_label, sizeof(hw_label), "Hardware (%c%d/%d)", mode_ch[hw_mode], hw_send_l, hw_recv_l);
    } else {
        hw_mode = HW_NONE;
        if (!successes)                snprintf(hw_label, sizeof(hw_label), "HW n/a: timeout");
        else if (blank == successes)   snprintf(hw_label, sizeof(hw_label), "HW n/a: blank %d", successes);
        else                           snprintf(hw_label, sizeof(hw_label), "HW n/a: d%d b%d/%d", best, blank, successes);
    }
done:
    if (test) linearFree(test);
    if (fbt) linearFree(fbt);
    free(ref);
    if (hw_mode != HW_NONE) return;
    if (hw_buf) { linearFree(hw_buf); hw_buf = NULL; }
    if (hw_lin) { linearFree(hw_lin); hw_lin = NULL; }
    if (y2r_end) { svcCloseHandle(y2r_end); y2r_end = 0; }
    y2rExit();
}

static void hw_preview_exit(void) {
    if (hw_mode == HW_NONE && !hw_buf) return;
    if (hw_buf) { linearFree(hw_buf); hw_buf = NULL; }
    if (hw_lin) { linearFree(hw_lin); hw_lin = NULL; }
    if (y2r_end) { svcCloseHandle(y2r_end); y2r_end = 0; }
    y2rExit();
    hw_mode = HW_NONE;
}

static int pv_clear = 2;                // framebuffers still showing an old size's borders

static bool preview_frame(int slot) {
    const FrameSlot* fs = &slots[slot];
    if (!fs->w || !fs->h) return false;
    u16* fb = (u16*)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
    if (!fb) return false;
    if (fs->w != pv_sw || fs->h != pv_sh) pv_clear = 2;
    if (pv_clear > 0) {
        pv_clear--;
        memset(fb, 0, 400 * 240 * 2);
        if (gfxIs3D()) memset(gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL), 0, 400 * 240 * 2);
    }
    preview_image(fs->data, fs->w, fs->h, fb);
    // Both outer cameras: live 3D on the top screen (left/right eye framebuffers).
    if (fs->dual && fs->data2 && gfxIs3D()) {
        u16* fbr = (u16*)gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL);
        if (fbr) preview_image(fs->data2, fs->w, fs->h, fbr);
    }
    return true;
}

// ---- asynchronous hardware preview: the main loop starts a conversion and keeps running
// (menu, input, UI); the frame is drawn on a later pass once Y2R reports it's finished.
static struct { bool active; int slot, eye; u64 t0; } hw_job = { false, -1, 0, 0 };

static bool hw_usable(const FrameSlot* fs) {
    return hw_mode != HW_NONE && s_pv_hw == 0 && fs->w && fs->h && hw_pv_fits(fs->w, fs->h);
}

static void hw_job_fail(void) {
    if (++hw_fail >= 3) { hw_mode = HW_NONE; snprintf(hw_label, sizeof(hw_label), "HW failed"); }
}

static bool hw_job_start(int slot) {
    const FrameSlot* fs = &slots[slot];
    GSPGPU_FlushDataCache(fs->data, (u32)fs->w * fs->h * 2);   // LUTs may have written it via the CPU cache
    if (!y2r_start(fs->data, fs->w, fs->h, ROTATION_NONE, hw_coef, hw_mode == HW_T)) { hw_job_fail(); return false; }
    hw_job.active = true; hw_job.slot = slot; hw_job.eye = 0; hw_job.t0 = osGetTime();
    return true;
}

static void hw_job_cancel(void) {
    if (!hw_job.active) return;
    Y2RU_StopConversion();
    release_slot(hw_job.slot);
    hw_job.active = false;
}

// true = a complete preview frame is in the back buffer (swap it).
static bool hw_job_poll(void) {
    if (!hw_job.active) return false;
    if (!y2r_done()) {
        if (osGetTime() - hw_job.t0 > 250) { hw_job_cancel(); hw_job_fail(); }
        return false;
    }
    const FrameSlot* fs = &slots[hw_job.slot];
    if (hw_job.eye == 0) {
        u16* fb = (u16*)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
        if (fs->w != pv_sw || fs->h != pv_sh) { pv_clear = 2; preview_setup(fs->w, fs->h); }
        if (pv_clear > 0) {
            pv_clear--;
            memset(fb, 0, 400 * 240 * 2);
            if (gfxIs3D()) memset(gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL), 0, 400 * 240 * 2);
        }
        hw_finish(fs->w, fs->h, fb);
        // Both outer cameras: the right eye is a second conversion.
        if (fs->dual && fs->data2 && gfxIs3D()) {
            GSPGPU_FlushDataCache(fs->data2, (u32)fs->w * fs->h * 2);
            if (y2r_start(fs->data2, fs->w, fs->h, ROTATION_NONE, hw_coef, hw_mode == HW_T)) {
                hw_job.eye = 1; hw_job.t0 = osGetTime();
                return false;
            }
            preview_image(fs->data2, fs->w, fs->h, (u16*)gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL));
        }
    } else {
        hw_finish(fs->w, fs->h, (u16*)gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL));
    }
    release_slot(hw_job.slot);
    hw_job.active = false;
    hw_fail = 0;
    return true;
}

static void clear_top_buffers(void) {
    for (int i = 0; i < 2; i++) {
        u16* fb = (u16*)gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
        if (fb) memset(fb, 0, 400 * 240 * 2);
        if (gfxIs3D()) {
            u16* fbr = (u16*)gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL);
            if (fbr) memset(fbr, 0, 400 * 240 * 2);
        }
        gfxFlushBuffers();
        gfxSwapBuffers();
        gspWaitForVBlank();
    }
}

// Second image buffer per slot for the dual outer cameras (allocated on first use).
static bool alloc_dual_buffers(void) {
    if (dual_ok) return true;
    for (int i = 0; i < NUM_SLOTS; i++) {
        if (!slots[i].data2) slots[i].data2 = linearAlloc(SLOT_BYTES);
        if (!slots[i].data2) {
            for (int j = 0; j < NUM_SLOTS; j++) { if (slots[j].data2) linearFree(slots[j].data2); slots[j].data2 = NULL; }
            return false;
        }
    }
    dual_ok = true;
    return true;
}

// ---------------------------------------------------------------- recording
// Encoders finish out of order (several cores). Each recorded frame carries a ticket
// in capture order; encoders drop finished frames into the queue and never wait, and
// the SD writer pulls them back out in ticket order. Frames are malloc'd at their real
// size from a byte budget, so a few MB absorb seconds of SD-card latency spikes.
enum { RQ_FREE, RQ_FILLING, RQ_READY };
typedef struct { u8* data; u32 size; u64 tick; u32 gen, ticket; u8 state; } RecFrame;
static RecFrame rec_q[REC_QUEUE];
static u32 rq_bytes = 0;                       // bytes reserved by queued frames
static u32 skip_tab[TICKET_WINDOW];            // ticket+1 when that ticket was dropped
static u32 skip_gen[TICKET_WINDOW];
static LightLock rq_lock;
static LightEvent rec_event;

static volatile bool rec_active = false;      // encoder should hand frames to the writer
static volatile bool rec_stop_req = false;
// Pause: a 4-state handshake between the main thread (asks) and the SD writer (confirms).
// Only one change is in flight at a time, so the threads can never disagree.
enum { RS_RUN = 0, RS_PAUSE_REQ, RS_PAUSED, RS_RESUME_REQ };
static volatile int rec_pstate = RS_RUN;
static volatile u64 rec_pause_from = 0;       // tick of the pause press (end of what's kept)
static volatile u64 rec_resume_at = 0;        // tick of the resume press
static bool rec_running(void) { return rec_active && rec_pstate == RS_RUN; }
static volatile bool rec_saving = false;      // writer thread still running
static volatile bool rec_error = false;
static volatile u32 stat_rec_drop = 0;
static volatile u32 stat_rec_frames = 0;
static volatile u64 rec_start_tick = 0;
static u32 rec_gen = 0, rec_ticket_next = 0;   // guarded by slot_lock
static Thread writer_th = NULL;
static char rec_base[128];
static volatile u16 rec_w = 0, rec_h = 0;     // size locked for the whole recording
static u32 rec_fps = 30;                      // locked at start
static u32 rec_audio_hz = 0;                  // locked at start, 0 = no audio track
static volatile u64 rec_bytes = 0;            // payload written, for the MB/min readout
static volatile int rec_part = 0;             // current file number (files split at ~1.9 GB)
static volatile u32 rec_part_fill = 0;        // current file: 0..1000 of the way to the split
static volatile u32 rec_part_mb = 0;          // current file size in MB
static volatile u32 rec_sd_kbs = 0;           // SD write speed of the current file (KB/s), 0 = unknown
static u64 rec_sd_free0 = 0;                  // SD free bytes when recording started (0 = unknown)

static u8* mic_buf = NULL;
static u32 mic_size = 0;
static bool mic_ok = false;                   // mic service initialised
static bool mic_sampling = false;

// (Re)start sampling at the rate chosen in the menu. Only called when not recording.
static void mic_apply_rate(void) {
    if (!mic_ok) return;
    if (mic_sampling) { MICU_StopSampling(); mic_sampling = false; }
    if (s_audio > 0)
        mic_sampling = R_SUCCEEDED(MICU_StartSampling(MICU_ENCODING_PCM16_SIGNED,
                                   val_audio_rate[s_audio], 0, mic_size, true));
}

static void rq_mark_skipped(u32 gen, u32 ticket) {   // rq_lock held
    skip_tab[ticket % TICKET_WINDOW] = ticket + 1;
    skip_gen[ticket % TICKET_WINDOW] = gen;
}

static void rq_free_entry(RecFrame* f) {             // rq_lock held
    free(f->data);
    f->data = NULL;
    rq_bytes -= f->size;
    f->state = RQ_FREE;
}

// Encoder side: never blocks.
static void rec_submit(u32 gen, u32 ticket, const u8* jpg, u32 size, u64 tick, bool ok) {
    LightLock_Lock(&rq_lock);
    int idx = -1;
    if (ok && rq_bytes + size <= REC_POOL_BYTES)
        for (int i = 0; i < REC_QUEUE; i++)
            if (rec_q[i].state == RQ_FREE) {
                idx = i;
                rec_q[i].state = RQ_FILLING;
                rec_q[i].size = size;
                rq_bytes += size;
                break;
            }
    if (idx < 0) {
        if (ok) stat_rec_drop++;                  // SD writer is behind: frame becomes a repeat
        rq_mark_skipped(gen, ticket);
        LightLock_Unlock(&rq_lock);
        LightEvent_Signal(&rec_event);
        return;
    }
    LightLock_Unlock(&rq_lock);

    u8* buf = malloc(size);                       // entry is ours while FILLING
    if (buf) memcpy(buf, jpg, size);

    LightLock_Lock(&rq_lock);
    RecFrame* f = &rec_q[idx];
    if (!buf) {
        rq_bytes -= size;
        f->state = RQ_FREE;
        stat_rec_drop++;
        rq_mark_skipped(gen, ticket);
    } else {
        f->data = buf;
        f->tick = tick;
        f->gen = gen;
        f->ticket = ticket;
        f->state = RQ_READY;
    }
    LightLock_Unlock(&rq_lock);
    LightEvent_Signal(&rec_event);
}

static u32 rec_tickets_issued(void) {
    LightLock_Lock(&slot_lock);
    u32 n = rec_ticket_next;
    LightLock_Unlock(&slot_lock);
    return n;
}

// Writer side: look for ticket `next`.
enum { NEXT_FRAME, NEXT_SKIPPED, NEXT_MISSING };
static int rec_take(u32 gen, u32 next, int* out_idx, int* ready_later, bool* pressure) {
    int result = NEXT_MISSING;
    *out_idx = -1;
    *ready_later = 0;
    LightLock_Lock(&rq_lock);
    for (int i = 0; i < REC_QUEUE; i++) {
        RecFrame* f = &rec_q[i];
        if (f->state != RQ_READY) continue;
        if (f->gen != gen || f->ticket < next) { rq_free_entry(f); continue; }   // stale / given up on
        if (f->ticket == next) { *out_idx = i; result = NEXT_FRAME; }
        else (*ready_later)++;
    }
    u32 slot = next % TICKET_WINDOW;
    if (result == NEXT_MISSING && skip_tab[slot] == next + 1 && skip_gen[slot] == gen) {
        skip_tab[slot] = 0;
        result = NEXT_SKIPPED;
    }
    *pressure = rq_bytes > REC_POOL_BYTES / 2 || *ready_later >= REC_QUEUE / 2;
    LightLock_Unlock(&rq_lock);
    return result;
}

static void rec_release(int idx) {
    LightLock_Lock(&rq_lock);
    rq_free_entry(&rec_q[idx]);
    LightLock_Unlock(&rq_lock);
}

// ---- mic EQ: the main thread bumps the version on any change; the writer and the
// level meter each keep their own filter state and redesign when it changes.
static volatile u32 miceq_version = 1;

static void miceq_params(MicEqParams* p) {
    p->enabled = s_eq_on != 0;
    for (int i = 0; i < MICEQ_BANDS; i++) p->gain_db[i] = s_eq[i];
    p->lowcut = s_eq_lowcut;
    p->boost_db = s_eq_boost;
    p->limiter = s_eq_limit != 0;
}

static MicEq writer_eq;
static u32 writer_eq_ver = 0;
static s16 eq_buf[2048];

static u32 mic_hw_pos(void) {
    u32 p = micGetLastSampleOffset() & ~1u;
    return p >= mic_size ? 0 : p;
}

static u32 mic_available(u32 rp) {
    u32 hw = mic_hw_pos();
    return hw >= rp ? hw - rp : mic_size - rp + hw;
}

static volatile u32 stat_audio_fix = 0;       // dropout/overrun corrections this recording

static void write_silence(AviWriter* avi, u64 samples) {
    memset(eq_buf, 0, sizeof(eq_buf));
    while (samples > 0) {
        u32 n = samples > sizeof(eq_buf) / 2 ? sizeof(eq_buf) / 2 : (u32)samples;
        avi_write_audio(avi, eq_buf, n * 2);
        samples -= n;
    }
}

// Audio is kept on the same clock as the video timeline (capture timestamps from
// part_start). The newest sample in the mic buffer was recorded "now", so the audio
// written so far plus what's waiting should equal (now - part_start) * rate. If the mic
// service paused (samples never arrived), the gap is filled with silence; if we somehow
// have too much, the oldest excess is skipped. Without this, a 0.2 s mic hiccup shifted
// all later audio 0.2 s early and the audio track ended before the video.
// Copy mic samples [*rp, hw) into the file (through the EQ), advancing *rp.
static void mic_copy(AviWriter* avi, u32* rp, u32 hw) {
    if (writer_eq_ver != miceq_version) {             // settings changed (also works mid-recording)
        MicEqParams p;
        writer_eq_ver = miceq_version;
        miceq_params(&p);
        miceq_design(&writer_eq, (float)rec_audio_hz, &p);
    }
    while (*rp != hw) {
        u32 end = hw > *rp ? hw : mic_size;
        u32 len = end - *rp;
        if (writer_eq.active) {
            // EQ into a scratch buffer (the mic buffer belongs to the mic service)
            for (u32 off = 0; off < len; ) {
                u32 n = len - off > sizeof(eq_buf) ? sizeof(eq_buf) : len - off;
                memcpy(eq_buf, mic_buf + *rp + off, n);
                miceq_process(&writer_eq, eq_buf, n / 2);
                avi_write_audio(avi, eq_buf, n);
                off += n;
            }
        } else {
            avi_write_audio(avi, mic_buf + *rp, len);
        }
        rec_bytes += len;
        *rp = end >= mic_size ? 0 : end;
    }
}

static void drain_audio(AviWriter* avi, u32* rp, u64 part_start) {
    if (!rec_audio_hz) return;
    u32 hw = mic_hw_pos();
    u64 now = svcGetSystemTick();
    {
        u64 have = avi_audio_samples(avi) + mic_available(*rp) / 2;
        u64 want = now > part_start ? (now - part_start) * rec_audio_hz / SYSCLOCK_ARM11 : 0;
        s64 diff = (s64)want - (s64)have;
        s64 tol = rec_audio_hz / 25;                 // 40 ms: ignore normal jitter
        if (diff > tol) {                            // mic dropped out: fill the hole
            write_silence(avi, (u64)diff);
            stat_audio_fix++;
        } else if (diff < -tol) {                    // more audio than time passed: skip the excess
            u32 skip = (u32)(-diff) * 2;
            u32 avail = mic_available(*rp);
            if (skip > avail) skip = avail;
            *rp = (*rp + skip) % mic_size;
            stat_audio_fix++;
        }
    }
    mic_copy(avi, rp, hw);
}

// Pause: keep exactly the audio recorded up to `until`, so the file's audio ends at the
// pause moment like its video; everything recorded after it is thrown away.
static void drain_audio_to(AviWriter* avi, u32* rp, u64 part_start, u64 until) {
    if (!rec_audio_hz) return;
    u64 now = svcGetSystemTick();
    u32 hw = mic_hw_pos();
    u32 avail = mic_available(*rp) / 2;
    u64 after = now > until ? (now - until) * rec_audio_hz / SYSCLOCK_ARM11 : 0;   // recorded after the press
    u32 keep = avail > after ? (u32)(avail - after) : 0;
    u64 want = until > part_start ? (until - part_start) * rec_audio_hz / SYSCLOCK_ARM11 : 0;
    u64 have = avi_audio_samples(avi);
    if (have + keep > want) keep = want > have ? (u32)(want - have) : 0;           // never past the pause
    mic_copy(avi, rp, (*rp + keep * 2) % mic_size);
    have = avi_audio_samples(avi);
    if (want > have) write_silence(avi, want - have);                             // mic gap before the pause
    *rp = hw;                                                                      // discard the rest
}

// Cut the recording for a pause: video and audio end at the same video-frame boundary
// (the first one at/after the press, or after the last frame already written), so they're
// exactly equal at every pause and nothing can build up over many pauses. Returns the
// tick the file's timeline stopped at.
static void pad_video_to(AviWriter* avi, u64 part_start, u64 until, u32 fps);
static u64 cut_for_pause(AviWriter* avi, u32* rp, u64 part_start, u64 pressed, u32 fps) {
    pad_video_to(avi, part_start, pressed, fps);
    u64 nv = avi_video_frames(avi);
    u64 cut = part_start + (nv * SYSCLOCK_ARM11 + fps / 2) / fps;
    drain_audio_to(avi, rp, part_start, cut);
    return cut;
}

// Repeat-frame the video up to the timeline position of `until`.
static void pad_video_to(AviWriter* avi, u64 part_start, u64 until, u32 fps) {
    if (until <= part_start) return;
    u64 n = ((until - part_start) * fps * 2 + SYSCLOCK_ARM11) / (2 * (u64)SYSCLOCK_ARM11);
    for (u32 w = avi_video_frames(avi); w < n; w++) avi_write_video(avi, NULL, 0);
}

// Repeat the last frame until the video is as long as the audio (A/V end together).
static void pad_video_to_audio(AviWriter* avi, u32 fps) {
    if (!rec_audio_hz || !avi_video_frames(avi)) return;
    u64 samples = avi_audio_samples(avi);
    u32 target = (u32)((samples * fps + rec_audio_hz / 2) / rec_audio_hz);
    for (u32 n = avi_video_frames(avi); n < target && !avi_failed(avi); n++) avi_write_video(avi, NULL, 0);
}

static AviWriter* open_part(int part, u16 w, u16 h) {
    char path[160];
    if (part == 0) snprintf(path, sizeof(path), "%s.avi", rec_base);
    else           snprintf(path, sizeof(path), "%s_part%d.avi", rec_base, part + 1);
    return avi_open(path, w, h, rec_fps, rec_audio_hz);
}

static void writer_thread(void* arg) {
    (void)arg;
    u16 w = rec_w, h = rec_h;
    int part = 0;
    u32 mic_rp = rec_audio_hz ? mic_hw_pos() : 0;
    u64 part_start = rec_start_tick;
    const u32 fps = rec_fps;
    const u32 audio_batch = rec_audio_hz * 2 / 10;  // write audio in ~100 ms chunks
    writer_eq_ver = 0;                              // (re)design the EQ for this recording's rate
    LightLock_Lock(&slot_lock);
    const u32 my_gen = rec_gen;
    LightLock_Unlock(&slot_lock);
    u32 next_ticket = 0;
    u64 wait_since = 0, stop_since = 0;
    bool pausing = false, paused = false;          // writer's own view of the pause handshake
    u64 pause_since = 0, pause_from = 0, pause_cut = 0;
    u64 drop_before = 0;                           // after a resume: frames captured before it

    AviWriter* avi = open_part(part, w, h);
    if (!avi) { rec_error = true; rec_active = false; rec_saving = false; return; }

    for (;;) {
        LightEvent_WaitTimeout(&rec_event, 20000000LL);
        bool stopping = rec_stop_req;

        // ---- pause / resume handshake
        if (!pausing && !paused && rec_pstate == RS_PAUSE_REQ) {
            pausing = true; pause_since = osGetTime(); pause_from = rec_pause_from;
        }
        if (paused && rec_pstate == RS_RESUME_REQ) {
            // Continue the timeline where it stopped: shift the clock by the paused time and
            // read the mic from the resume moment on (nothing in between gets into the file).
            u64 now = svcGetSystemTick(), resume = rec_resume_at;
            part_start += resume - pause_cut;       // timeline continues exactly at the cut
            drop_before = resume;
            if (rec_audio_hz) {
                u32 hw = mic_hw_pos();
                u64 back = now > resume ? (now - resume) * rec_audio_hz / SYSCLOCK_ARM11 * 2 : 0;
                if (back >= mic_size) back = 0;
                mic_rp = (u32)((hw + mic_size - (u32)back) % mic_size) & ~1u;
            }
            paused = false;
            __dmb();
            rec_pstate = RS_RUN;                    // encoders start claiming frames again
        }
        if (paused && rec_audio_hz) mic_rp = mic_hw_pos();   // keep the mic buffer from ever overflowing

        if (rec_audio_hz && !paused && !pausing && (stopping || mic_available(mic_rp) >= audio_batch))
            drain_audio(avi, &mic_rp, part_start);

        for (;;) {
            int idx, later; bool pressure;
            int r = rec_take(my_gen, next_ticket, &idx, &later, &pressure);
            if (r == NEXT_SKIPPED) { next_ticket++; wait_since = 0; continue; }
            if (r == NEXT_MISSING) {
                // An encoder is still working on it (e.g. the low-priority core-0 one).
                // Wait for it only while that's harmless: give up right away if the queue
                // is filling up, or after 400 ms if newer frames are waiting. The late
                // frame just becomes a repeat of the previous one.
                u64 now = osGetTime();
                if (next_ticket < rec_tickets_issued() && later) {
                    if (pressure) { next_ticket++; wait_since = 0; continue; }
                    if (!wait_since) wait_since = now;
                    else if (now - wait_since > 400) { next_ticket++; wait_since = 0; continue; }
                }
                break;
            }
            next_ticket++;
            wait_since = 0;
            RecFrame* f = &rec_q[idx];
            // paused: nothing goes in; pausing: only frames captured before the press
            // after a resume: never a frame captured during the pause (the newest camera frame
            // at the moment of resuming is usually one of those)
            if (paused || (pausing && f->tick >= pause_from) || f->tick < drop_before) { rec_release(idx); continue; }
            if (f->tick >= part_start) {
                // Constant-frame-rate timeline from capture timestamps: if the encoder
                // skipped frames, insert empty "repeat" chunks so video stays in sync
                // with the audio; if this slot is already filled, drop the frame.
                u64 n = ((f->tick - part_start) * fps * 2 + SYSCLOCK_ARM11) / (2 * (u64)SYSCLOCK_ARM11);
                u32 written = avi_video_frames(avi);
                if (n >= written) {
                    while (written < n) { avi_write_video(avi, NULL, 0); written++; }
                    avi_write_video(avi, f->data, f->size);
                    rec_bytes += f->size;
                    stat_rec_frames++;
                }
            }
            rec_release(idx);
            // Interleave audio between video frames. When the SD card is the bottleneck
            // (VGA + high quality) there is always another frame waiting, so without this
            // the loop never got back to the audio and the mic buffer was overwritten.
            if (rec_audio_hz && !pausing && !paused && rec_pstate == RS_RUN && mic_available(mic_rp) >= audio_batch)
                drain_audio(avi, &mic_rp, part_start);
        }

        // Pause takes effect once every frame captured before the press is in (or 2 s passed):
        // audio and video are both cut exactly at the press, then the writer confirms.
        if (pausing && (next_ticket >= rec_tickets_issued() || osGetTime() - pause_since > 2000)) {
            pause_cut = cut_for_pause(avi, &mic_rp, part_start, pause_from, fps);
            pausing = false;
            paused = true;
            __dmb();
            rec_pstate = RS_PAUSED;
        }

        rec_part = part;
        rec_part_fill = avi_fill_permille(avi);
        rec_part_mb = (u32)(avi_file_bytes(avi) >> 20);
        { u32 k = avi_write_kbs(avi); if (k) rec_sd_kbs = k; }
        if (avi_failed(avi)) { rec_error = true; break; }   // SD full / removed

        if (!paused && !pausing && avi_should_split(avi)) {
            drain_audio(avi, &mic_rp, part_start);
            pad_video_to_audio(avi, fps);
            part_start = svcGetSystemTick();
            avi_close(avi);
            avi = open_part(++part, w, h);
            if (!avi) { rec_error = true; break; }
        }

        if (stopping) {
            // Done once every frame handed out to an encoder has been written or skipped.
            if (!stop_since) stop_since = osGetTime();
            if (next_ticket >= rec_tickets_issued() || osGetTime() - stop_since > 3000) break;
        }
    }

    if (avi) {
        if (pausing) {                      // stopped while a pause was being applied
            cut_for_pause(avi, &mic_rp, part_start, pause_from, fps);
        } else if (!paused) {
            drain_audio(avi, &mic_rp, part_start);
        }                                   // (stopped while paused: the file already ends at the pause)
        pad_video_to_audio(avi, fps);       // video and audio always end together
        avi_close(avi);
    }
    rec_active = false;
    __dmb();
    rec_saving = false;
}

static void start_recording(void) {
    if (rec_active || rec_saving || writer_th) return;

    time_t t = time(NULL);
    struct tm* tm = localtime(&t);
    char stamp[64];
    strftime(stamp, sizeof(stamp), "VID_%Y%m%d_%H%M%S", tm);
    snprintf(rec_base, sizeof(rec_base), "%s/%s", SAVE_FOLDER, stamp);

    LightLock_Lock(&rq_lock);
    for (int i = 0; i < REC_QUEUE; i++)
        if (rec_q[i].state == RQ_READY) rq_free_entry(&rec_q[i]);  // FILLING ones get purged later
    memset(skip_tab, 0, sizeof(skip_tab));
    LightLock_Unlock(&rq_lock);
    LightEvent_Clear(&rec_event);
    stat_rec_drop = 0;
    stat_rec_frames = 0;
    stat_audio_fix = 0;
    rec_error = false;
    rec_stop_req = false;
    rec_pstate = RS_RUN;
    rec_resume_at = 0;
    LightLock_Lock(&slot_lock);
    rec_gen++;
    rec_ticket_next = 0;
    rec_start_tick = svcGetSystemTick();
    LightLock_Unlock(&slot_lock);
    rec_w = val_res_w[s_res];
    rec_h = val_res_h[s_res];
    rec_fps = val_recfps[s_rec_fps];
    rec_audio_hz = (mic_sampling && s_audio > 0) ? val_audio_hz[s_audio] : 0;
    rec_bytes = 0;
    rec_part = 0; rec_part_fill = 0; rec_part_mb = 0; rec_sd_kbs = 0;
    FS_ArchiveResource res;
    rec_sd_free0 = R_SUCCEEDED(FSUSER_GetArchiveResource(&res, SYSTEM_MEDIATYPE_SD))
                   ? (u64)res.freeClusters * res.clusterSize : 0;

    rec_saving = true;
    rec_active = true;
    writer_th = threadCreate(writer_thread, NULL, 0x8000, 0x31, -2, false);
    if (!writer_th) { rec_active = false; rec_saving = false; rec_error = true; }
}

// Camera being (re)started: settings changed and no frame from the new setup yet.
static bool cam_starting(void) { return cam_reconfig || slot_latest < 0; }

// X while recording. Ignored while the writer is still confirming the previous change.
static void toggle_pause(void) {
    if (!rec_active || rec_stop_req) return;
    if (rec_pstate == RS_RUN) {
        rec_pause_from = svcGetSystemTick();
        __dmb();
        rec_pstate = RS_PAUSE_REQ;                 // encoders stop claiming frames now
        LightEvent_Signal(&rec_event);
    } else if (rec_pstate == RS_PAUSED) {
        // After a camera switch, wait until the new camera delivers: resuming earlier would
        // freeze the picture for the ~0.5 s the camera needs to start.
        if (cam_starting()) return;
        u64 now = svcGetSystemTick();
        LightLock_Lock(&slot_lock);                // (64-bit values: written under the lock
        rec_start_tick += now - rec_pause_from;    //  the encoders read them under)
        rec_resume_at = now;                       // elapsed time / frame timeline skip the pause
        LightLock_Unlock(&slot_lock);
        __dmb();
        rec_pstate = RS_RESUME_REQ;
        LightEvent_Signal(&rec_event);
    }
}

static void stop_recording(void) {
    if (!rec_active) return;
    rec_active = false;           // encoder stops queueing
    rec_stop_req = true;          // writer flushes queue + audio, closes file
    LightEvent_Signal(&rec_event);
}

static void poll_recording(void) {
    if (writer_th && !rec_saving) {
        threadJoin(writer_th, U64_MAX);
        threadFree(writer_th);
        writer_th = NULL;
    }
}

// ---------------------------------------------------------------- encoder
// One encoder thread per usable CPU core, each with its own buffers. They take turns
// grabbing the newest frame, so N cores ~= N x the FPS. The default encoder is
// fastjpeg (ARM11 SIMD, reads the camera's YUYV directly); libjpeg-turbo remains as
// a selectable fallback.
typedef struct {
    tjhandle tj;
    u8 *py, *pu, *pv;          // planar scratch (libjpeg-turbo path)
    u8* jpeg;
    u32 jpeg_cap;
    FJQuant q_rec;             // fast encoder tables, rebuilt when the quality changes
    FJCache* c_rec;            // static-skip memory (pixels + coded bits of the last frame)
} EncCtx;

static EncCtx enc_ctx[MAX_ENCODERS];
static int num_encoders = 0;
static u32 enc_claimed_seq = 0;                  // guarded by slot_lock
static u64 claim_rec_start = 0, claim_last_n = ~0ull;

typedef struct { int slot; bool rec; u32 gen, ticket; } Claim;

// Atomically take the newest frame no other encoder has taken yet, and decide
// whether it's needed for the recording timeline (and with which ticket).
static bool claim_frame(Claim* c, bool do_rec) {
    LightLock_Lock(&slot_lock);
    int idx = slot_latest;
    if (idx < 0 || slots[idx].seq == enc_claimed_seq) { LightLock_Unlock(&slot_lock); return false; }
    enc_claimed_seq = slots[idx].seq;

    c->rec = false;
    if (do_rec) {
        u64 start = rec_start_tick, tick = slots[idx].tick;
        if (tick < rec_resume_at) { LightLock_Unlock(&slot_lock); return false; }   // captured during a pause
        if (start != claim_rec_start) { claim_rec_start = start; claim_last_n = ~0ull; }
        u64 n = tick > start ? ((tick - start) * rec_fps * 2 + SYSCLOCK_ARM11) / (2 * (u64)SYSCLOCK_ARM11) : 0;
        if (n != claim_last_n) {           // skip frames that would land in a filled timeline slot
            claim_last_n = n;
            c->rec = true;
            c->gen = rec_gen;
            c->ticket = rec_ticket_next++;
        }
    }
    if (!c->rec) { LightLock_Unlock(&slot_lock); return false; }
    slot_users[idx]++;
    c->slot = idx;
    LightLock_Unlock(&slot_lock);
    return true;
}

// Packed YUYV 4:2:2 -> planar 4:2:0 (chroma from even rows).
static void yuyv_to_420(EncCtx* e, const u8* src, int w, int h) {
    const u32* s = (const u32*)src;
    u8 *py = e->py, *pu = e->pu, *pv = e->pv;
    for (int y = 0; y < h; y++) {
        if (!(y & 1)) {
            for (int x = 0; x < w; x += 2) {
                u32 p = *s++;
                *py++ = p; *py++ = p >> 16;
                *pu++ = p >> 8; *pv++ = p >> 24;
            }
        } else {
            for (int x = 0; x < w; x += 2) {
                u32 p = *s++;
                *py++ = p; *py++ = p >> 16;
            }
        }
    }
}

// Packed YUYV 4:2:2 -> planar 4:2:2 (keeps every chroma row).
static void yuyv_to_422(EncCtx* e, const u8* src, int w, int h) {
    const u32* s = (const u32*)src;
    u8 *py = e->py, *pu = e->pu, *pv = e->pv;
    for (int i = 0, n = w * h / 2; i < n; i++) {
        u32 p = *s++;
        *py++ = p; *py++ = p >> 16;
        *pu++ = p >> 8; *pv++ = p >> 24;
    }
}

// Packed YUYV w x h -> planar 4:2:0 at (w/2) x (h/2), Y averaged over 2x2 (no aliasing).
static void yuyv_half_to_420(EncCtx* e, const u8* src, int w, int h) {
    const u32* s = (const u32*)src;
    const int pw = w / 2, ow = w / 2, oh = h / 2;
    for (int oy = 0; oy < oh; oy++) {
        const u32* r0 = s + (2 * oy) * pw;
        const u32* r1 = r0 + pw;
        u8* py = e->py + oy * ow;
        for (int ox = 0; ox < ow; ox++) {
            u32 a = r0[ox], b = r1[ox];
            py[ox] = ((a & 0xFF) + ((a >> 16) & 0xFF) + (b & 0xFF) + ((b >> 16) & 0xFF) + 2) >> 2;
        }
        if (!(oy & 1)) {
            u8* pu = e->pu + (oy / 2) * (ow / 2);
            u8* pv = e->pv + (oy / 2) * (ow / 2);
            for (int cx = 0; cx < ow / 2; cx++) {
                u32 a = r0[cx * 2];
                pu[cx] = (a >> 8) & 0xFF;
                pv[cx] = a >> 24;
            }
        }
    }
}

static bool encode_planes(EncCtx* e, int w, int h, int samp, int quality, int flags, unsigned long* jsize) {
    const unsigned char* planes[3] = { e->py, e->pu, e->pv };
    int strides[3] = { w, w / 2, w / 2 };
    *jsize = e->jpeg_cap;
    return tjCompressFromYUVPlanes(e->tj, planes, w, strides, h, samp, &e->jpeg, jsize, quality, flags) == 0;
}

static void encoder_thread(void* arg) {
    EncCtx* e = (EncCtx*)arg;
    while (running) {
        LightEvent_WaitTimeout(&enc_event, 100000000LL);   // one wake-up per new camera frame
        if (!running) break;

        if (!rec_running()) continue;                      // not recording / paused: stay idle

        Claim c;
        if (!claim_frame(&c, true)) continue;
        int s = c.slot;
        u16 w = slots[s].w, h = slots[s].h;
        u64 tick = slots[s].tick;
        unsigned long jsize;

        const bool fast = s_encoder == 0;
        const u8* src = slots[s].data;

        if (c.rec) {
            bool ok;
            if (fast) {
                // Encodes straight from the camera slot (held until done).
                if (e->q_rec.quality != s_rec_q) fj_set_quality(&e->q_rec, s_rec_q);
                int th = SKIP_THRESH[s_skip & 3];
                jsize = fj_encode_yuyv_cached(&e->q_rec, src, w, h, w * 2, s_rec_chroma, e->jpeg, e->jpeg_cap,
                                              th ? e->c_rec : NULL, th);
                ok = jsize > 0;
                if (th) { int sk, tot; fj_cache_stats(e->c_rec, &sk, &tot); stat_skip_pct = tot ? sk * 100 / tot : 0; }
                else stat_skip_pct = -1;
                release_slot(s); s = -1;
            } else if (!e->py && !(e->py = malloc(640 * 480), e->pu = malloc(640 * 480 / 2),
                                    e->pv = malloc(640 * 480 / 2), e->py && e->pu && e->pv)) {
                // planar scratch is only allocated if libjpeg-turbo is actually used
                free(e->py); free(e->pu); free(e->pv); e->py = e->pu = e->pv = NULL;
                release_slot(s); s = -1;
                ok = false; jsize = 0;
            } else {
                int samp  = s_rec_chroma ? TJSAMP_422 : TJSAMP_420;
                int flags = TJFLAG_NOREALLOC | (s_rec_dct ? TJFLAG_ACCURATEDCT : TJFLAG_FASTDCT);
                if (samp == TJSAMP_422) yuyv_to_422(e, src, w, h);
                else                    yuyv_to_420(e, src, w, h);
                release_slot(s); s = -1;
                ok = encode_planes(e, w, h, samp, s_rec_q, flags, &jsize);
            }
            ok = ok && w == rec_w && h == rec_h;
            if (ok) stat_enc++;
            rec_submit(c.gen, c.ticket, e->jpeg, (u32)jsize, tick, ok);
        }
        if (s >= 0) release_slot(s);
    }
}

static bool enc_ctx_init(EncCtx* e) {
    e->tj = tjInitCompress();
    // (py/pu/pv: libjpeg-turbo planar scratch, allocated on first use - saves 1.2 MB per core)
    e->c_rec = fj_cache_create();          // tiny; the pixel/bit memory is allocated on first use
    u32 cap = (u32)tjBufSize(640, 480, TJSAMP_422);
    if (cap < 1024 * 1024) cap = 1024 * 1024;   // fast encoder gives up (frame dropped) beyond this
    e->jpeg_cap = cap;
    e->jpeg = tjAlloc(cap);
    // Called from the main thread before encoders start: also builds fastjpeg's shared tables.
    fj_set_quality(&e->q_rec, s_rec_q);
    return e->tj && e->jpeg && e->c_rec;
}

static void enc_ctx_free(EncCtx* e) {
    if (e->tj) tjDestroy(e->tj);
    if (e->jpeg) tjFree(e->jpeg);
    free(e->py); free(e->pu); free(e->pv);
    fj_cache_destroy(e->c_rec);
    memset(e, 0, sizeof(*e));
}

// ---------------------------------------------------------------- mic level meter
// Runs the newest ~60 ms of mic audio through its own copy of the EQ so you can see
// the effect of every change (and whether boost clips) without recording.
static MicEq meter_eq;
static u32 meter_eq_ver = 0;
static s16 meter_buf[2048];
static int meter_in_db = -99, meter_out_db = -99;
static bool meter_clip = false;

static int peak_db(const s16* b, int n) {
    int pk = 1;
    for (int i = 0; i < n; i++) { int a = b[i] < 0 ? -b[i] : b[i]; if (a > pk) pk = a; }
    return (int)floorf(20.0f * log10f(pk / 32768.0f) + 0.5f);
}

static void meter_update(void) {
    if (!mic_sampling || s_audio == 0) { meter_in_db = meter_out_db = -99; return; }
    u32 hz = val_audio_hz[s_audio];
    if (meter_eq_ver != miceq_version) {
        MicEqParams p;
        meter_eq_ver = miceq_version;
        miceq_params(&p);
        miceq_design(&meter_eq, (float)hz, &p);
    }
    // copy the newest 2048 samples out of the ring buffer
    u32 bytes = sizeof(meter_buf), hw = mic_hw_pos();
    u32 start = hw >= bytes ? hw - bytes : mic_size - (bytes - hw);
    for (u32 i = 0; i < bytes; ) {
        u32 pos = (start + i) % mic_size;
        u32 n = mic_size - pos < bytes - i ? mic_size - pos : bytes - i;
        memcpy((u8*)meter_buf + i, mic_buf + pos, n);
        i += n;
    }
    const int n = 2048, tail = 1024;                 // measure the 2nd half (filters settled)
    meter_in_db = peak_db(meter_buf + n - tail, tail);
    miceq_reset(&meter_eq);
    miceq_process(&meter_eq, meter_buf, n);
    meter_out_db = peak_db(meter_buf + n - tail, tail);
    meter_clip = meter_out_db >= 0;
}

static void draw_meter(void) {
    // "in -18 out  -6 dB [##########------]" (fits the 40-column console)
    int db = meter_out_db < -60 ? -60 : meter_out_db;
    int fill = (db + 60) * 16 / 60;
    char bar[17];
    for (int i = 0; i < 16; i++) bar[i] = i < fill ? '#' : '-';
    bar[16] = 0;
    printf("\x1b[36min %3d out %3d\x1b[0m %s[%s]\x1b[0m%s\x1b[K\n", meter_in_db, meter_out_db,
           meter_clip ? "\x1b[31m" : (db > -6 ? "\x1b[33m" : "\x1b[32m"), bar, meter_clip ? "CLIP" : "");
}

// ---------------------------------------------------------------- photos
static FJQuant photo_q;
static u8* photo_jpeg = NULL;
static const u32 photo_jpeg_cap = 2 * 1024 * 1024;
static u8* photo_wide = NULL;                  // stitched image (up to 800x480 YUYV)
static char photo_msg[40] = "";
static u64 photo_due_ms = 0;                   // self-timer: when to shoot (0 = idle)

static bool save_jpeg(const char* path, const u8* yuyv, int w, int h, int stride) {
    if (photo_q.quality != s_photo_q) fj_set_quality(&photo_q, s_photo_q);
    u32 n = fj_encode_yuyv(&photo_q, yuyv, w, h, stride, s_photo_chroma, photo_jpeg, photo_jpeg_cap);
    if (!n) return false;
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    bool ok = fwrite(photo_jpeg, 1, n, f) == n;
    ok &= fclose(f) == 0;
    return ok;
}

// SAVE_FOLDER/IMG_<date>_<time>[_N] without extension, not clashing with an existing file
static void photo_base(char* out, int n) {
    time_t t = time(NULL);
    struct tm* tm = localtime(&t);
    char stamp[32];
    strftime(stamp, sizeof(stamp), "IMG_%Y%m%d_%H%M%S", tm);
    for (int i = 1; i < 100; i++) {
        char test[176];
        if (i == 1) snprintf(out, n, "%s/%s", SAVE_FOLDER, stamp);
        else        snprintf(out, n, "%s/%s_%d", SAVE_FOLDER, stamp, i);
        snprintf(test, sizeof(test), "%s.jpg", out);
        struct stat st;
        if (stat(test, &st) != 0) return;
    }
}

static void take_photo(void) {
    if (!photo_jpeg) photo_jpeg = malloc(photo_jpeg_cap);
    u32 any = 0;                               // seq 0 is never used -> always the newest frame
    int s = acquire_latest(&any);
    if (s < 0 || !photo_jpeg) { snprintf(photo_msg, sizeof(photo_msg), "%s", tr("No camera frame yet")); return; }
    if (s_shutter) CAMU_PlayShutterSound(SHUTTER_SOUND_TYPE_NORMAL);

    printf("%s", tr("\x1b[1;0H\x1b[33m[PHOTO] saving...\x1b[0m\x1b[K"));
    gfxFlushBuffers();

    const FrameSlot* fs = &slots[s];
    int w = fs->w, h = fs->h;
    char base[160], path[176];
    photo_base(base, sizeof(base));
    snprintf(path, sizeof(path), "%s.jpg", base);
    bool ok;
    if (fs->dual && fs->data2) {
        if (!photo_wide) photo_wide = malloc((640 + 160) * 480 * 2);
        if (!photo_wide) { release_slot(s); snprintf(photo_msg, sizeof(photo_msg), "%s", tr("Out of memory")); return; }
        StitchInfo si = stitch_estimate(fs->data, fs->data2, w, h);
        stitch_compose(fs->data, fs->data2, w, h, &si, photo_wide);
        ok = save_jpeg(path, photo_wide, si.out_w, h, si.out_w * 2);
        if (ok && s_photo_pair) {
            const u8* L = si.a_is_left ? fs->data : fs->data2;
            const u8* R = si.a_is_left ? fs->data2 : fs->data;
            snprintf(path, sizeof(path), "%s_L.jpg", base); ok &= save_jpeg(path, L, w, h, w * 2);
            snprintf(path, sizeof(path), "%s_R.jpg", base); ok &= save_jpeg(path, R, w, h, w * 2);
        }
        if (ok) snprintf(photo_msg, sizeof(photo_msg), tr("Saved %dx%d (+%d px wider)"), si.out_w, h, si.dx);
    } else {
        ok = save_jpeg(path, fs->data, w, h, w * 2);
        if (ok) snprintf(photo_msg, sizeof(photo_msg), tr("Saved %dx%d"), w, h);
    }
    release_slot(s);
    if (!ok) snprintf(photo_msg, sizeof(photo_msg), "%s", tr("Save failed (SD full?)"));
}

// Mode / dual-camera changes: buffers, 3D, camera restart.
static void apply_mode(void) {
    dual_level = 0;
    if (want_dual() && !alloc_dual_buffers()) {
        if (s_mode) s_photo_cam = 0; else s_cam = 0;
        snprintf(photo_msg, sizeof(photo_msg), "%s", tr("Not enough memory for 2 cams"));
    }
    gfxSet3D(want_dual());
    clear_top_buffers();
    cam_reconfig = true;
    photo_due_ms = 0;
    if (!setting_visible(menu_cursor)) menu_cursor = menu_step(menu_cursor, 1);
}

// ---------------------------------------------------------------- UI
static void format_value(const Setting* st, char* out, int n) {
    if (st->labels)           snprintf(out, n, "%s", tr(st->labels[*st->v - st->min]));
    else if (st->flags & SF_DB) snprintf(out, n, "%+d dB", *st->v);
    else                      snprintf(out, n, "%d", *st->v);
}

static void draw_ui(u32 fps_cap, u32 fps_prev, u32 fps_enc) {
    printf("\x1b[0;0H");
    int sk = stat_skip_pct;
    if (rec_active && sk >= 0 && s_encoder == 0)    // static skip at work: show how much was reused
        printf("\x1b[36mSC\x1b[0m cam %2lu view %2lu enc %2lu x%d skip %2d%%\x1b[K\n", fps_cap, fps_prev, fps_enc, num_encoders, sk);
    else
        printf("\x1b[36mSlopCam\x1b[0m cam %2lu view %2lu enc %2lu fps x%d\x1b[K\n", fps_cap, fps_prev, fps_enc, num_encoders);

    if (rec_active) {
        const int ps = rec_pstate;
        u64 t_end = (ps == RS_PAUSE_REQ || ps == RS_PAUSED) ? rec_pause_from : svcGetSystemTick();
        u32 secs = t_end > rec_start_tick ? (u32)((t_end - rec_start_tick) / SYSCLOCK_ARM11) : 0;
        u32 mbmin = secs ? (u32)(rec_bytes * 60 / secs / (1024 * 1024)) : 0;
        if (ps == RS_PAUSED && cam_starting())
            printf(tr("\x1b[33m[PAUSED]\x1b[0m %02lu:%02lu  camera starting...\x1b[K\n"), secs / 60, secs % 60);
        else if (ps == RS_PAUSED)
            printf(tr("\x1b[33m[PAUSED]\x1b[0m %02lu:%02lu  X = resume, Y = stop\x1b[K\n"), secs / 60, secs % 60);
        else if (ps == RS_PAUSE_REQ)
            printf(tr("\x1b[33m[PAUSING...]\x1b[0m %02lu:%02lu\x1b[K\n"), secs / 60, secs % 60);
        else
        {
            // data rate the recording needs / what the SD card manages: when the first gets
            // close to the second, frames start to be dropped ("late")
            u32 need = secs ? (u32)(rec_bytes * 10 / secs / (1024 * 1024)) : 0;   // tenths of MB/s
            u32 sd = rec_sd_kbs * 10 / 1024;
            (void)mbmin;
            if (sd) printf("\x1b[31m[REC]\x1b[0m %02lu:%02lu %4lufr %3lulate %lu.%lu/%lu.%luMB/s\x1b[K\n",
                           secs / 60, secs % 60, stat_rec_frames, stat_rec_drop, need / 10, need % 10, sd / 10, sd % 10);
            else printf("\x1b[31m[REC]\x1b[0m %02lu:%02lu %4lufr %3lulate %lu.%luMB/s\x1b[K\n",
                        secs / 60, secs % 60, stat_rec_frames, stat_rec_drop, need / 10, need % 10);
        }
        // File fill bar: each file stops at ~1.9 GB (AVI 1.0 limit, safely under FAT32's 4 GB)
        // and the next part starts by itself. Then SD space and time left at the current rate.
        char bar[13];
        u32 fill = rec_part_fill, nfill = (fill * 12 + 500) / 1000;
        for (int i = 0; i < 12; i++) bar[i] = i < (int)nfill ? '#' : '.';
        bar[12] = 0;
        printf(tr("File%2d [%s%s\x1b[0m]%3lu%%"), rec_part + 1, fill >= 900 ? "\x1b[33m" : "\x1b[32m", bar, fill / 10);
        if (rec_sd_free0) {
            u64 used = rec_bytes;
            u64 left = rec_sd_free0 > used ? rec_sd_free0 - used : 0;
            u32 bps = secs ? (u32)(used / secs) : 0;   // (paused time isn't counted)
            u32 tl = bps ? (u32)(left / bps) : 0;           // seconds left on the card
            bool low = left < (u64)200 << 20 || (bps && tl < 300);
            printf(" %sSD%3lu.%luG", low ? "\x1b[31m" : "", (u32)(left >> 30), (u32)((left >> 20) % 1024 * 10 / 1024));
            if (bps) { if (tl >= 36000) printf(" 9h+"); else printf(" %lu:%02lu", tl / 3600, tl / 60 % 60); }   // h:mm, fits 40 cols
            printf("\x1b[0m");
        }
        printf("\x1b[K\n");
    } else if (rec_saving) {
        printf("%s", tr("\x1b[33m[SAVING...]\x1b[0m\x1b[K\n"));
    } else if (rec_error) {
        printf("%s", tr("\x1b[31m[SD WRITE ERROR - recording stopped]\x1b[0m\x1b[K\n"));
    } else if (want_dual() && !(dual_ok && dual_level < 2)) {
        printf(tr("\x1b[31m3D off:\x1b[0m %s\x1b[K\n"), tr(!dual_ok ? "not enough memory" : "2nd camera not delivering"));
    } else if (s_mode == 1) {
        if (photo_due_ms) {
            u64 now = osGetTime();
            u32 left = photo_due_ms > now ? (u32)((photo_due_ms - now + 999) / 1000) : 0;
            printf(tr("\x1b[33m[PHOTO]\x1b[0m timer: %lu   (Y = cancel)\x1b[K\n"), left);
        } else {
            printf(tr("\x1b[32m[PHOTO]\x1b[0m %s\x1b[K\n"), photo_msg[0] ? photo_msg : tr("ready - Y to shoot"));
        }
    } else {
        printf(tr("\x1b[32m[VIDEO]\x1b[0m ready%s%s%s\x1b[K\n"), tr(mic_ok ? (s_audio ? "" : " (audio off)") : " (no mic)"),
               tr(want_dual() ? (dual_level ? " 3D 15fps" : " 3D on") : ""), tr(cam_note()));
    }

    static const char* const group_names[] = {"CAMERA", "RECORDING", "PHOTO", "MIC EQ"};
    bool locked = rec_active || rec_saving;

    // Scrolling menu: group headers + visible settings, window kept around the cursor.
    const int MENU_ROWS = rec_active ? 25 : 26;   // one line less while the file bar is shown
    short line_item[64];                  // >= 0 setting index, < 0 = header of group (-1 - g)
    int nlines = 0, cur_line = 0, group = -1;
    for (int i = 0; i < NUM_SETTINGS && nlines < 62; i++) {
        if (!setting_visible(i)) continue;
        if (settings[i].group != group) { group = settings[i].group; line_item[nlines++] = (short)(-1 - group); }
        if (i == menu_cursor) cur_line = nlines;
        line_item[nlines++] = (short)i;
    }
    int top = cur_line - MENU_ROWS / 2;
    if (top > nlines - MENU_ROWS) top = nlines - MENU_ROWS;
    if (top < 0) top = 0;
    char val[24];
    for (int r = 0; r < MENU_ROWS; r++) {
        int l = top + r;
        if (l >= nlines) { printf("\x1b[K\n"); continue; }
        int it = line_item[l];
        if (it < 0) {
            const char* more = (r == 0 && top > 0) ? " ^" : (r == MENU_ROWS - 1 && l < nlines - 1) ? " v" : "";
            printf("\x1b[33m-- %s --%s\x1b[0m\x1b[K\n", tr(group_names[-1 - it]), more);
            continue;
        }
        const Setting* st = &settings[it];
        format_value(st, val, sizeof(val));
        bool sel = it == menu_cursor;
        bool lk = locked && (st->flags & SF_LOCK) && !(rec_pstate == RS_PAUSED && (st->flags & SF_PAUSE));
        char mark = sel ? '>' : (r == 0 && top > 0) ? '^' : (r == MENU_ROWS - 1 && l < nlines - 1) ? 'v' : ' ';
        printf("%s%c %-11s %-17s%s\x1b[0m\x1b[K\n",
               sel ? "\x1b[47;30m" : (lk ? "\x1b[2m" : ""), mark, tr(st->name), val, lk ? (s_lang ? "blok.  " : "locked ") : "       ");
    }

    if (settings[menu_cursor].group == G_MICEQ) draw_meter();   // live level instead of help text
    else printf("\x1b[36m%.39s\x1b[0m\x1b[K\n", tr(settings[menu_cursor].help));
    printf("%s", tr(s_mode ? "A video  Y shoot  SEL gallery"
                           : rec_active ? "Y stop  X pause/resume  L/R x10" : "A photo  Y record  SEL gallery"));
}

// ---------------------------------------------------------------- main
int main(void) {
    gfxInitDefault();
    gfxSetScreenFormat(GFX_TOP, GSP_RGB565_OES);
    gfxSetDoubleBuffering(GFX_TOP, true);
    consoleInit(GFX_BOTTOM, NULL);
    osSetSpeedupEnable(true);
    mkdir(SAVE_FOLDER, 0777);
    settings_load();
    hidSetRepeatParameters(18, 4);       // hold left/right to change values quickly

    LightLock_Init(&slot_lock);
    LightLock_Init(&rq_lock);
    LightEvent_Init(&enc_event, RESET_ONESHOT);
    LightEvent_Init(&rec_event, RESET_ONESHOT);

    // ---- buffers
    bool alloc_ok = true;
    for (int i = 0; i < NUM_SLOTS; i++) {
        slots[i].data = linearAlloc(SLOT_BYTES);
        alloc_ok &= slots[i].data != NULL;
    }
    alloc_ok &= enc_ctx_init(&enc_ctx[0]);   // at least one encoder; more are added per core below

    if (!alloc_ok || R_FAILED(camInit())) {
        printf("\x1b[31m%s\x1b[0m\nPress START to exit.\n", alloc_ok ? "Camera init failed!" : "Out of memory!");
        while (aptMainLoop()) { hidScanInput(); if (hidKeysDown() & KEY_START) break; gspWaitForVBlank(); }
        gfxExit();
        return 0;
    }

    // ---- cameras: hold R = choose by hand, otherwise test which ones deliver frames
    hidScanInput();
    if (hidKeysHeld() & KEY_R) {
        if (!choose_camera()) { camExit(); gfxExit(); return 0; }
    } else {
        check_cameras(slots[0].data);
        if (!outer_avail() && !cam_ok_in) {
            printf("\x1b[31m%s\x1b[0m\n%s", s_lang ? "Zadna kamera nie dziala!" : "No camera is working!",
                   s_lang ? "START = wyjscie.\n" : "Press START to exit.\n");
            while (aptMainLoop()) { hidScanInput(); if (hidKeysDown() & KEY_START) break; gspWaitForVBlank(); }
            camExit(); gfxExit();
            return 0;
        }
    }

    // ---- microphone (optional: without it we record video-only AVIs)
    mic_buf = memalign(0x1000, MIC_BUF_SIZE);
    if (mic_buf && R_SUCCEEDED(micInit(mic_buf, MIC_BUF_SIZE))) {
        MICU_SetPower(true);
        MICU_SetGain((u8)s_mic_gain);
        s32 sz = micGetSampleDataSize();
        mic_size = (sz > 0 && (u32)sz <= MIC_BUF_SIZE - 4 ? (u32)sz : MIC_BUF_SIZE - 4) & ~1u;
        mic_ok = true;
        mic_apply_rate();
        if (s_audio > 0 && !mic_sampling) { MICU_SetPower(false); micExit(); mic_ok = false; }
    }

    // ---- threads
    preview_init();
    hw_preview_init();                   // Y2R + GPU preview self-test (falls back to software)
    fj_set_quality(&photo_q, s_photo_q);
    if (want_dual() && !alloc_dual_buffers()) { if (s_mode) s_photo_cam = 0; else s_cam = 0; }
    gfxSet3D(want_dual());
    rebuild_luts();
    if (!setting_visible(menu_cursor)) menu_cursor = menu_step(menu_cursor, 1);
    clear_top_buffers();

    Thread cap_th = threadCreate(capture_thread, NULL, 0x4000, 0x18, -2, false);

    // JPEG encoding is the heavy part: run one encoder on every core we can use.
    //   New 3DS: core 2 + core 1 + core 0 (idle time only)
    //   Old 3DS: core 1 + core 0 (idle time only)
    // Core 0 gets a low priority so the camera, preview, SD writer and network always win.
    bool is_new3ds = false;
    APT_CheckNew3DS(&is_new3ds);
    APT_SetAppCpuTimeLimit(80);
    Thread enc_th[MAX_ENCODERS] = { NULL };
    {
        const int cores[3]  = { 2, 1, 0 };
        const s32 prios[3]  = { 0x20, 0x20, 0x38 };
        for (int k = is_new3ds ? 0 : 1; k < 3 && num_encoders < MAX_ENCODERS; k++) {
            EncCtx* e = &enc_ctx[num_encoders];
            if (num_encoders > 0 && !enc_ctx_init(e)) { enc_ctx_free(e); break; }
            Thread t = threadCreate(encoder_thread, e, 0x10000, prios[k], cores[k], false);
            if (!t) { if (num_encoders > 0) enc_ctx_free(e); continue; }
            enc_th[num_encoders++] = t;
        }
        if (num_encoders == 0) {   // last resort: anywhere
            Thread t = threadCreate(encoder_thread, &enc_ctx[0], 0x10000, 0x38, -2, false);
            if (t) enc_th[num_encoders++] = t;
        }
    }

    u32 prev_seq = 0;
    u32 fps_cap = 0, fps_prev = 0, fps_enc = 0;
    u64 last_stat_ms = osGetTime();
    int ui_timer = 0;
    bool ui_dirty = true;
    u64 settings_dirty_ms = 0;

    while (aptMainLoop() && running) {
        hidScanInput();
        u32 kDown = hidKeysDown();
        if (kDown & KEY_START) break;

        poll_recording();

        if (kDown & KEY_Y) {
            if (s_mode == 0) {
                if (rec_active) stop_recording();
                else start_recording();
            } else if (photo_due_ms) {
                photo_due_ms = 0;                          // cancel the self-timer
            } else if (val_timer_ms[s_photo_timer]) {
                photo_due_ms = osGetTime() + val_timer_ms[s_photo_timer];
            } else {
                take_photo();
            }
            ui_dirty = true;
        }
        if ((kDown & KEY_X) && s_mode == 0 && rec_active) { toggle_pause(); ui_dirty = true; }
        if ((kDown & KEY_SELECT) && !rec_active && !rec_saving && !photo_due_ms) {
            hw_job_cancel();                       // (holds a camera slot + Y2R)
            cam_suspend = true;
            bool keep = gallery_run(SAVE_FOLDER);
            cam_suspend = false;
            cam_reconfig = true;
            pv_clear = 2;
            clear_top_buffers();
            consoleClear();
            ui_dirty = true;
            if (!keep) break;
            continue;
        }
        if (photo_due_ms && osGetTime() >= photo_due_ms) { photo_due_ms = 0; take_photo(); ui_dirty = true; }
        {   // 3D on exactly while both cameras are actually delivering
            bool want3d = want_dual() && dual_ok && dual_level < 2;
            if (gfxIs3D() != want3d) { gfxSet3D(want3d); clear_top_buffers(); ui_dirty = true; }
        }
        if (photo_due_ms && (osGetTime() % 1000) < 20) ui_dirty = true;   // refresh the countdown

        if ((kDown & KEY_A) && !rec_active && !rec_saving) {   // switch video <-> photo
            s_mode ^= 1;
            photo_msg[0] = 0;
            apply_mode();
            consoleClear();
            settings_dirty_ms = osGetTime();
            ui_dirty = true;
        }

        bool locked = rec_active || rec_saving;
        u32 kRep = hidKeysDownRepeat();
        if (kRep & KEY_UP)   { menu_cursor = menu_step(menu_cursor, -1); ui_dirty = true; }
        if (kRep & KEY_DOWN) { menu_cursor = menu_step(menu_cursor, 1); ui_dirty = true; }
        if (kRep & (KEY_LEFT | KEY_RIGHT | KEY_L | KEY_R)) {
            const Setting* st = &settings[menu_cursor];
            int dir = (kRep & (KEY_RIGHT | KEY_R)) ? 1 : -1;
            if (kRep & (KEY_L | KEY_R)) dir *= 10;               // L/R = big steps
            bool pause_ok = rec_active && rec_pstate == RS_PAUSED && (st->flags & SF_PAUSE);
            if (!(locked && (st->flags & SF_LOCK)) || pause_ok) {
                int v = *st->v + dir;
                if (st->labels) {                                // lists wrap around
                    int span = st->max - st->min + 1;
                    v = st->min + (((v - st->min) % span) + span) % span;
                } else {
                    v = v < st->min ? st->min : (v > st->max ? st->max : v);
                }
                if (v != *st->v) {
                    *st->v = v;
                    if (st->flags & SF_RECONF)   cam_reconfig = true;
                    if (st->flags & SF_DUAL)     { apply_mode(); consoleClear(); }
                    if (st->flags & SF_LUT)      rebuild_luts();
                    if (st->flags & SF_EQ)       miceq_version++;
                    if (st->flags & SF_IMAGE)    apply_camera_settings();
                    if (st->flags & SF_MIC_RATE) mic_apply_rate();
                    if (st->flags & SF_MIC_GAIN) MICU_SetGain((u8)s_mic_gain);
                    settings_dirty_ms = osGetTime();
                }
            }
            ui_dirty = true;
        }
        // Save settings to SD a moment after the last change.
        if (settings_dirty_ms && osGetTime() - settings_dirty_ms > 1500 && !rec_active) {
            settings_save();
            settings_dirty_ms = 0;
        }

        // Preview: newest frame only, written to the back buffer, swapped on vblank.
        // Hardware preview runs asynchronously: finish a pending conversion if it's done,
        // and only look for a new frame when the converter is free.
        bool drew = hw_job_poll();
        if (drew) stat_prev++;
        int s = hw_job.active ? -1 : acquire_latest(&prev_seq);
        if (s >= 0) {
            // While recording, optionally preview every other frame: frees core-0 CPU
            // for the encoder (matters on Old 3DS).
            static bool skip_toggle = false;
            skip_toggle = !skip_toggle;
            bool skip = rec_running() && ((s_rec_prev == 1 && skip_toggle) || s_rec_prev == 2);
            // Preview off: blank the top screen once (a frozen picture would look like a hang).
            static bool pv_blank = false;
            if (rec_running() && s_rec_prev == 2) {
                if (!pv_blank) { hw_job_cancel(); clear_top_buffers(); pv_blank = true; pv_clear = 2; }
            } else {
                pv_blank = false;
            }
            if (!skip && hw_usable(&slots[s]) && hw_job_start(s)) s = -1;    // slot now held by the job
            else if (!skip && preview_frame(s)) { drew = true; stat_prev++; }
            if (s >= 0) release_slot(s);
        }

        u64 now = osGetTime();
        if (now - last_stat_ms >= 1000) {
            fps_cap = stat_cap; fps_prev = stat_prev; fps_enc = stat_enc;
            stat_cap = stat_prev = stat_enc = 0;
            last_stat_ms = now;
            ui_dirty = true;
        }
        if (rec_active && ++ui_timer >= 15) { ui_timer = 0; ui_dirty = true; }
        static int meter_timer = 0;
        if (settings[menu_cursor].group == G_MICEQ && ++meter_timer >= 6) {   // ~10x per second
            meter_timer = 0;
            meter_update();
            ui_dirty = true;
        }
        if (ui_dirty) { draw_ui(fps_cap, fps_prev, fps_enc); ui_dirty = false; }

        gfxFlushBuffers();
        if (drew) gfxSwapBuffers();
        gspWaitForVBlank();
    }

    // ---- shutdown: finish the file first, then stop threads, then services
    if (settings_dirty_ms) settings_save();
    stop_recording();
    if (writer_th) {
        printf("%s", tr("\x1b[2J\x1b[0;0HSaving video...\n"));
        gfxFlushBuffers(); gfxSwapBuffers();
        threadJoin(writer_th, U64_MAX);
        threadFree(writer_th);
        writer_th = NULL;
    }
    running = false;
    LightEvent_Signal(&enc_event);
    for (int i = 0; i < num_encoders; i++) {
        LightEvent_Signal(&enc_event);
        threadJoin(enc_th[i], U64_MAX);
        threadFree(enc_th[i]);
    }
    if (cap_th) { threadJoin(cap_th, U64_MAX); threadFree(cap_th); }

    hw_job_cancel();
    CAMU_Activate(SELECT_NONE);
    camExit();
    hw_preview_exit();
    if (mic_ok) { if (mic_sampling) MICU_StopSampling(); MICU_SetPower(false); micExit(); }
    free(mic_buf);

    for (int i = 0; i < MAX_ENCODERS; i++) enc_ctx_free(&enc_ctx[i]);
    for (int i = 0; i < REC_QUEUE; i++) free(rec_q[i].data);   // anything left after a stop
    for (int i = 0; i < NUM_SLOTS; i++) { linearFree(slots[i].data); if (slots[i].data2) linearFree(slots[i].data2); }
    free(photo_jpeg);
    free(photo_wide);

    gfxExit();
    return 0;
}
