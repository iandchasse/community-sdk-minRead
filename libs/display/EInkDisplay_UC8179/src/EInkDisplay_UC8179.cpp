// EInkDisplay_UC8179.cpp
// Driver for GDEY075T7 (800x480, UC8179 controller).
//
// Sequence mirrors GxEPD2_750_GDEY075T7.cpp (ZinggJM/GxEPD2) exactly:
//
//   _initDisplay()  — PSR=0x1f, PWR, BTST, TRES, DUSPI, CDI, TCON, PWS (no PON)
//   _initFull()     — _initDisplay() + PSR=0x1f + PON
//   _initPart()     — _initDisplay() + PSR=0x3f + register LUTs + PON
//   _initGray()     — _initDisplay() + PSR=0x3f + gray LUTs + PON
//   _updateFull()   — CCSET=0x02 + TSSET=0x5A (90C) + DRF
//   _updatePart()   — DRF only
//
// DTM write pattern uses partial window addressing (GxEPD2 style):
//   CMD_PART_IN (0x91) + CMD_PART_WINDOW (0x90) + CMD_DTM + data + CMD_PART_OUT (0x92)
//
// Key facts:
//   - busy_level = LOW: BUSY pin LOW while busy, HIGH when ready
//   - PON fires once per mode-switch, never before DRF

#include "EInkDisplay_UC8179.h"

#include <cstring>

#ifndef ARDUINO
#include <fstream>
#include <vector>
#endif

// ============================================================================
// UC8179 command bytes
// ============================================================================
#define CMD_PSR 0x00
#define CMD_PWR 0x01
#define CMD_POF 0x02
#define CMD_PON 0x04
#define CMD_BTST 0x06
#define CMD_DSLP 0x07
#define CMD_DTM1 0x10
#define CMD_DSP 0x11
#define CMD_DRF 0x12
#define CMD_DTM2 0x13
#define CMD_DUSPI 0x15
#define CMD_LUTC 0x20
#define CMD_LUTWW 0x21
#define CMD_LUTKW 0x22
#define CMD_LUTWK 0x23
#define CMD_LUTKK 0x24
#define CMD_LUTBD 0x25
#define CMD_CDI 0x50
#define CMD_TCON 0x60
#define CMD_TRES 0x61
#define CMD_TSE 0x41
#define CMD_VDCS 0x82
#define CMD_CCSET 0xE0
#define CMD_TSSET 0xE5
#define CMD_PWS 0xE3

#define CMD_PART_IN 0x91
#define CMD_PART_WINDOW 0x90
#define CMD_PART_OUT 0x92

#define PSR_OTP 0x1f
#define PSR_REG 0x3f

#define CDI0_LUTKW 0x29
#define CDI0_LUTBD 0x39
#define CDI1 0x07

// ============================================================================
// LUT format: 7 groups x 6 bytes = 42 bytes, zero-padded
// Byte 0:   phase voltages 4x2-bit MSB-first: 00=GND 01=VDH+ 10=VDL- 11=VCOM
// Bytes 1-4: frame counts phases A-D
// Byte 5:   repeat count (0 = run once)
//
// Transition routing: DTM1=old, DTM2=new
//   WW->LUTWW, KW->LUTKW (blk->wht), WK->LUTWK (wht->blk), KK->LUTKK
// ============================================================================

// ============================================================================
// VOLTAGE ENCODING (confirmed from UC8179 datasheet):
//   00 = GND  (no drive)
//   01 = VDH  -> drives pixel BLACK  (positive high voltage)
//   10 = VDL  -> drives pixel WHITE  (negative low voltage)
//   11 = VDHR (unused in KW mode, treat as floating)
//
// Phase byte: [Ph_A | Ph_B | Ph_C | Ph_D] each 2 bits, MSB first
//   e.g. 0x80 = 10_00_00_00 = PhA=VDL(white), PhB-D=GND
//        0x40 = 01_00_00_00 = PhA=VDH(black), PhB-D=GND
//        0x60 = 01_10_00_00 = PhA=VDH(black), PhB=VDL(white)
//        0x90 = 10_01_00_00 = PhA=VDL(white), PhB=VDH(black)
// ============================================================================

// ============================================================================
// BW Partial LUT — anti-ghosting design
//
// DC-offset ghosting (faint previous text visible) means particles didn't fully
// switch from the prior state. Fix: reversal pre-phase dislodges stuck particles
// by briefly pushing the OPPOSITE direction before the main drive.
//
//   Phase A (reversal):   push OPPOSITE direction for TP_REV frames
//   Phase B (main drive): push to TARGET for TP_MAIN frames
//   Phase C (settle):     GND hold for TP_SETTLE frames
//
// LUTWW/LUTKK: unchanged pixels — genuine NOP, 1 frame GND.
// DRF time = TP_REV + TP_MAIN + TP_SETTLE frames.
//
// Tuning:
//   TP_REV:    increase (up to ~20) if ghosting persists after page turns
//   TP_MAIN:   increase (up to ~40) if new content looks gray/faint
//   TP_SETTLE: 2-4 is sufficient
// ============================================================================
#define TP_REV 10
#define TP_MAIN 30
#define TP_SETTLE 2

// LUTC: VCOM at GND throughout partial refresh
static const uint8_t lut_partial_LUTC[42] PROGMEM = {
    0x00, TP_REV, TP_MAIN, TP_SETTLE, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,      0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTWW: white->white NOP
static const uint8_t lut_partial_LUTWW[42] PROGMEM = {
    0x00, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTKW: black->white
//   PhA reversal: VDH (01) = push black (same direction) — wait, black is already
//   black. Reversal means push toward WHITE first to dislodge stuck black particles.
//   PhA=VDL(10)=white reversal, PhB=VDL(10)=continue white drive, PhC=GND settle.
//   Strong sustained white drive after brief white pre-charge.
//   0x80 = 10_00_00_00: single phase, pure white drive — simple and clean
//   With reversal: 0xA0 = 10_10_00_00: PhA+PhB both VDL(white)
//   Use two-phase: PhA short reversal push (toward black=VDH), PhB long white drive.
//   Reversal for K->W: particles are black, push toward black MORE first is wrong.
//   Correct reversal for K->W: particles STUCK at black = push white(VDL) hard.
//   There is no "opposite" for a fully-set black pixel. The pre-phase should be
//   a brief VDH(black) to re-saturate, then a long VDL(white) to fully flip.
//   PhA=VDH(01) re-saturate black, PhB=VDL(10) drive white, PhC=GND
//   Encoding: 01_10_00_00 = 0x60
static const uint8_t lut_partial_LUTKW[42] PROGMEM = {
    0x60, TP_REV, TP_MAIN, TP_SETTLE, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,      0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTWK: white->black
//   PhA=VDL(10) re-saturate white, PhB=VDH(01) drive black, PhC=GND
//   Encoding: 10_01_00_00 = 0x90
static const uint8_t lut_partial_LUTWK[42] PROGMEM = {
    0x90, TP_REV, TP_MAIN, TP_SETTLE, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,      0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTKK: black->black NOP
static const uint8_t lut_partial_LUTKK[42] PROGMEM = {
    0x00, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTBD: border same as LUTC
static const uint8_t lut_partial_LUTBD[42] PROGMEM = {
    0x00, TP_REV, TP_MAIN, TP_SETTLE, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,      0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

// ============================================================================
// AA Gray LUT — 2-level anti-aliasing from known BW starting state
//
// Starting state: white background (VDL-saturated), black text (VDH-saturated).
// storeBwBuffer captures this exact state into frameBufferPrev.
//
// Compositing routes pixels through LUT based on (DTM1=old, DTM2=new):
//   (K,K) LUTKK -> black stays black      NOP
//   (W,W) LUTWW -> white stays white      NOP
//   (K,W) LUTKW -> black->light gray      partial VDL drive (push toward white)
//   (W,K) LUTWK -> white->dark gray       partial VDH drive (push toward black)
//
// Single-phase drive: since starting state is known (fully saturated BW),
// we just drive partway toward the opposite pole to land at gray.
//
// DRF time = TG_MAIN + TG_SETTLE frames. NOPs run for 1 frame only.
//
// Tuning TG_MAIN:
//   Too low  -> gray too light / barely visible
//   Too high -> gray too dark, looks almost BW
//   Start 15, adjust +-3 until gray levels look right.
// ============================================================================
#define TG_MAIN 8    // frames to reach gray — reduce if text thins/disappears
#define TG_SETTLE 3  // settling at GND after drive

// LUTC: apply VCOM during gray drive (11=VCOM helps move particles)
static const uint8_t lut_gray_LUTC[42] PROGMEM = {
    0xC0, TG_MAIN, TG_SETTLE, 0, 0, 1,  // PhA=VCOM(11), PhB=GND(00)
    0,    0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTWW: white->white, same timing as active LUTs so DRF waits correctly
static const uint8_t lut_gray_LUTWW[42] PROGMEM = {
    0x00, TG_MAIN, TG_SETTLE, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTKW: black->light gray. VDL(10)=white drive for TG_MAIN frames.
// 0x80 = 10_00_00_00
static const uint8_t lut_gray_LUTKW[42] PROGMEM = {
    0x80, TG_MAIN, TG_SETTLE, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTWK: white->dark gray. VDH(01)=black drive for TG_MAIN frames.
// 0x40 = 01_00_00_00
static const uint8_t lut_gray_LUTWK[42] PROGMEM = {
    0x40, TG_MAIN, TG_SETTLE, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTKK: black->black, match timing
static const uint8_t lut_gray_LUTKK[42] PROGMEM = {
    0x00, TG_MAIN, TG_SETTLE, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
// LUTBD: border, match timing
static const uint8_t lut_gray_LUTBD[42] PROGMEM = {
    0x00, TG_MAIN, TG_SETTLE, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,       0,         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

// ============================================================================
// Constructor
// ============================================================================
EInkDisplay_UC8179::EInkDisplay_UC8179(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy)
    : _sclk(sclk),
      _mosi(mosi),
      _cs(cs),
      _dc(dc),
      _rst(rst),
      _busy(busy),
      frameBuffer(nullptr),
      bwBase(nullptr),
      cached_lsb_mask(nullptr)
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
      ,
      frameBufferPrev(nullptr)
#endif
{
  if (Serial)
    Serial.printf("[%lu] EInkDisplay_UC8179: SCLK=%d MOSI=%d CS=%d DC=%d RST=%d BUSY=%d\n", millis(), sclk, mosi, cs,
                  dc, rst, busy);
}

// ============================================================================
// begin()
// ============================================================================
void EInkDisplay_UC8179::begin() {
  if (Serial) Serial.printf("[%lu] begin()\n", millis());

  frameBuffer = frameBuffer0;
  memset(frameBuffer0, 0xFF, BUFFER_SIZE);

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  frameBufferPrev = frameBuffer1;
  memset(frameBuffer1, 0xFF, BUFFER_SIZE);
#endif

  SPI.begin(_sclk, /*miso*/ -1, _mosi, _cs);
  spiSettings = SPISettings(20000000, MSBFIRST, SPI_MODE0);

  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  pinMode(_dc, OUTPUT);
  digitalWrite(_dc, HIGH);
  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, HIGH);
  pinMode(_busy, INPUT);

  hardwareReset();
  _initDisplay();

  if (Serial) Serial.printf("[%lu] begin() done\n", millis());
}

// ============================================================================
// Frame-buffer operations
// ============================================================================
void EInkDisplay_UC8179::clearScreen(const uint8_t color) const { memset(frameBuffer, color, BUFFER_SIZE); }

void EInkDisplay_UC8179::setFramebuffer(const uint8_t* buf) const { memcpy(frameBuffer, buf, BUFFER_SIZE); }

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
void EInkDisplay_UC8179::swapBuffers() {
  uint8_t* tmp = frameBuffer;
  frameBuffer = frameBufferPrev;
  frameBufferPrev = tmp;
}
#endif

void EInkDisplay_UC8179::drawImage(const uint8_t* img, uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool pgm) const {
  if (!frameBuffer) return;
  const uint16_t wb = w / 8;
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t dy = y + row;
    if (dy >= DISPLAY_HEIGHT) break;
    uint16_t doff = dy * DISPLAY_WIDTH_BYTES + (x / 8);
    uint16_t soff = row * wb;
    for (uint16_t col = 0; col < wb; col++) {
      if ((x / 8 + col) >= DISPLAY_WIDTH_BYTES) break;
      frameBuffer[doff + col] = pgm ? pgm_read_byte(&img[soff + col]) : img[soff + col];
    }
  }
}

void EInkDisplay_UC8179::drawImageTransparent(const uint8_t* img, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                              bool pgm) const {
  if (!frameBuffer) return;
  const uint16_t wb = w / 8;
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t dy = y + row;
    if (dy >= DISPLAY_HEIGHT) break;
    uint16_t doff = dy * DISPLAY_WIDTH_BYTES + (x / 8);
    uint16_t soff = row * wb;
    for (uint16_t col = 0; col < wb; col++) {
      if ((x / 8 + col) >= DISPLAY_WIDTH_BYTES) break;
      uint8_t src = pgm ? pgm_read_byte(&img[soff + col]) : img[soff + col];
      frameBuffer[doff + col] &= src;
    }
  }
}

// ============================================================================
// Grayscale buffer helpers
// ============================================================================
void EInkDisplay_UC8179::storeBwBase(const uint8_t* bwBuffer) {
  // Called by GfxRenderer::storeBwBuffer() while frameBuffer still holds the
  // BW page — before clearScreen(0x00) for the gray render passes.
  if (!bwBase) bwBase = (uint8_t*)malloc(BUFFER_SIZE);
  if (bwBase)
    memcpy(bwBase, bwBuffer, BUFFER_SIZE);
  else if (Serial)
    Serial.println("storeBwBase: malloc failed");
}
void EInkDisplay_UC8179::copyGrayscaleLsbBuffers(const uint8_t* lsb) {
  if (!cached_lsb_mask) {
    cached_lsb_mask = (uint8_t*)malloc(BUFFER_SIZE);
  }
  if (cached_lsb_mask) {
    memcpy(cached_lsb_mask, lsb, BUFFER_SIZE);
  } else {
    if (Serial) Serial.println("malloc failed for cached_lsb_mask");
  }
}

void EInkDisplay_UC8179::copyGrayscaleMsbBuffers(const uint8_t* msb) {
  if (!cached_lsb_mask) return;

  // bwBase is snapshot of the BW frame taken inside displayBuffer before DRF.
  // frameBuffer/frameBufferPrev are wrong here: frameBuffer has been clearScreen'd
  // to 0x00 for the LSB/MSB render passes, and in SINGLE_BUFFER_MODE there is no
  // separate prev buffer. bwBase is the only reliable reference to the pre-gray
  // BW page state needed for compositing.
  uint8_t* base = bwBase;
  if (!base) {
    if (Serial) Serial.println("copyGrayscaleMsbBuffers: bwBase not set, gray composite aborted");
    free(cached_lsb_mask);
    cached_lsb_mask = nullptr;
    return;
  }

  uint8_t* target = (uint8_t*)malloc(BUFFER_SIZE);
  if (target) {
    // Compositing formula — derived from renderer + LUT routing:
    //
    // Renderer uses clearScreen(0x00) then sets bits to 1 for gray pixels:
    //   msb buf: bit=1 for bmpVal==1 (dark gray) OR bmpVal==2 (light gray)
    //   lsb buf: bit=1 for bmpVal==1 (dark gray) only
    //
    // UC8179 LUT routes on (DTM1=old, DTM2=new):
    //   (0,0) LUTKK  (1,1) LUTWW  (0,1) LUTKW  (1,0) LUTWK
    //
    // Goal: when msb=1, drive the pixel toward the OPPOSITE of its base state.
    //   base=0 (black), msb=1 -> LUTKW (black->light gray) -> need DTM1=0, DTM2=1
    //   base=1 (white), msb=1 -> LUTWK (white->dark gray)  -> need DTM1=1, DTM2=0
    //   base=0, msb=0 -> LUTKK NOP -> DTM1=0, DTM2=0
    //   base=1, msb=0 -> LUTWW NOP -> DTM1=1, DTM2=1
    //
    // Solution:
    //   DTM1 = base            (preserves routing direction)
    //   DTM2 = base ^ msb      (flips only where msb=1)
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
      target[i] = base[i];
    }
    writeDTM1(target, BUFFER_SIZE);

    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
      target[i] = base[i] ^ msb[i];
    }
    writeDTM2(target, BUFFER_SIZE);

    free(target);
  } else {
    if (Serial) Serial.println("malloc failed for gray composite");
  }

  free(cached_lsb_mask);
  cached_lsb_mask = nullptr;
  free(bwBase);
  bwBase = nullptr;
}

void EInkDisplay_UC8179::copyGrayscaleBuffers(const uint8_t* lsb, const uint8_t* msb) {
  copyGrayscaleLsbBuffers(lsb);
  copyGrayscaleMsbBuffers(msb);
}

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
void EInkDisplay_UC8179::cleanupGrayscaleBuffers(const uint8_t* bwBuffer) { writeDTM1(bwBuffer, BUFFER_SIZE); }
#endif

// ============================================================================
// displayBuffer
// ============================================================================
void EInkDisplay_UC8179::displayBuffer(RefreshMode mode, bool turnOffScreen) {
  if (inGrayscaleMode) {
    grayscaleRevert();
    inGrayscaleMode = false;
  }

  if (mode == HALF_REFRESH) {
    _initFull();
    writeDTM1(frameBuffer, BUFFER_SIZE);
    writeDTM2(frameBuffer, BUFFER_SIZE);
    _updateFull();
    _initPart();
  } else if (mode == FAST_REFRESH) {
    if (!_using_partial_mode) _initPart();
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    writeDTM1(frameBufferPrev, BUFFER_SIZE);
#endif
    writeDTM2(frameBuffer, BUFFER_SIZE);
    _updatePart();
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    swapBuffers();
#endif
  } else {
    // FULL_REFRESH
    _initFull();
    writeDTM1(frameBuffer, BUFFER_SIZE);
    writeDTM2(frameBuffer, BUFFER_SIZE);
    _updateFullSlow();
    _initPart();
  }

  if (turnOffScreen) {
    _powerOff();
  } else {
    // Pre-initialise gray mode so the controller SRAM stays live.
    // copyGrayscaleLsbBuffers/copyGrayscaleMsbBuffers will write DTM
    // while power is on. displayGrayBuffer then fires DRF directly —
    // no PON cycle that would wipe the SRAM we just loaded.
    _initGray();
  }
}

// ============================================================================
// displayGrayBuffer
// _initGray() was already called at the end of displayBuffer while power was
// still on. DTM SRAM was written by copyGrayscaleMsbBuffers with power live.
// Just fire DRF here — DO NOT call _initGray() again or it will POF/PON and
// wipe the SRAM contents before DRF can read them.
// ============================================================================
void EInkDisplay_UC8179::displayGrayBuffer(bool turnOffScreen) {
  if (Serial) Serial.printf("[%lu] displayGrayBuffer\n", millis());
  inGrayscaleMode = true;
  _updatePart();
  if (turnOffScreen) _powerOff();
}

// ============================================================================
// grayscaleRevert
// No-op: the next displayBuffer(FAST_REFRESH) supplies fresh DTM1/DTM2 with
// the BW frame, and the anti-ghosting partial LUT waveform will pull gray
// particles cleanly to full BW during the normal page turn.
// ============================================================================
void EInkDisplay_UC8179::grayscaleRevert() {
  if (Serial) Serial.printf("[%lu] grayscaleRevert (no-op)\n", millis());
}

void EInkDisplay_UC8179::setCustomLUT(bool enabled, const unsigned char*) { (void)enabled; }

// ============================================================================
// deepSleep
// ============================================================================
void EInkDisplay_UC8179::deepSleep() {
  if (Serial) Serial.printf("[%lu] deepSleep\n", millis());
  _powerOff();
  sendCommand(CMD_DSLP);
  sendData(0xA5);
  _using_partial_mode = false;
}

// ============================================================================
// refreshDisplay — shim for legacy call sites
// ============================================================================
void EInkDisplay_UC8179::refreshDisplay(RefreshMode mode, bool turnOffScreen) { displayBuffer(mode, turnOffScreen); }

// ============================================================================
// _initDisplay — exact GxEPD2 _InitDisplay() register order
// ============================================================================
void EInkDisplay_UC8179::_initDisplay() {
  if (Serial) Serial.printf("[%lu] _initDisplay\n", millis());

  sendCommand(CMD_PSR);
  sendData(PSR_OTP);

  sendCommand(CMD_PWR);
  sendData(0x07);
  sendData(0x07);
  sendData(0x3f);
  sendData(0x3f);
  sendData(0x09);

  sendCommand(CMD_BTST);
  sendData(0x17);
  sendData(0x17);
  sendData(0x28);
  sendData(0x17);

  sendCommand(CMD_TRES);
  sendData(0x03);
  sendData(0x20);
  sendData(0x01);
  sendData(0xE0);

  sendCommand(CMD_DUSPI);
  sendData(0x00);
  sendCommand(CMD_CDI);
  sendData(CDI0_LUTKW);
  sendData(CDI1);
  sendCommand(CMD_TCON);
  sendData(0x22);
  sendCommand(CMD_PWS);
  sendData(0x22);
}

// ============================================================================
// _initFull
// ============================================================================
void EInkDisplay_UC8179::_initFull() {
  if (Serial) Serial.printf("[%lu] _initFull\n", millis());
  _powerOff();
  _initDisplay();
  sendCommand(CMD_PSR);
  sendData(PSR_OTP);
  _powerOn();
  _using_partial_mode = false;
}

// ============================================================================
// _initPart — anti-ghosting register LUT partial mode
// ============================================================================
void EInkDisplay_UC8179::_initPart() {
  if (Serial) Serial.printf("[%lu] _initPart\n", millis());
  _initDisplay();
  sendCommand(CMD_PSR);
  sendData(PSR_REG);
  sendCommand(CMD_VDCS);
  sendData(0x30);
  sendCommand(CMD_CDI);
  sendData(CDI0_LUTBD);
  sendData(CDI1);
  sendLutRegister(CMD_LUTC, lut_partial_LUTC);
  sendLutRegister(CMD_LUTWW, lut_partial_LUTWW);
  sendLutRegister(CMD_LUTKW, lut_partial_LUTKW);
  sendLutRegister(CMD_LUTWK, lut_partial_LUTWK);
  sendLutRegister(CMD_LUTKK, lut_partial_LUTKK);
  sendLutRegister(CMD_LUTBD, lut_partial_LUTBD);
  _powerOn();
  _using_partial_mode = true;
}

// ============================================================================
// _initGray — fast 2-level AA gray mode
// ============================================================================
void EInkDisplay_UC8179::_initGray() {
  if (Serial) Serial.printf("[%lu] _initGray\n", millis());
  _initDisplay();
  sendCommand(CMD_PSR);
  sendData(PSR_REG);
  sendCommand(CMD_VDCS);
  sendData(0x30);
  sendCommand(CMD_CDI);
  sendData(0x31);
  sendData(CDI1);
  sendLutRegister(CMD_LUTC, lut_gray_LUTC);
  sendLutRegister(CMD_LUTWW, lut_gray_LUTWW);
  sendLutRegister(CMD_LUTKW, lut_gray_LUTKW);
  sendLutRegister(CMD_LUTWK, lut_gray_LUTWK);
  sendLutRegister(CMD_LUTKK, lut_gray_LUTKK);
  sendLutRegister(CMD_LUTBD, lut_gray_LUTBD);
  _powerOn();
  _using_partial_mode = false;  // gray uses different regs, force re-init on next BW
}

// ============================================================================
// _updateFull / _updateFullSlow / _updatePart
// ============================================================================
void EInkDisplay_UC8179::_updateFull() {
  if (Serial) Serial.printf("[%lu] _updateFull (DRF)...\n", millis());
  sendCommand(CMD_CCSET);
  sendData(0x02);
  sendCommand(CMD_TSSET);
  sendData(0x5A);
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF full");
}

void EInkDisplay_UC8179::_updateFullSlow() {
  if (Serial) Serial.printf("[%lu] _updateFullSlow (DRF)...\n", millis());
  sendCommand(CMD_CCSET);
  sendData(0x00);
  sendCommand(CMD_TSE);
  sendData(0x00);
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF full slow");
}

void EInkDisplay_UC8179::_updatePart() {
  if (Serial) Serial.printf("[%lu] _updatePart (DRF)...\n", millis());
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF part");
}

// ============================================================================
// _powerOn / _powerOff
// ============================================================================
void EInkDisplay_UC8179::_powerOn() {
  if (!_power_is_on) {
    if (Serial) Serial.printf("[%lu] PON\n", millis());
    sendCommand(CMD_PON);
    waitWhileBusy("PON");
  }
  _power_is_on = true;
}

void EInkDisplay_UC8179::_powerOff() {
  if (_power_is_on) {
    if (Serial) Serial.printf("[%lu] POF\n", millis());
    sendCommand(CMD_POF);
    waitWhileBusy("POF");
  }
  _power_is_on = false;
  _using_partial_mode = false;
}

// ============================================================================
// hardwareReset — GxEPD2 timing
// ============================================================================
void EInkDisplay_UC8179::hardwareReset() {
  if (Serial) Serial.printf("[%lu] hardwareReset\n", millis());
  digitalWrite(_rst, HIGH);
  delay(20);
  digitalWrite(_rst, LOW);
  delay(10);
  digitalWrite(_rst, HIGH);
  delay(200);
  if (Serial) Serial.printf("[%lu] hardwareReset done\n", millis());
}

// ============================================================================
// SPI
// ============================================================================
void EInkDisplay_UC8179::sendCommand(uint8_t cmd) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, LOW);
  digitalWrite(_cs, LOW);
  SPI.transfer(cmd);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

void EInkDisplay_UC8179::sendData(uint8_t data) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  SPI.transfer(data);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

void EInkDisplay_UC8179::sendData(const uint8_t* data, uint32_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  SPI.writeBytes(data, length);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

// ============================================================================
// waitWhileBusy — BUSY LOW = busy, HIGH = ready
// ============================================================================
void EInkDisplay_UC8179::waitWhileBusy(const char* tag) {
  delay(1);
  unsigned long t0 = millis();
  while (digitalRead(_busy) == LOW) {
    delay(1);
    if (millis() - t0 > 10000) {
      if (Serial) Serial.printf("[%lu] TIMEOUT: %s\n", millis(), tag ? tag : "");
      break;
    }
  }
  if (tag && Serial) Serial.printf("[%lu] done: %s (%lu ms)\n", millis(), tag, millis() - t0);
}

// ============================================================================
// Partial window addressing
// ============================================================================
void EInkDisplay_UC8179::_setPartialWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint16_t xe = (x + w - 1) | 0x0007;
  uint16_t ye = y + h - 1;
  x &= 0xFFF8;
  sendCommand(CMD_PART_WINDOW);
  sendData(x / 256);
  sendData(x % 256);
  sendData(xe / 256);
  sendData(xe % 256);
  sendData(y / 256);
  sendData(y % 256);
  sendData(ye / 256);
  sendData(ye % 256);
  sendData(0x01);
}

void EInkDisplay_UC8179::writeDTM1(const uint8_t* data, uint32_t size) {
  if (Serial) Serial.printf("[%lu] writeDTM1 %lu B\n", millis(), (unsigned long)size);
  sendCommand(CMD_PART_IN);
  _setPartialWindow(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  sendCommand(CMD_DTM1);
  sendData(data, size);
  sendCommand(CMD_PART_OUT);
}

void EInkDisplay_UC8179::writeDTM2(const uint8_t* data, uint32_t size) {
  if (Serial) Serial.printf("[%lu] writeDTM2 %lu B\n", millis(), (unsigned long)size);
  sendCommand(CMD_PART_IN);
  _setPartialWindow(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  sendCommand(CMD_DTM2);
  sendData(data, size);
  sendCommand(CMD_PART_OUT);
}

void EInkDisplay_UC8179::sendLutRegister(uint8_t cmd, const uint8_t* pgmData) {
  uint8_t buf[42];
  memcpy_P(buf, pgmData, 42);
  sendCommand(cmd);
  sendData(buf, 42);
}

// ============================================================================
// saveFrameBufferAsPBM (desktop/test only)
// ============================================================================
void EInkDisplay_UC8179::saveFrameBufferAsPBM(const char* filename) {
#ifndef ARDUINO
  std::ofstream file(filename, std::ios::binary);
  if (!file) return;
  const int W = DISPLAY_WIDTH, H = DISPLAY_HEIGHT, WB = DISPLAY_WIDTH_BYTES;
  file << "P4\n" << H << " " << W << "\n";
  std::vector<uint8_t> out((H / 8) * W, 0);
  for (int oY = 0; oY < W; oY++) {
    for (int oX = 0; oX < H; oX++) {
      int iX = oY, iY = H - 1 - oX;
      bool w = (frameBuffer[iY * WB + (iX / 8)] >> (7 - (iX % 8))) & 1;
      if (!w) out[oY * (H / 8) + (oX / 8)] |= (1 << (7 - (oX % 8)));
    }
  }
  file.write(reinterpret_cast<const char*>(out.data()), out.size());
#else
  (void)filename;
#endif
}

// ============================================================================
// diagnose()
// ============================================================================
void EInkDisplay_UC8179::diagnose() {
  Serial.println("=== EInkDisplay_UC8179::diagnose() ===");
  Serial.printf("  BUSY at idle: %d  (expected 1 = HIGH = ready)\n", digitalRead(_busy));
  Serial.println("  Sending PON and watching BUSY for 500ms...");
  sendCommand(CMD_PON);
  unsigned long t0 = millis();
  bool wentLow = false;
  while (millis() - t0 < 500) {
    if (digitalRead(_busy) == LOW) {
      wentLow = true;
      break;
    }
    delay(1);
  }
  if (wentLow) {
    Serial.printf("  BUSY went LOW %lu ms after PON\n", millis() - t0);
    t0 = millis();
    while (digitalRead(_busy) == LOW && millis() - t0 < 5000) delay(1);
    Serial.printf("  BUSY HIGH again after %lu ms\n", millis() - t0);
  } else {
    Serial.println("  BUSY never went LOW — booster not starting");
  }
  sendCommand(CMD_POF);
  delay(100);
  Serial.println("=== diagnose() done ===");
}

uint8_t EInkDisplay_UC8179::sendCommandReadByte(uint8_t cmd) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, LOW);
  digitalWrite(_cs, LOW);
  SPI.transfer(cmd);
  digitalWrite(_dc, HIGH);
  uint8_t result = SPI.transfer(0xFF);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  return result;
}