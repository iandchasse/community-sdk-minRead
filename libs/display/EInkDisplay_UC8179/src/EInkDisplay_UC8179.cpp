// EInkDisplay_UC8179.cpp
// Driver for GDEY075T7 (800x480, UC8179 controller).
//
// Sequence mirrors GxEPD2_750_GDEY075T7.cpp (ZinggJM/GxEPD2) exactly:
//
//   _InitDisplay()  — PSR=0x1f, PWR, BTST, TRES, DUSPI, CDI, TCON, PWS (no PON)
//   _Init_Full()    — _InitDisplay() + PSR=0x1f + PON
//   _Init_Part()    — _InitDisplay() + CCSET=0x02 + TSSET=0x6E (110C OTP fast partial) + PON
//                     Partial uses OTP waveform with forced temp — NOT register LUTs.
//                     PSR stays 0x1f. No LUT registers written.
//   _Update_Full()  — CCSET=0x02 + TSSET=0x5A (90C) + DRF
//   _Update_Part()  — DRF only
//
// DTM write pattern uses partial window addressing (GxEPD2 style):
//   CMD_PART_IN (0x91) + CMD_PART_WINDOW (0x90) [x,xe,y,ye,0x01] + CMD_DTM2 (0x13) + data + CMD_PART_OUT (0x92)
//
// Key facts:
//   - busy_level = LOW: BUSY pin LOW while busy, HIGH when ready
//   - PSR=0x1f for ALL modes — partial uses forced temperature not register LUTs
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

// Partial window addressing (GxEPD2 uses these around every DTM write)
#define CMD_PART_IN 0x91      // partial mode in
#define CMD_PART_WINDOW 0x90  // set partial window [x, xe, y, ye, flags]
#define CMD_PART_OUT 0x92     // partial mode out

#define PSR_OTP 0x1f  // KW mode, OTP LUTs  (full refresh)
#define PSR_REG 0x3f  // KW mode, register LUTs (partial/gray)

// CDI byte 0
#define CDI0_LUTKW 0x29  // LUTKW border, N2OCP (GxEPD2 _InitDisplay default)
#define CDI0_LUTBD 0x39  // LUTBD border, N2OCP (GxEPD2 _Init_Part)
#define CDI1 0x07

// ============================================================================
// LUT format: 7 groups × 6 bytes = 42 bytes, zero-padded
// Byte 0: phase voltages MSB-first, 4×2-bit: 00=GND 01=VDL- 10=VDH+ 11=VCOM
// Bytes 1-4: frame counts phases A-D
// Byte 5: repeat count (0 = run once)
//
// Transition routing (DTM1=old, DTM2=new):
//   WW → LUTWW,  KW → LUTKW,  WK → LUTWK,  KK → LUTKK
// ============================================================================

// ============================================================================
// ============================================================================
// ============================================================================
// ============================================================================
// Custom Strong Anti-Ghosting Partial LUTs
// Designed to aggressively clear mid-state gray particles during a normal B/W
// page turn, without flashing the unchanged background pixels.
//
// Phase A: Push opposite direction (3 frames) to loosen particles
// Phase B: Drive hard to target (35 frames)
// Phase C: Push opposite direction (2 frames) to brake
// Total: 40 frames (fast page turn speed)
// ============================================================================
static const uint8_t lut_partial_LUTC[42] PROGMEM = {
    0x00, 40, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
static const uint8_t lut_partial_LUTWW[42] PROGMEM = {
    0x00, 40, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
static const uint8_t lut_partial_LUTKW[42] PROGMEM = {
    // Black to White (Target VDH+) = 01_10_01_10 = 0x66
    0x66, 3, 35, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
static const uint8_t lut_partial_LUTWK[42] PROGMEM = {
    // White to Black (Target VDL-) = 10_01_10_01 = 0x99
    0x99, 3, 35, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
static const uint8_t lut_partial_LUTKK[42] PROGMEM = {
    0x00, 40, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
static const uint8_t lut_partial_LUTBD[42] PROGMEM = {
    0x00, 40, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

// ============================================================================
// Hybrid Absolute Gray Compositing (No-Flicker Background)
//
// The renderer computes absolute 4-shade pixels via DTM1/DTM2.
// 00=Black  11=White  01=DarkGray  10=LightGray
//
// To avoid flashing the background, 00 and 11 are pure NOP.
// The gray pixels (01, 10) use the validated 100-frame GxEPD2 waveforms
// which perfectly drive particles from any state to the correct gray level.
// ============================================================================
#define T_MAX 100  // GxEPD2 4G waveforms are exactly 100 frames

// LUTC (VCOM): no VCOM drive during gray overlay to minimize flicker
static const uint8_t lut_gray_LUTC[42] PROGMEM = {
    0x00, T_MAX, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

// LUTWW (11, White Target): Pure NOP to prevent flashing white background
static const uint8_t lut_gray_LUTWW[42] PROGMEM = {
    0x00, T_MAX, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

// LUTWK (01, Dark Gray Target): Multi-phase GxEPD2_4G waveform (100 frames)
static const uint8_t lut_gray_LUTWK[42] PROGMEM = {
    0x40, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x90, 0x14, 0x14, 0x00, 0x00, 0x01, 0x00, 0x14,
    0x0A, 0x00, 0x00, 0x01, 0x99, 0x0B, 0x04, 0x04, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// LUTKW (10, Light Gray Target): Multi-phase GxEPD2_4G waveform (100 frames)
static const uint8_t lut_gray_LUTKW[42] PROGMEM = {
    0x40, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x90, 0x14, 0x14, 0x00, 0x00, 0x01, 0x00, 0x14,
    0x0A, 0x00, 0x00, 0x01, 0x99, 0x0C, 0x01, 0x03, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// LUTKK (00, Black Target): Pure NOP to prevent flashing black background
static const uint8_t lut_gray_LUTKK[42] PROGMEM = {
    0x00, T_MAX, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
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
      frameBuffer(nullptr)
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
      ,
      frameBufferPrev(nullptr)
#endif
{
  if (Serial)
    Serial.printf(
        "[%lu] EInkDisplay_UC8179: "
        "SCLK=%d MOSI=%d CS=%d DC=%d RST=%d BUSY=%d\n",
        millis(), sclk, mosi, cs, dc, rst, busy);
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
  _initDisplay();  // registers only, no PON/DRF — mirrors GxEPD2 _InitDisplay()

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
// The renderer generates LSB and MSB masks sequentially and clears its own
// frame buffer in between. We must intercept the masks, cache the LSB pass,
// and then precisely composite them with the OLD base B/W buffer.
// We write exact 4-shade absolute pixel states to DTM1 (LSB) and DTM2 (MSB).
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

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  uint8_t* base = frameBufferPrev;  // Contains the actual visible B/W page
#else
  uint8_t* base = frameBuffer;  // Fallback if no dual buffer
#endif

  uint8_t* target = (uint8_t*)malloc(BUFFER_SIZE);

  if (target) {
    // Generate and send DTM1 (Target LSB)
    // Target_LSB = (~base & ~msb) | (msb & ~lsb)
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
      uint8_t not_b = ~base[i];
      uint8_t not_m = ~msb[i];
      target[i] = (not_b & not_m) | (msb[i] & ~cached_lsb_mask[i]);
    }
    writeDTM1(target, BUFFER_SIZE);

    // Generate and send DTM2 (Target MSB)
    // Target_MSB = (~base & ~msb) | (msb & lsb)
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
      uint8_t not_b = ~base[i];
      uint8_t not_m = ~msb[i];
      target[i] = (not_b & not_m) | (msb[i] & cached_lsb_mask[i]);
    }
    writeDTM2(target, BUFFER_SIZE);

    free(target);
  } else {
    if (Serial) Serial.println("malloc failed for target mask composition");
  }

  free(cached_lsb_mask);
  cached_lsb_mask = nullptr;
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
    // Fast full refresh — single flicker, drives all pixels.
    // CRITICAL: init BEFORE writing DTM. Writing PSR in _initDisplay()
    // resets the UC8179 RAM address counter. DTM must be written AFTER
    // init (while powered on) so the address pointer is valid for DRF.
    _initFull();                          // POF → registers → PSR=OTP → PON
    writeDTM1(frameBuffer, BUFFER_SIZE);  // write RAM while powered
    writeDTM2(frameBuffer, BUFFER_SIZE);  // same data → all pixels driven
    _updateFull();                        // CCSET + TSSET + DRF
    // Re-init partial mode so the next FAST_REFRESH works immediately.
    _initPart();
  } else if (mode == FAST_REFRESH) {
    // No-flicker OTP partial refresh with forced temperature (110°C).
    // Uses the OTP waveform — no register LUTs loaded.
    if (!_using_partial_mode) _initPart();

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    writeDTM1(frameBufferPrev, BUFFER_SIZE);  // old frame
#endif
    writeDTM2(frameBuffer, BUFFER_SIZE);  // new frame
    _updatePart();

    // N2OCP=1 in CDI means the controller automatically copies DTM2→DTM1
    // after every refresh, so the "old" frame is always up to date.
    // We still swap the CPU-side buffers in dual-buffer mode so frameBufferPrev
    // reflects the frame just displayed, but no DTM1 re-write is needed.
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    swapBuffers();
#endif
    // Single-buffer mode: N2OCP handles DTM1 sync in hardware — no writeDTM1 needed.
  } else {
    // FULL_REFRESH — slow OTP waveform, multi-flicker.
    // Rarely needed; use HALF_REFRESH for normal full-screen updates.
    // Init first, write RAM after (address counter reset by PSR write).
    _initFull();
    writeDTM1(frameBuffer, BUFFER_SIZE);
    writeDTM2(frameBuffer, BUFFER_SIZE);
    _updateFullSlow();
    _initPart();
  }

  if (turnOffScreen) _powerOff();
}

// ============================================================================
// displayGrayBuffer — DTM1/DTM2 already loaded by caller
// ============================================================================
void EInkDisplay_UC8179::displayGrayBuffer(bool turnOffScreen) {
  if (Serial) Serial.printf("[%lu] displayGrayBuffer\n", millis());
  inGrayscaleMode = true;
  _initGray();
  _updatePart();
  if (turnOffScreen) _powerOff();
}

// ============================================================================
// grayscaleRevert — clean up mode state without triggering redundant flashes
// ============================================================================
void EInkDisplay_UC8179::grayscaleRevert() {
  if (Serial) Serial.printf("[%lu] grayscaleRevert\n", millis());

  // We no longer trigger a forced hardware flash here.
  // Doing so combined with the next displayBuffer(FAST) call caused
  // redundant 600ms pauses that made page turns feel like full refreshes.
  //
  // Because `displayBuffer` correctly overwrites DTM1 with `frameBufferPrev`
  // and DTM2 with the new `frameBuffer` right after this function returns,
  // the custom 40-frame anti-ghosting partial LUTs will naturally and
  // cleanly pull the physical gray particles into the correct black/white
  // state during the normal page turn refresh automatically.
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
// refreshDisplay — public shim so existing HAL call sites still compile
// ============================================================================
void EInkDisplay_UC8179::refreshDisplay(RefreshMode mode, bool turnOffScreen) { displayBuffer(mode, turnOffScreen); }

// ============================================================================
// _initDisplay — mirrors GxEPD2 _InitDisplay() exactly
// Registers only. No PON, no DRF.
// ============================================================================
void EInkDisplay_UC8179::_initDisplay() {
  if (Serial) Serial.printf("[%lu] _initDisplay\n", millis());

  // Exact GxEPD2 _InitDisplay() order — PSR first, then PWR, then BTST.
  sendCommand(CMD_PSR);
  sendData(PSR_OTP);  // 0x1f — KW OTP mode

  sendCommand(CMD_PWR);
  sendData(0x07);
  sendData(0x07);
  sendData(0x3f);
  sendData(0x3f);
  sendData(0x09);

  // BTST — GxEPD2 uses 0x17, 0x17, 0x28, 0x17 exactly.
  // 0x28 = strength 5, 0.27us min-off for phase C (VDHR, red channel — unused in KW mode).
  // Restore GxEPD2 defaults; our earlier "fix" to 0x17 was wrong — the working
  // GxEPD2 code uses 0x28 for PHC1 and it works fine on this hardware.
  sendCommand(CMD_BTST);
  sendData(0x17);
  sendData(0x17);
  sendData(0x28);
  sendData(0x17);

  sendCommand(CMD_TRES);
  sendData(0x03);
  sendData(0x20);  // 800
  sendData(0x01);
  sendData(0xE0);  // 480

  sendCommand(CMD_DUSPI);
  sendData(0x00);

  sendCommand(CMD_CDI);
  sendData(CDI0_LUTKW);
  sendData(CDI1);  // 0x29, 0x07

  sendCommand(CMD_TCON);
  sendData(0x22);
  sendCommand(CMD_PWS);
  sendData(0x22);
}

// ============================================================================
// _initFull — mirrors GxEPD2 _Init_Full() exactly
// _InitDisplay() + PSR=0x1f (redundant, already set) + PON
// ============================================================================
void EInkDisplay_UC8179::_initFull() {
  if (Serial) Serial.printf("[%lu] _initFull\n", millis());
  _powerOff();  // ensure clean state, resets _power_is_on
  _initDisplay();
  sendCommand(CMD_PSR);
  sendData(PSR_OTP);  // 0x1f — GxEPD2 sends this again after _InitDisplay
  _powerOn();
  _using_partial_mode = false;
}

// ============================================================================
// _initPart — Partial refresh using actual Register LUTs
//
// The forced 110C OTP partial refresh (0x6E) caused extreme ghosting on this
// hardware. The proper GDEY075T7 fix is to load the Partial Register LUTs and
// drive the display manually.
// ============================================================================
void EInkDisplay_UC8179::_initPart() {
  if (Serial) Serial.printf("[%lu] _initPart\n", millis());
  _initDisplay();

  // PSR = 0x3f: register LUTs, KW mode
  sendCommand(CMD_PSR);
  sendData(PSR_REG);

  // CDI for partial: LUTBD border, N2OCP=1 (Auto-copy DTM2->DTM1 after update)
  sendCommand(CMD_CDI);
  sendData(CDI0_LUTBD);
  sendData(CDI1);

  // Load B/W Partial Register LUTs
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
// _initGray — like _initPart but with gray LUTs
// ============================================================================
void EInkDisplay_UC8179::_initGray() {
  if (Serial) Serial.printf("[%lu] _initGray\n", millis());
  _initDisplay();

  // PSR = 0x3f: register LUTs, KWR mode (needed for 4-gray differential)
  sendCommand(CMD_PSR);
  sendData(PSR_REG);

  sendCommand(CMD_CDI);
  sendData(0x31);  // DDX=00, VBD=LUTBD, N2OCP=0, CDI=001
  sendData(CDI1);

  // Load custom 4-gray absolute mapping LUTs
  sendLutRegister(CMD_LUTC, lut_gray_LUTC);
  sendLutRegister(CMD_LUTWW, lut_gray_LUTWW);
  sendLutRegister(CMD_LUTKW, lut_gray_LUTKW);
  sendLutRegister(CMD_LUTWK, lut_gray_LUTWK);
  sendLutRegister(CMD_LUTKK, lut_gray_LUTKK);
  sendLutRegister(CMD_LUTBD, lut_partial_LUTBD);

  _powerOn();
  // CRITICAL: We changed registers (PSR, CDI, LUTs). We are NO LONGER in the standard
  // _initPart state. We must clear this flag so the next B/W update forces a re-init.
  _using_partial_mode = false;
}

// ============================================================================
// _updateFull — mirrors GxEPD2 _Update_Full()
// CCSET + TSSET (temp sensor) + DRF. No PON — already on from _initFull().
// ============================================================================
// Fast full refresh — TSFIX forces 90° which gives the single-flicker waveform.
// Matches GxEPD2 _Update_Full() with useFastFullUpdate=true.
void EInkDisplay_UC8179::_updateFull() {
  if (Serial) Serial.printf("[%lu] _updateFull fast (DRF)...\n", millis());
  sendCommand(CMD_CCSET);
  sendData(0x02);  // TSFIX = use forced temperature
  sendCommand(CMD_TSSET);
  sendData(0x5A);  // 90°C — activates fast waveform
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF full");
}

// Slow full refresh — internal temperature sensor, multi-flicker OTP waveform.
// Only used by FULL_REFRESH mode. Matches GxEPD2 useFastFullUpdate=false path.
void EInkDisplay_UC8179::_updateFullSlow() {
  if (Serial) Serial.printf("[%lu] _updateFull slow (DRF)...\n", millis());
  sendCommand(CMD_CCSET);
  sendData(0x00);  // no TSFIX
  sendCommand(CMD_TSE);
  sendData(0x00);  // internal temperature sensor
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF full slow");
}

// ============================================================================
// _updatePart — mirrors GxEPD2 _Update_Part()
// DRF only. No PON — already on from _initPart().
// ============================================================================
void EInkDisplay_UC8179::_updatePart() {
  if (Serial) Serial.printf("[%lu] _updatePart (DRF)...\n", millis());
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF part");
}

// ============================================================================
// _powerOn / _powerOff — mirrors GxEPD2 _PowerOn / _PowerOff
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
// hardwareReset — matches GxEPD2 _reset() timing exactly
// ============================================================================
void EInkDisplay_UC8179::hardwareReset() {
  if (Serial) Serial.printf("[%lu] hardwareReset\n", millis());
  digitalWrite(_rst, HIGH);
  delay(20);
  digitalWrite(_rst, LOW);
  delay(10);  // GxEPD2 default reset_duration=10
  digitalWrite(_rst, HIGH);
  delay(200);  // GxEPD2 waits 200ms after reset
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
// waitWhileBusy
// busy_level = LOW (pin LOW while busy, HIGH when ready) — from GxEPD2 constructor
// ============================================================================
void EInkDisplay_UC8179::waitWhileBusy(const char* tag) {
  delay(1);  // margin for controller to assert BUSY
  unsigned long t0 = millis();
  while (digitalRead(_busy) == LOW) {  // LOW = busy
    delay(1);
    if (millis() - t0 > 10000) {
      if (Serial) Serial.printf("[%lu] TIMEOUT: %s\n", millis(), tag ? tag : "");
      break;
    }
  }
  if (tag && Serial) Serial.printf("[%lu] done: %s (%lu ms)\n", millis(), tag, millis() - t0);
}

// ============================================================================
// RAM helpers — partial window addressing (mirrors GxEPD2 exactly)
// GxEPD2 wraps every DTM write with CMD_PART_IN / _PART_WINDOW / _PART_OUT.
// Without the window commands the controller doesn't know which region to refresh.
// ============================================================================
void EInkDisplay_UC8179::_setPartialWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint16_t xe = (x + w - 1) | 0x0007;  // byte boundary inclusive
  uint16_t ye = y + h - 1;
  x &= 0xFFF8;  // byte boundary
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
// saveFrameBufferAsPBM
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
// diagnose() — call after begin(), prints to Serial
// ============================================================================
void EInkDisplay_UC8179::diagnose() {
  Serial.println("=== EInkDisplay_UC8179::diagnose() ===");
  int busyNow = digitalRead(_busy);
  Serial.printf("  BUSY at idle: %d  (expected 1 = HIGH = ready)\n", busyNow);

  // Send PON and watch BUSY
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
    Serial.printf("  BUSY went LOW %lu ms after PON — controller responding\n", millis() - t0);
    // wait for it to go HIGH again
    t0 = millis();
    while (digitalRead(_busy) == LOW && millis() - t0 < 5000) delay(1);
    Serial.printf("  BUSY HIGH again after %lu ms total\n", millis() - t0);
  } else {
    Serial.println("  BUSY never went LOW after PON — analog section not starting");
    Serial.println("  Check: PWR register, BTST register, VCC supply to panel");
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