#pragma once
#include <Arduino.h>
#include <SPI.h>

// ============================================================================
// EInkDisplay_UC8179
// Driver for GDEY075T7 (800x480, UC8179 controller).
//
// Sequence mirrors GxEPD2_750_GDEY075T7 exactly:
//   _initDisplay()  — registers only (PSR/PWR/BTST/TRES/CDI/TCON/PWS), no PON
//   _initFull()     — _initDisplay + PSR=0x1f (OTP) + _powerOn()
//   _initPart()     — _initDisplay + PSR=0x3f + LUTs + _powerOn()
//   _updateFull()   — CCSET/TSE + DRF  (no PON — already on from _initFull)
//   _updatePart()   — DRF only         (no PON — already on from _initPart)
//
//   BUSY polarity: LOW = busy, HIGH = ready  (matches GxEPD2 busy_level=LOW)
//
// Refresh modes:
//   FAST_REFRESH  — no-flicker differential waveform (register LUTs)
//   HALF_REFRESH  — same waveform, both buffers identical (drives all pixels)
//   FULL_REFRESH  — OTP waveform, then switches to partial mode
//
// Grayscale flow:
//   copyGrayscaleLsbBuffers(lsb) / copyGrayscaleMsbBuffers(msb)
//   displayGrayBuffer()
//   next displayBuffer() call auto-runs grayscaleRevert()
//
// Compile flags:
//   EINK_DISPLAY_SINGLE_BUFFER_MODE  — 48 KB RAM instead of 96 KB
// ============================================================================

class EInkDisplay_UC8179 {
 public:
  enum RefreshMode {
    FULL_REFRESH,
    HALF_REFRESH,
    FAST_REFRESH,
  };

  static constexpr uint16_t DISPLAY_WIDTH = 800;
  static constexpr uint16_t DISPLAY_HEIGHT = 480;
  static constexpr uint16_t DISPLAY_WIDTH_BYTES = DISPLAY_WIDTH / 8;
  static constexpr uint32_t BUFFER_SIZE = (uint32_t)DISPLAY_WIDTH_BYTES * DISPLAY_HEIGHT;  // 48000

  EInkDisplay_UC8179(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy);

  void begin();

  // Frame-buffer operations (CPU-side only, no SPI)
  void clearScreen(uint8_t color = 0xFF) const;
  void drawImage(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                 bool fromProgmem = false) const;
  void drawImageTransparent(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            bool fromProgmem = false) const;
  void setFramebuffer(const uint8_t* bwBuffer) const;

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  void swapBuffers();
#endif

  // Grayscale buffer helpers
  void copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer);
  void copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer);
  void copyGrayscaleMsbBuffers(const uint8_t* msbBuffer);

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  void cleanupGrayscaleBuffers(const uint8_t* bwBuffer);
#endif

  // Display update
  void displayBuffer(RefreshMode mode = FAST_REFRESH, bool turnOffScreen = false);
  void displayGrayBuffer(bool turnOffScreen = false);
  void grayscaleRevert();
  void refreshDisplay(RefreshMode mode = FAST_REFRESH, bool turnOffScreen = false);
  void setCustomLUT(bool enabled, const unsigned char* lutData = nullptr);

  // Power management
  void deepSleep();

  // Buffer access
  uint8_t* getFrameBuffer() const { return frameBuffer; }

  // Desktop/test only
  void saveFrameBufferAsPBM(const char* filename);

  // Diagnostics
  void diagnose();

 private:
  int8_t _sclk, _mosi, _cs, _dc, _rst, _busy;

  uint8_t frameBuffer0[BUFFER_SIZE];
  uint8_t* frameBuffer;

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  uint8_t frameBuffer1[BUFFER_SIZE];
  uint8_t* frameBufferPrev;
#endif

  SPISettings spiSettings;

  bool _power_is_on = false;
  bool _using_partial_mode = false;
  bool inGrayscaleMode = false;

  // Init/update — mirrors GxEPD2 structure
  void hardwareReset();
  void _initDisplay();
  void _initFull();
  void _initPart();
  void _initGray();
  void _updateFull();      // fast (TSFIX=90)
  void _updateFullSlow();  // slow (internal temp sensor)
  void _updatePart();
  void _powerOn();
  void _powerOff();
  void writeDTM1_antiGhost(const uint8_t* old_bw, const uint8_t* new_bw, const uint8_t* gray_mask_map, uint32_t size);

  // SPI
  void sendCommand(uint8_t cmd);
  void sendData(uint8_t data);
  void sendData(const uint8_t* data, uint32_t length);
  void waitWhileBusy(const char* tag = nullptr);

  // RAM write
  void writeDTM1(const uint8_t* data, uint32_t size);
  void writeDTM2(const uint8_t* data, uint32_t size);
  void _setPartialWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

  // LUT
  void sendLutRegister(uint8_t cmd, const uint8_t* lutProgmem);

  // Diagnostics helper
  uint8_t sendCommandReadByte(uint8_t cmd);

  uint8_t* cached_lsb_mask = nullptr;
};