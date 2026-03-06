#include "EInkDisplay_UC8179.h"

// ============================================================================
// LUT Definitions (GDEY075T7 / UC8179)
// ============================================================================

// Fast B/W LUT (Single flicker, 2-phase)
// Based on GxEPD2_750_T7 experimental timing: T1=30, T2=5, T3=30, T4=5
// Structure: [byte-groups of 6] x 7 = 42 bytes
const uint8_t lut_fastbw_LUTC[] = {0x00, 30, 5, 30, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0,    0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_fastbw_LUTWW[] = {0x00, 30, 5, 30, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0,    0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_fastbw_LUTKW[] = {0x5A, 30, 5, 30, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0,    0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_fastbw_LUTWK[] = {0x84, 30, 5, 30, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0,    0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_fastbw_LUTKK[] = {0x00, 30, 5, 30, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0,    0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_fastbw_LUTBD[] = {0x00, 30, 5, 30, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0,    0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Half/Full Refresh LUT (Register-based, slightly longer T)
const uint8_t lut_half_LUTC[] = {0x00, 50, 10, 50, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0,    0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_half_LUTWW[] = {0x00, 50, 10, 50, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_half_LUTKW[] = {0x5A, 50, 10, 50, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_half_LUTWK[] = {0x84, 50, 10, 50, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_half_LUTKK[] = {0x00, 50, 10, 50, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_half_LUTBD[] = {0x00, 50, 10, 50, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Grayscale LUT (4G-validated)
const uint8_t lut_gray_LUTC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x05, 0x1E, 0x05, 0x01, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_gray_LUTWW[] = {0x00, 0x1E, 0x05, 0x1E, 0x05, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,    0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_gray_LUTKW[] = {0x12, 0x1E, 0x05, 0x1E, 0x05, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,    0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_gray_LUTWK[] = {0x21, 0x1E, 0x05, 0x1E, 0x05, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,    0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_gray_LUTKK[] = {0x00, 0x1E, 0x05, 0x1E, 0x05, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,    0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint8_t lut_gray_LUTBD[] = {0x00, 0x1E, 0x05, 0x1E, 0x05, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0,    0,    0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const uint8_t lut_grayscale_revert[] = {0x00, 0x28, 0x28, 0x01, 0x01, 0x02, 0x48, 0x32, 0x14, 0x01, 0x01,
                                        0x01, 0x84, 0x32, 0x14, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ============================================================================
// UC8179 Command Definitions
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
#define CMD_VDCS 0x82
#define CMD_TCON 0x60
#define CMD_TRES 0x61
#define CMD_CDI 0x50
#define CMD_TSE 0x41
#define CMD_CCSET 0xE0
#define CMD_PWS 0xE3

EInkDisplay_UC8179::EInkDisplay_UC8179(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy)
    : EInkDisplay(sclk, mosi, cs, dc, rst, busy) {
  DISPLAY_WIDTH = 800;
  DISPLAY_HEIGHT = 480;
  BUFFER_SIZE = (uint32_t)DISPLAY_WIDTH * DISPLAY_HEIGHT / 8;
}

void EInkDisplay_UC8179::begin() {
  EInkDisplay::begin();  // Inits GPIOs and SPI
  initDisplayController();
}

void EInkDisplay_UC8179::initDisplayController() {
  if (Serial) Serial.printf("[%lu] initDisplayController\n", millis());

  resetDisplay();

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
  sendData(DISPLAY_WIDTH / 256);
  sendData(DISPLAY_WIDTH % 256);
  sendData(DISPLAY_HEIGHT / 256);
  sendData(DISPLAY_HEIGHT % 256);

  sendCommand(CMD_DUSPI);
  sendData(0x00);

  sendCommand(CMD_CDI);
  sendData(CDI0_LUTKW);
  sendData(CDI1);

  sendCommand(CMD_TCON);
  sendData(0x22);
  sendCommand(CMD_PWS);
  sendData(0x22);

  if (Serial) Serial.printf("[%lu] clearing RAM buffers\n", millis());
  writeDTM1(frameBuffer, BUFFER_SIZE);
  writeDTM2(frameBuffer, BUFFER_SIZE);

  loadFastBwLUT();
  customLutActive = true;
  if (Serial) Serial.printf("[%lu] initDisplayController complete\n", millis());
}

void EInkDisplay_UC8179::resetDisplay() {
  if (Serial) Serial.printf("[%lu] resetDisplay\n", millis());
  digitalWrite(_rst, HIGH);
  delay(20);
  digitalWrite(_rst, LOW);
  delay(20);
  digitalWrite(_rst, HIGH);
  delay(20);
  waitWhileBusy("reset");
}

void EInkDisplay_UC8179::waitWhileBusy(const char* comment) {
  delay(1);
  unsigned long start = millis();
  while (1) {
    if (digitalRead(_busy) != HIGH) break;
    delay(1);
    if (digitalRead(_busy) != HIGH) break;
    if (millis() - start > 10000) {
      if (Serial) Serial.printf("[%lu] TIMEOUT waiting for BUSY_N (%s)\n", millis(), comment ? comment : "");
      break;
    }
  }
  if (comment && Serial) Serial.printf("[%lu] busy done: %s (%lu ms)\n", millis(), comment, millis() - start);
}

void EInkDisplay_UC8179::powerOnDisplay() {
  if (isPoweredOn) return;
  if (Serial) Serial.printf("[%lu] power ON (waking from deep sleep)\n", millis());
  initDisplayController();
  isPoweredOn = true;
}

void EInkDisplay_UC8179::powerOffDisplay() {
  sendCommand(CMD_POF);
  waitWhileBusy("POF");
  isPoweredOn = false;
  if (Serial) Serial.printf("[%lu] power OFF\n", millis());
}

void EInkDisplay_UC8179::refreshDisplay(RefreshMode mode, bool turnOffScreen) {
  powerOnDisplay();

  switch (mode) {
    case FULL_REFRESH:
      sendCommand(CMD_PSR);
      sendData(PSR_OTP);
      customLutActive = false;
      break;
    case HALF_REFRESH:
      loadHalfRefreshLUT();
      break;
    case FAST_REFRESH:
      if (!customLutActive) loadFastBwLUT();
      break;
  }

  sendCommand(CMD_PON);
  waitWhileBusy("PON");

  if (Serial) Serial.printf("[%lu] DRF (%s)...\n", millis(), mode == FAST_REFRESH ? "FAST" : "FULL");
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF");

  if (turnOffScreen) powerOffDisplay();
}

void EInkDisplay_UC8179::clearScreen(uint8_t color) {
  memset(frameBuffer, color, BUFFER_SIZE);
  writeDTM1(frameBuffer, BUFFER_SIZE);
  writeDTM2(frameBuffer, BUFFER_SIZE);
  refreshDisplay(HALF_REFRESH);
}

void EInkDisplay_UC8179::deepSleep() {
  powerOffDisplay();
  sendCommand(CMD_DSLP);
  sendData(0xA5);
  if (Serial) Serial.printf("[%lu] Deep Sleep\n", millis());
}

void EInkDisplay_UC8179::writeDTM1(const uint8_t* data, uint32_t size) {
  sendCommand(CMD_DTM1);
  sendData(data, size);
  sendCommand(CMD_DSP);
}

void EInkDisplay_UC8179::writeDTM2(const uint8_t* data, uint32_t size) {
  sendCommand(CMD_DTM2);
  sendData(data, size);
  sendCommand(CMD_DSP);
}

void EInkDisplay_UC8179::sendLutRegister(uint8_t cmd, const uint8_t* lut) {
  sendCommand(cmd);
  sendData(lut, 42);
}

void EInkDisplay_UC8179::loadFastBwLUT() {
  if (Serial) Serial.printf("[%lu] loadFastBwLUT\n", millis());
  sendCommand(CMD_PSR);
  sendData(PSR_REG);
  sendCommand(CMD_VDCS);
  sendData(0x30);
  sendCommand(CMD_CDI);
  sendData(CDI0_LUTBD);
  sendData(CDI1);
  sendLutRegister(CMD_LUTC, lut_fastbw_LUTC);
  sendLutRegister(CMD_LUTWW, lut_fastbw_LUTWW);
  sendLutRegister(CMD_LUTKW, lut_fastbw_LUTKW);
  sendLutRegister(CMD_LUTWK, lut_fastbw_LUTWK);
  sendLutRegister(CMD_LUTKK, lut_fastbw_LUTKK);
  sendLutRegister(CMD_LUTBD, lut_fastbw_LUTBD);
  customLutActive = true;
}

void EInkDisplay_UC8179::loadHalfRefreshLUT() {
  if (Serial) Serial.printf("[%lu] loadHalfRefreshLUT\n", millis());
  sendCommand(CMD_PSR);
  sendData(PSR_REG);
  sendCommand(CMD_VDCS);
  sendData(0x30);
  sendCommand(CMD_CDI);
  sendData(CDI0_LUTBD);
  sendData(CDI1);
  sendLutRegister(CMD_LUTC, lut_half_LUTC);
  sendLutRegister(CMD_LUTWW, lut_half_LUTWW);
  sendLutRegister(CMD_LUTKW, lut_half_LUTKW);
  sendLutRegister(CMD_LUTWK, lut_half_LUTWK);
  sendLutRegister(CMD_LUTKK, lut_half_LUTKK);
  sendLutRegister(CMD_LUTBD, lut_half_LUTBD);
  customLutActive = true;
}

void EInkDisplay_UC8179::loadGrayscaleLUT() {
  sendCommand(CMD_PSR);
  sendData(PSR_REG);
  sendCommand(CMD_VDCS);
  sendData(0x12);
  sendCommand(CMD_CDI);
  sendData(CDI0_GRAY);
  sendData(CDI1);
  sendCommand(CMD_CCSET);
  sendData(0x00);
  sendLutRegister(CMD_LUTC, lut_gray_LUTC);
  sendLutRegister(CMD_LUTWW, lut_gray_LUTWW);
  sendLutRegister(CMD_LUTKW, lut_gray_LUTKW);
  sendLutRegister(CMD_LUTWK, lut_gray_LUTWK);
  sendLutRegister(CMD_LUTKK, lut_gray_LUTKK);
  sendLutRegister(CMD_LUTBD, lut_gray_LUTBD);
  customLutActive = true;
}

void EInkDisplay_UC8179::loadGrayscaleRevertLUT() {
  sendCommand(CMD_VDCS);
  sendData(0x30);
  sendCommand(CMD_CDI);
  sendData(0x17);
  sendData(CDI1);
  sendLutRegister(CMD_LUTC, lut_grayscale_revert);
  sendLutRegister(CMD_LUTWW, lut_grayscale_revert);
  sendLutRegister(CMD_LUTKW, lut_grayscale_revert);
  sendLutRegister(CMD_LUTWK, lut_grayscale_revert);
  sendLutRegister(CMD_LUTKK, lut_grayscale_revert);
  sendLutRegister(CMD_LUTBD, lut_grayscale_revert);
}

void EInkDisplay_UC8179::displayGrayBuffer(bool turnOffScreen) {
  powerOnDisplay();
  loadGrayscaleLUT();
  sendCommand(CMD_PON);
  waitWhileBusy("PON");
  sendCommand(CMD_DRF);
  waitWhileBusy("DRF-GRAY");
  if (turnOffScreen) powerOffDisplay();
  inGrayscaleMode = true;
}

void EInkDisplay_UC8179::copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer) {
  writeDTM1(lsbBuffer, BUFFER_SIZE);
  writeDTM2(msbBuffer, BUFFER_SIZE);
}
void EInkDisplay_UC8179::copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer) { writeDTM1(lsbBuffer, BUFFER_SIZE); }
void EInkDisplay_UC8179::copyGrayscaleMsbBuffers(const uint8_t* msbBuffer) { writeDTM2(msbBuffer, BUFFER_SIZE); }
void EInkDisplay_UC8179::cleanupGrayscaleBuffers(const uint8_t* bwBuffer) { writeDTM2(bwBuffer, BUFFER_SIZE); }

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
