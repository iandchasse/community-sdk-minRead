#ifndef EINK_DISPLAY_UC8179_H
#define EINK_DISPLAY_UC8179_H

#include "EInkDisplay.h"

// PSR constants for GDEY075T7 (800x480)
// Bits: [7:6]=11 (800x480), [5]=REG, [4]=KWR, [3]=UD, [2]=SHL, [1]=SHD_N, [0]=RST_N
// Default PSR_OTP is 0x1F: 800x480, OTP, KW mode, Scan up, Shift right, Booster ON, No reset
// Default PSR_REG is 0x3F: 800x480, REG, KW mode, Scan up, Shift right, Booster ON, No reset
static const uint8_t PSR_OTP = 0x1F;
static const uint8_t PSR_REG = 0x3F;

// CDI constants
static const uint8_t CDI0_LUTKW = 0x29;  // border LUTKW, copy N2O (differential)
static const uint8_t CDI0_LUTBD = 0x39;  // border LUTBD, copy N2O
static const uint8_t CDI0_GRAY = 0x00;   // no copy for grayscale
static const uint8_t CDI1 = 0x07;        // 10 hsync interval

class EInkDisplay_UC8179 : public EInkDisplay {
 public:
  EInkDisplay_UC8179(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy);

  void begin() override;
  void refreshDisplay(RefreshMode mode, bool turnOffScreen = false) override;
  void clearScreen(uint8_t color = 0xFF) override;
  void deepSleep() override;

  // These must align with the naming expected by HalDisplay and EInkDisplay base
  void powerOnDisplay();
  void powerOffDisplay();

  // Required by EInkDisplay common interface but specific to UC8179
  void copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer) override;
  void copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer) override;
  void copyGrayscaleMsbBuffers(const uint8_t* msbBuffer) override;
  void cleanupGrayscaleBuffers(const uint8_t* bwBuffer) override;
  void displayGrayBuffer(bool turnOffScreen = false) override;

 protected:
  void initDisplayController() override;
  void resetDisplay() override;
  void waitWhileBusy(const char* comment = nullptr) override;

  void sendCommand(uint8_t cmd) override;
  void sendData(uint8_t data) override;
  void sendData(const uint8_t* data, uint32_t length) override;

 private:
  void writeDTM1(const uint8_t* data, uint32_t size);
  void writeDTM2(const uint8_t* data, uint32_t size);
  void sendLutRegister(uint8_t cmd, const uint8_t* lut);

  void loadFastBwLUT();
  void loadHalfRefreshLUT();
  void loadGrayscaleLUT();
  void loadGrayscaleRevertLUT();

  bool isPoweredOn = false;
  bool customLutActive = false;
};

#endif
