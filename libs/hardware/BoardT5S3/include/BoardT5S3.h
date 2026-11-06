#pragma once

#include <Arduino.h>

#include "BoardT5S3Pins.h"

namespace BoardT5S3 {

class ScopedI2CLock {
 public:
  ScopedI2CLock();
  ~ScopedI2CLock();

  ScopedI2CLock(const ScopedI2CLock&) = delete;
  ScopedI2CLock& operator=(const ScopedI2CLock&) = delete;

 private:
  bool locked_ = false;
};

void begin();
void beginI2C();
void prepareSdBus();
void disableGpsLora();
bool pca9535Present();
bool readPca9535Pin(uint8_t pin, bool* high);
bool writePca9535Pin(uint8_t pin, bool high);
bool setPca9535PinMode(uint8_t pin, uint8_t mode);
bool readButton();

// T-253 bench instruments, defined in LilyGoT5S3LgfxConfig.cpp. Reached from
// the firmware through CMD:VCOM / CMD:TPS, which are devel-only (platformio.ini
// gates them on ENABLE_EPDLUT_CMD, the same flag that already guards the other
// instrument that degrades the image by design).
//
// Why they exist: VCOM is an assigned per-panel value with a +/-0.1 V window
// and the factory does not calibrate per unit, so our one hardcoded constant is
// a guess for every panel it drives (docs/panel-ed047tc1.md). Two T5 S3 Pro
// boards on identical firmware, identical settings and identical power-up
// timings render the same full-black frame as a deep black and a pale grey, and
// sweeping VCOM is the test that says whether that is the reason.
//
// The new value lands on the next epdPowerOn(), which runs before every
// refresh, so a sweep needs no reflash and no reboot.
void setEpdVcomMv(int mv);
int getEpdVcomMv();
int getEpdDefaultVcomMv();
bool readTpsRegister8Public(uint8_t reg, uint8_t* value);
// Arms a full register dump on the next epdPowerOn(). Deferred, not immediate,
// because the PMIC only answers I2C while the power sequence holds it awake --
// a read outside a refresh fails with ESP_ERR_INVALID_STATE.
void armTpsDump();
void dumpTpsRegisters();

}  // namespace BoardT5S3
