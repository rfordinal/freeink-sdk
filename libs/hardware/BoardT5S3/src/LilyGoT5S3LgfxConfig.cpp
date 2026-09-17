#include <BoardT5S3.h>
#include <LgfxEpdConfig.h>
#include <Wire.h>

namespace {

constexpr int kDefaultVcomMv = -1600;
constexpr uint8_t kTpsRegEnable = 0x01;
constexpr uint8_t kTpsRegVcom = 0x03;
constexpr uint8_t kTpsRegPowerGood = 0x0F;
constexpr uint8_t kTpsEnableOutputs = 0x3F;

#define LUT_MAKE(d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, da, db, dc, dd, de, df) \
  (uint32_t)((d0 << 0) | (d1 << 2) | (d2 << 4) | (d3 << 6) | (d4 << 8) | (d5 << 10) | (d6 << 12) | \
             (d7 << 14) | (d8 << 16) | (d9 << 18) | (da << 20) | (db << 22) | (dc << 24) | \
             (dd << 26) | (de << 28) | (df << 30))

// Single waveform for BOTH the B/W base push and the AA gray overlay push.
// Panel_EPD's per-pixel diff embeds the epd_mode LUT offset in the stored
// value, so alternating modes between the two pushes of a page turn defeats
// the diff and re-drives the whole screen — the LovyanGFX default lut_fast
// then flashes every white pixel black for two frames (the full-screen black
// "swipe"). Using one LUT under one mode keeps unchanged pixels skipped.
// Columns 0/15 carry the default lut_fast drive (changed B/W text pixels);
// columns 1-6 / 9-14 carry the AA nudge for the gray levels AA produces.
constexpr uint32_t kFastLut[] = {
    LUT_MAKE(2, 1, 1, 1, 1, 1, 1, 3, 3, 2, 2, 2, 2, 2, 2, 1),
    LUT_MAKE(2, 3, 1, 1, 1, 1, 3, 3, 3, 3, 2, 2, 2, 2, 3, 1),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    ~0u,
    0u,
};

// A 1-bit table for the marker-move path, and the reason it can exist at all:
// Panel_EPD's fast branch Bayer-thresholds every 8-bit value to 0 or 0xF
// (Panel_EPD.cpp, `readbuf[i] = (sum + (b << 4)) < 248 ? 0 : 0xF;`), so on this
// path **columns 1-14 are unreachable** -- only 0 and 15 are ever selected. The
// AA nudge columns kFastLut carries above are dead weight here, and so are the
// two opposite-direction passes it opens with: a pixel heading for white is
// driven black twice first, which is the flash a marker move produces.
//
// So: four passes of pure drive, no pre-drive, greys left inert. 4 drive + 1
// idle + the terminator and the trailing empty pass = **7 passes** against
// kFastLut's 11, at a measured 34 ms a pass (T-269).
//
// **Dose is the open question, not grey levels.** Fewer passes means less drive
// into the pixel, so black may land pale or leave residue. That is decided by a
// thumb on the panel, not here. EPD_Painter drives full black with 7 continuous
// passes on this same glass and calls its extremes "well into saturation", so
// four is a deliberate probe below that, not a safe default. Compare against
// kFastLut and the library's own lut_fastest before adopting it -- CMD:EPDLUT
// switches between the three at runtime.
constexpr uint32_t kFast1bitLut[] = {
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
    ~0u,
    0u,
};

// The clean-frame table, and the whole point of it is the tail.
//
// LovyanGFX's own lut_text is 12 drive rows followed by **19 idle rows** and a
// terminator; with the eraser prefix that is the 37 passes T-269 measured at
// 1,249 ms. An idle row drives nothing -- it is settle time -- but it costs a
// full pass, because blit_dmabuf still reads the row's slice of the step
// framebuffer and the bus still clocks every row. Measured: the 37-pass frame
// averaged 33.87 ms a pass against the 11-pass frame's 33.30, so idle passes
// are not cheaper. **Nineteen of them is ~646 ms of pure waiting, over half the
// clean frame.**
//
// That count is a time constant expressed in passes, and it was chosen for a
// panel whose passes are far quicker than ours. At 34 ms a pass we pay several
// times the settle the table's author intended.
//
// So: the 12 drive rows are LovyanGFX's, copied verbatim and checked against
// the library source mechanically rather than by eye, because a mistyped
// column would move a grey landing and look like a waveform problem. Only the
// idle tail is ours.
//
// **Why not just use epd_quality's slot instead.** Because the branch is
// picked by mode, and epd_text's is the only one that arms on
// `white != d1 || d1 != s0` -- every non-white pixel, changed or not. That
// weak diff is not a defect, it is what makes a clean frame clean: it
// re-drives all the ink, so residue goes everywhere rather than only where
// the image changed. epd_quality keeps the eraser but arms on `d1 != s0`, so
// routing cleans there would stop them cleaning.
#define LUT_MAKE(d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, da, db, dc, dd, de, df) \
  (uint32_t)((d0 << 0) | (d1 << 2) | (d2 << 4) | (d3 << 6) | (d4 << 8) | (d5 << 10) | (d6 << 12) | \
             (d7 << 14) | (d8 << 16) | (d9 << 18) | (da << 20) | (db << 22) | (dc << 24) | \
             (dd << 26) | (de << 28) | (df << 30))

// Idle rows kept after the drive rows. **Defaults to LovyanGFX's own 19**, so
// this file on its own changes nothing; a board opts into a shorter tail from
// platformio.ini. Checked: at 19 the table predicts 37 passes and 1.25 s, which
// is the 1,249 ms T-269 measured, so the knob reproduces the stock number
// before it changes it.
//
// This is the knob T-273's clean half turns, and it is judged on the panel:
// what it buys is time and what it costs is settle, and an under-settled clean
// leaves the residue the clean existed to remove.
#ifndef EXPLORINK_TEXT_LUT_IDLE
#define EXPLORINK_TEXT_LUT_IDLE 19
#endif

constexpr uint32_t kTextLut[] = {
    // 12 drive rows, verbatim from LovyanGFX lut_text.
    LUT_MAKE(2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1),
    LUT_MAKE(2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1),
    LUT_MAKE(2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1),
    LUT_MAKE(2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 1),
    LUT_MAKE(2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 1),
    LUT_MAKE(1, 2, 2, 1, 1, 1, 1, 1, 3, 3, 1, 1, 3, 3, 1, 2),
    LUT_MAKE(1, 3, 3, 1, 1, 1, 1, 3, 3, 1, 1, 1, 1, 3, 1, 2),
    LUT_MAKE(1, 3, 3, 1, 2, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2),
    LUT_MAKE(3, 1, 3, 2, 2, 2, 1, 1, 1, 2, 2, 1, 1, 1, 2, 3),
    LUT_MAKE(1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2),
    LUT_MAKE(1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2),
    LUT_MAKE(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 2),
#if EXPLORINK_TEXT_LUT_IDLE >= 1
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 2
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 3
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 4
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 5
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 6
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 7
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 8
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 9
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 10
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 11
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 12
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 13
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 14
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 15
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 16
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 17
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 18
    ~0u,
#endif
#if EXPLORINK_TEXT_LUT_IDLE >= 19
    ~0u,
#endif
    0u,
};

#undef LUT_MAKE

bool writeTpsRegister(uint8_t reg, const uint8_t* data, size_t len) {
  BoardT5S3::ScopedI2CLock lock;
  Wire.beginTransmission(T5S3_TPS65185_ADDR);
  Wire.write(reg);
  if (data && len) Wire.write(data, len);
  return Wire.endTransmission() == 0;
}

bool writeTpsRegister8(uint8_t reg, uint8_t value) { return writeTpsRegister(reg, &value, 1); }

bool readTpsRegister(uint8_t reg, uint8_t* data, size_t len) {
  if (!data || !len) return false;
  BoardT5S3::ScopedI2CLock lock;
  Wire.beginTransmission(T5S3_TPS65185_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t want = static_cast<uint8_t>(len);
  if (Wire.requestFrom(static_cast<uint8_t>(T5S3_TPS65185_ADDR), want) != want) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; ++i) data[i] = Wire.read();
  return true;
}

bool waitForPcaPinHigh(uint8_t pin, uint32_t timeoutMs) {
  const uint32_t start = millis();
  bool high = false;
  while (millis() - start < timeoutMs) {
    if (BoardT5S3::readPca9535Pin(pin, &high) && high) return true;
    delay(1);
  }
  return false;
}

bool waitForTpsReady(uint32_t timeoutMs) {
  const uint32_t start = millis();
  uint8_t powerGood = 0;
  while (millis() - start < timeoutMs) {
    if (readTpsRegister(kTpsRegPowerGood, &powerGood, 1) && (powerGood & 0xFA) == 0xFA) return true;
    delay(1);
  }
  return false;
}

bool prepareEpdPower() {
  // Deselect the SX1262 before the EPD bus is built, and do it here rather than
  // anywhere else, because this hook is the last thing that runs before
  // Bus_EPD::init() (LgfxEpdDriver.cpp, FreeInkBusEPD::init()).
  //
  // pinPwr below is T5S3_LORA_CS (GPIO46) and has to stay a real GPIO: it is
  // handed to the i80 driver as dc_gpio_num, and the IDF rejects a negative one
  // (esp_lcd_panel_io_i80.c, lcd_i80_bus_configure_gpio). GPIO46 is also the
  // LoRa radio's NSS, on the same SPI bus as the SD card. Bus_EPD::init() then
  // makes it an output with lgfx::pinMode(), which does NOT write a level
  // (common.cpp, the gpio_hi() there is guarded to non-output modes), so the pin
  // keeps whatever the output register held -- 0 out of reset. That asserts the
  // radio's chip select for the whole run, and measured on hardware 2026-09-03
  // the SD card is unreadable whenever the radio is selected while its supply
  // rail is off.
  //
  // Writing the level here survives, for the same reason the bug exists: nothing
  // downstream sets it. Between the i80 bus setup and the pinMode that hands the
  // pad back to plain GPIO out, the peripheral drives DC on it for a few
  // microseconds, and that is the one window this does not cover.
  pinMode(T5S3_LORA_CS, OUTPUT);
  digitalWrite(T5S3_LORA_CS, HIGH);

  pinMode(EP_STV, OUTPUT);
  digitalWrite(EP_STV, LOW);

  bool ok = true;
  ok &= BoardT5S3::setPca9535PinMode(PCA9535_IO10_EP_OE, OUTPUT);
  ok &= BoardT5S3::setPca9535PinMode(PCA9535_IO11_EP_MODE, OUTPUT);
  ok &= BoardT5S3::setPca9535PinMode(PCA9535_IO13_TPS_PWRUP, OUTPUT);
  ok &= BoardT5S3::setPca9535PinMode(PCA9535_IO14_VCOM_CTRL, OUTPUT);
  ok &= BoardT5S3::setPca9535PinMode(PCA9535_IO15_TPS_WAKEUP, OUTPUT);
  ok &= BoardT5S3::setPca9535PinMode(PCA9535_IO16_TPS_PWR_GOOD, INPUT);
  ok &= BoardT5S3::setPca9535PinMode(PCA9535_IO17_TPS_INT, INPUT);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO10_EP_OE, false);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO11_EP_MODE, false);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO13_TPS_PWRUP, false);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO14_VCOM_CTRL, false);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO15_TPS_WAKEUP, false);
  return ok;
}

void epdPowerOff() {
  BoardT5S3::writePca9535Pin(PCA9535_IO10_EP_OE, false);
  BoardT5S3::writePca9535Pin(PCA9535_IO11_EP_MODE, false);
  BoardT5S3::writePca9535Pin(PCA9535_IO13_TPS_PWRUP, false);
  BoardT5S3::writePca9535Pin(PCA9535_IO14_VCOM_CTRL, false);
  delay(1);
  BoardT5S3::writePca9535Pin(PCA9535_IO15_TPS_WAKEUP, false);
  digitalWrite(EP_STV, LOW);
}

bool epdPowerOn() {
  digitalWrite(EP_STV, HIGH);

  bool ok = true;
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO10_EP_OE, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO11_EP_MODE, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO15_TPS_WAKEUP, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO13_TPS_PWRUP, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO14_VCOM_CTRL, true);
  if (!ok) {
    epdPowerOff();
    return false;
  }

  delay(1);
  if (!waitForPcaPinHigh(PCA9535_IO16_TPS_PWR_GOOD, 400)) {
    epdPowerOff();
    return false;
  }
  if (!writeTpsRegister8(kTpsRegEnable, kTpsEnableOutputs)) {
    epdPowerOff();
    return false;
  }

  const uint16_t vcom = static_cast<uint16_t>(-kDefaultVcomMv / 10);
  const uint8_t vcomBytes[2] = {static_cast<uint8_t>(vcom & 0xFF), static_cast<uint8_t>(vcom >> 8)};
  if (!writeTpsRegister(kTpsRegVcom, vcomBytes, sizeof(vcomBytes))) {
    epdPowerOff();
    return false;
  }
  if (!waitForTpsReady(400)) {
    epdPowerOff();
    return false;
  }
  return true;
}

}  // namespace

namespace freeink {

const LgfxEpdConfig& lilygoT5S3LgfxConfig() {
  static const LgfxEpdConfig cfg = {
      {EP_D0, EP_D1, EP_D2, EP_D3, EP_D4, EP_D5, EP_D6, EP_D7},
      EP_STH,
      EP_STV,
      // pinOe: -1, not a dummy pin. This panel's real output-enable is
      // PCA9535_IO10_EP_OE, driven by the hooks above, so LovyanGFX needs no OE
      // of its own. It used to carry T5S3_LORA_CS as a placeholder, which put
      // the LoRa radio's chip select on an EPD pin. lgfx tolerates -1: pinMode()
      // returns early for a pin past GPIO_NUM_MAX and gpio_hi/gpio_lo are
      // guarded on pin >= 0.
      -1,
      EP_LEH,
      EP_CKH,
      EP_CKV,
      // pinPwr: still T5S3_LORA_CS and it has to be, because this one reaches
      // the i80 driver as dc_gpio_num and a negative value is rejected. There is
      // no free GPIO on this board to give it instead. prepareEpdPower() above
      // deselects it before the bus is built, which is what makes it harmless.
      T5S3_LORA_CS,
      16000000,
      8,
      0,
      {&prepareEpdPower, &epdPowerOn, &epdPowerOff},
      nullptr,
      0,
      kTextLut,
      sizeof(kTextLut) / sizeof(kTextLut[0]),
      kFastLut,
      sizeof(kFastLut) / sizeof(kFastLut[0]),
      // Null, so epd_fastest takes LovyanGFX's own lut_fastest: five drive rows
      // with **one** opposite pre-drive pass, eight passes in total. This is the
      // shipping fast table since 2026-09-17.
      //
      // Settled on two boards with matched content, forty marker moves, never
      // leaving the map: 11 passes (two pre-drive) and this one are
      // indistinguishable; kFast1bitLut's 7 (none) shows visible ghosting. So
      // one pre-drive pass is necessary and sufficient, and the pre-drive is a
      // mini-erase rather than the flash it was first read as.
      nullptr,
      0,
  };
  return cfg;
}

}  // namespace freeink
