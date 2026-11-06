#include <BoardT5S3.h>
#include <LgfxEpdConfig.h>
#include <Logging.h>
#include <Wire.h>

namespace {

constexpr int kDefaultVcomMv = -1600;

// Panel bus clock. Stock 16 MHz; EXPLORINK_T5S3_BUS_HZ overrides it for the
// T-253 load test. Hoisted to a named constant purely so epdPowerOn() can log
// it: a build flag that silently fails to reach this translation unit looks
// exactly like a knob that does nothing, and the first 8 MHz attempt could not
// tell those apart.
#ifndef EXPLORINK_T5S3_BUS_HZ
#define EXPLORINK_T5S3_BUS_HZ 16000000
#endif
constexpr uint32_t kBusHz = EXPLORINK_T5S3_BUS_HZ;

// The VCOM actually programmed on the next epdPowerOn(). Mutable, not constexpr,
// for the T-253 sweep: VCOM is an **assigned per-panel value with a +/-0.1 V
// window** whose centre nobody downstream knows -- the factory does not
// calibrate per unit (docs/panel-ed047tc1.md, "VCOM is an assigned per-panel
// value"). Too negative a VCOM weakens the black drive and strengthens the
// white one (effective field is source - VCOM), which is exactly how a black
// field lands grey. epdPowerOn() runs before every refresh, so a write here
// takes effect on the very next frame and the sweep costs no reflash.
int g_vcomMv = kDefaultVcomMv;

// Starts armed, so the first power-up after boot always dumps without anyone
// having to send a command -- CMD:TPS re-arms it for a later look. A latch
// rather than a direct read because the part only answers I2C while the power
// sequence holds it awake, and arming cannot be relied on from the console:
// FreeInkBusEPD::powerControl() short-circuits when the rails are already up
// (LgfxEpdDriver.cpp, `if (_pwr_on == powerOn) return true;`), so a refresh
// asked for from a console command may never reach epdPowerOn() at all.
bool g_dumpTpsOnNextPowerOn = true;

constexpr uint8_t kTpsRegTmstValue = 0x00;
constexpr uint8_t kTpsRegEnable = 0x01;
constexpr uint8_t kTpsRegVcom = 0x03;
constexpr uint8_t kTpsRegPowerGood = 0x0F;
constexpr uint8_t kTpsRegRevId = 0x10;
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
[[maybe_unused]] constexpr uint32_t kFastLut[] = {
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

// Extra pure black/white drive rows appended to the 12 stock ones, 0 = off.
//
// Why this knob exists. Decoding kTextLut's column 0 (the black target) gives
// 2,2,2,2,2 then 1,1,1,3,1,1,1 -- five passes driving the pixel toward WHITE
// (the eraser prefix) and then six driving it black. Six is what a black pixel
// gets out of a clean frame. kFast1bitLut, which the menus use and which board
// B renders as a solid black, spends four passes of pure black drive with no
// eraser in front of them.
//
// Board B lands a full-black epd_text frame grey while board A lands it black,
// on identical firmware, identical TPS65185 registers and an identical scan
// time, and the effect scales with how much of the panel is black. Six drive
// passes being marginally short on one panel and sufficient on another fits
// that; so does the top-to-bottom gradient. These rows add dose without
// touching the eraser or the greys: 1 drives black, 2 drives white, 3 is
// inert, which is exactly kFast1bitLut's row.
#ifndef EXPLORINK_TEXT_LUT_EXTRA_DRIVE
#define EXPLORINK_TEXT_LUT_EXTRA_DRIVE 0
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
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 1
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 2
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 3
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 4
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 5
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 6
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 7
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 8
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 9
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 10
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 11
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
#if EXPLORINK_TEXT_LUT_EXTRA_DRIVE >= 12
    LUT_MAKE(1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2),
#endif
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

// elapsedMs and lastRead are diagnostic out-params (T-253 / docs/panel-ed047tc1.md
// VCOM-per-panel investigation, 2026-11-06): epdPowerOn() below used to discard
// how close each wait came to its 400 ms timeout, so a board whose TPS65185 is
// marginally slow -- not outright failing, just eating most of the margin --
// looked identical to one booting instantly. Both are nullptr-safe for any
// future caller that does not care.
bool waitForPcaPinHigh(uint8_t pin, uint32_t timeoutMs, uint32_t* elapsedMs = nullptr) {
  const uint32_t start = millis();
  bool high = false;
  while (millis() - start < timeoutMs) {
    if (BoardT5S3::readPca9535Pin(pin, &high) && high) {
      if (elapsedMs) *elapsedMs = millis() - start;
      return true;
    }
    delay(1);
  }
  if (elapsedMs) *elapsedMs = millis() - start;
  return false;
}

bool waitForTpsReady(uint32_t timeoutMs, uint32_t* elapsedMs = nullptr, uint8_t* lastRead = nullptr) {
  const uint32_t start = millis();
  uint8_t powerGood = 0;
  while (millis() - start < timeoutMs) {
    if (readTpsRegister(kTpsRegPowerGood, &powerGood, 1) && (powerGood & 0xFA) == 0xFA) {
      if (elapsedMs) *elapsedMs = millis() - start;
      if (lastRead) *lastRead = powerGood;
      return true;
    }
    delay(1);
  }
  if (elapsedMs) *elapsedMs = millis() - start;
  if (lastRead) *lastRead = powerGood;
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
  // Logged so the rail cycling is visible. Whether the rails come down between
  // refreshes decides whether a VCOM written from the console is ever
  // programmed, because VCOM is only pushed to the PMIC inside epdPowerOn().
  LOG_ERR("T5S3PWR", "epdPowerOff: rails down");
  BoardT5S3::writePca9535Pin(PCA9535_IO10_EP_OE, false);
  BoardT5S3::writePca9535Pin(PCA9535_IO11_EP_MODE, false);
  BoardT5S3::writePca9535Pin(PCA9535_IO13_TPS_PWRUP, false);
  BoardT5S3::writePca9535Pin(PCA9535_IO14_VCOM_CTRL, false);
  delay(1);
  BoardT5S3::writePca9535Pin(PCA9535_IO15_TPS_WAKEUP, false);
  digitalWrite(EP_STV, LOW);
}

// Diagnostic logging added 2026-11-06 for the T-253 VCOM-per-panel
// investigation (docs/panel-ed047tc1.md): this function previously had zero
// log lines on any of its five failure returns (firmware/explorink's own
// Documenting Findings rule: "a silent no-op branch makes a hardware capture
// unable to tell nothing-to-do from never-ran" -- the same gap, here on a
// power-up that silently tears the rails back down). Board A (t5s3pro) shows
// a crisp black sleep/full-refresh screen; board B (t5s3pro-b) shows the same
// draw as a pale, washed-out field -- same firmware, same freeink-sdk commit,
// same hardcoded -1600 mV VCOM on both. This logging exists to tell apart a
// board that is simply slow to raise PWR_GOOD/PowerGood (still within margin,
// points at per-panel VCOM/dose) from one that is silently failing
// epdPowerOn() and running the scan with torn-down rails (points at the TPS
// chain itself). Remove once T-253 is settled, or promote to a permanent
// DBG line if it proves worth keeping.
bool epdPowerOn() {
  digitalWrite(EP_STV, HIGH);

  bool ok = true;
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO10_EP_OE, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO11_EP_MODE, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO15_TPS_WAKEUP, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO13_TPS_PWRUP, true);
  ok &= BoardT5S3::writePca9535Pin(PCA9535_IO14_VCOM_CTRL, true);
  if (!ok) {
    LOG_ERR("T5S3PWR", "epdPowerOn: PCA9535 pin write failed (step 1, enable pins)");
    epdPowerOff();
    return false;
  }

  delay(1);
  uint32_t pcaElapsedMs = 0;
  const bool pcaOk = waitForPcaPinHigh(PCA9535_IO16_TPS_PWR_GOOD, 400, &pcaElapsedMs);
  if (!pcaOk) {
    LOG_ERR("T5S3PWR", "epdPowerOn: PCA9535 PWR_GOOD pin never went high (%u ms, timeout 400)", pcaElapsedMs);
    epdPowerOff();
    return false;
  }
  if (pcaElapsedMs > 50) {
    LOG_ERR("T5S3PWR", "epdPowerOn: PCA9535 PWR_GOOD pin slow: %u ms of 400 budget", pcaElapsedMs);
  }
  if (!writeTpsRegister8(kTpsRegEnable, kTpsEnableOutputs)) {
    LOG_ERR("T5S3PWR", "epdPowerOn: TPS65185 I2C write of reg 0x%02X failed", kTpsRegEnable);
    epdPowerOff();
    return false;
  }

  const uint16_t vcom = static_cast<uint16_t>(-g_vcomMv / 10);
  const uint8_t vcomBytes[2] = {static_cast<uint8_t>(vcom & 0xFF), static_cast<uint8_t>(vcom >> 8)};
  if (!writeTpsRegister(kTpsRegVcom, vcomBytes, sizeof(vcomBytes))) {
    LOG_ERR("T5S3PWR", "epdPowerOn: TPS65185 I2C write of VCOM reg 0x%02X failed", kTpsRegVcom);
    epdPowerOff();
    return false;
  }
  // Read the VCOM pair straight back. A write that is ACKed is not proof the
  // part took the value, and this sweep's whole conclusion rests on the
  // programmed VCOM actually being what we asked for.
  uint8_t vcomBack[2] = {0, 0};
  const bool vcomReadOk = readTpsRegister(kTpsRegVcom, vcomBack, sizeof(vcomBack));
  uint32_t tpsElapsedMs = 0;
  uint8_t tpsLastPowerGood = 0;
  const bool tpsOk = waitForTpsReady(400, &tpsElapsedMs, &tpsLastPowerGood);
  if (!tpsOk) {
    LOG_ERR("T5S3PWR", "epdPowerOn: TPS65185 PowerGood (reg 0x%02X) never settled, last=0x%02X mask=0x%02X, %u ms of 400",
            kTpsRegPowerGood, tpsLastPowerGood, 0xFA, tpsElapsedMs);
    epdPowerOff();
    return false;
  }
  LOG_ERR("T5S3PWR", "epdPowerOn: OK, pca=%ums tps=%ums powerGood=0x%02X vcomMv=%d wrote=0x%02X%02X readback=%s0x%02X%02X busHz=%u textLutLen=%u",
          pcaElapsedMs, tpsElapsedMs, tpsLastPowerGood, g_vcomMv, vcomBytes[1], vcomBytes[0],
          vcomReadOk ? "" : "(read failed)", vcomBack[1], vcomBack[0], (unsigned)kBusHz,
          (unsigned)(sizeof(kTextLut) / sizeof(kTextLut[0])));

  // The register dump has to happen HERE, not from a console command: outside a
  // refresh epdPowerOff() drives TPS_WAKEUP low and every I2C read to the part
  // fails with ESP_ERR_INVALID_STATE (measured on board B, 2026-11-06). This is
  // the only moment the PMIC is awake and reachable.
  //
  // VADJ (0x02) is the register this was added for. It sets the VPOS/VNEG
  // source rails, and **nothing in this firmware ever writes it** -- the power
  // sequence writes ENABLE and VCOM only, so each part runs on whatever VADJ it
  // powers up with. Lower rails mean a weaker drive and a black that lands
  // grey, with power-good still asserted (PG compares against the configured
  // rails, not an absolute), and VCOM unable to recover it (VCOM shifts the
  // reference, it does not add headroom). That is the shape of board B's
  // symptom, so the value is worth seeing on both boards before anything else
  // is concluded. `[open]` until the two dumps are actually compared.
  if (g_dumpTpsOnNextPowerOn) {
    g_dumpTpsOnNextPowerOn = false;
    for (uint8_t reg = 0x00; reg <= kTpsRegRevId; ++reg) {
      uint8_t v = 0;
      if (readTpsRegister(reg, &v, 1)) {
        LOG_ERR("T5S3PWR", "  TPS[0x%02X] = 0x%02X (%u)", reg, v, v);
      } else {
        LOG_ERR("T5S3PWR", "  TPS[0x%02X] = READ FAILED", reg);
      }
    }
  }
  return true;
}

}  // namespace

namespace BoardT5S3 {

// T-253 bench instruments. Declared in BoardT5S3.h, defined here because the
// TPS65185 I2C helpers and the VCOM the power sequence programs both live in
// this file's anonymous namespace.

void setEpdVcomMv(int mv) { g_vcomMv = mv; }

int getEpdVcomMv() { return g_vcomMv; }

int getEpdDefaultVcomMv() { return kDefaultVcomMv; }

bool readTpsRegister8Public(uint8_t reg, uint8_t* value) { return readTpsRegister(reg, value, 1); }

void armTpsDump() { g_dumpTpsOnNextPowerOn = true; }

// Dumps the registers that say what the part is doing right now: temperature,
// enable mask, the VCOM pair, power-good and the revision id. The register
// numbers are TI's published TPS65185 map; **the datasheet is not on disk**, so
// treat the names as unverified -- what is verified is that 0x01, 0x03 and 0x0F
// are the enable, VCOM and power-good registers this board's own working power
// sequence has always used.
//
// TMST_VALUE (0x00) is reported raw. Whether it holds a live conversion or a
// stale one depends on the part's acquisition mode, which nothing here
// configures, so a reading is a lead and not a measurement. `[open]`
void dumpTpsRegisters() {
  const struct {
    uint8_t reg;
    const char* name;
  } regs[] = {
      {kTpsRegTmstValue, "TMST_VALUE"}, {kTpsRegEnable, "ENABLE"},    {0x02, "VADJ"},
      {kTpsRegVcom, "VCOM1"},           {0x04, "VCOM2"},              {kTpsRegPowerGood, "PG"},
      {kTpsRegRevId, "REVID"},
  };
  for (const auto& r : regs) {
    uint8_t v = 0;
    if (readTpsRegister(r.reg, &v, 1)) {
      LOG_ERR("T5S3PWR", "TPS reg 0x%02X %-10s = 0x%02X (%u)", r.reg, r.name, v, v);
    } else {
      LOG_ERR("T5S3PWR", "TPS reg 0x%02X %-10s = READ FAILED", r.reg, r.name);
    }
  }
  LOG_ERR("T5S3PWR", "vcomMv now %d (default %d)", g_vcomMv, kDefaultVcomMv);
}

}  // namespace BoardT5S3

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
      // busHz. Stock is 16 MHz; EXPLORINK_T5S3_BUS_HZ overrides it for the
      // T-253 load test, which is the whole reason this is not a literal.
      //
      // What the test is after: board B draws a full-black epd_text frame as
      // grey while board A draws it black, on bit-identical firmware, with all
      // 17 TPS65185 registers equal and the same 1,23 s scan. The effect tracks
      // **black coverage** -- 21 % black is black on both, 50 % is black with B
      // slightly greyer, 100 % splits them -- and the bottom of the panel comes
      // out lighter than the top, so it builds during the scan. That is the
      // shape of a supply drooping under load, and power-good cannot see it:
      // waitForTpsReady() samples once, before the scan starts.
      //
      // Halving the clock halves the row rate, which lowers the average draw
      // and gives the rail time to recover between rows. If B's full-black
      // frame closes to black at 8 MHz, the droop is real and this is a lever;
      // if nothing moves, droop is out and the next suspect is series
      // resistance in the panel's own power path (the FPC contact).
      kBusHz,
      8,
      0,
      {&prepareEpdPower, &epdPowerOn, &epdPowerOff},
      nullptr,
      0,
      kTextLut,
      sizeof(kTextLut) / sizeof(kTextLut[0]),
      // Null, so epd_fast takes LovyanGFX's own lut_fast. kFastLut is the same
      // eleven passes and differs only in its two pre-drive rows, which drive
      // the greys where the library drives only black and white. That
      // difference has never been judged against stock on a full-screen solid,
      // and on 2026-09-18 that is exactly the surface that caught epd_fastest
      // out, so the override goes away until something prices it.
      nullptr,
      0,
      // Null, so epd_fastest takes LovyanGFX's own lut_fastest. Reachable
      // through CMD:EPDLUT 1, and **not** the shipping mode: it shipped for one
      // day and left the sleep screen's logo as a watermark over Home.
      // LgfxEpdDriver.cpp's g_fastMode carries the reasoning.
      nullptr,
      0,
  };
  return cfg;
}

}  // namespace freeink
