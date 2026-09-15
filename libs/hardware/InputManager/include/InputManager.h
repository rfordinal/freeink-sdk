#pragma once

// FreeInk SDK — input abstraction.
//
// Reads buttons across board input styles (ADC resistor ladder, plain digital
// buttons, confirm-hold-for-back, five-key) selected from BoardConfig::ACTIVE,
// and exposes a uniform edge/level button API. Capacitive touch is abstracted
// behind the same object: hasTouch()/getTouchPoint() are inert on boards
// without a configured TouchController.

#include <Arduino.h>
#include <BoardConfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstdint>

class InputManager {
 public:
  InputManager();
  void begin();
  uint8_t getState();

  // Call regularly from the main loop to update button and touch edge state.
  void update();

  // Level state from the last update().
  bool isPressed(uint8_t buttonIndex) const;

  // Current electrical level of the configured power-button GPIO, before
  // logical click/hold classification. False when the board has no such GPIO.
  bool isPowerButtonPhysicallyPressed() const;

  // Press edge since the previous update().
  bool wasPressed(uint8_t buttonIndex) const;

  // Any button press edge since the previous update().
  bool wasAnyPressed() const;

  // Release edge since the previous update().
  bool wasReleased(uint8_t buttonIndex) const;

  // Any button release edge since the previous update().
  bool wasAnyReleased() const;

  // True while a raw state change is still inside the debounce window (the last
  // raw sample differs from the committed state). A change only commits after
  // two consecutive matching samples, so hosts that poll slowly (e.g. a
  // sleep-sliced idle loop) should re-poll quickly while this is set —
  // otherwise a press shorter than the poll period lands in a single sample and
  // is dropped.
  bool isDebouncePending() const { return lastState != currentState; }

  // Duration between the first button press and final release.
  unsigned long getHeldTime() const;

  // Duration of the current or most recent power-button hold.
  unsigned long getPowerButtonHeldTime() const;

  static constexpr uint8_t BTN_BACK = 0;
  static constexpr uint8_t BTN_CONFIRM = 1;
  static constexpr uint8_t BTN_LEFT = 2;
  static constexpr uint8_t BTN_RIGHT = 3;
  static constexpr uint8_t BTN_UP = 4;
  static constexpr uint8_t BTN_DOWN = 5;
  static constexpr uint8_t BTN_POWER = 6;

  // Pins. POWER_BUTTON_PIN stays constexpr (consumers reference it in
  // pin-config contexts) and is bound to the build's default device; the input
  // code reads the runtime-active power pin internally so multi-device builds
  // stay correct.
  static constexpr int BUTTON_ADC_PIN_1 = 1;
  static constexpr int BUTTON_ADC_PIN_2 = 2;
  static constexpr int POWER_BUTTON_PIN = BoardConfig::DEFAULT_DEVICE.input.power;

  bool isPowerButtonPressed() const;

  static const char* getButtonName(uint8_t buttonIndex);

  // --- Capacitive touch (inert unless BoardConfig::ACTIVE.touch is configured)
  // ---
  struct TouchPoint {
    bool valid;
    uint16_t x;
    uint16_t y;
    unsigned long timestamp;
  };

  // Fixed-size contact snapshot for allocation-free multi-touch polling. `id`
  // is the GT911 track ID when `idsStable` is true. Coordinate-only GT911
  // frames expose frame-local record slots instead, so callers must not retain
  // those IDs across frames. `count` is capped at MAX_TOUCH_CONTACTS;
  // `reportedCount` keeps the controller's actual count so callers can detect
  // truncation and opt into only the contact counts they support.
  static constexpr uint8_t MAX_TOUCH_CONTACTS = 4;
  struct MultiTouchPoint {
    uint8_t id;
    TouchPoint point;
  };
  struct TouchSnapshot {
    uint8_t count;
    uint8_t reportedCount;
    bool idsStable;
    MultiTouchPoint points[MAX_TOUCH_CONTACTS];
  };

  // One GT911 frame as the controller reported it, before any interpretation.
  //
  // Raw records rather than decoded points on purpose: decoding needs
  // BoardConfig and the mounting correction, which is policy, and a frame that
  // crosses a thread boundary should carry only what the chip said. `timestamp`
  // is when the frame was READ, which is the only honest time for it.
  struct Gt911Frame {
    unsigned long timestamp;
    uint8_t status;                            // 0x814E: bit 7 ready, bit 4 home key, bits 3..0 contacts
    uint8_t storedCount;                       // records actually read, <= MAX_TOUCH_CONTACTS
    uint8_t records[MAX_TOUCH_CONTACTS * 8];   // 0x8150 onward, undecoded
    bool pointsValid;                          // false when the point read failed but the status did not
  };

  // --- The GT911's own sampling task -------------------------------------
  //
  // Why this exists, measured on an X4 Pro 2026-09-14. The controller holds
  // exactly ONE unacknowledged frame and produces no further frame until 0x814E
  // is cleared: one frame sat in the register for 7.99 s of an 8.00 s capture
  // while the capacitive key was tapped throughout, and not one of those taps
  // reached it. The app's loop stops sampling for 2.80 s on a map redraw and
  // 4.34 s opening the map. Every edge of a gesture made in that window but the
  // first is therefore destroyed, and a double tap arrives as a single tap.
  //
  // No policy for acknowledging frames can fix that -- NOT clearing is worse,
  // because the controller then reports nothing at all. The only fix is to read
  // on a schedule the renderer cannot stall, which is what this task is.
  //
  // It owns the I2C half and the home key's recogniser. Contact frames are
  // handed to the app's thread through a queue and interpreted there, because
  // the glass is consumed as a live level (drag, touch-down select, hint boxes)
  // and a finished-gesture queue cannot answer "where is the finger now".
  //
  // No-op unless a GT911 is present, so a board without one pays no stack and no
  // task slot. Mutually exclusive with beginAsync(), which drives update() from
  // a task of its own -- two threads in this class's unlocked state is a data
  // race, so the second caller is refused.
  void beginGt911Task(uint8_t taskPriority = 3, uint32_t pollMs = 10);

  // What the home key's recogniser is looking for. A board that does not carry a
  // double tap sets doubleWindowMs = 0, which makes a tap fire the instant the
  // key is released -- the default, because waiting is a latency cost and only a
  // board that spends it should pay it.
  //
  // minInterTapMs is a LOWER bound on the gap between two taps, the way Android's
  // DOUBLE_TAP_MIN_TIME is (40 ms). It rejects contact bounce without swallowing
  // a fast third tap, which is what a refractory window does.
  // What the recogniser emits. Press is kept as its own event rather than
  // folded into Tap because the sleep timer counts any key activity, and
  // because ordering through one queue is what keeps a late drain honest.
  enum class HomeKeyGesture : uint8_t { Press, Tap, DoubleTap, LongPress, Cancel };

  struct HomeKeyGestureSpec {
    uint16_t longMs = 700;
    uint16_t doubleWindowMs = 0;
    uint16_t minInterTapMs = 40;
  };
  void setHomeKeyGestureSpec(const HomeKeyGestureSpec& spec);

  // How old a COMPLETED glass contact may be before its tap is refused, in ms.
  // 0 (the default) never refuses one.
  //
  // Same argument as the home key's tap, and the glass needs it more: a tap is
  // aimed at a particular row or a particular place on a map, and after a render
  // that took seconds the thing it was aimed at has moved or gone. The age is
  // tested at the RELEASE, not the press -- a finger still down when the loop
  // resumes is a live interaction whatever time it started, and only a gesture
  // that began and ended unseen is stale.
  //
  // A number, not a policy: what it should be is the app's call, like the key's.
  void setTouchStaleMs(uint16_t ms);

  // What the sampling task actually managed to do, so "the fix works" can be
  // told apart from "the run was lucky".
  //
  // The loop is SUPPOSED to stall -- that is the condition being survived -- so
  // a passing double tap proves nothing on its own unless the task's own cadence
  // is known to have held through it. `cancels` is the honest failure count: the
  // gap rule firing means a gesture was dropped rather than mistimed, which is
  // the intended degradation and not a success.
  struct Gt911TaskStats {
    uint32_t ticks;           // sampler iterations since the last reset
    uint32_t maxGapUs;        // worst interval between two of them
    uint32_t gapsOverLimit;   // intervals past the cancel threshold
    uint32_t cancels;         // gestures dropped because of one
    uint32_t frameOverflows;  // contact frames lost to a full queue
  };
  Gt911TaskStats gt911TaskStats(bool reset);

  // One-shot, cleared each #update(), like the rest of the key's events. Only
  // ever true when doubleWindowMs is non-zero.
  bool wasHomeKeyDoubleTapped() const;

  // When the key event delivered this frame actually happened, in millis().
  //
  // The whole reason for a recogniser next to the read is that a gesture carries
  // the time the FINGER made it rather than the time the loop got round to it.
  // Exposing that is what lets a caller refuse to act on a stale one -- and
  // whether a given gesture may go stale is a question about meaning, so it is
  // answered up in the app and not here. A tap is a contextual action and the
  // context moves; a double tap is a mode toggle and cannot age.
  //
  // Zero when no key event was delivered this frame.
  unsigned long homeKeyEventAtMs() const;

  // Bookkeeping for the same reason the task counts its cadence: so "a gesture
  // went missing" can be told apart from "a gesture was never made".
  struct HomeKeyCounters {
    uint32_t produced;    // gestures the recogniser emitted
    uint32_t delivered;   // gestures update() handed to the app
    uint32_t queueDrops;  // gestures lost because the queue was full
  };
  HomeKeyCounters homeKeyCounters(bool reset);

  // True if this board has a touch controller configured.
  bool hasTouch() const;
  // True only while a GT911 controller is present. Other touch controllers
  // retain their existing single-contact contract.
  bool supportsMultiTouch() const;
  // Latest GT911 contacts, capped at MAX_TOUCH_CONTACTS. Gesture consumers
  // should check reportedCount for the exact cardinality they support.
  TouchSnapshot getTouchSnapshot() const;
  // The most recent touch sampled during #update(). valid == false when idle.
  TouchPoint getTouchPoint() const;
  // True while a touch is currently down.
  bool isTouchPressed() const;
  // True if a touch began between the last two #update() calls.
  bool wasTouchPressed() const;
  // True if a touch ended between the last two #update() calls.
  bool wasTouchReleased() const;
  // One-shot tap on release. Returns the original touch-down position
  // normalized to 0..1 in the panel's native frame; false when no tap completed
  // this update.
  bool wasTouchTap(float& nx, float& ny) const;
  // Press edge with the touch-down position normalized in the panel's native
  // frame.
  bool wasTouchPressedAt(float& nx, float& ny) const;
  // True while the current touch is still a tap candidate: finger down,
  // movement remains within tap slop. Writes the original touch-down position
  // and held time.
  bool isTouchTapCandidate(float& nx, float& ny, unsigned long& heldMs) const;
  // True while a touch is down; writes the CURRENT contact position normalized
  // to 0..1 in the panel's native frame. Unlike #isTouchTapCandidate there is
  // no tap-slop gate — the position follows the moving finger, for drag
  // interactions (sliders). Callers own any threshold/hysteresis they need.
  bool isTouchHeldAt(float& nx, float& ny) const;
  // Duration (ms) of the last touch contact, latched on release.
  unsigned long lastTouchHeldMs() const;
  // Swipe gesture on release. Returns start/end positions normalized in the
  // panel's native frame; callers map orientation and check this before tap.
  bool wasSwipe(float& nxStart, float& nyStart, float& nxEnd, float& nyEnd) const;
  // One-shot 2-4 contact translation gesture. The SDK reports the number of
  // contacts plus their centroid start/end normalized in its panel-native
  // frame; applications opt into the exact counts they support, map display
  // orientation, and decide whether the result is up/down/left/right. Pinches,
  // diagonal motion, delayed gestures, ambiguous contact matching, track-ID
  // replacements, and sequences above MAX_TOUCH_CONTACTS are rejected.
  bool wasMultiTouchSwipe(uint8_t& contactCount, float& nxStart, float& nyStart, float& nxEnd, float& nyEnd,
                          unsigned long& durationMs) const;
  // One-shot two-contact rotation on release. Positive degrees are clockwise
  // and negative degrees counterclockwise in the corrected panel-native frame,
  // normalized to [-180, 180]. The center is the average of the start/end
  // contact centroids, normalized to 0..1. Pinches and sub-threshold turns are
  // rejected, and an accepted rotation cannot also become a translation.
  bool wasMultiTouchRotation(float& degrees, float& nxCenter, float& nyCenter, unsigned long& durationMs) const;
  // One-shot long-press: fires WHILE the finger is still down, once a
  // stationary contact (within tap slop) has been held TOUCH_LONG_PRESS_MS.
  // Position is the touch-down point, normalized like wasTouchTap. Fires at
  // most once per contact. Detection alone has no side effects; callers that
  // act on it should call suppressTouchContact() so the eventual lift cannot
  // also tap. Cleared each #update().
  bool wasTouchLongPress(float& nx, float& ny) const;
  // Ignore the remainder of the current contact: tap, swipe, release,
  // tap-candidate and held queries report nothing until the finger lifts and
  // the release edge has passed, then a fresh contact is delivered normally.
  // Self-clears; async tap, single-swipe, multi-touch-swipe, and rotation
  // queues are gated by the same latch.
  void suppressTouchContact();
  // True if a touch press or release happened this frame. Coarse "the user
  // touched the screen" signal (the touch analogue of wasAnyPressed/Released)
  // for resetting idle/sleep timers and restoring CPU frequency. False on
  // non-touch boards.
  bool wasTouchActivity() const;
  // True on the press edge of the GT911 capacitive home key (controllers
  // without one never report it). Cleared each #update().
  bool wasHomeKeyPressed() const;
  // True once on release of a SHORT home-key press (held < the long-press
  // threshold); the primary "home" action. Suppressed when the same hold
  // already fired wasHomeKeyLongPressed(). Cleared each #update().
  bool wasHomeKeyTapped() const;
  // True once when the home key has been held past the long-press threshold
  // (~700 ms), while still down — a hold shortcut (e.g. open the reader menu).
  // Cleared each #update().
  bool wasHomeKeyLongPressed() const;

  // Optional board hook for buttons that aren't direct GPIOs — e.g. a key
  // behind an I2C IO-expander (the LilyGo T5 S3 user button on its PCA9535). It
  // returns a (1<<BTN_*) bitmask that is OR'd into every update(); the board
  // reads its expander, so InputManager itself stays device-agnostic. Default:
  // none.
  using ButtonHook = uint8_t (*)();
  static void setButtonHook(ButtonHook hook) { s_buttonHook = hook; }

  // Boards such as Sticky wire OK/confirm and power/wake to the same GPIO. By
  // default a short click emits CONFIRM and a hold emits POWER. Apps that
  // expose a "short power click sleeps" option can flip short clicks to POWER.
  static void setSharedConfirmPowerShortPressEmitsPower(bool enabled) {
    s_sharedConfirmPowerShortPressEmitsPower = enabled;
  }

  // --- Optional background polling -------------------------------------------
  // Spawns a FreeRTOS task that samples the buttons every pollMs and latches
  // each press edge (a BTN_* index) into an internal queue. This decouples
  // input from rendering: on e-paper, a slow refresh blocks the app's main
  // loop, so a press that lands mid-refresh is otherwise lost — the task keeps
  // sampling (refresh busy-waits yield via delay()) and the app drains presses
  // with popPress() afterward. No-op if already started.
  //
  // When async polling is active the app must NOT call update()/wasPressed()
  // itself; the task owns the edge state. Drain with popPress() instead.
  void beginAsync(uint8_t taskPriority = 2, uint32_t pollMs = 15, uint8_t queueLen = 32);

  // Pop the next latched button index (BTN_*) into `button`. Returns false when
  // no press is pending (or async polling was never started).
  bool popPress(uint8_t& button);

  // Pop the next latched touch tap (normalized 0..1 panel-native coordinates,
  // same frame as wasTouchTap). The async task queues every completed tap, so
  // taps that land while the app thread renders or waits are never lost —
  // drain and route them afterwards. Returns false when no tap is pending.
  bool popTouchTap(float& nx, float& ny);

  // Pop the next latched swipe gesture (normalized 0..1 panel-native start/end
  // coordinates, same frame as wasSwipe). Like taps, async polling queues
  // swipes so gestures that complete during e-paper refreshes are not lost.
  bool popSwipe(float& nxStart, float& nyStart, float& nxEnd, float& nyEnd);

  // Pop a queued 2-4 contact translation, in the same normalized panel-native
  // coordinate frame as wasMultiTouchSwipe(). It has a dedicated queue so
  // existing popSwipe() consumers never receive multi-touch gestures. The
  // returned count is captured with the queued event and cannot be confused
  // with a later controller frame.
  bool popMultiTouchSwipe(uint8_t& contactCount, float& nxStart, float& nyStart, float& nxEnd, float& nyEnd,
                          unsigned long& durationMs);

  // Pop a queued completed two-contact rotation. Values use the same signed
  // degrees, normalized center, and duration contract as
  // wasMultiTouchRotation().
  bool popMultiTouchRotation(float& degrees, float& nxCenter, float& nyCenter, unsigned long& durationMs);

  // --- Diagnostics -----------------------------------------------------------
  // A live sample of one button-group ADC pin: the raw reading plus the BTN_*
  // it currently classifies as (-1 = no band matched). On the Xteink ADC ladder
  // the six buttons are resistor dividers multiplexed onto two ADC pins
  // (Back/Confirm/Left/Right on group 1, Up/Down on group 2); X3 and X4 share
  // this pinout. A button-test or calibration screen uses this to spot a
  // drifted divider whose reading no longer lands in the band the firmware
  // expects — visible from the raw value regardless of how it classifies.
  struct ButtonAdcSample {
    int pin;     // GPIO sampled (BUTTON_ADC_PIN_1 / BUTTON_ADC_PIN_2)
    int raw;     // raw analogRead() value, or -1 if this board has no ADC ladder
    int button;  // classified BTN_* index, or -1 for no match
  };

  // Sample both button-group ADC pins now (synchronous analogRead). Safe to
  // call alongside async polling. On boards without the Xteink ADC ladder both
  // samples report raw = -1, button = -1.
  void readButtonAdc(ButtonAdcSample& group1, ButtonAdcSample& group2);

 private:
  static ButtonHook s_buttonHook;

  QueueHandle_t _asyncQueue = nullptr;
  QueueHandle_t _asyncTapQueue = nullptr;
  QueueHandle_t _asyncSwipeQueue = nullptr;
  struct QueuedMultiTouchSwipe {
    uint16_t startX;
    uint16_t startY;
    uint16_t endX;
    uint16_t endY;
    uint8_t contactCount;
    uint16_t durationMs;
  };
  QueueHandle_t _asyncMultiTouchSwipeQueue = nullptr;
  struct QueuedMultiTouchRotation {
    float degrees;
    uint16_t centerX;
    uint16_t centerY;
    uint16_t durationMs;
  };
  QueueHandle_t _asyncMultiTouchRotationQueue = nullptr;
  TaskHandle_t _asyncTask = nullptr;
  uint32_t _asyncPollMs = 15;
  static void asyncTaskTrampoline(void* self);
  void asyncPoll();

  int getButtonFromADC(int adcValue, const int ranges[], int numButtons);
  bool isDigitalPressed(int8_t pin) const;
  uint8_t getDigitalState() const;
  void updateConfirmBackHold(unsigned long currentTime);
  void updateConfirmPowerHold(unsigned long currentTime);
  void updateDigitalTwoButton(unsigned long currentTime);
  void applyStateChange(uint8_t state, unsigned long currentTime);

  // Touch backend. Compiled only when FREEINK_CAP_TOUCH is set; dispatches on
  // BoardConfig::ACTIVE.touch.controller (CHSC6x, GT911, or FT5x06/FT6336).
  void beginTouch();
  uint8_t serviceTouch();  // runs the machine; returns synthesized button mask
  void updateTouchFromIrq(unsigned long now,
                          int irqRaw);  // CHSC6x I2C poll + touch-bit gate
  // GT911, split in two so the I2C half can run somewhere other than the app's
  // loop. Measured on an X4 Pro 2026-09-14: the controller holds exactly one
  // unacknowledged frame and produces no further frame until 0x814E is cleared,
  // so a loop blocked for a panel refresh (2.8 s for a redraw, 4.3 s to open a
  // map) destroys every edge of a gesture but the first. Reading has to happen
  // on a schedule the renderer cannot stall; applying does not.
  //
  // The split is exact: `readFrame` performs every I2C access and the clear and
  // touches no gesture state; `applyFrame` performs no I2C and is the whole of
  // the previous `pollGt911()` body. `now` was already a parameter, so a frame
  // applied late is stamped with when it was READ rather than when it was
  // handled, which is the property the whole redesign turns on.
  bool gt911ReadFrame(Gt911Frame& frame);
  void gt911ApplyFrame(const Gt911Frame& frame);
  void pollGt911(unsigned long now);    // read + apply, for the synchronous path

  // The home key's recogniser. Runs wherever the reading runs -- in the task
  // when there is one, in update() otherwise -- because a recogniser fed on a
  // slow thread measures that thread's latency instead of the finger's timing.
  // `frameReady` is false for a tick with no new frame; the hold timer and the
  // double-tap window still have to advance on those.
  void gt911RecogniseKey(unsigned long now, bool frameReady, bool keyDown);
  void gt911PushKeyGesture(uint8_t gesture, unsigned long now);
  static void gt911TaskTrampoline(void* self);
  void gt911TaskLoop();
  // True when a frame is worth queueing: an edge always, a resting contact at a
  // reduced rate. A 4.3 s render at 10 ms would otherwise be 430 frames.
  bool gt911FrameWorthQueueing(const Gt911Frame& frame) const;
  void beginFt5x06();
  void pollFt5x06(unsigned long now);
  bool ft5x06WriteReg(uint8_t reg, uint8_t value);
  bool ft5x06ReadReg(uint8_t reg, uint8_t* buf, uint8_t len);
  bool readChsc6xPoint(TouchPoint& point);
  bool decodeChsc6xFrame(const uint8_t* data, size_t len, TouchPoint& point) const;
  uint16_t mapTouchAxis(uint16_t raw, uint16_t rawMin, uint16_t rawMax, uint16_t outMax) const;
  void beginGt911();
  bool gt911ReadReg(uint16_t reg, uint8_t* buf, uint8_t len);
  void gt911ClearStatus();
  void beginFt6336u();
  void pollFt6336u(unsigned long now);
  void beginGslx680();
  void pollGslx680(unsigned long now);
  bool gslWrite(uint8_t reg, const uint8_t* data, uint8_t len);
  bool gslRead(uint8_t reg, uint8_t* buf, uint8_t len);
  bool gslUploadFirmware();

  enum class MultiTouchGestureState : uint8_t { Idle, Tracking, Blocked };
  struct TrackedTouchContact {
    uint8_t id;
    TouchPoint start;
    TouchPoint last;
  };
  void updateMultiTouchGesture(const TouchSnapshot& snapshot, unsigned long now);
  void startMultiTouchGesture(const TouchSnapshot& snapshot, unsigned long now);
  void blockMultiTouchGesture();
  void finishMultiTouchGesture(unsigned long now);
  void resetMultiTouchGesture();
  void cancelMultiTouchGesture();
  bool matchMultiTouchSnapshot(const TouchSnapshot& snapshot);
  bool expandMultiTouchGesture(const TouchSnapshot& snapshot, unsigned long now);
  bool findContactAssignment(const TouchSnapshot& snapshot, uint8_t trackedCount,
                             uint8_t assignment[MAX_TOUCH_CONTACTS]) const;
  bool isTrackedContact(const MultiTouchPoint& point) const;
  bool hasStableTranslationGeometry() const;
  bool hasEligibleRotationScale() const;
  bool isMultiTouchTranslation(unsigned long now) const;
  bool classifyMultiTouchRotation(unsigned long now);
  void normalizeTouchPoint(uint16_t x, uint16_t y, float& nx, float& ny) const;

  uint8_t currentState;
  uint8_t lastState;
  uint8_t pressedEvents;
  uint8_t releasedEvents;
  unsigned long lastDebounceTime;
  unsigned long buttonPressStart;
  unsigned long buttonPressFinish;
  unsigned long powerButtonPressStart;
  unsigned long powerButtonPressFinish;
  unsigned long confirmBackPressStart;
  bool confirmBackPhysicalPressed;
  bool confirmBackLongPressActive;
  unsigned long confirmPowerPressStart;
  bool confirmPowerPhysicalPressed;
  bool confirmPowerLongPressActive;
  uint8_t twoButtonPhysicalState;
  unsigned long twoButtonPressStart;
  bool twoButtonLongPressActive;

  bool touchDataEnabled = false;         // I2C up, controller present
  uint8_t gt911Addr = 0;                 // resolved GT911 address (0 until probed)
  unsigned long touchIrqPulseUntil = 0;  // synthesized-confirm window after a press
  unsigned long touchReadAt = 0;         // next scheduled I2C poll
  unsigned long touchReleaseAt = 0;
  bool touchPressed = false;
  bool touchPressedEvent = false;
  bool touchReleasedEvent = false;
  bool touchHomeKeyEvent = false;  // GT911 capacitive home key, press edge
  bool touchHomeKeyDown = false;
  bool touchHomeKeyTapEvent = false;   // short-press release edge (one-shot)
  bool touchHomeKeyLongEvent = false;  // held past the long-press threshold (one-shot)
  bool touchHomeKeyLongFired = false;  // latched for the current hold so long
                                       // fires once and suppresses the tap
  bool touchHomeKeyDoubleTapEvent = false;  // one-shot, cleared each update()
  unsigned long touchHomeKeyEventAtMs = 0;  // when this frame's event happened; 0 = none

  // One queued gesture: the type, and when the finger made it. The timestamp is
  // the payload that the old design could not carry -- without it a late
  // delivery is indistinguishable from a prompt one.
  struct HomeKeyEvent {
    uint8_t type;
    uint32_t atMs;
  };
  volatile uint32_t _keyProduced = 0;
  volatile uint32_t _keyDelivered = 0;
  volatile uint32_t _keyQueueDrops = 0;

  // Recogniser state, written only where the reading happens.
  HomeKeyGestureSpec homeKeySpec{};
  uint16_t touchStaleMs = 0;
  unsigned long keyPendingTapAt = 0;  // a tap waiting to find out if a second is coming; 0 = none
  unsigned long keyLastTickAt = 0;    // for the gap rule below
  bool keySwallowRelease = false;     // the release that completed a double tap emits nothing

  // GT911 task and its two channels.
  TaskHandle_t _gt911Task = nullptr;
  uint32_t _gt911PollMs = 10;
  QueueHandle_t _gt911FrameQueue = nullptr;    // contact frames, applied on the app thread
  QueueHandle_t _gt911GestureQueue = nullptr;  // finished key gestures
  volatile bool _gt911FrameOverflow = false;   // the queue filled; the stream is torn
  unsigned long _gt911LastQueuedMs = 0;
  uint8_t _gt911LastQueuedStatus = 0xFF;
  // The most recent frame coalescing threw away. It is queued ahead of the next
  // edge so a release is never the first thing the app sees after a press --
  // see gt911TaskLoop() for what that costs when it is missing.
  Gt911Frame _gt911Skipped{};
  bool _gt911HasSkipped = false;
  volatile uint32_t _gt911Ticks = 0;
  volatile uint32_t _gt911MaxGapUs = 0;
  volatile uint32_t _gt911GapsOverLimit = 0;
  volatile uint32_t _gt911Cancels = 0;
  volatile uint32_t _gt911Overflows = 0;
  unsigned long _gt911LastTickUs = 0;
  unsigned long touchHomeKeyDownAt = 0;
  static constexpr unsigned long HOME_KEY_LONG_PRESS_MS = 700;
  TouchPoint touchPoint = {false, 0, 0, 0};
  TouchSnapshot touchSnapshot{};
  MultiTouchGestureState multiTouchGestureState = MultiTouchGestureState::Idle;
  TrackedTouchContact multiTouchContacts[MAX_TOUCH_CONTACTS] = {};
  uint8_t trackedTouchContactCount = 0;
  bool multiTouchRotationEligible = false;  // latched false if any frame leaves the allowed scale band
  bool touchMultiContactSequence = false;   // suppresses single-contact classifiers until full release
  bool multiTouchSwipeEvent = false;
  uint8_t multiTouchSwipeContactCount = 0;
  uint16_t multiTouchSwipeStartX = 0;
  uint16_t multiTouchSwipeStartY = 0;
  uint16_t multiTouchSwipeEndX = 0;
  uint16_t multiTouchSwipeEndY = 0;
  uint16_t multiTouchSwipeDurationMs = 0;
  bool multiTouchRotationEvent = false;
  float multiTouchRotationDegrees = 0.0f;
  uint16_t multiTouchRotationCenterX = 0;
  uint16_t multiTouchRotationCenterY = 0;
  uint16_t multiTouchRotationDurationMs = 0;
  TouchPoint touchDownPoint = {false, 0, 0, 0};  // first sample of the current contact (tap routing)
  TouchPoint touchUpPoint = {false, 0, 0, 0};    // last sample before release (swipe routing)
  unsigned long lastTouchHeldDurationMs = 0;     // contact duration, latched at release
  bool touchMovedBeyondTapSlop = false;          // cancels stationary hold/long-press classification
  bool touchMovedBeyondTapReleaseSlop = false;   // cancels tap-on-release once motion reaches swipe distance
  bool touchLongPressEvent = false;              // one-shot, mirrors touchHomeKeyLongEvent
  bool touchLongPressFired = false;              // latched for the current contact so long-press fires once
  bool touchSuppressed = false;                  // suppressTouchContact() latch; holds through
                                                 // the release-edge frame, cleared in
                                                 // serviceTouch() once the contact is over

  static constexpr int NUM_BUTTONS_1 = 4;
  static const int ADC_RANGES_1[];

  static constexpr int NUM_BUTTONS_2 = 2;
  static const int ADC_RANGES_2[];

  static constexpr int ADC_NO_BUTTON = 3900;
  static constexpr unsigned long DEBOUNCE_DELAY = 5;
  static constexpr unsigned long CONFIRM_BACK_HOLD_MS = 650;
  static constexpr unsigned long CONFIRM_POWER_HOLD_MS = 400;
  static constexpr unsigned long TWO_BUTTON_HOLD_MS = 650;

  // Touch timing / protocol constants (ported from the Murphy M3 CHSC6x
  // driver).
  static constexpr unsigned long TOUCH_IRQ_PULSE_MS = 120;   // release hold-over after last valid read
  static constexpr unsigned long TOUCH_SAMPLE_DELAY_MS = 8;  // I2C poll cadence
  static constexpr int TOUCH_TAP_SLOP_PX = 28;
  static constexpr int TOUCH_SWIPE_MIN_PX = 60;
  static constexpr int TOUCH_TAP_RELEASE_SLOP_PX = TOUCH_SWIPE_MIN_PX - 1;
  static constexpr unsigned long TOUCH_SWIPE_MAX_MS = 700;
  static constexpr unsigned long TOUCH_MULTI_SWIPE_MAX_MS = 2000;
  static constexpr int TOUCH_MULTI_CONTACT_SEPARATION_SLOP_PX = 45;
  static constexpr int64_t TOUCH_CONTACT_ASSIGNMENT_AMBIGUITY_PX_SQ = 64;
  static constexpr unsigned long TOUCH_LONG_PRESS_MS = 500;  // shorter than HOME_KEY_LONG_PRESS_MS: a screen hold has
                                                             // no button travel to absorb
  static constexpr uint8_t TOUCH_READ_COMMAND = 0x00;
  static constexpr uint8_t TOUCH_FRAME_SIZE = 16;

  static const char* BUTTON_NAMES[];
  static bool s_sharedConfirmPowerShortPressEmitsPower;
};
