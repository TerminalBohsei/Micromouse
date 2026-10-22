#pragma once
#include <stdint.h>
#include "Decision.h"
#include "Sensors.h"
#include "Motors.h"
#include "Stepper.h"
#include "Floor.h"

enum class State : uint8_t {
  WAIT_START,   // idle, waiting for the start button / serial 's'
  COUNTDOWN,    // button pressed, waiting START_DELAY_MS
  FOLLOW,       // wall following
  ADVANCE,      // driving on a little before a left turn
  TURNING,      // spinning on the spot for a fixed time
  FINISHED,     // end-zone reached (or time limit)
  FAULT,        // sensor failure - stopped for safety
  WANDER        // endless obstacle-avoiding drive (BOOT starts it, BOOT stops it)
};

class Navigator {
 public:
  Navigator(Sensors& s, Motors& m, Stepper& st);
  void  begin();
  void  update();                    // call from loop()
  State state() const { return state_; }
  uint32_t finishTimeMs() const { return finishMs_; }
  const char* finishReason() const { return finishWhy_; }

 private:
  // states
  void waitStart(uint32_t now);
  void countdown(uint32_t now);
  void follow(uint32_t now);
  void advance(uint32_t now);
  void turning(uint32_t now);

  // helpers
  bool running(uint32_t now, bool pauseOnSilence = true);        // safety + end-zone checks; false = stopped
  void beginCountdown(uint32_t now, bool wanderMode);
  void wander(uint32_t now);
  bool stopRequested();                              // BOOT pressed again, or any serial key
  void stopRun(const char* why);
  void beginAdvance(uint32_t now, const Ranges& r);
  void beginTurn(uint32_t now);
  void startSpin(uint32_t now, float deg);          // + = left
  void finish(uint32_t now, const char* why);
  void fault(const char* why);
  void driveWalls(const Ranges& r, bool slowForFront = true);
  float steer(const Ranges& r, uint32_t now);         // wall-centring correction (PWM, + = left)
  void driveAt(const Ranges& r, float base);
  void blink(uint32_t now, uint32_t periodMs);
  void telemetry(uint32_t now);

  // bench tests (idle state only), driven from the serial console
  void console();
  void printHelp();
  void testMotors();
  void testDrive();
  void testTurn(float deg);
  void testStepper();
  void testGo();
  void autoDrive();
  void testWheels();
  void adjustTrim(float fl, float fr);   // multiply each side's trim by these factors
  void pause(uint32_t ms);           // wait while keeping the sensors updating

  Sensors& s_;
  Motors&  m_;
  Stepper& st_;
  FloorSensor floor_;
  Thresholds thr_;

  State    state_    = State::WAIT_START;
  uint32_t lastTick_ = 0, lastDbg_ = 0;
  uint32_t startMs_  = 0, finishMs_ = 0, countdownEnd_ = 0;
  uint32_t advanceEnd_ = 0, turnBegin_ = 0, turnDeadline_ = 0, lastTurnEnd_ = 0;
  uint32_t leftOpenAt_ = 0;
  const char* finishWhy_ = "";

  Action  pending_       = Action::FOLLOW;
  float   lastU_ = 0;
  float   prevErr_ = 0, errRate_ = 0;      // wall-centring error and its smoothed rate
  bool    errValid_ = false;
  uint32_t prevErrMs_ = 0;
  uint32_t turnEnd_ = 0;
  float   turnDir_ = 1;                     // +1 left, -1 right
  int     advanceFront0_ = 0;
  bool    leftOpen_      = false;
  bool    advanceViaFront_ = false;
  bool    armed_         = false;    // start button must be released before it counts
  bool     wander_       = false;    // endless wander mode (no end-zone, no time limit)
  bool     btnReleased_  = false;    // BOOT has been released since the run started
  uint8_t  btnDownTicks_ = 0;
  uint32_t autoAt_       = 0;        // when the automatic wheel test starts (0 = never)
  bool    paused_        = false;    // waiting for a silent ToF sensor (message printed once)
  bool    sensorFault_   = false;    // the FAULT was a sensor dropout: resume when they return
  bool    testTurn_      = false;    // a console turn test is running
};