#pragma once
#include <stdint.h>
#include "Decision.h"
#include "Sensors.h"
#include "Gyro.h"
#include "Motors.h"

enum class State : uint8_t {
  WAIT_START,   // idle, waiting for the start button
  COUNTDOWN,    // button pressed, waiting START_DELAY_MS
  FOLLOW,       // wall following
  ADVANCE,      // driving on a little before a left turn
  TURNING,      // closed-loop turn using the gyro
  FINISHED,     // end-zone reached (or time limit)
  FAULT         // sensor failure - stopped for safety
};

class Navigator {
 public:
  Navigator(Sensors& s, Gyro& g, Motors& m);
  void  begin();
  void  update();                    // call from loop()
  State state() const { return state_; }

 private:
  void waitStart(uint32_t now);
  void countdown(uint32_t now);
  void follow(uint32_t now, float dt);
  void advance(uint32_t now, float dt);
  void turning(uint32_t now);

  bool running(uint32_t now);        // safety + end-zone checks; false = stopped
  void beginTurn(uint32_t now);
  void finish(uint32_t now, const char* why);
  void fault(const char* why);
  void driveWalls(const Ranges& r, float dt);
  void driveHeading(float baseSpeed);
  int  readIR() const;
  void blink(uint32_t now, uint32_t periodMs);

  Sensors& s_;
  Gyro&    g_;
  Motors&  m_;
  Thresholds thr_;

  State    state_   = State::WAIT_START;
  uint32_t lastTick_ = 0, lastDbg_ = 0;
  uint32_t startMs_ = 0, countdownEnd_ = 0, advanceEnd_ = 0;
  uint32_t turnBegin_ = 0, turnDeadline_ = 0, lastTurnEnd_ = 0;

  Action  pending_       = Action::FOLLOW;
  float   headingTarget_ = 0, turnTarget_ = 0;
  uint8_t leftOpenCnt_   = 0, settle_ = 0, irCnt_ = 0;
  int     irBase_        = 0;

  // wall-following PD state
  enum class Mode : uint8_t { NONE, BOTH, LEFT, RIGHT, GYRO } mode_ = Mode::NONE;
  float lastErr_ = 0, dErr_ = 0, lastU_ = 0;
};
