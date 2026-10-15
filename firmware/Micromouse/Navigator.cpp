#include "Navigator.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

Navigator::Navigator(Sensors& s, Motors& m, Stepper& st)
    : s_(s), m_(m), st_(st) {
  thr_ = { SIDE_OPEN_MM, FRONT_BLOCKED_MM };
}

// ------------------------------------------------------------------ setup ---
void Navigator::begin() {
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  floor_.begin();
  m_.stop();
  state_ = State::WAIT_START;
  armed_ = false;
  Serial.printf("Ready. Corridor %d mm -> wall target %d mm, opening > %d mm, advance %d mm.\n",
                CORRIDOR_MM, TARGET_SIDE_MM, SIDE_OPEN_MM, ADVANCE_MM);
  const int margin = CORRIDOR_MM / 2 - ROBOT_RADIUS_MM;
  Serial.printf("Spin clearance: %d mm per side when centred.%s\n", margin,
                margin < 20 ? "  WARNING: under 20 mm - any sideways error will scrape a wall "
                              "while turning. Centre the wheels / shorten the body." : "");
  Serial.println("Press the BOOT button, or type  s  to start.  Type  ?  for bench tests.");
}

void Navigator::blink(uint32_t now, uint32_t periodMs) {
  digitalWrite(PIN_LED, ((now / periodMs) & 1) ? HIGH : LOW);
}

// ------------------------------------------------------------------- loop ---
void Navigator::update() {
  s_.update();      // non-blocking
  st_.update();     // non-blocking

  const uint32_t now = millis();
  if (now - lastTick_ < LOOP_MS) return;
  lastTick_ = now;

  switch (state_) {
    case State::WAIT_START: waitStart(now); break;
    case State::COUNTDOWN:  countdown(now); break;
    case State::FOLLOW:     follow(now);    break;
    case State::ADVANCE:    advance(now);   break;
    case State::TURNING:    turning(now);   break;
    case State::FINISHED:
    case State::FAULT:
      blink(now, state_ == State::FINISHED ? 150 : 60);
      if (digitalRead(PIN_START) == LOW) {            // BOOT again = back to idle
        m_.stop();
        state_ = State::WAIT_START;
        armed_ = false;
        Serial.println("Back to idle.");
      }
      break;
  }
  telemetry(now);
}

void Navigator::telemetry(uint32_t now) {
#if DEBUG_SERIAL
  if (now - lastDbg_ < 100) return;
  lastDbg_ = now;
  const Ranges r = s_.ranges();
  Serial.printf("st=%d L=%d F=%d R=%d u=%.1f ir=%d\n",
                (int)state_, r.left, r.front, r.right, lastU_, floor_.raw());
#else
  (void)now;
#endif
}

// ----------------------------------------------------------- start / stop ---
void Navigator::waitStart(uint32_t now) {
  m_.stop();
  blink(now, 500);
  console();                                        // may start a bench test or set state_

  if (state_ != State::WAIT_START) return;
  const bool pressed = digitalRead(PIN_START) == LOW;
  if (!armed_) { if (!pressed) armed_ = true; return; }
  if (pressed) beginCountdown(now);
}

void Navigator::beginCountdown(uint32_t now) {
#if BENCH_FRONT_ONLY
  (void)now;
  Serial.println("Bench mode: the maze run needs all three ToF sensors. Use m, d or g. "
                 "(Set BENCH_FRONT_ONLY to 0 in Config.h for the real run.)");
  return;
#endif
  countdownEnd_ = now + START_DELAY_MS;
  state_ = State::COUNTDOWN;
  Serial.println("Starting - take your hands away and keep the robot still!");
}

void Navigator::countdown(uint32_t now) {
  m_.stop();
  blink(now, 100);
  if (now < countdownEnd_) return;

  // The robot is standing still and aligned with the maze: remember what the
  // floor looks like (the finish line is anything different).
  digitalWrite(PIN_LED, HIGH);
  floor_.captureBaseline();
  Serial.printf("Floor reading at start = %d (finish = anything different)\n", floor_.baseline());

  // Let the ToF filters fill with fresh readings before judging them.
  const uint32_t t0 = millis();
  while (millis() - t0 < 150) { s_.update(); delay(1); }
  if (!s_.fresh(SENSOR_STALE_MS)) { fault("sensors not ready at start"); return; }

  errValid_      = false;
  leftOpen_      = false;
  startMs_       = millis();
  lastTurnEnd_   = startMs_ - TURN_COOLDOWN_MS;
  state_         = State::FOLLOW;
  Serial.println("GO");
}

void Navigator::finish(uint32_t now, const char* why) {
  m_.brake();
  finishMs_  = now - startMs_;
  finishWhy_ = why;
  state_     = State::FINISHED;
  st_.moveSteps(STEPPER_FINISH_STEPS);
  Serial.printf("FINISHED (%s) in %lu ms\n", why, (unsigned long)finishMs_);
}

void Navigator::fault(const char* why) {
  m_.brake();
  state_ = State::FAULT;
  Serial.printf("FAULT: %s\n", why);
}

// Safety, time limit and end-zone detection. Returns false once stopped.
bool Navigator::running(uint32_t now) {
  if (!s_.fresh(SENSOR_STALE_MS)) { fault("ToF sensor stopped responding"); return false; }
  if (now - startMs_ > MAX_RUN_MS) { finish(now, "time limit"); return false; }
  if (IR_ENABLED) {
    floor_.update(now);
    if (floor_.finishDetected()) { finish(now, "end-zone"); return false; }
  }
  return true;
}

// ----------------------------------------------------------------- follow ---
void Navigator::follow(uint32_t now) {
  if (!running(now)) return;

  // Straight after a turn the ToF filters still hold readings taken while
  // rotating. Stand still (a wall may be only a few cm ahead) until they have
  // been refilled with fresh readings.
  if (now - lastTurnEnd_ < POST_TURN_MS) { m_.brake(); return; }

  const Ranges r = s_.ranges();

  // "Left is open" must persist for OPEN_CONFIRM_MS. Mid-corridor it is also
  // ignored for a moment after a turn, but NOT when a wall is right ahead:
  // then the left opening is the only way on (two left turns in a row), and
  // ignoring it would wrongly be read as a dead end.
  if (r.left > thr_.sideOpen) { if (!leftOpen_) { leftOpen_ = true; leftOpenAt_ = now; } }
  else                        { leftOpen_ = false; }
  const bool frontBlocked = r.front < thr_.frontBlocked;
  const bool leftOk = leftOpen_ && (now - leftOpenAt_) >= OPEN_CONFIRM_MS &&
                      (frontBlocked || (now - lastTurnEnd_) >= TURN_COOLDOWN_MS);

  Ranges rr = r;
  if (!leftOk && r.left > thr_.sideOpen) rr.left = thr_.sideOpen;   // not trusted yet: treat as wall

  const Decision d = decide(rr, thr_);
  if (d.action == Action::FOLLOW) { driveWalls(r); return; }

  pending_ = d.action;
  if (d.advanceFirst) beginAdvance(now, r);
  else                beginTurn(now);
}

void Navigator::beginAdvance(uint32_t now, const Ranges& r) {
  // With a wall visible ahead, the front ToF is a better odometer than a timer
  // (it does not care how charged the battery is). The timer is the fallback
  // and always the safety cap.
  advanceFront0_   = r.front;
  advanceViaFront_ = ADVANCE_USE_FRONT_TOF && r.front < (TOF_MAX_MM - 100);
  const uint32_t nominalMs = (uint32_t)ADVANCE_MM * 1000UL / CRUISE_MM_S;
  advanceEnd_ = now + (advanceViaFront_ ? nominalMs * ADVANCE_TIMEOUT_PCT / 100 : nominalMs);
  state_ = State::ADVANCE;
}

void Navigator::advance(uint32_t now) {
  if (!running(now)) return;
  const Ranges r = s_.ranges();
  bool done = now >= advanceEnd_ || r.front < thr_.frontBlocked;
  if (advanceViaFront_ && (advanceFront0_ - r.front) >= ADVANCE_MM) done = true;
  if (done) { beginTurn(now); return; }
  // Keep following whichever wall is still there (the right one, usually) so the
  // robot is centred in the corridor when it starts spinning; with no wall at
  // all this drives straight. Constant speed: the advance is
  // measured by distance/time.
  driveWalls(r, false);
}

// ------------------------------------------------------------------- turn ---
// Turns are open loop: stop, spin on the spot for a calibrated time, stop.
void Navigator::beginTurn(uint32_t now) {
  startSpin(now, turnAngle(pending_));
}

void Navigator::startSpin(uint32_t now, float deg) {
  turnDir_ = deg >= 0 ? 1.0f : -1.0f;
  const bool half = fabsf(deg) > 135.0f;                    // 180
  m_.brake();
  turnBegin_ = now + BRAKE_MS;                              // let the robot stop first
  turnEnd_   = turnBegin_ + (half ? TURN_180_MS : TURN_90_MS);
  state_     = State::TURNING;
}

void Navigator::turning(uint32_t now) {
  if (!testTurn_ && !running(now)) return;
  if (now < turnBegin_) return;                 // still braking

  if (now < turnEnd_) {
    m_.drive(-turnDir_ * TURN_SPEED, turnDir_ * TURN_SPEED);   // spin on the spot
    return;
  }
  m_.brake();
  if (testTurn_) {
    testTurn_ = false;
    pause(500);                                 // keep braking, exactly like a real turn does
    m_.stop();
    Serial.println("Turn done. Measure how far it really turned (protractor, or a printed "
                   "square corner) and fix TURN_90_MS / TURN_180_MS in Config.h.");
    state_ = State::WAIT_START;
    return;
  }
  lastTurnEnd_ = now;
  leftOpen_    = false;
  errValid_    = false;
  s_.restart();                                 // drop readings taken while rotating
  state_       = State::FOLLOW;
}

// ---------------------------------------------------------------- driving ---
// Steering convention: u > 0 steers LEFT (right wheel faster).
float Navigator::steer(const Ranges& r, uint32_t now) {
  const bool haveL = r.left  < STEER_WALL_MAX_MM;
  const bool haveR = r.right < STEER_WALL_MAX_MM;

  float u = 0;                                        // no wall: just go straight
  if (haveL || haveR) {
    float e;                                          // + = needs to move LEFT
    if (haveL && haveR) e = (r.left - r.right) * 0.5f;
    else if (haveL)     e = (float)r.left - TARGET_SIDE_MM;
    else                e = TARGET_SIDE_MM - (float)r.right;
    e = clampf(e, -MAX_WALL_ERR_MM, MAX_WALL_ERR_MM);

    // Rate of change of the error (smoothed): damps the weaving.
    if (errValid_ && now > prevErrMs_) {
      const float dt = (now - prevErrMs_) * 0.001f;
      errRate_ += D_ALPHA * ((e - prevErr_) / dt - errRate_);
    } else {
      errRate_ = 0;
    }
    prevErr_ = e; prevErrMs_ = now; errValid_ = true;
    u = KP_WALL * e + KD_WALL * errRate_;
  } else {
    errValid_ = false;
  }
  u = clampf(u, -MAX_CORR, MAX_CORR);
  lastU_ = u;
  return u;
}

void Navigator::driveAt(const Ranges& r, float baseCmd) {
  const float u = steer(r, millis());
  const float base = baseCmd * (1.0f - 0.4f * fabsf(u) / MAX_CORR);
  m_.drive(base - u, base + u);
}

void Navigator::driveWalls(const Ranges& r, bool slowForFront) {
  // Slow down as a wall approaches, and when steering hard.
  const float k = slowForFront
      ? clampf((float)(r.front - thr_.frontBlocked) / (float)(FRONT_SLOW_MM - thr_.frontBlocked),
               MIN_SPEED_SCALE, 1.0f)
      : 1.0f;
  driveAt(r, BASE_SPEED * k);
}

// ------------------------------------------------------------ bench tests ---
void Navigator::printHelp() {
  Serial.println("Bench tests (idle only, wheels OFF the ground for m):\n"
                 "  s  start the run          m  spin each wheel (check direction)\n"
                 "  d  drive straight 1 s     l/r/u  turn 90 left / 90 right / 180 (timed)\n"
                 "  f  stepper test           ?  this help\n"
                 "  g  (bench mode) drive until the front sensor sees a wall");
}

void Navigator::pause(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) { s_.update(); st_.update(); delay(1); }
}

void Navigator::console() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    switch (c) {
      case 's': beginCountdown(millis()); return;
      case 'm': testMotors();   break;
      case 'd': testDrive();    break;
#if BENCH_FRONT_ONLY
      case 'g': testGo();       break;
#endif
      case 'l': testTurn(90);   return;
      case 'r': testTurn(-90);  return;
      case 'u': testTurn(180);  return;
      case 'f': testStepper();  break;
      case '?': printHelp();    break;
      default:  break;          // ignore CR/LF/space/garbage
    }
  }
}

void Navigator::testMotors() {
  Serial.println("LEFT wheel forward...");
  m_.drive(120, 0);   pause(700); m_.stop(); pause(300);
  Serial.println("RIGHT wheel forward...");
  m_.drive(0, 120);   pause(700); m_.stop(); pause(300);
  Serial.println("BOTH backward...");
  m_.drive(-120, -120); pause(700); m_.stop();
  Serial.println("Each wheel must have turned the way the robot's FRONT points. If not, flip "
                 "MOTOR_LEFT_INVERT / MOTOR_RIGHT_INVERT in Config.h.");
}

void Navigator::testDrive() {
  Serial.println("Driving straight for 1 s at BASE_SPEED - measure the distance in mm, and note "
                 "which way it veers...");
  const uint32_t t0 = millis();
  while (millis() - t0 < 1000) { s_.update(); m_.drive(BASE_SPEED, BASE_SPEED); delay(5); }
  m_.brake();
  Serial.println("Done. Set CRUISE_MM_S in Config.h to the distance you measured (mm per second).\n"
                 "If it veered, nudge MOTOR_TRIM_LEFT / MOTOR_TRIM_RIGHT (e.g. 0.97) so it goes straight.");
}

void Navigator::testTurn(float deg) {
  Serial.printf("Timed turn of %.0f degrees (%s)...\n", deg, deg >= 0 ? "left" : "right");
  testTurn_ = true;
  startSpin(millis(), deg);
}

// Bench: drive straight until the front ToF sees a wall (or 4 s pass).
void Navigator::testGo() {
  Serial.printf("Driving forward; stops below %d mm or after 4 s. Keep a hand ready.\n", BENCH_STOP_MM);
  const uint32_t t0 = millis();
  while (millis() - t0 < 4000) {
    s_.update();
    if (!s_.fresh(SENSOR_STALE_MS)) {
      if (millis() - t0 > 500) { Serial.println("Front sensor went silent - STOP"); break; }
      m_.stop(); delay(5); continue;                  // wait for the first reading
    }
    const int front = s_.ranges().front;
    if (front < BENCH_STOP_MM) { Serial.printf("Wall at %d mm - STOP\n", front); break; }
    m_.drive(BASE_SPEED, BASE_SPEED);
    delay(5);
  }
  m_.brake();
  Serial.println("Stopped.");
}

void Navigator::testStepper() {
  if (!STEPPER_ENABLED) { Serial.println("Stepper disabled: set STEPPER_ENABLED true in Config.h."); return; }
  Serial.println("Stepper: forward then back...");
  st_.moveSteps(STEPPER_FINISH_STEPS);
  while (st_.busy()) { st_.update(); delay(1); }
  st_.moveSteps(-STEPPER_FINISH_STEPS);
  while (st_.busy()) { st_.update(); delay(1); }
}
