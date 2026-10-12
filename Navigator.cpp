#include "Navigator.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

Navigator::Navigator(Sensors& s, Gyro& g, Motors& m) : s_(s), g_(g), m_(m) {
  thr_ = { SIDE_OPEN_MM, FRONT_BLOCKED_MM };
}

// ------------------------------------------------------------------ setup ---
void Navigator::begin() {
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_IR, INPUT);

  // Floor baseline for end-zone detection.
  long sum = 0;
  for (int i = 0; i < 32; i++) { sum += readIR(); delay(2); }
  irBase_ = (int)(sum / 32);

  m_.stop();
  state_ = State::WAIT_START;
  Serial.printf("Ready. IR floor baseline = %d. Press the start button.\n", irBase_);
}

int Navigator::readIR() const {
  return IR_ANALOG ? analogRead(PIN_IR) : (digitalRead(PIN_IR) ? 4095 : 0);
}

void Navigator::blink(uint32_t now, uint32_t periodMs) {
  digitalWrite(PIN_LED, ((now / periodMs) & 1) ? HIGH : LOW);
}

// ------------------------------------------------------------------- loop ---
void Navigator::update() {
  g_.update();      // integrate the gyro as often as possible
  s_.update();      // non-blocking

  const uint32_t now = millis();
  if (now - lastTick_ < LOOP_MS) return;
  float dt = (now - lastTick_) * 0.001f;
  lastTick_ = now;
  if (dt > 0.05f) dt = 0.05f;

  switch (state_) {
    case State::WAIT_START: waitStart(now);    break;
    case State::COUNTDOWN:  countdown(now);    break;
    case State::FOLLOW:     follow(now, dt);   break;
    case State::ADVANCE:    advance(now, dt);  break;
    case State::TURNING:    turning(now);      break;
    case State::FINISHED:   blink(now, 150);   break;
    case State::FAULT:      blink(now, 60);    break;
  }

#if DEBUG_SERIAL
  if (now - lastDbg_ >= 100) {
    lastDbg_ = now;
    const Ranges r = s_.ranges();
    Serial.printf("st=%d L=%d F=%d R=%d hdg=%.1f u=%.1f\n",
                  (int)state_, r.left, r.front, r.right, g_.heading(), lastU_);
  }
#endif
}

// ----------------------------------------------------------- start / stop ---
void Navigator::waitStart(uint32_t now) {
  m_.stop();
  blink(now, 500);
  if (digitalRead(PIN_START) == LOW) {
    countdownEnd_ = now + START_DELAY_MS;
    state_ = State::COUNTDOWN;
    Serial.println("Starting - take your hand away!");
  }
}

void Navigator::countdown(uint32_t now) {
  m_.stop();
  blink(now, 100);
  if (now < countdownEnd_) return;

  if (!s_.fresh(SENSOR_STALE_MS)) { fault("sensors not ready at start"); return; }

  g_.resetHeading();            // the robot is aligned with the maze here
  headingTarget_ = 0;
  lastTurnEnd_   = now - TURN_COOLDOWN_MS;
  leftOpenCnt_   = 0;
  irCnt_         = 0;
  mode_          = Mode::NONE;
  startMs_       = now;
  digitalWrite(PIN_LED, HIGH);
  state_ = State::FOLLOW;
}

void Navigator::finish(uint32_t now, const char* why) {
  m_.brake();
  state_ = State::FINISHED;
  Serial.printf("FINISHED (%s) in %lu ms\n", why, (unsigned long)(now - startMs_));
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
    if (abs(readIR() - irBase_) > IR_DELTA) {
      if (irCnt_ < 255) irCnt_++;
    } else {
      irCnt_ = 0;
    }
    if (irCnt_ >= IR_DEBOUNCE) { finish(now, "end-zone"); return false; }
  }
  return true;
}

// ----------------------------------------------------------------- follow ---
void Navigator::follow(uint32_t now, float dt) {
  if (!running(now)) return;

  const Ranges r = s_.ranges();

  // Debounce the "left is open" signal and ignore it right after a turn.
  if (r.left > thr_.sideOpen) { if (leftOpenCnt_ < 255) leftOpenCnt_++; }
  else                        { leftOpenCnt_ = 0; }
  const bool leftOk = leftOpenCnt_ >= OPEN_DEBOUNCE &&
                      (now - lastTurnEnd_) >= TURN_COOLDOWN_MS;

  Ranges rr = r;
  if (!leftOk && r.left > thr_.sideOpen) rr.left = thr_.sideOpen;   // treat as wall

  const Decision d = decide(rr, thr_);
  if (d.action == Action::FOLLOW) {
    driveWalls(r, dt);
    return;
  }

  pending_ = d.action;
  if (d.advanceFirst) {
    advanceEnd_ = now + ADVANCE_MS;
    state_ = State::ADVANCE;
  } else {
    beginTurn(now);
  }
}

void Navigator::advance(uint32_t now, float dt) {
  (void)dt;
  if (!running(now)) return;
  const Ranges r = s_.ranges();
  if (now >= advanceEnd_ || r.front < thr_.frontBlocked) {
    beginTurn(now);
    return;
  }
  driveHeading(ADVANCE_SPEED);
}

// ------------------------------------------------------------------- turn ---
void Navigator::beginTurn(uint32_t now) {
  turnTarget_   = snap90(g_.heading() + turnAngle(pending_));
  m_.brake();
  turnBegin_    = now + BRAKE_MS;
  turnDeadline_ = turnBegin_ + TURN_TIMEOUT_MS;
  settle_       = 0;
  state_        = State::TURNING;
}

void Navigator::turning(uint32_t now) {
  if (now < turnBegin_) return;                 // still braking

  const float err = turnTarget_ - g_.heading(); // + = need to turn left
  if (fabsf(err) < TURN_TOL_DEG) {
    m_.brake();
    if (fabsf(g_.rate()) < TURN_RATE_TOL_DPS) settle_++; else settle_ = 0;
  } else {
    settle_ = 0;
    const float s   = clampf(KP_TURN * fabsf(err), MIN_TURN_PWM, TURN_SPEED);
    const float dir = err > 0 ? 1.0f : -1.0f;
    m_.drive(-dir * s, dir * s);                // spin on the spot
  }

  if (settle_ >= 2 || now > turnDeadline_) {
    headingTarget_ = turnTarget_;
    lastTurnEnd_   = now;
    leftOpenCnt_   = 0;
    mode_          = Mode::NONE;
    state_         = State::FOLLOW;
  }
}

// ---------------------------------------------------------------- driving ---
// Steering convention: u > 0 steers LEFT (right wheel faster).
void Navigator::driveWalls(const Ranges& r, float dt) {
  const bool haveL = r.left  < thr_.sideOpen;
  const bool haveR = r.right < thr_.sideOpen;

  Mode mode;
  float e = 0;
  if (haveL && haveR) { mode = Mode::BOTH;  e = (r.left - r.right) * 0.5f; }
  else if (haveL)     { mode = Mode::LEFT;  e = (float)r.left - TARGET_SIDE_MM; }
  else if (haveR)     { mode = Mode::RIGHT; e = TARGET_SIDE_MM - (float)r.right; }
  else                { mode = Mode::GYRO; }

  float u;
  if (mode == Mode::GYRO) {
    u = KP_HEADING * (headingTarget_ - g_.heading());
    dErr_ = 0;
  } else {
    if (mode != mode_) { lastErr_ = e; dErr_ = 0; }   // avoid a derivative spike
    const float de = (e - lastErr_) / dt;
    dErr_   += 0.3f * (de - dErr_);
    lastErr_ = e;
    u = KP_WALL * e + KD_WALL * dErr_;
    headingTarget_ = snap90(g_.heading());            // remember the maze axis
  }
  mode_ = mode;
  u = clampf(u, -MAX_CORR, MAX_CORR);
  lastU_ = u;

  // Slow down as a wall approaches, and when steering hard.
  const float k = clampf((float)(r.front - thr_.frontBlocked) /
                         (float)(FRONT_SLOW_MM - thr_.frontBlocked),
                         MIN_SPEED_SCALE, 1.0f);
  const float base = BASE_SPEED * k * (1.0f - 0.4f * fabsf(u) / MAX_CORR);
  m_.drive(base - u, base + u);
}

void Navigator::driveHeading(float baseSpeed) {
  const float u = clampf(KP_HEADING * (headingTarget_ - g_.heading()), -MAX_CORR, MAX_CORR);
  lastU_ = u;
  m_.drive(baseSpeed - u, baseSpeed + u);
}
