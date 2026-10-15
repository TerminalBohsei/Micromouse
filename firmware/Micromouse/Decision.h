#pragma once
// Pure decision logic - no Arduino dependencies, so it can be unit-tested on a
// PC (see tests/test_decision.cpp).
#include <stdint.h>
#include <math.h>

struct Ranges     { int left; int front; int right; };   // mm
struct Thresholds { int sideOpen; int frontBlocked; };   // mm

enum class Action : uint8_t { FOLLOW, TURN_LEFT, TURN_RIGHT, TURN_AROUND };

struct Decision {
  Action action;
  bool   advanceFirst;   // true = drive on a little before turning
};

// Left-hand wall rule:
//   1. left is open            -> turn left (advance first if we're mid-corridor)
//   2. otherwise, front clear  -> carry on
//   3. otherwise, right open   -> turn right
//   4. otherwise               -> dead end, turn around
inline Decision decide(const Ranges& r, const Thresholds& t) {
  const bool leftOpen     = r.left  > t.sideOpen;
  const bool rightOpen    = r.right > t.sideOpen;
  const bool frontBlocked = r.front < t.frontBlocked;

  if (leftOpen)      return { Action::TURN_LEFT, !frontBlocked };
  if (!frontBlocked) return { Action::FOLLOW, false };
  if (rightOpen)     return { Action::TURN_RIGHT, false };
  return { Action::TURN_AROUND, false };
}

// Signed turn in degrees; positive = left (counter-clockwise).
inline float turnAngle(Action a) {
  switch (a) {
    case Action::TURN_LEFT:   return  90.0f;
    case Action::TURN_RIGHT:  return -90.0f;
    case Action::TURN_AROUND: return 180.0f;
    default:                  return   0.0f;
  }
}

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
