// Host-side tests for the pure decision logic.
//   g++ -std=c++11 -Wall -I. tests/test_unit.cpp -o test_decision && ./test_decision
#include <cassert>
#include <cstdio>
#include "Decision.h"
#include "RangeFilter.h"
#include "MotorMap.h"

int main() {
  const Thresholds t{150, 70};

  // Straight corridor with both walls: keep going.
  assert(decide({80, 400, 80}, t).action == Action::FOLLOW);

  // Left opens mid-corridor: turn left, but drive on first.
  {
    Decision d = decide({400, 400, 80}, t);
    assert(d.action == Action::TURN_LEFT && d.advanceFirst);
  }
  // Left open with a wall right ahead: turn now.
  {
    Decision d = decide({400, 50, 80}, t);
    assert(d.action == Action::TURN_LEFT && !d.advanceFirst);
  }
  // T-junction, both sides open: left has priority.
  assert(decide({400, 50, 400}, t).action == Action::TURN_LEFT);

  // Wall ahead and on the left, right open: turn right.
  assert(decide({80, 50, 400}, t).action == Action::TURN_RIGHT);

  // Dead end: turn around.
  assert(decide({80, 50, 80}, t).action == Action::TURN_AROUND);

  // Boundaries are strict: exactly at the threshold counts as a wall.
  assert(decide({150, 400, 80}, t).action == Action::FOLLOW);
  assert(decide({80, 70, 80}, t).action == Action::FOLLOW);

  // Turn angles.
  assert(turnAngle(Action::TURN_LEFT) == 90.0f);
  assert(turnAngle(Action::TURN_RIGHT) == -90.0f);
  assert(turnAngle(Action::TURN_AROUND) == 180.0f);
  assert(turnAngle(Action::FOLLOW) == 0.0f);

  // Heading snapping to the maze axes.
  assert(snap90(0.0f) == 0.0f);
  assert(snap90(44.0f) == 0.0f);
  assert(snap90(80.0f) == 90.0f);
  assert(snap90(-47.0f) == -90.0f);
  assert(snap90(172.0f) == 180.0f);

  // Accumulated turn error is removed: left, left, left, left, with 5 degrees
  // of error each time, still lands on a multiple of 90.
  {
    float h = 0;
    for (int i = 0; i < 4; i++) h = snap90(h + turnAngle(Action::TURN_LEFT) - 5.0f);
    assert(h == 360.0f);
  }

  // clampf
  assert(clampf(5, 0, 3) == 3);
  assert(clampf(-1, 0, 3) == 0);
  assert(clampf(2, 0, 3) == 2);

  // ---- RangeFilter: a single wild reading must NOT reach the output ----
  {
    RangeFilter f(0.6f);
    for (int i = 0; i < 5; i++) f.push(45);          // wall at 45 mm
    f.push(1000);                                    // one glitch ("far")
    assert(f.value() < 60);                          // still reads as a wall
    f.push(46);
    assert(f.value() < 60);
    for (int i = 0; i < 4; i++) f.push(1000);        // a REAL opening
    assert(f.value() > 900);
  }
  // The old low-pass-only filter (alpha .45) would have fake-opened here:
  {
    float lp = 45; const float a = 0.45f; int loopsOpen = 0;
    lp += a * (1000 - lp);                           // one glitch sample
    for (int i = 0; i < 12; i++) { if (lp > 150) loopsOpen++; lp += a * (45 - lp) * (i % 2 == 0 ? 0 : 1); }
    assert(loopsOpen >= 3);                          // documents the bug that was fixed
  }
  assert(RangeFilter::median3(1, 2, 3) == 2 && RangeFilter::median3(3, 1, 2) == 2 &&
         RangeFilter::median3(9, 9, 1) == 9 && RangeFilter::median3(5, 1, 1) == 1);
  {
    RangeFilter f(1.0f);
    f.push(100); f.push(100); f.push(100);
    f.restart(); f.push(300);                        // after restart: only new data counts
    assert(f.value() == 300);
  }

  // ---- MotorMap: dead-band ----
  assert(motorDuty(0, 60) == 0 && motorDuty(0.5f, 60) == 0);
  assert(motorDuty(1, 60) >= 60 && motorDuty(-1, 60) >= 60);
  assert(motorDuty(255, 60) == 255 && motorDuty(-255, 60) == 255 && motorDuty(999, 60) == 255);
  for (int v = 2; v < 255; v++) assert(motorDuty((float)v, 60) >= motorDuty((float)(v - 1), 60));  // monotonic

  std::printf("All unit tests passed.\n");
  return 0;
}
