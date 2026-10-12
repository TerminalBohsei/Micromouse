// Host-side tests for the pure decision logic.
//   g++ -std=c++11 -Wall -I. tests/test_decision.cpp -o test_decision && ./test_decision
#include <cassert>
#include <cstdio>
#include "../Decision.h"

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

  std::printf("All decision tests passed.\n");
  return 0;
}
