#pragma once
// Pure maths: maps a signed drive command (-255..255) to an L298N duty cycle.
// TT gear motors do not turn at all below a certain PWM ("dead-band"), so any
// non-zero command is lifted to at least `minPwm`.
#include <math.h>

inline int motorDuty(float cmd, int minPwm) {
  const float a = fabsf(cmd);
  if (a < 1.0f) return 0;
  if (a > 255.0f) return 255;
  return minPwm + (int)((a / 255.0f) * (255 - minPwm));
}
