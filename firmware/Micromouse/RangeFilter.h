#pragma once
// Pure maths (no Arduino code) so it can be unit-tested on a PC.
//
// A ToF sensor occasionally returns a single wild reading (a "far" glitch when
// a wall is very close, a reflection, ...). A plain low-pass filter smears
// such a spike over several loops, which is enough to fake a junction. A
// median-of-3 removes any single outlier completely; the low-pass on top then
// just smooths the remaining noise.
#include <stdint.h>

class RangeFilter {
 public:
  explicit RangeFilter(float alpha = 0.5f) : alpha_(alpha) {}

  // Feed one valid sample in millimetres.
  void push(int mm) {
    buf_[idx_] = mm;
    idx_ = (uint8_t)((idx_ + 1) % 3);
    if (n_ < 3) n_++;
    const float m = (n_ < 3) ? (float)mm : (float)median3(buf_[0], buf_[1], buf_[2]);
    if (!seeded_) { lp_ = m; seeded_ = true; }
    else          { lp_ += alpha_ * (m - lp_); }
  }

  // Forget the history (e.g. after a turn, when old samples point the wrong
  // way). The last value stays readable until the first new sample arrives.
  void restart() { n_ = 0; idx_ = 0; seeded_ = false; }

  bool seeded() const { return seeded_; }
  int  value()  const { return (int)(lp_ + 0.5f); }

  static int median3(int a, int b, int c) {
    if (a > b) { int t = a; a = b; b = t; }   // now a <= b
    if (b > c) { b = c; }                     // b = min(b, c)
    return a > b ? a : b;                     // median = max(a, min(b, c))
  }

 private:
  float   alpha_;
  float   lp_     = 0;
  int     buf_[3] = {0, 0, 0};
  uint8_t idx_    = 0, n_ = 0;
  bool    seeded_ = false;
};
