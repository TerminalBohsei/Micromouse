#include "world.h"
#include "Config.h"
#include "MotorMap.h"
#include <math.h>
#include <algorithm>

World* g_world = nullptr;

double World::rnd() {                  // uniform in [0, 1)
  rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
  return (double)(rng >> 33) / 2147483648.0;
}
double World::gauss() {
  double u = rnd() + 1e-12, v = rnd();
  return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

static void addRect(std::vector<Seg>& s, double x1, double y1, double x2, double y2) {
  s.push_back({x1, y1, x2, y1}); s.push_back({x2, y1, x2, y2});
  s.push_back({x2, y2, x1, y2}); s.push_back({x1, y2, x1, y1});
}

void World::generateMaze(int w, int h, uint64_t seed) {
  W = w; H = h; rng = seed * 2654435761ULL + 99;
  for (int i = 0; i < 5; i++) rnd();
  // hw[j][i]: horizontal wall on line j (0..H) above column i; vw[j][i]: vertical wall on line i (0..W) in row j
  std::vector<std::vector<bool>> hw(H + 1, std::vector<bool>(W, true)), vw(H, std::vector<bool>(W + 1, true));
  std::vector<bool> seen(W * H, false);
  std::vector<int> stack;
  stack.push_back(0); seen[0] = true;
  while (!stack.empty()) {                       // randomised DFS -> perfect maze (no loops)
    int c = stack.back(), cx = c % W, cy = c / W;
    int nb[4], n = 0;
    if (cx + 1 < W && !seen[c + 1]) nb[n++] = 0;
    if (cx > 0     && !seen[c - 1]) nb[n++] = 1;
    if (cy + 1 < H && !seen[c + W]) nb[n++] = 2;
    if (cy > 0     && !seen[c - W]) nb[n++] = 3;
    if (!n) { stack.pop_back(); continue; }
    int d = nb[(int)(rnd() * n) % n], nc = c;
    if (d == 0) { vw[cy][cx + 1] = false; nc = c + 1; }
    if (d == 1) { vw[cy][cx] = false;     nc = c - 1; }
    if (d == 2) { hw[cy + 1][cx] = false; nc = c + W; }
    if (d == 3) { hw[cy][cx] = false;     nc = c - W; }
    seen[nc] = true; stack.push_back(nc);
  }
  segs.clear();
  const double t = T / 2;
  for (int j = 0; j <= H; j++) for (int i = 0; i < W; i++) if (hw[j][i])
    addRect(segs, i * P - t, j * P - t, (i + 1) * P + t, j * P + t);
  for (int j = 0; j < H; j++) for (int i = 0; i <= W; i++) if (vw[j][i])
    addRect(segs, i * P - t, j * P - t, i * P + t, (j + 1) * P + t);
}

void World::placeRobot(int cx, int cy, double dx, double dy, double yawDeg) {
  // Axle in the middle of the cell, facing +x.
  x = (cx + 0.5) * P + dx; y = (cy + 0.5) * P + dy; th = yawDeg * M_PI / 180.0;
  vl = vr = 0; contacts = 0; inContact = false; distance = 0;
  for (int i = 0; i < 3; i++) { nextSample[i] = nowUs * 1e-6 + 0.005 * (i + 1); ready[i] = false; }
}

void World::samplePose(double lx, double ly, double& wx, double& wy) const {
  wx = x + lx * cos(th) - ly * sin(th);
  wy = y + lx * sin(th) + ly * cos(th);
}

double World::raycast(double sx, double sy, double ang, double maxd) const {
  const double dx = cos(ang), dy = sin(ang);
  double best = maxd;
  for (const Seg& s : segs) {
    const double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
    const double den = dx * ey - dy * ex;
    if (fabs(den) < 1e-9) continue;
    const double qx = s.x1 - sx, qy = s.y1 - sy;
    const double t = (qx * ey - qy * ex) / den;
    const double u = (qx * dy - qy * dx) / den;
    if (t >= 0 && t < best && u >= 0 && u <= 1) best = t;
  }
  return best;
}

static bool segHit(const Seg& a, const Seg& b) {
  auto cr = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
  double rx = a.x2 - a.x1, ry = a.y2 - a.y1, sx = b.x2 - b.x1, sy = b.y2 - b.y1;
  double den = cr(rx, ry, sx, sy);
  if (fabs(den) < 1e-9) return false;
  double t = cr(b.x1 - a.x1, b.y1 - a.y1, sx, sy) / den;
  double u = cr(b.x1 - a.x1, b.y1 - a.y1, rx, ry) / den;
  return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

bool World::collides() const {
  double cx[4], cy[4];
  const double lx[4] = {bodyFront, bodyFront, -bodyRear, -bodyRear};
  const double ly[4] = {halfW, -halfW, -halfW, halfW};
  for (int i = 0; i < 4; i++) samplePose(lx[i], ly[i], cx[i], cy[i]);
  for (int i = 0; i < 4; i++) {
    Seg e{cx[i], cy[i], cx[(i + 1) % 4], cy[(i + 1) % 4]};
    double minx = std::min(e.x1, e.x2) - 1, maxx = std::max(e.x1, e.x2) + 1;
    double miny = std::min(e.y1, e.y2) - 1, maxy = std::max(e.y1, e.y2) + 1;
    for (const Seg& s : segs) {
      if (std::max(s.x1, s.x2) < minx || std::min(s.x1, s.x2) > maxx ||
          std::max(s.y1, s.y2) < miny || std::min(s.y1, s.y2) > maxy) continue;
      if (segHit(e, s)) return true;
    }
  }
  return false;
}

bool World::irOnFinish() const {
  double wx, wy; samplePose(bodyFront, 0, wx, wy);
  const double m = T;   // stay clear of the wall lines
  return wx > finishCx * P + m && wx < (finishCx + 1) * P - m &&
         wy > finishCy * P + m && wy < (finishCy + 1) * P - m;
}

void World::step(uint32_t us) {
  const double dt = us * 1e-6;
  nowUs += us;

  // ---- motors: dead-band, first-order lag ----
  const double duty[2] = {dutyL, dutyR};
  double* v[2] = {&vl, &vr};
  for (int i = 0; i < 2; i++) {
    const double f = fabs(duty[i]) / 255.0;
    double tgt = (duty[i] < 0 ? -1 : 1) * speedMax * std::max(0.0, f - 0.18) / 0.82 * wheelK[i];
    double tau = braking ? 0.02 : (fabs(duty[i]) < 1 ? 0.15 : 0.06);
    if (braking) tgt = 0;
    *v[i] += (tgt - *v[i]) * (1 - exp(-dt / tau));
  }
  const double vc = (vl + vr) / 2, w = (vr - vl) / track;
  x += vc * cos(th + w * dt / 2) * dt;
  y += vc * sin(th + w * dt / 2) * dt;
  th += w * dt;
  omegaDps = w * 180.0 / M_PI;
  distance += fabs(vc) * dt;

  // ---- ToF sensors: each samples every ~22 ms, independently ----
  struct P { double lx, ly, a; };
  const P pose[3] = {{SENSOR_AHEAD_OF_AXLE_MM, halfW, M_PI / 2},   // left
                     {bodyFront, 0, 0},                           // front
                     {SENSOR_AHEAD_OF_AXLE_MM, -halfW, -M_PI / 2}}; // right
  const double now = nowUs * 1e-6;
  for (int i = 0; i < 3; i++) {
    if (now < nextSample[i]) continue;
    nextSample[i] = now + 0.022 + 0.002 * rnd();
    double sx, sy; samplePose(pose[i].lx, pose[i].ly, sx, sy);
    double d = raycast(sx, sy, th + pose[i].a, 1200);
    d += tofSigma * gauss();
    if (d > 1000 || d < 12) d = 8190;                 // out of range / too close
    if (rnd() < glitchProb) d = (rnd() < 0.5) ? 8190 : 40 + 400 * rnd();   // outlier
    mm[i] = (int)std::max(0.0, d);
    ready[i] = true;
  }

  // ---- contacts (rising edge counts once) ----
  const bool c = collides();
  if (c && !inContact) contacts++;
  inContact = c;
}
