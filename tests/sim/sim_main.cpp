// Closed-loop simulation: the real Navigator drives a simulated robot through
// random perfect mazes (left-hand rule is guaranteed to find any cell in them).
//   ./sim            run the standard suite
//   ./sim -v 7       run maze seed 7 verbosely
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdio.h>
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Stepper.h"
#include "Navigator.h"
#include "world.h"

struct Result { bool ok; double secs; int contacts; const char* why; double dist; };

static bool g_trace = false;
static FILE* g_rec = nullptr;
static bool g_rec_first = true;
static const char* g_rec_dump_maze = nullptr;
static std::string g_walls;
static bool g_stress = false;
static Result runMaze(int w, int h, uint64_t seed, int fcx, int fcy, bool verbose,
                      double wheelSkew, double battery) {
  World world;
  g_world = &world;
  world.generateMaze(w, h, seed);
  if (getenv("MM_REAR")) world.bodyRear = atof(getenv("MM_REAR"));
  if (g_rec) {                                  // wall centre-lines for the replay
    g_walls.clear();
    char buf[96];
    for (size_t i = 0; i + 3 < world.segs.size(); i += 4) {   // rectangles: 4 edges each
      const Seg& a = world.segs[i]; const Seg& c = world.segs[i + 2];
      snprintf(buf, sizeof buf, "%s[%.1f,%.1f,%.1f,%.1f]", i ? "," : "", a.x1, a.y1, c.x1, c.y1);
      g_walls += buf;
    }
  }
  world.finishCx = fcx; world.finishCy = fcy;
  world.wheelK[1] = wheelSkew;
  world.speedMax *= battery;             // weaker battery -> slower wheels -> timed turns fall short
  if (g_stress) { world.glitchProb = 0.015; world.tofSigma = 5.0; }
  // start in cell (0,0) with a random placement error, like a human would
  const double sl = g_stress ? 2.0 : 1.0;
  world.placeRobot(0, 0, sl * 12 * (world.rnd() - 0.5), sl * 14 * (world.rnd() - 0.5), sl * 6 * (world.rnd() - 0.5));
  Serial.verbose = verbose || (g_trace && getenv("MM_VERBOSE"));

  Sensors sensors; Motors motors; Stepper stepper;
  Navigator nav(sensors, motors, stepper);
  nav.begin();

  const uint64_t limitUs = 400ULL * 1000000ULL;
  bool pressedOnce = false;
  while (world.nowUs < limitUs) {
    world.startPressed = (world.nowUs > 100000 && world.nowUs < 200000);   // press BOOT once
    if (world.nowUs > 200000) pressedOnce = true;
    (void)pressedOnce;
    const int before = world.contacts;
    world.step(1000);
    nav.update();
    if (g_rec && (world.nowUs / 1000) % 50 == 0) {
      const Ranges rg = sensors.ranges();
      fprintf(g_rec, "%s[%.1f,%.1f,%.4f,%d,%d,%d,%d,%d]", g_rec_first ? "" : ",",
              world.x, world.y, world.th, (int)nav.state(), rg.left, rg.front, rg.right, world.contacts);
      g_rec_first = false;
    }
    if (getenv("MM_FINE") && world.nowUs / 1000 % 30 == 0 && world.nowUs * 1e-6 > atof(getenv("MM_FINE")) && world.nowUs * 1e-6 < atof(getenv("MM_FINE")) + 2.0) {
      const Ranges rg = sensors.ranges();
      double yaw = fmod(world.th * 57.2958, 90.0); if (yaw > 45) yaw -= 90; if (yaw < -45) yaw += 90;
      printf("  t=%6.3f st=%d pos(%.0f,%.0f) yaw %+5.1f  L=%3d F=%3d R=%3d  vl=%.0f vr=%.0f\n", world.nowUs * 1e-6, (int)nav.state(), world.x, world.y, yaw, rg.left, rg.front, rg.right, world.vl, world.vr);
    }
    if (getenv("MM_STATES")) {
      static int last = -1;
      if ((int)nav.state() != last) {
        last = (int)nav.state();
        double yaw = fmod(world.th * 57.2958, 90.0); if (yaw > 45) yaw -= 90; if (yaw < -45) yaw += 90;
        const double cxm = fmod(world.x, world.P) - world.P / 2, cym = fmod(world.y, world.P) - world.P / 2;
        printf("  t=%6.2f -> st=%d cell(%d,%d) off(%+.0f,%+.0f)mm yawErr %+.1f deg (abs %.1f)\n", world.nowUs * 1e-6, last,
               (int)(world.x / world.P), (int)(world.y / world.P), cxm, cym, yaw, world.th * 57.2958);
      }
    }
    if (g_trace && world.contacts > before) {
      Ranges r = sensors.ranges();
      printf("  [contact] t=%.2fs st=%d cell(%d,%d) pos(%.0f,%.0f) yaw=%.1f  L=%d F=%d R=%d\n",
             world.nowUs * 1e-6, (int)nav.state(), (int)(world.x / world.P), (int)(world.y / world.P),
             world.x, world.y, world.th * 57.2958, r.left, r.front, r.right);
    }
    if (nav.state() == State::FINISHED || nav.state() == State::FAULT) break;
  }
  Result r;
  r.ok = nav.state() == State::FINISHED && !strcmp(nav.finishReason(), "end-zone");
  r.secs = nav.finishTimeMs() / 1000.0;
  r.contacts = world.contacts;
  r.dist = world.distance / 1000.0;
  r.why = nav.state() == State::FINISHED ? nav.finishReason()
         : nav.state() == State::FAULT ? "FAULT" : "timeout";
  return r;
}

// Measure what a timed turn really does:  ./sim_run --turn l|r|u [battery]
static int turnTest(char cmd, double battery) {
  World world; g_world = &world;
  world.generateMaze(3, 3, 1);
  world.speedMax *= battery;
  world.placeRobot(1, 1, 0, 0, 0);
  Sensors sensors; Motors motors; Stepper stepper;
  Navigator nav(sensors, motors, stepper);
  nav.begin();
  for (int i = 0; i < 300; i++) { world.step(1000); nav.update(); }
  const double th0 = world.th;
  static char buf[2] = {0, 0}; buf[0] = cmd; Serial.feed(buf);
  for (int i = 0; i < 3000; i++) { world.step(1000); nav.update(); }
  double d = (world.th - th0) * 180.0 / M_PI;
  while (d > 270) d -= 360; while (d < -270) d += 360;
  printf("battery x%.2f  turn '%c' -> %.1f degrees\n", battery, cmd, d);
  return 0;
}

int main(int argc, char** argv) {
  if (argc >= 3 && !strcmp(argv[1], "--turn")) return turnTest(argv[2][0], argc > 3 ? atof(argv[3]) : 1.0);
  if (argc >= 8 && !strcmp(argv[1], "--record")) {
    const int seed = atoi(argv[2]), w = atoi(argv[3]), h = atoi(argv[4]), fcx = atoi(argv[5]), fcy = atoi(argv[6]);
    g_rec = fopen(argv[7], "w");
    if (!g_rec) return 2;
    g_rec_dump_maze = argv[7];
    fprintf(g_rec, "{\"frames\":[");
    Result r = runMaze(w, h, (uint64_t)seed, fcx, fcy, false, 1.0, 1.0);
    fprintf(g_rec, "],\"ok\":%d,\"secs\":%.2f,\"contacts\":%d,\"dist\":%.2f,", r.ok ? 1 : 0, r.secs, r.contacts, r.dist);
    fprintf(g_rec, "\"pitch\":212,\"wall\":12,\"w\":%d,\"h\":%d,\"finish\":[%d,%d],\"walls\":[", w, h, fcx, fcy);
    // walls were dumped into g_walls by runMaze
    fputs(g_walls.c_str(), g_rec);
    fprintf(g_rec, "]}\n");
    fclose(g_rec);
    printf("%s: %s %.1f s, %d contacts, %.1f m\n", argv[7], r.why, r.secs, r.contacts, r.dist);
    return 0;
  }
  if (argc >= 3 && !strcmp(argv[1], "-t")) {
    g_trace = true; g_stress = getenv("MM_STRESS") != nullptr; Serial.verbose = getenv("MM_VERBOSE") != nullptr;
    Result r = runMaze(9, 9, atoi(argv[2]), atoi(argv[2]) % 2 ? 8 : 4, atoi(argv[2]) % 3 == 0 ? 4 : 8, false,
                       (double[]){0.97, 1.00, 1.04}[atoi(argv[2]) % 3], (double[]){1.00, 0.94, 1.05}[(atoi(argv[2]) / 3) % 3]);
    printf("-> %s, %.1f s, %d contacts, %.1f m\n", r.why, r.secs, r.contacts, r.dist);
    return 0;
  }
  if (argc >= 3 && !strcmp(argv[1], "-v")) {
    Result r = runMaze(9, 9, atoi(argv[2]), 8, 8, true, 0.97, 1.0);
    printf("-> %s, %.1f s, %d contacts\n", r.why, r.secs, r.contacts);
    return r.ok ? 0 : 1;
  }
  if (argc >= 2 && !strcmp(argv[1], "--stress")) g_stress = true;
  printf("Micromouse closed-loop simulation (real Navigator code, 9x9 perfect mazes)%s\n",
         g_stress ? "  [STRESS: 4x glitches, 2x noise, +-7 pct wheel skew, battery +-10 pct]" : "");
  printf("%-6s %-9s %-8s %-9s %-9s %s\n", "seed", "result", "time(s)", "dist(m)", "contacts", "conditions");
  int fail = 0, total = 0, clean = 0;
  const double skews[3] = {g_stress ? 0.93 : 0.97, 1.00, g_stress ? 1.07 : 1.04};
  const double batts[3] = {1.00, g_stress ? 0.90 : 0.94, g_stress ? 1.10 : 1.05};
  for (int seed = 1; seed <= 24; seed++) {
    const double sk = skews[seed % 3], bi = batts[(seed / 3) % 3];
    int fcx = (seed % 2) ? 8 : 4, fcy = (seed % 3 == 0) ? 4 : 8;
    Result r = runMaze(9, 9, seed, fcx, fcy, false, sk, bi);
    total++;
    if (!r.ok) fail++;
    if (r.ok && r.contacts == 0) clean++;
    printf("%-6d %-9s %-8.1f %-9.1f %-9d finish(%d,%d) skew %.2f battery x%.2f\n", seed,
           r.ok ? "FINISH" : r.why, r.secs, r.dist, r.contacts, fcx, fcy, sk, bi);
  }
  printf("\n%d/%d reached the end-zone, %d/%d with zero wall contacts\n", total - fail, total, clean, total);
  return fail ? 1 : 0;
}
