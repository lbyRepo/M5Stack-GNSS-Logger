/*
 * test_ins.cpp - host unit tests for the 15-state INS / GNSS EKF.
 *
 * ins_ekf.cpp has no Arduino dependencies, so this file simply includes it and
 * drives it directly. That also gives the tests access to the internals in the
 * anonymous namespace (geodesy, quaternions) which have no public seam.
 *
 * Build and run with run_tests.bat (MSVC).
 */

#include <math.h>
#include <stdio.h>

#include "../../sensorLogger/ins_ekf.cpp"

// ---------------------------------------------------------------------------
// Tiny test harness
// ---------------------------------------------------------------------------

static int g_pass = 0;
static int g_fail = 0;

static void check(bool ok, const char* name) {
  if (ok) {
    ++g_pass;
  } else {
    ++g_fail;
    printf("  FAIL: %s\n", name);
  }
}

static void checkNear(double value, double expect, double tol, const char* name) {
  if (fabs(value - expect) <= tol) {
    ++g_pass;
  } else {
    ++g_fail;
    printf("  FAIL: %s (got %.9g, expected %.9g +/- %.3g)\n", name, value,
           expect, tol);
  }
}

static void checkLess(double value, double limit, const char* name) {
  if (value < limit) {
    ++g_pass;
  } else {
    ++g_fail;
    printf("  FAIL: %s (got %.9g, expected < %.9g)\n", name, value, limit);
  }
}

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

static const double TEST_LAT = 1.3;      // rad, ~74 deg
static const double TEST_LON = 1.8;      // rad
static const double TEST_HGT = 50.0;     // m

// Specific force for a device held at (roll, pitch) with no linear motion.
// At rest the accelerometer measures specific force, which points up, so a
// level device reads (0, 0, -g) in body FRD.
static void makeStaticSample(float roll, float pitch, const float gyroBias[3],
                             uint32_t t_us, ins::ImuSample& s) {
  float q[4];
  ins::quatFromEuler(roll, pitch, 0.0f, q);
  float C[3][3];
  ins::quatToDcm(q, C);

  const float g = ins::normalGravity(TEST_LAT, TEST_HGT);
  const float fn[3] = {0.0f, 0.0f, -g};
  for (int i = 0; i < 3; ++i) {
    // Row i of C_n^b is column i of C_b^n.
    s.acc[i] = C[0][i] * fn[0] + C[1][i] * fn[1] + C[2][i] * fn[2];
    s.gyro[i] = gyroBias[i];
  }
  s.t_us = t_us;
}

static ins::GnssFix makeFix(uint32_t t_ms, double lat, double lon, double h,
                            float vn, float ve, bool velValid) {
  ins::GnssFix f;
  f.t_ms = t_ms;
  f.lat_rad = lat;
  f.lon_rad = lon;
  f.h_m = h;
  f.vn = vn;
  f.ve = ve;
  f.velocity_valid = velValid;
  return f;
}

// Local metres from a reference, using the same radii the filter uses.
static void toLocalMetres(double lat, double lon, double lat0, double lon0,
                          double h, double& north, double& east) {
  double rm, rn;
  ins::earthRadii(lat0, rm, rn);
  north = (lat - lat0) * (rm + h);
  east = (lon - lon0) * (rn + h) * cos(lat0);
}

// Runs alignment from a fresh reset and returns the output afterwards.
static void alignAt(float roll, float pitch, const float gyroBias[3],
                    ins::Output& out, uint32_t& tUs) {
  ins::reset();
  tUs = 1000000u;
  ins::updateGnss(makeFix(tUs / 1000u, TEST_LAT, TEST_LON, TEST_HGT, 0.0f, 0.0f,
                          false));

  ins::ImuSample s;
  for (int i = 0; i < 260; ++i) {
    tUs += 10000u;
    makeStaticSample(roll, pitch, gyroBias, tUs, s);
    ins::updateImu(s);
  }
  ins::getOutput(out);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void testQuaternions() {
  printf("quaternions\n");

  float q[4], C[3][3], roll, pitch, yaw;

  ins::quatFromEuler(0.0f, 0.0f, 0.0f, q);
  ins::quatToDcm(q, C);
  checkNear(C[0][0], 1.0, 1e-6, "identity DCM");
  checkNear(C[1][1], 1.0, 1e-6, "identity DCM yy");
  checkNear(C[2][2], 1.0, 1e-6, "identity DCM zz");

  // Round trip through Euler angles and back.
  ins::quatFromEuler(0.3f, -0.2f, 1.1f, q);
  ins::quatToEuler(q, roll, pitch, yaw);
  checkNear(roll, 0.3, 1e-5, "roll round trip");
  checkNear(pitch, -0.2, 1e-5, "pitch round trip");
  checkNear(yaw, 1.1, 1e-5, "yaw round trip");

  // Yaw is clockwise from north: +90 deg must take body X to nav east.
  ins::quatFromEuler(0.0f, 0.0f, 1.5707963268f, q);
  ins::quatToDcm(q, C);
  const float bodyX[3] = {1.0f, 0.0f, 0.0f};
  float navX[3];
  for (int i = 0; i < 3; ++i) {
    navX[i] = C[i][0] * bodyX[0] + C[i][1] * bodyX[1] + C[i][2] * bodyX[2];
  }
  checkNear(navX[0], 0.0, 1e-6, "yaw +90 maps body X to north component 0");
  checkNear(navX[1], 1.0, 1e-6, "yaw +90 maps body X to east");

  // Roll about north: the right-hand rule takes body Y (starboard) toward
  // down, i.e. a positive roll is right-wing-down.
  ins::quatFromEuler(0.5f, 0.0f, 0.0f, q);
  ins::quatToDcm(q, C);
  float navY[3];
  for (int i = 0; i < 3; ++i) navY[i] = C[i][1];
  check(navY[2] > 0.0f, "positive roll tips body Y downward");
  checkNear(navY[2], sinf(0.5f), 1e-5, "roll magnitude maps to sin(roll)");

  // Normalisation must survive a large rotation vector.
  float rv[3] = {3.0f, -2.0f, 1.0f};
  ins::quatFromRotVec(rv, q);
  const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  checkNear(n, 1.0, 1e-6, "rotation vector quaternion is normalised");
}

static void testGeodesy() {
  printf("geodesy\n");

  double rm, rn;
  ins::earthRadii(0.0, rm, rn);
  checkNear(rn, 6378137.0, 1e-3, "transverse radius at the equator");
  checkNear(rm, 6335439.3, 1.0, "meridian radius at the equator");

  ins::earthRadii(1.5707963267948966, rm, rn);
  checkNear(rn, 6399593.6, 1.0, "transverse radius at the pole");
  checkNear(rm, rn, 1e-3, "radii coincide at the pole");

  checkNear(ins::normalGravity(0.0, 0.0), 9.780327, 1e-5,
            "normal gravity at the equator");
  checkNear(ins::normalGravity(1.5707963267948966, 0.0), 9.832186, 1e-4,
            "normal gravity at the pole");
  // Gravity must fall with height at about 3.086 um/s^2 per metre.
  const double g0 = ins::normalGravity(0.7, 0.0);
  const double g1 = ins::normalGravity(0.7, 1000.0);
  checkNear((g0 - g1) / 1000.0, 3.086e-6, 1e-8, "free-air gravity gradient");
}

static void testAlignmentLevel() {
  printf("alignment (level, known gyro bias)\n");

  const float bias[3] = {0.010f, -0.004f, 0.002f};
  ins::Output out;
  uint32_t tUs = 0;
  alignAt(0.0f, 0.0f, bias, out, tUs);

  check(out.mode == ins::Mode::Run, "alignment completes");
  checkNear(out.roll_deg, 0.0, 0.05, "level roll");
  checkNear(out.pitch_deg, 0.0, 0.05, "level pitch");
  checkNear(out.bg_dps[0], 0.010 * 57.29577951, 0.01, "gyro bias x estimated");
  checkNear(out.bg_dps[1], -0.004 * 57.29577951, 0.01, "gyro bias y estimated");
  checkNear(out.bg_dps[2], 0.002 * 57.29577951, 0.01, "gyro bias z estimated");
  check(out.healthy, "filter healthy after alignment");

  // The published alignment means are what the axis mapping gets verified
  // against on hardware, so they must reflect the (0, 0, -g) convention.
  checkNear(out.align_acc_mean[0], 0.0, 0.01, "align mean accel x");
  checkNear(out.align_acc_mean[1], 0.0, 0.01, "align mean accel y");
  checkLess(out.align_acc_mean[2], -9.7, "align mean accel z is -g");
}

static void testAlignmentTilted() {
  printf("alignment (tilted)\n");

  const float bias[3] = {0.0f, 0.0f, 0.0f};
  const float roll = 0.17453293f;   // 10 deg
  const float pitch = -0.08726646f; // -5 deg

  ins::Output out;
  uint32_t tUs = 0;
  alignAt(roll, pitch, bias, out, tUs);

  check(out.mode == ins::Mode::Run, "tilted alignment completes");
  checkNear(out.roll_deg, 10.0, 0.05, "tilted roll recovered");
  checkNear(out.pitch_deg, -5.0, 0.05, "tilted pitch recovered");
}

static void testStationaryHold() {
  printf("stationary hold with GNSS aiding (60 s)\n");

  const float bias[3] = {0.003f, 0.001f, -0.002f};
  ins::Output out;
  uint32_t tUs = 0;
  alignAt(0.0f, 0.0f, bias, out, tUs);

  ins::ImuSample s;
  uint32_t tMs = tUs / 1000u;
  for (int i = 0; i < 6000; ++i) {  // 60 s at 100 Hz
    tUs += 10000u;
    tMs += 10u;
    makeStaticSample(0.0f, 0.0f, bias, tUs, s);
    ins::updateImu(s);
    if (i % 100 == 0) {
      ins::updateGnss(makeFix(tMs, TEST_LAT, TEST_LON, TEST_HGT, 0.0f, 0.0f,
                              true));
    }
  }
  ins::getOutput(out);

  double north, east;
  toLocalMetres(out.lat_deg * ins::RAD, out.lon_deg * ins::RAD, TEST_LAT,
                TEST_LON, TEST_HGT, north, east);
  checkNear(north, 0.0, 0.5, "stationary north drift < 0.5 m");
  checkNear(east, 0.0, 0.5, "stationary east drift < 0.5 m");
  checkLess(fabs(out.vn), 0.05, "stationary vN near zero");
  checkLess(fabs(out.ve), 0.05, "stationary vE near zero");
  check(out.healthy, "covariance stays healthy over 6000 steps");
}

static void testCovarianceHealth() {
  printf("covariance health\n");

  const float bias[3] = {0.0f, 0.0f, 0.0f};
  ins::Output out;
  uint32_t tUs = 0;
  alignAt(0.0f, 0.0f, bias, out, tUs);

  ins::ImuSample s;
  uint32_t tMs = tUs / 1000u;
  bool symmetric = true;
  for (int i = 0; i < 2000; ++i) {
    tUs += 10000u;
    tMs += 10u;
    makeStaticSample(0.01f, 0.01f, bias, tUs, s);
    ins::updateImu(s);
    if (i % 500 == 0) {
      ins::updateGnss(makeFix(tMs, TEST_LAT, TEST_LON, TEST_HGT, 0.0f, 0.0f,
                              true));
    }
    for (int a = 0; a < 15 && symmetric; ++a) {
      for (int b = 0; b < 15; ++b) {
        const float p = ins::s_P[a][b];
        const float q = ins::s_P[b][a];
        if (fabsf(p - q) > 1e-3f * (1.0f + fabsf(p))) symmetric = false;
        if (!(p == p)) symmetric = false;  // NaN
      }
    }
  }
  check(symmetric, "P stays symmetric and free of NaN");
  ins::getOutput(out);
  check(out.healthy, "filter reports healthy");
}

static void testGnssPullIn() {
  printf("GNSS position aiding\n");

  const float bias[3] = {0.0f, 0.0f, 0.0f};
  ins::Output out;
  uint32_t tUs = 0;
  alignAt(0.0f, 0.0f, bias, out, tUs);

  // Truth is 20 m north and 15 m east of where the filter was seeded.
  double rm, rn;
  ins::earthRadii(TEST_LAT, rm, rn);
  const double latTrue = TEST_LAT + 20.0 / (rm + TEST_HGT);
  const double lonTrue = TEST_LON + 15.0 / ((rn + TEST_HGT) * cos(TEST_LAT));

  ins::ImuSample s;
  uint32_t tMs = tUs / 1000u;
  for (int i = 0; i < 1000; ++i) {
    tUs += 10000u;
    tMs += 10u;
    makeStaticSample(0.0f, 0.0f, bias, tUs, s);
    ins::updateImu(s);
    if (i % 100 == 0) {
      ins::updateGnss(makeFix(tMs, latTrue, lonTrue, TEST_HGT, 0.0f, 0.0f, true));
    }
  }
  ins::getOutput(out);

  double north, east;
  toLocalMetres(out.lat_deg * ins::RAD, out.lon_deg * ins::RAD, TEST_LAT,
                TEST_LON, TEST_HGT, north, east);
  checkNear(north, 20.0, 3.0, "filter pulled in to the GNSS north offset");
  checkNear(east, 15.0, 3.0, "filter pulled in to the GNSS east offset");
}

static void testDuplicateTimestamps() {
  printf("timestamp handling\n");

  const float bias[3] = {0.0f, 0.0f, 0.0f};
  ins::Output out;
  uint32_t tUs = 0;
  alignAt(0.0f, 0.0f, bias, out, tUs);

  ins::ImuSample s;
  makeStaticSample(0.0f, 0.0f, bias, tUs, s);
  const float vnBefore = out.vn;
  for (int i = 0; i < 10; ++i) ins::updateImu(s);  // same timestamp, ten times
  ins::getOutput(out);
  checkNear(out.vn, vnBefore, 1e-6, "duplicate timestamps are ignored");
}

static void testLatencyCompensation() {
  printf("GNSS latency compensation (10 m/s north, 150 ms lag)\n");

  // Two identical runs, differing only in COMPENSATE_GNSS_LATENCY. The
  // constant is compile-time, so the comparison is done by measuring the
  // residual against a truth trajectory that the filter is not told about.
  const float bias[3] = {0.0f, 0.0f, 0.0f};
  ins::Output out;
  uint32_t tUs = 0;
  alignAt(0.0f, 0.0f, bias, out, tUs);

  double rm, rn;
  ins::earthRadii(TEST_LAT, rm, rn);

  ins::ImuSample s;
  uint32_t tMs = tUs / 1000u;
  const float speed = 10.0f;  // m/s north
  double latTrue = TEST_LAT;
  double lonTrue = TEST_LON;

  // Start the filter moving: give it true velocity through the GNSS aid.
  for (int i = 0; i < 3000; ++i) {  // 30 s
    tUs += 10000u;
    tMs += 10u;
    latTrue += (double)(speed * 0.01f) / (rm + TEST_HGT);

    makeStaticSample(0.0f, 0.0f, bias, tUs, s);
    ins::updateImu(s);

    if (i % 100 == 0) {
      // The fix describes where the platform was GNSS_LATENCY_MS ago.
      const double lagM = speed * (ins::GNSS_LATENCY_MS * 0.001f);
      const double latFix = latTrue - lagM / (rm + TEST_HGT);
      ins::updateGnss(makeFix(tMs, latFix, lonTrue, TEST_HGT, speed, 0.0f, true));
    }
  }
  ins::getOutput(out);

  double north, east;
  toLocalMetres(out.lat_deg * ins::RAD, out.lon_deg * ins::RAD, latTrue,
                lonTrue, TEST_HGT, north, east);
  const double residual = sqrt(north * north + east * east);
  printf("  residual with compensation = %.3f m\n", residual);
  checkLess(residual, 3.0, "lagged fixes tracked within 3 m");
}

// Validates the sign of the attitude correction through the real observability
// path rather than by asserting a convention: with the filter aligned level but
// the platform actually pitched, the gravity leak into the horizontal channel
// must be visible to GNSS aiding, and the filter must drive pitch towards the
// true value. A wrong sign makes the loop positive-feedback and it runs away.
static void testAttitudeObservability() {
  printf("attitude observability (2 deg pitch error)\n");

  const float bias[3] = {0.0f, 0.0f, 0.0f};
  const float truePitch = 0.034906585f;  // 2 deg

  ins::Output out;
  uint32_t tUs = 0;
  alignAt(0.0f, 0.0f, bias, out, tUs);  // filter believes it is level
  checkNear(out.pitch_deg, 0.0, 0.05, "filter starts level (deliberately wrong)");

  // The platform is in fact pitched by 2 deg and stationary.
  ins::ImuSample s;
  uint32_t tMs = tUs / 1000u;
  for (int i = 0; i < 4000; ++i) {  // 40 s
    tUs += 10000u;
    tMs += 10u;
    makeStaticSample(0.0f, truePitch, bias, tUs, s);
    ins::updateImu(s);
    if (i % 100 == 0) {
      ins::updateGnss(makeFix(tMs, TEST_LAT, TEST_LON, TEST_HGT, 0.0f, 0.0f,
                              true));
    }
  }
  ins::getOutput(out);

  printf("  converged pitch = %.3f deg (truth %.3f)\n", out.pitch_deg,
         truePitch * 57.29577951);
  checkLess(fabs(out.pitch_deg - 2.0), 0.5,
            "pitch converges to the true value, not away from it");

  double north, east;
  toLocalMetres(out.lat_deg * ins::RAD, out.lon_deg * ins::RAD, TEST_LAT,
                TEST_LON, TEST_HGT, north, east);
  checkLess(sqrt(north * north + east * east), 3.0,
            "position stays bounded while the attitude error is trimmed");
}

// ---------------------------------------------------------------------------

int main() {
  printf("=== INS / EKF host tests ===\n\n");
  testQuaternions();
  testGeodesy();
  testAttitudeObservability();
  testAlignmentLevel();
  testAlignmentTilted();
  testStationaryHold();
  testCovarianceHealth();
  testGnssPullIn();
  testDuplicateTimestamps();
  testLatencyCompensation();
  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
