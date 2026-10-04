/*
 * ins_ekf.cpp - 15-state INS / GNSS Extended Kalman Filter
 *
 * See ins_ekf.h for frames, conventions and the state definition.
 *
 * Layout:
 *   1. constants and linear algebra
 *   2. geodesy (radii of curvature, normal gravity)
 *   3. quaternion utilities
 *   4. nominal-state mechanisation (Titterton)
 *   5. error-state dynamics and the measurement update
 *   6. history ring for GNSS latency compensation
 *   7. stationary detection, alignment, public API
 */

#include "ins_ekf.h"

#include <math.h>
#include <string.h>

namespace ins {
namespace {

// ---------------------------------------------------------------------------
// 1. Constants and linear algebra
// ---------------------------------------------------------------------------

constexpr int NS = 15;  // number of error states
constexpr int MAXM = 5; // largest measurement block (lat, lon, h, vN, vE)

// WGS-84 and Earth
constexpr double WGS_A  = 6378137.0;
constexpr double WGS_F  = 1.0 / 298.257223563;
constexpr double WGS_E2 = WGS_F * (2.0 - WGS_F);
constexpr double EARTH_RATE = 7.2921151467e-5;  // rad/s

constexpr float DEG = 57.29577951308232f;
constexpr float RAD = 0.017453292519943295f;
constexpr float TWO_PI_F = 6.28318530717959f;
constexpr float PI_F = 3.14159265358979f;

// GNSS latency history: 128 samples at 100 Hz is ~1.28 s of margin.
constexpr int HIST_CAP = 128;

// ZUPT and magnetometer updates are strongly persistent constraints, so they
// do not need to run at the full propagation rate. Rate limiting them keeps
// the covariance work off the critical path.
constexpr uint32_t AUX_UPDATE_PERIOD_MS = 100;

void mulNN(const float A[NS][NS], const float B[NS][NS], float C[NS][NS]) {
  for (int i = 0; i < NS; ++i) {
    for (int j = 0; j < NS; ++j) {
      float s = 0.0f;
      for (int k = 0; k < NS; ++k) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
}

// C = A * B^T
void mulNBt(const float A[NS][NS], const float B[NS][NS], float C[NS][NS]) {
  for (int i = 0; i < NS; ++i) {
    for (int j = 0; j < NS; ++j) {
      float s = 0.0f;
      for (int k = 0; k < NS; ++k) s += A[i][k] * B[j][k];
      C[i][j] = s;
    }
  }
}

void symmetrise(float P[NS][NS]) {
  for (int i = 0; i < NS; ++i) {
    for (int j = i + 1; j < NS; ++j) {
      const float v = 0.5f * (P[i][j] + P[j][i]);
      P[i][j] = v;
      P[j][i] = v;
    }
  }
}

// Cholesky factorisation of a symmetric positive-definite m x m matrix.
bool cholFactor(const float S[MAXM][MAXM], int m, float L[MAXM][MAXM]) {
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j <= i; ++j) {
      float sum = S[i][j];
      for (int k = 0; k < j; ++k) sum -= L[i][k] * L[j][k];
      if (i == j) {
        if (!(sum > 1e-20f)) return false;  // not positive definite
        L[i][j] = sqrtf(sum);
      } else {
        L[i][j] = sum / L[j][j];
      }
    }
    for (int j = i + 1; j < m; ++j) L[i][j] = 0.0f;
  }
  return true;
}

void cholSolve(const float L[MAXM][MAXM], int m, const float b[MAXM],
               float x[MAXM]) {
  float y[MAXM];
  for (int i = 0; i < m; ++i) {
    float s = b[i];
    for (int k = 0; k < i; ++k) s -= L[i][k] * y[k];
    y[i] = s / L[i][i];
  }
  for (int i = m - 1; i >= 0; --i) {
    float s = y[i];
    for (int k = i + 1; k < m; ++k) s -= L[k][i] * x[k];
    x[i] = s / L[i][i];
  }
}

bool invertSPD(const float S[MAXM][MAXM], int m, float Sinv[MAXM][MAXM]) {
  float L[MAXM][MAXM];
  memset(L, 0, sizeof(L));
  if (!cholFactor(S, m, L)) return false;

  for (int c = 0; c < m; ++c) {
    float e[MAXM];
    float x[MAXM];
    for (int i = 0; i < m; ++i) e[i] = 0.0f;
    e[c] = 1.0f;
    cholSolve(L, m, e, x);
    for (int r = 0; r < m; ++r) Sinv[r][c] = x[r];
  }
  return true;
}

// ---------------------------------------------------------------------------
// 2. Geodesy
// ---------------------------------------------------------------------------

void earthRadii(double lat, double& rm, double& rn) {
  const double s = sin(lat);
  const double w = 1.0 - WGS_E2 * s * s;
  const double sw = sqrt(w);
  rm = WGS_A * (1.0 - WGS_E2) / (w * sw);  // meridian radius of curvature
  rn = WGS_A / sw;                          // transverse radius
}

// Somigliana normal gravity plus the free-air height correction.
float normalGravity(double lat, double h) {
  const double s = sin(lat);
  const double s2 = s * s;
  const double t = sin(2.0 * lat);
  const double t2 = t * t;
  return (float)(9.780327 * (1.0 + 0.0053024 * s2 - 0.0000058 * t2) -
                 3.086e-6 * h);
}

// ---------------------------------------------------------------------------
// 3. Quaternions (Hamilton, scalar first: w, x, y, z)
// ---------------------------------------------------------------------------

void quatMul(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void quatNormalise(float q[4]) {
  const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(n > 1e-12f)) {
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    return;
  }
  const float inv = 1.0f / n;
  for (int i = 0; i < 4; ++i) q[i] *= inv;
}

// Exponential map: rotation of |rv| radians about the axis rv.
void quatFromRotVec(const float rv[3], float q[4]) {
  const float angle = sqrtf(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
  if (angle < 1e-9f) {
    q[0] = 1.0f;
    q[1] = 0.5f * rv[0];
    q[2] = 0.5f * rv[1];
    q[3] = 0.5f * rv[2];
    quatNormalise(q);
    return;
  }
  const float h = 0.5f * angle;
  const float s = sinf(h) / angle;
  q[0] = cosf(h);
  q[1] = rv[0] * s;
  q[2] = rv[1] * s;
  q[3] = rv[2] * s;
}

// Rotation matrix body -> nav.
void quatToDcm(const float q[4], float C[3][3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  C[0][0] = 1.0f - 2.0f * (y * y + z * z);
  C[0][1] = 2.0f * (x * y - w * z);
  C[0][2] = 2.0f * (x * z + w * y);
  C[1][0] = 2.0f * (x * y + w * z);
  C[1][1] = 1.0f - 2.0f * (x * x + z * z);
  C[1][2] = 2.0f * (y * z - w * x);
  C[2][0] = 2.0f * (x * z - w * y);
  C[2][1] = 2.0f * (y * z + w * x);
  C[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

// Applies a nav-frame (psi-angle) error correction: q <- dq (x) q.
//
// The sign here is tied to the sign convention of the -[f^n x] coupling in F.
// It is validated dynamically by testAttitudeObservability, which induces a
// known attitude error and checks that GNSS aiding pulls it back to truth.
// Flipping it makes the loop positive-feedback and the filter diverges.
void quatApplyNavError(float q[4], const float dpsi[3]) {
  float dq[4];
  quatFromRotVec(dpsi, dq);
  float out[4];
  quatMul(dq, q, out);
  for (int i = 0; i < 4; ++i) q[i] = out[i];
  quatNormalise(q);
}

// ZYX Euler angles from a body -> nav DCM.
void quatToEuler(const float q[4], float& roll, float& pitch, float& yaw) {
  float C[3][3];
  quatToDcm(q, C);
  float sp = -C[2][0];
  if (sp > 1.0f) sp = 1.0f;
  if (sp < -1.0f) sp = -1.0f;
  pitch = asinf(sp);
  roll = atan2f(C[2][1], C[2][2]);
  yaw = atan2f(C[1][0], C[0][0]);
}

void quatFromEuler(float roll, float pitch, float yaw, float q[4]) {
  const float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
  const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
  const float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
  quatNormalise(q);
}

float wrapPi(float a) {
  while (a > PI_F) a -= TWO_PI_F;
  while (a < -PI_F) a += TWO_PI_F;
  return a;
}

// ---------------------------------------------------------------------------
// 4. Nominal state and the Titterton mechanisation
// ---------------------------------------------------------------------------

struct Nominal {
  double lat;      // rad
  double lon;      // rad
  double h;        // m, ellipsoidal
  float vn, ve, vd;
  float q[4];      // body -> nav
  float ba[3];     // accel bias, FRD
  float bg[3];     // gyro bias, FRD
  float aPrev[3];  // nav-frame acceleration from the previous step
  float fNav[3];   // nav-frame specific force, for the error model
};

struct ImuSampleMs {
  uint32_t t_ms;
  float acc[3];
  float gyro[3];
};

// One mechanisation step. A pure function of (state, sample, dt), so it can be
// replayed unchanged when compensating GNSS latency.
void propagate(Nominal& n, const ImuSampleMs& s, float dt) {
  if (!(dt > 0.0f)) return;

  double rm, rn;
  earthRadii(n.lat, rm, rn);

  const double sinLat = sin(n.lat);
  const double cosLat = cos(n.lat);
  const double tanLat = sinLat / cosLat;
  const double hRm = rm + n.h;
  const double hRn = rn + n.h;

  // Earth rate and transport rate in the nav frame.
  const float wie[3] = {(float)(EARTH_RATE * cosLat), 0.0f,
                        (float)(-EARTH_RATE * sinLat)};
  const float wen[3] = {(float)(n.ve / hRn), (float)(-n.vn / hRm),
                        (float)(-n.ve * tanLat / hRn)};
  const float win[3] = {wie[0] + wen[0], wie[1] + wen[1], wie[2] + wen[2]};

  // Attitude: omega_nb^b = omega_ib^b - bg - C_n^b * omega_in^n
  float C[3][3];
  quatToDcm(n.q, C);
  const float wib[3] = {s.gyro[0] - n.bg[0], s.gyro[1] - n.bg[1],
                        s.gyro[2] - n.bg[2]};
  float wnb[3];
  for (int i = 0; i < 3; ++i) {
    // C_n^b is the transpose of C_b^n, so row i of C_n^b is column i of C_b^n.
    const float cnb = C[0][i] * win[0] + C[1][i] * win[1] + C[2][i] * win[2];
    wnb[i] = wib[i] - cnb;
  }
  float dq[4];
  const float rv[3] = {wnb[0] * dt, wnb[1] * dt, wnb[2] * dt};
  quatFromRotVec(rv, dq);
  float qNew[4];
  quatMul(n.q, dq, qNew);  // body-frame increment, applied on the right
  for (int i = 0; i < 4; ++i) n.q[i] = qNew[i];
  quatNormalise(n.q);

  // Specific force in the nav frame, using the updated attitude.
  quatToDcm(n.q, C);
  const float fb[3] = {s.acc[0] - n.ba[0], s.acc[1] - n.ba[1],
                       s.acc[2] - n.ba[2]};
  float fn[3];
  for (int i = 0; i < 3; ++i) {
    fn[i] = C[i][0] * fb[0] + C[i][1] * fb[1] + C[i][2] * fb[2];
  }
  for (int i = 0; i < 3; ++i) n.fNav[i] = fn[i];

  // a^n = f^n - (2 w_ie^n + w_en^n) x v^n + g^n
  const float cor[3] = {(float)(2.0 * wie[0] + wen[0]),
                        (float)(2.0 * wie[1] + wen[1]),
                        (float)(2.0 * wie[2] + wen[2])};
  const float v[3] = {n.vn, n.ve, n.vd};
  const float g = normalGravity(n.lat, n.h);
  float a[3];
  a[0] = fn[0] - (cor[1] * v[2] - cor[2] * v[1]);
  a[1] = fn[1] - (cor[2] * v[0] - cor[0] * v[2]);
  a[2] = fn[2] - (cor[0] * v[1] - cor[1] * v[0]) + g;

  // Trapezoidal velocity update using the previous step's acceleration.
  float vNew[3];
  for (int i = 0; i < 3; ++i) {
    vNew[i] = v[i] + 0.5f * (n.aPrev[i] + a[i]) * dt;
    n.aPrev[i] = a[i];
  }

  // Trapezoidal position update. lat/lon are double on purpose: one metre is
  // ~1.6e-7 rad, which is below single-precision resolution at these values,
  // so a float position would silently discard the Kalman correction.
  const double halfDt = 0.5 * (double)dt;
  n.lat += halfDt * (double)(v[0] + vNew[0]) / hRm;
  n.lon += halfDt * (double)(v[1] + vNew[1]) / (hRn * cosLat);
  n.h -= halfDt * (double)(v[2] + vNew[2]);

  n.vn = vNew[0];
  n.ve = vNew[1];
  n.vd = vNew[2];
}

// ---------------------------------------------------------------------------
// 5. Filter state, error dynamics, measurement update
// ---------------------------------------------------------------------------

struct HistEntry {
  uint32_t t_ms;
  ImuSampleMs imu;
  Nominal nom;
};

// File-static rather than stack locals: the 15x15 covariance work would
// otherwise dominate the owning task's stack.
float s_P[NS][NS];
Nominal s_nom;
Mode s_mode = Mode::WaitGnss;

uint32_t s_lastImuMs = 0;
bool s_haveImu = false;
float s_lastDtMs = 0.0f;
uint32_t s_lastAuxMs = 0;

// Alignment accumulation.
float s_alignAccSum[3];
float s_alignGyroSum[3];
uint32_t s_alignCount = 0;
float s_alignElapsedS = 0.0f;
float s_alignAccMean[3];
float s_alignGyroMean[3];

// Stationary detection.
bool s_stationary = false;
float s_stillTimer = 0.0f;
float s_moveTimer = 0.0f;
float s_lastGnssSpeed = 0.0f;
bool s_haveGnssSpeed = false;
bool s_velocityInit = false;

// Magnetometer.
MagSample s_mag;
float s_magNormEstimate = 0.0f;
bool s_magNormValid = false;

// History ring.
HistEntry s_hist[HIST_CAP];
int s_histHead = -1;
int s_histCount = 0;

// Diagnostics.
uint32_t s_imuCountWindow = 0;
uint32_t s_imuHz = 0;
uint32_t s_rateWindowStartMs = 0;
uint32_t s_gnssCount = 0;
uint32_t s_gnssRejected = 0;

void skew(const float v[3], float S[3][3]) {
  S[0][0] = 0.0f;  S[0][1] = -v[2]; S[0][2] = v[1];
  S[1][0] = v[2];  S[1][1] = 0.0f;  S[1][2] = -v[0];
  S[2][0] = -v[1]; S[2][1] = v[0];  S[2][2] = 0.0f;
}

// F = I + Fc*dt, then P <- F P F^T + Qd.
void propagateCovariance(const Nominal& n, float dt) {
  if (dt > MAX_DT_S) return;  // do not integrate a stale, oversized step

  double rm, rn;
  earthRadii(n.lat, rm, rn);
  const double sinLat = sin(n.lat);
  const double cosLat = cos(n.lat);
  const double tanLat = sinLat / cosLat;
  const double hRm = rm + n.h;
  const double hRn = rn + n.h;

  const float wie[3] = {(float)(EARTH_RATE * cosLat), 0.0f,
                        (float)(-EARTH_RATE * sinLat)};
  const float wen[3] = {(float)(n.ve / hRn), (float)(-n.vn / hRm),
                        (float)(-n.ve * tanLat / hRn)};
  const float wCor[3] = {(float)(2.0 * wie[0] + wen[0]),
                         (float)(2.0 * wie[1] + wen[1]),
                         (float)(2.0 * wie[2] + wen[2])};
  const float wIn[3] = {wie[0] + wen[0], wie[1] + wen[1], wie[2] + wen[2]};

  float C[3][3];
  quatToDcm(n.q, C);

  // The specific force is taken straight from the mechanisation step that just
  // ran, rather than being reconstructed from the acceleration.
  float Sf[3][3], Sw[3][3], Sc[3][3];
  skew(n.fNav, Sf);
  skew(wIn, Sw);
  skew(wCor, Sc);

  float F[NS][NS];
  memset(F, 0, sizeof(F));
  for (int i = 0; i < NS; ++i) F[i][i] = 1.0f;

  // d(dN)/dt = dvN, and likewise for E and D (position error is in metres).
  for (int i = 0; i < 3; ++i) F[i][3 + i] += dt;

  // d(dv)/dt = -[f^n x] dpsi - [2 wie + wen x] dv + C dba
  // d(dpsi)/dt = -[w_in x] dpsi - C dbg
  // Bias states are constant; only Qd drives them.
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      F[3 + i][6 + j] += -Sf[i][j] * dt;
      F[3 + i][3 + j] += -Sc[i][j] * dt;
      F[3 + i][9 + j] += C[i][j] * dt;
      F[6 + i][6 + j] += -Sw[i][j] * dt;
      F[6 + i][12 + j] += -C[i][j] * dt;
    }
  }

  float FP[NS][NS], FPt[NS][NS];
  mulNN(F, s_P, FP);
  mulNBt(FP, F, FPt);

  const float qa = SIGMA_ACC_MS2 * SIGMA_ACC_MS2 * dt;
  const float qg = SIGMA_GYRO_RADS * SIGMA_GYRO_RADS * dt;
  const float qba = SIGMA_BA_MS2 * SIGMA_BA_MS2 * dt;
  const float qbg = SIGMA_BG_RADS * SIGMA_BG_RADS * dt;
  for (int i = 0; i < 3; ++i) {
    FPt[3 + i][3 + i] += qa;
    FPt[6 + i][6 + i] += qg;
    FPt[9 + i][9 + i] += qba;
    FPt[12 + i][12 + i] += qbg;
  }

  memcpy(s_P, FPt, sizeof(s_P));
  symmetrise(s_P);
}

// Generic measurement update, Joseph form. Returns false when the innovation
// is gated out or the innovation covariance is not positive definite.
bool applyUpdate(Nominal& n, const float H[MAXM][NS], const float z[MAXM],
                 const float R[MAXM][MAXM], int m) {
  float PHt[NS][MAXM];
  for (int i = 0; i < NS; ++i) {
    for (int j = 0; j < m; ++j) {
      float s = 0.0f;
      for (int k = 0; k < NS; ++k) s += s_P[i][k] * H[j][k];
      PHt[i][j] = s;
    }
  }

  float S[MAXM][MAXM];
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      float s = 0.0f;
      for (int k = 0; k < NS; ++k) s += H[i][k] * PHt[k][j];
      S[i][j] = s + R[i][j];
    }
  }

  float Sinv[MAXM][MAXM];
  if (!invertSPD(S, m, Sinv)) return false;

  // Innovation gate: z^T S^-1 z, scaled by the number of measurements.
  float chi2 = 0.0f;
  for (int i = 0; i < m; ++i) {
    float s = 0.0f;
    for (int j = 0; j < m; ++j) s += Sinv[i][j] * z[j];
    chi2 += z[i] * s;
  }
  if (!(chi2 < CHI2_GATE * (float)m)) return false;

  float K[NS][MAXM];
  for (int i = 0; i < NS; ++i) {
    for (int j = 0; j < m; ++j) {
      float s = 0.0f;
      for (int k = 0; k < m; ++k) s += PHt[i][k] * Sinv[k][j];
      K[i][j] = s;
    }
  }

  float dx[NS];
  for (int i = 0; i < NS; ++i) {
    float s = 0.0f;
    for (int j = 0; j < m; ++j) s += K[i][j] * z[j];
    dx[i] = s;
  }

  // P = (I-KH) P (I-KH)^T + K R K^T
  float A[NS][NS];
  for (int i = 0; i < NS; ++i) {
    for (int j = 0; j < NS; ++j) {
      float s = 0.0f;
      for (int k = 0; k < m; ++k) s += K[i][k] * H[k][j];
      A[i][j] = (i == j ? 1.0f : 0.0f) - s;
    }
  }
  float AP[NS][NS], APA[NS][NS];
  mulNN(A, s_P, AP);
  mulNBt(AP, A, APA);

  for (int i = 0; i < NS; ++i) {
    for (int j = 0; j < NS; ++j) {
      float s = 0.0f;
      for (int a = 0; a < m; ++a) {
        for (int b = 0; b < m; ++b) s += K[i][a] * R[a][b] * K[j][b];
      }
      s_P[i][j] = APA[i][j] + s;
    }
  }
  symmetrise(s_P);

  // Apply the correction to the nominal state. The error state is zero again
  // by definition once the correction has been fed back.
  double rm, rn;
  earthRadii(n.lat, rm, rn);
  const double cosLat = cos(n.lat);

  n.lat += (double)dx[0] / (rm + n.h);
  n.lon += (double)dx[1] / ((rn + n.h) * cosLat);
  n.h -= (double)dx[2];
  n.vn += dx[3];
  n.ve += dx[4];
  n.vd += dx[5];

  const float dpsi[3] = {dx[6], dx[7], dx[8]};
  quatApplyNavError(n.q, dpsi);

  for (int i = 0; i < 3; ++i) {
    n.ba[i] += dx[9 + i];
    n.bg[i] += dx[12 + i];
  }
  return true;
}

// ---------------------------------------------------------------------------
// 6. History ring
// ---------------------------------------------------------------------------

HistEntry& historyAt(int logical) {
  int idx = s_histHead - (s_histCount - 1) + logical;
  idx %= HIST_CAP;
  if (idx < 0) idx += HIST_CAP;
  return s_hist[idx];
}

void historyPush(uint32_t t_ms, const ImuSampleMs& imu, const Nominal& nom) {
  s_histHead = (s_histHead + 1) % HIST_CAP;
  HistEntry& e = s_hist[s_histHead];
  e.t_ms = t_ms;
  e.imu = imu;
  e.nom = nom;
  if (s_histCount < HIST_CAP) ++s_histCount;
}

// Index of the newest entry at or before t_ms, or -1 if none.
int historyFind(uint32_t t_ms) {
  for (int i = s_histCount - 1; i >= 0; --i) {
    if ((int32_t)(historyAt(i).t_ms - t_ms) <= 0) return i;
  }
  return -1;
}

// ---------------------------------------------------------------------------
// 7. Stationary detection, alignment, helpers
// ---------------------------------------------------------------------------

void applyImuAxisMapping(const ImuSample& in, ImuSampleMs& out) {
  const float a[3] = {in.acc[0], in.acc[1], in.acc[2]};
  const float g[3] = {in.gyro[0], in.gyro[1], in.gyro[2]};
  for (int i = 0; i < 3; ++i) {
    out.acc[i] = IMU_TO_BODY[i][0] * a[0] + IMU_TO_BODY[i][1] * a[1] +
                 IMU_TO_BODY[i][2] * a[2];
    out.gyro[i] = IMU_TO_BODY[i][0] * g[0] + IMU_TO_BODY[i][1] * g[1] +
                  IMU_TO_BODY[i][2] * g[2];
  }
}

void updateStationary(float dt, const ImuSampleMs& s) {
  const float accNorm = sqrtf(s.acc[0] * s.acc[0] + s.acc[1] * s.acc[1] +
                              s.acc[2] * s.acc[2]);
  const float gLocal = normalGravity(s_nom.lat, s_nom.h);
  const float accDev = fabsf(accNorm - gLocal);
  const float gyroNorm = sqrtf(s.gyro[0] * s.gyro[0] + s.gyro[1] * s.gyro[1] +
                               s.gyro[2] * s.gyro[2]);

  bool still = (accDev < STATIONARY_ACC_TOL) && (gyroNorm < STATIONARY_GYR_TOL);
  if (s_haveGnssSpeed && s_lastGnssSpeed > STATIONARY_GNSS_MS) still = false;

  if (still) {
    s_stillTimer += dt;
    s_moveTimer = 0.0f;
  } else {
    s_moveTimer += dt;
    s_stillTimer = 0.0f;
  }

  if (!s_stationary && s_stillTimer >= STATIONARY_ENTER_S) s_stationary = true;
  if (s_stationary && s_moveTimer >= STATIONARY_EXIT_S) s_stationary = false;
}

// Tilt-compensated magnetic heading for a given attitude, or false if the
// sample is unusable or locally disturbed.
bool magneticHeadingFrom(const float C[3][3], float& headingRad) {
  if (!s_mag.valid) return false;

  const float mx = s_mag.m[0] - MAG_HARD_IRON_UT[0];
  const float my = s_mag.m[1] - MAG_HARD_IRON_UT[1];
  const float mz = s_mag.m[2] - MAG_HARD_IRON_UT[2];
  const float norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm < 5.0f) return false;  // no usable field

  if (!s_magNormValid) {
    s_magNormEstimate = norm;
    s_magNormValid = true;
  }
  if (fabsf(norm - s_magNormEstimate) > MAG_NORM_TOL_UT) return false;
  s_magNormEstimate += 0.01f * (norm - s_magNormEstimate);

  const float m[3] = {mx, my, mz};
  float mn[3];
  for (int i = 0; i < 3; ++i) {
    mn[i] = C[i][0] * m[0] + C[i][1] * m[1] + C[i][2] * m[2];
  }
  // Heading of the horizontal field component, clockwise from north.
  headingRad = atan2f(mn[1], mn[0]) + MAG_DECLINATION_DEG * RAD;
  return true;
}

bool gnssUpdateAt(Nominal& n, const GnssFix& fix) {
  double rm, rn;
  earthRadii(n.lat, rm, rn);
  const double cosLat = cos(n.lat);

  const bool useVel = fix.velocity_valid;
  const int m = useVel ? 5 : 3;

  float H[MAXM][NS];
  memset(H, 0, sizeof(H));
  for (int i = 0; i < 3; ++i) H[i][i] = 1.0f;  // position error is already metres
  if (useVel) {
    H[3][3] = 1.0f;
    H[4][4] = 1.0f;
  }

  // Innovation expressed in metres.
  float z[MAXM];
  z[0] = (float)((fix.lat_rad - n.lat) * (rm + n.h));
  z[1] = (float)((fix.lon_rad - n.lon) * (rn + n.h) * cosLat);
  z[2] = (float)(-(fix.h_m - n.h));
  if (useVel) {
    z[3] = fix.vn - n.vn;
    z[4] = fix.ve - n.ve;
  }

  float R[MAXM][MAXM];
  memset(R, 0, sizeof(R));
  const float sp2 = SIGMA_GNSS_POS_M * SIGMA_GNSS_POS_M;
  R[0][0] = sp2;
  R[1][1] = sp2;
  R[2][2] = SIGMA_GNSS_HGT_M * SIGMA_GNSS_HGT_M;
  if (useVel) {
    const float sv2 = SIGMA_GNSS_VEL_MS * SIGMA_GNSS_VEL_MS;
    R[3][3] = sv2;
    R[4][4] = sv2;
  }

  return applyUpdate(n, H, z, R, m);
}

bool zuptUpdateAt(Nominal& n) {
  float H[MAXM][NS];
  memset(H, 0, sizeof(H));
  H[0][3] = 1.0f;
  H[1][4] = 1.0f;
  H[2][5] = 1.0f;

  const float z[3] = {-n.vn, -n.ve, -n.vd};
  float R[MAXM][MAXM];
  memset(R, 0, sizeof(R));
  const float s2 = SIGMA_ZUPT_MS * SIGMA_ZUPT_MS;
  R[0][0] = s2;
  R[1][1] = s2;
  R[2][2] = s2;
  return applyUpdate(n, H, z, R, 3);
}

bool magUpdateAt(Nominal& n) {
  float C[3][3];
  quatToDcm(n.q, C);

  float heading = 0.0f;
  if (!magneticHeadingFrom(C, heading)) return false;

  float roll, pitch, yaw;
  quatToEuler(n.q, roll, pitch, yaw);

  float H[MAXM][NS];
  memset(H, 0, sizeof(H));
  H[0][8] = 1.0f;  // yaw error is the down-axis attitude error

  const float z[1] = {wrapPi(heading - yaw)};
  float R[MAXM][MAXM];
  memset(R, 0, sizeof(R));
  R[0][0] = (SIGMA_MAG_HDG_DEG * RAD) * (SIGMA_MAG_HDG_DEG * RAD);
  return applyUpdate(n, H, z, R, 1);
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void reset() {
  memset(s_P, 0, sizeof(s_P));
  for (int i = 0; i < 3; ++i) {
    s_P[i][i] = P0_POS_M * P0_POS_M;
    s_P[3 + i][3 + i] = P0_VEL_MS * P0_VEL_MS;
    s_P[6 + i][6 + i] = P0_ATT_RAD * P0_ATT_RAD;
    s_P[9 + i][9 + i] = P0_BA_MS2 * P0_BA_MS2;
    s_P[12 + i][12 + i] = P0_BG_RADS * P0_BG_RADS;
  }
  memset(&s_nom, 0, sizeof(s_nom));
  s_nom.q[0] = 1.0f;

  memset(&s_mag, 0, sizeof(s_mag));
  memset(s_alignAccMean, 0, sizeof(s_alignAccMean));
  memset(s_alignGyroMean, 0, sizeof(s_alignGyroMean));
  memset(s_hist, 0, sizeof(s_hist));

  s_mode = Mode::WaitGnss;
  s_haveImu = false;
  s_lastImuMs = 0;
  s_lastDtMs = 0.0f;
  s_lastAuxMs = 0;
  s_alignCount = 0;
  s_alignElapsedS = 0.0f;
  memset(s_alignAccSum, 0, sizeof(s_alignAccSum));
  memset(s_alignGyroSum, 0, sizeof(s_alignGyroSum));
  s_stationary = false;
  s_stillTimer = 0.0f;
  s_moveTimer = 0.0f;
  s_lastGnssSpeed = 0.0f;
  s_haveGnssSpeed = false;
  s_velocityInit = false;
  s_magNormEstimate = 0.0f;
  s_magNormValid = false;
  s_histHead = -1;
  s_histCount = 0;
  s_imuCountWindow = 0;
  s_imuHz = 0;
  s_rateWindowStartMs = 0;
  s_gnssCount = 0;
  s_gnssRejected = 0;
}

void initialise(double lat_rad, double lon_rad, double h_m, uint32_t t_ms) {
  s_nom.lat = lat_rad;
  s_nom.lon = lon_rad;
  s_nom.h = h_m;
  s_nom.vn = s_nom.ve = s_nom.vd = 0.0f;
  s_nom.q[0] = 1.0f;
  s_nom.q[1] = s_nom.q[2] = s_nom.q[3] = 0.0f;
  memset(s_nom.ba, 0, sizeof(s_nom.ba));
  memset(s_nom.bg, 0, sizeof(s_nom.bg));
  memset(s_nom.aPrev, 0, sizeof(s_nom.aPrev));
  memset(s_nom.fNav, 0, sizeof(s_nom.fNav));

  s_histHead = -1;
  s_histCount = 0;
  s_mode = Mode::Align;
  s_alignCount = 0;
  s_alignElapsedS = 0.0f;
  memset(s_alignAccSum, 0, sizeof(s_alignAccSum));
  memset(s_alignGyroSum, 0, sizeof(s_alignGyroSum));
}

void updateImu(const ImuSample& sample) {
  ImuSampleMs s;
  s.t_ms = sample.t_us / 1000u;
  applyImuAxisMapping(sample, s);

  if (!s_haveImu) {
    s_haveImu = true;
    s_lastImuMs = s.t_ms;
    return;
  }

  int32_t dms = (int32_t)(s.t_ms - s_lastImuMs);
  if (dms <= 0) return;  // duplicate or out-of-order
  if (dms > 1000) dms = 1000;
  s_lastImuMs = s.t_ms;
  const float dt = (float)dms * 0.001f;
  s_lastDtMs = (float)dms;

  // Measured IMU rate over a one second window.
  ++s_imuCountWindow;
  if (s_rateWindowStartMs == 0) s_rateWindowStartMs = s.t_ms;
  const uint32_t elapsed = s.t_ms - s_rateWindowStartMs;
  if (elapsed >= 1000u) {
    s_imuHz = (uint32_t)((uint64_t)s_imuCountWindow * 1000ull / elapsed);
    s_imuCountWindow = 0;
    s_rateWindowStartMs = s.t_ms;
  }

  if (s_mode == Mode::WaitGnss) return;

  if (s_mode == Mode::Align) {
    for (int i = 0; i < 3; ++i) {
      s_alignAccSum[i] += s.acc[i];
      s_alignGyroSum[i] += s.gyro[i];
    }
    ++s_alignCount;
    s_alignElapsedS += dt;

    if (s_alignElapsedS >= ALIGN_DURATION_S && s_alignCount > 10) {
      const float inv = 1.0f / (float)s_alignCount;
      for (int i = 0; i < 3; ++i) {
        s_alignAccMean[i] = s_alignAccSum[i] * inv;
        s_alignGyroMean[i] = s_alignGyroSum[i] * inv;
      }

      // Accelerometer levelling: at rest the specific force points up, so a
      // level device reads (0, 0, -g) in FRD.
      const float ax = s_alignAccMean[0];
      const float ay = s_alignAccMean[1];
      const float az = s_alignAccMean[2];
      const float roll = atan2f(-ay, -az);
      const float pitch = atan2f(ax, sqrtf(ay * ay + az * az));

      // Yaw: prefer the GNSS course when already moving, else the magnetic
      // heading computed with the attitude just derived.
      float yaw = 0.0f;
      if (s_haveGnssSpeed && s_lastGnssSpeed > GNSS_SPEED_GATE_MS) {
        yaw = atan2f(s_nom.ve, s_nom.vn);
      } else {
        float qTmp[4];
        quatFromEuler(roll, pitch, 0.0f, qTmp);
        float CTmp[3][3];
        quatToDcm(qTmp, CTmp);
        float heading = 0.0f;
        if (magneticHeadingFrom(CTmp, heading)) yaw = heading;
      }

      quatFromEuler(roll, pitch, yaw, s_nom.q);
      for (int i = 0; i < 3; ++i) s_nom.bg[i] = s_alignGyroMean[i];
      memset(s_nom.ba, 0, sizeof(s_nom.ba));
      memset(s_nom.aPrev, 0, sizeof(s_nom.aPrev));
      memset(s_nom.fNav, 0, sizeof(s_nom.fNav));

      s_mode = Mode::Run;
      s_stationary = true;
      s_stillTimer = STATIONARY_ENTER_S;
      s_lastAuxMs = s.t_ms;
    }
    return;
  }

  // Mode::Run
  propagate(s_nom, s, dt);
  propagateCovariance(s_nom, dt);
  historyPush(s.t_ms, s, s_nom);

  updateStationary(dt, s);

  const bool auxDue =
      (uint32_t)(s.t_ms - s_lastAuxMs) >= AUX_UPDATE_PERIOD_MS;
  if (auxDue) {
    s_lastAuxMs = s.t_ms;
    if (ZUPT_ENABLE && s_stationary) {
      Nominal copy = s_nom;
      if (zuptUpdateAt(copy)) s_nom = copy;
    }
    if (MAG_AIDING_ENABLE && s_stationary) {
      Nominal copy = s_nom;
      if (magUpdateAt(copy)) s_nom = copy;
    }
  }
}

void updateGnss(const GnssFix& fix) {
  s_lastGnssSpeed = sqrtf(fix.vn * fix.vn + fix.ve * fix.ve);
  s_haveGnssSpeed = true;

  if (s_mode == Mode::WaitGnss) {
    initialise(fix.lat_rad, fix.lon_rad, fix.h_m, fix.t_ms);
    ++s_gnssCount;
    return;
  }
  if (s_mode != Mode::Run) return;  // still aligning: ignore aiding

  ++s_gnssCount;

  // Bootstrap. The INS always starts at rest, so a platform that is already
  // moving when the first fix lands presents a velocity innovation far outside
  // the innovation gate. Rejecting it would leave the filter free-running with
  // a growing covariance until some later fix is accepted at a huge gain. Seed
  // the velocity directly from the first valid GNSS velocity instead, and
  // apply that fix at the current epoch (there is no meaningful history yet).
  bool bootstrapped = false;
  if (fix.velocity_valid && !s_velocityInit) {
    s_velocityInit = true;
    bootstrapped = true;
    s_nom.vn = fix.vn;
    s_nom.ve = fix.ve;
    const float sv2 = SIGMA_GNSS_VEL_MS * SIGMA_GNSS_VEL_MS;
    s_P[3][3] = sv2;
    s_P[4][4] = sv2;
  }

  // Apply the update at the epoch the fix describes. Replaying is only valid
  // while the history is a pure IMU propagation; once ZUPT or magnetometer
  // updates have been interleaved, replaying would silently undo them, so
  // apply at the current epoch instead.
  int idx = -1;
  if (COMPENSATE_GNSS_LATENCY && !s_stationary && !bootstrapped) {
    const uint32_t applyMs = fix.t_ms - (uint32_t)GNSS_LATENCY_MS;
    idx = historyFind(applyMs);
  }

  if (idx >= 0) {
    Nominal target = historyAt(idx).nom;
    if (!gnssUpdateAt(target, fix)) {
      ++s_gnssRejected;
      return;
    }
    // Replay the stored samples forward from the corrected epoch, writing the
    // corrected trajectory back into the history as we go. Without the write
    // back, the next fix would replay from a stale, uncorrected state and
    // silently discard the correction that was just applied.
    //
    // The covariance is not re-propagated: over a sub-second replay the
    // difference is negligible and it keeps the filter numerically simple.
    Nominal n = target;
    historyAt(idx).nom = n;
    for (int i = idx + 1; i < s_histCount; ++i) {
      HistEntry& e = historyAt(i);
      HistEntry& prev = historyAt(i - 1);
      const int32_t d = (int32_t)(e.t_ms - prev.t_ms);
      propagate(n, e.imu, (d > 0 ? (float)d : 0.0f) * 0.001f);
      e.nom = n;
    }
    s_nom = n;
  } else {
    if (!gnssUpdateAt(s_nom, fix)) {
      ++s_gnssRejected;
      return;
    }
  }
}

void updateMag(const MagSample& sample) {
  s_mag = sample;
  if (!sample.valid) s_magNormValid = false;
}

void getOutput(Output& out) {
  out.mode = s_mode;
  out.lat_deg = s_nom.lat * (double)DEG;
  out.lon_deg = s_nom.lon * (double)DEG;
  out.h_m = s_nom.h;
  out.vn = s_nom.vn;
  out.ve = s_nom.ve;
  out.vd = s_nom.vd;

  float roll, pitch, yaw;
  quatToEuler(s_nom.q, roll, pitch, yaw);
  out.roll_deg = roll * DEG;
  out.pitch_deg = pitch * DEG;
  out.yaw_deg = wrapPi(yaw) * DEG;
  if (out.yaw_deg < 0.0f) out.yaw_deg += 360.0f;

  for (int i = 0; i < 3; ++i) {
    out.ba_ms2[i] = s_nom.ba[i];
    out.bg_dps[i] = s_nom.bg[i] * DEG;
    out.align_acc_mean[i] = s_alignAccMean[i];
    out.align_gyro_mean[i] = s_alignGyroMean[i];
  }

  out.dt_ms = s_lastDtMs;
  out.imu_hz = s_imuHz;
  out.gnss_count = s_gnssCount;
  out.gnss_rejected = s_gnssRejected;
  out.zupt_active = s_stationary && ZUPT_ENABLE;
  out.mag_active = s_stationary && MAG_AIDING_ENABLE;

  out.healthy = true;
  for (int i = 0; i < NS; ++i) {
    if (!(s_P[i][i] > 0.0f) || !(s_P[i][i] < 1e12f)) out.healthy = false;
  }
}

}  // namespace ins
