/*
 * ins_ekf.h - 15-state INS / GNSS Extended Kalman Filter
 *
 * Strapdown inertial navigation mechanisation following Titterton & Weston,
 * "Strapdown Inertial Navigation Technology" (2nd ed.), with a 15-state
 * error-state EKF for GNSS aiding.
 *
 * Deliberately free of Arduino dependencies so the whole filter can be
 * compiled and unit-tested on a PC (see test/host/test_ins.cpp).
 *
 * FRAMES AND CONVENTIONS
 *   Navigation frame : NED  (north, east, down)
 *   Body frame       : FRD  (forward, right, down)
 *   Attitude         : quaternion, body -> nav
 *   Position         : geodetic, WGS-84, height is ELLIPSOIDAL
 *   Angles           : radians internally, degrees at the API boundary
 *
 * THE 15 STATES (error state, i.e. what the filter estimates)
 *   0..2   dN, dE, dD          local position error      [m]
 *   3..5   dvN, dvE, dvD       velocity error            [m/s]
 *   6..8   dpsiN, dpsiE, dpsiD attitude (psi-angle) error[rad]
 *   9..11  dba_x,y,z           accel bias error, FRD     [m/s^2]
 *   12..14 dbg_x,y,z           gyro bias error, FRD      [rad/s]
 *
 * Why position error is metres rather than Titterton's radians: a radian
 * position error is ~1e-7 for a metre, so P would carry entries spanning
 * 1e-14 (position variance) to 25 (height variance). Single precision has
 * roughly 7 significant digits and silently loses the small terms during the
 * covariance update. Metres keeps every state in a comparable magnitude.
 * The mechanisation itself remains Titterton's; only the error-state
 * parameterisation differs.
 */

#ifndef INS_EKF_H
#define INS_EKF_H

#include <stdint.h>

namespace ins {

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

// Set to 0 to disable a feature (useful when isolating a problem on hardware).
constexpr bool ZUPT_ENABLE      = true;   // zero-velocity update when stationary
constexpr bool MAG_AIDING_ENABLE = true;  // magnetometer heading when stationary
constexpr bool COMPENSATE_GNSS_LATENCY = true;

// Assumed age of a GNSS fix when it reaches the filter: receiver solution
// latency plus NMEA transmission time at 38400 baud.
constexpr float GNSS_LATENCY_MS = 150.0f;

// Magnetic declination (true north = magnetic north + declination).
// Positive east. Set for your location; a wrong value biases yaw directly.
constexpr float MAG_DECLINATION_DEG = 0.0f;

// Maps the IMU's own axes onto the INS body frame (FRD). Identity means the
// IMU frame already IS the body frame.
//
// VERIFY THIS ON HARDWARE. Lay the device flat and level: the INS expects the
// accelerometer to read approximately (0, 0, -g) in body FRD, because a
// stationary accelerometer measures specific force, which points up while
// +D points down. If the device instead reports (0, 0, +g), or reports
// gravity on a different axis, fix it here rather than in the filter. The
// alignment means are published in Output::align_acc_mean so this can be
// checked from the serial console.
constexpr float IMU_TO_BODY[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

// Hard-iron offset applied to the raw magnetometer sample, in microtesla.
// Zero means "uncalibrated": heading aiding will then be biased but bounded.
constexpr float MAG_HARD_IRON_UT[3] = {0.0f, 0.0f, 0.0f};

// Measurement noise (1 sigma).
constexpr float SIGMA_GNSS_POS_M   = 2.5f;   // horizontal
constexpr float SIGMA_GNSS_HGT_M   = 5.0f;   // vertical, notably worse
constexpr float SIGMA_GNSS_VEL_MS  = 0.25f;  // ground velocity
constexpr float SIGMA_GNSS_CRS_DEG = 2.0f;   // course, folded into velocity
constexpr float SIGMA_ZUPT_MS      = 0.02f;
constexpr float SIGMA_MAG_HDG_DEG  = 8.0f;

// Process noise (spectral densities) - expect to tune these.
constexpr float SIGMA_ACC_MS2      = 0.05f;   // accel white noise   [m/s^2/sqrt(Hz)]
constexpr float SIGMA_GYRO_RADS    = 0.003f;  // gyro white noise    [rad/s/sqrt(Hz)]
constexpr float SIGMA_BA_MS2       = 0.001f;  // accel bias walk     [m/s^2/sqrt(s)]
constexpr float SIGMA_BG_RADS      = 2.0e-5f; // gyro bias walk      [rad/s/sqrt(s)]

// Initial 1-sigma uncertainties after alignment.
constexpr float P0_POS_M    = 10.0f;
constexpr float P0_VEL_MS   = 1.0f;
constexpr float P0_ATT_RAD  = 0.05f;   // ~3 deg
constexpr float P0_BA_MS2   = 0.20f;
constexpr float P0_BG_RADS  = 0.01f;   // ~0.6 deg/s

// Gates and limits.
constexpr float GNSS_SPEED_GATE_MS = 1.0f;   // below this, RMC course is noise
constexpr float STATIONARY_ACC_TOL = 0.15f;  // | |acc| - g | threshold [m/s^2]
constexpr float STATIONARY_GYR_TOL = 0.035f; // ~2 deg/s
constexpr float STATIONARY_GNSS_MS = 0.5f;
constexpr float STATIONARY_ENTER_S = 1.0f;
constexpr float STATIONARY_EXIT_S  = 0.3f;
constexpr float MAX_DT_S           = 0.05f;  // skip covariance update above this
constexpr float ALIGN_DURATION_S   = 2.0f;
constexpr float CHI2_GATE          = 25.0f;  // 5-DOF-ish innovation gate

// Magnetometer disturbance rejection: reject a sample whose magnitude is
// further than this from the running median estimate.
constexpr float MAG_NORM_TOL_UT = 25.0f;

// ---------------------------------------------------------------------------

enum class Mode : uint8_t {
  WaitGnss = 0,  // no usable fix yet
  Align    = 1,  // collecting static samples for attitude and gyro bias
  Run      = 2,  // propagating and aiding
};

struct ImuSample {
  uint32_t t_us;
  float acc[3];   // m/s^2, FRD, includes gravity (specific force)
  float gyro[3];  // rad/s, FRD
};

struct GnssFix {
  uint32_t t_ms;
  double lat_rad;
  double lon_rad;
  double h_m;        // ellipsoidal
  float vn;          // m/s, from RMC speed and course
  float ve;
  bool velocity_valid;
};

struct MagSample {
  uint32_t t_ms;
  float m[3];        // microtesla, FRD
  bool valid;
};

// Everything the application needs, already in display/log units.
struct Output {
  Mode mode;
  double lat_deg;
  double lon_deg;
  double h_m;         // ellipsoidal WGS-84
  float vn, ve, vd;   // m/s
  float roll_deg, pitch_deg, yaw_deg;
  float ba_ms2[3];    // FRD
  float bg_dps[3];    // FRD, degrees per second
  float dt_ms;        // last propagation step
  uint32_t imu_hz;    // measured IMU rate
  uint32_t gnss_count;
  uint32_t gnss_rejected;
  bool zupt_active;
  bool mag_active;
  bool healthy;       // false if the filter hit a numerical problem

  // Mean IMU readings collected during alignment. Published so the axis
  // mapping (IMU_TO_BODY) can be verified from the serial console on real
  // hardware, where it cannot be reasoned about from the source alone.
  float align_acc_mean[3];
  float align_gyro_mean[3];
};

void reset();

// Feed one IMU sample. Propagates the nominal state and the covariance.
void updateImu(const ImuSample& sample);

// Feed a GNSS fix. Applies the measurement update, optionally at the epoch
// the fix actually describes (see COMPENSATE_GNSS_LATENCY).
void updateGnss(const GnssFix& fix);

// Feed a magnetometer sample. Only used for heading aiding while stationary.
void updateMag(const MagSample& sample);

void getOutput(Output& out);

// Seed the filter with a first fix and begin alignment. Called automatically
// by updateGnss() when the mode is WaitGnss.
void initialise(double lat_rad, double lon_rad, double h_m, uint32_t t_ms);

}  // namespace ins

#endif  // INS_EKF_H
