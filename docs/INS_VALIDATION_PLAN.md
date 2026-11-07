# INS orientation and validation plan

Prepared 2026-11-07 against `99c038f` on `master`. The user's corrected mounting description is **screen facing up, DC-input side facing the user, walking away from that side**. The USB-C side is not the rear reference.

Implementation update: the `(Y, X, -Z)` mounting rotation is now applied to accel, gyro and magnetometer inputs, with input-frame comments, mounting instructions in the README, and host regression coverage. The findings and 45-check baseline below describe the original assessment; the remaining heading, timing and replay work is still planned. Hardware verification remains outstanding.

## 1. Establish the sensor and body frames first

For the normally assembled CoreS3/DinBase shown in M5Stack's diagrams:

| Axis returned by M5Unified | Physical direction in this mounting | INS FRD component |
| --- | --- | --- |
| +X | Right, parallel to the screen | +Right |
| +Y | Forward, away from the DC-input side | +Forward |
| +Z | Up, out of the screen | -Down |

Top view, with the user's walking direction at the top:

```text
                   forward = +Y
                         ^
                         |
               +------------------+
               |     screen up    | ----> right = +X
               +------------------+
                    DC-input side
                         user

             +Z out of screen; INS +Down into screen
```

The proposed M5Unified-to-FRD rotation is:

```cpp
constexpr float IMU_TO_BODY[3][3] = {
  {0.0f, 1.0f,  0.0f},  // forward = sensor Y
  {1.0f, 0.0f,  0.0f},  // right   = sensor X
  {0.0f, 0.0f, -1.0f},  // down    = -sensor Z
};
```

This is a proper rotation, with determinant +1. Negating Z alone would be a reflection. Apply the same physical rotation to accelerometer, gyro and magnetometer vectors, once only. Display rotation does not define the body frame. Body forward follows the device; it only follows walking direction while the device remains mounted as described.

Evidence:

- [M5Stack CoreS3 hardware documentation](https://docs.m5stack.com/en/core/CoreS3), including the BMI270 axis diagram and physical connector layout.
- [M5Stack CoreS3 IMU example](https://docs.m5stack.com/en/arduino/m5cores3/imu) explicitly describes the axes after M5Unified correction.
- [M5Unified 0.2.5 IMU implementation](https://github.com/m5stack/M5Unified/blob/0.2.5/src/utility/IMU_Class.cpp): the version named in this repository's README preserves the CoreS3 accel/gyro orientation and flips the BMM150 Y/Z axes internally to align its output. Do not apply that chip-specific correction a second time. The application still needs the mounting-to-FRD rotation above.

These establish the expected mapping, not a measurement from this particular device. Record the actual installed library version and check the assembled base orientation before treating it as hardware-verified.

### Hardware acceptance checks

Use the existing page 5 values for static checks; these are M5Unified axes before the filter mapping. For rotations, capture full-rate readings with timestamps rather than relying on the slow display. Perform checks away from magnetic objects and hold each static pose for several seconds.

| Test | Expected M5Unified accel/gyro | Expected mapped FRD value |
| --- | --- | --- |
| Screen up, level and still | accel approximately (0, 0, +9.8) m/s² | (0, 0, -9.8) m/s² |
| Screen down, level and still | accel approximately (0, 0, -9.8) | (0, 0, +9.8) |
| Raise far edge until +Y points vertically up | accel approximately (0, +9.8, 0) | (+9.8, 0, 0) |
| Raise right edge until +X points vertically up | accel approximately (+9.8, 0, 0) | (0, +9.8, 0) |
| Turn right while level, clockwise viewed from above | gyro Z negative during turn | gyro Down positive |

Also test the opposite X/Y faces for all six accelerometer orientations. Confirm positive body roll when the right side drops and positive pitch when the forward edge rises. A level check alone cannot distinguish horizontal-axis swaps. After mapping, the serial `ALIGN acc=(...)` should be approximately `(0,0,-9.8)`, with roll/pitch near zero. Use approximately 0.5 m/s² dominant-axis tolerance and 3 degrees level-angle tolerance as initial screening limits, then refine from calibration data.

## 2. Repository findings that affect validation

| Finding | Evidence in current source | Required action before interpreting performance |
| --- | --- | --- |
| Identity sensor mapping | `ins_ekf.h`, `IMU_TO_BODY` | Verify and implement the mounting rotation; update synthetic fixtures to respect the chosen API frame. |
| Magnetometer bypasses mapping | `updateMag()` copies directly; sketch passes M5Unified values | Define one consistent frame and calibration convention for all sensors. |
| CSV is not a replay input stream | `imuTask` and `insTask` publish snapshots only once per second; `logTask` writes them at 10 Hz | Capture every consumed filter input separately. Repeating/interpolating the old CSV cannot reconstruct missing IMU dynamics. |
| Magnetic heading formula fails a cardinal-direction check | `magneticHeadingFrom()` and `magUpdateAt()` | Add cardinal/tilted heading tests and fix heading/residual conventions before enabling mag performance claims. |
| Microsecond rollover is mishandled | `updateImu()` divides uint32 microseconds by 1000 before differencing | Test/fix wrap near 71.6 minutes, preserving a common IMU/GNSS timebase. |
| GNSS epoch and arrival time are conflated | `fix.t_ms = now`, fixed 150 ms subtraction; fractional UTC is discarded | Preserve fractional measurement epoch and arrival/consumption times. Match GGA/RMC epochs and enforce freshness. |
| Slow walking loses the stationarity speed veto | Below 1 m/s the sketch sends vn=ve=0; filter uses their norm as GNSS speed | Preserve actual speed independently from course-valid velocity; test walking between 0.5 and 1 m/s. |
| Initial alignment assumes static data | Align averages samples without checking stationarity; moving-course branch reads nominal velocity that is still zero | Require a stationary startup or add an explicit, separately tested initialization mode. |
| Runtime flags are insufficient diagnostics | `mag_active`/`zupt_active` describe enabled/stationary state; `healthy` checks covariance diagonals | Log actual update acceptance, residuals, covariance validity and input freshness. |
| Sensor update/freshness assumptions need checking | Any nonzero `M5.Imu.update()` result enqueues all sensors and marks mag valid | Inspect accel/gyro/mag readiness bits separately; do not label stale samples as new. |

The GNSS queue comment says “newest wins,” but the current loop actually calls `updateGnss()` for every queued fix. Faithful replay must preserve actual consumed order. Both producer queue sends currently ignore failure, so add loss counters. Delayed GNSS updates replay nominal history without rewinding/repropagating covariance; validate this approximation during turns and stop/start transitions.

Baseline verification performed:

- Existing host suite: **45 passed, 0 failed**, using the real `ins_ekf.cpp` through `test_ins.cpp`.
- A temporary diagnostic probe supplied a level east-facing FRD magnetic field `(0,-30,20)` µT at zero declination: alignment heading was -90 degrees instead of +90. With a correct +90-degree attitude, the update residual was -90 degrees instead of zero. This isolates a formula issue independently of physical mounting.
- A temporary rollover probe supplied consecutive IMU samples 10 ms apart across uint32 wrap: the second sample was rejected and the saved time did not advance.
- No on-device test or measured navigation-accuracy result has been obtained.

On this Mac, the default compiler/SDK combination failed to link. The existing suite passed with:

```sh
/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/clang++ \
  -std=c++17 -O2 -Wall -Wextra \
  -isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk \
  test/host/test_ins.cpp -o /tmp/m5-test-ins
/tmp/m5-test-ins
```

There is one existing unused-parameter warning for `initialise(..., t_ms)`. A portable build target should use the selected compiler's matching SDK instead of hardcoding this machine's path.

## 3. Record data that can reproduce the device run

Keep the existing 44-column summary CSV compatible. Add a separate versioned replay stream and session metadata. Use buffered records sent to the existing SD-owning task; no SD writes or blocking waits in sensor/filter loops. Preserve `g_spiMutex` around every file operation, and size the buffering from measured SD stalls and available RAM.

Minimum records:

- **IMU:** sequence, original sample timestamp, unwrapped monotonic timestamp, accel in m/s² including specific force, gyro in rad/s, frame ID, readiness flags. Preserve every sample consumed by the INS, approximately 100 Hz.
- **Mag:** original measurement timestamp, vector in µT, frame, validity/freshness; distinguish a new reading from reuse of the last one.
- **GNSS:** full fractional epoch, local arrival time, accepted fix fields, ellipsoidal height, speed/course and their validity/age, fix quality, GGA/RMC association. Preserve raw timestamped NMEA as well for parser/integration tests.
- **Execution:** monotonic sequence of `updateImu`, `updateMag`, `updateGnss` calls and their arguments, plus queue drop counts. This permits reproduction of the real scheduling and delayed arrivals.
- **Output:** full-rate state for replay comparison; diagnostic cadence may be lower if storage limits require it. Include accepted/rejected aiding counts, innovations, innovation covariance/NIS, state covariance, mode, and numerical faults.
- **Metadata:** firmware commit, library versions, sensor ODR/range/filter settings, units, axis rotation, calibration/NVS offsets, declination, EKF configuration, antenna-to-IMU lever arm, timestamp convention, schema version.

Record both original sensor coordinates and the mapping metadata so a future mapping correction can reuse a recording. Avoid redundant storage by deriving body values offline. If either the capture queue or storage loses records, report exact gaps and mark that interval unsuitable for exact reproduction.

## 4. Build a deterministic desktop replay harness

Compile and link the production `ins_ekf.cpp`; do not rewrite the EKF in Python. Use Python only for dataset conversion, metrics and plotting. Add portable CMake/CTest support while retaining the MSVC runner.

Resolve the API mismatch explicitly: `ImuSample` says FRD, but `updateImu()` currently applies a hardware mapping. Prefer an Arduino-free filter accepting FRD and a shared, tested input adapter applying the mounting rotation. Alternatively retain the mapping inside the filter with an explicit configurable input frame. Never apply the CoreS3 mounting rotation to an already-FRD public dataset.

Two replay modes are needed:

1. **Device reproduction:** replay recorded API calls in their original consumed order and with original timestamps; compare against on-device states. Treat floating-point differences with declared tolerances, not bit equality across CPU architectures.
2. **Algorithm benchmark:** schedule measurement epochs and simulated arrival times explicitly. Sweep latency, GNSS outages and sensor noise using the same kernel. Inputs must remain causal; ground truth is used for scoring only, apart from an explicitly labeled initialization experiment.

Use uint64 monotonic time in the replay format; test conversion to the device clock and wrap behavior. Select an explicit latency policy for each run: dataset epoch timestamps must not automatically get another 150 ms subtracted. Preserve irregular dt; never replace it with an assumed 0.01 s. Apply appropriate aggregation/filtering before downsampling public IMU data.

Proposed artifacts: `test/replay/` for the C++ runner, `tools/ins_validation/` for adapters/scoring, small deterministic fixtures under `test/data/`, and generated reports outside versioned source. Record dataset hashes and configuration with every result. Keep large recordings out of Git.

## 5. Use public data in increasing order of difficulty

| Dataset | Purpose | Adapter requirements / limits |
| --- | --- | --- |
| [WHU Awesome GINS](https://github.com/i2Nav-WHU/awesome-gins-datasets) | First real-data benchmark, starting with consumer-grade ICM20602 | Includes FRD IMU increments, RTK positions, reference navigation and lever arms. Convert delta-angle/delta-velocity to interval-average rates with measured dt; document interval timing. Account for antenna lever arm. GNSS file is position-only: do not invent velocity from reference truth. |
| [KF-GINS demo](https://github.com/i2Nav-WHU/KF-GINS) | Small integration check and optional independent implementation comparison | Similar input conventions, with configured initial state. A reference-initialized run must be labeled separately from autonomous cold start. Match aiding/configuration before comparing algorithms. |
| [UrbanNav](https://github.com/IPNL-POLYU/UrbanNavDataset) | Urban multipath and tunnel/outage stress after the basic replay works | Download IMU/GNSS/reference subsets. Convert clocks, frames and extrinsics; produce GNSS-only positions from receiver data or RINEX processing. Do not feed the reference INS trajectory back as GNSS. Respect the reference-validity intervals. |

For increments, dividing by dt is an adapter approximation to this rate-driven mechanisation; assess coning/sculling and interval-boundary effects on dynamic runs. Prefer the native sample rate first, then evaluate a properly reduced 100 Hz stream. When data lack a magnetometer or a static startup, use a documented initialization experiment and disable unavailable aiding rather than fabricating it.

Public vehicle datasets exercise navigation mathematics and robustness, but cannot establish hand-carried BMI270 walking performance. Real CoreS3 recordings remain necessary. RTK-aided benchmark results also do not predict M9N standalone accuracy.

## 6. Validation experiments and scoring

Run the following in order; correct frame/timing defects before tuning noise parameters.

1. **Deterministic correctness:** six poses; all gyro signs; four magnetic headings plus tilt and declination; frame orthogonality; stationary/moving alignment; no-fix startup; stale/missing RMC; duplicate/out-of-order samples; long gaps; timestamp wrap; known constant-velocity and turning trajectories. Add measurement outliers and covariance positive-definiteness checks. Generate truth independently of the filter equations where possible.
2. **Recorded hardware equivalence:** replay a short complete on-device stream; verify the same event counts/modes/accepted updates and small documented state differences. Require zero unexplained dropped events.
3. **Static characterization:** 10-minute rest tests and a 1–2-hour recording for noise/bias characterization and rollover. Check gyro bias, accel norm, roll/pitch, false motion and temperature dependence. Allan-deviation estimates inform tuning but do not by themselves prove navigation accuracy.
4. **Controlled walks:** open sky, 30 seconds stationary before and after; straight out-and-back; rectangle with known turn directions; repeated stop/start; slow walking around the 0.5/1.0 m/s thresholds. First use a rigid mount, then normal hand carrying as a separate condition. Record magnetic disturbance and device rotation relative to travel.
5. **Fault injection:** mask GNSS for 1, 5, 10 and 30 seconds in straight motion, turns and stops. Sweep delay and jitter around the current 150 ms assumption; add realistic outliers, loss, stale values and sample gaps. Measure recovery, not just error during the outage.
6. **Ablations:** GNSS-only baseline; INS+GNSS; plus ZUPT; plus calibrated mag; latency compensation off/on. Use identical input recordings and separate tuning runs from held-out evaluation runs.

Report horizontal RMSE/median/95th percentile/max, vertical RMSE, NED velocity RMSE, roll/pitch/yaw error where reference exists, outage endpoint error and drift rate, recovery time, false ZUPT events, GNSS rejection rates, actual mag acceptance, timing gaps and runtime headroom. Plot N/E paths, error versus time, speed/yaw, biases, covariance bounds, innovations/NIS and aiding decisions.

Use a fixed WGS-84 local NED origin, correct ellipsoidal heights, documented antenna/IMU reference points, and synchronized reference times. Do not rotate/translate the whole estimated path to best fit the reference; that would hide heading and position failures. Score only intervals with valid truth. A measured return to the start is a closure check, not a complete accuracy reference. The M9N observations used for aiding are not independent ground truth; use an independent RTK/surveyed reference for absolute accuracy claims and describe any shared-error limitations of public reference solutions.

Initial release gates: all deterministic checks pass; no NaNs or invalid covariance; no silent data loss; correct clock rollover; no sustained ZUPT during known motion; repeatable replay. Set numerical navigation-error targets after collecting a baseline and an application accuracy requirement, then freeze them before evaluating held-out walks. Smoother-looking tracks alone do not establish improvement.

## 7. Implementation order

1. Verify the corrected DC-side mounting on hardware; establish common accel/gyro/mag frame and targeted regression tests.
2. Fix demonstrated heading/timestamp issues and startup/freshness behavior needed for meaningful replay.
3. Add full-rate buffered capture and execution diagnostics; verify acquisition is not disrupted by SD writes.
4. Deliver desktop replay, first with synthetic fixtures and a short CoreS3 recording.
5. Add WHU adapter and automated reports, then UrbanNav stress cases.
6. Collect independent-reference walks, evaluate ablations/outages, and tune on a separate training set.
