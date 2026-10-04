# AGENTS.md

## Project overview

Arduino firmware for an **M5Stack CoreS3 GNSS/INS logger**. It reads NMEA 0183 sentences from a **u-blox M9N** over UART, parses them with an in-sketch parser (no parsing library), tracks every constellation the receiver reports, and fuses the onboard BMI270 with GNSS in a **15-state INS/GNSS Extended Kalman Filter** running at 100 Hz.

Satellites from **GPS, Galileo, BeiDou, GLONASS, QZSS and SBAS** are tracked separately.

Three files make up the firmware:

| File | Role |
|---|---|
| `sensorLogger/sensorLogger.ino` | Application: tasks, display, logging, NMEA parser, UBX config |
| `sensorLogger/ins_ekf.h` | INS/EKF public API, state definition, all tuning constants |
| `sensorLogger/ins_ekf.cpp` | Titterton mechanisation + error-state EKF. **No Arduino dependencies** |

Because `ins_ekf.cpp` is Arduino-free it compiles directly on a PC, which is how it is unit-tested (`test/host/`). Keep it that way.

## Hardware

| Part | Notes |
|---|---|
| M5Stack CoreS3 | ESP32-S3, dual core, LCD with touch, PWR button, BMI270 IMU |
| u-blox M9N GNSS module | NMEA output, wired to UART2 @ 38400 8N1 |
| microSD card | SPI mode, **shares SPI2_HOST with the LCD** |

Pin definitions (top of the sketch):

| Signal | GPIO |
|---|---|
| SD SPI SCK / LCD SCLK | 36 |
| SD SPI MISO / LCD D-C | 35 |
| SD SPI MOSI / LCD MOSI | 37 |
| SD SPI CS | 4 |
| LCD CS | 3 |
| GNSS `RxPin` (ESP32 RX) | 18 |
| GNSS `TxPin` (ESP32 TX) | 17 |
| Internal I2C SDA / SCL | 12 / 11 |

## Build, test and flash

- Use the **Arduino IDE** (there is no arduino-cli config or PlatformIO setup in the repo). Open the `sensorLogger/` folder itself, not the repo root: Arduino requires the sketch folder name to match the `.ino` name.
- Board package: **M5Stack**; select the `M5CoreS3` board.
- Libraries: only **M5Unified** + **M5GFX** (the vendor board support package for the LCD, AXP2101 PMU, AW9523B IO expander and BMI270/BMM150 IMU). `SPI`, `SD`, `FS` and FreeRTOS ship inside the board package.
- **There is no NMEA parsing library dependency.** Do not re-add TinyGPSPlus; the parser lives in the sketch.
- Compile: `arduino-cli compile --fqbn m5stack:esp32:m5stack_cores3 --build-path .build sensorLogger`
- Unit tests: run `test/host/run_tests.bat` (MSVC). It builds and runs the INS/EKF suite against the real `ins_ekf.cpp`.
- Flash: `arduino-cli upload -p <PORT> --fqbn m5stack:esp32:m5stack_cores3 --build-path .build sensorLogger`
- `Serial` on this board is **HWCDC** (`usb_mode=1` = "Hardware CDC and JTAG"), i.e. the same USB-Serial-JTAG peripheral the ROM uses. Debug @ 115200, gated by `DEBUG_SERIAL`.

## Architecture

### FreeRTOS tasks

| Task | Core | Prio | Cadence | Stack | Exclusive ownership |
|---|---|---|---|---|---|
| `gnssTask` | 0 | 6 | 2 ms UART drain | 4096 | `Serial2`, NMEA parser, satellite table, `g_gnss` |
| `imuTask` | 1 | 6 | 5 ms poll, data-ready gated | 3072 | `M5.Imu`, `g_imu` |
| `insTask` | 1 | 5 | per IMU sample (~100 Hz) | 8192 | the EKF, trajectory ring |
| `logTask` | 0 | 3 | 10 Hz | 4096 | `SD`, `File` |
| `uiTask` | 0 | 2 | 5 Hz + button | 4096 | `M5.update()`, `M5.Display`, `M5.BtnPWR` |
| `loop()` | 1 | 1 | idle | — | nothing |

`setup()` starts the tasks **last**, after M5, `Serial2`, the UBX config and the log file are ready.

### Data flow

- `imuTask` → `insTask`: `xQueue` of `ImuQueueItem` (depth 16), carrying `ins::ImuSample` plus the magnetometer reading. `t_us` comes from `data.usec`, so `dt` is **measured, never assumed**.
- `gnssTask` → `insTask`: `xQueue` of `ins::GnssFix` (depth 4), one per GGA epoch with a usable position. InsTask drains it and the newest wins.
- Producers → `uiTask` / `logTask`: a single `AppSnapshot` guarded by `g_stateMutex`. UI and log keep private copies so they hold the lock only for the copy.

### Locking

1. **`g_spiMutex` guards every `M5.Display` call and every `SD`/`File` call.** This is mandatory, not defensive: the LCD and microSD share `SPI2_HOST`, and the CoreS3 LCD driver repurposes **GPIO35 between MISO and D/C on every chip select** (`M5GFX.cpp`, `Panel_M5StackCoreS3::cs_control`). A concurrent SD transaction would have its MISO line driven by the LCD driver.
2. `g_stateMutex` guards `g_snapshot`. Lock order is always `spi` → `traj` → `state`; nothing takes them in the reverse order.
3. `g_trajMutex` guards the trajectory ring (appended by `insTask`, read by `uiTask`).
4. **No I2C mutex is needed.** M5GFX's I2C layer takes a per-port mutex around every transaction, and each M5 subsystem has exactly one owning task. `M5.update()` never calls `Imu.update()`, so the IMU task has the BMI270 to itself.

### NMEA parser

| Sentence | Fields consumed |
|---|---|
| `GGA` | 1 time, 2/3 lat, 4/5 lon, 6 fix quality, 7 sats used, 9 altitude, 11 geoid separation |
| `RMC` | 1 time, 2 status, 3/4 lat, 5/6 lon, 7 speed (knots), 8 course, 9 date |
| `GSV` | 1 message count, 3 sats in view, then 4 fields per satellite |

- A sentence must carry a **valid checksum** to be accepted at all; an **overlong sentence is discarded**, never truncated.
- Parsing is in place: `*` becomes `\0` and commas become `\0`. Field access is **1-based** via `nmeaField()`.
- **Fields start at offset 6, not 5**: the first comma follows the 5-character sentence id. Getting this wrong shifts every field by one and fails silently.
- `strtoul` stops at the decimal point, so an `hhmmss.ss` time field already yields `hhmmss` — do not divide by 100.
- The sentence type is matched with `strncmp(..., 3)` because the pointer used for matching sits mid-line.
- Talker → constellation: `GP`→GPS, `GA`→Galileo, `GB`/`BD`→BeiDou, `GL`→GLONASS, `GQ`→QZSS. `GN` means merged numbering and cannot be attributed; such satellites are tracked as `CONST_UNKNOWN` with a one-time warning.
- **PRNs 120+ inside `GPGSV` are SBAS**, not GPS.

### INS / EKF

Nominal-state propagation uses the Titterton & Weston strapdown mechanisation (NED nav frame, FRD body frame, quaternion attitude, trapezoidal velocity and position). The filter is a **15-state error-state (indirect) EKF with feedback**: the EKF estimates errors, each update corrects the nominal state, and the error state is zero again by definition.

Error states: `dN,dE,dD` [m] · `dvN,dvE,dvD` [m/s] · `dpsiN,dpsiE,dpsiD` [rad] · accel bias FRD [m/s²] · gyro bias FRD [rad/s].

- **Position error is metres, not Titterton's radians, on purpose.** A radian position error is ~1e-7 per metre, so P would span 1e-14 to 25 across its diagonal; single precision has ~7 significant digits and would lose the small terms. `lat`/`lon`/`h` in the nominal state are `double` for the same reason — a float position silently discards the Kalman correction, which is ~1e-8 rad.
- Error dynamics: `dv = -[f^n x]dpsi - [2wie+wen x]dv + C dba`, `dpsi = -[win x]dpsi - C dbg`. The specific force is taken from the mechanisation step (`Nominal::fNav`), not reconstructed.
- Measurements: GNSS position (3), GNSS velocity from RMC speed+course (2, **speed-gated at 1 m/s** because course is meaningless below that), ZUPT (3), magnetometer heading (1). Joseph-form covariance update, Cholesky solve, chi-squared innovation gate.
- **ZUPT and magnetometer heading are additions to the original spec** and are the only reason the bias states are observable at rest.
- Alignment: `WaitGnss` → `Align` (2 s of static samples for accel levelling and gyro bias) → `Run`. No propagation happens during `Align`.

### Display pages

1 SD capacity · **2 INS** · 3 GNSS summary · 4 per-satellite SNR · 5 IMU · 6 trajectory. PWR cycles; touch unused.

- **Page 2 (INS)** — one status line plus two 13-character columns: lat/lon/h/vN/vE/vD/roll/pitch/yaw on the left, the six biases and the live IMU/EKF rates and INS stack watermark on the right. Bias units are m/s² and **deg/s**, as specified.
- **Page 6 (trajectory)** — north vs east in metres from the origin (the first valid fix), symmetric auto-scale, origin cross-hair. Points are appended at 5 Hz into a 2048-entry ring, decimated to ≤400 segments per redraw.
- Page 3 shows the 27 strongest fresh satellites (3 columns × 9 rows), sorted by SNR descending.
- Page 2 and 3 are at the 10-row/20 px ceiling on a 240 px panel. Adding a line overflows.

### CSV schema (44 columns)

Original 28 columns, then:

```
ins_lat_deg,ins_lon_deg,ins_hgt_wgs84_m,ins_vn_ms,ins_ve_ms,ins_vd_ms,
ins_roll_deg,ins_pitch_deg,ins_yaw_deg,
ins_bax_ms2,ins_bay_ms2,ins_baz_ms2,ins_bgx_degs,ins_bgy_degs,ins_bgz_degs,
ins_status
```

- `ins_status` bit-packed: bits 0-1 mode (0 wait, 1 align, 2 run), bit 2 ZUPT active, bit 3 mag active.
- Columns are only ever **appended**, so existing indices stay valid. Update **both** `CSV_HEADER` and the `printf` in `logNavRow()`. 44 columns / 49 specifiers: `pc_time_s`, `date` and `time` each consume several.
- `ins_hgt_wgs84_m` is ellipsoidal, matching the mechanisation.

## Gotchas / known quirks

- **The LCD and microSD share SPI2_HOST and GPIO35.** Every display and SD access must hold `g_spiMutex`. Removing it will corrupt SD reads.
- **Satellites are keyed on (constellation, PRN), never PRN alone.** PRNs collide across constellations.
- **Arduino injects auto-generated prototypes before the first function definition in the sketch.** Every function must sit *below* the type definitions, or those prototypes reference undeclared types (`'DateTimeFields' was not declared`).
- **The INS bootstraps velocity from the first valid GNSS fix.** The INS always starts at rest, so a platform already moving presents a velocity innovation far outside the gate; rejecting it leaves the filter free-running with a growing covariance until a later fix is accepted at a huge gain, which is how the first implementation diverged. See `updateGnss()`.
- **The attitude correction sign is tied to the sign of the `-[f^n x]` coupling in F, and is validated dynamically** by `testAttitudeObservability`, which induces a known attitude error and checks GNSS aiding pulls it back. Flipping it makes the loop positive-feedback.
- **GNSS latency replay must write the corrected trajectory back into the history ring.** Without the write-back the next fix replays from a stale state and silently discards the correction just applied. Replay is also disabled while stationary, because interleaved ZUPT/mag updates would be undone.
- **`IMU_TO_BODY` in `ins_ekf.h` must be verified on hardware.** The INS expects the accelerometer to read approximately `(0, 0, -g)` in body FRD at rest, because a stationary accelerometer measures specific force, which points up. The measured alignment means are printed on the boot line `ALIGN acc=(...)` and shown in `Output::align_acc_mean`. Fix the mapping there, not in the filter.
- `UBX-CFG-GNSS` is volatile, re-sent every boot, and must never be followed by `UBX-CFG-CFG`. It uses `numTrkChUse = 0xFF` and a reserved-channel sum of 17 so both 24- and 32-channel parts accept it.
- Only `GPGSV` etc. per-constellation talkers carry satellite identity. A `GNGSV` stream needs `UBX-CFG-NMEA` (`mainTalkerId = 0`), not a parser change.
- **The BMI270 ODR is not set by M5Unified** and defaults to roughly 100 Hz, which caps the propagation rate at ~100 Hz. The measured rate is reported in `TEL imu=..Hz` telemetry and on page 2.
- Don't add blocking work to any task loop; each uses `vTaskDelayUntil` and the GNSS task drains the UART inline, so a stalled loop **drops** sentences.
- `drawLine()` formats into a 96-byte buffer. At text size 2 the panel fits about 26 characters.
- A card removed mid-session stops logging: `logTask` clears `sdCardFlag` and logs once.

## Conventions

- `ins_ekf.{h,cpp}` must stay free of Arduino dependencies so the host tests keep working.
- Keep the 15×15 filter matrices as **file-static**, not stack locals: `applyUpdate` peaks at ~4.5 KB of locals, which would otherwise dominate the 8 KB `insTask` stack.
- Style: two-space indent, `constexpr` configuration at the top, plain structs, `static` free functions.
- Preserve the "don't overwrite good data with a gap" rule: a field is committed only when present **and** plausible.
- Debug logging goes through `debugPrintf()`, gated by `DEBUG_SERIAL`. The 5 s `TEL`/one-shot `ALIGN` lines are the only way to verify on-target rates and the axis mapping without a screen.
- Git: work on `master`; remote is GitHub `lbyRepo/M5Stack-GNSS-Logger`. Commit messages are short and descriptive.
