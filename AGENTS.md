# AGENTS.md

## Project overview

Arduino firmware for an **M5Stack CoreS3 GNSS logger**. It reads NMEA sentences from an M5Stack GNSS module over UART, parses them with TinyGPSPlus, shows live data on the CoreS3 LCD across 4 pages, and logs GNSS + IMU samples to a CSV file on microSD.

The entire firmware is one sketch: `sensorLogger/sensorLogger.ino`. There is no test suite, CI, or build config in the repo.

## Hardware

| Part | Notes |
|---|---|
| M5Stack CoreS3 | ESP32-S3, LCD with touch, PWR button, onboard IMU |
| M5Stack GNSS module | NMEA output, wired to UART2 @ 38400 8N1 |
| microSD card | SPI mode; logs are written here |

Pin definitions (top of the sketch):

| Signal | GPIO |
|---|---|
| SD SPI SCK | 36 |
| SD SPI MISO | 35 |
| SD SPI MOSI | 37 |
| SD SPI CS | 4 |
| GNSS `RxPin` (ESP32 RX) | 18 |
| GNSS `TxPin` (ESP32 TX) | 17 |

## Build and flash

- Use the **Arduino IDE** (there is no arduino-cli config or PlatformIO setup in the repo). Open the `sensorLogger/` folder itself, not the repo root: Arduino requires the sketch folder name to match the `.ino` name.
- Board package: **M5Stack**; select the `M5CoreS3` board. The sketch uses `M5Unified`/`M5GFX` directly (the `M5CoreS3` wrapper library is not required).
- Library: **TinyGPSPlus** from Library Manager is sufficient. The sketch uses only stock APIs, including `TinyGPSCustom` for GPGSV fields and for the GGA geoid separation, so no patched/forked library is needed.
- `TinyGPSPlus` and `TinyGPSPlus-ESP32` both ship a `TinyGPSPlus.h`. With both installed the build prints `Multiple libraries were found for "TinyGPSPlus.h"`, but it deterministically selects stock `TinyGPSPlus` and ignores the fork, so this is a cosmetic warning rather than a silent mis-selection. Verified with arduino-cli 1.5.1 against `m5stack:esp32` 2.1.4.
- Compile check with the arduino-cli bundled with Arduino IDE 2, which lives under the IDE install at `resources/app/lib/backend/resources/arduino-cli.exe`:
  `arduino-cli compile --fqbn m5stack:esp32:m5stack_cores3 sensorLogger`
- Serial debug @ 115200, gated by `DEBUG_SERIAL`.

## Architecture

`setup()`:
1. Initializes M5 (display, IMU, buttons), Serial (debug), Serial2 (GNSS), SD over SPI.
2. Registers `TinyGPSCustom` handlers: GPGSV satellite fields (4 sats per sentence) and GGA field 11 (geoid separation) for every common talker id (`GP`/`GL`/`GA`/`GB`/`GN`).
3. Picks `/log.csv`; if it already exists, tries `/log_1.csv`, `/log_2.csv`, ... and writes the CSV header. The file stays open for the session.

`loop()` (non-blocking; ends with `delay(1)` to yield):
1. `M5.update()` and PWR button page cycling.
2. `pumpGps()` drains `Serial2` into TinyGPSPlus on every iteration.
3. `readGnssData()` commits parsed values into `navData` and refreshes satellite freshness stamps.
4. `updateImu()` (capped at 100 Hz) and `updateSdInfo()` (1 Hz).
5. Redraws the current page every `DISPLAY_REFRESH_MS` (200 ms).
6. Appends a CSV row every `SD_CARD_LOGGING_RATE_MS` (100 ms) and flushes every 5 s.

### Key functions

| Function | Purpose |
|---|---|
| `pumpGps` | Feeds `Serial2` bytes into TinyGPSPlus |
| `readGnssData` | Copies latest NMEA values into `GnssData`: positions, UTC/local time, numsat, speed/heading, per-satellite GPGSV data, geoid/WGS84 height |
| `updateSatellite` / `isSatFresh` | Upsert satellites by PRN with a `last_seen_ms` stamp; stale satellites are hidden |
| `updateImu` | `M5.Imu.update()` / `getImuData()` / `getTemp()` into accel [m/s^2], gyro [rad/s], mag [uT], temperature |
| `updateSdInfo` | Caches SD used/total bytes once per second |
| `openLogFile` / `logNavRow` | Opens the next free log file; writes one CSV row (kept open, periodic flush) |
| `drawPage*` / `drawCurrentPage` / `drawHeader` | Render the LCD pages; content area is cleared per refresh |
| `applyTimezoneOffset` | Local-time shift with day/month/year rollover |

### Display pages

`enum Page`: 1 = SD card capacity, 2 = GNSS summary, 3 = per-satellite SNR (fresh sats with SNR > 0 only, max 2 columns x 9 rows), 4 = IMU summary. `PAGE_COUNT` is the sentinel; `MAX_PAGE` is derived from it. Page switching is via the PWR button; touch input is unused.

### CSV log format

Header (kept in `CSV_HEADER`; must match `logNavRow()`):

```
pc_time_s,utc_time,date,time,numsat,lat_deg,lon_deg,hgt_msl_m,hgt_wgs84_m,geoid_separation_m,speed_ms,heading_deg,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads,mx_uT,my_uT,mz_uT,temp_degC
```

- `pc_time_s` = seconds since boot with millisecond precision (e.g. `12.345`, from `millis()`); `utc_time` = HHMMSS; `date` = D/M/Y and `time` = H:M:S in local time (UTC + `TIMEZONE_OFFSET_HRS`, default +8).
- IMU columns: accel [m/s^2], gyro [rad/s], mag [uT], temperature [degC].
- Each row carries the latest GNSS sample plus the current IMU sample, so GNSS values repeat between fixes (or stay 0 until the first fix).
- 22 columns. `pc_time_s`, `date` and `time` each consume several `printf` specifiers, so the 27 format arguments do not map one-to-one onto columns.
- If you add fields, update **both** `CSV_HEADER` and the `printf` in `logNavRow()`; keep column order consistent.

## Gotchas / known quirks

- Only `GPGSV` is parsed for per-satellite data. In multi-GNSS mode a receiver may also emit `GLGSV`/`GAGSV`/`GBGSV`; those are currently ignored, so the SNR page and `sats_in_view` are GPS-only (`numsat` from GGA covers all constellations).
- Geoid separation uses one `TinyGPSCustom` per talker id (`GGA_TALKERS`); if the receiver emits a different talker, add it there. This exists because stock TinyGPSPlus has no geoid field.
- Satellites are hidden from the SNR page if not seen for `SAT_FRESH_MS` (5 s).
- `hgt_wgs84_m` is `hgt_msl_m + geoid_separation_m` when the geoid value is valid; otherwise it follows MSL.
- The timezone shift handles day/month/year rollover, but only whole-hour offsets.
- Don't add long blocking work (`delay`, busy loops) to `loop()`: it stalls the UART pump and the display/log cadence.
- `drawLine()` formats into a 96-byte buffer; keep display lines short.
- Only one GGA geoid value is applied per epoch (`break` after the first updated talker).
- `TinyGPSCustom::value()` clears the object's `updated` flag. Code that tests `isUpdated()` and then calls `value()` therefore sees each sentence once; other readers of the same sentence are unaffected because they have their own flags.
- The satellite table never compacts: `sat_count` only grows, and `updateSatellite()` drops a new PRN when all `MAX_SATELLITES` slots are taken. Harmless while only GPS is parsed (~32 PRNs), but revisit before adding other constellations.
- Date commits on RMC only, while time commits on either GGA or RMC, so the `date` column can lag by a day if the receiver emits GGA but no RMC.
- A card removed during a session is not detected: `logFile` stays open and writes fail silently.

## Conventions

- Single translation unit: keep firmware code in `sensorLogger/sensorLogger.ino` unless there is a reason to split.
- Style: two-space indent, `constexpr` configuration at the top, plain structs (`SatInfo`, `GnssData`, `ImuData`, `NavData`), `static` free functions.
- Debug logging goes through `debugPrintf()` (USB serial only) and is compile-time-ish gated by `DEBUG_SERIAL`.
- Git: work on `master`; remote is GitHub `lbyRepo/M5Stack-GNSS-Logger`. Commit messages are short and descriptive.
