# AGENTS.md

## Project overview

Arduino firmware for an **M5Stack CoreS3 GNSS logger**. It reads NMEA 0183 sentences from a **u-blox M9N** GNSS module over UART, parses them with an in-sketch parser (no parsing library), shows live data on the CoreS3 LCD across 4 pages, and logs GNSS + IMU samples to a CSV file on microSD.

Satellites from every constellation the receiver reports are tracked separately — **GPS, Galileo, BeiDou, GLONASS, QZSS and SBAS**.

The entire firmware is one sketch: `sensorLogger/sensorLogger.ino`. There is no test suite, CI, or build config in the repo.

## Hardware

| Part | Notes |
|---|---|
| M5Stack CoreS3 | ESP32-S3, LCD with touch, PWR button, onboard IMU |
| u-blox M9N GNSS module | NMEA output, wired to UART2 @ 38400 8N1 |
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
- Board package: **M5Stack**; select the `M5CoreS3` board.
- Libraries: only **M5Unified** + **M5GFX** are needed, and they are the vendor board support package for the LCD, AXP2101 PMU, AW9523B IO expander and BMI270/BMM150 IMU. `SPI`, `SD` and `FS` ship inside the board package.
- **There is no NMEA parsing library dependency.** Do not re-add TinyGPSPlus: the parser lives in the sketch. (Historical note: the project previously used TinyGPSPlus, and both `TinyGPSPlus` and `TinyGPSPlus-ESP32` may still be installed in `Documents/Arduino/libraries`. Neither is used by this sketch, and both will show up as "Multiple libraries were found" noise only if something includes `TinyGPSPlus.h` again.)
- Compile check with the arduino-cli bundled with Arduino IDE 2, which lives under the IDE install at `resources/app/lib/backend/resources/arduino-cli.exe`:
  `arduino-cli compile --fqbn m5stack:esp32:m5stack_cores3 sensorLogger`
- Serial debug @ 115200, gated by `DEBUG_SERIAL`.

## Architecture

`setup()`:
1. Optionally runs the NMEA parser self-test (`NMEA_SELFTEST`), then clears `navData`.
2. Initializes M5 (display, IMU, buttons) and Serial (debug).
3. Opens `Serial2` on the GNSS module.
4. Sends `UBX-CFG-GNSS` (unless `UBX_CONFIGURE_GNSS` is false) to enable GPS/SBAS/Galileo/BeiDou/QZSS/GLONASS, then discards anything the receiver emitted meanwhile.
5. Picks `/log.csv`; if it already exists, tries `/log_1.csv`, `/log_2.csv`, ... and writes the CSV header. The file stays open for the session.

`loop()` (non-blocking; ends with `delay(1)` to yield):
1. `M5.update()` and PWR button page cycling.
2. Drains `Serial2` straight into `nmeaFeed()` — the parser runs on the bytes.
3. `expireInViewCounts()` drops in-view counts for constellations that stopped reporting.
4. `updateImu()` (capped at 100 Hz) and `updateSdInfo()` (1 Hz).
5. Redraws the current page every `DISPLAY_REFRESH_MS` (200 ms).
6. Appends a CSV row every `SD_CARD_LOGGING_RATE_MS` (100 ms) and flushes every 5 s.

### NMEA parser

Only three sentence types are needed, so the parser is a sentence splitter, not a library:

| Sentence | Fields consumed |
|---|---|
| `GGA` | 1 time, 2/3 lat, 4/5 lon, 6 fix quality, 7 sats used, 9 altitude, 11 geoid separation |
| `RMC` | 1 time, 2 status, 3/4 lat, 5/6 lon, 7 speed (knots), 8 course, 9 date |
| `GSV` | 1 message count, 3 sats in view, then 4 fields per satellite: PRN, elevation, azimuth, SNR |

Rules that matter when changing it:

- A sentence must carry a **valid checksum** (XOR of the bytes between `$` and `*`) to be accepted at all.
- An **overlong sentence is discarded**, never truncated, so a partial sentence cannot be parsed.
- Parsing is in place: `*` is overwritten with `\0` and commas become `\0`, with `nmeaFields[]` pointing at each field.
- Field access is **1-based** via `nmeaField()`; an absent or empty field returns `nullptr`.
- `strtoul` stops at the decimal point, so an `hhmmss.ss` time field already yields `hhmmss` — do not divide by 100. `setUtcTime()` range-checks the digits before committing.
- The sentence type is matched with `strncmp(..., 3)`, because the pointer used for matching sits in the middle of the line.

Talker id → constellation: `GP`→GPS, `GA`→Galileo, `GB` (and the legacy `BD`)→BeiDou, `GL`→GLONASS, `GQ`→QZSS. A `GN` talker means the receiver is using merged numbering and cannot be attributed — such satellites are tracked as `CONST_UNKNOWN` and a one-time serial warning is emitted.

### Key functions

| Function | Purpose |
|---|---|
| `nmeaFeed` | Consumes one UART byte; assembles, checksum-checks and dispatches sentences |
| `handleGga` / `handleRmc` / `handleGsv` | Commit parsed fields into `navData`; GSV declares its own in-view count and then feeds `updateSatellite` |
| `setUtcTime` / `refreshLocalTime` / `applyTimezoneOffset` | Time-of-day handling and the local-time shift with day/month/year rollover |
| `updateSatellite` / `findSatSlot` / `isSatFresh` | Upsert satellites by **(constellation, PRN)** with a `last_seen_ms` stamp; stale satellites are hidden |
| `expireInViewCounts` | Zeroes per-constellation in-view counts older than `SAT_FRESH_MS` |
| `ubxSend` / `ubxConfigureGnss` | Builds and transmits a UBX frame; sends `UBX-CFG-GNSS` once at boot |
| `updateImu` | `M5.Imu.update()` / `getImuData()` / `getTemp()` into accel [m/s^2], gyro [rad/s], mag [uT], temperature |
| `updateSdInfo` | Caches SD used/total bytes once per second |
| `openLogFile` / `logNavRow` | Opens the next free log file; writes one CSV row (kept open, periodic flush) |
| `drawPage*` / `drawCurrentPage` / `drawHeader` | Render the LCD pages; content area is cleared per refresh |
| `runNmeaSelftest` | Device-side parser regression test (see below) |

### Display pages

`enum Page`: 1 = SD card capacity, 2 = GNSS summary, 3 = per-satellite SNR, 4 = IMU summary. `PAGE_COUNT` is the sentinel; `MAX_PAGE` is derived from it. `PWR` cycles pages; touch input is unused.

- **Page 2 (GNSS summary)** is at its 10-line ceiling (9 rows of 20 px starting at `CONTENT_Y` on a 240 px panel). Line 4 carries the per-constellation breakdown as `Invw G/E/C/R: n/n/n/n`. Adding an 11th line will run off the bottom.
- **Page 3 (SNR)** shows a single header line (`SNR G01 E08 C12 R07`, which doubles as the in-view summary) followed by a **3 column x 9 row** grid, entries formatted `G12 45` (constellation letter, PRN, SNR in dBHz). Only fresh satellites with SNR > 0 are shown, **sorted by SNR descending**, so when more than `MAX_SNR_SHOWN` (27) are visible the strongest are always the ones displayed.

### CSV log format

Header (kept in `CSV_HEADER`; must match `logNavRow()`):

```
pc_time_s,utc_time,date,time,numsat,lat_deg,lon_deg,hgt_msl_m,hgt_wgs84_m,geoid_separation_m,speed_ms,heading_deg,ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads,mx_uT,my_uT,mz_uT,temp_degC,inview_gps,inview_gal,inview_bds,inview_glo,inview_qzss,inview_sbas
```

- `pc_time_s` = seconds since boot with millisecond precision (e.g. `12.345`, from `millis()`); `utc_time` = HHMMSS; `date` = D/M/Y and `time` = H:M:S in local time (UTC + `TIMEZONE_OFFSET_HRS`, default +8).
- IMU columns: accel [m/s^2], gyro [rad/s], mag [uT], temperature [degC].
- The six trailing `inview_*` columns are satellites **in view** per constellation, from GSV. `numsat` remains satellites **used** in the fix, from GGA, all constellations combined.
- 28 columns. `pc_time_s`, `date` and `time` each consume several `printf` specifiers, so the 33 format arguments do not map one-to-one onto columns.
- New columns are only ever **appended**, so existing column indices stay valid. If you add fields, update **both** `CSV_HEADER` and the `printf` in `logNavRow()`.

### Self-test

`NMEA_SELFTEST` (default `false`) runs canned sentences through the parser at boot and reports `NMEA self-test: N passed, M failed` on the serial console. It covers checksum accept/reject, truncated and overlong sentences, lat/lon/altitude/geoid, speed/heading, UTC time and date, the timezone shift, and constellation attribution — including that GPS, Galileo and GLONASS **PRN 12 all occupy distinct slots**, and that PRNs 120+ inside `GPGSV` are classified as SBAS. There is no host test harness; this runs on the device.

## Gotchas / known quirks

- **Satellites are keyed on (constellation, PRN), never PRN alone.** PRNs collide across constellations: GPS PRN 12 and Galileo PRN 12 are different satellites. Any change that keys on PRN only will silently merge unrelated satellites.
- The satellite table never compacts in place; when all `MAX_SATELLITES` (96) slots are taken, `updateSatellite()` evicts the **stalest** entry. `prn == 0` marks a free slot, so removing a satellite means zeroing its `prn`.
- **SBAS satellites arrive inside `GPGSV`**, numbered 120+, which is why `handleGsv` reclassifies them.
- `GNGSV` cannot be attributed to a constellation — the receiver is using merged numbering. Fix it at the receiver with `UBX-CFG-NMEA` (`mainTalkerId = 0`, `gsvTalkerId = 0`) rather than in the parser.
- **`UBX-CFG-GNSS` is volatile.** It is re-sent on every boot and must never be followed by `UBX-CFG-CFG`, which would write flash on every boot for no benefit.
- The UBX payload uses `numTrkChUse = 0xFF` ("use all hardware channels") and a reserved-channel sum of 17, so the same bytes are accepted by both 24- and 32-channel receivers. u-center's defaults sum to 28 and would be rejected by a 24-channel part. u-blox also requires `maxTrkCh >= 4` for every enabled major GNSS, and GPS/QZSS to be enabled or disabled together.
- UBX **output** is left disabled on the receiver, so nothing acknowledges the configuration. Success is confirmed by `GAGSV`/`GBGSV` appearing on the SNR page and in the `inview_*` columns.
- `hgt_wgs84_m` is `hgt_msl_m + geoid_separation_m` when the geoid value is valid; otherwise it follows MSL.
- Geoid separation comes straight from GGA field 11 now, so the old per-talker `TinyGPSCustom` workaround is gone.
- Date commits on RMC only, while time commits on either GGA or RMC, so the `date` column can lag by a day if the receiver emits GGA but no RMC.
- The timezone shift handles day/month/year rollover, but only whole-hour offsets.
- Don't add long blocking work (`delay`, busy loops) to `loop()`: the parser consumes the UART inline, so a stalled loop **drops** sentences rather than merely delaying them.
- `drawLine()` formats into a 96-byte buffer; keep display lines short. At text size 2 the panel fits about 26 characters per line.
- **Every function must sit below the type definitions.** Arduino injects auto-generated prototypes immediately before the first function definition in the sketch, so a function placed above the structs makes those prototypes reference undeclared types (`'DateTimeFields' was not declared`, `'SatInfo' does not name a type`). If a build fails that way, look for a function defined above the `Data types` section rather than suspecting the type itself.
- A card removed during a session is not detected: `logFile` stays open and writes fail silently.
- Four constellations roughly triple the GSV traffic. At 1 Hz the estimate is ~1.1 KB/s against 3.84 KB/s at 38400 baud, so the log and redraw cadence is unaffected — but if the link is ever pushed harder, raising `GNSS_BAUD` (and the module to match) is the lever.

## Conventions

- Single translation unit: keep firmware code in `sensorLogger/sensorLogger.ino` unless there is a reason to split.
- Style: two-space indent, `constexpr` configuration at the top, plain structs (`SatInfo`, `GnssData`, `ImuData`, `NavData`), `static` free functions.
- Preserve the "don't overwrite good data with a gap" rule: a field is committed only when it is present **and** plausible, so a temporary dropout never zeroes the last good value.
- Debug logging goes through `debugPrintf()` (USB serial only) and is gated by `DEBUG_SERIAL`.
- Git: work on `master`; remote is GitHub `lbyRepo/M5Stack-GNSS-Logger`. Commit messages are short and descriptive.
