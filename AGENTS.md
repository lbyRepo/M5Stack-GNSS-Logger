# AGENTS.md

## Project overview

Arduino firmware for an **M5Stack CoreS3 GNSS logger**. It reads NMEA sentences from an M5Stack GNSS module over UART, parses them with TinyGPSPlus, shows live data on the CoreS3 LCD across 4 pages, and appends GNSS fixes to a CSV file on a microSD card. Onboard IMU data is read and displayed, but is not written to the SD card yet.

The entire firmware is one sketch: `sensorLogger.ino` (~770 lines). There is no test suite, CI, or build config in the repo.

## Hardware

| Part | Notes |
|---|---|
| M5Stack CoreS3 | ESP32-S3, LCD with touch, PWR button, onboard IMU |
| M5Stack GNSS module | u-blox, NMEA output, wired to UART2 @ 38400 8N1 |
| microSD card | SPI mode; logs are written here |

Pin definitions (top of `sensorLogger.ino`):

| Signal | GPIO |
|---|---|
| SD SPI SCK | 36 |
| SD SPI MISO | 35 |
| SD SPI MOSI | 37 |
| SD SPI CS | 4 |
| GNSS `rxPin` (ESP32 RX) | 18 |
| GNSS `txPin` (ESP32 TX) | 17 |

## Build and flash

- Use the **Arduino IDE** (there is no arduino-cli/PlatformIO setup in this repo).
- Board package: **M5Stack** board manager; select the `M5CoreS3` board.
- Libraries: `M5Unified` + `M5GFX` (from the M5Stack package) and **TinyGPSPlus** (Library Manager, by Mikal Hart).
- Arduino requires the sketch folder to match the `.ino` name (`sensorLogger/`). If the IDE offers to move/rename the folder on open, that is expected.
- Serial monitor @ 115200 for debug output.
- No compile check exists in CI; make sure the sketch compiles before considering a change done.

## Architecture

`setup()`:
1. Initializes M5 (display, IMU, buttons), Serial (debug), Serial2 (GNSS), SD over SPI.
2. Initializes the `TinyGPSCustom` handlers for `GPGSV` fields (satellite number/elevation/azimuth/SNR, 4 sats per sentence).
3. Picks `/log.csv`; if it already exists, tries `/log_1.csv`, `/log_2.csv`, ... and writes the CSV header.

`loop()` (runs continuously, no fixed rate):
1. Computes SD usage.
2. `CoreS3.BtnPWR.wasClicked()` cycles the display page (wraps 1..4).
3. `assignGnssDataStruct()` and `assignImuDataStruct()` fill the global `nav_data_struct`.
4. `printData2Screen()` renders the current page.
5. `logSdCardGnssData()` appends one CSV row.

### Key functions

| Function | Purpose |
|---|---|
| `assignGnssDataStruct` | Copies latest TinyGPSPlus values into `T_UBX_DATA_STRUCT`: position, time/date (with UTC+8 shift), numsat, speed/heading, per-satellite GPGSV fields, geoid/ellipsoid height |
| `assignImuDataStruct` | `M5.Imu.update()` / `getImuData()` into accel [m/s^2], gyro [rad/s], mag [uT] |
| `printData2Screen` | Renders the current page |
| `logSdCardGnssData` | `sprintf`s one CSV row and appends it to the log file |
| `smartDelay` | Pumps `Serial2` bytes into TinyGPSPlus for `ms` milliseconds (blocking) |
| `printf_log` / `println_log` | Serial + `canvas` output (see gotchas) |
| `listDir` / `createDir` / `removeDir` / `readFile` / `writeFile` / `appendFile` / `renameFile` / `deleteFile` / `testFileIO` | Standard SD example helpers; only `writeFile` (header) and `appendFile` (rows) are used |

### Display pages

`T_ENUM_PAGE_NUM_DISPLAY`: 1 = SD card capacity, 2 = GNSS summary, 3 = per-satellite SNR (active sats with SNR > 0 only, flows into a second column after 9 rows), 4 = IMU summary. `DISPLAY_MAX_PAGE` and the enum must stay in sync when pages are added/removed. Page switching is via the PWR button; touch input is declared but unused.

### CSV log format

Header written once per new file:

```
pc_time,TOW,date,time,numsat,lat_deg,lon_deg,hgt_msl_m,hgt_wgs84_m,geoid_separation_m,speed_ms,heading_deg
```

- `pc_time` = `millis()/1000` (seconds since boot, resets on reboot).
- `date` = D/M/Y and `time` = H:M:S, both shifted to local time by `TIMEZONE_OFFSET_HRS` (hardcoded +8).
- `TOW` = GPS time of week in seconds.
- Data rows end with a trailing comma (fields after `heading_deg` are reserved/unused).
- Only GNSS is logged; `imuData` is display-only for now.
- If you add fields, update **both** the header string in `setup()` and the `sprintf` in `logSdCardGnssData()`; keep column order consistent.

## Gotchas / known quirks

- The GNSS UART is only read inside `smartDelay()`, which is called from pages 2 and 3. On pages 1 and 4 nothing reads `Serial2`, so NMEA bytes can fill/overflow the UART FIFO and data may be stale after switching back. Keep this in mind if you remove those calls or add long blocking work.
- `smartDelay()` always blocks for the full interval (1000 ms on the GNSS/SNR pages).
- `SD_CARD_LOGGING_RATE_MS` is defined but unused; a row is currently appended every `loop()` iteration.
- `imuData.time_s` and `imuData.imu_rpy_rad` are never populated.
- `hgt_wgs84_m` is only recomputed when a new geoid value arrives (`hgt_msl_m + geoid_separation_m`); it can be stale relative to `hgt_msl_m`.
- The timezone shift only handles day rollover (`HOUR + 8 >= 24`); month/year rollover is not handled.
- `M5Canvas canvas` is declared but `createSprite()` is never called, so `printf_log()` / `println_log()` output may not render on screen.
- Arduino `printf("...")` goes to the USB serial console, not the LCD.
- Comments marked `LBY:` are author annotations; preserve or extend them when editing nearby code.

## Conventions

- Single translation unit: keep firmware code in `sensorLogger.ino` unless there is a reason to split into `.h`/`.cpp` (remember to keep the sketch folder name in mind).
- Style: two-space indent, `#define` constants at the top, `T_...` typedef structs, small integer types (`unsigned char`/`unsigned short`) for compact fields.
- Debug logging uses `printf(...)` (Serial only) or `printf_log`/`println_log` (Serial + canvas).
- Git: work on `master`; remote is GitHub `lbyRepo/M5Stack-GNSS-Logger`. Commit messages are short and descriptive.
