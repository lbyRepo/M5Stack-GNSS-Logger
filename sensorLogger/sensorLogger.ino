/*
 * M5Stack CoreS3 GNSS Logger
 *
 * Reads NMEA sentences from an M5Stack GNSS module on UART2, parses them
 * with TinyGPSPlus, displays GNSS + IMU data on the CoreS3 screen (4 pages,
 * PWR button cycles) and logs timestamped samples to a CSV file on microSD.
 *
 * See AGENTS.md for hardware wiring and the CSV format.
 */

#include <M5Unified.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Display
constexpr uint32_t DISPLAY_REFRESH_MS = 200;  // screen redraw interval
constexpr int      PIXEL_HEIGHT       = 20;   // line height at text size 2
constexpr int      LEFT_X             = 1;
constexpr int      CONTENT_Y          = 2 * PIXEL_HEIGHT;  // below page header

// SD card logging
constexpr uint32_t SD_CARD_LOGGING_RATE_MS = 100;   // one CSV row per interval
constexpr uint32_t SD_FLUSH_INTERVAL_MS    = 5000;  // flush the open log file
constexpr uint32_t SD_INFO_INTERVAL_MS     = 1000;  // refresh capacity numbers

// Sensors
constexpr uint32_t IMU_INTERVAL_MS  = 10;    // IMU reads capped at 100 Hz
constexpr uint32_t SAT_FRESH_MS     = 5000;  // hide satellites not seen recently
constexpr int      MAX_SATELLITES   = 40;    // tracked satellite slots
constexpr int      MAX_SNR_ROWS     = 9;     // rows per SNR column
constexpr int      SNR_COLUMN_COUNT = 2;

// Physical constants
constexpr float CONST_G = 9.80665f;  // standard gravity [m/s^2]

// Local time = UTC + this offset. Set to 0 to keep everything in UTC.
constexpr int TIMEZONE_OFFSET_HRS = 8;

// M5Stack CoreS3 pin map
constexpr int SD_SPI_SCK_PIN  = 36;
constexpr int SD_SPI_MISO_PIN = 35;
constexpr int SD_SPI_MOSI_PIN = 37;
constexpr int SD_SPI_CS_PIN   = 4;
constexpr int GNSS_RX_PIN     = 18;  // ESP32 RX  (GNSS module TX)
constexpr int GNSS_TX_PIN     = 17;  // ESP32 TX  (GNSS module RX)
constexpr uint32_t GNSS_BAUD  = 38400;

// Set to false to silence the USB serial debug output.
constexpr bool DEBUG_SERIAL = true;

// Byte size units
constexpr uint64_t KILO_BYTE = 1024ULL;
constexpr uint64_t MEGA_BYTE = KILO_BYTE * 1024ULL;
constexpr uint64_t GIGA_BYTE = MEGA_BYTE * 1024ULL;

// CSV columns. Keep in sync with logNavRow().
constexpr char CSV_HEADER[] =
    "pc_time_s,utc_time,date,time,numsat,lat_deg,lon_deg,hgt_msl_m,"
    "hgt_wgs84_m,geoid_separation_m,speed_ms,heading_deg,"
    "ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads,mx_uT,my_uT,mz_uT,"
    "temp_degC\n";

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

struct DateTimeFields {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

struct SatInfo {
  uint8_t prn;            // NMEA satellite number
  uint8_t elevation_deg;
  uint8_t azimuth_deg;
  uint8_t snr_dB;
  uint32_t last_seen_ms;
};

struct ImuData {
  float time_s;
  float acc_ms2[3];
  float gyr_rads[3];
  float mag_uT[3];
  float temperature_degC;
};

struct GnssData {
  bool position_valid;
  bool height_valid;
  bool geoid_valid;
  uint32_t utc_time;         // UTC time of day, HHMMSS
  double lat_deg;
  double lon_deg;
  float hgt_msl_m;
  float hgt_wgs84_m;
  float geoid_separation_m;
  float speed_ms;
  float heading_deg;
  uint8_t numsat;            // satellites used (GGA)
  uint8_t sats_in_view;      // GPS satellites in view (GPGSV)
  DateTimeFields local;      // UTC shifted by TIMEZONE_OFFSET_HRS
};

struct NavData {
  ImuData imu;
  GnssData gnss;
  SatInfo sats[MAX_SATELLITES];
  uint8_t sat_count;
};

enum Page : uint8_t {
  PAGE_SD_CARD = 1,
  PAGE_GNSS_SUMMARY,
  PAGE_SNR,
  PAGE_IMU_SUMMARY,
  PAGE_COUNT,  // keep last
};
constexpr uint8_t MAX_PAGE = static_cast<uint8_t>(PAGE_COUNT) - 1;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

NavData navData = {};

TinyGPSPlus gps;

// GPGSV fields: up to 4 satellites per sentence.
TinyGPSCustom gsvTotalMessages(gps, "GPGSV", 1);
TinyGPSCustom gsvSatsInView(gps, "GPGSV", 3);
TinyGPSCustom gsvSatNumber[4];
TinyGPSCustom gsvElevation[4];
TinyGPSCustom gsvAzimuth[4];
TinyGPSCustom gsvSnr[4];

// GGA geoid separation (field 11) for every talker id a multi-GNSS receiver
// may use. This uses the stock TinyGPSCustom API, so no patched TinyGPSPlus
// fork is required.
const char *const GGA_TALKERS[] = {"GPGGA", "GLGGA", "GAGGA", "GBGGA", "GNGGA"};
constexpr size_t GGA_TALKER_COUNT = sizeof(GGA_TALKERS) / sizeof(GGA_TALKERS[0]);
TinyGPSCustom geoidSeparation[GGA_TALKER_COUNT];

// UI state
uint8_t currentPage = PAGE_SD_CARD;

// SD card state
bool sdCardFlag = false;
char logFileName[32] = {0};
File logFile;
uint64_t sdUsedBytes = 0;
uint64_t sdTotalBytes = 0;
float sdCardRatio = 0.0f;

// Timers
uint32_t lastDrawMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastFlushMs = 0;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static void debugPrintf(const char *format, ...) {
  if (!DEBUG_SERIAL) return;
  char buffer[160];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

static void drawLine(int x, int y, const char *format, ...) {
  char buffer[96];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  M5.Display.setCursor(x, y);
  M5.Display.print(buffer);
}

static void clearScreen() {
  M5.Display.fillRect(0, 0, M5.Display.width(), M5.Display.height(), BLACK);
}

static void clearContentArea() {
  M5.Display.fillRect(0, CONTENT_Y, M5.Display.width(),
                      M5.Display.height() - CONTENT_Y, BLACK);
}

static void formatBytes(uint64_t bytes, char *out, size_t outLen) {
  if (bytes >= GIGA_BYTE) {
    snprintf(out, outLen, "%llu GB", (unsigned long long)(bytes / GIGA_BYTE));
  } else if (bytes >= MEGA_BYTE) {
    snprintf(out, outLen, "%llu MB", (unsigned long long)(bytes / MEGA_BYTE));
  } else if (bytes >= KILO_BYTE) {
    snprintf(out, outLen, "%llu kB", (unsigned long long)(bytes / KILO_BYTE));
  } else {
    snprintf(out, outLen, "%llu Bytes", (unsigned long long)bytes);
  }
}

// ---------------------------------------------------------------------------
// Time helpers
// ---------------------------------------------------------------------------

static bool isLeapYear(uint16_t year) {
  return (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
}

static uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t DAYS_PER_MONTH[12] = {31, 28, 31, 30, 31, 30,
                                             31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 31;
  if (month == 2 && isLeapYear(year)) return 29;
  return DAYS_PER_MONTH[month - 1];
}

// Applies a whole-hour offset, handling day/week/month/year rollover.
static void applyTimezoneOffset(DateTimeFields &dt, int offsetHours) {
  int hour = (int)dt.hour + offsetHours;
  int dayShift = 0;
  while (hour >= 24) {
    hour -= 24;
    ++dayShift;
  }
  while (hour < 0) {
    hour += 24;
    --dayShift;
  }
  dt.hour = (uint8_t)hour;

  while (dayShift > 0) {
    --dayShift;
    if (++dt.day > daysInMonth(dt.year, dt.month)) {
      dt.day = 1;
      if (++dt.month > 12) {
        dt.month = 1;
        ++dt.year;
      }
    }
  }
  while (dayShift < 0) {
    ++dayShift;
    if (--dt.day == 0) {
      if (--dt.month == 0) {
        dt.month = 12;
        --dt.year;
      }
      dt.day = daysInMonth(dt.year, dt.month);
    }
  }
}

// ---------------------------------------------------------------------------
// GNSS
// ---------------------------------------------------------------------------

static void updateSatellite(uint8_t prn, uint8_t elevation, uint8_t azimuth,
                            uint8_t snr, uint32_t now) {
  if (prn == 0) return;

  int index = -1;
  for (int i = 0; i < navData.sat_count; ++i) {
    if (navData.sats[i].prn == prn) {
      index = i;
      break;
    }
  }
  if (index < 0) {
    if (navData.sat_count >= MAX_SATELLITES) return;
    index = navData.sat_count++;
    memset(&navData.sats[index], 0, sizeof(navData.sats[index]));
    navData.sats[index].prn = prn;
  }

  SatInfo &sat = navData.sats[index];
  sat.elevation_deg = elevation;
  sat.azimuth_deg = azimuth;
  sat.snr_dB = snr;
  sat.last_seen_ms = now;
}

static bool isSatFresh(const SatInfo &sat, uint32_t now) {
  return sat.snr_dB > 0 && sat.last_seen_ms != 0 &&
         (uint32_t)(now - sat.last_seen_ms) < SAT_FRESH_MS;
}

static void pumpGps() {
  while (Serial2.available()) {
    gps.encode((char)Serial2.read());
  }
}

static void readGnssData(uint32_t now) {
  GnssData &gnss = navData.gnss;

  // UTC time (GGA/RMC) and date (RMC); the date may not be known yet.
  if (gps.time.isUpdated() && gps.time.isValid()) {
    gnss.utc_time = gps.time.value() / 100;  // HHMMSS
    if (gps.date.isValid()) {
      DateTimeFields local;
      local.year = gps.date.year();
      local.month = gps.date.month();
      local.day = gps.date.day();
      local.hour = gps.time.hour();
      local.minute = gps.time.minute();
      local.second = gps.time.second();
      applyTimezoneOffset(local, TIMEZONE_OFFSET_HRS);
      gnss.local = local;
    }
  }

  // Position
  if (gps.location.isUpdated() && gps.location.isValid()) {
    gnss.position_valid = true;
    gnss.lat_deg = gps.location.lat();
    gnss.lon_deg = gps.location.lng();
  }

  // MSL height (GGA)
  if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
    gnss.hgt_msl_m = (float)gps.altitude.meters();
    gnss.height_valid = true;
    gnss.hgt_wgs84_m = gnss.hgt_msl_m +
                       (gnss.geoid_valid ? gnss.geoid_separation_m : 0.0f);
  }

  // Satellites used (GGA)
  if (gps.satellites.isUpdated() && gps.satellites.isValid()) {
    gnss.numsat = (uint8_t)gps.satellites.value();
  }

  // Speed and heading (RMC)
  if (gps.speed.isUpdated() && gps.speed.isValid()) {
    gnss.speed_ms = (float)gps.speed.mps();
    if (gps.course.isValid()) {
      gnss.heading_deg = (float)gps.course.deg();
    }
  }

  // GPGSV: 4 satellites per sentence. value() consumes the update flag so
  // this block runs once per received GPGSV sentence.
  if (gsvTotalMessages.isUpdated() && gsvTotalMessages.isValid()) {
    (void)gsvTotalMessages.value();
    for (int i = 0; i < 4; ++i) {
      int prn = atoi(gsvSatNumber[i].value());
      if (prn <= 0) continue;
      updateSatellite((uint8_t)prn,
                      (uint8_t)atoi(gsvElevation[i].value()),
                      (uint8_t)atoi(gsvAzimuth[i].value()),
                      (uint8_t)atoi(gsvSnr[i].value()),
                      now);
    }
  }

  // Satellites in view (GPGSV field 3)
  if (gsvSatsInView.isUpdated() && gsvSatsInView.isValid()) {
    gnss.sats_in_view = (uint8_t)atoi(gsvSatsInView.value());
  }

  // Geoid separation (GGA field 11)
  for (size_t i = 0; i < GGA_TALKER_COUNT; ++i) {
    if (geoidSeparation[i].isUpdated() && geoidSeparation[i].isValid()) {
      gnss.geoid_separation_m = (float)atof(geoidSeparation[i].value());
      gnss.geoid_valid = true;
      if (gnss.height_valid) {
        gnss.hgt_wgs84_m = gnss.hgt_msl_m + gnss.geoid_separation_m;
      }
      break;  // only one GGA sentence per epoch
    }
  }
}

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------

static void updateImu(uint32_t now) {
  static uint32_t lastReadMs = 0;
  if (now - lastReadMs < IMU_INTERVAL_MS) return;
  lastReadMs = now;

  if (!M5.Imu.update()) return;

  const auto &data = M5.Imu.getImuData();
  ImuData &imu = navData.imu;

  imu.time_s = data.usec / 1000000.0f;

  imu.acc_ms2[0] = data.accel.x * CONST_G;
  imu.acc_ms2[1] = data.accel.y * CONST_G;
  imu.acc_ms2[2] = data.accel.z * CONST_G;

  imu.gyr_rads[0] = data.gyro.x * DEG_TO_RAD;
  imu.gyr_rads[1] = data.gyro.y * DEG_TO_RAD;
  imu.gyr_rads[2] = data.gyro.z * DEG_TO_RAD;

  imu.mag_uT[0] = data.mag.x;
  imu.mag_uT[1] = data.mag.y;
  imu.mag_uT[2] = data.mag.z;

  float temperature = 0.0f;
  if (M5.Imu.getTemp(&temperature)) {
    imu.temperature_degC = temperature;
  }
}

// ---------------------------------------------------------------------------
// SD card
// ---------------------------------------------------------------------------

static void updateSdInfo(uint32_t now) {
  static uint32_t lastUpdateMs = 0;
  if (now - lastUpdateMs < SD_INFO_INTERVAL_MS) return;
  lastUpdateMs = now;

  if (!sdCardFlag) {
    sdUsedBytes = sdTotalBytes = 0;
    sdCardRatio = 0.0f;
    return;
  }
  sdUsedBytes = SD.usedBytes();
  sdTotalBytes = SD.cardSize();
  sdCardRatio = (sdTotalBytes > 0)
                    ? (float)((double)sdUsedBytes / (double)sdTotalBytes)
                    : 0.0f;
}

static bool openLogFile() {
  snprintf(logFileName, sizeof(logFileName), "/log.csv");

  // Pick the first free /log_N.csv name so old logs are never overwritten.
  unsigned fileCounter = 0;
  while (SD.exists(logFileName)) {
    ++fileCounter;
    snprintf(logFileName, sizeof(logFileName), "/log_%u.csv", fileCounter);
  }

  logFile = SD.open(logFileName, FILE_WRITE);
  if (!logFile) {
    debugPrintf("Failed to open %s\n", logFileName);
    logFileName[0] = '\0';
    return false;
  }
  logFile.print(CSV_HEADER);
  debugPrintf("Logging to %s\n", logFileName);
  return true;
}

static void logNavRow(uint32_t now) {
  if (!sdCardFlag || !logFile) return;

  const GnssData &gnss = navData.gnss;
  const ImuData &imu = navData.imu;

  logFile.printf(
      "%lu.%03lu,%06u,%02u/%02u/%04u,%02u:%02u:%02u,%u,%lf,%lf,"
      "%.3f,%.3f,%.3f,%.3f,%.1f,"
      "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f\n",
      (unsigned long)(now / 1000U), (unsigned long)(now % 1000U),
      (unsigned int)gnss.utc_time,
      gnss.local.day, gnss.local.month, gnss.local.year,
      gnss.local.hour, gnss.local.minute, gnss.local.second,
      (unsigned int)gnss.numsat,
      gnss.lat_deg, gnss.lon_deg,
      gnss.hgt_msl_m, gnss.hgt_wgs84_m, gnss.geoid_separation_m,
      gnss.speed_ms, gnss.heading_deg,
      imu.acc_ms2[0], imu.acc_ms2[1], imu.acc_ms2[2],
      imu.gyr_rads[0], imu.gyr_rads[1], imu.gyr_rads[2],
      imu.mag_uT[0], imu.mag_uT[1], imu.mag_uT[2],
      imu.temperature_degC);

  if (now - lastFlushMs >= SD_FLUSH_INTERVAL_MS) {
    lastFlushMs = now;
    logFile.flush();
  }
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

static void drawHeader() {
  drawLine(LEFT_X, 0, "=== M5Stack Data Logger ===");
  drawLine(LEFT_X, PIXEL_HEIGHT, "Log File    : %s",
           sdCardFlag ? logFileName : "No SD card");
}

static void drawPageSdCard() {
  int y = CONTENT_Y;
  char text[24];

  drawLine(LEFT_X, y, "=== SD Card Log Status ===");
  y += PIXEL_HEIGHT;

  formatBytes(sdUsedBytes, text, sizeof(text));
  drawLine(LEFT_X, y, "SD used     : %s", text);
  y += PIXEL_HEIGHT;

  formatBytes(sdTotalBytes, text, sizeof(text));
  drawLine(LEFT_X, y, "SD card size: %s", text);
  y += PIXEL_HEIGHT;

  drawLine(LEFT_X, y, "SD card [%%] : %.1f %%", sdCardRatio * 100.0f);
}

static void drawPageGnss() {
  int y = CONTENT_Y;
  const GnssData &gnss = navData.gnss;

  drawLine(LEFT_X, y, "==== GNSS Summary ====");
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "UTC time    : %06u", (unsigned int)gnss.utc_time);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Numsat used : %u", (unsigned int)gnss.numsat);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Sat in view : %u", (unsigned int)gnss.sats_in_view);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Lat [deg]   : %.6f", gnss.lat_deg);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Lon [deg]   : %.6f", gnss.lon_deg);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Hgt_MSL [m] : %.3f", gnss.hgt_msl_m);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Hgt_WGS [m] : %.3f", gnss.hgt_wgs84_m);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Speed [m/s] : %.3f", gnss.speed_ms);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Heading[deg]: %.1f", gnss.heading_deg);
}

static void drawPageSnr() {
  const uint32_t now = millis();
  const int columnWidth = M5.Display.width() / SNR_COLUMN_COUNT;

  drawLine(LEFT_X, CONTENT_Y, "==== GNSS Sat SNR [dB] ====");

  int shown = 0;
  for (int i = 0; i < navData.sat_count &&
                  shown < MAX_SNR_ROWS * SNR_COLUMN_COUNT;
       ++i) {
    const SatInfo &sat = navData.sats[i];
    if (!isSatFresh(sat, now)) continue;

    const int column = shown / MAX_SNR_ROWS;
    const int row = shown % MAX_SNR_ROWS;
    drawLine(LEFT_X + column * columnWidth,
             CONTENT_Y + PIXEL_HEIGHT + row * PIXEL_HEIGHT,
             "SV %3u: %u", (unsigned int)sat.prn, (unsigned int)sat.snr_dB);
    ++shown;
  }
}

static void drawPageImu() {
  int y = CONTENT_Y;
  const ImuData &imu = navData.imu;

  drawLine(LEFT_X, y, "==== IMU %.1f degC ====", imu.temperature_degC);
  y += PIXEL_HEIGHT;

  drawLine(LEFT_X, y, "Ax [m/s2] : %.3f", imu.acc_ms2[0]);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Ay [m/s2] : %.3f", imu.acc_ms2[1]);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Az [m/s2] : %.3f", imu.acc_ms2[2]);
  y += PIXEL_HEIGHT;

  drawLine(LEFT_X, y, "Gx [deg/s]: %.3f", imu.gyr_rads[0] * RAD_TO_DEG);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Gy [deg/s]: %.3f", imu.gyr_rads[1] * RAD_TO_DEG);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Gz [deg/s]: %.3f", imu.gyr_rads[2] * RAD_TO_DEG);
  y += PIXEL_HEIGHT;

  drawLine(LEFT_X, y, "Mx [uT]   : %.3f", imu.mag_uT[0]);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "My [uT]   : %.3f", imu.mag_uT[1]);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Mz [uT]   : %.3f", imu.mag_uT[2]);
}

static void drawCurrentPage() {
  clearContentArea();
  switch (currentPage) {
    case PAGE_SD_CARD:
      drawPageSdCard();
      break;
    case PAGE_GNSS_SUMMARY:
      drawPageGnss();
      break;
    case PAGE_SNR:
      drawPageSnr();
      break;
    case PAGE_IMU_SUMMARY:
      drawPageImu();
      break;
    default:
      break;
  }
}

static void handlePageButton() {
  if (!M5.BtnPWR.wasClicked()) return;

  ++currentPage;
  if (currentPage > MAX_PAGE) {
    currentPage = PAGE_SD_CARD;
  }
  clearScreen();
  drawHeader();
  drawCurrentPage();
  debugPrintf("Page -> %u\n", (unsigned int)currentPage);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setTextSize(2);

  delay(200);
  if (!M5.Imu.begin()) {
    debugPrintf("IMU init failed\n");
  }

  Serial2.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

  for (int i = 0; i < 4; ++i) {
    gsvSatNumber[i].begin(gps, "GPGSV", 4 + 4 * i);  // satellite number
    gsvElevation[i].begin(gps, "GPGSV", 5 + 4 * i);  // elevation [deg]
    gsvAzimuth[i].begin(gps, "GPGSV", 6 + 4 * i);    // azimuth [deg]
    gsvSnr[i].begin(gps, "GPGSV", 7 + 4 * i);        // SNR [dB]
  }
  for (size_t i = 0; i < GGA_TALKER_COUNT; ++i) {
    geoidSeparation[i].begin(gps, GGA_TALKERS[i], 11);  // geoid sep. [m]
  }

  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
  sdCardFlag = SD.begin(SD_SPI_CS_PIN, SPI, 25000000) &&
               SD.cardType() != CARD_NONE;
  if (sdCardFlag) {
    sdCardFlag = openLogFile();
  }
  if (!sdCardFlag) {
    debugPrintf("No SD card / log file unavailable\n");
  }

  clearScreen();
  drawHeader();
  drawCurrentPage();
}

void loop() {
  const uint32_t now = millis();

  M5.update();
  handlePageButton();

  pumpGps();
  readGnssData(now);
  updateImu(now);
  updateSdInfo(now);

  if (now - lastDrawMs >= DISPLAY_REFRESH_MS) {
    lastDrawMs = now;
    drawCurrentPage();
  }

  if (sdCardFlag && now - lastLogMs >= SD_CARD_LOGGING_RATE_MS) {
    lastLogMs = now;
    logNavRow(now);
  }

  delay(1);  // yield to the RTOS tasks
}
