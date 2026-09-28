/*
 * M5Stack CoreS3 GNSS Logger
 *
 * Reads NMEA sentences from a u-blox M9N GNSS module on UART2, parses them
 * with an in-sketch NMEA 0183 parser (no external parsing library), displays
 * GNSS + IMU data on the CoreS3 screen (4 pages, PWR button cycles) and logs
 * timestamped samples to a CSV file on microSD.
 *
 * Satellites from every constellation the receiver reports are tracked
 * separately: GPS, Galileo, BeiDou, GLONASS, QZSS and SBAS.
 *
 * See AGENTS.md for hardware wiring, NMEA coverage and the CSV format.
 */

#include <M5Unified.h>
#include <SPI.h>
#include <SD.h>

#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Display
constexpr uint32_t DISPLAY_REFRESH_MS = 1000;  // screen redraw interval
constexpr int      PIXEL_HEIGHT       = 20;   // line height at text size 2
constexpr int      LEFT_X             = 1;
constexpr int      CONTENT_Y          = 2 * PIXEL_HEIGHT;  // below page header

// SD card logging
constexpr uint32_t SD_CARD_LOGGING_RATE_MS = 100;   // one CSV row per interval
constexpr uint32_t SD_FLUSH_INTERVAL_MS    = 5000;  // flush the open log file
constexpr uint32_t SD_INFO_INTERVAL_MS     = 1000;  // refresh capacity numbers

// Sensors
constexpr uint32_t IMU_INTERVAL_MS = 10;    // IMU reads capped at 100 Hz
constexpr uint32_t SAT_FRESH_MS    = 5000;  // drop satellite data older than this

// Satellite tracking. Four concurrent constellations can legitimately put more
// than 40 satellites in view, so the table is sized for the worst case and
// evicts the stalest entry rather than dropping new ones.
constexpr int MAX_SATELLITES   = 96;
constexpr int MAX_SNR_ROWS     = 9;   // rows per SNR column
constexpr int SNR_COLUMN_COUNT = 3;   // 3 x 9 = 27 strongest satellites shown
constexpr int MAX_SNR_SHOWN    = MAX_SNR_ROWS * SNR_COLUMN_COUNT;

// Physical constants
constexpr float CONST_G      = 9.80665f;   // standard gravity [m/s^2]
constexpr float KNOTS_TO_MPS = 0.514444f;  // RMC reports speed in knots

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

// u-blox receiver configuration (UBX-CFG-GNSS), sent once after boot.
// Set to false to leave the receiver's stored configuration untouched.
// The change is volatile RAM only and is re-applied on every boot.
constexpr bool     UBX_CONFIGURE_GNSS  = true;
constexpr uint32_t UBX_CONFIG_DELAY_MS = 1500;  // let the receiver start up

// Set to false to silence the USB serial debug output.
constexpr bool DEBUG_SERIAL = true;

// Set to true to run the NMEA parser self-test at boot (see runNmeaSelftest).
constexpr bool NMEA_SELFTEST = false;

// Byte size units
constexpr uint64_t KILO_BYTE = 1024ULL;
constexpr uint64_t MEGA_BYTE = KILO_BYTE * 1024ULL;
constexpr uint64_t GIGA_BYTE = MEGA_BYTE * 1024ULL;

// CSV columns. Keep in sync with logNavRow(). The per-constellation in-view
// columns are appended last so existing column indices stay valid.
constexpr char CSV_HEADER[] =
    "pc_time_s,utc_time,date,time,numsat,lat_deg,lon_deg,hgt_msl_m,"
    "hgt_wgs84_m,geoid_separation_m,speed_ms,heading_deg,"
    "ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads,mx_uT,my_uT,mz_uT,"
    "temp_degC,inview_gps,inview_gal,inview_bds,inview_glo,"
    "inview_qzss,inview_sbas\n";

// ---------------------------------------------------------------------------
// Constellations
// ---------------------------------------------------------------------------

// Letters follow the RINEX convention so they are unambiguous on screen.
enum Constellation : uint8_t {
  CONST_GPS = 0,   // G
  CONST_GALILEO,   // E
  CONST_BEIDOU,    // B
  CONST_GLONASS,   // R
  CONST_QZSS,      // J
  CONST_SBAS,      // S
  CONST_UNKNOWN,   // ?  (a "GN" talker, i.e. merged numbering)
  CONST_COUNT,
};

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
  uint8_t constellation;
  uint8_t prn;            // NMEA satellite number within its constellation
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
  bool date_valid;
  uint32_t utc_time;                // UTC time of day, HHMMSS
  double lat_deg;
  double lon_deg;
  float hgt_msl_m;
  float hgt_wgs84_m;
  float geoid_separation_m;
  float speed_ms;
  float heading_deg;
  uint8_t numsat;                   // satellites used in the fix (GGA)
  uint8_t in_view[CONST_COUNT];     // satellites in view per constellation
  uint32_t in_view_ms[CONST_COUNT]; // when each count was last reported
  DateTimeFields utc;               // raw UTC date/time as received
  DateTimeFields local;             // utc shifted by TIMEZONE_OFFSET_HRS
};

struct NavData {
  ImuData imu;
  GnssData gnss;
  SatInfo sats[MAX_SATELLITES];
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

// RINEX-style constellation letter, used on the SNR page.
//
// ORDERING MATTERS: Arduino injects auto-generated function prototypes just
// before the first function definition in the sketch. Every function must
// therefore sit *below* the type definitions above, or those prototypes will
// reference types that are not declared yet.
static char constellationLetter(uint8_t constellation) {
  switch (constellation) {
    case CONST_GPS:     return 'G';
    case CONST_GALILEO: return 'E';
    case CONST_BEIDOU:  return 'B';
    case CONST_GLONASS: return 'R';
    case CONST_QZSS:    return 'J';
    case CONST_SBAS:    return 'S';
    default:            return '?';
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

// Recomputes the local timestamp. GGA carries the time and RMC the date, so
// this runs whenever either one changes, and only once a date is known.
static void refreshLocalTime() {
  GnssData &gnss = navData.gnss;
  if (!gnss.date_valid) return;
  DateTimeFields local = gnss.utc;
  applyTimezoneOffset(local, TIMEZONE_OFFSET_HRS);
  gnss.local = local;
}

// ---------------------------------------------------------------------------
// Satellite tracking
// ---------------------------------------------------------------------------
//
// Satellites are keyed on (constellation, prn), never on prn alone: GPS PRN 12,
// Galileo PRN 12 and BeiDou PRN 12 are three different satellites.

static int findSatSlot(uint8_t constellation, uint8_t prn) {
  for (int i = 0; i < MAX_SATELLITES; ++i) {
    if (navData.sats[i].prn == prn &&
        navData.sats[i].constellation == constellation) {
      return i;
    }
  }
  return -1;
}

static void updateSatellite(uint8_t constellation, uint8_t prn,
                            uint8_t elevation, uint8_t azimuth, uint8_t snr,
                            uint32_t now) {
  if (prn == 0) return;

  int index = findSatSlot(constellation, prn);

  if (index < 0) {
    // Prefer a free slot (prn == 0 marks an empty entry).
    for (int i = 0; i < MAX_SATELLITES; ++i) {
      if (navData.sats[i].prn == 0) {
        index = i;
        break;
      }
    }
  }

  if (index < 0) {
    // Table full: evict the entry whose data is oldest.
    index = 0;
    for (int i = 1; i < MAX_SATELLITES; ++i) {
      if ((int32_t)(navData.sats[i].last_seen_ms -
                    navData.sats[index].last_seen_ms) < 0) {
        index = i;
      }
    }
  }

  SatInfo &sat = navData.sats[index];
  sat.constellation = constellation;
  sat.prn = prn;
  sat.elevation_deg = elevation;
  sat.azimuth_deg = azimuth;
  sat.snr_dB = snr;
  sat.last_seen_ms = now;
}

static bool isSatFresh(const SatInfo &sat, uint32_t now) {
  return sat.prn != 0 && sat.snr_dB > 0 && sat.last_seen_ms != 0 &&
         (uint32_t)(now - sat.last_seen_ms) < SAT_FRESH_MS;
}

// Drops per-constellation in-view counts for constellations that stopped
// reporting, so neither the screen nor the CSV shows a stale count.
static void expireInViewCounts(uint32_t now) {
  for (int c = 0; c < CONST_COUNT; ++c) {
    if (navData.gnss.in_view[c] != 0 &&
        (uint32_t)(now - navData.gnss.in_view_ms[c]) >= SAT_FRESH_MS) {
      navData.gnss.in_view[c] = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// NMEA 0183 parser
// ---------------------------------------------------------------------------
//
// Replaces TinyGPSPlus. Only GGA, RMC and GSV are needed, so this is a
// sentence splitter rather than a general-purpose library.
//
// Robustness rules: a sentence must carry a valid checksum to be accepted at
// all, and an overlong sentence is discarded rather than truncated. Line noise
// -- or UBX binary, if the receiver is ever configured to emit it -- is
// therefore rejected instead of being mis-parsed.

constexpr size_t NMEA_LINE_MAX   = 100;
constexpr int    NMEA_MAX_FIELDS = 24;

static char    nmeaLine[NMEA_LINE_MAX];
static uint8_t nmeaLen = 0;
static bool    nmeaCollecting = false;

// Split-sentence state, valid only while a handler runs.
static char *nmeaFields[NMEA_MAX_FIELDS];
static int   nmeaFieldCount = 0;

// Diagnostics
static uint32_t nmeaBadChecksum = 0;
static bool     nmeaMergedTalkerWarned = false;

static void nmeaReset() {
  nmeaLen = 0;
  nmeaCollecting = false;
  nmeaFieldCount = 0;
}

static int nmeaHexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Field access is 1-based: field 1 is the first field after the sentence id.
// An absent or empty field yields nullptr, which callers read as "no value".
static const char *nmeaField(int oneBased) {
  if (oneBased < 1 || oneBased > nmeaFieldCount) return nullptr;
  const char *field = nmeaFields[oneBased - 1];
  return (*field == '\0') ? nullptr : field;
}

static bool nmeaUint(int oneBased, uint32_t &out) {
  const char *field = nmeaField(oneBased);
  if (field == nullptr) return false;
  char *end = nullptr;
  const unsigned long value = strtoul(field, &end, 10);
  if (end == field) return false;
  out = (uint32_t)value;
  return true;
}

static bool nmeaFloat(int oneBased, double &out) {
  const char *field = nmeaField(oneBased);
  if (field == nullptr) return false;
  char *end = nullptr;
  const double value = strtod(field, &end);
  if (end == field) return false;
  out = value;
  return true;
}

// NMEA latitude/longitude: ddmm.mmmm or dddmm.mmmm plus a hemisphere field.
static bool nmeaDegrees(int fieldIndex, int hemisphereIndex, double &out) {
  const char *field = nmeaField(fieldIndex);
  if (field == nullptr) return false;
  char *end = nullptr;
  const double raw = strtod(field, &end);
  if (end == field) return false;

  const double degrees = floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  if (minutes < 0.0 || minutes >= 60.0) return false;

  double value = degrees + minutes / 60.0;
  const char *hemisphere = nmeaField(hemisphereIndex);
  if (hemisphere != nullptr && (*hemisphere == 'S' || *hemisphere == 'W')) {
    value = -value;
  }
  out = value;
  return true;
}

static uint8_t constellationFromTalker(const char *talker) {
  if (talker[0] == 'B' && talker[1] == 'D') return CONST_BEIDOU;  // legacy
  if (talker[0] != 'G') return CONST_UNKNOWN;
  switch (talker[1]) {
    case 'P': return CONST_GPS;
    case 'A': return CONST_GALILEO;
    case 'B': return CONST_BEIDOU;
    case 'L': return CONST_GLONASS;
    case 'Q': return CONST_QZSS;
    default:  return CONST_UNKNOWN;   // includes the merged "GN" talker
  }
}

// Time of day arrives as hhmmss.ss. strtoul stops at the decimal point, so it
// already yields hhmmss; the digits are range-checked before being committed.
static void setUtcTime(uint32_t hhmmss) {
  const uint32_t hour = hhmmss / 10000UL;
  const uint32_t minute = (hhmmss / 100UL) % 100UL;
  const uint32_t second = hhmmss % 100UL;
  if (hour > 23 || minute > 59 || second > 59) return;

  GnssData &gnss = navData.gnss;
  gnss.utc_time = hhmmss;
  gnss.utc.hour = (uint8_t)hour;
  gnss.utc.minute = (uint8_t)minute;
  gnss.utc.second = (uint8_t)second;
  refreshLocalTime();
}

static void handleGga() {
  GnssData &gnss = navData.gnss;

  // Field 1: UTC time of day. Committed even without a fix.
  uint32_t rawTime = 0;
  if (nmeaUint(1, rawTime)) setUtcTime(rawTime);

  // Field 6: fix quality, 0 means no fix.
  uint32_t quality = 0;
  const bool hasFix = nmeaUint(6, quality) && quality > 0;

  // Fields 2/3 and 4/5: latitude and longitude.
  double lat = 0.0;
  double lon = 0.0;
  if (hasFix && nmeaDegrees(2, 3, lat) && nmeaDegrees(4, 5, lon) &&
      lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
    gnss.lat_deg = lat;
    gnss.lon_deg = lon;
    gnss.position_valid = true;
  }

  // Field 7: satellites used in the fix, all constellations combined.
  uint32_t used = 0;
  if (nmeaUint(7, used) && used <= 255) {
    gnss.numsat = (uint8_t)used;
  }

  // Field 9: altitude above mean sea level.
  double altitude = 0.0;
  if (hasFix && nmeaFloat(9, altitude)) {
    gnss.hgt_msl_m = (float)altitude;
    gnss.height_valid = true;
    gnss.hgt_wgs84_m =
        gnss.hgt_msl_m + (gnss.geoid_valid ? gnss.geoid_separation_m : 0.0f);
  }

  // Field 11: geoid separation. Reading it directly here is why the firmware
  // no longer needs a custom handler per GGA talker id.
  double geoid = 0.0;
  if (nmeaFloat(11, geoid)) {
    gnss.geoid_separation_m = (float)geoid;
    gnss.geoid_valid = true;
    if (gnss.height_valid) {
      gnss.hgt_wgs84_m = gnss.hgt_msl_m + gnss.geoid_separation_m;
    }
  }
}

static void handleRmc() {
  GnssData &gnss = navData.gnss;

  // Field 1: UTC time of day.
  uint32_t rawTime = 0;
  if (nmeaUint(1, rawTime)) setUtcTime(rawTime);

  // Field 9: date, ddmmyy.
  uint32_t rawDate = 0;
  if (nmeaUint(9, rawDate) && rawDate >= 100000UL) {
    const uint8_t day = (uint8_t)(rawDate / 10000UL);
    const uint8_t month = (uint8_t)((rawDate / 100UL) % 100UL);
    const uint16_t year = (uint16_t)(2000UL + (rawDate % 100UL));
    if (day >= 1 && day <= 31 && month >= 1 && month <= 12) {
      gnss.utc.day = day;
      gnss.utc.month = month;
      gnss.utc.year = year;
      gnss.date_valid = true;
      refreshLocalTime();
    }
  }

  // Field 2: status. 'A' means the rest of the sentence is usable.
  const char *status = nmeaField(2);
  if (status == nullptr || status[0] != 'A') return;

  // Fields 3/4 and 5/6: latitude and longitude.
  double lat = 0.0;
  double lon = 0.0;
  if (nmeaDegrees(3, 4, lat) && nmeaDegrees(5, 6, lon) &&
      lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
    gnss.lat_deg = lat;
    gnss.lon_deg = lon;
    gnss.position_valid = true;
  }

  // Field 7: speed over ground, in knots.
  double knots = 0.0;
  if (nmeaFloat(7, knots) && knots >= 0.0) {
    gnss.speed_ms = (float)(knots * KNOTS_TO_MPS);
  }

  // Field 8: course over ground, in degrees.
  double course = 0.0;
  if (nmeaFloat(8, course) && course >= 0.0 && course <= 360.0) {
    gnss.heading_deg = (float)course;
  }
}

// GSV carries up to four satellite blocks per sentence, each being
//   prn, elevation, azimuth, snr
static void handleGsv(uint32_t now) {
  const uint8_t talkerConstellation = constellationFromTalker(nmeaLine);

  // Field 3: satellites in view for this constellation. Every GSV sentence of
  // a constellation repeats the same total, so the latest value wins.
  uint32_t inView = 0;
  if (nmeaUint(3, inView)) {
    if (talkerConstellation == CONST_UNKNOWN) {
      if (!nmeaMergedTalkerWarned) {
        nmeaMergedTalkerWarned = true;
        debugPrintf(
            "GSV talker '%c%c' carries no constellation id; set CFG-NMEA "
            "mainTalkerId=0 for per-constellation GSV\n",
            nmeaLine[0], nmeaLine[1]);
      }
    } else if (inView <= 255) {
      navData.gnss.in_view[talkerConstellation] = (uint8_t)inView;
      navData.gnss.in_view_ms[talkerConstellation] = now;
    }
  }

  for (int i = 0; i < 4; ++i) {
    uint32_t prn = 0;
    if (!nmeaUint(4 + 4 * i, prn)) continue;
    if (prn == 0 || prn > 255) continue;

    uint8_t constellation = talkerConstellation;
    // u-blox reports SBAS satellites inside GPGSV, numbered from 120 upwards.
    if (constellation == CONST_GPS && prn >= 120) constellation = CONST_SBAS;

    uint32_t elevation = 0;
    uint32_t azimuth = 0;
    uint32_t snr = 0;
    (void)nmeaUint(5 + 4 * i, elevation);
    (void)nmeaUint(6 + 4 * i, azimuth);
    (void)nmeaUint(7 + 4 * i, snr);

    updateSatellite(constellation, (uint8_t)prn, (uint8_t)elevation,
                    (uint8_t)azimuth, (uint8_t)snr, now);
  }
}

// Splits the comma-separated payload in place. `body` points just past the
// 5-character sentence id (2-character talker plus 3-character type).
static void nmeaSplitFields(char *body) {
  nmeaFieldCount = 0;
  nmeaFields[nmeaFieldCount++] = body;
  for (char *p = body; *p != '\0'; ++p) {
    if (*p != ',') continue;
    *p = '\0';
    if (nmeaFieldCount < NMEA_MAX_FIELDS) {
      nmeaFields[nmeaFieldCount++] = p + 1;
    }
  }
}

static void nmeaHandleSentence(uint32_t now) {
  nmeaLine[nmeaLen] = '\0';

  // A usable sentence is at least a 5-character id plus "*CS".
  if (nmeaLen < 9) return;

  char *star = strrchr(nmeaLine, '*');
  if (star == nullptr || (star - nmeaLine) < 6) return;

  const int high = nmeaHexVal(star[1]);
  const int low = nmeaHexVal(star[2]);
  if (high < 0 || low < 0) return;

  // The checksum is the XOR of every byte between '$' and '*'.
  uint8_t sum = 0;
  for (const char *p = nmeaLine; p < star; ++p) {
    sum ^= (uint8_t)*p;
  }
  if (sum != (uint8_t)((high << 4) | low)) {
    ++nmeaBadChecksum;
    return;
  }

  // Terminate the payload in place, then dispatch on the sentence type.
  // `type` points into the middle of the line, so compare 3 characters only.
  *star = '\0';
  const char *type = nmeaLine + 2;  // skip the 2-character talker id

  // Fields begin after the first comma. That comma follows the 5-character
  // sentence id (2-character talker plus 3-character type), so the first field
  // starts at offset 6, not 5.
  char *body = strchr(nmeaLine, ',');
  if (body == nullptr) return;

  if (strncmp(type, "GGA", 3) == 0) {
    nmeaSplitFields(body + 1);
    handleGga();
  } else if (strncmp(type, "RMC", 3) == 0) {
    nmeaSplitFields(body + 1);
    handleRmc();
  } else if (strncmp(type, "GSV", 3) == 0) {
    nmeaSplitFields(body + 1);
    handleGsv(now);
  }
}

static void nmeaFeed(char c, uint32_t now) {
  if (c == '$') {
    nmeaCollecting = true;
    nmeaLen = 0;
    return;
  }
  if (!nmeaCollecting) return;
  if (c == '\r') return;

  if (c == '\n') {
    if (nmeaLen > 0) nmeaHandleSentence(now);
    nmeaCollecting = false;
    nmeaLen = 0;
    return;
  }

  if (nmeaLen >= NMEA_LINE_MAX - 1) {
    // Overlong: abandon the sentence rather than parse a truncated one.
    nmeaCollecting = false;
    nmeaLen = 0;
    return;
  }
  nmeaLine[nmeaLen++] = c;
}

// ---------------------------------------------------------------------------
// u-blox UBX configuration
// ---------------------------------------------------------------------------

// UBX frame: B5 62, class, id, length (little endian), payload, CK_A, CK_B.
// The checksum is a Fletcher-16 over everything after the two sync bytes.
static void ubxSend(uint8_t msgClass, uint8_t msgId, const uint8_t *payload,
                    uint16_t length) {
  const uint8_t header[6] = {0xB5, 0x62, msgClass, msgId,
                             (uint8_t)(length & 0xFF),
                             (uint8_t)(length >> 8)};

  uint8_t ckA = 0;
  uint8_t ckB = 0;
  for (int i = 2; i < 6; ++i) {
    ckA = (uint8_t)(ckA + header[i]);
    ckB = (uint8_t)(ckB + ckA);
  }
  for (uint16_t i = 0; i < length; ++i) {
    ckA = (uint8_t)(ckA + payload[i]);
    ckB = (uint8_t)(ckB + ckA);
  }

  Serial2.write(header, sizeof(header));
  if (length > 0) Serial2.write(payload, length);
  Serial2.write(ckA);
  Serial2.write(ckB);
  Serial2.flush();
}

// UBX-CFG-GNSS (class 0x06, id 0x3E). The payload is a 4-byte header followed
// by one 8-byte block per constellation:
//   gnssId, resTrkCh, maxTrkCh, reserved, flags (u32 little endian, bit0 = on)
//
// numTrkChUse = 0xFF means "use every hardware tracking channel", so the same
// payload works on a 24- or a 32-channel receiver. The reserved-channel sum is
// 17, deliberately below u-center's defaults (28) for the same reason.
//
// u-blox constraints honoured: at least one major GNSS stays enabled, every
// enabled major GNSS gets maxTrkCh >= 4, and GPS/QZSS are enabled together to
// avoid cross-correlation.
static void ubxConfigureGnss() {
  static const uint8_t payload[] = {
      0x00, 0x00, 0xFF, 0x07,  // msgVer, numTrkChHw, numTrkChUse, numBlocks
      0x00, 0x04, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,  // GPS
      0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,  // SBAS
      0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,  // Galileo
      0x03, 0x04, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,  // BeiDou
      0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01,  // IMES (disabled)
      0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,  // QZSS
      0x06, 0x04, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01,  // GLONASS
  };
  ubxSend(0x06, 0x3E, payload, sizeof(payload));
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
      "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,"
      "%u,%u,%u,%u,%u,%u\n",
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
      imu.temperature_degC,
      gnss.in_view[CONST_GPS], gnss.in_view[CONST_GALILEO],
      gnss.in_view[CONST_BEIDOU], gnss.in_view[CONST_GLONASS],
      gnss.in_view[CONST_QZSS], gnss.in_view[CONST_SBAS]);

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
  // Per-constellation in-view counts. RINEX letters: G/E/B/R.
  drawLine(LEFT_X, y, "Invw G/E/B/R: %u/%u/%u/%u", gnss.in_view[CONST_GPS],
           gnss.in_view[CONST_GALILEO], gnss.in_view[CONST_BEIDOU],
           gnss.in_view[CONST_GLONASS]);
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
  const GnssData &gnss = navData.gnss;

  // The title doubles as the per-constellation in-view summary.
  drawLine(LEFT_X, CONTENT_Y, "NUMSAT G:%02u E:%02u B:%02u R:%02u",
           gnss.in_view[CONST_GPS], gnss.in_view[CONST_GALILEO],
           gnss.in_view[CONST_BEIDOU], gnss.in_view[CONST_GLONASS]);

  // Keep the strongest MAX_SNR_SHOWN satellites, sorted by SNR descending, so
  // a busy multi-constellation sky still shows the most useful entries.
  const SatInfo *visible[MAX_SNR_SHOWN];
  int shown = 0;

  for (int i = 0; i < MAX_SATELLITES; ++i) {
    const SatInfo &sat = navData.sats[i];
    if (!isSatFresh(sat, now)) continue;
    if (shown == MAX_SNR_SHOWN &&
        sat.snr_dB <= visible[MAX_SNR_SHOWN - 1]->snr_dB) {
      continue;
    }

    int pos = (shown < MAX_SNR_SHOWN) ? shown : MAX_SNR_SHOWN - 1;
    while (pos > 0 && visible[pos - 1]->snr_dB < sat.snr_dB) {
      visible[pos] = visible[pos - 1];
      --pos;
    }
    visible[pos] = &sat;
    if (shown < MAX_SNR_SHOWN) ++shown;
  }

  const int columnWidth = M5.Display.width() / SNR_COLUMN_COUNT;
  for (int i = 0; i < shown; ++i) {
    const int column = i / MAX_SNR_ROWS;
    const int row = i % MAX_SNR_ROWS;
    drawLine(LEFT_X + column * columnWidth,
             CONTENT_Y + PIXEL_HEIGHT + row * PIXEL_HEIGHT, "%c%02u %2u",
             constellationLetter(visible[i]->constellation),
             visible[i]->prn, visible[i]->snr_dB);
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
// NMEA parser self-test
// ---------------------------------------------------------------------------
//
// There is no host-side test harness in this repo, so the fixtures run on the
// device. Set NMEA_SELFTEST to true and watch the serial console.

static void selftestCheck(bool ok, const char *name, int &passed, int &failed) {
  if (ok) {
    ++passed;
  } else {
    ++failed;
    debugPrintf("  FAIL: %s\n", name);
  }
}

static void selftestFeed(const char *sentence) {
  for (const char *p = sentence; *p != '\0'; ++p) {
    nmeaFeed(*p, millis());
  }
  nmeaFeed('\n', millis());
}

static void runNmeaSelftest() {
  int passed = 0;
  int failed = 0;
  debugPrintf("NMEA self-test:\n");

  const uint32_t badBefore = nmeaBadChecksum;

  // GGA: position, altitude, geoid, satellite count, UTC time.
  selftestFeed(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
  const GnssData &g = navData.gnss;
  selftestCheck(g.position_valid, "GGA sets position_valid", passed, failed);
  selftestCheck(fabs(g.lat_deg - 48.1173) < 1e-4, "GGA latitude", passed, failed);
  selftestCheck(fabs(g.lon_deg - 11.516667) < 1e-4, "GGA longitude", passed, failed);
  selftestCheck(fabs(g.hgt_msl_m - 545.4f) < 0.01f, "GGA MSL height", passed, failed);
  selftestCheck(fabs(g.geoid_separation_m - 46.9f) < 0.01f,
                "GGA geoid separation", passed, failed);
  selftestCheck(fabs(g.hgt_wgs84_m - 592.3f) < 0.01f,
                "GGA WGS84 height = MSL + geoid", passed, failed);
  selftestCheck(g.numsat == 8, "GGA satellites used", passed, failed);
  selftestCheck(g.utc_time == 123519UL, "GGA UTC time", passed, failed);

  // RMC: date, speed, heading and the timezone shift.
  // NMEA carries a 2-digit year, and the parser maps it with 2000 + yy, which
  // is what TinyGPSPlus did as well.
  selftestFeed(
      "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230326,003.1,W*7D");
  selftestCheck(g.date_valid, "RMC sets date_valid", passed, failed);
  selftestCheck(g.utc.year == 2026 && g.utc.month == 3 && g.utc.day == 23,
                "RMC date", passed, failed);
  selftestCheck(fabs(g.speed_ms - 22.4 * KNOTS_TO_MPS) < 0.01f,
                "RMC speed converted to m/s", passed, failed);
  selftestCheck(fabs(g.heading_deg - 84.4f) < 0.01f, "RMC course", passed, failed);
  selftestCheck(g.local.hour == 20 && g.local.minute == 35 && g.local.second == 19,
                "local time = UTC + offset", passed, failed);
  selftestCheck(g.local.day == 23 && g.local.month == 3 && g.local.year == 2026,
                "local date unchanged by a +8h shift", passed, failed);

  // A +8h shift must roll the date over at month and year boundaries.
  selftestFeed("$GNRMC,200000,A,4807.038,N,01131.000,E,000.0,000.0,310326,,*06");
  selftestCheck(g.local.year == 2026 && g.local.month == 4 && g.local.day == 1 &&
                    g.local.hour == 4,
                "local date rolls into the next month", passed, failed);
  selftestFeed("$GNRMC,200000,A,4807.038,N,01131.000,E,000.0,000.0,311226,,*06");
  selftestCheck(g.local.year == 2027 && g.local.month == 1 && g.local.day == 1 &&
                    g.local.hour == 4,
                "local date rolls into the next year", passed, failed);

  // GSV: four constellations, including a GPS/Galileo/GLONASS PRN 12 collision.
  selftestFeed(
      "$GPGSV,1,1,04,03,03,111,15,04,15,270,22,12,45,090,30,13,06,292,31*7A");
  selftestCheck(g.in_view[CONST_GPS] == 4, "GPGSV in-view count", passed, failed);
  selftestCheck(findSatSlot(CONST_GPS, 13) >= 0, "GPGSV satellite stored",
                passed, failed);
  const int gps12 = findSatSlot(CONST_GPS, 12);
  selftestCheck(gps12 >= 0 && navData.sats[gps12].snr_dB == 30,
                "GPGSV PRN 12 SNR", passed, failed);

  selftestFeed(
      "$GAGSV,1,1,04,11,45,180,42,12,30,090,45,19,60,270,35,36,20,315,31*65");
  selftestCheck(g.in_view[CONST_GALILEO] == 4, "GAGSV in-view count",
                passed, failed);
  selftestCheck(g.in_view[CONST_GPS] == 4,
                "GAGSV leaves the GPS count alone", passed, failed);
  const int gal12 = findSatSlot(CONST_GALILEO, 12);
  selftestCheck(gal12 >= 0 && navData.sats[gal12].snr_dB == 45,
                "GAGSV PRN 12 SNR", passed, failed);
  // The whole reason for keying on (constellation, prn): these must not merge.
  selftestCheck(gal12 >= 0 && gps12 >= 0 && gal12 != gps12,
                "GPS PRN 12 and Galileo PRN 12 are distinct slots", passed,
                failed);

  selftestFeed("$GBGSV,1,1,03,07,55,120,44,08,40,200,39,13,25,300,33*5E");
  selftestCheck(g.in_view[CONST_BEIDOU] == 3, "GBGSV in-view count",
                passed, failed);
  selftestCheck(findSatSlot(CONST_BEIDOU, 7) >= 0, "GBGSV satellite stored",
                passed, failed);

  selftestFeed("$GLGSV,1,1,02,12,50,100,41,13,35,250,29*6D");
  selftestCheck(g.in_view[CONST_GLONASS] == 2, "GLGSV in-view count",
                passed, failed);
  const int glo12 = findSatSlot(CONST_GLONASS, 12);
  selftestCheck(glo12 >= 0 && glo12 != gps12 && glo12 != gal12,
                "GLONASS PRN 12 is a third distinct slot", passed, failed);

  // PRNs 120-158 inside GPGSV are SBAS, not GPS.
  selftestFeed("$GPGSV,1,1,01,137,40,200,38*70");
  selftestCheck(findSatSlot(CONST_SBAS, 137) >= 0,
                "GPGSV PRN 137 classified as SBAS", passed, failed);

  // A merged "GN" talker carries no constellation id, so its satellites must
  // not be attributed to a real constellation.
  selftestFeed("$GNGSV,1,1,01,12,50,100,40*55");
  selftestCheck(findSatSlot(CONST_UNKNOWN, 12) >= 0,
                "GNGSV tracked as unknown constellation", passed, failed);
  selftestCheck(findSatSlot(CONST_GPS, 12) == gps12,
                "GNGSV does not overwrite the GPS PRN 12 slot", passed, failed);

  // A corrupted checksum must be rejected outright.
  selftestFeed(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00");
  selftestCheck(nmeaBadChecksum == badBefore + 1, "bad checksum rejected",
                passed, failed);

  // Truncated and overlong sentences must be ignored, not mis-parsed.
  selftestFeed("$GPGGA,123519,4807.038,N");
  selftestFeed(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
      "0000000000000000000000000000000000000000000000000000000000000*47");
  selftestCheck(nmeaBadChecksum == badBefore + 1,
                "malformed sentences add no checksum errors", passed, failed);

  debugPrintf("NMEA self-test: %d passed, %d failed\n", passed, failed);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  navData = {};

  if (NMEA_SELFTEST) {
    runNmeaSelftest();
    navData = {};
    nmeaReset();
  }

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setTextSize(2);

  delay(200);
  if (!M5.Imu.begin()) {
    debugPrintf("IMU init failed\n");
  }

  Serial2.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

  if (UBX_CONFIGURE_GNSS) {
    // Wait for the receiver to finish its own start-up before reconfiguring it.
    delay(UBX_CONFIG_DELAY_MS);
    ubxConfigureGnss();
    debugPrintf("UBX-CFG-GNSS sent (GPS/SBAS/Galileo/BeiDou/QZSS/GLONASS)\n");
    // Drop anything the receiver emitted while we were configuring it.
    while (Serial2.available()) Serial2.read();
    nmeaReset();
  }

  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
  sdCardFlag =
      SD.begin(SD_SPI_CS_PIN, SPI, 25000000) && SD.cardType() != CARD_NONE;
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

  // Drain the GNSS UART every iteration: the parser consumes the bytes, so a
  // stalled loop would drop sentences rather than merely delay them.
  while (Serial2.available()) {
    nmeaFeed((char)Serial2.read(), now);
  }

  expireInViewCounts(now);
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
