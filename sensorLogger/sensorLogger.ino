/*
 * M5Stack CoreS3 GNSS Logger - real-time INS edition
 *
 * Reads NMEA 0183 sentences from a u-blox M9N on UART2, tracks every
 * constellation it reports, and fuses the onboard BMI270 with GNSS in a
 * 15-state INS/GNSS Extended Kalman Filter (see ins_ekf.h).
 *
 * Runs as six concurrent FreeRTOS tasks: GNSS, IMU, INS/EKF, UI and logging,
 * plus an idle Arduino loop(). Each M5 subsystem has exactly one owning task,
 * and all display and SD traffic is serialised by a single mutex because the
 * LCD and microSD share SPI2_HOST and the LCD driver repurposes GPIO35 on
 * every chip select.
 *
 * See AGENTS.md for wiring, the CSV schema and the concurrency rules.
 */

#include <M5Unified.h>
#include <SD.h>
#include <SPI.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "ins_ekf.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Display
constexpr uint32_t UI_REFRESH_MS = 200;   // screen redraw interval (5 Hz)
constexpr int      PIXEL_HEIGHT  = 20;    // line height at text size 2
constexpr int      LEFT_X        = 1;
constexpr int      CONTENT_Y     = 2 * PIXEL_HEIGHT;  // below page header

// SD card logging
constexpr uint32_t SD_CARD_LOGGING_RATE_MS = 100;   // one CSV row per interval
constexpr uint32_t SD_FLUSH_INTERVAL_MS    = 5000;  // flush the open log file
constexpr uint32_t SD_INFO_INTERVAL_MS     = 1000;  // refresh capacity numbers

// Sensors
constexpr uint32_t IMU_POLL_MS  = 5;      // 200 Hz poll, data-ready gated
constexpr uint32_t GNSS_POLL_MS = 2;      // UART drain period
constexpr uint32_t SAT_FRESH_MS = 5000;   // drop satellite data older than this

constexpr int MAX_SATELLITES   = 96;
constexpr int MAX_SNR_ROWS     = 9;
constexpr int SNR_COLUMN_COUNT = 3;
constexpr int MAX_SNR_SHOWN    = MAX_SNR_ROWS * SNR_COLUMN_COUNT;

// Trajectory
constexpr int      TRAJ_CAP       = 2048;
constexpr uint32_t TRAJ_SAMPLE_MS = 200;   // 5 Hz record rate

// Physical constants
constexpr float CONST_G      = 9.80665f;    // standard gravity [m/s^2]
constexpr float KNOTS_TO_MPS = 0.514444f;   // RMC reports speed in knots
constexpr int   TIMEZONE_OFFSET_HRS = 8;

// M5Stack CoreS3 pin map
constexpr int SD_SPI_SCK_PIN  = 36;
constexpr int SD_SPI_MISO_PIN = 35;
constexpr int SD_SPI_MOSI_PIN = 37;
constexpr int SD_SPI_CS_PIN   = 4;
constexpr int GNSS_RX_PIN     = 18;   // ESP32 RX (GNSS module TX)
constexpr int GNSS_TX_PIN     = 17;   // ESP32 TX (GNSS module RX)
constexpr uint32_t GNSS_BAUD  = 38400;

// u-blox receiver configuration (UBX-CFG-GNSS), sent once after boot.
constexpr bool     UBX_CONFIGURE_GNSS  = true;
constexpr uint32_t UBX_CONFIG_DELAY_MS = 1500;

constexpr bool DEBUG_SERIAL  = true;
constexpr bool NMEA_SELFTEST = false;

// Byte size units
constexpr uint64_t KILO_BYTE = 1024ULL;
constexpr uint64_t MEGA_BYTE = KILO_BYTE * 1024ULL;
constexpr uint64_t GIGA_BYTE = MEGA_BYTE * 1024ULL;

// Task configuration. INS and IMU are pinned together on core 1 so the 100 Hz
// propagation is not perturbed by display or SD work; everything that touches
// SPI lives on core 0, away from the filter.
constexpr int TASK_PRIO_GNSS = 6;
constexpr int TASK_PRIO_IMU  = 6;
constexpr int TASK_PRIO_INS  = 5;
constexpr int TASK_PRIO_LOG  = 3;
constexpr int TASK_PRIO_UI   = 2;

constexpr uint32_t STACK_GNSS = 4096;
constexpr uint32_t STACK_IMU  = 3072;
constexpr uint32_t STACK_INS  = 8192;
constexpr uint32_t STACK_LOG  = 4096;
constexpr uint32_t STACK_UI   = 4096;

constexpr BaseType_t CORE_FILTER = 1;
constexpr BaseType_t CORE_IO     = 0;

// CSV columns. Keep in sync with logNavRow(). The INS columns are appended
// after the original 28 so existing column indices stay valid.
constexpr char CSV_HEADER[] =
    "pc_time_s,utc_time,date,time,numsat,lat_deg,lon_deg,hgt_msl_m,"
    "hgt_wgs84_m,geoid_separation_m,speed_ms,heading_deg,"
    "ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads,mx_uT,my_uT,mz_uT,"
    "temp_degC,inview_gps,inview_gal,inview_bds,inview_glo,"
    "inview_qzss,inview_sbas,"
    "ins_lat_deg,ins_lon_deg,ins_hgt_wgs84_m,ins_vn_ms,ins_ve_ms,ins_vd_ms,"
    "ins_roll_deg,ins_pitch_deg,ins_yaw_deg,"
    "ins_bax_ms2,ins_bay_ms2,ins_baz_ms2,ins_bgx_degs,ins_bgy_degs,"
    "ins_bgz_degs,ins_status\n";

// ---------------------------------------------------------------------------
// Constellations
// ---------------------------------------------------------------------------

// Letters follow the RINEX convention so they are unambiguous on screen.
enum Constellation : uint8_t {
  CONST_GPS = 0,   // G
  CONST_GALILEO,   // E
  CONST_BEIDOU,    // C
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
  uint8_t prn;
  uint8_t elevation_deg;
  uint8_t azimuth_deg;
  uint8_t snr_dB;
  uint32_t last_seen_ms;
};

struct ImuData {
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
  uint32_t utc_time;                // HHMMSS
  double lat_deg;
  double lon_deg;
  float hgt_msl_m;
  float hgt_wgs84_m;
  float geoid_separation_m;
  float speed_ms;
  float heading_deg;
  uint8_t numsat;
  uint8_t in_view[CONST_COUNT];
  uint32_t in_view_ms[CONST_COUNT];
  DateTimeFields utc;
  DateTimeFields local;
};

struct TrajectoryPoint {
  float north;
  float east;
};

// Published to the UI and logging tasks. Guarded by g_stateMutex.
struct AppSnapshot {
  GnssData gnss;
  SatInfo  sats[MAX_SATELLITES];
  ImuData  imu;
  ins::Output ins;

  uint32_t imu_hz;      // measured task rates
  uint32_t ins_hz;
  uint32_t gnss_hz;
  uint32_t imu_stack;   // stack high-water marks, bytes
  uint32_t ins_stack;
  uint32_t gnss_stack;
  uint32_t ui_stack;
  uint32_t log_stack;
  uint32_t log_rows;
  uint32_t log_errors;
  uint32_t gnss_sentences;
  uint32_t gnss_bad_checksum;
};

// One IMU sample plus the magnetometer reading taken at the same time.
struct ImuQueueItem {
  ins::ImuSample imu;
  float mag[3];
  uint32_t mag_t_ms;
  bool mag_valid;
};

enum Page : uint8_t {
  PAGE_SD_CARD = 1,
  PAGE_INS,          // between the SD page and the GNSS page, as requested
  PAGE_GNSS_SUMMARY,
  PAGE_SNR,
  PAGE_IMU_SUMMARY,
  PAGE_TRAJECTORY,
  PAGE_COUNT,        // keep last
};
constexpr uint8_t MAX_PAGE = static_cast<uint8_t>(PAGE_COUNT) - 1;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

// Task-owned state, published through g_snapshot under g_stateMutex.
static GnssData g_gnss = {};                    // owned by gnssTask
static SatInfo  g_sats[MAX_SATELLITES] = {};    // owned by gnssTask
static ImuData  g_imu = {};                     // owned by imuTask

static AppSnapshot g_snapshot = {};             // guarded by g_stateMutex
static AppSnapshot g_uiView = {};               // uiTask-private copy
static AppSnapshot g_logView = {};              // logTask-private copy

// Trajectory buffer, guarded by g_trajMutex.
static TrajectoryPoint g_traj[TRAJ_CAP];
static int g_trajHead = 0;
static int g_trajCount = 0;
static bool g_trajOriginSet = false;
static double g_trajLat0 = 0.0;
static double g_trajLon0 = 0.0;
static double g_trajH0 = 0.0;

// Synchronisation.
static SemaphoreHandle_t g_stateMutex = nullptr;
static SemaphoreHandle_t g_spiMutex = nullptr;   // display AND SD
static SemaphoreHandle_t g_trajMutex = nullptr;
static QueueHandle_t     g_imuQueue = nullptr;
static QueueHandle_t     g_gnssQueue = nullptr;

// UI state.
static uint8_t currentPage = PAGE_SD_CARD;

// SD card state.
static bool sdCardFlag = false;
static char logFileName[32] = {0};
static File logFile;
static uint64_t sdUsedBytes = 0;
static uint64_t sdTotalBytes = 0;
static float sdCardRatio = 0.0f;

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

// Callers must hold g_spiMutex: the LCD and the microSD share SPI2_HOST and
// the LCD driver repurposes GPIO35 on every chip select.
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
// before the first function definition in the sketch, so every function must
// sit below the type definitions or those prototypes reference undeclared
// types.
static char constellationLetter(uint8_t constellation) {
  switch (constellation) {
    case CONST_GPS:     return 'G';
    case CONST_GALILEO: return 'E';
    case CONST_BEIDOU:  return 'C';
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

static void refreshLocalTime() {
  if (!g_gnss.date_valid) return;
  DateTimeFields local = g_gnss.utc;
  applyTimezoneOffset(local, TIMEZONE_OFFSET_HRS);
  g_gnss.local = local;
}

// ---------------------------------------------------------------------------
// Satellite tracking  (owned by gnssTask)
// ---------------------------------------------------------------------------
//
// Keyed on (constellation, prn), never prn alone: GPS PRN 12, Galileo PRN 12
// and BeiDou PRN 12 are three different satellites.

static int findSatSlot(uint8_t constellation, uint8_t prn) {
  for (int i = 0; i < MAX_SATELLITES; ++i) {
    if (g_sats[i].prn == prn && g_sats[i].constellation == constellation) {
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
    for (int i = 0; i < MAX_SATELLITES; ++i) {
      if (g_sats[i].prn == 0) {
        index = i;
        break;
      }
    }
  }
  if (index < 0) {
    // Table full: evict the entry whose data is oldest.
    index = 0;
    for (int i = 1; i < MAX_SATELLITES; ++i) {
      if ((int32_t)(g_sats[i].last_seen_ms - g_sats[index].last_seen_ms) < 0) {
        index = i;
      }
    }
  }

  SatInfo &sat = g_sats[index];
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

static void expireInViewCounts(uint32_t now) {
  for (int c = 0; c < CONST_COUNT; ++c) {
    if (g_gnss.in_view[c] != 0 &&
        (uint32_t)(now - g_gnss.in_view_ms[c]) >= SAT_FRESH_MS) {
      g_gnss.in_view[c] = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// NMEA 0183 parser  (owned by gnssTask)
// ---------------------------------------------------------------------------
//
// A sentence must carry a valid checksum to be accepted at all, and an
// overlong sentence is discarded rather than truncated, so line noise or UBX
// binary is rejected instead of being mis-parsed.

constexpr size_t NMEA_LINE_MAX   = 100;
constexpr int    NMEA_MAX_FIELDS = 24;

static char    nmeaLine[NMEA_LINE_MAX];
static uint8_t nmeaLen = 0;
static bool    nmeaCollecting = false;

static char *nmeaFields[NMEA_MAX_FIELDS];
static int   nmeaFieldCount = 0;

static uint32_t nmeaBadChecksum = 0;
static uint32_t nmeaGoodSentences = 0;
static bool     nmeaMergedTalkerWarned = false;

// Incremented whenever a GGA with a usable position completes, so gnssTask can
// hand a fix to the filter exactly once per epoch.
static uint32_t nmeaFixSeq = 0;

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

// Field access is 1-based; an absent or empty field yields nullptr.
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

  g_gnss.utc_time = hhmmss;
  g_gnss.utc.hour = (uint8_t)hour;
  g_gnss.utc.minute = (uint8_t)minute;
  g_gnss.utc.second = (uint8_t)second;
  refreshLocalTime();
}

static void handleGga() {
  uint32_t rawTime = 0;
  if (nmeaUint(1, rawTime)) setUtcTime(rawTime);

  uint32_t quality = 0;
  const bool hasFix = nmeaUint(6, quality) && quality > 0;

  double lat = 0.0;
  double lon = 0.0;
  if (hasFix && nmeaDegrees(2, 3, lat) && nmeaDegrees(4, 5, lon) &&
      lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
    g_gnss.lat_deg = lat;
    g_gnss.lon_deg = lon;
    g_gnss.position_valid = true;
    ++nmeaFixSeq;   // one complete fix for this epoch
  }

  uint32_t used = 0;
  if (nmeaUint(7, used) && used <= 255) {
    g_gnss.numsat = (uint8_t)used;
  }

  double altitude = 0.0;
  if (hasFix && nmeaFloat(9, altitude)) {
    g_gnss.hgt_msl_m = (float)altitude;
    g_gnss.height_valid = true;
    g_gnss.hgt_wgs84_m =
        g_gnss.hgt_msl_m + (g_gnss.geoid_valid ? g_gnss.geoid_separation_m : 0.0f);
  }

  double geoid = 0.0;
  if (nmeaFloat(11, geoid)) {
    g_gnss.geoid_separation_m = (float)geoid;
    g_gnss.geoid_valid = true;
    if (g_gnss.height_valid) {
      g_gnss.hgt_wgs84_m = g_gnss.hgt_msl_m + g_gnss.geoid_separation_m;
    }
  }
}

static void handleRmc() {
  uint32_t rawTime = 0;
  if (nmeaUint(1, rawTime)) setUtcTime(rawTime);

  uint32_t rawDate = 0;
  if (nmeaUint(9, rawDate) && rawDate >= 100000UL) {
    const uint8_t day = (uint8_t)(rawDate / 10000UL);
    const uint8_t month = (uint8_t)((rawDate / 100UL) % 100UL);
    const uint16_t year = (uint16_t)(2000UL + (rawDate % 100UL));
    if (day >= 1 && day <= 31 && month >= 1 && month <= 12) {
      g_gnss.utc.day = day;
      g_gnss.utc.month = month;
      g_gnss.utc.year = year;
      g_gnss.date_valid = true;
      refreshLocalTime();
    }
  }

  const char *status = nmeaField(2);
  if (status == nullptr || status[0] != 'A') return;

  double lat = 0.0;
  double lon = 0.0;
  if (nmeaDegrees(3, 4, lat) && nmeaDegrees(5, 6, lon) &&
      lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
    g_gnss.lat_deg = lat;
    g_gnss.lon_deg = lon;
    g_gnss.position_valid = true;
  }

  double knots = 0.0;
  if (nmeaFloat(7, knots) && knots >= 0.0) {
    g_gnss.speed_ms = (float)(knots * KNOTS_TO_MPS);
  }

  double course = 0.0;
  if (nmeaFloat(8, course) && course >= 0.0 && course <= 360.0) {
    g_gnss.heading_deg = (float)course;
  }
}

// GSV carries up to four satellite blocks per sentence.
static void handleGsv(uint32_t now) {
  const uint8_t talkerConstellation = constellationFromTalker(nmeaLine);

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
      g_gnss.in_view[talkerConstellation] = (uint8_t)inView;
      g_gnss.in_view_ms[talkerConstellation] = now;
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
  if (nmeaLen < 9) return;

  char *star = strrchr(nmeaLine, '*');
  if (star == nullptr || (star - nmeaLine) < 6) return;

  const int high = nmeaHexVal(star[1]);
  const int low = nmeaHexVal(star[2]);
  if (high < 0 || low < 0) return;

  uint8_t sum = 0;
  for (const char *p = nmeaLine; p < star; ++p) sum ^= (uint8_t)*p;
  if (sum != (uint8_t)((high << 4) | low)) {
    ++nmeaBadChecksum;
    return;
  }
  ++nmeaGoodSentences;

  *star = '\0';
  const char *type = nmeaLine + 2;  // skip the 2-character talker id

  // Fields begin after the first comma, which follows the 5-character
  // sentence id (2-character talker plus 3-character type).
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
    nmeaCollecting = false;   // abandon rather than parse a truncated sentence
    nmeaLen = 0;
    return;
  }
  nmeaLine[nmeaLen++] = c;
}

// ---------------------------------------------------------------------------
// u-blox UBX configuration
// ---------------------------------------------------------------------------

static void ubxSend(uint8_t msgClass, uint8_t msgId, const uint8_t *payload,
                    uint16_t length) {
  const uint8_t header[6] = {0xB5, 0x62, msgClass, msgId,
                             (uint8_t)(length & 0xFF), (uint8_t)(length >> 8)};

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

// UBX-CFG-GNSS (class 0x06, id 0x3E): enable GPS, SBAS, Galileo, BeiDou, QZSS
// and GLONASS. numTrkChUse = 0xFF means "all hardware channels", so the same
// payload is accepted by 24- and 32-channel parts.
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
// SD card  (owned by logTask)
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

// Packed so a row is self-describing without a look-up table:
// bits 0-1 mode (0 wait, 1 align, 2 run), bit 2 ZUPT active, bit 3 mag active.
static uint8_t insStatus(const ins::Output &o) {
  uint8_t s = (uint8_t)o.mode;
  if (o.zupt_active) s |= 0x04;
  if (o.mag_active) s |= 0x08;
  return s;
}

// Callers must hold g_spiMutex.
static bool logNavRow(const AppSnapshot &s, uint32_t now) {
  if (!sdCardFlag || !logFile) return false;

  const GnssData &gnss = s.gnss;
  const ImuData &imu = s.imu;
  const ins::Output &io = s.ins;

  const int written = logFile.printf(
      "%lu.%03lu,%06u,%02u/%02u/%04u,%02u:%02u:%02u,%u,%lf,%lf,"
      "%.3f,%.3f,%.3f,%.3f,%.1f,"
      "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,"
      "%u,%u,%u,%u,%u,%u,"
      "%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
      "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u\n",
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
      gnss.in_view[CONST_QZSS], gnss.in_view[CONST_SBAS],
      io.lat_deg, io.lon_deg, io.h_m,
      io.vn, io.ve, io.vd,
      io.roll_deg, io.pitch_deg, io.yaw_deg,
      io.ba_ms2[0], io.ba_ms2[1], io.ba_ms2[2],
      io.bg_dps[0], io.bg_dps[1], io.bg_dps[2],
      (unsigned int)insStatus(io));

  if (written <= 0) return false;

  static uint32_t lastFlushMs = 0;
  if (now - lastFlushMs >= SD_FLUSH_INTERVAL_MS) {
    lastFlushMs = now;
    logFile.flush();
  }
  return true;
}

// ---------------------------------------------------------------------------
// Display  (owned by uiTask; callers must hold g_spiMutex)
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

// Page 2: the 15 INS states. Two columns of 13 characters at text size 2 fit
// the 320 px panel exactly.
static void drawPageIns(const AppSnapshot &s) {
  const ins::Output &o = s.ins;

  const char *mode = "WAIT";
  if (o.mode == ins::Mode::Align) mode = "ALGN";
  else if (o.mode == ins::Mode::Run) mode = "RUN";

  drawLine(LEFT_X, CONTENT_Y, "INS %s %s%s", mode,
           o.zupt_active ? "ZUPT " : "     ", o.mag_active ? "MAG" : "");

  const int y0 = CONTENT_Y + PIXEL_HEIGHT;
  const int colX = M5.Display.width() / 2;

  int y = y0;
  drawLine(LEFT_X, y, "Lat %9.5f", o.lat_deg);    y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Lon %9.5f", o.lon_deg);    y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Hgt %7.1f", o.h_m);        y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "vN  %7.2f", o.vn);         y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "vE  %7.2f", o.ve);         y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "vD  %7.2f", o.vd);         y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Rll %7.2f", o.roll_deg);   y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Pit %7.2f", o.pitch_deg);  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Yaw %7.2f", o.yaw_deg);

  y = y0;
  drawLine(colX, y, "bax %7.3f", o.ba_ms2[0]);    y += PIXEL_HEIGHT;
  drawLine(colX, y, "bay %7.3f", o.ba_ms2[1]);    y += PIXEL_HEIGHT;
  drawLine(colX, y, "baz %7.3f", o.ba_ms2[2]);    y += PIXEL_HEIGHT;
  drawLine(colX, y, "bgx %7.3f", o.bg_dps[0]);    y += PIXEL_HEIGHT;
  drawLine(colX, y, "bgy %7.3f", o.bg_dps[1]);    y += PIXEL_HEIGHT;
  drawLine(colX, y, "bgz %7.3f", o.bg_dps[2]);    y += PIXEL_HEIGHT;
  drawLine(colX, y, "IMU %4uHz", (unsigned)s.imu_hz);  y += PIXEL_HEIGHT;
  drawLine(colX, y, "EKF %4uHz", (unsigned)s.ins_hz);  y += PIXEL_HEIGHT;
  drawLine(colX, y, "stk %4uB", (unsigned)s.ins_stack);
}

static void drawPageGnss(const AppSnapshot &s) {
  int y = CONTENT_Y;
  const GnssData &gnss = s.gnss;

  drawLine(LEFT_X, y, "==== GNSS Summary ====");
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "UTC time    : %06u", (unsigned int)gnss.utc_time);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Numsat used : %u", (unsigned int)gnss.numsat);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Invw G/E/C/R: %u/%u/%u/%u", gnss.in_view[CONST_GPS],
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

static void drawPageSnr(const AppSnapshot &s) {
  const uint32_t now = millis();
  const GnssData &gnss = s.gnss;

  drawLine(LEFT_X, CONTENT_Y, "SNR G%02u E%02u C%02u R%02u",
           gnss.in_view[CONST_GPS], gnss.in_view[CONST_GALILEO],
           gnss.in_view[CONST_BEIDOU], gnss.in_view[CONST_GLONASS]);

  // Strongest first, so a busy multi-constellation sky still shows the most
  // useful entries.
  const SatInfo *visible[MAX_SNR_SHOWN];
  int shown = 0;

  for (int i = 0; i < MAX_SATELLITES; ++i) {
    const SatInfo &sat = s.sats[i];
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

static void drawPageImu(const AppSnapshot &s) {
  int y = CONTENT_Y;
  const ImuData &imu = s.imu;

  drawLine(LEFT_X, y, "==== IMU %.1f degC ====", imu.temperature_degC);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Ax [m/s2] : %.3f", imu.acc_ms2[0]);  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Ay [m/s2] : %.3f", imu.acc_ms2[1]);  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Az [m/s2] : %.3f", imu.acc_ms2[2]);  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Gx [deg/s]: %.3f", imu.gyr_rads[0] * RAD_TO_DEG);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Gy [deg/s]: %.3f", imu.gyr_rads[1] * RAD_TO_DEG);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Gz [deg/s]: %.3f", imu.gyr_rads[2] * RAD_TO_DEG);
  y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Mx [uT]   : %.3f", imu.mag_uT[0]);   y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "My [uT]   : %.3f", imu.mag_uT[1]);   y += PIXEL_HEIGHT;
  drawLine(LEFT_X, y, "Mz [uT]   : %.3f", imu.mag_uT[2]);
}

// Page 6: north vs east in metres from the origin, auto-scaled. Callers must
// hold g_spiMutex and g_trajMutex so the track cannot be appended mid-draw.
static void drawPageTrajectory() {
  const int plotX0 = 2;
  const int plotY0 = CONTENT_Y;
  const int plotW = M5.Display.width() - 4;
  const int plotH = M5.Display.height() - CONTENT_Y - 2;

  if (g_trajCount < 2) {
    drawLine(LEFT_X, CONTENT_Y, "Trajectory: waiting for INS...");
    return;
  }

  float maxAbsN = 1.0f;
  float maxAbsE = 1.0f;
  for (int i = 0; i < g_trajCount; ++i) {
    const TrajectoryPoint &p = g_traj[i];
    if (fabsf(p.north) > maxAbsN) maxAbsN = fabsf(p.north);
    if (fabsf(p.east) > maxAbsE) maxAbsE = fabsf(p.east);
  }

  // Symmetric scale so the track is not distorted.
  const float halfW = (float)(plotW / 2 - 6);
  const float halfH = (float)(plotH / 2 - 6);
  float span = (maxAbsN > maxAbsE ? maxAbsN : maxAbsE) * 1.15f;
  const float scale = (halfW < halfH ? halfW : halfH) / span;

  const int originX = plotX0 + plotW / 2;
  const int originY = plotY0 + plotH / 2;

  drawLine(LEFT_X, plotY0, "N vs E  %5.0f m/div  n=%d", span, g_trajCount);

  M5.Display.drawFastHLine(originX - 4, originY, 9, DARKGREY);
  M5.Display.drawFastVLine(originX, originY - 4, 9, DARKGREY);

  const int step = (g_trajCount > 400) ? (g_trajCount / 400) : 1;
  int prevX = 0;
  int prevY = 0;
  bool havePrev = false;
  for (int i = 0; i < g_trajCount; i += step) {
    const TrajectoryPoint &p = g_traj[i];
    const int x = originX + (int)(p.east * scale);
    const int y = originY - (int)(p.north * scale);
    if (havePrev) M5.Display.drawLine(prevX, prevY, x, y, GREEN);
    prevX = x;
    prevY = y;
    havePrev = true;
  }
}

static void drawCurrentPage() {
  clearContentArea();
  switch (currentPage) {
    case PAGE_SD_CARD:
      drawPageSdCard();
      break;
    case PAGE_INS:
      drawPageIns(g_uiView);
      break;
    case PAGE_GNSS_SUMMARY:
      drawPageGnss(g_uiView);
      break;
    case PAGE_SNR:
      drawPageSnr(g_uiView);
      break;
    case PAGE_IMU_SUMMARY:
      drawPageImu(g_uiView);
      break;
    case PAGE_TRAJECTORY:
      xSemaphoreTake(g_trajMutex, portMAX_DELAY);
      drawPageTrajectory();
      xSemaphoreGive(g_trajMutex);
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

static void publishGnss() {
  xSemaphoreTake(g_stateMutex, portMAX_DELAY);
  g_snapshot.gnss = g_gnss;
  memcpy(g_snapshot.sats, g_sats, sizeof(g_sats));
  g_snapshot.gnss_sentences = nmeaGoodSentences;
  g_snapshot.gnss_bad_checksum = nmeaBadChecksum;
  xSemaphoreGive(g_stateMutex);
}

static void gnssTask(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(GNSS_POLL_MS);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastSeq = 0;
  uint32_t lastPublishMs = 0;
  uint32_t published = 0;
  uint32_t windowStart = millis();

  for (;;) {
    const uint32_t now = millis();

    // Drain the UART on every pass: the parser consumes bytes inline, so a
    // stalled loop drops sentences rather than merely delaying them.
    while (Serial2.available()) nmeaFeed((char)Serial2.read(), now);

    expireInViewCounts(now);

    // Hand a fix to the filter exactly once per epoch.
    if (nmeaFixSeq != lastSeq) {
      lastSeq = nmeaFixSeq;
      if (g_gnss.position_valid && g_gnss.height_valid) {
        ins::GnssFix fix;
        fix.t_ms = now;
        fix.lat_rad = g_gnss.lat_deg * (M_PI / 180.0);
        fix.lon_rad = g_gnss.lon_deg * (M_PI / 180.0);
        fix.h_m = g_gnss.hgt_wgs84_m;

        // RMC gives ground speed and course. Below the gate the course is
        // meaningless, so the filter is told the velocity is unavailable.
        const bool velOk = g_gnss.speed_ms > ins::GNSS_SPEED_GATE_MS;
        const float courseRad = g_gnss.heading_deg * (float)(M_PI / 180.0);
        fix.vn = velOk ? g_gnss.speed_ms * cosf(courseRad) : 0.0f;
        fix.ve = velOk ? g_gnss.speed_ms * sinf(courseRad) : 0.0f;
        fix.velocity_valid = velOk;
        xQueueSend(g_gnssQueue, &fix, 0);

        if (!g_trajOriginSet) {
          xSemaphoreTake(g_trajMutex, portMAX_DELAY);
          g_trajLat0 = fix.lat_rad;
          g_trajLon0 = fix.lon_rad;
          g_trajH0 = fix.h_m;
          g_trajOriginSet = true;
          xSemaphoreGive(g_trajMutex);
        }
      }
    }

    if (now - lastPublishMs >= 100) {
      lastPublishMs = now;
      publishGnss();
      ++published;

      const uint32_t elapsed = now - windowStart;
      if (elapsed >= 1000) {
        xSemaphoreTake(g_stateMutex, portMAX_DELAY);
        g_snapshot.gnss_hz = (uint32_t)((uint64_t)published * 1000ull / elapsed);
        g_snapshot.gnss_stack =
            uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
        xSemaphoreGive(g_stateMutex);
        published = 0;
        windowStart = now;
      }
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

static void imuTask(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(IMU_POLL_MS);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t samples = 0;
  uint32_t windowStart = millis();

  for (;;) {
    // Data-ready gated inside M5Unified, so polling faster than the BMI270 ODR
    // simply returns nothing rather than duplicating samples.
    if (M5.Imu.update()) {
      const auto &d = M5.Imu.getImuData();

      ImuQueueItem item;
      item.imu.t_us = d.usec;
      for (int i = 0; i < 3; ++i) {
        item.imu.acc[i] = d.sensor[0].value[i] * CONST_G;
        item.imu.gyro[i] = d.sensor[1].value[i] * DEG_TO_RAD;
        item.mag[i] = d.sensor[2].value[i];
        g_imu.acc_ms2[i] = item.imu.acc[i];
        g_imu.gyr_rads[i] = item.imu.gyro[i];
        g_imu.mag_uT[i] = item.mag[i];
      }
      item.mag_t_ms = millis();
      item.mag_valid = true;

      xQueueSend(g_imuQueue, &item, 0);
      ++samples;

      const uint32_t now = millis();
      const uint32_t elapsed = now - windowStart;
      if (elapsed >= 1000) {
        xSemaphoreTake(g_stateMutex, portMAX_DELAY);
        g_snapshot.imu_hz = (uint32_t)((uint64_t)samples * 1000ull / elapsed);
        g_snapshot.imu = g_imu;
        g_snapshot.imu_stack =
            uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
        xSemaphoreGive(g_stateMutex);
        samples = 0;
        windowStart = now;
      }
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

static void recordTrajectory(double latDeg, double lonDeg, double h) {
  if (!g_trajOriginSet) return;

  static uint32_t lastSampleMs = 0;
  const uint32_t now = millis();
  if (now - lastSampleMs < TRAJ_SAMPLE_MS) return;
  lastSampleMs = now;

  // The same WGS-84 radii the filter uses, so the plot matches the INS frame.
  const double lat0 = g_trajLat0;
  const double s = sin(lat0);
  const double w = 1.0 - 6.69437999014e-3 * s * s;
  const double sw = sqrt(w);
  const double rm = 6378137.0 * (1.0 - 6.69437999014e-3) / (w * sw);
  const double rn = 6378137.0 / sw;

  const double lat = latDeg * (M_PI / 180.0);
  const double lon = lonDeg * (M_PI / 180.0);

  TrajectoryPoint p;
  p.north = (float)((lat - lat0) * (rm + h));
  p.east = (float)((lon - g_trajLon0) * (rn + h) * cos(lat0));

  xSemaphoreTake(g_trajMutex, portMAX_DELAY);
  g_traj[g_trajHead] = p;
  g_trajHead = (g_trajHead + 1) % TRAJ_CAP;
  if (g_trajCount < TRAJ_CAP) ++g_trajCount;
  xSemaphoreGive(g_trajMutex);
}

static void insTask(void *arg) {
  (void)arg;
  ImuQueueItem item;
  uint32_t steps = 0;
  uint32_t windowStart = millis();

  for (;;) {
    if (xQueueReceive(g_imuQueue, &item, pdMS_TO_TICKS(50)) == pdTRUE) {
      ins::updateImu(item.imu);

      ins::MagSample mag;
      mag.t_ms = item.mag_t_ms;
      mag.valid = item.mag_valid;
      for (int i = 0; i < 3; ++i) mag.m[i] = item.mag[i];
      ins::updateMag(mag);

      // Consume every pending fix; the newest one wins.
      ins::GnssFix fix;
      while (xQueueReceive(g_gnssQueue, &fix, 0) == pdTRUE) {
        ins::updateGnss(fix);
      }

      ++steps;
      const uint32_t now = millis();
      const uint32_t elapsed = now - windowStart;
      if (elapsed >= 1000) {
        ins::Output out;
        ins::getOutput(out);

        xSemaphoreTake(g_stateMutex, portMAX_DELAY);
        g_snapshot.ins = out;
        g_snapshot.ins_hz = (uint32_t)((uint64_t)steps * 1000ull / elapsed);
        g_snapshot.ins_stack =
            uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
        xSemaphoreGive(g_stateMutex);

        if (out.mode == ins::Mode::Run) {
          recordTrajectory(out.lat_deg, out.lon_deg, out.h_m);
        }
        steps = 0;
        windowStart = now;
      }
    }
  }
}

static void logTask(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(SD_CARD_LOGGING_RATE_MS);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t rows = 0;
  uint32_t errors = 0;

  for (;;) {
    const uint32_t now = millis();
    updateSdInfo(now);

    // Copy under the state lock, then write holding only the SPI lock.
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    g_logView = g_snapshot;
    xSemaphoreGive(g_stateMutex);

    if (sdCardFlag) {
      xSemaphoreTake(g_spiMutex, portMAX_DELAY);
      if (logNavRow(g_logView, now)) {
        ++rows;
      } else {
        ++errors;
        sdCardFlag = false;   // stop hammering a card that has gone away
        debugPrintf("Log write failed; logging stopped\n");
      }
      xSemaphoreGive(g_spiMutex);
    }

    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    g_snapshot.log_rows = rows;
    g_snapshot.log_errors = errors;
    g_snapshot.log_stack =
        uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
    xSemaphoreGive(g_stateMutex);

    vTaskDelayUntil(&lastWake, period);
  }
}

// Bring-up telemetry. This is the only way to verify the 100 Hz claim and the
// IMU axis mapping without a screen, so it stays in the build behind
// DEBUG_SERIAL.
static void printTelemetry(const AppSnapshot &s) {
  if (!DEBUG_SERIAL) return;
  const ins::Output &o = s.ins;
  debugPrintf(
      "TEL imu=%uHz ekf=%uHz gnss=%uHz rows=%u err=%u sat=%u bad=%u | "
      "stk imu=%u ins=%u gnss=%u ui=%u log=%u | "
      "mode=%u zupt=%u mag=%u dt=%.1fms rej=%u | "
      "lat=%.6f lon=%.6f h=%.1f vN=%.2f vE=%.2f vD=%.2f "
      "rpy=%.1f/%.1f/%.1f\n",
      (unsigned)s.imu_hz, (unsigned)s.ins_hz, (unsigned)s.gnss_hz,
      (unsigned)s.log_rows, (unsigned)s.log_errors,
      (unsigned)s.gnss_sentences, (unsigned)s.gnss_bad_checksum,
      (unsigned)s.imu_stack, (unsigned)s.ins_stack, (unsigned)s.gnss_stack,
      (unsigned)s.ui_stack, (unsigned)s.log_stack,
      (unsigned)o.mode, o.zupt_active ? 1u : 0u, o.mag_active ? 1u : 0u,
      o.dt_ms, (unsigned)o.gnss_rejected, o.lat_deg, o.lon_deg, o.h_m, o.vn,
      o.ve, o.vd, o.roll_deg, o.pitch_deg, o.yaw_deg);
}

static void uiTask(void *arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastTelemetryMs = 0;
  ins::Mode lastMode = ins::Mode::WaitGnss;

  for (;;) {
    M5.update();

    if (M5.BtnPWR.wasClicked()) {
      ++currentPage;
      if (currentPage > MAX_PAGE) currentPage = PAGE_SD_CARD;
      debugPrintf("Page -> %u\n", (unsigned int)currentPage);
      xSemaphoreTake(g_spiMutex, portMAX_DELAY);
      clearScreen();
      drawHeader();
      xSemaphoreGive(g_spiMutex);
    }

    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    g_uiView = g_snapshot;
    g_uiView.ui_stack =
        uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
    xSemaphoreGive(g_stateMutex);

    xSemaphoreTake(g_spiMutex, portMAX_DELAY);
    drawCurrentPage();
    xSemaphoreGive(g_spiMutex);

    // Alignment completed: report the measured means so the IMU axis mapping
    // (IMU_TO_BODY in ins_ekf.h) can be checked against reality. A level
    // device must report accel z negative, i.e. specific force pointing up.
    if (g_uiView.ins.mode != lastMode) {
      lastMode = g_uiView.ins.mode;
      if (lastMode == ins::Mode::Run) {
        const ins::Output &o = g_uiView.ins;
        debugPrintf("ALIGN acc=(%.3f %.3f %.3f) gyro=(%.4f %.4f %.4f) "
                    "rpy=%.1f/%.1f/%.1f\n",
                    o.align_acc_mean[0], o.align_acc_mean[1],
                    o.align_acc_mean[2], o.align_gyro_mean[0],
                    o.align_gyro_mean[1], o.align_gyro_mean[2], o.roll_deg,
                    o.pitch_deg, o.yaw_deg);
      }
    }

    const uint32_t now = millis();
    if (now - lastTelemetryMs >= 5000) {
      lastTelemetryMs = now;
      printTelemetry(g_uiView);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(UI_REFRESH_MS));
  }
}

// ---------------------------------------------------------------------------
// NMEA parser self-test
// ---------------------------------------------------------------------------

static void selftestCheck(bool ok, const char *name, int &passed, int &failed) {
  if (ok) {
    ++passed;
  } else {
    ++failed;
    debugPrintf("  FAIL: %s\n", name);
  }
}

static void selftestFeed(const char *sentence) {
  for (const char *p = sentence; *p != '\0'; ++p) nmeaFeed(*p, millis());
  nmeaFeed('\n', millis());
}

static void runNmeaSelftest() {
  int passed = 0;
  int failed = 0;
  debugPrintf("NMEA self-test:\n");

  const uint32_t badBefore = nmeaBadChecksum;

  selftestFeed(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
  selftestCheck(g_gnss.position_valid, "GGA sets position_valid", passed, failed);
  selftestCheck(fabs(g_gnss.lat_deg - 48.1173) < 1e-4, "GGA latitude", passed, failed);
  selftestCheck(fabs(g_gnss.lon_deg - 11.516667) < 1e-4, "GGA longitude", passed, failed);
  selftestCheck(fabs(g_gnss.hgt_msl_m - 545.4f) < 0.01f, "GGA MSL height", passed, failed);
  selftestCheck(fabs(g_gnss.geoid_separation_m - 46.9f) < 0.01f,
                "GGA geoid separation", passed, failed);
  selftestCheck(fabs(g_gnss.hgt_wgs84_m - 592.3f) < 0.01f,
                "GGA WGS84 height = MSL + geoid", passed, failed);
  selftestCheck(g_gnss.numsat == 8, "GGA satellites used", passed, failed);
  selftestCheck(g_gnss.utc_time == 123519UL, "GGA UTC time", passed, failed);

  selftestFeed(
      "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230326,003.1,W*7D");
  selftestCheck(g_gnss.date_valid, "RMC sets date_valid", passed, failed);
  selftestCheck(g_gnss.utc.year == 2026 && g_gnss.utc.month == 3 &&
                    g_gnss.utc.day == 23,
                "RMC date", passed, failed);
  selftestCheck(fabs(g_gnss.speed_ms - 22.4 * KNOTS_TO_MPS) < 0.01f,
                "RMC speed converted to m/s", passed, failed);
  selftestCheck(fabs(g_gnss.heading_deg - 84.4f) < 0.01f, "RMC course",
                passed, failed);
  selftestCheck(g_gnss.local.hour == 20 && g_gnss.local.minute == 35 &&
                    g_gnss.local.second == 19,
                "local time = UTC + offset", passed, failed);
  selftestCheck(g_gnss.local.day == 23 && g_gnss.local.month == 3 &&
                    g_gnss.local.year == 2026,
                "local date unchanged by a +8h shift", passed, failed);

  selftestFeed("$GNRMC,200000,A,4807.038,N,01131.000,E,000.0,000.0,310326,,*06");
  selftestCheck(g_gnss.local.year == 2026 && g_gnss.local.month == 4 &&
                    g_gnss.local.day == 1 && g_gnss.local.hour == 4,
                "local date rolls into the next month", passed, failed);
  selftestFeed("$GNRMC,200000,A,4807.038,N,01131.000,E,000.0,000.0,311226,,*06");
  selftestCheck(g_gnss.local.year == 2027 && g_gnss.local.month == 1 &&
                    g_gnss.local.day == 1 && g_gnss.local.hour == 4,
                "local date rolls into the next year", passed, failed);

  selftestFeed(
      "$GPGSV,1,1,04,03,03,111,15,04,15,270,22,12,45,090,30,13,06,292,31*7A");
  selftestCheck(g_gnss.in_view[CONST_GPS] == 4, "GPGSV in-view count", passed, failed);
  selftestCheck(findSatSlot(CONST_GPS, 13) >= 0, "GPGSV satellite stored",
                passed, failed);
  const int gps12 = findSatSlot(CONST_GPS, 12);
  selftestCheck(gps12 >= 0 && g_sats[gps12].snr_dB == 30, "GPGSV PRN 12 SNR",
                passed, failed);

  selftestFeed(
      "$GAGSV,1,1,04,11,45,180,42,12,30,090,45,19,60,270,35,36,20,315,31*65");
  selftestCheck(g_gnss.in_view[CONST_GALILEO] == 4, "GAGSV in-view count",
                passed, failed);
  selftestCheck(g_gnss.in_view[CONST_GPS] == 4,
                "GAGSV leaves the GPS count alone", passed, failed);
  const int gal12 = findSatSlot(CONST_GALILEO, 12);
  selftestCheck(gal12 >= 0 && g_sats[gal12].snr_dB == 45, "GAGSV PRN 12 SNR",
                passed, failed);
  selftestCheck(gal12 >= 0 && gps12 >= 0 && gal12 != gps12,
                "GPS PRN 12 and Galileo PRN 12 are distinct slots", passed,
                failed);

  selftestFeed("$GBGSV,1,1,03,07,55,120,44,08,40,200,39,13,25,300,33*5E");
  selftestCheck(g_gnss.in_view[CONST_BEIDOU] == 3, "GBGSV in-view count",
                passed, failed);

  selftestFeed("$GLGSV,1,1,02,12,50,100,41,13,35,250,29*6D");
  selftestCheck(g_gnss.in_view[CONST_GLONASS] == 2, "GLGSV in-view count",
                passed, failed);

  selftestFeed("$GPGSV,1,1,01,137,40,200,38*70");
  selftestCheck(findSatSlot(CONST_SBAS, 137) >= 0,
                "GPGSV PRN 137 classified as SBAS", passed, failed);

  selftestFeed("$GNGSV,1,1,01,12,50,100,40*55");
  selftestCheck(findSatSlot(CONST_UNKNOWN, 12) >= 0,
                "GNGSV tracked as unknown constellation", passed, failed);

  selftestFeed(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00");
  selftestCheck(nmeaBadChecksum == badBefore + 1, "bad checksum rejected",
                passed, failed);

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

  g_stateMutex = xSemaphoreCreateMutex();
  g_spiMutex = xSemaphoreCreateMutex();
  g_trajMutex = xSemaphoreCreateMutex();
  g_imuQueue = xQueueCreate(16, sizeof(ImuQueueItem));
  g_gnssQueue = xQueueCreate(4, sizeof(ins::GnssFix));

  ins::reset();

  if (NMEA_SELFTEST) {
    runNmeaSelftest();
    g_gnss = GnssData();
    memset(g_sats, 0, sizeof(g_sats));
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
    delay(UBX_CONFIG_DELAY_MS);
    ubxConfigureGnss();
    debugPrintf("UBX-CFG-GNSS sent (GPS/SBAS/Galileo/BeiDou/QZSS/GLONASS)\n");
    while (Serial2.available()) Serial2.read();
    nmeaReset();
  }

  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);
  sdCardFlag =
      SD.begin(SD_SPI_CS_PIN, SPI, 25000000) && SD.cardType() != CARD_NONE;
  if (sdCardFlag) sdCardFlag = openLogFile();
  if (!sdCardFlag) debugPrintf("No SD card / log file unavailable\n");

  clearScreen();
  drawHeader();
  drawCurrentPage();

  // Start the tasks last, once M5, the UART, the receiver config and the log
  // file are all ready.
  xTaskCreatePinnedToCore(gnssTask, "gnss", STACK_GNSS, nullptr, TASK_PRIO_GNSS,
                          nullptr, CORE_IO);
  xTaskCreatePinnedToCore(imuTask, "imu", STACK_IMU, nullptr, TASK_PRIO_IMU,
                          nullptr, CORE_FILTER);
  xTaskCreatePinnedToCore(insTask, "ins", STACK_INS, nullptr, TASK_PRIO_INS,
                          nullptr, CORE_FILTER);
  xTaskCreatePinnedToCore(logTask, "log", STACK_LOG, nullptr, TASK_PRIO_LOG,
                          nullptr, CORE_IO);
  xTaskCreatePinnedToCore(uiTask, "ui", STACK_UI, nullptr, TASK_PRIO_UI,
                          nullptr, CORE_IO);

  debugPrintf("Tasks started\n");
}

void loop() {
  // Every job lives in its own task; the Arduino loop only needs to exist.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
