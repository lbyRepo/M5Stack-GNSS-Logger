#include <TinyGPSPlus.h>
#include <M5CoreS3.h>
#include <String.h>
#include <SPI.h>
#include <SD.h>

// Display flags
#define DISPLAY_IMU_DATA (0)
#define DISPLAY_UBX_DATA (1)
// Byte Conversion
#define KILO_BYTE (1024)
#define MEGA_BYTE (1024 * 1024)
#define GIGA_BYTE (1024 * 1024 * 1024)

#define CONST_G (9.80665)  // gravity constant
#define PIXEL_HEIGHT (20)
#define PROCESS_IMU_FLAG (0)
#define TIMEZONE_OFFSET_HRS (8)

// M5Stack Core S3 Pin Declaration
#define SD_SPI_SCK_PIN (36)
#define SD_SPI_MISO_PIN (35)
#define SD_SPI_MOSI_PIN (37)
#define SD_SPI_CS_PIN (4)
#define GNSS_MODULE_RX_PIN (18)
#define GNSS_MODULE_TX_PIN (17)

// FIle name
char outputFileName[255];
unsigned int fileCounter = 0;

//SD Card Functions
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void printf_log(const char *format, ...);
void println_log(const char *str);
M5Canvas canvas(&CoreS3.Display);

TinyGPSPlus gps;
TinyGPSCustom GPGSV_msgnum(gps, "GPGSV", 2);
TinyGPSCustom SNR_SAT1(gps, "GPGSV", 7);
TinyGPSCustom SNR_SAT2(gps, "GPGSV", 7 + 1 * 4);
TinyGPSCustom SNR_SAT3(gps, "GPGSV", 7 + 2 * 4);
TinyGPSCustom SNR_SAT4(gps, "GPGSV", 7 + 3 * 4);

typedef struct T_IMU_DATA_STRUCT {
  float time_s;
  float acc_ms2[3];   //xyz
  float gyr_rads[3];  //xyz
  float mag_uT[3];    //xyz
  float imu_rpy_rad[3];
  float temperature_degC;
} T_IMU_DATA_STRUCT;

typedef struct T_STRUCT_DATE_TIME {
  unsigned short YEAR;
  unsigned char MONTH;
  unsigned char DAY;

  unsigned char HOUR;
  unsigned char MINUTE;
  unsigned char SECOND;

} T_STRUCT_DATE_TIME;

typedef struct T_STRUCT_GPGSV_MSG {
  unsigned char numMessages;
  unsigned char messageNum;
  unsigned char satelliteInView;
  unsigned char satID[4];  // in NMEA there are 4 channels per message for GPGSV
  char elevation_deg[4];
  int azimuth_deg[4];
  unsigned char SNR_dBHz[4];
} T_STRUCT_GPGSV_MSG;

typedef struct T_UBX_DATA_STRUCT {
  double lat_rad;
  double lon_rad;
  float hgt_m;
  bool valid;
  float gnssSpeed_ms;
  float gnssHeading_rad;
  unsigned char numsat;
  char ss;
  char snr;


  T_STRUCT_DATE_TIME gnssDateTime;
  T_STRUCT_GPGSV_MSG svData;

} T_UBX_DATA_STRUCT;

typedef struct T_NAV_SENSOR_STRUCT {
  float navTime_s;
  T_IMU_DATA_STRUCT imuData;
  T_UBX_DATA_STRUCT gnssData;
} T_NAV_SENSOR_STRUCT;
T_NAV_SENSOR_STRUCT nav_data_struct;

// GNSS Module Serial Functions
static void smartDelay(unsigned long ms);
static void printFloat(float val, bool valid, int len, int prec);
static void printInt(unsigned long val, bool valid, int len);
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
static void printStr(const char *str, int len);
void printGnssModuleSerialHeader(void);
void printGnssModuleSerialData(void);
void printData2Screen(T_NAV_SENSOR_STRUCT *dataIn, float sdCardCap, unsigned char rowStart, unsigned char colStart);
void logSdCardGnssData(T_NAV_SENSOR_STRUCT *dataIn, char *fileNameInput);
void assignGnssDataStruct(T_NAV_SENSOR_STRUCT *dataOut, TinyGPSPlus *dataIn);

unsigned char RowStore = 0;
unsigned char ColStore = 0;
unsigned char sdCardFlag = 0;
#if PROCESS_IMU_FLAG
void assignImuDataStruct(T_NAV_SENSOR_STRUCT *dataOut);
#endif

void setup() {
  // Init data structure of nav sensors
  memset(&nav_data_struct, 0, sizeof(nav_data_struct));
  unsigned char row = 0;
  unsigned char col = 0;
  unsigned char headerPrintFlag = 0;

  // Initialise M5 Stack
  auto cfg = M5.config();
  CoreS3.begin(cfg);
  CoreS3.Display.setTextSize(2);  // Set text size.
  CoreS3.Display.setCursor(col, row);
  row += PIXEL_HEIGHT;  // Set the cursor.
  CoreS3.Display.printf("=== Ublox Data Readout ===");
  CoreS3.Display.setCursor(col, row);
  row += PIXEL_HEIGHT;  // Set the cursor.
  CoreS3.Display.printf("Log File  : ");
  RowStore = row;
  ColStore = col;

  delay(200);          // Delay 200ms.
  CoreS3.Imu.begin();  // Init IMU.

  // Initialise Serial Monitor
  Serial.begin(115200);
  // Initialise GNSS Module Serial Connection
  Serial2.begin(38400, SERIAL_8N1, GNSS_MODULE_RX_PIN, GNSS_MODULE_TX_PIN);  //begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms, uint8_t rxfifo_full_thrhd)
  // SD Card Initialization
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);

  if (!SD.begin(SD_SPI_CS_PIN, SPI, 25000000)) {
    println_log("Card failed, or not present\n");
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    println_log("No SD card attached");
    sdCardFlag = 0;
    CoreS3.Display.printf("No SD");
    return;
  } else {
    sdCardFlag = 1;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    println_log("MMC");
  } else if (cardType == CARD_SD) {
    println_log("SDSC");
  } else if (cardType == CARD_SDHC) {
    println_log("SDHC");
  } else {
    println_log("UNKNOWN");
  }


  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  printf_log("SD Card Size: %llu MB\n", cardSize);

  // List every file is in this current directory
  listDir(SD, "/", 0);

  sprintf(outputFileName, "/log.csv");

  // Only print csv header once
  if (headerPrintFlag == 0) {
    headerPrintFlag = 1;
    while (SD.exists(outputFileName)) {  // LBY: if a file already exist, then create a new file
      fileCounter++;
      sprintf(outputFileName, "/log_%u.csv", fileCounter);
    }
    writeFile(SD, outputFileName, "pc_time,date,time,numsat,lat_deg,lon_deg,hgt_m,speed_ms,heading_deg\n");
  }
  CoreS3.Display.printf(outputFileName);

  /*
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    */
  listDir(SD, "/", 0);
  printf_log("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  printf_log("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

  //printGnssModuleSerialHeader();
}

void loop() {
  float sdCardCapicity = 0.0;
  if (sdCardFlag) {
    sdCardCapicity = 1.0 * SD.usedBytes() / SD.totalBytes();
  } else {
    sdCardCapicity = 0.0;
  }

  /*Data Assignment*/
  assignGnssDataStruct(&nav_data_struct, &gps);

  /*Print to M5Stack LCD Screen GNSS Data*/
  printData2Screen(&nav_data_struct, sdCardCapicity, RowStore, ColStore);

  /*Print to serial monitor GNSS data*/
  //printGnssModuleSerialData();

  /*Print GNSS Data to SD card*/
  logSdCardGnssData(&nav_data_struct, outputFileName);

/*
  printf_log("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  printf_log("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  */
}

void assignImuDataStruct(T_NAV_SENSOR_STRUCT *dataOut) {
  auto imu_update = M5.Imu.update();
  if (imu_update) {
    auto data = M5.Imu.getImuData();

    // The data obtained by getImuData can be used as follows.
    dataOut->imuData.acc_ms2[0] = data.accel.x*CONST_G;      // accel x-axis value.
    dataOut->imuData.acc_ms2[1] = data.accel.y*CONST_G;      // accel y-axis value.
    dataOut->imuData.acc_ms2[2] = data.accel.z*CONST_G;      // accel z-axis value.

    dataOut->imuData.gyr_rads[0] = data.gyro.x*DEG_TO_RAD;      // gyro x-axis value.
    dataOut->imuData.gyr_rads[1] = data.gyro.y*DEG_TO_RAD;      // gyro y-axis value.
    dataOut->imuData.gyr_rads[2] = data.gyro.z*DEG_TO_RAD;      // gyro z-axis value.

    dataOut->imuData.mag_uT[0] = data.mag.x;      // mag x-axis value.
    dataOut->imuData.mag_uT[1] = data.mag.y;      // mag y-axis value.
    dataOut->imuData.mag_uT[2] = data.mag.z;      // mag z-axis value.

  }
}

void printGnssModuleSerialHeader(void) {
  Serial.println();
  Serial.println(F(
    "Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt   "
    " Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(
    F("           (deg)      (deg)       Age                      Age  (m) "
      "   --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F(
    "----------------------------------------------------------------------"
    "------------------------------------------------------------------"));
}

void printGnssModuleSerialData(void) {
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(
    gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ",
    6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), LONDON_LAT, LONDON_LON)
    / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon = TinyGPSPlus::courseTo(
    gps.location.lat(), gps.location.lng(), LONDON_LAT, LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

void printData2Screen(T_NAV_SENSOR_STRUCT *dataIn, float sdCardCap, unsigned char rowStart, unsigned char colStart) {
  unsigned char jj = colStart;
  unsigned char ii = rowStart;
#if DISPLAY_IMU_DATA
  // Acc
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Ax [m/s2]: %.3f", dataIn->imuData.acc_ms2[0]);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Ay [m/s2]: %.3f", dataIn->imuData.acc_ms2[1]);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Az [m/s2]: %.3f", dataIn->imuData.acc_ms2[2]);
  //Gyr
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Gx [deg/s]: %.3f", dataIn->imuData.gyr_rads[0] * RAD_TO_DEG);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Gy [deg/s]: %.3f", dataIn->imuData.gyr_rads[1] * RAD_TO_DEG);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Gz [deg/s]: %.3f", dataIn->imuData.gyr_rads[2] * RAD_TO_DEG);
  //Mag
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Mx [uT]: %.3f", dataIn->imuData.mag_uT[0]);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("My [uT]: %.3f", dataIn->imuData.mag_uT[1]);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Mz [uT]: %.3f", dataIn->imuData.mag_uT[2]);
#endif
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  if (SD.usedBytes() >= GIGA_BYTE) {
    CoreS3.Display.printf("SD used    : %llu GB", SD.usedBytes() / GIGA_BYTE);
  } else if (SD.usedBytes() >= MEGA_BYTE) {
    CoreS3.Display.printf("SD used    : %llu MB", SD.usedBytes() / MEGA_BYTE);
  } else if (SD.usedBytes() >= KILO_BYTE) {
    CoreS3.Display.printf("SD used    : %llu kB", SD.usedBytes() / KILO_BYTE);
  } else {
    CoreS3.Display.printf("SD used    : %llu Bytes", SD.usedBytes());
  }
  // SD Card Capacity
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("SDcard[%%]  : %.3f", sdCardCap);

#if DISPLAY_UBX_DATA
  // GNSS Valid
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Numsat     : %u", dataIn->gnssData.numsat);
  // Date Time Data
  /*
  CoreS3.Display.setCursor(jj, ii);ii+=PIXEL_HEIGHT;
  CoreS3.Display.printf("Date       : %2d/%2d/%4d", dataIn->gnssData.gnssDateTime.DAY, dataIn->gnssData.gnssDateTime.MONTH, dataIn->gnssData.gnssDateTime.YEAR);
  CoreS3.Display.setCursor(jj, ii);ii+=PIXEL_HEIGHT;
  CoreS3.Display.printf("GMT+%u Time : %2d:%2d:%2d", (unsigned char)TIMEZONE_OFFSET_HRS, dataIn->gnssData.gnssDateTime.HOUR, dataIn->gnssData.gnssDateTime.MINUTE, dataIn->gnssData.gnssDateTime.SECOND);
  */
  //Gnss Pos
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Lat [deg]  : %lf", dataIn->gnssData.lat_rad * RAD_TO_DEG);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Lon [deg]  : %lf", dataIn->gnssData.lon_rad * RAD_TO_DEG);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Hgt [m]    : %.5f", dataIn->gnssData.hgt_m);
  //Gnss Vel
  /*
  CoreS3.Display.setCursor(jj, ii);ii+=PIXEL_HEIGHT;
  CoreS3.Display.printf("Spd [m/s]  : %lf", dataIn->gnssData.gnssSpeed_ms);
  CoreS3.Display.setCursor(jj, ii);ii+=PIXEL_HEIGHT;
  CoreS3.Display.printf("Head [deg] : %lf", dataIn->gnssData.gnssHeading_rad * RAD_TO_DEG);
*/
  if ((SNR_SAT1.isUpdated()) && (dataIn->gnssData.numsat > 3)) {
    CoreS3.Display.setCursor(jj, ii);
    ii += PIXEL_HEIGHT;
    CoreS3.Display.printf("SNR 1[dBHz]: ");
    CoreS3.Display.printf(SNR_SAT1.value());
    CoreS3.Display.setCursor(jj, ii);
    ii += PIXEL_HEIGHT;
    CoreS3.Display.printf("SNR 2[dBHz]: ");
    CoreS3.Display.printf(SNR_SAT2.value());
    CoreS3.Display.setCursor(jj, ii);
    ii += PIXEL_HEIGHT;
    CoreS3.Display.printf("SNR 3[dBHz]: ");
    CoreS3.Display.printf(SNR_SAT3.value());
    CoreS3.Display.setCursor(jj, ii);
    ii += PIXEL_HEIGHT;
    CoreS3.Display.printf("SNR 4[dBHz]: ");
    CoreS3.Display.printf(SNR_SAT4.value());
  }
#endif
}

void logSdCardGnssData(T_NAV_SENSOR_STRUCT *dataIn, char *fileNameInput) {
  char text[255] = { 0 };

  sprintf(text,
          "%f,%2u/%2u/%4u,%2u:%2u:%2u,%u,%lf,%lf,%f,%f,%f,\n",
          millis() / 1000.0,
          dataIn->gnssData.gnssDateTime.DAY, dataIn->gnssData.gnssDateTime.MONTH, dataIn->gnssData.gnssDateTime.YEAR,
          dataIn->gnssData.gnssDateTime.HOUR, dataIn->gnssData.gnssDateTime.MINUTE, dataIn->gnssData.gnssDateTime.SECOND,
          dataIn->gnssData.numsat,
          dataIn->gnssData.lat_rad * RAD_TO_DEG, dataIn->gnssData.lon_rad * RAD_TO_DEG, dataIn->gnssData.hgt_m,
          dataIn->gnssData.gnssSpeed_ms, dataIn->gnssData.gnssHeading_rad * RAD_TO_DEG);

  //Serial.println(text);
  appendFile(SD, fileNameInput, text);
}

void assignGnssDataStruct(T_NAV_SENSOR_STRUCT *dataOut, TinyGPSPlus *dataIn) {
  dataOut->gnssData.valid = (bool)dataIn->satellites.isValid();
  if (dataOut->gnssData.valid == 1) {
    dataOut->gnssData.numsat = (unsigned char)dataIn->satellites.value();
    dataOut->gnssData.gnssSpeed_ms = (float)dataIn->speed.mps();
    dataOut->gnssData.gnssHeading_rad = (float)dataIn->course.deg() * DEG_TO_RAD;

    dataOut->gnssData.lat_rad = (double)dataIn->location.lat() * DEG_TO_RAD;
    dataOut->gnssData.lon_rad = (double)dataIn->location.lng() * DEG_TO_RAD;
    dataOut->gnssData.hgt_m = (float)dataIn->altitude.meters();

    // Timezone compensation
    if ((dataIn->time.hour() + TIMEZONE_OFFSET_HRS) >= 24) {
      dataOut->gnssData.gnssDateTime.HOUR = (unsigned short)(dataIn->time.hour() + TIMEZONE_OFFSET_HRS - 24);
      dataOut->gnssData.gnssDateTime.DAY = (unsigned short)(dataIn->date.day() + 1);
    } else {
      dataOut->gnssData.gnssDateTime.HOUR = (unsigned short)(dataIn->time.hour() + TIMEZONE_OFFSET_HRS);
      dataOut->gnssData.gnssDateTime.DAY = (unsigned short)(dataIn->date.day());
    }

    dataOut->gnssData.gnssDateTime.MINUTE = (unsigned short)dataIn->time.minute();
    dataOut->gnssData.gnssDateTime.SECOND = (unsigned short)dataIn->time.second();

    dataOut->gnssData.gnssDateTime.YEAR = (unsigned short)dataIn->date.year();
    dataOut->gnssData.gnssDateTime.MONTH = (unsigned short)dataIn->date.month();

  } else {
    memset(dataOut, 0, sizeof(dataOut));
  }
  smartDelay(1000);
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial2.available()) gps.encode(Serial2.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1) Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i) Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid) sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i) sz[i] = ' ';
  if (len > 0) sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i) Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  printf_log("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    println_log("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    println_log("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      println_log(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      println_log(String(file.size()).c_str());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  printf_log("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    println_log("Dir created");
  } else {
    println_log("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  printf_log("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    println_log("Dir removed");
  } else {
    println_log("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  printf_log("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    println_log("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  printf_log("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    println_log("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    println_log("File written");
  } else {
    println_log("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  printf_log("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    println_log("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    println_log("Message appended");
  } else {
    println_log("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  printf_log("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    println_log("File renamed");
  } else {
    println_log("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  printf_log("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    println_log("File deleted");
  } else {
    println_log("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    printf_log("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    println_log("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    println_log("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  printf_log("%u bytes written for %lu ms\n", 2048 * 512, end);
  file.close();
}

void printf_log(const char *format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, 256, format, args);
  va_end(args);
  Serial.print(buf);
  canvas.printf(buf);
  canvas.pushSprite(0, 0);
}

void println_log(const char *str) {
  Serial.println(str);
  canvas.println(str);
  canvas.pushSprite(0, 0);
}
