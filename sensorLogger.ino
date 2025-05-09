#include <TinyGPSPlus.h>
#include <M5CoreS3.h>
#include <String.h>
#include <SPI.h>
#include <SD.h>

// Display flags
#define DISPLAY_IMU_DATA (0)
#define DISPLAY_UBX_DATA (1)
#define DISPLAY_SNR_DATA (1)
#define DISPLAY_INTVL_MS (1000)

// Byte Conversion
#define KILO_BYTE (1024)
#define MEGA_BYTE (1024 * 1024)
#define GIGA_BYTE (1024 * 1024 * 1024)

// CONSTANT VALUES
#define CONST_G (9.80665)  // gravity constant
#define PIXEL_HEIGHT (20)
#define PROCESS_IMU_FLAG (0)
#define TIMEZONE_OFFSET_HRS (8)
static const int MAX_SATELLITES = 40;

// M5Stack Core S3 Pin Declaration
#define SD_SPI_SCK_PIN (36)
#define SD_SPI_MISO_PIN (35)
#define SD_SPI_MOSI_PIN (37)
#define SD_SPI_CS_PIN (4)
#define GNSS_MODULE_RX_PIN (18)
#define GNSS_MODULE_TX_PIN (17)

// File name
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

// M5 Stack Display
M5Canvas canvas(&CoreS3.Display);
static unsigned char m5StackPageNum = 0x00;

TinyGPSPlus gps;
// LBY: Addition of GPGSV
TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
TinyGPSCustom satNumber[4]; // to be initialized later
TinyGPSCustom elevation_deg[4];
TinyGPSCustom azimuth_deg[4];
TinyGPSCustom SNR_dB[4];
// LBY: Addition of GPGSA for GNSS Fix Value
//TinyGPSCustom gpsFix(gps, "GPGGA", 6); //GPGSA for GPS only constellation, GNGSA for multiconstellation, which is the default for ublox m9n


typedef struct T_STRUCT_SATS
{
  bool active;
  int elevation_deg;
  int azimuth_deg;
  int SNR_dB;
} T_STRUCT_SATS;


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

typedef struct T_UBX_DATA_STRUCT {
  //unsigned char gnssFix; //1 = not avail, 2 = 2D , 3 = 3D
  double lat_rad;
  double lon_rad;
  float hgt_m;
  bool valid;
  float gnssSpeed_ms;
  float gnssHeading_rad;
  unsigned char numsat;
  unsigned char satsInView;

  T_STRUCT_DATE_TIME gnssDateTime;
  T_STRUCT_SATS sat[MAX_SATELLITES];

} T_UBX_DATA_STRUCT;

typedef struct T_NAV_SENSOR_STRUCT {
  float navTime_s;
  T_IMU_DATA_STRUCT imuData;
  T_UBX_DATA_STRUCT gnssData;
} T_NAV_SENSOR_STRUCT;
T_NAV_SENSOR_STRUCT nav_data_struct;

// GNSS Module Serial Functions
static void smartDelay(unsigned long ms, TinyGPSPlus* inputGps);
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
  CoreS3.Display.printf("Log File    : ");
  RowStore = row;
  ColStore = col;

  delay(200);          // Delay 200ms.
  CoreS3.Imu.begin();  // Init IMU.

  // Init TinyGPSCustom
  // Initialize all the uninitialized TinyGPSCustom objects
  for (int i=0; i<4; ++i)
  {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation_deg[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth_deg[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    SNR_dB[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }

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
  //printf_log("SD Card Size: %llu MB\n", cardSize);

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

  listDir(SD, "/", 0);
  //printf_log("Total space: %lluMB\n", SD.totalBytes() / MEGA_BYTE);
  //printf_log("Used space: %lluMB\n", SD.usedBytes() / MEGA_BYTE);

}

void loop() {
  /*SD card capacity calculation*/
  float sdCardCapicity = 0.0;
  if (sdCardFlag) {
    sdCardCapicity = 1.0 * SD.usedBytes() / SD.totalBytes();
  } else {
    sdCardCapicity = 0.0;
  }

  /*Touch Screen Detection*/
  /*
  if (CoreS3.Display.pushState()){
    m5StackPageNum^=0x01;
    printf_log("Page Num: %u \n", m5StackPageNum);

  }
  */

  /*Data Assignment*/
  assignGnssDataStruct(&nav_data_struct, &gps);

  /*Print to M5Stack LCD Screen GNSS Data*/
  printData2Screen(&nav_data_struct, sdCardCapicity, RowStore, ColStore);

  /*Print GNSS Data to SD card*/
  logSdCardGnssData(&nav_data_struct, outputFileName);

}

/*LBY: takes in M5 Stack Core S3 IMU Data, converts to SI Unit and assign to data struct*/
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

  // SD Card Used
  /*
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  if (SD.usedBytes() >= GIGA_BYTE) {
    CoreS3.Display.printf("SD used     : %llu GB", SD.usedBytes() / GIGA_BYTE);
  } else if (SD.usedBytes() >= MEGA_BYTE) {
    CoreS3.Display.printf("SD used     : %llu MB", SD.usedBytes() / MEGA_BYTE);
  } else if (SD.usedBytes() >= KILO_BYTE) {
    CoreS3.Display.printf("SD used     : %llu kB", SD.usedBytes() / KILO_BYTE);
  } else {
    CoreS3.Display.printf("SD used     : %llu Bytes", SD.usedBytes());
  }
  */
  // SD Card Capacity
  /*
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("SDcard[%%]  : %.3f", sdCardCap);
  */
#if DISPLAY_UBX_DATA
  // GNSS Position
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Lat [deg]   : %lf", dataIn->gnssData.lat_rad * RAD_TO_DEG);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Lon [deg]   : %lf", dataIn->gnssData.lon_rad * RAD_TO_DEG);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Hgt [m]     : %.5f", dataIn->gnssData.hgt_m);


  // GNSS Numsat
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("Numsat      : %u", dataIn->gnssData.numsat);
  
  // GNSS Valid
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("GNSS Valid  : %u", dataIn->gnssData.valid);
  
  



                                                
#endif

/*LBY: Rewriting SNR_dB Display, impossible to display all SNR on small screen
  hence, I will display top 4 SNR values*/
  #if DISPLAY_SNR_DATA
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  CoreS3.Display.printf("==== GPS Sat SNR [dB] ====");
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  int kk = 0;
  // Satellite 1 to 9
  CoreS3.Display.printf("%2d %2d %2d %2d %2d %2d %2d %2d %2d", 
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  // Satellite 10 to 18
  CoreS3.Display.printf("%2d %2d %2d %2d %2d %2d %2d %2d %2d", 
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  // Satellite 19 to 27
  CoreS3.Display.printf("%2d %2d %2d %2d %2d %2d %2d %2d %2d", 
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB);
  CoreS3.Display.setCursor(jj, ii);
  ii += PIXEL_HEIGHT;
  // Satellite 28 to 36
  CoreS3.Display.printf("%2d %2d %2d %2d %2d %2d %2d %2d %2d", 
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB,
                        dataIn->gnssData.sat[kk++].SNR_dB);

  #endif

  //refresh display at 1000ms interval
  smartDelay(DISPLAY_INTVL_MS, &gps);
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

/*LBY: takes in M5 Stack GNSS Module Data and assign to NAV data struct, UBLOX default config sends out NMEA*/
void assignGnssDataStruct(T_NAV_SENSOR_STRUCT *dataOut, TinyGPSPlus *dataIn) {
  // Check NMEA GPGGA validity
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

  }
  // Check NMEA GPGSV validity
  if(totalGPGSVMessages.isUpdated()){
    for (int i=0; i<4; ++i)
      {
        int no = atoi(satNumber[i].value());
        if (no >= 1 && no <= MAX_SATELLITES)
        {
          dataOut->gnssData.sat[no-1].elevation_deg = atoi(elevation_deg[i].value());
          dataOut->gnssData.sat[no-1].azimuth_deg = atoi(azimuth_deg[i].value());
          dataOut->gnssData.sat[no-1].SNR_dB = atoi(SNR_dB[i].value());
          dataOut->gnssData.sat[no-1].active = true;
        }
/*
        int totalMessages = atoi(totalGPGSVMessages.value());
        int currentMessage = atoi(messageNumber.value());
        if (totalMessages == currentMessage)
        {
          // Insert things to do here
        }
*/
      }
  }
  if (satsInView.isUpdated()){
    dataOut->gnssData.satsInView = atoi(satsInView.value());
    //printf("sats in view: %u\n",dataOut->gnssData.satsInView);
  }

}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms, TinyGPSPlus* inputGps) {
  unsigned long start = millis();
  do {
    while (Serial2.available()) inputGps->encode(Serial2.read());
  } while (millis() - start < ms);
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  //printf_log("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    //println_log("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    //println_log("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      //Serial.print("  DIR : ");
      //println_log(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      //Serial.print("  FILE: ");
      //Serial.print(file.name());
      //Serial.print("  SIZE: ");
      //println_log(String(file.size()).c_str());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  //printf_log("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    //println_log("Dir created");
  } else {
    //println_log("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  //printf_log("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    //println_log("Dir removed");
  } else {
    //println_log("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  //printf_log("Reading file: %s\n", path);

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
  //printf_log("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    //println_log("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    //println_log("File written");
  } else {
    //println_log("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  //printf_log("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //println_log("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //println_log("Message appended");
  } else {
    //println_log("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  //printf_log("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    //println_log("File renamed");
  } else {
    //println_log("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  //printf_log("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    //println_log("File deleted");
  } else {
    //println_log("Delete failed");
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
    //printf_log("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    //println_log("Failed to open file for reading");
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
  //printf_log("%u bytes written for %lu ms\n", 2048 * 512, end);
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
