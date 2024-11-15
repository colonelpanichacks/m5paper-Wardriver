#include <M5EPD.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include <mutex>

const String BUILD = "1.0.4";
const String VERSION = "1.1";

/* Slow debugging serial prints
 *  Set 0 to disable and 1 to enable
 */
#define MUTEXDEBUG 0
#define BATTDEBUG 0
#define GPSDEBUG 0
#define RTCDEBUG 0
#define GPSMUTEXDEBUG 0

// Set timeouts in seconds for users
const int display_timeout_s = 120;
const int clean_every_s = 60;

// Convert S to uS because ts is in uS
const int S_TO_uS = 1000000;
const int display_timeout = display_timeout_s * S_TO_uS;

// Battery constants
const int BATT_MAX = 4200; // it says 4350 on the back but I haven't seen it that high while unplugged
const int BATT_MIN = 3200; // mine died at 3010 and we want a buffer
const int BATT_DIFF = BATT_MAX - BATT_MIN;

int xmax = 540;
int ymax = 960;

M5EPD_Canvas canvas(&M5.EPD);
#define SD_CS_PIN 4
int written_lines = 0;
char last_flush[128] = "never";

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;
std::mutex gpsMutex;

int64_t last_set_rtc = 0;
const int64_t set_rtc_every_s = 60 * 60; // Set RTC from GPS once per hour
const int64_t set_rtc_every = set_rtc_every_s * S_TO_uS;

struct GPSData {
  double latitude;
  double longitude;
  double altitude;
  double accuracy;
};

BLEScan* pBLEScan;
int scanTime = 5;

File logFile;

struct Device {
  String type;
  String ssid;
  String mac;
  int rssi;
  int64_t ts = esp_timer_get_time() - display_timeout - 1; //timestamp since boot in microseconds, initialized to be outside the display timeout
};

const int max_devices = 1025;
int64_t init_timeout = esp_timer_get_time() - display_timeout;
Device deviceList[max_devices];
std::mutex deviceListMutex;

void mutexDebug(String msg="") {
  #if MUTEXDEBUG == 1
  Serial.println(msg);
  #endif
}

void rtcDebug(String msg="") {
  #if RTCDEBUG == 1
  Serial.println(msg);
  #endif
}

void gpsMutexDebug(String msg="") {
  #if GPSMUTEXDEBUG == 1
  Serial.println(msg);
  #endif
}

void updateGPS() {
  gpsMutexDebug("Attempting to lock GPS Mutex");
  gpsMutex.lock();
  gpsMutexDebug("GPS mutex locked");
  while (GPS_Serial.available() > 0) {
    char c = GPS_Serial.read();
    gps.encode(c);
  }
  gpsMutex.unlock();
  gpsMutexDebug("GPS mutex unlocked");
  if (last_set_rtc == 0) {
    // RTC hasn't been set yet
    rtcDebug("RTC never been set, attempting to set...");
    setRTC();
  } else if ( (last_set_rtc + set_rtc_every) < esp_timer_get_time() ) {
    // RTC hasn't been set for a while
    rtcDebug("RTC has not been set in a while, attempting to set...");
    rtcDebug(String(last_set_rtc));
    rtcDebug(String(set_rtc_every));
    rtcDebug(String(esp_timer_get_time()));
    setRTC();
  }
}

GPSData getGPSData() {
  GPSData gpsData;
  char utc[21];
  updateGPS();
  gpsData.latitude = gps.location.lat();
  gpsData.longitude = gps.location.lng();
  gpsData.altitude = gps.altitude.meters();
  gpsData.accuracy = gps.hdop.hdop();
  return gpsData;
}

void setRTC() {
  #if GPSDEBUG == 1
  rtcDebug("Checking for valid GPS signal...");
  #endif
  if (gps.date.isValid() && gps.time.isValid()) {
    rtcDebug("GPS Valid, setting time and date");
    rtc_time_t RTCtime;
    RTCtime.hour = gps.time.hour();
    RTCtime.min  = gps.time.minute();
    RTCtime.sec  = gps.time.second();
    M5.RTC.setTime(&RTCtime);

    rtc_date_t RTCDate;
    RTCDate.year = gps.date.year();
    RTCDate.mon  = gps.date.month();
    RTCDate.day  = gps.date.day();
    M5.RTC.setDate(&RTCDate);

    last_set_rtc = esp_timer_get_time();
  } else {
    rtcDebug("Unable to set RTC without valid GPS signal");
  }
}

void setRotation(bool rotate = false) {
  if (rotate) {
    int temp = xmax;
    xmax = ymax;
    ymax = temp;
    canvas.deleteCanvas();
  }
  if (xmax == 540) {
    M5.EPD.SetRotation(90);  // Correct rotation for full vertical (portrait) display
    M5.TP.SetRotation(90);
  }
  if (xmax == 960) {
    M5.EPD.SetRotation(0);  // Correct rotation for full horizontal (landscape) display
    M5.TP.SetRotation(0);
  }
  M5.EPD.Clear(true);
  canvas.createCanvas(xmax, ymax);
  canvas.setTextSize(2);
}

void checkButtons() {
  if (M5.BtnL.wasPressed()) {
    Serial.println("Btn L Pressed");
    setRotation(true);
  }
  if (M5.BtnP.wasPressed()) {
    Serial.println("Btn P Pressed");
    setRotation(true);
  }
  if (M5.BtnR.wasPressed()) {
    Serial.println("Btn R Pressed");
    setRotation(true);
  }
  M5.BtnL.lastChange();
  M5.update();
}

void idelay(int sleepytyme = 0) {
  //interruptible delay will delay for the requested time while periodically checking for button input
  for (int i = 0; i < sleepytyme; i+=50) {
    delay(50);
    checkButtons();
  }
}

void battDebug(String msg="", float var=0) {
  #if BATTDEBUG == 1
  // Serial.printf("msg%d",var);
  Serial.print(msg);
  Serial.println(var);
  #endif
}

void writeCSVHeader() {
  if (logFile) {
    // https://api.wigle.net/csvFormat.html
    // [Format version],appRelease=[version],model=[device model],release=[device release],device=[device name],display=[device display characteristics],board=[device board descriptor],brand=[device brand],star=[system star],body=[orbiting body index],subBody=[orbit index of sub-body]
    logFile.println("WigleWifi-1.6,appRelease=" + BUILD + ",model=M5Paper,release=" + VERSION + ",device=M5Paper,display=EPD_ED047TC1,board=ESP32-D0WDQ6-V3,brand=M5Stack,Sol,3,0");
    logFile.println("MAC,SSID,AuthMode,FirstSeen,Channel,Frequency,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,RCOIs,MfgrId,Type");
    logFile.flush();
  }
}

bool initSDCard() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    return false;
  }

  String base_name = "/m5paperwardriver-wigle";
  String logFileName = base_name + "0.csv";
  for (int i = 0; i < 999999; i++) {
    logFileName = base_name + i + ".csv";
    if (!SD.exists(logFileName))
      break;
  }
  Serial.println("Using logFileName");
  Serial.println(logFileName);
  logFile = SD.open(logFileName, FILE_WRITE);
  if (!logFile) {
    Serial.println("Error opening log file!");
    return false;
  }
  writeCSVHeader();
  return true;
}

// band should be an enum but I don't know how
int channel_to_freq(int chan, int band=2) {
  // shamelessly adapted from https://github.com/torvalds/linux/blob/master/net/wireless/util.c
  switch (band) {
    case 2: // 2.4GHz
      if (chan == 14)
        return(2484);
      else if (chan < 14)
        return(2407 + (chan * 5));
      break;
    case 5: // 5GHz
      if (chan >= 182 && chan <= 196)
        return(4000 + (chan * 5));
      else
        return(5000 + (chan * 5));
      break;
    case 6: // 6GHz
      if (chan == 2)
        return(5935);
      if (chan <= 233)
        return(5950  + (chan * 5));
      break;
    case 60: // 60GHz
      if (chan < 7)
        return(56160 + (chan * 2160));
      break;
    //case 1: // S1G
    //  return(902 + (chan * 0.5);
    default:
      ;
  }
  return 0;
}

void logToCSV(const char* netid, const char* ssid, const char* authType, int channel, int signal, const char* type, const char* rcoi="", const char* mfgrid="") {
  // Don't try to write if we don't even know what time it is
  if (last_set_rtc == 0)
    return;

  if (!logFile)
    return;

  // 1.4
  //logFile.printf("%s,\"%s\",%s,%s,%d,%d,%.6f,%.6f,%.2f,%.2f,%s\n",
  //              netid, ssid, authType, time, channel, signal, lat, lon, altitude, accuracy, type);
  // 1.6 added frequency, rcoi, mfgrid
  int frequency = 0;

  if (type == "BLE")
    frequency = 7936;

  if (type == "WiFi")
    frequency = channel_to_freq(channel);

  GPSData gpsData = getGPSData(); // this automatically gets the most update info from the gps and even sets time in RTC if needed
  rtc_time_t RTCtime;
  M5.RTC.getTime(&RTCtime);
  rtc_date_t RTCDate;
  M5.RTC.getDate(&RTCDate);
  char time[128];
  sprintf(time, "%04d-%02d-%02d %02d:%02d:%02d", RTCDate.year, RTCDate.mon, RTCDate.day, RTCtime.hour, RTCtime.min, RTCtime.sec);
  logFile.printf("%s,\"%s\",%s,%s,%d,%d,%d,%.6f,%.6f,%.2f,%.2f,rcoi,mfgid,%s\n",
                netid, ssid, authType, time, channel, channel_to_freq(channel), gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.accuracy, rcoi, mfgrid, type);
                // WiFi [BSSID],  [SSID],       [Capabilities],[First timestamp seen],[Channel],[Frequency],[RSSI],[Latitude],[Longitude],[Altitude],[Accuracy],[RCOIs],[MfgrId],[Type]
                // BLE  [BD_ADDR],[Device Name],[Capabilities],[First timestamp seen],[Channel],[Frequency],[RSSI],[Latitude],[Longitude],[Altitude],[Accuracy],[RCOIs],[MfgrId],[Type]
  written_lines ++;

  // Slow down the flushing to a max of once per second.
  if (time != last_flush) {
    logFile.flush();
    sprintf(last_flush, "%04d-%02d-%02d %02d:%02d:%02d", RTCDate.year, RTCDate.mon, RTCDate.day, RTCtime.hour, RTCtime.min, RTCtime.sec);
  }
}

float getBatteryPercent() {
  uint32_t battVolt = M5.getBatteryVoltage();
  battDebug("Raw battvolt: ", battVolt);
  // Normalize battery voltage
  if (battVolt < BATT_MIN) {
    battVolt = BATT_MIN;
  } else if (battVolt > BATT_MAX) {
    battVolt = BATT_MAX;
  }
  battDebug("Normalized battVolt: ", battVolt);
  // Calculate percentage
  float batteryPercent = ((float)(battVolt - BATT_MIN) / (float)BATT_DIFF );
  battDebug("battery percent: ", batteryPercent);
  return batteryPercent;
}

void drawHeader(int mNumWifi, int mNumBLE) {
  canvas.fillCanvas(0);
  canvas.setTextSize(2);

  // Normal Info header
  updateGPS();
  String gpsValid = gps.location.isValid() ? "Valid" : "Invalid";  // gps status for top text
  // Over 20 hdop is considered "poor" so there really isn't a need to show the default over 2000 that the gps reports
  // Max it at 99 and add a + for when it's higher.  This prevents The UI from looking bad when location is invalid and HDOP is high.
  String hdop = "99+";
  if (gps.hdop.value() < 100)
    hdop = String(gps.hdop.value());
  canvas.drawString("GPS:" + gpsValid + " | HDOP:" + hdop + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Memory debugging Info header
  // canvas.drawString("Free " + String(esp_get_minimum_free_heap_size()) + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Runtime Duration Info header
  // canvas.drawString("Time: " + String(esp_timer_get_time() / S_TO_uS / 60 ) + "m | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Battery debugging Info header
  // canvas.drawString("BATT " + String(M5.getBatteryVoltage()) + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Normal line
  // canvas.drawLine(0, 40, xmax, 40, 20);

  // Battery level indicator line
  float batteryPercent = getBatteryPercent();
  // slowing pulling line in from both sides toward the middle as it drains
  int halfLineSize = (float)xmax/2 * batteryPercent;
  // Left half of line
  canvas.drawLine(xmax/2 - halfLineSize, 40, xmax/2, 40, 15);
  // Right half of line
  canvas.drawLine(xmax/2, 40, xmax/2 + halfLineSize, 40, 15);
  canvas.setTextSize(2);
}

void displayDevices() {
  int mNumWifi = 0;
  int mNumBLE = 0;
  int64_t now = esp_timer_get_time();

  // todo: don't bubble sort
  mutexDebug("Mutex held by dD sort");
  for (int i = 0; i < max_devices; i++) {
    std::lock_guard<std::mutex> lck(deviceListMutex);
    if ((now - deviceList[i].ts) < display_timeout) {
      for (int j = i + 1; j < max_devices - 1; j++) {
        if ((now - deviceList[j].ts) < display_timeout) {
          if (deviceList[i].rssi < deviceList[j].rssi) {
            Device temp = deviceList[i];
            deviceList[i] = deviceList[j];
            deviceList[j] = temp;
          }
        }
      }
    }
    if (deviceList[i].type == "WiFi") {
      mNumWifi ++;
    }
    if (deviceList[i].type == "BLE") {
      mNumBLE ++;
    }
  }
  mutexDebug("Mutex released by dD sort");

  checkButtons();
  canvas.createCanvas(xmax, ymax);
  canvas.setTextSize(2);
  drawHeader(mNumWifi, mNumBLE);

  /* Unlocking the mutex during the sleep can cause devices
   *  to get updated and end up out of order.  It's better
   *  than before the mutexs though...
   */

  int y = 25;
  int dCount = 0;
  deviceListMutex.lock();
  mutexDebug("Mutex held by main device display loop");
  for (int i = 0; i < max_devices; i++) {
    if ((now - deviceList[i].ts) < display_timeout) {
      y += 30;
      dCount ++;
      if (y > canvas.height() - 20) {
        canvas.pushCanvas(0, 0, UPDATE_MODE_DU);
        deviceListMutex.unlock();
        mutexDebug("Mutex released before sleep inside device display loop");
        idelay(3000);
        deviceListMutex.lock();
        mutexDebug("Mutex reheld after sleep inside device displayloop");
        drawHeader(mNumWifi, mNumBLE);
        y = 55;
      }

      canvas.drawString(String(dCount) + ": " +
        deviceList[i].type + ": " +
        deviceList[i].ssid +
        " (" + deviceList[i].mac + ") " +
        deviceList[i].rssi
        , 10, y);
    }
  }
  deviceListMutex.unlock();
  mutexDebug("Mutex released by main device display loop");
  canvas.pushCanvas(0, 0, UPDATE_MODE_DU);
}

int magicIndex(const char* mac) {
  mutexDebug("Mutex held by mI mac match");
  for (int i = 0; i < max_devices; i++) {
    std::lock_guard<std::mutex> lck(deviceListMutex);
    if (deviceList[i].mac == String(mac)) {
      mutexDebug("Mutex released by mI mac match success");
      return i;
    }
  }
  mutexDebug("Mutex released by mI mac match failure");
  int64_t ts_to_beat = esp_timer_get_time();
  int oldest_index = 0;
  mutexDebug("Mutex held by mI oldest match");
  for (int i = max_devices; i >= 0; i--) {
    std::lock_guard<std::mutex> lck(deviceListMutex);
    if (deviceList[i].ts < ts_to_beat) {
      ts_to_beat = deviceList[i].ts;
      oldest_index = i;
    }
  }
  mutexDebug("Mutex released by mI oldest match");
  return oldest_index;
}

/* Where did all this come from?
 *
 * ESP32 type codes
 * apparently only these ones are supported
 * https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/STA.cpp#L88
 * despite having a bigger list here
 * https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/STA.cpp#L703
 * and this is a different esp lib entirely which we aren't using?  Can we?
 * https://github.com/espressif/esp-idf/blob/master/components/esp_wifi/include/esp_wifi_types_generic.h#L66
 *
 * Android Type codes, what wigle csv wants
 * https://developer.android.com/reference/android/net/wifi/WifiConfiguration.KeyMgmt ?
 * https://android.googlesource.com/platform/prebuilts/fullsdk/sources/android-30/+/refs/heads/main/com/android/server/wifi/util/InformationElementUtil.java#1016 ??
 */
const char* getAuthType(uint8_t wifiAuth) {
  switch (wifiAuth) {
    case WIFI_AUTH_OPEN:
      return "[OPEN]";
    case WIFI_AUTH_WEP:
      return "[WEP]";
    case WIFI_AUTH_WPA_PSK:
      return "[WPA_PSK]";
    case WIFI_AUTH_WPA2_PSK:
      return "[WPA2_PSK]";
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "[WPA_WPA2_PSK]";
    // case WIFI_AUTH_ENTERPRISE:
    //  return "[WPA_ENTERPRISE]";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "[WPA2_ENTERPRISE]";
    case WIFI_AUTH_WPA3_PSK:
      return "[WPA3_PSK]";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "[WPA2_WPA3_PSK]";
    case WIFI_AUTH_WAPI_PSK:
      return "[WAPI_PSK]";
    // case WIFI_AUTH_OWE:
    //  return "[OWE]";
    // case WIFI_AUTH_WPA3_ENT_192:
    //  return "[RSN-EAP_SUITE_B_192-GCMP-256]";
    // case WIFI_AUTH_WPA3_EXT_PSK:
    //  return "[WPA3_PSK_DEPRECATED_EXT]";
    // case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
    //  return "[WPA3_PSK_DEPRECATED_EXT]";
    // case WIFI_AUTH_DPP:
      // DPP is an AUTH mechanism that works with both WPA2 and WPA3
      // esp32 gives no indication of which, so I'm just calling it WPA2 since that's the lowest option
    //  return "[WPA2_DPP]";
    default:
      return "[UNKNOWN]";
  }
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String macStr = advertisedDevice.getAddress().toString().c_str();
    String ssidStr = advertisedDevice.getName().c_str();
    int rssi = advertisedDevice.getRSSI();
    const char* mac = macStr.c_str();
    const char* ssid = ssidStr.c_str();

    logToCSV(mac, ssid, "[BLE]", 0, rssi, "BLE");

    int deviceIndex = magicIndex(mac);
    mutexDebug("Mutex held by BT Callback");
    {
      std::lock_guard<std::mutex> lck(deviceListMutex);
      deviceList[deviceIndex].type = "BLE";
      deviceList[deviceIndex].ssid = ssid;
      deviceList[deviceIndex].mac = mac;
      deviceList[deviceIndex].rssi = rssi;
      deviceList[deviceIndex].ts = esp_timer_get_time();
    }
    mutexDebug("Mutex released by BT Callback");
  }
};


void startWiFiScan() {
  // Arduino-esp 3.x adds minimum time per channel for active scanning
  // https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiScanTime/WiFiScanTime.ino#L22
  // But m5paper/m5epd only supports arduino-esp 2.x
  // WiFi.setScanActiveMinTime(103); // default minimum is 100ms

  // Scan + hidden networks, at default because interval instead of 2x since we scan async
  // async true, show_hidden true, passive false, max_ms_per_chan
  WiFi.scanNetworks(true,true,false,103);
}

void initializeScanning() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  startWiFiScan();
  Serial.println("WiFi scanning Initialized. OK");
  canvas.drawString("WiFi scanning Initialized. OK", 10, 190);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);

  // BT Classic Scanning
  // Docs indicate support for Bluetooth v4.2 BR/EDR

  // BLE 4.x scanning (5.x not supported by ESP32-D0WDQ6-V3 in m5paper)
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  Serial.println("BLE Scanning Initialized. OK");
  canvas.drawString("BLE Scanning Initialized. OK", 10, 220);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);

  Serial.println("Waiting for devices...");
  canvas.drawString("Waiting for devices...", 10, 280);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
}

void setup() {
  //Serial.begin(115200); // part of M5.begin
  // Defaults begin(bool touchEnable = true, bool SDEnable = true, bool SerialEnable = true, bool BatteryADCEnable = true, bool I2CEnable = false)
  // Disabling unused touch to save battery
  M5.begin(false, true, true, true, false);
  M5.RTC.begin();
  Serial.print("Current battery voltage: ");
  Serial.println(String(M5.getBatteryVoltage() / (float)1000));
  Serial.println("M5paper initialized.");
  setRotation();
  Serial.println("M5paper screen initialized.");

  canvas.drawString("Current battery " + String(int(getBatteryPercent() * 100)) + "% (" + String(M5.getBatteryVoltage() / (float)1000) + "v)", 10, 10);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);

  if (!initSDCard()) {
    Serial.println("Failed to initialize SD card. Halted");
    canvas.drawString("Failed to initialize SD card. Halted.", 10, 40);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
    while (true) delay(1000);
  }
  Serial.println("SD card initialized. OK");
  canvas.drawString("SD card initialized. OK", 10, 40);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);

  GPS_Serial.begin(9600, SERIAL_8N1, 19, 18);
  Serial.println("GPS initialized. OK");
  canvas.drawString("GPS initialized. OK", 10, 70);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);

  canvas.drawString("Maximum tracked devices: " + String(max_devices), 10, 100);
  canvas.drawString("Display timeout        : " + String(display_timeout_s), 10, 130);

  Serial.println("Initializing Scanning...");
  canvas.drawString("Initializing Scanning...", 10, 160);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
  initializeScanning();
}

void parseWiFiScan(uint16_t networksFound) {
  if (networksFound > 0) {
    GPSData gpsData = getGPSData();
    for (int i = 0; i < networksFound; ++i) {
      String ssidStr = WiFi.SSID(i);
      String bssidStr = WiFi.BSSIDstr(i);
      int rssi = WiFi.RSSI(i);
      String encryptionStr = getAuthType(WiFi.encryptionType(i));
      int channel = WiFi.channel(i);

      const char* ssid = ssidStr.c_str();
      const char* bssid = bssidStr.c_str();
      const char* encryption = encryptionStr.c_str();

      logToCSV(bssid, ssid, encryption, channel, rssi, "WiFi");

      int deviceIndex = magicIndex(bssid);
      mutexDebug("Mutex held by wifi loop");
      {
        std::lock_guard<std::mutex> lck(deviceListMutex);
        deviceList[deviceIndex].type = "WiFi";
        deviceList[deviceIndex].ssid = ssid;
        deviceList[deviceIndex].mac = bssid;
        deviceList[deviceIndex].rssi = rssi;
        deviceList[deviceIndex].ts = esp_timer_get_time();
      }
      mutexDebug("Mutex released by wifi loop");
    }
  }
  WiFi.scanDelete();
}

void loop() {
  checkButtons();

  updateGPS();

  #if GPSDEBUG == 1
  if (!gps.location.isValid()) {
    Serial.println("Waiting for valid GPS data...");
  }
  #endif

  int16_t WiFiScanStatus = WiFi.scanComplete();
  if (WiFiScanStatus < 0) {  // it is busy scanning or got an error
    if (WiFiScanStatus == WIFI_SCAN_FAILED) {
      Serial.println("WiFi Scan has failed. Starting again.");
      startWiFiScan();
    }
    // other option is status WIFI_SCAN_RUNNING - just wait.
  } else {  // Found Zero or more Wireless Networks
    parseWiFiScan(WiFiScanStatus);
    startWiFiScan();  // start over...
  }

  pBLEScan->start(scanTime, false);
  pBLEScan->clearResults();

  displayDevices();
  // displayDevices has the only delays in the main loop
}
