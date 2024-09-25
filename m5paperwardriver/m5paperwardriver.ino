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

const String BUILD = "1.0.2";
const String VERSION = "1.0";

/* Slow debugging serial prints
 *  Set 0 to disable and 1 to enable
 */

#define MUTEXDEBUG 0
#define BATTDEBUG 0

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

M5EPD_Canvas canvas(&M5.EPD);
#define SD_CS_PIN 4

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

struct GPSData {
  String time;
  double latitude;
  double longitude;
  double altitude;
  double accuracy;
};

BLEScan* pBLEScan;
int scanTime = 5;

File logFile;
String logFileName = "/WiFiScanLog.csv";

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

void battDebug(String msg="", float var=0) {
  #if BATTDEBUG == 1
  Serial.print(msg);
  Serial.println(var);
  #endif
}

void writeCSVHeader() {
  if (logFile) {
    logFile.println("WigleWifi-1.4,appRelease=" + BUILD + ",model=M5Paper,release=" + VERSION + ",device=M5Paper,display=ePaper,board=ESP32,brand=M5");
    logFile.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
    logFile.flush();
  }
}

bool initSDCard() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    return false;
  }
  logFile = SD.open(logFileName, FILE_WRITE);
  if (!logFile) {
    Serial.println("Error opening log file!");
    return false;
  }
  writeCSVHeader();
  return true;
}

void logToCSV(const char* netid, const char* ssid, const char* authType, const char* time, int channel, int signal, double lat, double lon, double altitude, double accuracy, const char* type) {
  if (logFile) {
    logFile.printf("%s,\"%s\",%s,%s,%d,%d,%.6f,%.6f,%.2f,%.2f,%s\n",
                   netid, ssid, authType, time, channel, signal, lat, lon, altitude, accuracy, type);
    logFile.flush();
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
  // canvas.setTextSize(3);

  // Normal Info header
  String gpsValid = gps.location.isValid() ? "Valid" : "Invalid";  // gps status for top text
  canvas.drawString("GPS: " + gpsValid + " | HDOP: " + String(gps.hdop.value()) + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Memory debugging Info header
  // canvas.drawString("Free " + String(esp_get_minimum_free_heap_size()) + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Runtime Duration Info header
  // canvas.drawString("Runtime: " + String(esp_timer_get_time() / S_TO_uS / 60 ) + "m | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Battery debugging Info header
  // canvas.drawString("BATT " + String(M5.getBatteryVoltage()) + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);

  // Normal line
  // canvas.drawLine(10, 30, 540, 30, 15);

  // Battery level indicator line
  float batteryPercent = getBatteryPercent();
  // slowing pulling line in from both sides toward the middle as it drains
  int halfLineSize = (float)270 * batteryPercent;
  // Left half of line
  canvas.drawLine(270 - halfLineSize, 30, 270, 30, 15);
  // Right half of line
  canvas.drawLine(270, 30, 270 + halfLineSize, 30, 15);
  // canvas.setTextSize(2);
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

  canvas.createCanvas(540, 960);
  canvas.setTextSize(2);
  drawHeader(mNumWifi, mNumBLE);

  /* Unlocking the mutex during the sleep can cause devices
   *  to get updated and end up out of order.  It's better
   *  than before the mutexs though...
   */

  int y = 15;
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
        delay(3000);
        deviceListMutex.lock();
        mutexDebug("Mutex reheld after sleep inside device displayloop");
        drawHeader(mNumWifi, mNumBLE);
        y = 45;
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

GPSData getGPSData() {
  GPSData gpsData;
  char utc[21];
  sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
  gpsData.time = String(utc);
  gpsData.latitude = gps.location.lat();
  gpsData.longitude = gps.location.lng();
  gpsData.altitude = gps.altitude.meters();
  gpsData.accuracy = gps.hdop.hdop();
  return gpsData;
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
 * https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/STA.cpp#L703
 * https://github.com/espressif/esp-idf/blob/master/components/esp_wifi/include/esp_wifi_types_generic.h#L66
 *
 * Android Type codes
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
    case WIFI_AUTH_ENTERPRISE:
      return "[WPA_ENTERPRISE]";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "[WPA2_ENTERPRISE]";
    case WIFI_AUTH_WPA3_PSK:
      return "[WPA3_PSK]";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "[WPA2_WPA3_PSK]";
    case WIFI_AUTH_WAPI_PSK:
      return "[WAPI_PSK]";
    case WIFI_AUTH_OWE:
      return "[OWE]";
    case WIFI_AUTH_WPA3_ENT_192:
      return "[RSN-EAP_SUITE_B_192-GCMP-256]";
    case WIFI_AUTH_WPA3_EXT_PSK:
      return "[WPA3_PSK_DEPRECATED_EXT]";
    case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
      return "[WPA3_PSK_DEPRECATED_EXT]";
    case WIFI_AUTH_DPP:
      // DPP is an AUTH mechanism that works with both WPA2 and WPA3
      // esp32 gives no indication of which, so I'm just calling it WPA2 since that's the lowest option
      return "[WPA2_DPP]";
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

    GPSData gpsData = getGPSData();
    logToCSV(mac, ssid, "", gpsData.time.c_str(), 0, rssi, gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.accuracy, "BLE");

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

void initializeScanning() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // TODO BLE5
  // https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/BLE5_extended_scan/BLE5_extended_scan.ino
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  Serial.println("Scanning Initialized. OK");
  canvas.drawString("Scanning Initialized. OK", 10, 190);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
}

void setup() {
  Serial.begin(115200);
  M5.begin();
  Serial.print("Current battery voltage: ");
  Serial.println(String(M5.getBatteryVoltage() / (float)1000));
  Serial.println("M5paper initialized.");
  M5.EPD.SetRotation(1);  // Correct rotation for full vertical (portrait) display
  M5.EPD.Clear(true);
  canvas.createCanvas(540, 960);  // Correct full vertical canvas size
  canvas.setTextSize(2);
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

void loop() {
  while (GPS_Serial.available() > 0) {
    char c = GPS_Serial.read();
    gps.encode(c);
  }

  if (!gps.location.isValid()) {
    Serial.println("Waiting for valid GPS data...");
  }
  // TODO Convert to async
  // https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiScanAsync/WiFiScanAsync.ino
  // Scan + hidden networks, at Nyquist sampling rate of 200ms per channel
  int n = WiFi.scanNetworks(false,true,false,200);
  if (n > 0) {
    GPSData gpsData = getGPSData();
    for (int i = 0; i < n; ++i) {
      String ssidStr = WiFi.SSID(i);
      String bssidStr = WiFi.BSSIDstr(i);
      int rssi = WiFi.RSSI(i);
      String encryptionStr = getAuthType(WiFi.encryptionType(i));
      int channel = WiFi.channel(i);

      const char* ssid = ssidStr.c_str();
      const char* bssid = bssidStr.c_str();
      const char* encryption = encryptionStr.c_str();

      logToCSV(bssid, ssid, encryption, gpsData.time.c_str(), channel, rssi, gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.accuracy, "WiFi");

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

  pBLEScan->start(scanTime, false);
  pBLEScan->clearResults();

  displayDevices(); // the wifi scan takes time so no need to delay here
}
