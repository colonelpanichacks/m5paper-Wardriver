#include <M5EPD.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

const String BUILD = "1.0.1";
const String VERSION = "1.0";

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

int mNumWifi = 0;
int mNumBLE = 0;

File logFile;
String logFileName = "/WiFiScanLog.csv";

struct Device {
  char type[10];
  char ssid[35];
  char mac[18];
  int rssi;
  char info[100];
};

Device deviceList[150];
int deviceIndex = 0;

// Filesystem
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

// Display
void displayDevices() {
  for (int i = 0; i < deviceIndex - 1; i++) {
    for (int j = i + 1; j < deviceIndex; j++) {
      if (deviceList[i].rssi < deviceList[j].rssi) {
        Device temp = deviceList[i];
        deviceList[i] = deviceList[j];
        deviceList[j] = temp;
      }
    }
  }

  String gpsValid = gps.location.isValid() ? "Valid" : "Invalid";  // gps status for top text

  canvas.createCanvas(540, 960);
  canvas.fillCanvas(0);
  canvas.setTextSize(2);
  canvas.drawString("GPS: " + gpsValid + " | HDOP: " + String(gps.hdop.value()) + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);
  canvas.drawLine(10, 30, 540, 30, 15);

  int y = 40;
  for (int i = 0; i < deviceIndex; i++) {
    canvas.drawString(String(i + 1) + ": " + deviceList[i].info, 10, y);
    y += 30;
    if (y > canvas.height() - 20) {
      canvas.pushCanvas(0, 0, UPDATE_MODE_DU4);
      delay(2000);
      canvas.fillCanvas(0);
      y = 40;
    }
  }

  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
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

void replaceLowestRSSIDevice(const char* type, const char* ssid, const char* mac, int rssi, int channel = 0, const char* encryption = "") {
  if (deviceIndex == 0) return;  // Handle empty list case

  int minIndex = 0;
  for (int i = 1; i < deviceIndex; ++i) {
    if (deviceList[i].rssi < deviceList[minIndex].rssi) {
      minIndex = i;
    }
  }

  auto safeStrCopy = [](char* dest, const char* src, size_t destSize) {  // safer string helper
    strncpy(dest, src, destSize - 1);
    dest[destSize - 1] = '\0';
  };

  safeStrCopy(deviceList[minIndex].type, type, sizeof(deviceList[minIndex].type));
  safeStrCopy(deviceList[minIndex].ssid, ssid, sizeof(deviceList[minIndex].ssid));
  safeStrCopy(deviceList[minIndex].mac, mac, sizeof(deviceList[minIndex].mac));
  deviceList[minIndex].rssi = rssi;

  // Format for device type
  snprintf(deviceList[minIndex].info, sizeof(deviceList[minIndex].info),
           strcmp(type, "WiFi") == 0 ? "WiFi: %s [%d] (%s) %d dBm" : "BLE: %s (%s) %d dBm",
           ssid, channel, strcmp(type, "WiFi") == 0 ? encryption : mac, rssi);
}

// WiFi and BT scanning
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String macStr = advertisedDevice.getAddress().toString().c_str();
    String ssidStr = advertisedDevice.getName().c_str();
    int rssi = advertisedDevice.getRSSI();
    const char* mac = macStr.c_str();
    const char* ssid = ssidStr.c_str();

    if (isDuplicate(mac)) {
      return;
    }

    GPSData gpsData = getGPSData();
    logToCSV(mac, ssid, "", gpsData.time.c_str(), 0, rssi, gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.accuracy, "BLE");

    if (deviceIndex < 150) {
      strncpy(deviceList[deviceIndex].type, "BLE", sizeof(deviceList[deviceIndex].type));
      strncpy(deviceList[deviceIndex].ssid, ssid, sizeof(deviceList[deviceIndex].ssid) - 1);
      strncpy(deviceList[deviceIndex].mac, mac, sizeof(deviceList[deviceIndex].mac) - 1);
      deviceList[deviceIndex].rssi = rssi;
      snprintf(deviceList[deviceIndex].info, sizeof(deviceList[deviceIndex].info),
               "BLE: %s (%s) %d dBm", ssid, mac, rssi);
      deviceIndex++;
    } else {
      replaceLowestRSSIDevice("BLE", ssid, mac, rssi);
    }
    mNumBLE++;
    displayDevices();
  }
};

void initializeScanning() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
}

void setup() {
  Serial.begin(115200);
  M5.begin();
  M5.EPD.SetRotation(1);  // Correct rotation for full vertical (portrait) display
  M5.EPD.Clear(true);
  canvas.createCanvas(540, 960);  // Correct full vertical canvas size
  canvas.setTextSize(2);

  if (!initSDCard()) {
    Serial.println("Failed to initialize SD card. Halting...");
    while (true) delay(1000);
  }

  GPS_Serial.begin(9600, SERIAL_8N1, 19, 18);
  canvas.drawString("GPS initialized.", 10, 40);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GC16);
  delay(2000);

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

  int n = WiFi.scanNetworks(false, true, false, 200);
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

      if (ssidStr == "") {
        ssidStr = "HIDDEN";
      }
      
      if (isDuplicate(bssid)) {
        continue;
      }

      logToCSV(bssid, ssid, encryption, gpsData.time.c_str(), channel, rssi, gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.accuracy, "WiFi");

      if (deviceIndex < 150) {
        // Add WiFi device if there is space
        strncpy(deviceList[deviceIndex].type, "WiFi", sizeof(deviceList[deviceIndex].type));
        strncpy(deviceList[deviceIndex].ssid, ssid, sizeof(deviceList[deviceIndex].ssid) - 1);
        strncpy(deviceList[deviceIndex].mac, bssid, sizeof(deviceList[deviceIndex].mac) - 1);
        deviceList[deviceIndex].rssi = rssi;
        snprintf(deviceList[deviceIndex].info, sizeof(deviceList[deviceIndex].info),
                 "WiFi: %s [%d] (%s) %d dBm", ssid, channel, encryption, rssi);

        deviceIndex++;
      } else {
        replaceLowestRSSIDevice("WiFi", ssid, bssid, rssi, channel, encryption);
      }
      mNumWifi++;
      displayDevices();
    }
  }

  WiFi.scanDelete();  // flush scan

  pBLEScan->start(scanTime, false);
  pBLEScan->clearResults();

  delay(2000);  // scan delay
}

bool isDuplicate(const char* mac) {
  for (int i = 0; i < deviceIndex; i++) {
    if (strcmp(deviceList[i].mac, mac) == 0) {
      return true;
    }
  }
  return false;
}

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
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "[WPA2_ENTERPRISE]";
    case WIFI_AUTH_WPA3_PSK:
      return "[WPA3_PSK]";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "[WPA2_WPA3_PSK]";
    case WIFI_AUTH_WAPI_PSK:
      return "[WAPI_PSK]";
    default:
      return "[UNKNOWN]";
  }
}
