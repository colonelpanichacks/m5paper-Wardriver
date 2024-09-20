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
  String type;
  String ssid;
  String mac;
  int rssi;
  String info;
};

Device deviceList[150];
int deviceIndex = 0;

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

  int y = 15;
  for (int i = 0; i < deviceIndex; i++) {
    y += 30;
    if (y > canvas.height() - 20) {
      canvas.pushCanvas(0, 0, UPDATE_MODE_DU4);
      delay(2000);
      canvas.fillCanvas(0);
      canvas.drawString("GPS: " + gpsValid + " | HDOP: " + String(gps.hdop.value()) + " | WiFi:" + String(mNumWifi) + " | BLE:" + String(mNumBLE), 10, 10);
      canvas.drawLine(10, 30, 540, 30, 15);
      y = 45;
    }
    canvas.drawString(String(i + 1) + ": " + deviceList[i].info, 10, y);
  }

  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
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

    deviceList[deviceIndex].type = "BLE";
    deviceList[deviceIndex].ssid = ssid;
    deviceList[deviceIndex].mac = mac;
    deviceList[deviceIndex].rssi = rssi;
    deviceList[deviceIndex].info = "BLE: " + String(ssid) + " (" + String(mac) + ") RSSI: " + String(rssi) + " dBm";
    deviceIndex++;
    mNumBLE ++;
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

  delay(2000);
  if (!initSDCard()) {
    Serial.println("Failed to initialize SD card. Halting...");
    canvas.drawString("Failed to initialize SD card. Halting...", 10, 40);
    canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
    while (true) delay(1000);
  }
  canvas.drawString("SD card initialized...", 10, 40);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
  delay(1000);

  GPS_Serial.begin(9600, SERIAL_8N1, 19, 18);
  canvas.drawString("GPS initialized...", 10, 70);
  canvas.pushCanvas(0, 0, UPDATE_MODE_GLR16);
  delay(1000);

  canvas.drawString("Scanning initialized...", 10, 100);
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

      if (ssidStr == "") {
        ssidStr = "HIDDEN";
      }

      if (isDuplicate(bssid)) { // skip to the next
        continue;
      }

      logToCSV(bssid, ssid, encryption, gpsData.time.c_str(), channel, rssi, gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.accuracy, "WiFi");

      deviceList[deviceIndex].type = "WiFi";
      deviceList[deviceIndex].ssid = ssid;
      deviceList[deviceIndex].mac = bssid;
      deviceList[deviceIndex].rssi = rssi;
      deviceList[deviceIndex].info = "WiFi: " + ssidStr + " (" + bssidStr + ") RSSI: " + String(rssi) + " dBm";
      deviceIndex++;
      mNumWifi ++;
    }
  }
  WiFi.scanDelete();

  pBLEScan->start(scanTime, false);
  pBLEScan->clearResults();

  displayDevices();

  delay(2000);
}

bool isDuplicate(const char* mac) {
  for (int i = 0; i < deviceIndex; i++) {
    if (deviceList[i].mac == String(mac)) {
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
