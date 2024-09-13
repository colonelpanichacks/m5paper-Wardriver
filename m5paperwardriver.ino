#include <M5EPD.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

M5EPD_Canvas canvas(&M5.EPD);
#define SD_CS_PIN 4

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

BLEScan* pBLEScan;
int scanTime = 5;

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
    logFile.println("netid,ssid,lat,lon,accuracy,time,frequency,encryption,signal,type");
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

void logToCSV(String netid, String ssid, double lat, double lon, double accuracy, String time, int frequency, String encryption, int signal, String type) {
  if (logFile) {
    logFile.printf("%s,%s,%.6f,%.6f,%.2f,%s,%d,%s,%d,%s\n",
                   netid.c_str(), ssid.c_str(), lat, lon, accuracy, time.c_str(), frequency, encryption.c_str(), signal, type.c_str());
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

  canvas.createCanvas(540, 960);  // Correct full vertical canvas size
  canvas.fillCanvas(0);
  canvas.setTextSize(2);
  canvas.drawString("Discovered Devices (sorted by RSSI):", 10, 10);

  int y = 40;
  for (int i = 0; i < deviceIndex; i++) {
    canvas.drawString(String(i+1) + ": " + deviceList[i].info, 10, y);
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

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String mac = advertisedDevice.getAddress().toString().c_str();
    String ssid = advertisedDevice.getName().c_str();
    int rssi = advertisedDevice.getRSSI();
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    double accuracy = gps.hdop.hdop();
    String time = String(millis());

    logToCSV(mac, ssid, lat, lon, accuracy, time, 0, "N/A", rssi, "BLE");

    deviceList[deviceIndex].type = "BLE";
    deviceList[deviceIndex].ssid = ssid;
    deviceList[deviceIndex].mac = mac;
    deviceList[deviceIndex].rssi = rssi;
    deviceList[deviceIndex].info = "BLE: " + ssid + " (" + mac + ") RSSI: " + String(rssi) + " dBm";
    deviceIndex++;

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

  int n = WiFi.scanNetworks();
  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      String ssid = WiFi.SSID(i);
      String bssid = WiFi.BSSIDstr(i);
      int rssi = WiFi.RSSI(i);
      String encryption = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured";
      int channel = WiFi.channel(i);
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      double accuracy = gps.hdop.hdop();
      String time = String(millis());

      logToCSV(bssid, ssid, lat, lon, accuracy, time, channel, encryption, rssi, "WiFi");

      deviceList[deviceIndex].type = "WiFi";
      deviceList[deviceIndex].ssid = ssid;
      deviceList[deviceIndex].mac = bssid;
      deviceList[deviceIndex].rssi = rssi;
      deviceList[deviceIndex].info = "WiFi: " + ssid + " (" + bssid + ") RSSI: " + String(rssi) + " dBm";
      deviceIndex++;

      displayDevices();
    }
  }

  pBLEScan->start(scanTime, false);
  pBLEScan->clearResults();

  delay(2000);
}
