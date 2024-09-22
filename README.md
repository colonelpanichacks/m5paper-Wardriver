M5Paper Wardriver

This project scans for WiFi and BLE devices using the M5Paper, logs the data to an SD card (wigle.net compatible CSV), and displays it on the M5Paper's screen.
Prerequisites

Before starting, ensure you have the following components and software set up:

    M5Paper Unit
    M5 GPS i2c Unit & Cable
    SD Card (formatted and ready for use)
    Arduino IDE (latest version)
    ESP32 Board Package - Installed via the Boards Manager:
        Open Arduino IDE and go to File > Preferences.
        In the "Additional Board Manager URLs" field, add:
        https://dl.espressif.com/dl/package_esp32_index.json.
        Go to the Sidebar > Boards Manager, search for ESP32, and click Install next to ESP32 by Espressif Systems.

Libraries Needed

The ESP32 board package includes some essential libraries, while others must be manually installed through the Library Manager.
Bundled Libraries (Included with ESP32 Core)

    WiFi - For scanning and connecting to WiFi networks.
    SD - For handling SD card operations.
    SPI - For SPI communication.
    ESP32 BLE Arduino - For Bluetooth Low Energy (BLE) functionality.

External Libraries (Requires Installation)

    M5EPD - For M5Paper-specific functionality.
    TinyGPS++ - For handling GPS data.

Installing External Libraries

    For TinyGPS++:
        In Arduino IDE, open the Library Manager via the Sidebar or Sketch > Include Library > Manage Libraries.
        Search for TinyGPS++ and click Install.
    For M5EPD:
        Similarly, search for M5EPD in the Library Manager and click Install.

Board Setup

    In Arduino IDE, go to Tools > Board and select M5Paper from the ESP32 list.
    Ensure the correct port is selected under Tools > Port (e.g., COM3 on Windows or /dev/cu.SLAB_USBtoUART on macOS).

Project Setup
Step 1: Clone or Download the .ino File

Download or clone the required code repository. Open the .ino file in Arduino IDE to prepare for uploading.
Step 2: Initialize SD Card

The SD card will be used to log detected WiFi networks, BLE devices, and GPS data. Make sure the card is formatted and ready for use in the M5Paper.
Step 3: Upload Code

After verifying that your board, port, and libraries are properly configured, click the Upload button in Arduino IDE to flash the code onto your M5Paper.

Once uploaded, your M5Paper will start logging WiFi, BLE, and GPS data to the SD card.
Step 4: View Output

The M5Paper will begin scanning for WiFi and BLE devices, displaying the results sorted by RSSI on its e-paper screen. The GPS data, along with WiFi and BLE scan results, will be logged to a CSV file on the SD card.
CSV Log Format

The following data is logged to the SD card in CSV format:

    MAC - Device MAC address
    SSID - WiFi network name (if applicable)
    AuthMode - Authentication type (for WiFi)
    FirstSeen - Timestamp when the device was first detected
    Channel - WiFi channel
    RSSI - Signal strength
    Latitude - GPS latitude
    Longitude - GPS longitude
    AltitudeMeters - Altitude from GPS
    AccuracyMeters - Accuracy of GPS data
    Type - WiFi or BLE

Disclaimer

This project is provided "as-is" without any express or implied warranty. In no event shall the author or contributors be held liable for any damages arising from the use of this software. By using this code, you agree to take full responsibility for any consequences, direct or indirect, resulting from its use.

Use at your own risk. This software is open-source and intended for educational and experimental purposes only. The author disclaims any responsibility for any issues that may arise from the use of this project, including, but not limited to, loss of data, hardware damage, or violations of legal regulations.

For further information, please review the terms of the MIT License.
