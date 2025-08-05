#include <WiFi.h>
#include <HTTPClient.h>
#include "time.h"
#include <ModbusMaster.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Update.h>
const char* ssid = "Airtel_KUPPISMART";
const char* password = "Kuppismart@23";
#define GITHUB_USER "vishnukechem"
#define GITHUB_REPO "water_quality"
#define CURRENT_VERSION "v1.1"
const unsigned long updateInterval = 1 * 60 * 1000;
unsigned long lastCheckTime = 0;

const char* serverURL = "https://api.livestockify.com/api/iot/post/data";  //"https://api.livestockify.com/api/iot/data";
#define MAX485_DE_RE 4                                                     // RS485 Direction Control Pin
#define TX_PIN 17                                                          // ESP32 TX2
#define RX_PIN 16                                                          // Replace with actual API URL
ModbusMaster node;
HardwareSerial RS485Serial(2);
float ph, DO, cod, bod, tss, Temp = NAN;
float LEVEL = NAN;
//int  api_id = 191962;
int api_id = 191961;

void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH);
}
void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW);
}
// WiFi and MQTT
WiFiClientSecure net;
PubSubClient client(net);

// NTP Server for timestamp
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // Adjust for your timezone (e.g., 19800 for IST)
const int daylightOffset_sec = 0;


// === Firmware Update ===
void checkForUpdates() {
  String latestVersion = getLatestVersion();
  if (!latestVersion.isEmpty() && latestVersion != CURRENT_VERSION) {
    Serial.println("New version found! Updating...");
    updateFirmware(latestVersion);
  } else {
    Serial.println("Already up to date (Version: " CURRENT_VERSION ")");
  }
}

String getLatestVersion() {
  HTTPClient http;
  String latestVersion = "";
  String url = "https://api.github.com/repos/" GITHUB_USER "/" GITHUB_REPO "/releases/latest";
  Serial.println("Checking for updates...");
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, http.getString());
    latestVersion = doc["tag_name"].as<String>();
    Serial.println("Latest Version: " + latestVersion);
  } else {
    Serial.println("Failed to check updates. HTTP: " + String(httpCode));
  }
  http.end();
  return latestVersion;
}

void updateFirmware(String version) {
  HTTPClient http;
  String firmwareURL = "https://github.com/" GITHUB_USER "/" GITHUB_REPO "/releases/download/" + version + "/firmware.bin";
  Serial.println("Downloading: " + firmwareURL);
  http.begin(firmwareURL);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    WiFiClient& stream = http.getStream();
    size_t totalSize = http.getSize();
    if (totalSize < 100000) {
      Serial.println("Invalid firmware size, skipping update.");
      return;
    }
    if (Update.begin(totalSize)) {
      Serial.println("Updating firmware...");
      size_t written = Update.writeStream(stream);
      if (written == totalSize && Update.end()) {
        Serial.println("Update Successful! Restarting...");
        ESP.restart();
      } else {
        Serial.println("Update Failed!");
      }
    }
  } else {
    Serial.println("Firmware fetch failed. HTTP: " + String(httpCode));
  }
  http.end();
}

// Convert IEEE 754 float from ABCD format
float convertABCD(uint16_t reg1, uint16_t reg2) {
  union {
    float value;
    uint8_t bytes[4];
  } data;
  data.bytes[0] = (reg1 >> 8) & 0xFF;
  data.bytes[1] = reg1 & 0xFF;
  data.bytes[2] = (reg2 >> 8) & 0xFF;
  data.bytes[3] = reg2 & 0xFF;
  return data.value;
}


float convertIEEE754Float(uint16_t registers[]) {
  union {
    uint32_t u32;
    float f;
  } converter;
  converter.u32 = ((uint32_t)registers[0] << 16) | registers[1];
  return converter.f;
}
/*
void processPressureData(uint16_t reg1, uint16_t reg2) {
    uint16_t registers[2] = {reg1, reg2};
    float pressure = convertIEEE754Float(registers);
     LEVEL = pressure * 101.94;
    Serial.printf("Final Pressure Reading: %.5f MPa\n", pressure);
    Serial.printf("Equivalent Water Height: %.2f meters\n", LEVEL);
}*/
float convertScaledValue(uint16_t rawValue, uint16_t decimalPlaces) {
  return rawValue / pow(10, decimalPlaces);
}

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi. Check credentials or signal strength.");
  }
}



String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "1970-01-01T00:00:00Z";  // Default timestamp if NTP fails
  }

  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
}
void sendSensorData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    // Read Sensor Values (Replace with actual sensor readings)

    // Create JSON Object
    StaticJsonDocument<256> doc;
    doc["deviceId"] = api_id;       // Assign the device ID
    doc["apiKey"] = "iotdata2025";  // Add API key
    //doc["timestamp"] = getTimestamp();  // Use real timestamp function

    // Create the "sensors" object
    JsonObject sensors = doc.createNestedObject("sensors");
    sensors["pH"] = ph;
    sensors["DO"] = DO;
    sensors["BOD"] = bod;
    sensors["COD"] = cod;
    sensors["TSS"] = tss;           // Add a placeholder for humidity (replace with actual data)
    sensors["Level"] = LEVEL;       // Assuming LEVEL represents pressure
    sensors["temperature"] = Temp;  // Use the correct temperature variable

    // Convert JSON to String
    String jsonData;
    serializeJson(doc, jsonData);

    Serial.println("Sending JSON:");
    Serial.println(jsonData);


    // Send HTTP POST Request
    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      Serial.println("Data sent successfully! Response code: " + String(httpResponseCode));
      Serial.println("Response: " + http.getString());
    } else {
      Serial.println("Error sending data! HTTP code: " + String(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected! Reconnecting...");
    WiFi.begin(ssid, password);
  }
}


void setup() {
  Serial.begin(115200);
  RS485Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  connectToWiFi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);  // Initialize NTP
}

void loop() {
  sendSensorData();
  readSensors();
  delay(5000);
    if (millis() - lastCheckTime >= updateInterval) {
    lastCheckTime = millis();
    checkForUpdates();
}}
void readSensors() {
  uint8_t result;

  // **Read from Slave ID 2**
  node.begin(2, RS485Serial);
  Serial.println("📡 Reading from Slave ID 2...");


  result = node.readHoldingRegisters(0x2604, 2);
  if (result == node.ku8MBSuccess) {
    DO = convertABCD(node.getResponseBuffer(0), node.getResponseBuffer(1));
    Serial.printf("💧 DO (Slave 2): %.2f mg/L\n", DO);
  } else {
    Serial.printf("❌ Error reading DO from Slave 2: 0x%02X\n", result);
  }

  delay(500);

  // **Read from Slave ID 4**
  node.begin(9, RS485Serial);


result = node.readHoldingRegisters(0x0000, 2);
if (result == node.ku8MBSuccess) {
  cod = convertScaledValue(node.getResponseBuffer(0), node.getResponseBuffer(1));
  Serial.printf("💧 COD (Slave 4): %.2f mg/L\n", cod);
} else {
  Serial.printf("❌ Error reading COD from Slave 4: 0x%02X\n", result);
}


  delay(500);

result = node.readHoldingRegisters(0x0006, 2);
if (result == node.ku8MBSuccess) {
  bod = convertScaledValue(node.getResponseBuffer(0), node.getResponseBuffer(1));
  Serial.printf("🌱 BOD (Slave 4): %.2f mg/L\n", bod);
} else {
  Serial.printf("❌ Error reading BOD from Slave 4: 0x%02X\n", result);
}


  delay(500);



result = node.readHoldingRegisters(0x0004, 2);
if (result == node.ku8MBSuccess) {
  tss = convertScaledValue(node.getResponseBuffer(0), node.getResponseBuffer(1));
  Serial.printf("🪨 TSS (Slave 4): %.2f mg/L\n", tss);
} else {
  Serial.printf("❌ Error reading TSS from Slave 4: 0x%02X\n", result);
}

  delay(500);

  // **Read pH from Slave 3**
  node.begin(3, RS485Serial);
  Serial.println("📡 Reading pH from Slave ID 3...");

  result = node.readHoldingRegisters(0x0000, 2);
  if (result == node.ku8MBSuccess) {
    ph = convertScaledValue(node.getResponseBuffer(0), node.getResponseBuffer(1));
    Serial.printf("🔬 pH (Slave 3): %.2f\n", ph);
  } else {
    Serial.printf("❌ Error reading pH from Slave 3: 0x%02X\n", result);
  }

  delay(500);

  result = node.readHoldingRegisters(0x0002, 2);
  if (result == node.ku8MBSuccess) {
    Temp = convertScaledValue(node.getResponseBuffer(0), node.getResponseBuffer(1));
    Serial.printf("🔬 temp (Slave 3): %.2f\n", Temp);
  } else {
    //  Serial.printf("❌ Error reading temp from Slave 3: 0x%02X\n", result);
  }

  delay(500);
  node.begin(1, RS485Serial);
  // **Read Pressure from Slave 1**
  result = node.readHoldingRegisters(0x0000, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t reg0 = node.getResponseBuffer(0);
    uint16_t reg1 = node.getResponseBuffer(1);

    uint32_t combined = ((uint32_t)reg0 << 16) | reg1;

    memcpy(&LEVEL, &combined, sizeof(LEVEL));

    Serial.print("LEVEL: ");
    Serial.print(LEVEL, 2);
    Serial.println(" m");

    Serial.println("----------------------");
    delay(15000);  // Final delay to prevent constant resets
  }
}
