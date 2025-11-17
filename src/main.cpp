#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ====== CONFIG ======
const char* ssid     = "snailshouse";
const char* password = "chiffons";

// Change this to your PC's IP running FastAPI
const char* serverBase = "http://139.59.119.186:8000";

// Must match device_id used in backend
String deviceId = "DEV024";

// Pins
const int buttonPin = 4;   // simulate motion (active LOW)
const int buzzerPin = 5;   // active buzzer

// GPS on Serial1 (adjust pins)
const int gpsRxPin = 16;   // ESP32 RX <-- GPS TX
const int gpsTxPin = 17;   // ESP32 TX --> GPS RX (often unused)

// Timing
unsigned long lastCommandCheck = 0;
const unsigned long commandInterval = 2000; // ms

unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 15000; // ms

bool isArmed = true;

// GPS
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

void beepBuzzer(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(onMs);
    digitalWrite(buzzerPin, LOW);
    delay(offMs);
  }
}

void longAlarm() {
  beepBuzzer(5, 200, 100);
}

void sendAlert(const String& status) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ALERT] WiFi not connected, cannot send");
    return;
  }

  double lat = 0;
  double lon = 0;

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  } else {
    Serial.println("[ALERT] GPS not valid, sending lat=0, lon=0");
  }

  String url = String(serverBase) + "/api/alert";

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = "{";
  payload += "\"device_id\":\"" + deviceId + "\",";
  payload += "\"status\":\"" + status + "\",";
  payload += "\"lat\":" + String(lat, 6) + ",";
  payload += "\"lon\":" + String(lon, 6);
  payload += "}";

  Serial.print("[ALERT] Sending payload: ");
  Serial.println(payload);

  int code = http.POST(payload);
  Serial.print("[ALERT] HTTP status: ");
  Serial.println(code);

  if (code > 0) {
    String resp = http.getString();
    Serial.print("[ALERT] Response: ");
    Serial.println(resp);
  }

  http.end();
}

void checkCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastCommandCheck < commandInterval) return;
  lastCommandCheck = now;

  String url = String(serverBase) + "/api/commands/" + deviceId;

  HTTPClient http;
  http.begin(url);
  int code = http.GET();

  if (code == 200) {
    String resp = http.getString();
    Serial.print("[CMD] Response: ");
    Serial.println(resp);

    if (resp.indexOf("\"command\":\"BUZZ\"") >= 0) {
      Serial.println("[CMD] BUZZ received");
      longAlarm();
      sendAlert("BUZZ Executed");

    } else if (resp.indexOf("\"command\":\"ARM\"") >= 0) {
      Serial.println("[CMD] ARM received");
      isArmed = true;
      sendAlert("System Armed");

    } else if (resp.indexOf("\"command\":\"DISARM\"") >= 0) {
      Serial.println("[CMD] DISARM received");
      isArmed = false;
      sendAlert("System Disarmed");

    } else if (resp.indexOf("\"command\":\"REQUEST_POSITION\"") >= 0) {
      Serial.println("[CMD] REQUEST_POSITION received");
      sendAlert("Posisi Diminta");

    } else if (resp.indexOf("\"command\":") >= 0 &&
               resp.indexOf("\"command\":\"null\"") == -1 &&
               resp.indexOf("\"command\": null") == -1) {
      Serial.println("[CMD] Custom command received");
      sendAlert("Custom Command Executed");
    }

  } else {
    Serial.print("[CMD] GET failed, code=");
    Serial.println(code);
  }

  http.end();
}

void maybeSendHeartbeat() {
  unsigned long now = millis();
  if (now - lastHeartbeat < heartbeatInterval) return;
  lastHeartbeat = now;

  Serial.println("[HEARTBEAT] Sending heartbeat");
  sendAlert("Heartbeat");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[BOOT] ESP32 Anti-Theft Tracker");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  gpsSerial.begin(9600, SERIAL_8N1, gpsRxPin, gpsTxPin);
  Serial.println("[BOOT] GPS serial started");

  Serial.print("[BOOT] Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[BOOT] WiFi connected");
  Serial.print("[BOOT] ESP IP: ");
  Serial.println(WiFi.localIP());

  sendAlert("Device Booted");
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  static unsigned long lastGpsPrint = 0;
  unsigned long now = millis();
  if (now - lastGpsPrint > 2000) {
    lastGpsPrint = now;
    if (gps.location.isValid()) {
      Serial.print("[GPS] Lat: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Lon: ");
      Serial.print(gps.location.lng(), 6);
      Serial.print(" Sat: ");
      Serial.print(gps.satellites.value());
      Serial.print(" HDOP: ");
      Serial.println(gps.hdop.value());
    } else {
      Serial.println("[GPS] No valid fix yet");
    }
  }

  if (digitalRead(buttonPin) == LOW) {
    Serial.println("[EVENT] Button pressed -> movement detected");

    if (isArmed) {
      Serial.println("[EVENT] System armed -> triggering alarm + alert");
      longAlarm();
      sendAlert("Gerakan Terdeteksi");
    } else {
      Serial.println("[EVENT] System disarmed -> ignoring movement");
    }

    delay(700);
  }

  checkCommands();
  maybeSendHeartbeat();
}
