#include <Arduino.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

const char* ssid     = "snailshouse";
const char* password = "chiffons";

// ganti dengan IP PC kamu
const char* baseUrl = "http://10.116.165.183:8000";

const int buttonPin = 4;
const int buzzerPin = 26;

String deviceId = "MOTOR-ABC123";

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

unsigned long lastCommandCheck = 0;
const unsigned long commandInterval = 2000; // 2s

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

void sendAlert(const char* status) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot send alert");
    return;
  }

  String latField;
  String lonField;

  if (gps.location.isValid()) {
    latField = String(gps.location.lat(), 6);
    lonField = String(gps.location.lng(), 6);
  } else {
    Serial.println("GPS not valid, sending \"invalid\"");
    latField = String("\"invalid\"");
    lonField = String("\"invalid\"");
  }

  String url = String(baseUrl) + "/api/alert";

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String payload = "{";
  payload += "\"device_id\":\"" + deviceId + "\",";
  payload += "\"status\":\"" + String(status) + "\",";
  payload += "\"lat\":" + latField + ",";
  payload += "\"lon\":" + lonField;
  payload += "}";

  Serial.println("Sending alert: " + payload);

  int code = http.POST(payload);
  Serial.print("HTTP code: ");
  Serial.println(code);

  if (code > 0) {
    String resp = http.getString();
    Serial.println("Response: " + resp);
  }
  http.end();
}

void checkCommands() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();
  if (now - lastCommandCheck < commandInterval) {
    return;
  }
  lastCommandCheck = now;

  String url = String(baseUrl) + "/api/commands/" + deviceId;
  HTTPClient http;
  http.begin(url);
  int code = http.GET();

  if (code == 200) {
    String resp = http.getString();
    Serial.println("Command response: " + resp);

    // simple manual parse (cheap way)
    if (resp.indexOf("\"command\":\"BUZZ\"") >= 0) {
      Serial.println("Executing BUZZ command");
      longAlarm();
    } else if (resp.indexOf("\"command\":\"SILENT\"") >= 0) {
      Serial.println("Received SILENT command (no action)");
    }
  } else {
    Serial.print("Command GET failed, code=");
    Serial.println(code);
  }

  http.end();
}

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  gpsSerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX ke GPS

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // keep feeding GPS parser
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // button pressed = simulate movement detected
  if (digitalRead(buttonPin) == LOW) {
    Serial.println("Button pressed -> movement detected");
    longAlarm();                     // local alarm
    sendAlert("Gerakan Terdeteksi"); // send to server
    delay(1000);                     // debouncing / anti-spam
  }

  // poll server for commands
  checkCommands();
}
