#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>   // MQTT library
#include <ArduinoJson.h>
#include <Wire.h>
#include "mpu6500.h"

// ====== MIDI NOTES ======
#define ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))
#define A4 465
#define C5 548
#define Db5 647
#define E5 684
#define Gb4 440
#define D5 612
#define G5 809
#define Fb5 765
#define F5 723
#define B4 519
#define Ab4 491

const int midi1[25][3] = {
 {A4, 166, 167},
 {C5, 166, 167},
 {Db5, 124, 126},
 {E5, 78, 172},
 {Gb4, 83, 84},
 {A4, 83, 84},
 {C5, 83, 84},
 {D5, 83, 84},
 {C5, 83, 84},
 {Db5, 83, 84},
 {Db5, 78, 5},
 {E5, 237, 180},
 {G5, 83, 84},
 {Fb5, 83, 84},
 {F5, 83, 84},
 {Db5, 83, 84},
 {D5, 83, 84},
 {C5, 83, 84},
 {B4, 83, 84},
 {Ab4, 83, 84},
 {B4, 83, 84},
 {C5, 83, 84},
 {D5, 83, 84},
 {Db5, 83, 84},
 {E5, 1000, 0},
};

// ====== CONFIG ======
const char* ssid     = "snailshouse";
const char* password = "chiffons";
const char* serverBase = "http://139.59.119.186:8000";
String deviceId = "DEV024";

// MQTT config
const char* mqttServer = "139.59.119.186";   // same VPS or wherever Mosquitto is
const int   mqttPort   = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

String mqttCmdTopic;  // will be "motor/<deviceId>/cmd"

// Pins
const int buttonPin = 4;
const int buzzerPin = 13;
const int gpsRxPin = 16;
const int gpsTxPin = 17;
const int sdaPin = 21;  // I2C SDA for MPU6500
const int sclPin = 22;  // I2C SCL for MPU6500

// GPS
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// MPU6500 (using invensense-imu library)
bfs::Mpu6500 mpu;
bool mpuInitialized = false;
int mpuFailCount = 0;
float accelThreshold = 1.5;  // m/s^2 threshold for motion detection
unsigned long lastMotionTime = 0;
const unsigned long motionCooldown = 2000; // 2 seconds between motion detections
int motionCount = 0;  // Motion detections within time window
unsigned long motionWindowStart = 0;  // Start of detection window
const unsigned long motionWindowTime = 500;  // 500ms window
const int motionRequiredCount = 2;  // Require 2 detections within window

// State
bool isArmed = true;

// Timers
unsigned long lastButtonPress = 0;
const unsigned long buttonDebounceTime = 700; // ms
unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 15000; // 15 seconds

// Background HTTP task
QueueHandle_t alertQueue;
TaskHandle_t httpTaskHandle;

#define ALERT_MSG_SIZE 64
struct AlertMessage {
  char status[ALERT_MSG_SIZE];
  double lat;
  double lon;
};

// ===============================================
// BUZZER FUNCTIONS
// ===============================================

void playMidi(int pin, const int notes[][3], size_t len) {
  for (int i = 0; i < len; i++) {
    tone(pin, notes[i][0]);
    delay(notes[i][1]);
    noTone(pin);
    delay(notes[i][2]);
  }
}

void beepBuzzer(int times, int onMs, int offMs) {
  pinMode(buzzerPin, OUTPUT);
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

// ===============================================
// NETWORKING FUNCTIONS
// ===============================================

// Queue an alert to be sent in the background (non-blocking)
void queueAlert(const String& status) {
  AlertMessage msg;
  strncpy(msg.status, status.c_str(), ALERT_MSG_SIZE - 1);
  msg.status[ALERT_MSG_SIZE - 1] = '\0';
  
  if (gps.location.isValid()) {
    msg.lat = gps.location.lat();
    msg.lon = gps.location.lng();
    Serial.print("[GPS] Using position: ");
    Serial.print(msg.lat, 6);
    Serial.print(", ");
    Serial.println(msg.lon, 6);
  } else {
    msg.lat = 0;
    msg.lon = 0;
    Serial.println("[GPS] No valid fix, using 0,0");
  }
  
  if (xQueueSend(alertQueue, &msg, 0) == pdTRUE) {
    Serial.print("[ALERT] Queued: ");
    Serial.println(status);
  } else {
    Serial.println("[ALERT] Queue full, alert dropped!");
  }
}

// Background task that sends HTTP alerts without blocking main loop
void httpTask(void *pvParameters) {
  AlertMessage msg;
  
  for (;;) {
    // Wait for alert messages in the queue
    if (xQueueReceive(alertQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[HTTP] WiFi not connected, skipping alert");
        continue;
      }
      
      String url = String(serverBase) + "/api/alert";
      HTTPClient http;
      http.begin(url);
      http.addHeader("Content-Type", "application/json");
      http.setTimeout(3000); // 3 second timeout

      String payload = "{";
      payload += "\"device_id\":\"" + deviceId + "\",";
      payload += "\"status\":\"" + String(msg.status) + "\",";
      payload += "\"lat\":" + String(msg.lat, 6) + ",";
      payload += "\"lon\":" + String(msg.lon, 6);
      payload += "}";

      Serial.print("[HTTP] Sending: ");
      Serial.println(payload);

      int code = http.POST(payload);
      Serial.print("[HTTP] Status: ");
      Serial.println(code);

      if (code > 0) {
        String resp = http.getString();
        Serial.print("[HTTP] Response: ");
        Serial.println(resp);
      }
      http.end();
    }
  }
}

// Pull current armed_state from backend and sync isArmed
void syncArmStateFromServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[SYNC] WiFi not connected, cannot sync arm state");
    return;
  }

  String url = String(serverBase) + "/devices/" + deviceId + "/current";
  HTTPClient http;
  http.begin(url);
  Serial.print("[SYNC] GET ");
  Serial.println(url);

  int code = http.GET();
  Serial.print("[SYNC] HTTP status: ");
  Serial.println(code);

  if (code > 0) {
    String body = http.getString();
    Serial.print("[SYNC] Body: ");
    Serial.println(body);

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      Serial.print("[SYNC] JSON parse failed: ");
      Serial.println(err.f_str());
    } else {
      const char* state = doc["armed_state"];
      if (state) {
        isArmed = (String(state) == "armed");
        Serial.print("[SYNC] Armed state from server: ");
        Serial.println(isArmed ? "ARMED" : "DISARMED");
      } else {
        Serial.println("[SYNC] No 'armed_state' in JSON");
      }
    }
  } else {
    Serial.println("[SYNC] HTTP GET failed");
  }

  http.end();
}

// [MQTT] Handle command string from MQTT payload
void handleCommandString(const String &resp) {
  Serial.print("[MQTT] Parsing command: ");
  Serial.println(resp);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    Serial.print("[MQTT] JSON parse failed: ");
    Serial.println(err.f_str());
    return;
  }

  const char* command = doc["command"];
  if (!command) {
    Serial.println("[MQTT] No 'command' field");
    return;
  }

  String cmd = String(command);
  Serial.print("[MQTT] Command parsed: ");
  Serial.println(cmd);

  if (cmd == "ARM") {
    Serial.println("[ACTION] ARM");
    isArmed = true;
    queueAlert("System Armed");

  } else if (cmd == "DISARM") {
    Serial.println("[ACTION] DISARM");
    isArmed = false;
    queueAlert("System Disarmed");

  } else if (cmd == "BUZZ") {
    Serial.println("[ACTION] BUZZ");
    longAlarm();
    queueAlert("BUZZ Executed");

  } else if (cmd == "REQUEST_POSITION") {
    Serial.println("[ACTION] REQUEST_POSITION");
    queueAlert("Posisi Diminta");

  } else {
    Serial.print("[MQTT] Unknown command: ");
    Serial.println(cmd);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.println(msg);

  // Only handle our command topic, just in case
  if (String(topic) == mqttCmdTopic) {
    handleCommandString(msg);
  }
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("[MQTT] Attempting connection...");
    String clientId = "motor-esp32-";
    clientId += deviceId;

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      mqttClient.subscribe(mqttCmdTopic.c_str());
      Serial.print("[MQTT] Subscribed to ");
      Serial.println(mqttCmdTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


// ===============================================
// SETUP
// ===============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[BOOT] ESP32 Anti-Theft Tracker");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Initialize I2C for MPU6500
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(50000);  // 50kHz I2C
  Serial.println("[BOOT] Initializing MPU6500...");
  
  // Configure I2C bus and address (0x68 is primary address)
  mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  
  if (mpu.Begin()) {
    Serial.println("[BOOT] MPU6500 initialized successfully");
    
    // Set sample rate divider (99 = 10Hz with 1kHz base rate)
    if (mpu.ConfigSrd(99)) {
      Serial.println("[BOOT] MPU6500 sample rate configured (10Hz)");
    } else {
      Serial.println("[BOOT] Warning: Failed to configure sample rate");
    }
    
    mpuInitialized = true;
  } else {
    Serial.println("[BOOT] Failed to initialize MPU6500 - continuing without it");
    mpuInitialized = false;
  }

  Serial.println("[BOOT] Playing startup melody...");
  playMidi(buzzerPin, midi1, ARRAY_LEN(midi1));

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

  // NEW: sync initial armed/disarmed state from backend
  syncArmStateFromServer();

  // [MQTT] Initialize MQTT
  mqttCmdTopic = "motor/" + deviceId + "/cmd";
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  Serial.println("[BOOT] MQTT configured");

  // Create background HTTP task
  alertQueue = xQueueCreate(10, sizeof(AlertMessage));
  xTaskCreatePinnedToCore(
    httpTask,
    "HTTPTask",
    8192,
    NULL,
    1,
    &httpTaskHandle,
    0  // Run on Core 0
  );
  Serial.println("[BOOT] Background HTTP task started");

  queueAlert("Device Booted");
  
  Serial.println("[BOOT] Setup complete.");
}

// ===============================================
// LOOP
// ===============================================
void loop() {
  // 1. Always feed the GPS parser
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Handle MQTT
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  unsigned long now = millis();

  // 3. Send heartbeat
  if (now - lastHeartbeat >= heartbeatInterval) {
    lastHeartbeat = now;
    queueAlert("Heartbeat");
  }

  // 5. Check the button (non-blocking)
  if (digitalRead(buttonPin) == LOW && (now - lastButtonPress > buttonDebounceTime)) {
    lastButtonPress = now;
    Serial.println("[EVENT] Button pressed -> movement detected");

    if (isArmed) {
      Serial.println("[EVENT] System armed -> triggering alarm + sending alert");
      longAlarm();
      queueAlert("Gerakan Terdeteksi (Button)");
    } else {
      Serial.println("[EVENT] System disarmed -> ignoring movement");
    }
  }

  // 6. Check MPU6500 for motion (non-blocking)
  static unsigned long lastMpuReadTime = 0;
  const unsigned long mpuReadInterval = 100;  // 10Hz

  if (mpuInitialized &&
      (now - lastMotionTime > motionCooldown) &&
      (now - lastMpuReadTime >= mpuReadInterval)) {

    lastMpuReadTime = now;

    // Try reading from MPU
    if (!mpu.Read()) {
      mpuFailCount++;
      Serial.print("[MPU] Read failed, count=");
      Serial.println(mpuFailCount);

      // If too many consecutive failures, try to recover the bus + sensor
      if (mpuFailCount > 10) {
        Serial.println("[MPU] Too many I2C errors, resetting I2C + MPU");

        Wire.end();
        delay(10);
        Wire.begin(sdaPin, sclPin);
        Wire.setClock(50000);

        mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
        if (mpu.Begin()) {
          Serial.println("[MPU] Re-init OK");
          if (mpu.ConfigSrd(99)) {
            Serial.println("[MPU] SRD reset OK");
          } else {
            Serial.println("[MPU] SRD reset FAILED");
          }
          mpuInitialized = true;
          mpuFailCount = 0;
        } else {
          Serial.println("[MPU] Re-init FAILED, disabling MPU");
          mpuInitialized = false;
        }
      }

      // On any failed read, skip motion logic this loop
      return;
    }

    // Successful read: reset fail counter and process motion
    mpuFailCount = 0;

    // Get acceleration values in m/s^2
    float accel_x = mpu.accel_x_mps2();
    float accel_y = mpu.accel_y_mps2();
    float accel_z = mpu.accel_z_mps2();

    float accelMagnitude = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    float deviation = fabs(accelMagnitude - 9.81f);

    if (deviation > accelThreshold) {
      // Start new window if this is first detection or window expired
      if (motionCount == 0 || (now - motionWindowStart > motionWindowTime)) {
        motionWindowStart = now;
        motionCount = 1;
        Serial.print("[MPU6500] Motion #1 detected! Deviation: ");
        Serial.println(deviation);
      } else {
        // Within the window, increment count
        motionCount++;
        Serial.print("[MPU6500] Motion #");
        Serial.print(motionCount);
        Serial.print(" detected! Deviation: ");
        Serial.print(deviation);
        Serial.print(" m/s^2 (within ");
        Serial.print(now - motionWindowStart);
        Serial.println("ms window)");
        
        // Trigger alarm if we hit the threshold within the window
        if (motionCount >= motionRequiredCount) {
          lastMotionTime = now;
          motionCount = 0;  // Reset counter
          
          if (isArmed) {
            Serial.println("[EVENT] System armed -> triggering alarm + sending alert");
            longAlarm();
            queueAlert("Gerakan Terdeteksi (MPU6500)");
          } else {
            Serial.println("[EVENT] System disarmed -> ignoring MPU motion");
          }
        }
      }
    } else {
      // Reset if window expires without reaching threshold
      if (motionCount > 0 && (now - motionWindowStart > motionWindowTime)) {
        Serial.println("[MPU6500] Window expired, resetting counter");
        motionCount = 0;
      }
    }
  }
}
