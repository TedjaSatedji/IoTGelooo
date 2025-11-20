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
const int buzzerPin = 5;
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
float accelThreshold = 5.5;  // m/s^2 threshold for motion detection
unsigned long lastMotionTime = 0;
const unsigned long motionCooldown = 2000; // 2 seconds between motion detections

// [NEW] FreeRTOS handles for multi-core coordination
TaskHandle_t networkTaskHandle; // Handle for our Core 0 task
QueueHandle_t alertQueue;       // Queue to send alert messages from Core 1 to Core 0
SemaphoreHandle_t armMutex;     // Mutex to protect the 'isArmed' variable
SemaphoreHandle_t buzzerMutex;  // Mutex to protect the buzzer (so cores don't fight for it)

// [FIX] Use fixed-size char array for queue messages instead of String
#define ALERT_MSG_SIZE 64
typedef struct {
  char message[ALERT_MSG_SIZE];
} AlertMessage;

// [NEW] 'volatile' keyword is critical for variables shared between cores
volatile bool isArmed = true;

// [NEW] Non-blocking timer for the button debounce
unsigned long lastButtonPress = 0;
const unsigned long buttonDebounceTime = 700; // ms

// ===============================================
// BUZZER FUNCTIONS (Now with Mutex)
// ===============================================

void playMidi(int pin, const int notes[][3], size_t len) {
  // [NEW] Wait to get exclusive access to the buzzer
  if (xSemaphoreTake(buzzerMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < len; i++) {
      tone(pin, notes[i][0]);
      delay(notes[i][1]); // delay() is okay here, it's inside a protected task
      noTone(pin);
      delay(notes[i][2]);
    }
    // [NEW] Release the buzzer for other tasks
    xSemaphoreGive(buzzerMutex);
  }
}

void beepBuzzer(int times, int onMs, int offMs) {
  if (xSemaphoreTake(buzzerMutex, portMAX_DELAY) == pdTRUE) {
    pinMode(buzzerPin, OUTPUT);
    for (int i = 0; i < times; i++) {
      digitalWrite(buzzerPin, HIGH);
      delay(onMs);
      digitalWrite(buzzerPin, LOW);
      delay(offMs);
    }
    xSemaphoreGive(buzzerMutex);
  }
}

void longAlarm() {
  beepBuzzer(5, 200, 100); // This function now automatically uses the mutex
}

// ===============================================
// NETWORKING FUNCTIONS (Will run on Core 0)
// ===============================================

// [NEW] This function is now ONLY called by the network task on Core 0
void sendAlert(const String& status) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ALERT] WiFi not connected, cannot send");
    return;
  }

  double lat = 0;
  double lon = 0;
  
  // No need for a mutex on GPS, Core 1 only writes to it, 
  // Core 0 only reads. The values might be *slightly* old, but won't crash.
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

  // [BLOCKING] This POST call now blocks Core 0, which is fine!
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
        bool newIsArmed = (String(state) == "armed");
        if (xSemaphoreTake(armMutex, portMAX_DELAY) == pdTRUE) {
          isArmed = newIsArmed;
          xSemaphoreGive(armMutex);
        }
        Serial.print("[SYNC] Armed state from server: ");
        Serial.println(newIsArmed ? "ARMED" : "DISARMED");
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
    if (xSemaphoreTake(armMutex, portMAX_DELAY) == pdTRUE) {
      isArmed = true;
      xSemaphoreGive(armMutex);
    }
    sendAlert("System Armed");

  } else if (cmd == "DISARM") {
    Serial.println("[ACTION] DISARM");
    if (xSemaphoreTake(armMutex, portMAX_DELAY) == pdTRUE) {
      isArmed = false;
      xSemaphoreGive(armMutex);
    }
    sendAlert("System Disarmed");

  } else if (cmd == "BUZZ") {
    Serial.println("[ACTION] BUZZ");
    longAlarm();
    sendAlert("BUZZ Executed");

  } else if (cmd == "REQUEST_POSITION") {
    Serial.println("[ACTION] REQUEST_POSITION");
    sendAlert("Posisi Diminta");

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
  // Loop until reconnected
  while (!mqttClient.connected()) {
    Serial.print("[MQTT] Attempting connection...");
    // Client ID can be anything unique
    String clientId = "motor-esp32-";
    clientId += deviceId;

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      // Subscribe to command topic
      mqttClient.subscribe(mqttCmdTopic.c_str());
      Serial.print("[MQTT] Subscribed to ");
      Serial.println(mqttCmdTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}


// [NEW] This is the main function for our network task on Core 0
void networkTask(void *pvParameters) {
  Serial.println("[TASK] Network task started on Core 0");

  unsigned long lastHeartbeat = 0;
  const unsigned long heartbeatInterval = 15000;

  // This is the infinite loop for Core 0
  for (;;) {
    // 1. Make sure WiFi is connected
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[TASK] WiFi disconnected. Reconnecting...");
      WiFi.reconnect();
      vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait 5s before retrying
      continue; // Skip the rest of the loop
    }

    // Ensure MQTT is connected
    if (!mqttClient.connected()) {
      mqttReconnect();
    }
    mqttClient.loop();  // process incoming MQTT packets

    // 2. Check for alerts from the queue
    AlertMessage alertMessage;
    // [NEW] Check the queue. '0' timeout means it returns instantly
    if (xQueueReceive(alertQueue, &alertMessage, (TickType_t)0) == pdTRUE) {
      Serial.print("[TASK] Received alert from queue: ");
      Serial.println(alertMessage.message);
      sendAlert(alertMessage.message); // This is a blocking call on Core 0
    }

    unsigned long now = millis();

    // 3. Send heartbeat (non-blocking timer)
    if (now - lastHeartbeat >= heartbeatInterval) {
      lastHeartbeat = now;
      Serial.println("[TASK] Queuing heartbeat");
      // [NEW] Don't send directly, add to the queue to be processed
      AlertMessage hbMsg;
      strncpy(hbMsg.message, "Heartbeat", ALERT_MSG_SIZE - 1);
      hbMsg.message[ALERT_MSG_SIZE - 1] = '\0';
      xQueueSend(alertQueue, &hbMsg, 0);
    }

    // [NEW] Give the scheduler a tiny break
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// ===============================================
// SETUP (Runs on Core 1)
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
  Wire.setClock(100000);  // 100kHz I2C (same as the working minimal test)
  Serial.println("[BOOT] Initializing MPU6500...");
  
  // Configure I2C bus and address (0x68 is primary address)
  mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  
  if (mpu.Begin()) {
    Serial.println("[BOOT] MPU6500 initialized successfully");
    
    // Set sample rate divider (19 = 50Hz with 1kHz base rate)
    if (mpu.ConfigSrd(19)) {
      Serial.println("[BOOT] MPU6500 sample rate configured (50Hz)");
    } else {
      Serial.println("[BOOT] Warning: Failed to configure sample rate");
    }
    
    mpuInitialized = true;
  } else {
    Serial.println("[BOOT] Failed to initialize MPU6500 - continuing without it");
    mpuInitialized = false;
  }

  // [NEW] Initialize the mutexes and queue
  armMutex = xSemaphoreCreateMutex();
  buzzerMutex = xSemaphoreCreateMutex();
  alertQueue = xQueueCreate(10, sizeof(AlertMessage)); // Queue for 10 AlertMessage structs

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

  // [NEW] Send the boot message to the queue
  AlertMessage bootMsg;
  strncpy(bootMsg.message, "Device Booted", ALERT_MSG_SIZE - 1);
  bootMsg.message[ALERT_MSG_SIZE - 1] = '\0';
  xQueueSend(alertQueue, &bootMsg, 0);

  // [NEW] Create the network task and pin it to Core 0
  xTaskCreatePinnedToCore(
    networkTask,       // Function to run
    "NetworkTask",     // Name (for debugging)
    10000,             // Stack size (networking needs a lot)
    NULL,              // Task input parameters (none)
    1,                 // Priority (1 is good)
    &networkTaskHandle, // Task handle
    0                  // Pin to Core 0 (Core 1 is default for Arduino)
  );
  
  Serial.println("[BOOT] Setup complete. Main loop (Core 1) starting.");
}

// ===============================================
// LOOP (Runs on Core 1 - Now super fast!)
// ===============================================
void loop() {
  // 1. Always feed the GPS parser
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Print GPS data (non-blocking)
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

  // 3. Check the button (non-blocking)
  // [NEW] Replaced delay(700) with a non-blocking millis() check
  if (digitalRead(buttonPin) == LOW && (now - lastButtonPress > buttonDebounceTime)) {
    lastButtonPress = now; // Reset the timer
    Serial.println("[EVENT] Button pressed -> movement detected");

    // [NEW] Safely read the shared variable
    bool currentlyArmed = false;
    if (xSemaphoreTake(armMutex, portMAX_DELAY) == pdTRUE) {
      currentlyArmed = isArmed; // Make a local copy
      xSemaphoreGive(armMutex);
    }

    if (currentlyArmed) {
      Serial.println("[EVENT] System armed -> triggering alarm + queuing alert");
      longAlarm(); // This is safe, it uses the buzzer mutex
      
      // [NEW] Instead of calling sendAlert(), we send a message to the queue
      AlertMessage alertMsg;
      strncpy(alertMsg.message, "Gerakan Terdeteksi (Button)", ALERT_MSG_SIZE - 1);
      alertMsg.message[ALERT_MSG_SIZE - 1] = '\0';
      xQueueSend(alertQueue, &alertMsg, portMAX_DELAY);

    } else {
      Serial.println("[EVENT] System disarmed -> ignoring movement");
    }
  }

  // 4. Check MPU6500 for motion (non-blocking)
  static unsigned long lastMpuReadTime = 0;
  const unsigned long mpuReadInterval = 20;  // ~50Hz, like the minimal test

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
        Wire.setClock(100000);

        mpu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
        if (mpu.Begin()) {
          Serial.println("[MPU] Re-init OK");
          if (mpu.ConfigSrd(19)) {
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
      lastMotionTime = now;
      Serial.print("[MPU6500] Motion detected! Accel deviation: ");
      Serial.print(deviation);
      Serial.println(" m/s^2");
      
      // Check if system is armed
      bool currentlyArmed = false;
      if (xSemaphoreTake(armMutex, portMAX_DELAY) == pdTRUE) {
        currentlyArmed = isArmed;
        xSemaphoreGive(armMutex);
      }
      
      if (currentlyArmed) {
        Serial.println("[EVENT] System armed -> triggering alarm + queuing alert");
        longAlarm();
        
        AlertMessage alertMsg;
        strncpy(alertMsg.message, "Gerakan Terdeteksi (MPU6500)", ALERT_MSG_SIZE - 1);
        alertMsg.message[ALERT_MSG_SIZE - 1] = '\0';
        xQueueSend(alertQueue, &alertMsg, portMAX_DELAY);
      } else {
        Serial.println("[EVENT] System disarmed -> ignoring MPU motion");
      }
    }
  }

  // [REMOVED]
  // All networking (HTTP alerts + MQTT commands) is now on Core 0!
  // No more HTTP polling for commands - everything comes via MQTT
  
  // This loop now finishes in microseconds.
}
