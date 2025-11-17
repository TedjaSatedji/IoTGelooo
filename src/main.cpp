#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

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

// Pins
const int buttonPin = 4;
const int buzzerPin = 5;
const int gpsRxPin = 16;
const int gpsTxPin = 17;

// GPS
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

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

// [NEW] This function is now ONLY called by the network task on Core 0
void checkCommands() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url = String(serverBase) + "/api/commands/" + deviceId;
  HTTPClient http;
  http.begin(url);

  // [BLOCKING] This GET call now blocks Core 0, which is fine!
  int code = http.GET();

  if (code == 200) {
    String resp = http.getString();
    Serial.print("[CMD] Response: ");
    Serial.println(resp);

    // We use the mutex to safely change the 'isArmed' variable
    if (resp.indexOf("\"command\":\"ARM\"") >= 0) {
      Serial.println("[CMD] ARM received");
      // [NEW] Safely write to the shared variable
      if (xSemaphoreTake(armMutex, portMAX_DELAY) == pdTRUE) {
        isArmed = true;
        xSemaphoreGive(armMutex);
      }
      sendAlert("System Armed"); // This is fine, we are on Core 0

    } else if (resp.indexOf("\"command\":\"DISARM\"") >= 0) {
      Serial.println("[CMD] DISARM received");
      // [NEW] Safely write to the shared variable
      if (xSemaphoreTake(armMutex, portMAX_DELAY) == pdTRUE) {
        isArmed = false;
        xSemaphoreGive(armMutex);
      }
      sendAlert("System Disarmed");

    } else if (resp.indexOf("\"command\":\"BUZZ\"") >= 0) {
      Serial.println("[CMD] BUZZ received");
      longAlarm(); // This is fine, it uses the buzzer mutex
      sendAlert("BUZZ Executed");

    } else if (resp.indexOf("\"command\":\"REQUEST_POSITION\"") >= 0) {
      Serial.println("[CMD] REQUEST_POSITION received");
      sendAlert("Posisi Diminta");
    }
    // [FIXED] No more "custom command" bug

  } else {
    Serial.print("[CMD] GET failed, code=");
    Serial.println(code);
  }
  http.end();
}


// [NEW] This is the main function for our network task on Core 0
void networkTask(void *pvParameters) {
  Serial.println("[TASK] Network task started on Core 0");

  unsigned long lastCommandCheck = 0;
  unsigned long lastHeartbeat = 0;
  const unsigned long commandInterval = 3000;
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

    // 2. Check for alerts from the queue
    AlertMessage alertMessage;
    // [NEW] Check the queue. '0' timeout means it returns instantly
    if (xQueueReceive(alertQueue, &alertMessage, (TickType_t)0) == pdTRUE) {
      Serial.print("[TASK] Received alert from queue: ");
      Serial.println(alertMessage.message);
      sendAlert(alertMessage.message); // This is a blocking call on Core 0
    }

    unsigned long now = millis();

    // 3. Check for commands (non-blocking timer)
    if (now - lastCommandCheck >= commandInterval) {
      lastCommandCheck = now;
      checkCommands(); // This is a blocking call on Core 0
    }

    // 4. Send heartbeat (non-blocking timer)
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
      strncpy(alertMsg.message, "Gerakan Terdeteksi", ALERT_MSG_SIZE - 1);
      alertMsg.message[ALERT_MSG_SIZE - 1] = '\0';
      xQueueSend(alertQueue, &alertMsg, portMAX_DELAY);

    } else {
      Serial.println("[EVENT] System disarmed -> ignoring movement");
    }
  }

  // [REMOVED]
  // checkCommands();
  // maybeSendHeartbeat();
  // All networking is now on Core 0!
  
  // This loop now finishes in microseconds.
}
