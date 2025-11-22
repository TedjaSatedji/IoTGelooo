# ESP32 Anti-Theft Tracker

A real-time IoT anti-theft tracking system built with ESP32, featuring GPS tracking, motion detection, remote control via MQTT, and offline alert queueing.

## Features

### Core Functionality
- **Motion Detection**: Dual detection system using:
  - Physical button sensor
  - MPU6500 6-axis IMU (accelerometer/gyroscope)
- **GPS Tracking**: Real-time location tracking using TinyGPS++
- **Remote Control**: MQTT-based commands for arm/disarm and remote buzzer activation
- **Alert System**: HTTP-based alerts with offline queueing capability
- **Heartbeat Monitoring**: Periodic status updates every 15 seconds

### Smart Features
- **Offline Queue**: Stores up to 10 alerts when WiFi is unavailable
- **Auto-sync**: Automatically sends queued alerts when connection is restored
- **State Synchronization**: Syncs armed/disarmed state with backend server on startup
- **Non-blocking Design**: Background task handling for HTTP requests
- **Debouncing**: Button debouncing and motion cooldown to prevent false triggers
- **Multi-window Motion Detection**: Requires multiple motion detections within a time window to reduce false positives

## Hardware Requirements

### Components
- ESP32 Development Board
- MPU6500 6-axis IMU (I2C)
- GPS Module (UART, 9600 baud)
- Push Button
- Buzzer
- LED (onboard or external)

### Pin Configuration
```cpp
Button:     GPIO 4
Buzzer:     GPIO 13
LED:        GPIO 2 (onboard)
GPS RX:     GPIO 16
GPS TX:     GPIO 17
I2C SDA:    GPIO 21 (MPU6500)
I2C SCL:    GPIO 22 (MPU6500)
```

## Software Requirements

### PlatformIO Dependencies
- `Arduino.h`
- `WiFi.h`
- `HTTPClient.h`
- `TinyGPSPlus`
- `HardwareSerial`
- `PubSubClient` (MQTT)
- `ArduinoJson`
- `Wire.h`
- `invensense-imu` (MPU6500 library)

### Python Server (Optional)
```bash
pip install -r requirements.txt
python server.py
```

## Configuration

### WiFi Settings
```cpp
const char* ssid     = "your_wifi_ssid";
const char* password = "your_wifi_password";
```

### Server Configuration
```cpp
const char* serverBase = "http://your_server_ip:8000";
String deviceId = "DEV024";  // Unique device identifier
```

### MQTT Configuration
```cpp
const char* mqttServer = "your_mqtt_broker_ip";
const int   mqttPort   = 1883;
```

### Motion Detection Settings
```cpp
float accelThreshold = 1.5;  // m/s² threshold for motion detection
const unsigned long motionCooldown = 2000;  // 2 seconds between detections
const int motionRequiredCount = 2;  // Required detections within window
const unsigned long motionWindowTime = 500;  // 500ms detection window
```

## API Endpoints

### HTTP Endpoints
- `POST /api/alert` - Send alert with device status and GPS coordinates
- `GET /devices/{deviceId}/current` - Get current armed state

### MQTT Topics
- Subscribe: `motor/{deviceId}/cmd` - Receive commands

### MQTT Commands
Commands should be sent as JSON:
```json
{"command": "ARM"}
{"command": "DISARM"}
{"command": "BUZZ"}
{"command": "REQUEST_POSITION"}
```

## Alert Types

The system sends various alert types to the backend:
- `"Device Booted"` - System startup
- `"System Armed"` - Armed via MQTT command
- `"System Disarmed"` - Disarmed via MQTT command
- `"Gerakan Terdeteksi (Button)"` - Motion detected via button
- `"Gerakan Terdeteksi (MPU6500)"` - Motion detected via IMU
- `"BUZZ Executed"` - Remote buzzer activated
- `"Posisi Diminta"` - Position requested remotely
- `"Heartbeat"` - Periodic status update

## System Behavior

### Armed Mode
- LED indicator ON
- Motion detection active
- Triggers alarm and sends alert on motion
- Button press triggers alarm

### Disarmed Mode
- LED indicator OFF
- Motion detection active but ignored
- No alarms triggered
- Button press logged but ignored

### Offline Operation
- Queues up to 10 alerts when WiFi unavailable
- Automatically sends queued alerts when connection restored
- Oldest alerts dropped if queue is full

## Startup Sequence

1. Initialize hardware (button, buzzer, LED, I2C)
2. Initialize MPU6500 with 10Hz sampling rate
3. Play startup melody
4. Initialize GPS serial communication
5. Connect to WiFi
6. Sync armed state from server
7. Configure MQTT connection
8. Create background HTTP task
9. Send "Device Booted" alert

## Error Handling

### MPU6500 Recovery
- Tracks consecutive I2C read failures
- After 10 failures, attempts to reset I2C bus and reinitialize sensor
- Continues operation even if MPU6500 initialization fails

### MQTT Reconnection
- Non-blocking reconnection attempts every 5 seconds
- Automatically resubscribes to command topic on reconnection

### HTTP Retry Logic
- Failed HTTP requests added to offline queue
- Retries queued requests when WiFi restored

## Development

### Building with PlatformIO
```bash
pio run
```

### Uploading to ESP32
```bash
pio run --target upload
```

### Monitoring Serial Output
```bash
pio device monitor
```

## Project Structure

```
.
├── platformio.ini          # PlatformIO configuration
├── requirements.txt        # Python server dependencies
├── server.py              # Backend server (optional)
├── include/               # Header files
├── lib/                   # Libraries
│   └── invensense-imu-main/  # MPU6500 library
├── src/
│   └── main.cpp           # Main application code
└── test/                  # Test files
```

## Troubleshooting

### MPU6500 Not Initializing
- Check I2C connections (SDA=21, SCL=22)
- Verify MPU6500 address (0x68 default)
- Reduce I2C clock speed if needed (currently 50kHz)

### GPS Not Getting Fix
- Ensure GPS has clear view of sky
- Wait 1-2 minutes for initial fix
- Check GPS RX/TX connections

### WiFi Connection Issues
- Verify SSID and password
- Check WiFi signal strength
- Ensure 2.4GHz WiFi (ESP32 doesn't support 5GHz)

### MQTT Not Connecting
- Verify MQTT broker IP and port
- Check firewall rules
- Ensure broker is running

## License

[Add your license here]

## Author

Device ID: DEV024

## Version History

- **v1.0** - Initial release with full feature set
