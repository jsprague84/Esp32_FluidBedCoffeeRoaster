// filepath: //esp32CoffeeRoaster/include/config.example.h
#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";

// MQTT Configuration
#define MQTT_BROKER "YourMQTTBrokerAddress"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32_roaster_01"
#define MQTT_CLEAN_SESSION true

// Topic structure
#define MQTT_BASE_TOPIC "roaster/"
#define MQTT_STATUS_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/status"
#define MQTT_TELEMETRY_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/telemetry"
#define MQTT_CONTROL_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/control"

// Add debug topics
#define MQTT_DEBUG_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/debug"

// Hardware Pin Configuration
#define BT_CS_PIN   4
#define ET_CS_PIN   5
#define SSR_PIN     33
#define FAN_PIN     25

// Safety Parameters
#define SAFETY_MIN_FAN_PWM 100
#define MAX_BEAN_TEMP 240.0
#define MIN_BEAN_TEMP 0.0
#define MAX_HEATER_PWM 100
#define MIN_HEATER_PWM 0

// Temperature Calibration Offsets
#define TEMP_CALIBRATION_BEAN 0.0
#define TEMP_CALIBRATION_ENV 0.0

// Timing Configuration (milliseconds)
#define TEMP_READ_INTERVAL 1000
#define PID_COMPUTE_INTERVAL 100
#define SERIAL_OUTPUT_INTERVAL 1000
#define MQTT_PUBLISH_INTERVAL 1000
#define MQTT_RECONNECT_INTERVAL 5000
#define WIFI_CHECK_INTERVAL 30000

// System Configuration
#define WATCHDOG_TIMEOUT_SEC 30
#define WIFI_TIMEOUT_MS 20000
#define MQTT_BUFFER_SIZE 512
#define EEPROM_SIZE 512
#define RATE_HISTORY_SIZE 10

// PID Default Values
#define DEFAULT_KP 15.0
#define DEFAULT_KI 1.0
#define DEFAULT_KD 25.0

#endif