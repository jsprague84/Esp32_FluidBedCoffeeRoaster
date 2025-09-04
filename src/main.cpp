// ESP32 Coffee Roaster - Streamlined MQTT-Only Version
// Core control functions with WiFi, OTA, and MQTT communication

#include <SPI.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PID_v1.h>
#include "MAX6675Handler.h"
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "driver/gpio.h"
#include "config.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

// Debugging Macros
#define DEBUG true
#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
  #define DEBUG_PRINTF_P(fmt, ...) Serial.printf_P(fmt, ##__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
  #define DEBUG_PRINTF_P(fmt, ...)
#endif

// Hardware Pin Definitions (now in config.h)
#define BT_CS   BT_CS_PIN
#define ET_CS   ET_CS_PIN

// Control Modes
#define MODE_MANUAL 0
#define MODE_AUTO 1

// System Status
typedef enum {
    SYSTEM_OK = 0,
    WIFI_ERROR = 1,
    MQTT_ERROR = 2,
    SENSOR_ERROR = 3,
    SAFETY_ERROR = 4
} SystemStatus;

SystemStatus systemStatus = SYSTEM_OK;
unsigned long lastStatusUpdate = 0;
const unsigned long statusUpdateInterval = 5000;

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Control Variables
double beanSetpoint = 0.0;
double beanTemperature = 0.0;
double heaterOutput = 0.0;
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;
float beanTempOffset = TEMP_CALIBRATION_BEAN;
float envTempOffset = TEMP_CALIBRATION_ENV;
PID beanPID(&beanTemperature, &heaterOutput, &beanSetpoint, Kp, Ki, Kd, DIRECT);
float envTemperature = 0.0;
int fanPWM = 0;
int controlMode = MODE_MANUAL;
bool heaterEnabled = false;

// Timing variables
unsigned long lastTempRead = 0;
unsigned long lastPidCompute = 0;
unsigned long lastSerialOutput = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastMqttReconnect = 0;

// Rate of Rise calculation
float tempHistory[RATE_HISTORY_SIZE];
unsigned long timeHistory[RATE_HISTORY_SIZE];
int historyIndex = 0;
int historyCount = 0;

MAX6675Handler beanThermocouple(BT_CS);
MAX6675Handler envThermocouple(ET_CS);

// Function prototypes
void initializePins();
void setupMQTT();
void connectMQTT();
void handleMQTTMessage(char* topic, byte* payload, unsigned int length);
void publishMQTTTelemetry();
void publishMQTTStatus(const String& status);
void updateRateOfRise(float currentTemp);
float getRateOfRise();
void applyCalibration();
void updatePIDParameters();
void savePIDParameters();
void loadPIDParameters();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void checkWiFiConnection();
void updateSystemStatus();
SystemStatus checkSensors();
SystemStatus checkConnectivity();
void handleControlSetpoint(const String& payload);
void handleControlFan(const String& payload);
void handleControlHeater(const String& payload);
void handleControlMode(const String& payload);
void handleControlEnable(const String& payload);
void handleControlPID(const String& payload);
void handleEmergencyStop(const String& payload);

void setup() {
    Serial.begin(115200);
    delay(1000);
    DEBUG_PRINTLN(F("Starting Coffee Roaster Control with MQTT..."));

    EEPROM.begin(EEPROM_SIZE);
    initializePins();

    // Initialize LEDC for PWM
    ledcSetup(0, 5000, 8);
    ledcAttachPin(SSR_PIN, 0);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(FAN_PIN, 1);
    gpio_set_drive_capability((gpio_num_t)FAN_PIN, GPIO_DRIVE_CAP_3);

    loadPIDParameters();
    
    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_TIMEOUT_MS) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("\nWiFi connected!"));
        Serial.printf_P(PSTR("IP Address: %s\n"), WiFi.localIP().toString().c_str());
        
        setupMQTT();
        delay(100);
        connectMQTT();
    } else {
        Serial.println(F("\nWiFi connection failed!"));
    }

    // Initialize thermocouples
    beanThermocouple.begin();
    envThermocouple.begin();
    DEBUG_PRINTLN(F("MAX6675 Initialized"));

    ArduinoOTA.begin();
    DEBUG_PRINTLN(F("OTA Ready"));

    // Initialize PID
    beanPID.SetMode(AUTOMATIC);
    beanPID.SetOutputLimits(0, 255);
    
    DEBUG_PRINTLN(F("Setup complete - All systems ready"));

    esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);
}

void loop() {
    esp_task_wdt_reset();
    
    checkWiFiConnection();
    updateSystemStatus();
    ArduinoOTA.handle();
    
    // MQTT handling
    if (!mqttClient.connected()) {
        static unsigned long lastReconnectAttempt = 0;
        unsigned long now = millis();
        if (now - lastReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
            lastReconnectAttempt = now;
            connectMQTT();
        }
    }
    mqttClient.loop();

    // Read Temperatures
    if (millis() - lastTempRead >= TEMP_READ_INTERVAL) {
        lastTempRead = millis();
        beanTemperature = beanThermocouple.readTemperature();
        envTemperature = envThermocouple.readTemperature();
        
        if (!isnan(beanTemperature) && !isnan(envTemperature)) {
            applyCalibration();
            updateRateOfRise(beanTemperature);
        }
    }

    // Control fan
    ledcWrite(1, fanPWM);

    // Heater Control Logic
    if (!heaterEnabled) {
        heaterOutput = 0;
        ledcWrite(0, 0);
    } else {
        if (controlMode == MODE_MANUAL) {
            // In manual mode, heaterOutput is set directly via MQTT
            if (fanPWM > SAFETY_MIN_FAN_PWM) {
                ledcWrite(0, static_cast<int>(heaterOutput));
            } else {
                ledcWrite(0, 0);
                DEBUG_PRINTLN(F("Failsafe: Fan too low, heater off."));
            }
        } else if (controlMode == MODE_AUTO) {
            if (!isnan(beanTemperature)) {
                if (millis() - lastPidCompute >= PID_COMPUTE_INTERVAL) {
                    lastPidCompute = millis();
                    beanPID.Compute();
                }
                if (fanPWM > SAFETY_MIN_FAN_PWM) {
                    ledcWrite(0, static_cast<int>(heaterOutput));
                } else {
                    ledcWrite(0, 0);
                    DEBUG_PRINTLN(F("Failsafe: Fan too low, heater off."));
                }
            } else {
                DEBUG_PRINTLN(F("Invalid bean temperature! Heater disabled."));
                ledcWrite(0, 0);
            }
        }
    }

    // MQTT Telemetry Publishing
    if (mqttClient.connected() && millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL) {
        lastMqttPublish = millis();
        publishMQTTTelemetry();
    }

    // Serial Output
    if (millis() - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL) {
        lastSerialOutput = millis();
        DEBUG_PRINTLN(F("System Status:"));
        DEBUG_PRINTF("Mode: %s, BT: %.2f°C, ET: %.2f°C, ROR: %.2f°C/min\n",
                     (controlMode == MODE_MANUAL) ? "Manual" : "Auto",
                     beanTemperature, envTemperature, getRateOfRise());
        DEBUG_PRINTF("Heater: %d, Fan: %d, Enabled: %d, MQTT: %s\n",
                     static_cast<int>(heaterOutput), fanPWM, heaterEnabled,
                     mqttClient.connected() ? "OK" : "DISCONNECTED");
    }
}

void setupMQTT() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setKeepAlive(15);
    mqttClient.setSocketTimeout(10);
    mqttClient.setBufferSize(MQTT_BUFFER_SIZE);

    DEBUG_PRINTF_P(PSTR("MQTT broker configured: %s:%d\n"), MQTT_BROKER, MQTT_PORT);
}

void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        DEBUG_PRINTLN(F("WiFi not connected - can't connect to MQTT"));
        return;
    }
    
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char clientId[50];
    snprintf(clientId, sizeof(clientId), "%s-%02X%02X%02X", MQTT_CLIENT_ID, mac[3], mac[4], mac[5]);
    
    DEBUG_PRINTF_P(PSTR("Attempting MQTT connection as %s..."), clientId);
    
    if (mqttClient.connect(
            clientId,
            MQTT_STATUS_TOPIC,
            1,
            true,
            "{\"status\":\"offline\"}"
        )) {
        DEBUG_PRINTLN(F(" connected!"));
    
        // Subscribe to all control topics
        String controlPattern = String(MQTT_CONTROL_TOPIC) + "/#";
        if (mqttClient.subscribe(controlPattern.c_str())) {
            DEBUG_PRINTLN(F("Subscribed to control topics"));

            // Publish "online" status
            StaticJsonDocument<200> doc;
            doc["status"] = "online";
            doc["id"] = clientId;
            doc["ip"] = WiFi.localIP().toString();
            doc["rssi"] = WiFi.RSSI();
            String statusMsg;
            serializeJson(doc, statusMsg);
            mqttClient.publish(MQTT_STATUS_TOPIC, statusMsg.c_str(), true);
        }   
    } else {
        int state = mqttClient.state();
        DEBUG_PRINTF(" failed, rc=%d\n", state);
        
        DEBUG_PRINTF("Network diagnostics:\n");
        DEBUG_PRINTF("- WiFi RSSI: %d dBm\n", WiFi.RSSI());
        DEBUG_PRINTF("- Local IP: %s\n", WiFi.localIP().toString().c_str());
        
        IPAddress brokerIP;
        if (WiFi.hostByName(MQTT_BROKER, brokerIP)) {
            DEBUG_PRINTF("- Broker IP resolved: %s\n", brokerIP.toString().c_str());
        } else {
            DEBUG_PRINTLN(F("- DNS resolution failed!"));
        }
    }
}

void handleMQTTMessage(char* topic, byte* payload, unsigned int length) {
    String topicStr = String(topic);
    String payloadStr;
    payloadStr.reserve(length);
    for (unsigned int i = 0; i < length; i++) {
        payloadStr += (char)payload[i];
    }

    DEBUG_PRINTF_P(PSTR("MQTT RX: %s = %s\n"), topic, payloadStr.c_str());
    
    String controlTopicBase = String(MQTT_CONTROL_TOPIC);
    
    if (topicStr.startsWith(controlTopicBase)) {
        String command = topicStr.substring(controlTopicBase.length());
        if (command.startsWith("/")) command.remove(0, 1);
        
        // Dispatch to individual handlers
        if (command == "setpoint") {
            handleControlSetpoint(payloadStr);
        }
        else if (command == "fan_pwm") {
            handleControlFan(payloadStr);
        }
        else if (command == "heater_pwm") {
            handleControlHeater(payloadStr);
        }
        else if (command == "mode") {
            handleControlMode(payloadStr);
        }
        else if (command == "heater_enable") {
            handleControlEnable(payloadStr);
        }
        else if (command == "pid") {
            handleControlPID(payloadStr);
        }
        else if (command == "emergency_stop") {
            handleEmergencyStop(payloadStr);
        }
    }
}

void publishMQTTTelemetry() {
    DynamicJsonDocument doc(512);
    
    doc["timestamp"] = millis();
    doc["beanTemp"] = round(beanTemperature * 10) / 10.0;
    doc["envTemp"] = round(envTemperature * 10) / 10.0;
    doc["rateOfRise"] = round(getRateOfRise() * 100) / 100.0;
    doc["heaterPWM"] = static_cast<int>(heaterOutput * 100 / 255);  // Convert to percentage like original
    doc["fanPWM"] = fanPWM;
    doc["setpoint"] = round(beanSetpoint * 10) / 10.0;
    doc["controlMode"] = controlMode;
    doc["heaterEnable"] = heaterEnabled ? 1 : 0;  // Match original field name
    doc["uptime"] = millis() / 1000;
    doc["Kp"] = Kp;
    doc["Ki"] = Ki;  
    doc["Kd"] = Kd;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["rssi"] = WiFi.RSSI();
    doc["systemStatus"] = systemStatus;
    
    String payload;
    serializeJson(doc, payload);
    
    mqttClient.publish(MQTT_TELEMETRY_TOPIC, payload.c_str());
    DEBUG_PRINTF("MQTT: Published telemetry (%d bytes)\n", payload.length());
}

void publishMQTTStatus(const String& status) {
    DynamicJsonDocument doc(256);
    doc["status"] = status;
    doc["timestamp"] = millis();
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["version"] = "2.0.0-mqtt-only";
    
    String payload;
    serializeJson(doc, payload);
    
    mqttClient.publish(MQTT_STATUS_TOPIC, payload.c_str(), true);
}

void updateRateOfRise(float currentTemp) {
    tempHistory[historyIndex] = currentTemp;
    timeHistory[historyIndex] = millis();
    historyIndex = (historyIndex + 1) % RATE_HISTORY_SIZE;
    if (historyCount < RATE_HISTORY_SIZE) historyCount++;
}

float getRateOfRise() {
    if (historyCount < 3) return 0.0;
    
    int startIdx = (historyIndex - historyCount + RATE_HISTORY_SIZE) % RATE_HISTORY_SIZE;
    int endIdx = (historyIndex - 1 + RATE_HISTORY_SIZE) % RATE_HISTORY_SIZE;
    
    float tempDiff = tempHistory[endIdx] - tempHistory[startIdx];
    float timeDiff = (timeHistory[endIdx] - timeHistory[startIdx]) / 1000.0;
    
    if (timeDiff > 0) {
        return (tempDiff / timeDiff) * 60.0;
    }
    return 0.0;
}

void initializePins() {
    pinMode(SSR_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
}

void applyCalibration() {
    beanTemperature += beanTempOffset;
    envTemperature += envTempOffset;
}

void updatePIDParameters() {
    // This function is kept for compatibility but PID params are now updated via MQTT
}

void savePIDParameters() {
    EEPROM.put(0, Kp);
    EEPROM.put(8, Ki);
    EEPROM.put(16, Kd);
    EEPROM.commit();
    DEBUG_PRINTLN(F("PID Parameters Saved to EEPROM"));
}

void loadPIDParameters() {
    // Read PID parameters from EEPROM
    double tempKp, tempKi, tempKd;
    EEPROM.get(0, tempKp);
    EEPROM.get(8, tempKi);
    EEPROM.get(16, tempKd);
    
    // Validate EEPROM data (check for reasonable PID values)
    bool validData = true;
    if (isnan(tempKp) || tempKp <= 0 || tempKp > 1000) validData = false;
    if (isnan(tempKi) || tempKi < 0 || tempKi > 100) validData = false;
    if (isnan(tempKd) || tempKd < 0 || tempKd > 1000) validData = false;
    
    if (validData) {
        // Use EEPROM values
        Kp = tempKp;
        Ki = tempKi;
        Kd = tempKd;
        DEBUG_PRINTLN(F("PID Parameters Loaded from EEPROM"));
        DEBUG_PRINTF("Loaded PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
    } else {
        // Use default values and save them to EEPROM
        Kp = DEFAULT_KP;
        Ki = DEFAULT_KI;
        Kd = DEFAULT_KD;
        savePIDParameters();
        DEBUG_PRINTLN(F("EEPROM invalid - using default PID parameters"));
        DEBUG_PRINTF("Default PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
    }
    
    // Apply the PID parameters to the controller
    beanPID.SetTunings(Kp, Ki, Kd);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    handleMQTTMessage(topic, payload, length);
}

void checkWiFiConnection() {
    static unsigned long lastWiFiCheck = 0;
    const unsigned long wifiCheckInterval = WIFI_CHECK_INTERVAL;
    
    if (millis() - lastWiFiCheck >= wifiCheckInterval) {
        lastWiFiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            DEBUG_PRINTLN(F("WiFi disconnected. Reconnecting..."));
            WiFi.disconnect();
            WiFi.begin(ssid, password);
        }
    }
}

void updateSystemStatus() {
    if (millis() - lastStatusUpdate >= statusUpdateInterval) {
        lastStatusUpdate = millis();
        
        SystemStatus oldStatus = systemStatus;
        systemStatus = SYSTEM_OK;
        
        // Check connectivity
        SystemStatus connStatus = checkConnectivity();
        if (connStatus != SYSTEM_OK) systemStatus = connStatus;
        
        // Check sensors
        SystemStatus sensorStatus = checkSensors();
        if (sensorStatus != SYSTEM_OK) systemStatus = sensorStatus;
        
        // Log status changes
        if (systemStatus != oldStatus) {
            DEBUG_PRINTF_P(PSTR("System status changed: %d -> %d\n"), oldStatus, systemStatus);
        }
    }
}

SystemStatus checkConnectivity() {
    if (WiFi.status() != WL_CONNECTED) {
        return WIFI_ERROR;
    }
    if (!mqttClient.connected()) {
        return MQTT_ERROR;
    }
    return SYSTEM_OK;
}

SystemStatus checkSensors() {
    if (isnan(beanTemperature) || isnan(envTemperature)) {
        return SENSOR_ERROR;
    }
    if (beanTemperature < -50 || beanTemperature > 300) {
        return SENSOR_ERROR;
    }
    if (envTemperature < -50 || envTemperature > 300) {
        return SENSOR_ERROR;
    }
    return SYSTEM_OK;
}

void handleControlSetpoint(const String& payload) {
    float newSetpoint = payload.toFloat();
    if (newSetpoint >= MIN_BEAN_TEMP && newSetpoint <= MAX_BEAN_TEMP) {
        beanSetpoint = newSetpoint;
        DEBUG_PRINTF_P(PSTR("MQTT: Setpoint set to %.1f°C\n"), newSetpoint);
    }
}

void handleControlFan(const String& payload) {
    int newFanPWM = payload.toInt();
    if (newFanPWM >= 0 && newFanPWM <= 255) {
        fanPWM = newFanPWM;
        DEBUG_PRINTF_P(PSTR("MQTT: Fan PWM set to %d\n"), newFanPWM);
    }
}

void handleControlHeater(const String& payload) {
    int newHeaterPWM = payload.toInt();
    if (newHeaterPWM >= MIN_HEATER_PWM && newHeaterPWM <= MAX_HEATER_PWM) {
        heaterOutput = newHeaterPWM * 255 / 100;  // Convert percentage to 0-255
        DEBUG_PRINTF_P(PSTR("MQTT: Heater PWM set to %d%%\n"), newHeaterPWM);
    }
}

void handleControlMode(const String& payload) {
    if (payload == "manual" || payload == "0") {
        controlMode = MODE_MANUAL;
        DEBUG_PRINTLN(F("MQTT: Mode set to Manual"));
    } else if (payload == "auto" || payload == "1") {
        controlMode = MODE_AUTO;
        DEBUG_PRINTLN(F("MQTT: Mode set to Auto"));
    }
}

void handleControlEnable(const String& payload) {
    heaterEnabled = (payload == "1" || payload == "true");
    DEBUG_PRINTF_P(PSTR("MQTT: Heater enable set to %d\n"), heaterEnabled ? 1 : 0);
}

void handleControlPID(const String& payload) {
    // Expect JSON: {"kp": 15.0, "ki": 1.0, "kd": 25.0}
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
        if (doc.containsKey("kp")) Kp = doc["kp"];
        if (doc.containsKey("ki")) Ki = doc["ki"];
        if (doc.containsKey("kd")) Kd = doc["kd"];
        beanPID.SetTunings(Kp, Ki, Kd);
        savePIDParameters();
        DEBUG_PRINTF_P(PSTR("MQTT: PID updated Kp=%.2f, Ki=%.2f, Kd=%.2f\n"), Kp, Ki, Kd);
    }
}

void handleEmergencyStop(const String& payload) {
    if (payload == "1" || payload == "true") {
        DEBUG_PRINTLN(F("MQTT: EMERGENCY STOP RECEIVED!"));
        heaterEnabled = false;
        fanPWM = 255;
        heaterOutput = 0;
        ledcWrite(0, 0);
        ledcWrite(1, 255);
    }
}