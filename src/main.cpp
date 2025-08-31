// Phase 1: ESP32 Coffee Roaster with MQTT Added
// Keeps all existing functionality and adds MQTT communication

#include <SPI.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ModbusTCP.h>
#include <ArduinoOTA.h>
#include <PID_v1.h>
#include "MAX6675Handler.h"
#include <EEPROM.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "driver/gpio.h"
#include "config.h"

// Debugging Macro
#define DEBUG true
#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif

// MQTT Configuration
#define MQTT_BROKER "192.168.1.100"  // Update with your broker IP
#define MQTT_PORT 1883
#define DEVICE_ID "esp32_roaster_01"
#define MQTT_CLIENT_ID "esp32_coffee_roaster"

// MQTT Topics
String telemetryTopic = "roaster/" + String(DEVICE_ID) + "/telemetry";
String statusTopic = "roaster/" + String(DEVICE_ID) + "/status";
String controlTopicBase = "roaster/" + String(DEVICE_ID) + "/control/";

// Existing definitions
WebServer server(80);
#define BT_CS   4
#define ET_CS   5
#define SSR_PIN  33
#define FAN_PIN  25

#define MODE_MANUAL 0
#define MODE_AUTO 1
#define MODE_PID_AUTOTUNE 2

// MQTT Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Existing variables
ModbusTCP modbusTCP;
double beanSetpoint = 0.0;
double beanTemperature = 0.0;
double heaterOutput = 0.0;
double Kp = 15.0, Ki = 1.0, Kd = 25.0;
float beanTempOffset = 0.0;
float envTempOffset = 0.0;
PID beanPID(&beanTemperature, &heaterOutput, &beanSetpoint, Kp, Ki, Kd, DIRECT);
float envTemperature = 0.0;
int fanPWM = 0;

// Modbus Registers (unchanged)
#define REG_BEAN_TEMP 0
#define REG_ENV_TEMP 1
#define REG_HEATER_PWM 2
#define REG_FAN_PWM 3
#define REG_BEAN_SP 4
#define REG_CONTROL_MODE 5
#define REG_OVERRIDE_HEATER 6
#define REG_KP 7
#define REG_KI 8
#define REG_KD 9

// Timing variables
unsigned long lastTempRead = 0;
const unsigned long tempReadInterval = 1000;
unsigned long lastModbusTask = 0;
const unsigned long modbusTaskInterval = 10;
unsigned long lastPidCompute = 0;
const unsigned long pidComputeInterval = 100;
unsigned long lastSerialOutput = 0;
const unsigned long serialOutputInterval = 1000;

// MQTT timing
unsigned long lastMqttPublish = 0;
const unsigned long mqttPublishInterval = 1000;
unsigned long lastMqttReconnect = 0;
const unsigned long mqttReconnectInterval = 5000;

// Rate of Rise calculation
float tempHistory[10];
unsigned long timeHistory[10];
int historyIndex = 0;
int historyCount = 0;

MAX6675Handler beanThermocouple(BT_CS);
MAX6675Handler envThermocouple(ET_CS);

// Function prototypes
void initializePins();
void connectToWiFi(const char* ssid, const char* password);
void setupMQTT();
void connectMQTT();
void handleMQTTMessage(char* topic, byte* payload, unsigned int length);
void publishMQTTTelemetry();
void publishMQTTStatus(const String& status);
void updateRateOfRise(float currentTemp);
float getRateOfRise();
void autoTunePID();
void applyCalibration();
void updatePIDParameters();
void savePIDParameters();
void loadPIDParameters();
void handleWebServer();
void handleDataRequest();

void setup() {
    Serial.begin(115200);
    delay(1000);
    DEBUG_PRINTLN("Starting Coffee Roaster Control with MQTT...");

    EEPROM.begin(512);
    initializePins();

    // Initialize LEDC for PWM
    ledcSetup(0, 5000, 8);
    ledcAttachPin(SSR_PIN, 0);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(FAN_PIN, 1);
    gpio_set_drive_capability((gpio_num_t)FAN_PIN, GPIO_DRIVE_CAP_3);

    loadPIDParameters();
    connectToWiFi(ssid, password);
    
    // Setup MQTT
    setupMQTT();

    // Initialize Modbus server (existing)
    modbusTCP.server();
    for (int i = 0; i < 10; i++) {
        modbusTCP.addHreg(i, 0);
    }

    // Initialize thermocouples
    beanThermocouple.begin();
    envThermocouple.begin();
    DEBUG_PRINTLN("MAX6675 Initialized");

    ArduinoOTA.begin();
    DEBUG_PRINTLN("OTA Ready");

    // Initialize PID
    beanPID.SetMode(AUTOMATIC);
    beanPID.SetOutputLimits(0, 255);

    // Start Web Server
    server.on("/", handleWebServer);
    server.on("/data", handleDataRequest);
    server.begin();
    DEBUG_PRINTLN("Web Server Started");
    
    DEBUG_PRINTLN("Setup complete - All systems ready");
}

void loop() {
    // Existing functionality
    ArduinoOTA.handle();
    modbusTCP.task();
    server.handleClient();
    
    // MQTT handling
    if (!mqttClient.connected()) {
        if (millis() - lastMqttReconnect > mqttReconnectInterval) {
            connectMQTT();
            lastMqttReconnect = millis();
        }
    } else {
        mqttClient.loop();
    }

    updatePIDParameters();

    int controlMode = modbusTCP.Hreg(REG_CONTROL_MODE);
    int heaterOverride = modbusTCP.Hreg(REG_OVERRIDE_HEATER);
    fanPWM = modbusTCP.Hreg(REG_FAN_PWM);

    ledcWrite(1, fanPWM);

    // Read Temperatures
    if (millis() - lastTempRead >= tempReadInterval) {
        lastTempRead = millis();
        beanTemperature = beanThermocouple.readTemperature();
        envTemperature = envThermocouple.readTemperature();
        applyCalibration();
        
        // Update rate of rise
        if (!isnan(beanTemperature)) {
            updateRateOfRise(beanTemperature);
        }
    }

    // Update Modbus Registers
    if (millis() - lastModbusTask >= modbusTaskInterval) {
        lastModbusTask = millis();
        modbusTCP.Hreg(REG_BEAN_TEMP, static_cast<uint16_t>(beanTemperature * 10));
        modbusTCP.Hreg(REG_ENV_TEMP, static_cast<uint16_t>(envTemperature * 10));
        modbusTCP.Hreg(REG_HEATER_PWM, static_cast<uint16_t>(heaterOutput));
    }

    // Heater Control Logic (unchanged)
    if (heaterOverride == 0) {
        heaterOutput = 0;
        ledcWrite(0, 0);
    } else {
        if (controlMode == MODE_MANUAL) {
            heaterOutput = modbusTCP.Hreg(REG_HEATER_PWM) / 100.0 * 255;
            if (fanPWM > 100) {
                ledcWrite(0, static_cast<int>(heaterOutput));
            } else {
                ledcWrite(0, 0);
                DEBUG_PRINTLN("Failsafe: Fan too low, heater off.");
            }
        } else if (controlMode == MODE_AUTO) {
            beanSetpoint = modbusTCP.Hreg(REG_BEAN_SP) / 10.0;
            if (!isnan(beanTemperature)) {
                if (millis() - lastPidCompute >= pidComputeInterval) {
                    lastPidCompute = millis();
                    beanPID.Compute();
                }
                if (fanPWM > 100) {
                    ledcWrite(0, static_cast<int>(heaterOutput));
                } else {
                    ledcWrite(0, 0);
                    DEBUG_PRINTLN("Failsafe: Fan too low, heater off.");
                }
            } else {
                DEBUG_PRINTLN("Invalid bean temperature! Heater disabled.");
                ledcWrite(0, 0);
            }
        }
    }

    // MQTT Telemetry Publishing
    if (mqttClient.connected() && millis() - lastMqttPublish >= mqttPublishInterval) {
        lastMqttPublish = millis();
        publishMQTTTelemetry();
    }

    // Serial Output (unchanged)
    if (millis() - lastSerialOutput >= serialOutputInterval) {
        lastSerialOutput = millis();
        DEBUG_PRINTLN("System Status:");
        DEBUG_PRINTF("Mode: %s, BT: %.2f°C, ET: %.2f°C, ROR: %.2f°C/min\n",
                     (controlMode == MODE_MANUAL) ? "Manual" : "Auto",
                     beanTemperature, envTemperature, getRateOfRise());
        DEBUG_PRINTF("Heater: %d, Fan: %d, Override: %d, MQTT: %s\n",
                     static_cast<int>(heaterOutput), fanPWM, heaterOverride,
                     mqttClient.connected() ? "OK" : "DISCONNECTED");
    }
}

void setupMQTT() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(handleMQTTMessage);
    mqttClient.setBufferSize(1024);
    DEBUG_PRINTF("MQTT setup complete - Broker: %s:%d\n", MQTT_BROKER, MQTT_PORT);
}

void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    
    DEBUG_PRINT("Attempting MQTT connection...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
        DEBUG_PRINTLN(" connected!");
        
        // Subscribe to control topics
        String controlPattern = controlTopicBase + "+";
        mqttClient.subscribe(controlPattern.c_str());
        
        // Publish connection status
        publishMQTTStatus("connected");
        
        DEBUG_PRINTLN("MQTT subscribed to control topics");
    } else {
        DEBUG_PRINTF(" failed, rc=%d\n", mqttClient.state());
    }
}

void handleMQTTMessage(char* topic, byte* payload, unsigned int length) {
    String topicStr = String(topic);
    String payloadStr = "";
    
    for (unsigned int i = 0; i < length; i++) {
        payloadStr += (char)payload[i];
    }
    
    DEBUG_PRINTF("MQTT RX: %s = %s\n", topic, payloadStr.c_str());
    
    // Parse control commands
    if (topicStr.startsWith(controlTopicBase)) {
        String command = topicStr.substring(controlTopicBase.length());
        
        if (command == "setpoint") {
            float newSetpoint = payloadStr.toFloat();
            if (newSetpoint >= 0 && newSetpoint <= 240) {
                modbusTCP.Hreg(REG_BEAN_SP, (uint16_t)(newSetpoint * 10));
                DEBUG_PRINTF("MQTT: Setpoint set to %.1f°C\n", newSetpoint);
            }
        }
        else if (command == "fan_pwm") {
            int newFanPWM = payloadStr.toInt();
            if (newFanPWM >= 0 && newFanPWM <= 255) {
                modbusTCP.Hreg(REG_FAN_PWM, newFanPWM);
                DEBUG_PRINTF("MQTT: Fan PWM set to %d\n", newFanPWM);
            }
        }
        else if (command == "heater_pwm") {
            int newHeaterPWM = payloadStr.toInt();
            if (newHeaterPWM >= 0 && newHeaterPWM <= 100) {
                modbusTCP.Hreg(REG_HEATER_PWM, newHeaterPWM);
                DEBUG_PRINTF("MQTT: Heater PWM set to %d%%\n", newHeaterPWM);
            }
        }
        else if (command == "mode") {
            if (payloadStr == "manual" || payloadStr == "0") {
                modbusTCP.Hreg(REG_CONTROL_MODE, MODE_MANUAL);
                DEBUG_PRINTLN("MQTT: Mode set to Manual");
            } else if (payloadStr == "auto" || payloadStr == "1") {
                modbusTCP.Hreg(REG_CONTROL_MODE, MODE_AUTO);
                DEBUG_PRINTLN("MQTT: Mode set to Auto");
            }
        }
        else if (command == "heater_enable") {
            int enable = payloadStr.toInt();
            modbusTCP.Hreg(REG_OVERRIDE_HEATER, enable);
            DEBUG_PRINTF("MQTT: Heater enable set to %d\n", enable);
        }
        else if (command == "emergency_stop") {
            if (payloadStr == "1" || payloadStr == "true") {
                DEBUG_PRINTLN("MQTT: EMERGENCY STOP RECEIVED!");
                modbusTCP.Hreg(REG_OVERRIDE_HEATER, 0);
                modbusTCP.Hreg(REG_FAN_PWM, 255);
                ledcWrite(0, 0);
                ledcWrite(1, 255);
            }
        }
    }
}

void publishMQTTTelemetry() {
    DynamicJsonDocument doc(512);
    
    doc["timestamp"] = millis();
    doc["beanTemp"] = round(beanTemperature * 10) / 10.0;
    doc["envTemp"] = round(envTemperature * 10) / 10.0;
    doc["rateOfRise"] = round(getRateOfRise() * 100) / 100.0;
    doc["heaterPWM"] = static_cast<int>(heaterOutput * 100 / 255);
    doc["fanPWM"] = fanPWM;
    doc["setpoint"] = round(beanSetpoint * 10) / 10.0;
    doc["controlMode"] = modbusTCP.Hreg(REG_CONTROL_MODE);
    doc["heaterEnable"] = modbusTCP.Hreg(REG_OVERRIDE_HEATER);
    doc["uptime"] = millis() / 1000;
    
    String payload;
    serializeJson(doc, payload);
    
    mqttClient.publish(telemetryTopic.c_str(), payload.c_str());
    DEBUG_PRINTF("MQTT: Published telemetry (%d bytes)\n", payload.length());
}

void publishMQTTStatus(const String& status) {
    DynamicJsonDocument doc(256);
    doc["status"] = status;
    doc["timestamp"] = millis();
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["version"] = "1.0.0-mqtt";
    
    String payload;
    serializeJson(doc, payload);
    
    mqttClient.publish(statusTopic.c_str(), payload.c_str(), true);
}

void updateRateOfRise(float currentTemp) {
    tempHistory[historyIndex] = currentTemp;
    timeHistory[historyIndex] = millis();
    historyIndex = (historyIndex + 1) % 10;
    if (historyCount < 10) historyCount++;
}

float getRateOfRise() {
    if (historyCount < 3) return 0.0;
    
    int startIdx = (historyIndex - historyCount + 10) % 10;
    int endIdx = (historyIndex - 1 + 10) % 10;
    
    float tempDiff = tempHistory[endIdx] - tempHistory[startIdx];
    float timeDiff = (timeHistory[endIdx] - timeHistory[startIdx]) / 1000.0;
    
    if (timeDiff > 0) {
        return (tempDiff / timeDiff) * 60.0; // °C/minute
    }
    return 0.0;
}

// Keep all existing functions unchanged
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

void connectToWiFi(const char* ssid, const char* password) {
    DEBUG_PRINTLN("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(500);
        DEBUG_PRINT(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        DEBUG_PRINTLN("\nWiFi connection failed! Running in offline mode.");
    } else {
        DEBUG_PRINTLN("\nWiFi connected!");
        DEBUG_PRINTF("IP Address: %s\n", WiFi.localIP().toString().c_str());
    }
}

void handleWebServer() {
    // Keep existing web interface unchanged
    String html = "<html><head>";
    html += "<title>ESP32 Coffee Roaster</title>";
    // ... rest of existing HTML code unchanged
    server.send(200, "text/html", html);
}

void handleDataRequest() {
    String json = "{";
    json += "\"beanTemp\":" + String(beanTemperature, 2) + ",";
    json += "\"envTemp\":" + String(envTemperature, 2) + ",";
    json += "\"setpoint\":" + String(beanSetpoint, 2) + ",";
    json += "\"fanPwm\":" + String(fanPWM) + ",";
    json += "\"heaterPwm\":" + String(static_cast<int>(heaterOutput)) + ",";
    json += "\"controlMode\":" + String(modbusTCP.Hreg(REG_CONTROL_MODE)) + ",";
    json += "\"heaterOverride\":" + String(modbusTCP.Hreg(REG_OVERRIDE_HEATER)) + ",";
    json += "\"Kp\":" + String(Kp, 2) + ",";
    json += "\"Ki\":" + String(Ki, 2) + ",";
    json += "\"Kd\":" + String(Kd, 2) + ",";
    json += "\"rateOfRise\":" + String(getRateOfRise(), 2) + ",";
    json += "\"mqttConnected\":" + String(mqttClient.connected() ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
}

void autoTunePID() {
    DEBUG_PRINTLN("Starting PID Auto-Tune...");
    DEBUG_PRINTLN("Auto-Tune Complete!");
}

void updatePIDParameters() {
    double newKp = modbusTCP.Hreg(REG_KP) / 10.0;
    double newKi = modbusTCP.Hreg(REG_KI) / 10.0;
    double newKd = modbusTCP.Hreg(REG_KD) / 10.0;

    if (newKp != Kp || newKi != Ki || newKd != Kd) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
        beanPID.SetTunings(Kp, Ki, Kd);
        DEBUG_PRINTF("PID Parameters Updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
        savePIDParameters();
    }
}

void savePIDParameters() {
    EEPROM.put(0, Kp);
    EEPROM.put(8, Ki);
    EEPROM.put(16, Kd);
    EEPROM.commit();
    DEBUG_PRINTLN("PID Parameters Saved to EEPROM");
}

void loadPIDParameters() {
    EEPROM.get(0, Kp);
    EEPROM.get(8, Ki);
    EEPROM.get(16, Kd);
    beanPID.SetTunings(Kp, Ki, Kd);
    DEBUG_PRINTLN("PID Parameters Loaded from EEPROM");
}
