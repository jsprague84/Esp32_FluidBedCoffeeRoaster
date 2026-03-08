// ESP32 Coffee Roaster - Streamlined MQTT-Only Version
// Core control functions with WiFi, OTA, and MQTT communication

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "driver/gpio.h"
#include "config.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "debug.h"
#include "roaster_state.h"
#include "temperature.h"
#include "heater_control.h"

// WiFi credentials (declared extern in config.h)
const char* ssid = "jswifi";
const char* password = "helloworld";

// Global roaster state
RoasterState state = {};

unsigned long lastStatusUpdate = 0;
const unsigned long statusUpdateInterval = 5000;

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Timing variables
unsigned long lastSerialOutput = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastMqttReconnect = 0;

// Auto-tune state variables
typedef enum {
    AUTOTUNE_IDLE = 0,
    AUTOTUNE_HEATING = 1,        // Heating to target temperature
    AUTOTUNE_STABILIZING = 2,    // Stabilizing at target temperature
    AUTOTUNE_RUNNING = 3,        // Running oscillation tests
    AUTOTUNE_ANALYZING = 4,      // Analyzing collected data
    AUTOTUNE_COMPLETE = 5,       // Analysis complete
    AUTOTUNE_FAILED = 6
} AutoTuneState;

typedef struct {
    float time;
    float temperature;
} PeakValley;

AutoTuneState autoTuneState = AUTOTUNE_IDLE;
double autoTuneTargetTemp = 200.0;
double autoTuneSetpoint = 200.0;
unsigned long autoTuneStartTime = 0;
unsigned long autoTuneStepStartTime = 0;
unsigned long autoTuneStabilizationStartTime = 0;
int autoTuneCurrentStep = 0;
float autoTuneOutputBias = AUTOTUNE_OUTPUT_BIAS;
float autoTuneOutputAmplitude = AUTOTUNE_OUTPUT_AMPLITUDE;
double autoTuneOriginalKp = 0.0, autoTuneOriginalKi = 0.0, autoTuneOriginalKd = 0.0;
double autoTuneRecommendedKp = 0.0, autoTuneRecommendedKi = 0.0, autoTuneRecommendedKd = 0.0;
bool autoTuneUsedFallback = false;
bool autoTuneReachedSetpoint = false;
float autoTuneStepTimeout = AUTOTUNE_MAX_STEP_TIME;  // Configurable per step
unsigned long autoTuneRorStableStartTime = 0;
unsigned long autoTuneFastTrackStartTime = 0;
bool autoTuneOutputHigh = false;

// Auto-tune data storage
PeakValley autoTunePeaks[10];
PeakValley autoTuneValleys[10];
int autoTunePeakCount = 0;
int autoTuneValleyCount = 0;

// Auto-tune temperature history for oscillation detection
float autoTuneTempHistory[20];
unsigned long autoTuneTempTimeHistory[20];
int autoTuneTempHistoryIndex = 0;
int autoTuneTempHistoryCount = 0;

unsigned long lastAutoTuneStatusPublish = 0;
const unsigned long autoTuneStatusPublishInterval = 2000; // Publish status every 2 seconds

// Function prototypes
void initializePins();
void initStateDefaults();
void setupMQTT();
void connectMQTT();
void handleMQTTMessage(char* topic, byte* payload, unsigned int length);
void publishMQTTTelemetry();
void publishMQTTStatus(const String& status);
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

// Auto-tune function prototypes
void handleAutoTuneStart(const String& payload);
void handleAutoTuneStop(const String& payload);
void handleAutoTuneApply(const String& payload);
void updateAutoTune();
void publishAutoTuneStatus();
void publishAutoTuneResults();
bool checkAutoTuneCrossedSetpoint();
bool calculateAutoTunePIDParameters();
void resetAutoTuneData();
const char* getAutoTuneStateString(AutoTuneState atState);

void initStateDefaults() {
    state.beanSetpoint = 0.0;
    state.beanTemperature = 0.0;
    state.heaterOutput = 0.0;
    state.envTemperature = 0.0;
    state.fanPWM = 0;
    state.prevFanPWM = -1;
    state.controlMode = MODE_MANUAL;
    state.heaterEnabled = false;
    state.Kp = DEFAULT_KP;
    state.Ki = DEFAULT_KI;
    state.Kd = DEFAULT_KD;
    state.beanTempOffset = TEMP_CALIBRATION_BEAN;
    state.envTempOffset = TEMP_CALIBRATION_ENV;
    state.systemStatus = SYSTEM_OK;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    DEBUG_PRINTLN(F("Starting Coffee Roaster Control with MQTT..."));

    initStateDefaults();
    initializePins();
    initHeaterControl();

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

    initTemperature();

    // Configure OTA
    ArduinoOTA.setHostname(MQTT_CLIENT_ID);
    ArduinoOTA.setPassword("roaster123");  // Set OTA password

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_SPIFFS
            type = "filesystem";
        }
        DEBUG_PRINTLN("Start updating " + type);
    });

    ArduinoOTA.onEnd([]() {
        DEBUG_PRINTLN(F("\nEnd"));
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        DEBUG_PRINTF("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            DEBUG_PRINTLN(F("Auth Failed"));
        } else if (error == OTA_BEGIN_ERROR) {
            DEBUG_PRINTLN(F("Begin Failed"));
        } else if (error == OTA_CONNECT_ERROR) {
            DEBUG_PRINTLN(F("Connect Failed"));
        } else if (error == OTA_RECEIVE_ERROR) {
            DEBUG_PRINTLN(F("Receive Failed"));
        } else if (error == OTA_END_ERROR) {
            DEBUG_PRINTLN(F("End Failed"));
        }
    });

    ArduinoOTA.begin();
    DEBUG_PRINTLN(F("OTA Ready"));

    DEBUG_PRINTLN(F("Setup complete - All systems ready"));

    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
        .idle_core_mask = (1 << 0),
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
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
    readTemperatures();

    // Heater and fan control
    updateHeaterControl();

    // Auto-tune update: run state machine in all active phases
    if (autoTuneState == AUTOTUNE_HEATING ||
        autoTuneState == AUTOTUNE_STABILIZING ||
        autoTuneState == AUTOTUNE_RUNNING ||
        autoTuneState == AUTOTUNE_ANALYZING) {
        updateAutoTune();
    }

    // Auto-tune status publishing
    if (mqttClient.connected() && millis() - lastAutoTuneStatusPublish >= autoTuneStatusPublishInterval) {
        lastAutoTuneStatusPublish = millis();
        if (autoTuneState != AUTOTUNE_IDLE) {
            publishAutoTuneStatus();
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
                     (state.controlMode == MODE_MANUAL) ? "Manual" : "Auto",
                     state.beanTemperature, state.envTemperature, getRateOfRise());
        DEBUG_PRINTF("Heater: %d, Fan: %d, Enabled: %d, MQTT: %s\n",
                     static_cast<int>(state.heaterOutput), state.fanPWM, state.heaterEnabled,
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
        }

        // Subscribe to auto-tune topics
        if (mqttClient.subscribe(MQTT_AUTOTUNE_START_TOPIC)) {
            DEBUG_PRINTLN(F("Subscribed to auto-tune start topic"));
        }
        if (mqttClient.subscribe(MQTT_AUTOTUNE_STOP_TOPIC)) {
            DEBUG_PRINTLN(F("Subscribed to auto-tune stop topic"));
        }
        if (mqttClient.subscribe(MQTT_AUTOTUNE_APPLY_TOPIC)) {
            DEBUG_PRINTLN(F("Subscribed to auto-tune apply topic"));
        }

        // Publish "online" status
        JsonDocument doc;
        doc["status"] = "online";
        doc["id"] = clientId;
        doc["ip"] = WiFi.localIP().toString();
        doc["rssi"] = WiFi.RSSI();
        char statusBuf[200];
        serializeJson(doc, statusBuf, sizeof(statusBuf));
        mqttClient.publish(MQTT_STATUS_TOPIC, statusBuf, true);
    } else {
        int mqttState = mqttClient.state();
        DEBUG_PRINTF(" failed, rc=%d\n", mqttState);

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

    // Handle auto-tune topics
    if (topicStr == MQTT_AUTOTUNE_START_TOPIC) {
        handleAutoTuneStart(payloadStr);
    }
    else if (topicStr == MQTT_AUTOTUNE_STOP_TOPIC) {
        handleAutoTuneStop(payloadStr);
    }
    else if (topicStr == MQTT_AUTOTUNE_APPLY_TOPIC) {
        handleAutoTuneApply(payloadStr);
    }
}

void publishMQTTTelemetry() {
    JsonDocument doc;

    doc["timestamp"] = millis();
    doc["beanTemp"] = round(state.beanTemperature * 10) / 10.0;
    doc["envTemp"] = round(state.envTemperature * 10) / 10.0;
    doc["rateOfRise"] = round(getRateOfRise() * 100) / 100.0;
    doc["heaterPWM"] = static_cast<int>(state.heaterOutput * 100 / 255);  // Convert to percentage like original
    doc["fanPWM"] = state.fanPWM;
    doc["setpoint"] = round(state.beanSetpoint * 10) / 10.0;
    doc["controlMode"] = state.controlMode;
    doc["heaterEnable"] = state.heaterEnabled ? 1 : 0;  // Match original field name
    doc["uptime"] = millis() / 1000;
    doc["Kp"] = state.Kp;
    doc["Ki"] = state.Ki;
    doc["Kd"] = state.Kd;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["rssi"] = WiFi.RSSI();
    doc["systemStatus"] = state.systemStatus;

    if (doc.overflowed()) {
        DEBUG_PRINTLN(F("WARNING: Telemetry JSON document overflowed"));
    }

    static char telemetryBuf[512];
    size_t len = serializeJson(doc, telemetryBuf, sizeof(telemetryBuf));

    mqttClient.publish(MQTT_TELEMETRY_TOPIC, telemetryBuf);
    DEBUG_PRINTF("MQTT: Published telemetry (%d bytes)\n", len);
}

void publishMQTTStatus(const String& status) {
    JsonDocument doc;
    doc["status"] = status;
    doc["timestamp"] = millis();
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["version"] = "2.0.0-mqtt-only";

    static char statusBuf[256];
    serializeJson(doc, statusBuf, sizeof(statusBuf));

    mqttClient.publish(MQTT_STATUS_TOPIC, statusBuf, true);
}


void initializePins() {
    pinMode(SSR_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
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

        SystemStatus oldStatus = state.systemStatus;
        state.systemStatus = SYSTEM_OK;

        // Check connectivity
        SystemStatus connStatus = checkConnectivity();
        if (connStatus != SYSTEM_OK) state.systemStatus = connStatus;

        // Check sensors
        SystemStatus sensorStatus = checkSensors();
        if (sensorStatus != SYSTEM_OK) state.systemStatus = sensorStatus;

        // Log status changes
        if (state.systemStatus != oldStatus) {
            DEBUG_PRINTF_P(PSTR("System status changed: %d -> %d\n"), oldStatus, state.systemStatus);
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
    if (isnan(state.beanTemperature) || isnan(state.envTemperature)) {
        return SENSOR_ERROR;
    }
    if (state.beanTemperature < -50 || state.beanTemperature > 300) {
        return SENSOR_ERROR;
    }
    if (state.envTemperature < -50 || state.envTemperature > 300) {
        return SENSOR_ERROR;
    }
    return SYSTEM_OK;
}

void handleControlSetpoint(const String& payload) {
    float newSetpoint = payload.toFloat();
    if (newSetpoint >= MIN_BEAN_TEMP && newSetpoint <= MAX_BEAN_TEMP) {
        state.beanSetpoint = newSetpoint;
        DEBUG_PRINTF_P(PSTR("MQTT: Setpoint set to %.1f°C\n"), newSetpoint);
    }
}

void handleControlFan(const String& payload) {
    int newFanPWM = payload.toInt();
    if (newFanPWM >= 0 && newFanPWM <= 255) {
        state.fanPWM = newFanPWM;
        DEBUG_PRINTF_P(PSTR("MQTT: Fan PWM set to %d\n"), newFanPWM);
    }
}

void handleControlHeater(const String& payload) {
    int newHeaterPWM = payload.toInt();
    if (newHeaterPWM >= MIN_HEATER_PWM && newHeaterPWM <= MAX_HEATER_PWM) {
        state.heaterOutput = (newHeaterPWM * 255 + 50) / 100;  // Convert percentage to 0-255 with rounding
        DEBUG_PRINTF_P(PSTR("MQTT: Heater PWM set to %d%%\n"), newHeaterPWM);
    }
}

void handleControlMode(const String& payload) {
    if (payload == "manual" || payload == "0") {
        state.controlMode = MODE_MANUAL;
        DEBUG_PRINTLN(F("MQTT: Mode set to Manual"));
    } else if (payload == "auto" || payload == "1") {
        state.controlMode = MODE_AUTO;
        DEBUG_PRINTLN(F("MQTT: Mode set to Auto"));
    }
}

void handleControlEnable(const String& payload) {
    state.heaterEnabled = (payload == "1" || payload == "true");
    DEBUG_PRINTF_P(PSTR("MQTT: Heater enable set to %d\n"), state.heaterEnabled ? 1 : 0);
}

void handleControlPID(const String& payload) {
    // Expect JSON: {"kp": 15.0, "ki": 1.0, "kd": 25.0}
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
        if (doc["kp"].is<double>()) state.Kp = doc["kp"];
        if (doc["ki"].is<double>()) state.Ki = doc["ki"];
        if (doc["kd"].is<double>()) state.Kd = doc["kd"];
        updatePIDTunings();
        savePIDParameters();
        DEBUG_PRINTF_P(PSTR("MQTT: PID updated Kp=%.2f, Ki=%.2f, Kd=%.2f\n"), state.Kp, state.Ki, state.Kd);
    }
}

void handleEmergencyStop(const String& payload) {
    if (payload == "1" || payload == "true") {
        DEBUG_PRINTLN(F("MQTT: EMERGENCY STOP RECEIVED!"));
        state.heaterEnabled = false;
        state.fanPWM = 255;
        state.prevFanPWM = 255;
        state.heaterOutput = 0;
        ledcWrite(SSR_PIN, 0);
        ledcWrite(FAN_PIN, 255);
    }
}

// Auto-tune function implementations
void handleAutoTuneStart(const String& payload) {
    // Parse JSON payload for target temperature
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        DEBUG_PRINTLN(F("Auto-tune start: Invalid JSON payload"));
        return;
    }

    double targetTemp = doc["target_temperature"].as<double>();
    if (targetTemp < 150.0 || targetTemp > 250.0) {
        DEBUG_PRINTLN(F("Auto-tune start: Invalid target temperature"));
        return;
    }

    if (autoTuneState != AUTOTUNE_IDLE) {
        DEBUG_PRINTLN(F("Auto-tune already running"));
        return;
    }

    DEBUG_PRINTF("Starting auto-tune for target temperature: %.1f°C\n", targetTemp);

    // Initialize auto-tune
    autoTuneTargetTemp = targetTemp;
    autoTuneSetpoint = targetTemp;
    autoTuneState = AUTOTUNE_HEATING;
    autoTuneStartTime = millis();
    autoTuneStepStartTime = millis();
    autoTuneCurrentStep = 0;
    autoTuneReachedSetpoint = false;
    autoTuneStabilizationStartTime = 0;
    autoTuneStepTimeout = AUTOTUNE_MAX_STEP_TIME;

    // Save original PID parameters
    autoTuneOriginalKp = state.Kp;
    autoTuneOriginalKi = state.Ki;
    autoTuneOriginalKd = state.Kd;

    // Reset data arrays
    resetAutoTuneData();

    // Switch to manual mode for auto-tune
    state.controlMode = MODE_MANUAL;
    state.beanSetpoint = autoTuneTargetTemp;

    // Start with full heating output for initial heating phase
    state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;  // Convert percentage to PWM
    state.heaterEnabled = true;

    publishAutoTuneStatus();
}

void handleAutoTuneStop(const String& payload) {
    DEBUG_PRINTLN(F("Stopping auto-tune"));

    // Reset to original PID parameters
    state.Kp = autoTuneOriginalKp;
    state.Ki = autoTuneOriginalKi;
    state.Kd = autoTuneOriginalKd;
    updatePIDTunings();

    // Reset state
    autoTuneState = AUTOTUNE_IDLE;
    state.controlMode = MODE_AUTO;  // Return to auto mode
    state.heaterOutput = 0;
    state.heaterEnabled = false;

    publishAutoTuneStatus();
}

void handleAutoTuneApply(const String& payload) {
    DEBUG_PRINTLN(F("Applying auto-tune results"));

    // Only apply if we have completed auto-tune results
    if (autoTuneState != AUTOTUNE_COMPLETE) {
        DEBUG_PRINTLN(F("Auto-tune apply: No completed results to apply"));
        return;
    }

    // Validate that we have valid PID results
    if (autoTuneRecommendedKp <= 0 || autoTuneRecommendedKi < 0 || autoTuneRecommendedKd < 0) {
        DEBUG_PRINTLN(F("Auto-tune apply: Invalid PID parameters"));
        return;
    }

    DEBUG_PRINTF("Applying auto-tune results: Kp=%.2f, Ki=%.4f, Kd=%.2f\n",
                 autoTuneRecommendedKp, autoTuneRecommendedKi, autoTuneRecommendedKd);

    // Apply the recommended PID parameters
    state.Kp = autoTuneRecommendedKp;
    state.Ki = autoTuneRecommendedKi;
    state.Kd = autoTuneRecommendedKd;

    // Update the PID controller
    updatePIDTunings();

    // Save to NVS for persistence
    savePIDParameters();

    // Reset auto-tune state to idle
    autoTuneState = AUTOTUNE_IDLE;
    state.controlMode = MODE_AUTO;  // Return to auto mode with new PID parameters

    DEBUG_PRINTLN(F("Auto-tune results applied and saved to NVS"));

    // Publish updated status
    publishAutoTuneStatus();
}

void updateAutoTune() {
    if (autoTuneState != AUTOTUNE_RUNNING && autoTuneState != AUTOTUNE_ANALYZING && autoTuneState != AUTOTUNE_HEATING && autoTuneState != AUTOTUNE_STABILIZING) {
        return;
    }

    unsigned long now = millis();

    // Check for timeout
    if (now - autoTuneStartTime > AUTOTUNE_MAX_DURATION) {
        DEBUG_PRINTLN(F("Auto-tune timeout"));
        autoTuneState = AUTOTUNE_FAILED;
        handleAutoTuneStop("");  // Stop with empty payload
        return;
    }

    // Add current temperature to history
    if (autoTuneTempHistoryCount < 20) {
        autoTuneTempHistory[autoTuneTempHistoryIndex] = state.beanTemperature;
        autoTuneTempTimeHistory[autoTuneTempHistoryIndex] = now;
        autoTuneTempHistoryIndex = (autoTuneTempHistoryIndex + 1) % 20;
        autoTuneTempHistoryCount++;
    } else {
        autoTuneTempHistory[autoTuneTempHistoryIndex] = state.beanTemperature;
        autoTuneTempTimeHistory[autoTuneTempHistoryIndex] = now;
        autoTuneTempHistoryIndex = (autoTuneTempHistoryIndex + 1) % 20;
    }

    if (autoTuneState == AUTOTUNE_HEATING) {
        // Initial heating phase - bring system to target temperature with proportional control
        double tempError = autoTuneSetpoint - state.beanTemperature;

        if (tempError > 50) {
            // Far from target - use high heating power
            state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;
        } else if (tempError > 10) {
            // Approaching target - reduce power proportionally from (bias+amp) down to ~bias
            double powerRatio = tempError / 50.0; // 1.0 .. 0.2 as error 50..10
            double pct = autoTuneOutputBias + autoTuneOutputAmplitude * powerRatio;
            state.heaterOutput = pct * 255 / 100;
        } else if (tempError > 0) {
            // Very close to target: keep at baseline bias to continue approaching setpoint
            double pct = autoTuneOutputBias; // do not drop below bias while below setpoint
            state.heaterOutput = pct * 255 / 100;
        } else {
            // Above target - turn off heater completely
            state.heaterOutput = 0;
        }
        // Clamp and debug
        if (state.heaterOutput > 255) state.heaterOutput = 255;
        if (state.heaterOutput < 0) state.heaterOutput = 0;
        DEBUG_PRINTF("Auto-tune HEATING: err=%.1fC, out=%.0f\n", tempError, state.heaterOutput);

        // Check if we've reached within tolerance of setpoint
        float absErr = abs(state.beanTemperature - autoTuneSetpoint);
        if (absErr <= AUTOTUNE_SETPOINT_TOLERANCE) {
            if (!autoTuneReachedSetpoint) {
                autoTuneReachedSetpoint = true;
                autoTuneStabilizationStartTime = now;
                autoTuneRorStableStartTime = 0;
                autoTuneFastTrackStartTime = 0;
                DEBUG_PRINTF("Auto-tune reached setpoint %.1f°C, starting stabilization\n", autoTuneSetpoint);
            }
            // RoR stability check
            float absRor = fabs(getRateOfRise());
            if (absRor <= AUTOTUNE_STABILITY_ROR) {
                if (autoTuneRorStableStartTime == 0) autoTuneRorStableStartTime = now;
            } else {
                autoTuneRorStableStartTime = 0;
            }

            bool timeStable = (now - autoTuneStabilizationStartTime) >= AUTOTUNE_STABILIZATION_TIME;
            bool rorStable = (autoTuneRorStableStartTime != 0) && ((now - autoTuneRorStableStartTime) >= AUTOTUNE_STABILITY_ROR_TIME);

            // Check if we've been stable for required time OR RoR is very low for a short window
            if (timeStable || rorStable) {
                autoTuneState = AUTOTUNE_STABILIZING;
                autoTuneCurrentStep = 1;
                autoTuneStepStartTime = now;
                DEBUG_PRINTLN(F("Auto-tune stabilization complete, starting oscillation test"));
            }
        } else if (autoTuneReachedSetpoint) {
            // Allow small excursions (tolerance + hysteresis) without resetting the stabilization timer
            if (absErr > (AUTOTUNE_SETPOINT_TOLERANCE + AUTOTUNE_STABILITY_HYST)) {
                autoTuneReachedSetpoint = false;
                autoTuneStabilizationStartTime = now; // restart stabilization window when we re-enter tolerance
                autoTuneRorStableStartTime = 0;
                DEBUG_PRINTLN(F("Auto-tune: left tolerance band, resetting stabilization timer"));
            }
        } else {
            // Fast-track A: near tolerance band with low RoR for some time
            if (absErr <= (AUTOTUNE_SETPOINT_TOLERANCE + AUTOTUNE_FASTTRACK_EXTRA_BAND)) {
                float absRor = fabs(getRateOfRise());
                if (absRor <= AUTOTUNE_STABILITY_ROR) {
                    if (autoTuneFastTrackStartTime == 0) autoTuneFastTrackStartTime = now;
                } else {
                    autoTuneFastTrackStartTime = 0;
                }
                if (autoTuneFastTrackStartTime != 0 && (now - autoTuneFastTrackStartTime) >= AUTOTUNE_FASTTRACK_ROR_TIME) {
                    autoTuneState = AUTOTUNE_STABILIZING;
                    autoTuneCurrentStep = 1;
                    autoTuneStepStartTime = now;
                    DEBUG_PRINTLN(F("Auto-tune fast-track stabilization reached (near setpoint, low RoR)"));
                }
            } else {
                autoTuneFastTrackStartTime = 0;
            }

            // Fast-track B: Equilibrium at bias regardless of original setpoint
            float absRor2 = fabs(getRateOfRise());
            if (state.beanTemperature >= AUTOTUNE_EQUIL_MIN_TEMP && absRor2 <= AUTOTUNE_STABILITY_ROR) {
                if (autoTuneRorStableStartTime == 0) autoTuneRorStableStartTime = now;
            } else {
                autoTuneRorStableStartTime = 0;
            }
            if (autoTuneRorStableStartTime != 0 && (now - autoTuneRorStableStartTime) >= AUTOTUNE_FASTTRACK_ROR_TIME) {
                autoTuneSetpoint = state.beanTemperature; // anchor oscillation around current equilibrium
                autoTuneState = AUTOTUNE_STABILIZING;
                autoTuneCurrentStep = 1;
                autoTuneStepStartTime = now;
                autoTuneReachedSetpoint = true;
                autoTuneStabilizationStartTime = now;
                DEBUG_PRINTF("Auto-tune equilibrium accepted at %.1f°C, proceeding to stabilizing.\n", autoTuneSetpoint);
            }
        }

        // Check for heating timeout
        if (now - autoTuneStartTime >= AUTOTUNE_INITIAL_STEP_TIME) {
            DEBUG_PRINTLN(F("Auto-tune heating phase timeout: forcing progression using equilibrium"));
            // Accept current equilibrium to avoid stalling
            autoTuneSetpoint = state.beanTemperature;
            autoTuneState = AUTOTUNE_STABILIZING;
            autoTuneCurrentStep = 1;
            autoTuneStepStartTime = now;
            autoTuneReachedSetpoint = true;
            autoTuneStabilizationStartTime = now;
        }
    }
    else if (autoTuneState == AUTOTUNE_STABILIZING) {
        // Stabilizing phase - maintain baseline output
        state.heaterOutput = autoTuneOutputBias * 255 / 100;

        // Check if stable for minimum time before starting oscillation
        if (now - autoTuneStepStartTime >= AUTOTUNE_MIN_STEP_TIME) {
            autoTuneState = AUTOTUNE_RUNNING;
            autoTuneCurrentStep = 1;
            autoTuneStepStartTime = now;
            // Immediately choose an initial oscillation side based on current temp
            if (state.beanTemperature <= autoTuneSetpoint) {
                autoTuneOutputHigh = true;
                state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;
            } else {
                autoTuneOutputHigh = false;
                state.heaterOutput = (autoTuneOutputBias - autoTuneOutputAmplitude) * 255 / 100;
            }
            if (state.heaterOutput > 255) state.heaterOutput = 255;
            if (state.heaterOutput < 0) state.heaterOutput = 0;
            DEBUG_PRINTLN(F("Auto-tune starting oscillation phase"));
        }
    }
    else if (autoTuneState == AUTOTUNE_RUNNING) {
        // Oscillation phase - relay with hysteresis around setpoint
        unsigned long stepDuration = now - autoTuneStepStartTime;
        unsigned long stepTimeout = (autoTuneCurrentStep <= 2) ? AUTOTUNE_INITIAL_STEP_TIME : autoTuneStepTimeout;
        float error = state.beanTemperature - autoTuneSetpoint;

        bool toggled = false;
        if (autoTuneOutputHigh) {
            if (error >= AUTOTUNE_RELAY_HYST || stepDuration >= stepTimeout) {
                // Record peak on high-to-low transition
                if (autoTunePeakCount < 10) {
                    autoTunePeaks[autoTunePeakCount].temperature = state.beanTemperature;
                    autoTunePeaks[autoTunePeakCount].time = now / 1000.0f;
                    autoTunePeakCount++;
                }
                autoTuneOutputHigh = false;
                toggled = true;
                state.heaterOutput = (autoTuneOutputBias - autoTuneOutputAmplitude) * 255 / 100;
            }
        } else {
            if (error <= -AUTOTUNE_RELAY_HYST || stepDuration >= stepTimeout) {
                // Record valley on low-to-high transition
                if (autoTuneValleyCount < 10) {
                    autoTuneValleys[autoTuneValleyCount].temperature = state.beanTemperature;
                    autoTuneValleys[autoTuneValleyCount].time = now / 1000.0f;
                    autoTuneValleyCount++;
                }
                autoTuneOutputHigh = true;
                toggled = true;
                state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;
            }
        }

        if (toggled) {
            autoTuneCurrentStep++;
            autoTuneStepStartTime = now;
            if (state.heaterOutput > 255) state.heaterOutput = 255;
            if (state.heaterOutput < 0) state.heaterOutput = 0;
            DEBUG_PRINTF("Auto-tune step %d: output = %.0f, error=%.1f\n", autoTuneCurrentStep, state.heaterOutput, error);
            if (autoTuneCurrentStep >= AUTOTUNE_TOTAL_STEPS) {
                autoTuneState = AUTOTUNE_ANALYZING;
                DEBUG_PRINTLN(F("Auto-tune data collection complete, analyzing..."));
            }
        }
    } else if (autoTuneState == AUTOTUNE_ANALYZING) {
        // Calculate PID parameters
        if (calculateAutoTunePIDParameters()) {
            autoTuneUsedFallback = false;
            autoTuneState = AUTOTUNE_COMPLETE;
            DEBUG_PRINTLN(F("Auto-tune complete!"));
        } else {
            // Fallback: use current PID as conservative recommendation and mark complete
            autoTuneRecommendedKp = state.Kp;
            autoTuneRecommendedKi = state.Ki;
            autoTuneRecommendedKd = state.Kd;
            autoTuneUsedFallback = true;
            autoTuneState = AUTOTUNE_COMPLETE;
            DEBUG_PRINTLN(F("Auto-tune: fallback PID used due to insufficient oscillation data"));
        }
        publishAutoTuneResults();
        // Safe state: disable heater, return to auto control awaiting apply/stop
        state.heaterEnabled = false;
        state.controlMode = MODE_AUTO;
        publishAutoTuneStatus();
    }
}

void publishAutoTuneStatus() {
    if (!mqttClient.connected()) return;

    JsonDocument doc;

    doc["state"] = getAutoTuneStateString(autoTuneState);
    doc["message"] = getAutoTuneStateString(autoTuneState);

    if (autoTuneState == AUTOTUNE_RUNNING || autoTuneState == AUTOTUNE_ANALYZING) {
        // Calculate progress based on steps completed
        float progress = ((float)autoTuneCurrentStep / AUTOTUNE_TOTAL_STEPS) * 100.0;
        if (autoTuneState == AUTOTUNE_ANALYZING) {
            progress = 90.0; // Show 90% during analysis
        }
        doc["progress"] = progress;
        doc["current_step"] = autoTuneCurrentStep;
        doc["total_steps"] = AUTOTUNE_TOTAL_STEPS;
    } else {
        doc["progress"] = 0;
        doc["current_step"] = nullptr;
        doc["total_steps"] = AUTOTUNE_TOTAL_STEPS;
    }

    if (autoTuneState == AUTOTUNE_COMPLETE) {
        doc["recommended_kp"] = autoTuneRecommendedKp;
        doc["recommended_ki"] = autoTuneRecommendedKi;
        doc["recommended_kd"] = autoTuneRecommendedKd;
        doc["progress"] = 100;
    } else {
        doc["recommended_kp"] = nullptr;
        doc["recommended_ki"] = nullptr;
        doc["recommended_kd"] = nullptr;
    }

    doc["timestamp"] = millis();

    static char atStatusBuf[512];
    size_t len = serializeJson(doc, atStatusBuf, sizeof(atStatusBuf));

    mqttClient.publish(MQTT_AUTOTUNE_STATUS_TOPIC, atStatusBuf, true);  // Retained message
    DEBUG_PRINTF("MQTT: Published auto-tune status (%d bytes)\n", len);
}

void publishAutoTuneResults() {
    if (!mqttClient.connected()) return;

    JsonDocument doc;

    doc["state"] = "complete";
    doc["recommended_kp"] = autoTuneRecommendedKp;
    doc["recommended_ki"] = autoTuneRecommendedKi;
    doc["recommended_kd"] = autoTuneRecommendedKd;
    doc["quality"] = autoTuneUsedFallback ? "fallback" : "estimated";
    doc["original_kp"] = autoTuneOriginalKp;
    doc["original_ki"] = autoTuneOriginalKi;
    doc["original_kd"] = autoTuneOriginalKd;
    doc["target_temperature"] = autoTuneTargetTemp;
    doc["peak_count"] = autoTunePeakCount;
    doc["valley_count"] = autoTuneValleyCount;
    doc["timestamp"] = millis();
    doc["duration"] = (millis() - autoTuneStartTime) / 1000; // Duration in seconds

    static char atResultsBuf[512];
    size_t len = serializeJson(doc, atResultsBuf, sizeof(atResultsBuf));

    mqttClient.publish(MQTT_AUTOTUNE_RESULTS_TOPIC, atResultsBuf, true);  // Retained message
    DEBUG_PRINTF("MQTT: Published auto-tune results (%d bytes)\n", len);
}

bool checkAutoTuneCrossedSetpoint() {
    if (autoTuneTempHistoryCount < 3) return false;

    // Check if temperature crossed setpoint in recent history
    int recentIdx = (autoTuneTempHistoryIndex - 1 + 20) % 20;
    int prevIdx = (autoTuneTempHistoryIndex - 2 + 20) % 20;

    float current = autoTuneTempHistory[recentIdx];
    float previous = autoTuneTempHistory[prevIdx];

    // Check for crossing (either direction) around the active oscillation setpoint
    const float ref = autoTuneSetpoint; // use current setpoint (may be equilibrium)
    bool crossed = (previous < ref && current >= ref) ||
                   (previous > ref && current <= ref);

    if (crossed) {
        // Record peak or valley
        if (autoTunePeakCount < 10 && current > ref) {
            autoTunePeaks[autoTunePeakCount].temperature = current;
            autoTunePeaks[autoTunePeakCount].time = autoTuneTempTimeHistory[recentIdx] / 1000.0;
            autoTunePeakCount++;
            DEBUG_PRINTF("Auto-tune peak %d: %.2f°C\n", autoTunePeakCount, current);
        }

        if (autoTuneValleyCount < 10 && current < ref) {
            autoTuneValleys[autoTuneValleyCount].temperature = current;
            autoTuneValleys[autoTuneValleyCount].time = autoTuneTempTimeHistory[recentIdx] / 1000.0;
            autoTuneValleyCount++;
            DEBUG_PRINTF("Auto-tune valley %d: %.2f°C\n", autoTuneValleyCount, current);
        }
    }

    return crossed;
}

bool calculateAutoTunePIDParameters() {
    // Prefer peak/valley method when we have at least a couple of extrema
    float avgPeriod = 0;
    float avgAmplitude = 0;
    bool computed = false;

    if (autoTunePeakCount >= 2 && autoTuneValleyCount >= 2) {
        // Average peak-to-peak period
        float totalPeriod = 0; int periodCount = 0;
        for (int i = 1; i < autoTunePeakCount; i++) {
            float period = autoTunePeaks[i].time - autoTunePeaks[i-1].time;
            if (period > 0) { totalPeriod += period; periodCount++; }
        }
        if (periodCount > 0) avgPeriod = totalPeriod / periodCount;

        // Amplitude: average of matched peak-valley pairs near half-period apart
        float totalAmplitude = 0; int amplitudeCount = 0;
        for (int i = 0; i < autoTunePeakCount; i++) {
            for (int j = 0; j < autoTuneValleyCount; j++) {
                if (fabs(autoTunePeaks[i].time - autoTuneValleys[j].time) < (avgPeriod / 2 + 2)) {
                    float amplitude = fabs(autoTunePeaks[i].temperature - autoTuneValleys[j].temperature) / 2.0f;
                    totalAmplitude += amplitude;
                    amplitudeCount++;
                    break;
                }
            }
        }
        if (periodCount > 0 && amplitudeCount > 0) {
            avgAmplitude = totalAmplitude / amplitudeCount;
            computed = true;
        }
    }

    // Fallback: derive from recent history around oscillation setpoint
    if (!computed) {
        // Estimate period from zero-crossings of (temp - setpoint) in recent history
        const float ref = autoTuneSetpoint;
        int crossings[10]; int crossCount = 0;
        for (int i = 1; i < autoTuneTempHistoryCount && crossCount < 10; i++) {
            int idxPrev = (autoTuneTempHistoryIndex - i - 1 + 20) % 20;
            int idxCur = (autoTuneTempHistoryIndex - i + 20) % 20;
            float prev = autoTuneTempHistory[idxPrev] - ref;
            float cur = autoTuneTempHistory[idxCur] - ref;
            if ((prev < 0 && cur >= 0) || (prev > 0 && cur <= 0)) {
                crossings[crossCount++] = autoTuneTempTimeHistory[idxCur];
            }
        }
        if (crossCount >= 2) {
            // approximate full period as 2 * average half-period between alternating crossings
            float totalHalf = 0; int halfCount = 0;
            for (int i = 1; i < crossCount; i++) {
                float dt = (crossings[i] - crossings[i-1]) / 1000.0f;
                if (dt > 0) { totalHalf += dt; halfCount++; }
            }
            if (halfCount > 0) avgPeriod = 2.0f * (totalHalf / halfCount);
        }
        // Amplitude estimate: range/2 from history
        float tmin = 1e9f, tmax = -1e9f;
        for (int i = 0; i < autoTuneTempHistoryCount; i++) {
            int idx = (autoTuneTempHistoryIndex - i - 1 + 20) % 20;
            float v = autoTuneTempHistory[idx];
            if (v < tmin) tmin = v; if (v > tmax) tmax = v;
        }
        float range = (tmax > tmin) ? (tmax - tmin) : 0;
        avgAmplitude = range / 2.0f;
        computed = (avgPeriod > 0.5f && avgAmplitude > 0.2f);
    }

    if (!computed) {
        DEBUG_PRINTLN(F("Insufficient oscillation data for PID calculation"));
        return false;
    }

    // Calculate ultimate gain (Ku) from output swing and amplitude
    float outputSwing = autoTuneOutputAmplitude * 2;  // total percentage swing
    float Ku = (outputSwing * 4) / (avgAmplitude * 3.14159f);  // heuristic

    // Ziegler-Nichols PID rules
    autoTuneRecommendedKp = 0.6f * Ku;
    autoTuneRecommendedKi = (2.0f * autoTuneRecommendedKp) / avgPeriod;
    autoTuneRecommendedKd = (autoTuneRecommendedKp * avgPeriod) / 8.0f;

    // Bounds
    if (autoTuneRecommendedKp > 50) autoTuneRecommendedKp = 50;
    if (autoTuneRecommendedKp < 0.5f) autoTuneRecommendedKp = 0.5f;
    if (autoTuneRecommendedKi > 5) autoTuneRecommendedKi = 5;
    if (autoTuneRecommendedKi < 0.05f) autoTuneRecommendedKi = 0.05f;
    if (autoTuneRecommendedKd > 100) autoTuneRecommendedKd = 100;
    if (autoTuneRecommendedKd < 0.5f) autoTuneRecommendedKd = 0.5f;

    DEBUG_PRINTF("Auto-tune results: Ku=%.2f, Period=%.1fs, Amplitude=%.2f°C\n", Ku, avgPeriod, avgAmplitude);
    DEBUG_PRINTF("Recommended PID: Kp=%.2f, Ki=%.4f, Kd=%.2f\n", autoTuneRecommendedKp, autoTuneRecommendedKi, autoTuneRecommendedKd);
    return true;
}

void resetAutoTuneData() {
    autoTunePeakCount = 0;
    autoTuneValleyCount = 0;
    autoTuneTempHistoryCount = 0;
    autoTuneTempHistoryIndex = 0;
    autoTuneRecommendedKp = 0;
    autoTuneRecommendedKi = 0;
    autoTuneRecommendedKd = 0;
}

const char* getAutoTuneStateString(AutoTuneState atState) {
    switch (atState) {
        case AUTOTUNE_IDLE: return "idle";
        case AUTOTUNE_HEATING: return "heating";
        case AUTOTUNE_STABILIZING: return "stabilizing";
        case AUTOTUNE_RUNNING: return "running";
        case AUTOTUNE_ANALYZING: return "analyzing";
        case AUTOTUNE_COMPLETE: return "complete";
        case AUTOTUNE_FAILED: return "failed";
        default: return "unknown";
    }
}
