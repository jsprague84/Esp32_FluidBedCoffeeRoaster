#include <Arduino.h>
#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "debug.h"
#include "roaster_state.h"
#include "mqtt_handler.h"
#include "heater_control.h"
#include "safety.h"
#include "temperature.h"
#include "autotune.h"

// MQTT client instance
static ESP32MQTTClient mqttClient;

// MQTT URI built from config
static char mqttUri[128];

// Last connected timestamp for safety module
static unsigned long lastConnectedTime = 0;

// Forward declarations for control handlers
static void handleControlSetpoint(const char* payload, size_t len);
static void handleControlFan(const char* payload, size_t len);
static void handleControlHeater(const char* payload, size_t len);
static void handleControlMode(const char* payload, size_t len);
static void handleControlEnable(const char* payload, size_t len);
static void handleControlPID(const char* payload, size_t len);
static void handleEmergencyStop(const char* payload, size_t len);

// Autotune handlers are now in autotune.cpp via autotune.h

// ESP-IDF 5.x MQTT event handler
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
    mqttClient.onEventCallback(event);
}

// Called by ESP32MQTTClient when connection is established
void onMqttConnect(esp_mqtt_client_handle_t client) {
    if (mqttClient.isMyTurn(client)) {
        DEBUG_PRINTLN(F("MQTT connected!"));
        lastConnectedTime = millis();
        safetyNotifyMqttConnected();

        // Subscribe to control topics with wildcard
        mqttClient.subscribe(std::string(MQTT_CONTROL_TOPIC) + "/#",
            [](const std::string &topic, const std::string &message) {
                // Extract command from topic: roaster/{id}/control/{command}
                const char* topicStr = topic.c_str();
                const char* controlBase = MQTT_CONTROL_TOPIC;
                size_t baseLen = strlen(controlBase);

                if (strncmp(topicStr, controlBase, baseLen) != 0) return;

                const char* command = topicStr + baseLen;
                if (command[0] == '/') command++;

                const char* payload = message.c_str();
                size_t len = message.length();

                if (strcmp(command, "setpoint") == 0) {
                    handleControlSetpoint(payload, len);
                } else if (strcmp(command, "fan_pwm") == 0) {
                    handleControlFan(payload, len);
                } else if (strcmp(command, "heater_pwm") == 0) {
                    handleControlHeater(payload, len);
                } else if (strcmp(command, "mode") == 0) {
                    handleControlMode(payload, len);
                } else if (strcmp(command, "heater_enable") == 0) {
                    handleControlEnable(payload, len);
                } else if (strcmp(command, "pid") == 0) {
                    handleControlPID(payload, len);
                } else if (strcmp(command, "emergency_stop") == 0) {
                    handleEmergencyStop(payload, len);
                } else {
                    DEBUG_PRINTF("MQTT: Unknown control command: %s\n", command);
                }
            });

        // Subscribe to auto-tune topics
        mqttClient.subscribe(std::string(MQTT_AUTOTUNE_START_TOPIC),
            [](const std::string &message) {
                handleAutoTuneStart(message.c_str(), message.length());
            });

        mqttClient.subscribe(std::string(MQTT_AUTOTUNE_STOP_TOPIC),
            [](const std::string &message) {
                handleAutoTuneStop(message.c_str(), message.length());
            });

        mqttClient.subscribe(std::string(MQTT_AUTOTUNE_APPLY_TOPIC),
            [](const std::string &message) {
                handleAutoTuneApply(message.c_str(), message.length());
            });

        // Publish online status
        publishStatus("online");

        DEBUG_PRINTLN(F("MQTT: Subscribed to control and auto-tune topics"));
    }
}

void initMQTT() {
    // Build URI from config
    snprintf(mqttUri, sizeof(mqttUri), "mqtt://%s:%d", MQTT_BROKER, MQTT_PORT);

    mqttClient.setURI(mqttUri);
    mqttClient.setMqttClientName(MQTT_CLIENT_ID);
    mqttClient.setKeepAlive(15);
    mqttClient.setMaxPacketSize(MQTT_BUFFER_SIZE);

    // LWT: offline status message
    mqttClient.enableLastWillMessage(MQTT_STATUS_TOPIC,
        "{\"status\":\"offline\",\"id\":\"" MQTT_CLIENT_ID "\",\"timestamp\":0}",
        true);

    DEBUG_PRINTF("MQTT: Configured URI %s\n", mqttUri);

    // Start MQTT in background FreeRTOS task
    if (mqttClient.loopStart()) {
        DEBUG_PRINTLN(F("MQTT: Background task started"));
        lastConnectedTime = millis();
    } else {
        DEBUG_PRINTLN(F("MQTT: Failed to start background task"));
    }
}

bool mqttIsConnected() {
    return mqttClient.isConnected();
}

unsigned long getLastConnectedTime() {
    if (mqttClient.isConnected()) {
        lastConnectedTime = millis();
    }
    return lastConnectedTime;
}

void publishTelemetry() {
    if (!mqttClient.isConnected()) return;

    JsonDocument doc;

    doc["timestamp"] = millis();
    doc["beanTemp"] = round(state.beanTemperature * 10) / 10.0;
    doc["envTemp"] = round(state.envTemperature * 10) / 10.0;
    doc["rateOfRise"] = round(getRateOfRise() * 100) / 100.0;
    doc["heaterPWM"] = static_cast<int>(state.heaterOutput * 100 / 255);
    doc["fanPWM"] = state.fanPWM;
    doc["setpoint"] = round(state.beanSetpoint * 10) / 10.0;
    doc["controlMode"] = state.controlMode;
    doc["heaterEnable"] = state.heaterEnabled ? 1 : 0;
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

    mqttClient.publish(std::string(MQTT_TELEMETRY_TOPIC),
                       std::string(telemetryBuf, len));
    DEBUG_PRINTF("MQTT: Published telemetry (%d bytes)\n", len);
}

void publishStatus(const char* status) {
    if (!mqttClient.isConnected()) return;

    JsonDocument doc;
    doc["status"] = status;
    doc["id"] = MQTT_CLIENT_ID;
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["version"] = FIRMWARE_VERSION;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["timestamp"] = millis();

    static char statusBuf[256];
    size_t len = serializeJson(doc, statusBuf, sizeof(statusBuf));

    mqttClient.publish(std::string(MQTT_STATUS_TOPIC),
                       std::string(statusBuf, len), 0, true);
}

// Raw publish helpers called by autotune.cpp
void mqttPublishAutoTuneStatus(const char* buf, size_t len) {
    mqttClient.publish(std::string(MQTT_AUTOTUNE_STATUS_TOPIC),
                       std::string(buf, len), 0, true);
}

void mqttPublishAutoTuneResults(const char* buf, size_t len) {
    mqttClient.publish(std::string(MQTT_AUTOTUNE_RESULTS_TOPIC),
                       std::string(buf, len), 0, true);
}

// --- Control message handlers (char buffer based, no Arduino String) ---

static void handleControlSetpoint(const char* payload, size_t len) {
    char buf[32];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    char* endptr;
    float newSetpoint = strtof(buf, &endptr);
    if (endptr == buf) {
        DEBUG_PRINTF("MQTT: Invalid setpoint payload: %s\n", buf);
        return;
    }
    if (newSetpoint >= MIN_BEAN_TEMP && newSetpoint <= MAX_BEAN_TEMP) {
        state.beanSetpoint = newSetpoint;
        DEBUG_PRINTF("MQTT: Setpoint set to %.1f°C\n", newSetpoint);
    }
}

static void handleControlFan(const char* payload, size_t len) {
    char buf[16];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    char* endptr;
    long newFanPWM = strtol(buf, &endptr, 10);
    if (endptr == buf) {
        DEBUG_PRINTF("MQTT: Invalid fan PWM payload: %s\n", buf);
        return;
    }
    if (newFanPWM >= 0 && newFanPWM <= 255) {
        state.fanPWM = (int)newFanPWM;
        DEBUG_PRINTF("MQTT: Fan PWM set to %d\n", state.fanPWM);
    }
}

static void handleControlHeater(const char* payload, size_t len) {
    char buf[16];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    char* endptr;
    long newHeaterPWM = strtol(buf, &endptr, 10);
    if (endptr == buf) {
        DEBUG_PRINTF("MQTT: Invalid heater PWM payload: %s\n", buf);
        return;
    }
    if (newHeaterPWM >= MIN_HEATER_PWM && newHeaterPWM <= MAX_HEATER_PWM) {
        state.heaterOutput = (newHeaterPWM * 255 + 50) / 100;
        DEBUG_PRINTF("MQTT: Heater PWM set to %ld%%\n", newHeaterPWM);
    }
}

static void handleControlMode(const char* payload, size_t len) {
    char buf[16];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    if (strcmp(buf, "manual") == 0 || strcmp(buf, "0") == 0) {
        state.controlMode = MODE_MANUAL;
        DEBUG_PRINTLN(F("MQTT: Mode set to Manual"));
    } else if (strcmp(buf, "auto") == 0 || strcmp(buf, "1") == 0) {
        state.controlMode = MODE_AUTO;
        DEBUG_PRINTLN(F("MQTT: Mode set to Auto"));
    } else {
        DEBUG_PRINTF("MQTT: Invalid mode payload: %s\n", buf);
    }
}

static void handleControlEnable(const char* payload, size_t len) {
    char buf[16];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    state.heaterEnabled = (strcmp(buf, "1") == 0 || strcmp(buf, "true") == 0);
    DEBUG_PRINTF("MQTT: Heater enable set to %d\n", state.heaterEnabled ? 1 : 0);
}

static void handleControlPID(const char* payload, size_t len) {
    // Copy payload to stack buffer for parsing
    char buf[256];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, buf);
    if (!error) {
        if (doc["kp"].is<double>()) state.Kp = doc["kp"];
        if (doc["ki"].is<double>()) state.Ki = doc["ki"];
        if (doc["kd"].is<double>()) state.Kd = doc["kd"];
        updatePIDTunings();
        savePIDParameters();
        DEBUG_PRINTF("MQTT: PID updated Kp=%.2f, Ki=%.2f, Kd=%.2f\n", state.Kp, state.Ki, state.Kd);
    } else {
        DEBUG_PRINTF("MQTT: Invalid PID JSON: %s\n", buf);
    }
}

static void handleEmergencyStop(const char* payload, size_t len) {
    char buf[16];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    if (strcmp(buf, "1") == 0 || strcmp(buf, "true") == 0) {
        DEBUG_PRINTLN(F("MQTT: EMERGENCY STOP RECEIVED!"));
        triggerSafeShutdown();
    }
}
