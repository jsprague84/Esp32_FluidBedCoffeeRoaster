// ESP32 Coffee Roaster - Streamlined MQTT-Only Version
// Core control functions with WiFi, OTA, and MQTT communication

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "debug.h"
#include "roaster_state.h"
#include "temperature.h"
#include "heater_control.h"
#include "safety.h"
#include "mqtt_handler.h"
#include "autotune.h"

// WiFi credentials (declared extern in config.h)
const char* ssid = "jswifi";
const char* password = "helloworld";

// Global roaster state
RoasterState state = {};

unsigned long lastStatusUpdate = 0;
const unsigned long statusUpdateInterval = 5000;

// Timing variables
unsigned long lastSerialOutput = 0;
unsigned long lastMqttPublish = 0;

// Function prototypes
void initializePins();
void initStateDefaults();
void checkWiFiConnection();
void updateSystemStatus();
SystemStatus checkSensors();
SystemStatus checkConnectivity();

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
    initSafety();
    initAutoTune();

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

        initMQTT();
    } else {
        Serial.println(F("\nWiFi connection failed!"));
    }

    initTemperature();

    // Configure OTA
    ArduinoOTA.setHostname(MQTT_CLIENT_ID);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
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

    // No mqttClient.loop() needed — ESP32MQTTClient runs in background FreeRTOS task

    // Read Temperatures
    readTemperatures();

    // Safety checks BEFORE heater control
    runSafetyChecks();

    // Heater and fan control
    updateHeaterControl();

    // Auto-tune update
    updateAutoTune();

    // MQTT Telemetry Publishing
    if (mqttIsConnected() && millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL) {
        lastMqttPublish = millis();
        publishTelemetry();
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
                     mqttIsConnected() ? "OK" : "DISCONNECTED");
    }
}

void initializePins() {
    pinMode(SSR_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
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
    if (!mqttIsConnected()) {
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
