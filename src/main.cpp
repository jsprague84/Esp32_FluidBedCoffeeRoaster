// ESP32 Coffee Roaster - Orchestration Layer
// Calls module functions for temperature, heater, safety, MQTT, and autotune

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

// Timing variables
static unsigned long lastStatusUpdate = 0;
static unsigned long lastSerialOutput = 0;
static unsigned long lastMqttPublish = 0;

// Forward declarations
static void initializePins();
static void initStateDefaults();
static void setupWiFi();
static void setupOTA();
static void checkWiFiConnection();
static void updateSystemStatus();

void setupNTP() {
    configTime(NTP_GMT_OFFSET, NTP_DAYLIGHT_OFFSET, NTP_SERVER_1, NTP_SERVER_2);
    DEBUG_PRINTLN(F("NTP: Waiting for time sync..."));
    for (int i = 0; i < 10; i++) {
        time_t now = time(nullptr);
        if (now > 1000000000) {
            DEBUG_PRINTF("NTP: Time synced (epoch: %ld)\n", (long)now);
            return;
        }
        delay(1000);
    }
    DEBUG_PRINTLN(F("NTP: Sync timeout - continuing without NTP"));
}

time_t getEpochTime() {
    time_t now = time(nullptr);
    return (now > 1000000000) ? now : 0;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    DEBUG_PRINTLN(F("Starting Coffee Roaster Control with MQTT..."));

    initStateDefaults();
    initializePins();
    initTemperature();
    setupWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        setupNTP();
        initMQTT();
    }
    initHeaterControl();
    initAutoTune();
    setupOTA();
    initSafety();

    // Watchdog setup
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
        .idle_core_mask = (1 << 0),
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    DEBUG_PRINTLN(F("Setup complete - All systems ready"));
}

void loop() {
    esp_task_wdt_reset();
    checkWiFiConnection();
    ArduinoOTA.handle();
    readTemperatures();
    runSafetyChecks();
    updateHeaterControl();
    updateAutoTune();
    updateSystemStatus();

    // MQTT Telemetry Publishing at 1Hz
    if (mqttIsConnected() && millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL) {
        lastMqttPublish = millis();
        publishTelemetry();
    }

    // Periodic serial debug output
    if (millis() - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL) {
        lastSerialOutput = millis();
        DEBUG_PRINTF("BT:%.1f ET:%.1f ROR:%.1f Heater:%d Fan:%d Mode:%s MQTT:%s\n",
                     state.beanTemperature, state.envTemperature, getRateOfRise(),
                     static_cast<int>(state.heaterOutput), state.fanPWM,
                     (state.controlMode == MODE_MANUAL) ? "M" : "A",
                     mqttIsConnected() ? "OK" : "X");
    }
}

static void initStateDefaults() {
    state = {};
    state.prevFanPWM = -1;
    state.controlMode = MODE_MANUAL;
    state.Kp = DEFAULT_KP;
    state.Ki = DEFAULT_KI;
    state.Kd = DEFAULT_KD;
    state.beanTempOffset = TEMP_CALIBRATION_BEAN;
    state.envTempOffset = TEMP_CALIBRATION_ENV;
}

static void initializePins() {
    pinMode(SSR_PIN, OUTPUT);  pinMode(FAN_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW); digitalWrite(FAN_PIN, LOW);
}

static void setupWiFi() {
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
    } else {
        Serial.println(F("\nWiFi connection failed!"));
    }
}

static void setupOTA() {
    ArduinoOTA.setHostname(MQTT_CLIENT_ID);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        DEBUG_PRINTF("OTA: Start updating %s\n",
                     (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    });
    ArduinoOTA.onEnd([]() { DEBUG_PRINTLN(F("OTA: Done")); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DEBUG_PRINTF("OTA: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        DEBUG_PRINTF("OTA Error[%u]\n", error);
    });
    ArduinoOTA.begin();
    DEBUG_PRINTLN(F("OTA Ready"));
}

static void checkWiFiConnection() {
    static unsigned long lastWiFiCheck = 0;
    if (millis() - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
        lastWiFiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            DEBUG_PRINTLN(F("WiFi disconnected. Reconnecting..."));
            WiFi.disconnect();
            WiFi.begin(ssid, password);
        }
    }
}

static void updateSystemStatus() {
    if (millis() - lastStatusUpdate < 5000) return;
    lastStatusUpdate = millis();

    SystemStatus oldStatus = state.systemStatus;
    state.systemStatus = SYSTEM_OK;

    if (WiFi.status() != WL_CONNECTED) {
        state.systemStatus = WIFI_ERROR;
    } else if (!mqttIsConnected()) {
        state.systemStatus = MQTT_ERROR;
    }

    if (isnan(state.beanTemperature) || isnan(state.envTemperature) ||
        state.beanTemperature < -50 || state.beanTemperature > 300 ||
        state.envTemperature < -50 || state.envTemperature > 300) {
        state.systemStatus = SENSOR_ERROR;
    }

    if (state.systemStatus != oldStatus) {
        DEBUG_PRINTF_P(PSTR("System status changed: %d -> %d\n"), oldStatus, state.systemStatus);
    }
}
