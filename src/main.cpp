// ESP32 Coffee Roaster - Streamlined MQTT-Only Version
// Core control functions with WiFi, OTA, and MQTT communication

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "config.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "debug.h"
#include "roaster_state.h"
#include "temperature.h"
#include "heater_control.h"
#include "safety.h"
#include "mqtt_handler.h"

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

// Auto-tune state variables
typedef enum {
    AUTOTUNE_IDLE = 0,
    AUTOTUNE_HEATING = 1,
    AUTOTUNE_STABILIZING = 2,
    AUTOTUNE_RUNNING = 3,
    AUTOTUNE_ANALYZING = 4,
    AUTOTUNE_COMPLETE = 5,
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
float autoTuneStepTimeout = AUTOTUNE_MAX_STEP_TIME;
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
const unsigned long autoTuneStatusPublishInterval = 2000;

// Function prototypes
void initializePins();
void initStateDefaults();
void checkWiFiConnection();
void updateSystemStatus();
SystemStatus checkSensors();
SystemStatus checkConnectivity();

// Auto-tune function prototypes
void updateAutoTune();
bool calculateAutoTunePIDParameters();
void resetAutoTuneData();
const char* getAutoTuneStateString(AutoTuneState atState);

// MQTT message handlers called from mqtt_handler.cpp
void handleAutoTuneStartMsg(const char* payload, size_t len);
void handleAutoTuneStopMsg(const char* payload, size_t len);
void handleAutoTuneApplyMsg(const char* payload, size_t len);

// Publish helpers called from mqtt_handler.cpp
void publishAutoTuneStatusImpl();
void publishAutoTuneResultsImpl();

// Defined in mqtt_handler.cpp
extern void mqttPublishAutoTuneStatus(const char* buf, size_t len);
extern void mqttPublishAutoTuneResults(const char* buf, size_t len);

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

    // Auto-tune update: run state machine in all active phases
    if (autoTuneState == AUTOTUNE_HEATING ||
        autoTuneState == AUTOTUNE_STABILIZING ||
        autoTuneState == AUTOTUNE_RUNNING ||
        autoTuneState == AUTOTUNE_ANALYZING) {
        updateAutoTune();
    }

    // Auto-tune status publishing
    if (mqttIsConnected() && millis() - lastAutoTuneStatusPublish >= autoTuneStatusPublishInterval) {
        lastAutoTuneStatusPublish = millis();
        if (autoTuneState != AUTOTUNE_IDLE) {
            publishAutoTuneStatus();
        }
    }

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

// --- Auto-tune MQTT message handlers (called from mqtt_handler.cpp) ---

void handleAutoTuneStartMsg(const char* payload, size_t len) {
    char buf[256];
    size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, buf);

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

    autoTuneTargetTemp = targetTemp;
    autoTuneSetpoint = targetTemp;
    autoTuneState = AUTOTUNE_HEATING;
    autoTuneStartTime = millis();
    autoTuneStepStartTime = millis();
    autoTuneCurrentStep = 0;
    autoTuneReachedSetpoint = false;
    autoTuneStabilizationStartTime = 0;
    autoTuneStepTimeout = AUTOTUNE_MAX_STEP_TIME;

    autoTuneOriginalKp = state.Kp;
    autoTuneOriginalKi = state.Ki;
    autoTuneOriginalKd = state.Kd;

    resetAutoTuneData();

    state.controlMode = MODE_MANUAL;
    state.beanSetpoint = autoTuneTargetTemp;
    state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;
    state.heaterEnabled = true;

    publishAutoTuneStatus();
}

void handleAutoTuneStopMsg(const char* payload, size_t len) {
    (void)payload; (void)len;
    DEBUG_PRINTLN(F("Stopping auto-tune"));

    // Preserve FAILED state if it was set before stop
    bool wasFailed = (autoTuneState == AUTOTUNE_FAILED);

    state.Kp = autoTuneOriginalKp;
    state.Ki = autoTuneOriginalKi;
    state.Kd = autoTuneOriginalKd;
    updatePIDTunings();

    if (!wasFailed) {
        autoTuneState = AUTOTUNE_IDLE;
    }
    state.controlMode = MODE_AUTO;
    state.heaterOutput = 0;
    state.heaterEnabled = false;

    publishAutoTuneStatus();
}

void handleAutoTuneApplyMsg(const char* payload, size_t len) {
    (void)payload; (void)len;
    DEBUG_PRINTLN(F("Applying auto-tune results"));

    if (autoTuneState != AUTOTUNE_COMPLETE) {
        DEBUG_PRINTLN(F("Auto-tune apply: No completed results to apply"));
        return;
    }

    if (autoTuneRecommendedKp <= 0 || autoTuneRecommendedKi < 0 || autoTuneRecommendedKd < 0) {
        DEBUG_PRINTLN(F("Auto-tune apply: Invalid PID parameters"));
        return;
    }

    DEBUG_PRINTF("Applying auto-tune results: Kp=%.2f, Ki=%.4f, Kd=%.2f\n",
                 autoTuneRecommendedKp, autoTuneRecommendedKi, autoTuneRecommendedKd);

    state.Kp = autoTuneRecommendedKp;
    state.Ki = autoTuneRecommendedKi;
    state.Kd = autoTuneRecommendedKd;

    updatePIDTunings();
    savePIDParameters();

    autoTuneState = AUTOTUNE_IDLE;
    state.controlMode = MODE_AUTO;

    DEBUG_PRINTLN(F("Auto-tune results applied and saved to NVS"));
    publishAutoTuneStatus();
}

// --- Auto-tune publish implementation (called from mqtt_handler.cpp) ---

void publishAutoTuneStatusImpl() {
    JsonDocument doc;

    doc["state"] = getAutoTuneStateString(autoTuneState);
    doc["message"] = getAutoTuneStateString(autoTuneState);

    if (autoTuneState == AUTOTUNE_RUNNING || autoTuneState == AUTOTUNE_ANALYZING) {
        float progress = ((float)autoTuneCurrentStep / AUTOTUNE_TOTAL_STEPS) * 100.0;
        if (autoTuneState == AUTOTUNE_ANALYZING) {
            progress = 90.0;
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

    mqttPublishAutoTuneStatus(atStatusBuf, len);
    DEBUG_PRINTF("MQTT: Published auto-tune status (%d bytes)\n", len);
}

void publishAutoTuneResultsImpl() {
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
    doc["duration"] = (millis() - autoTuneStartTime) / 1000;

    static char atResultsBuf[512];
    size_t len = serializeJson(doc, atResultsBuf, sizeof(atResultsBuf));

    mqttPublishAutoTuneResults(atResultsBuf, len);
    DEBUG_PRINTF("MQTT: Published auto-tune results (%d bytes)\n", len);
}

// --- Auto-tune state machine (unchanged Ziegler-Nichols) ---

void updateAutoTune() {
    if (autoTuneState != AUTOTUNE_RUNNING && autoTuneState != AUTOTUNE_ANALYZING && autoTuneState != AUTOTUNE_HEATING && autoTuneState != AUTOTUNE_STABILIZING) {
        return;
    }

    unsigned long now = millis();

    // Check for timeout
    if (now - autoTuneStartTime > AUTOTUNE_MAX_DURATION) {
        DEBUG_PRINTLN(F("Auto-tune timeout"));
        autoTuneState = AUTOTUNE_FAILED;
        publishAutoTuneStatus();
        handleAutoTuneStopMsg("", 0);
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
        double tempError = autoTuneSetpoint - state.beanTemperature;

        if (tempError > 50) {
            state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;
        } else if (tempError > 10) {
            double powerRatio = tempError / 50.0;
            double pct = autoTuneOutputBias + autoTuneOutputAmplitude * powerRatio;
            state.heaterOutput = pct * 255 / 100;
        } else if (tempError > 0) {
            double pct = autoTuneOutputBias;
            state.heaterOutput = pct * 255 / 100;
        } else {
            state.heaterOutput = 0;
        }
        if (state.heaterOutput > 255) state.heaterOutput = 255;
        if (state.heaterOutput < 0) state.heaterOutput = 0;
        DEBUG_PRINTF("Auto-tune HEATING: err=%.1fC, out=%.0f\n", tempError, state.heaterOutput);

        float absErr = abs(state.beanTemperature - autoTuneSetpoint);
        if (absErr <= AUTOTUNE_SETPOINT_TOLERANCE) {
            if (!autoTuneReachedSetpoint) {
                autoTuneReachedSetpoint = true;
                autoTuneStabilizationStartTime = now;
                autoTuneRorStableStartTime = 0;
                autoTuneFastTrackStartTime = 0;
                DEBUG_PRINTF("Auto-tune reached setpoint %.1f°C, starting stabilization\n", autoTuneSetpoint);
            }
            float absRor = fabs(getRateOfRise());
            if (absRor <= AUTOTUNE_STABILITY_ROR) {
                if (autoTuneRorStableStartTime == 0) autoTuneRorStableStartTime = now;
            } else {
                autoTuneRorStableStartTime = 0;
            }

            bool timeStable = (now - autoTuneStabilizationStartTime) >= AUTOTUNE_STABILIZATION_TIME;
            bool rorStable = (autoTuneRorStableStartTime != 0) && ((now - autoTuneRorStableStartTime) >= AUTOTUNE_STABILITY_ROR_TIME);

            if (timeStable || rorStable) {
                autoTuneState = AUTOTUNE_STABILIZING;
                autoTuneCurrentStep = 1;
                autoTuneStepStartTime = now;
                DEBUG_PRINTLN(F("Auto-tune stabilization complete, starting oscillation test"));
            }
        } else if (autoTuneReachedSetpoint) {
            if (absErr > (AUTOTUNE_SETPOINT_TOLERANCE + AUTOTUNE_STABILITY_HYST)) {
                autoTuneReachedSetpoint = false;
                autoTuneStabilizationStartTime = now;
                autoTuneRorStableStartTime = 0;
                DEBUG_PRINTLN(F("Auto-tune: left tolerance band, resetting stabilization timer"));
            }
        } else {
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

            float absRor2 = fabs(getRateOfRise());
            if (state.beanTemperature >= AUTOTUNE_EQUIL_MIN_TEMP && absRor2 <= AUTOTUNE_STABILITY_ROR) {
                if (autoTuneRorStableStartTime == 0) autoTuneRorStableStartTime = now;
            } else {
                autoTuneRorStableStartTime = 0;
            }
            if (autoTuneRorStableStartTime != 0 && (now - autoTuneRorStableStartTime) >= AUTOTUNE_FASTTRACK_ROR_TIME) {
                autoTuneSetpoint = state.beanTemperature;
                autoTuneState = AUTOTUNE_STABILIZING;
                autoTuneCurrentStep = 1;
                autoTuneStepStartTime = now;
                autoTuneReachedSetpoint = true;
                autoTuneStabilizationStartTime = now;
                DEBUG_PRINTF("Auto-tune equilibrium accepted at %.1f°C, proceeding to stabilizing.\n", autoTuneSetpoint);
            }
        }

        if (now - autoTuneStartTime >= AUTOTUNE_INITIAL_STEP_TIME) {
            DEBUG_PRINTLN(F("Auto-tune heating phase timeout: forcing progression using equilibrium"));
            autoTuneSetpoint = state.beanTemperature;
            autoTuneState = AUTOTUNE_STABILIZING;
            autoTuneCurrentStep = 1;
            autoTuneStepStartTime = now;
            autoTuneReachedSetpoint = true;
            autoTuneStabilizationStartTime = now;
        }
    }
    else if (autoTuneState == AUTOTUNE_STABILIZING) {
        state.heaterOutput = autoTuneOutputBias * 255 / 100;

        if (now - autoTuneStepStartTime >= AUTOTUNE_MIN_STEP_TIME) {
            autoTuneState = AUTOTUNE_RUNNING;
            autoTuneCurrentStep = 1;
            autoTuneStepStartTime = now;
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
        unsigned long stepDuration = now - autoTuneStepStartTime;
        unsigned long stepTimeout = (autoTuneCurrentStep <= 2) ? AUTOTUNE_INITIAL_STEP_TIME : autoTuneStepTimeout;
        float error = state.beanTemperature - autoTuneSetpoint;

        bool toggled = false;
        if (autoTuneOutputHigh) {
            if (error >= AUTOTUNE_RELAY_HYST || stepDuration >= stepTimeout) {
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
        if (calculateAutoTunePIDParameters()) {
            autoTuneUsedFallback = false;
            autoTuneState = AUTOTUNE_COMPLETE;
            DEBUG_PRINTLN(F("Auto-tune complete!"));
        } else {
            autoTuneRecommendedKp = state.Kp;
            autoTuneRecommendedKi = state.Ki;
            autoTuneRecommendedKd = state.Kd;
            autoTuneUsedFallback = true;
            autoTuneState = AUTOTUNE_COMPLETE;
            DEBUG_PRINTLN(F("Auto-tune: fallback PID used due to insufficient oscillation data"));
        }
        publishAutoTuneResults();
        state.heaterEnabled = false;
        state.controlMode = MODE_AUTO;
        publishAutoTuneStatus();
    }
}

bool calculateAutoTunePIDParameters() {
    float avgPeriod = 0;
    float avgAmplitude = 0;
    bool computed = false;

    if (autoTunePeakCount >= 2 && autoTuneValleyCount >= 2) {
        float totalPeriod = 0; int periodCount = 0;
        for (int i = 1; i < autoTunePeakCount; i++) {
            float period = autoTunePeaks[i].time - autoTunePeaks[i-1].time;
            if (period > 0) { totalPeriod += period; periodCount++; }
        }
        if (periodCount > 0) avgPeriod = totalPeriod / periodCount;

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

    if (!computed) {
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
            float totalHalf = 0; int halfCount = 0;
            for (int i = 1; i < crossCount; i++) {
                float dt = (crossings[i] - crossings[i-1]) / 1000.0f;
                if (dt > 0) { totalHalf += dt; halfCount++; }
            }
            if (halfCount > 0) avgPeriod = 2.0f * (totalHalf / halfCount);
        }
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

    float outputSwing = autoTuneOutputAmplitude * 2;
    float Ku = (outputSwing * 4) / (avgAmplitude * 3.14159f);

    // Ziegler-Nichols PID rules
    autoTuneRecommendedKp = 0.6f * Ku;
    autoTuneRecommendedKi = (2.0f * autoTuneRecommendedKp) / avgPeriod;
    autoTuneRecommendedKd = (autoTuneRecommendedKp * avgPeriod) / 8.0f;

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
