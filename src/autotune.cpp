#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "debug.h"
#include "roaster_state.h"
#include "autotune.h"
#include "mqtt_handler.h"
#include "heater_control.h"
#include "temperature.h"

// --- Static file-scope state variables ---

typedef struct {
    float time;
    float temperature;
} PeakValley;

static AutoTuneState autoTuneState = AUTOTUNE_IDLE;
static double autoTuneTargetTemp = 200.0;
static double autoTuneSetpoint = 200.0;
static unsigned long autoTuneStartTime = 0;
static unsigned long autoTuneStepStartTime = 0;
static unsigned long autoTuneStabilizationStartTime = 0;
static int autoTuneCurrentStep = 0;
static float autoTuneOutputBias = AUTOTUNE_OUTPUT_BIAS;
static float autoTuneOutputAmplitude = AUTOTUNE_OUTPUT_AMPLITUDE;
static float autoTuneRelayHyst = AUTOTUNE_RELAY_HYST;
static double autoTuneOriginalKp = 0.0, autoTuneOriginalKi = 0.0, autoTuneOriginalKd = 0.0;
static double autoTuneRecommendedKp = 0.0, autoTuneRecommendedKi = 0.0, autoTuneRecommendedKd = 0.0;
static bool autoTuneUsedFallback = false;
static bool autoTuneReachedSetpoint = false;
static float autoTuneStepTimeout = AUTOTUNE_MAX_STEP_TIME;
static unsigned long autoTuneRorStableStartTime = 0;
static bool autoTuneOutputHigh = false;

// Tuning method selection
static const char* autoTuneTuningMethod = "tyreus_luyben";

// EMA noise filter for auto-tune temperature readings
static float autoTuneFilteredTemp = 0.0f;
static bool autoTuneFilterInitialized = false;

// Running extrema tracking for accurate peak/valley recording
static float autoTuneRunningMax = -1e9f;
static float autoTuneRunningMin = 1e9f;

// Auto-tune data storage
static PeakValley autoTunePeaks[10];
static PeakValley autoTuneValleys[10];
static int autoTunePeakCount = 0;
static int autoTuneValleyCount = 0;

// Auto-tune temperature history for oscillation detection
static float autoTuneTempHistory[AUTOTUNE_TEMP_HISTORY_SIZE];
static unsigned long autoTuneTempTimeHistory[AUTOTUNE_TEMP_HISTORY_SIZE];
static int autoTuneTempHistoryIndex = 0;
static int autoTuneTempHistoryCount = 0;

// Heating/cooling half-period tracking for asymmetry correction
static float autoTuneHeatingTime = 0.0f;
static float autoTuneCoolingTime = 0.0f;
static int autoTuneHeatingCount = 0;
static int autoTuneCoolingCount = 0;

// Auto-tune mode: "relay" or "step_response"
static const char* autoTuneMode = "relay";

// Step response data buffer
typedef struct {
    unsigned long time_ms;
    float temperature;
} StepDataPoint;
static StepDataPoint autoTuneStepData[AUTOTUNE_STEP_DATA_SIZE];
static int autoTuneStepDataCount = 0;
static float autoTuneStepBaselineTemp = 0.0f;
static float autoTuneStepOutputPct = 0.0f;
static unsigned long autoTuneStepSettleStart = 0;
static float autoTuneStepAggressiveness = 1.0f;

// FOPDT model parameters (for step response mode)
static float autoTuneFOPDT_K = 0.0f;
static float autoTuneFOPDT_tau = 0.0f;
static float autoTuneFOPDT_theta = 0.0f;

// Quality metrics for reporting
static float autoTuneConsistencyPct = 0.0f;
static float autoTuneAsymmetryRatio = 0.0f;
static float autoTuneHysteresisCorrection = 0.0f;

// Status publishing interval
static unsigned long lastAutoTuneStatusPublish = 0;
static const unsigned long autoTuneStatusPublishInterval = 2000;

// Forward declarations
static bool calculateAutoTunePIDParameters();
static void resetAutoTuneData();
static void publishAutoTuneStatusMsg();
static void publishAutoTuneResultsMsg();

void initAutoTune() {
    autoTuneState = AUTOTUNE_IDLE;
    resetAutoTuneData();
}

bool isAutoTuneActive() {
    return autoTuneState != AUTOTUNE_IDLE && autoTuneState != AUTOTUNE_COMPLETE && autoTuneState != AUTOTUNE_FAILED;
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
        case AUTOTUNE_STEP_BASELINE: return "step_baseline";
        case AUTOTUNE_STEP_UP: return "step_up";
        case AUTOTUNE_STEP_SETTLE: return "step_settle";
        case AUTOTUNE_STEP_ANALYZE: return "step_analyze";
        default: return "unknown";
    }
}

// --- MQTT message handlers ---

void handleAutoTuneStart(const char* payload, size_t len) {
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

    // Parse optional mode: 'relay' (default) or 'step_response'
    const char* modeStr = doc["mode"] | "relay";
    const char* selectedMode = "relay";
    if (strcmp(modeStr, "step_response") == 0) {
        selectedMode = "step_response";
    }

    // Parse optional aggressiveness for SIMC (step response mode)
    float aggressiveness = doc["aggressiveness"] | 1.0f;
    if (aggressiveness < 0.1f || aggressiveness > 2.0f) {
        DEBUG_PRINTF("Auto-tune: invalid aggressiveness %.2f, using default 1.0\n", aggressiveness);
        aggressiveness = 1.0f;
    }

    // Parse optional tuning method (store to local, assign after reset)
    const char* method = doc["tuning_method"] | "tyreus_luyben";
    const char* selectedMethod = "tyreus_luyben";
    if (strcmp(method, "zn_classic") == 0) {
        selectedMethod = "zn_classic";
    } else if (strcmp(method, "zn_some_overshoot") == 0) {
        selectedMethod = "zn_some_overshoot";
    } else if (strcmp(method, "zn_no_overshoot") == 0) {
        selectedMethod = "zn_no_overshoot";
    } else if (strcmp(method, "tyreus_luyben") == 0) {
        selectedMethod = "tyreus_luyben";
    } else {
        DEBUG_PRINTF("Auto-tune: unrecognized tuning_method '%s', defaulting to tyreus_luyben\n", method);
    }
    // Parse optional relay parameters with validation
    float bias = doc["bias"] | (float)AUTOTUNE_OUTPUT_BIAS;
    float amplitude = doc["amplitude"] | (float)AUTOTUNE_OUTPUT_AMPLITUDE;
    float hysteresis = doc["hysteresis"] | (float)AUTOTUNE_RELAY_HYST;

    if (bias < 10.0f || bias > 90.0f) {
        DEBUG_PRINTF("Auto-tune: invalid bias %.1f, using default %d\n", bias, AUTOTUNE_OUTPUT_BIAS);
        bias = AUTOTUNE_OUTPUT_BIAS;
    }
    if (amplitude < 5.0f || amplitude > 45.0f) {
        DEBUG_PRINTF("Auto-tune: invalid amplitude %.1f, using default %d\n", amplitude, AUTOTUNE_OUTPUT_AMPLITUDE);
        amplitude = AUTOTUNE_OUTPUT_AMPLITUDE;
    }
    if (hysteresis < 0.1f || hysteresis > 5.0f) {
        DEBUG_PRINTF("Auto-tune: invalid hysteresis %.1f, using default %.1f\n", hysteresis, (float)AUTOTUNE_RELAY_HYST);
        hysteresis = AUTOTUNE_RELAY_HYST;
    }
    if (bias + amplitude > 100.0f) {
        DEBUG_PRINTF("Auto-tune: bias+amplitude > 100 (%.1f+%.1f), using defaults\n", bias, amplitude);
        bias = AUTOTUNE_OUTPUT_BIAS;
        amplitude = AUTOTUNE_OUTPUT_AMPLITUDE;
    }
    if (bias - amplitude < 0.0f) {
        DEBUG_PRINTF("Auto-tune: bias-amplitude < 0 (%.1f-%.1f), using defaults\n", bias, amplitude);
        bias = AUTOTUNE_OUTPUT_BIAS;
        amplitude = AUTOTUNE_OUTPUT_AMPLITUDE;
    }

    DEBUG_PRINTF("Starting auto-tune: target=%.1f°C, mode=%s, method=%s, bias=%.0f%%, amp=%.0f%%, hyst=%.1f°C\n",
                 targetTemp, selectedMode, selectedMethod, bias, amplitude, hysteresis);

    autoTuneTargetTemp = targetTemp;
    autoTuneSetpoint = targetTemp;
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

    // Set configurable params after reset (reset restores defaults)
    autoTuneOutputBias = bias;
    autoTuneOutputAmplitude = amplitude;
    autoTuneRelayHyst = hysteresis;
    autoTuneTuningMethod = selectedMethod;
    autoTuneMode = selectedMode;
    autoTuneStepAggressiveness = aggressiveness;

    state.controlMode = MODE_MANUAL;
    state.beanSetpoint = autoTuneTargetTemp;
    state.heaterEnabled = true;

    // Route to correct initial state based on mode
    if (strcmp(autoTuneMode, "step_response") == 0) {
        autoTuneState = AUTOTUNE_STEP_BASELINE;
        state.heaterOutput = autoTuneOutputBias * 255 / 100;
        DEBUG_PRINTLN(F("Auto-tune starting step response mode: STEP_BASELINE"));
    } else {
        autoTuneState = AUTOTUNE_HEATING;
        state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;
    }

    publishAutoTuneStatusMsg();
}

void handleAutoTuneStop(const char* payload, size_t len) {
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

    publishAutoTuneStatusMsg();
}

void handleAutoTuneApply(const char* payload, size_t len) {
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
    publishAutoTuneStatusMsg();
}

// --- Auto-tune publish helpers ---

static void publishAutoTuneStatusMsg() {
    JsonDocument doc;

    doc["state"] = getAutoTuneStateString(autoTuneState);
    doc["message"] = getAutoTuneStateString(autoTuneState);

    doc["mode"] = autoTuneMode;
    doc["tuning_method"] = autoTuneTuningMethod;

    if (autoTuneState == AUTOTUNE_RUNNING || autoTuneState == AUTOTUNE_ANALYZING) {
        float progress = ((float)autoTuneCurrentStep / AUTOTUNE_TOTAL_STEPS) * 100.0;
        if (autoTuneState == AUTOTUNE_ANALYZING) progress = 90.0f;
        doc["progress"] = progress;
        doc["current_step"] = autoTuneCurrentStep;
        doc["total_steps"] = AUTOTUNE_TOTAL_STEPS;
    } else if (autoTuneState == AUTOTUNE_STEP_BASELINE) {
        float elapsed = (millis() - autoTuneStepStartTime) / (float)AUTOTUNE_STEP_BASELINE_TIME;
        doc["progress"] = elapsed * 25.0f;
    } else if (autoTuneState == AUTOTUNE_STEP_UP) {
        doc["progress"] = 25.0f + 50.0f * (autoTuneStepDataCount / (float)AUTOTUNE_STEP_DATA_SIZE);
    } else if (autoTuneState == AUTOTUNE_STEP_SETTLE) {
        doc["progress"] = 80.0f;
    } else if (autoTuneState == AUTOTUNE_STEP_ANALYZE) {
        doc["progress"] = 90.0f;
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

static void publishAutoTuneResultsMsg() {
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
    // Periodic status publishing for active auto-tune
    if (autoTuneState != AUTOTUNE_IDLE && mqttIsConnected()) {
        if (millis() - lastAutoTuneStatusPublish >= autoTuneStatusPublishInterval) {
            lastAutoTuneStatusPublish = millis();
            publishAutoTuneStatusMsg();
        }
    }

    if (autoTuneState != AUTOTUNE_RUNNING && autoTuneState != AUTOTUNE_ANALYZING &&
        autoTuneState != AUTOTUNE_HEATING && autoTuneState != AUTOTUNE_STABILIZING &&
        autoTuneState != AUTOTUNE_STEP_BASELINE && autoTuneState != AUTOTUNE_STEP_UP &&
        autoTuneState != AUTOTUNE_STEP_SETTLE && autoTuneState != AUTOTUNE_STEP_ANALYZE) {
        return;
    }

    unsigned long now = millis();

    // Compute EMA-filtered temperature for relay decisions and peak tracking
    if (!autoTuneFilterInitialized) {
        autoTuneFilteredTemp = state.beanTemperature;
        autoTuneFilterInitialized = true;
    } else {
        autoTuneFilteredTemp = AUTOTUNE_EMA_ALPHA * state.beanTemperature + (1.0f - AUTOTUNE_EMA_ALPHA) * autoTuneFilteredTemp;
    }

    // Check for timeout
    if (now - autoTuneStartTime > AUTOTUNE_MAX_DURATION) {
        DEBUG_PRINTLN(F("Auto-tune timeout"));
        autoTuneState = AUTOTUNE_FAILED;
        publishAutoTuneStatusMsg();
        handleAutoTuneStop("", 0);
        return;
    }

    // Add filtered temperature to history
    if (autoTuneTempHistoryCount < AUTOTUNE_TEMP_HISTORY_SIZE) {
        autoTuneTempHistory[autoTuneTempHistoryIndex] = autoTuneFilteredTemp;
        autoTuneTempTimeHistory[autoTuneTempHistoryIndex] = now;
        autoTuneTempHistoryIndex = (autoTuneTempHistoryIndex + 1) % AUTOTUNE_TEMP_HISTORY_SIZE;
        autoTuneTempHistoryCount++;
    } else {
        autoTuneTempHistory[autoTuneTempHistoryIndex] = autoTuneFilteredTemp;
        autoTuneTempTimeHistory[autoTuneTempHistoryIndex] = now;
        autoTuneTempHistoryIndex = (autoTuneTempHistoryIndex + 1) % AUTOTUNE_TEMP_HISTORY_SIZE;
    }

    if (autoTuneState == AUTOTUNE_HEATING) {
        // Heater output: full power far from setpoint, proportional approaching, bias-only near
        double tempError = autoTuneSetpoint - state.beanTemperature;
        if (tempError > 50) {
            state.heaterOutput = (autoTuneOutputBias + autoTuneOutputAmplitude) * 255 / 100;
        } else if (tempError > 10) {
            double powerRatio = tempError / 50.0;
            double pct = autoTuneOutputBias + autoTuneOutputAmplitude * powerRatio;
            state.heaterOutput = pct * 255 / 100;
        } else if (tempError > 0) {
            state.heaterOutput = autoTuneOutputBias * 255 / 100;
        } else {
            state.heaterOutput = 0;
        }
        if (state.heaterOutput > 255) state.heaterOutput = 255;
        if (state.heaterOutput < 0) state.heaterOutput = 0;
        DEBUG_PRINTF("Auto-tune HEATING: err=%.1fC, out=%.0f\n", tempError, state.heaterOutput);

        // Use filtered temperature for setpoint comparisons
        float absErr = fabs(autoTuneFilteredTemp - autoTuneSetpoint);
        float absRor = fabs(getRateOfRise());

        // --- Path 1 (Normal): Within tolerance AND low RoR for STABILITY_ROR_TIME ---
        if (absErr <= AUTOTUNE_SETPOINT_TOLERANCE) {
            if (!autoTuneReachedSetpoint) {
                autoTuneReachedSetpoint = true;
                autoTuneRorStableStartTime = 0;
                DEBUG_PRINTF("Auto-tune HEATING: reached setpoint %.1f°C\n", autoTuneSetpoint);
            }
            if (absRor <= AUTOTUNE_STABILITY_ROR) {
                if (autoTuneRorStableStartTime == 0) autoTuneRorStableStartTime = now;
            } else {
                autoTuneRorStableStartTime = 0;
            }
            if (autoTuneRorStableStartTime != 0 && (now - autoTuneRorStableStartTime) >= AUTOTUNE_STABILITY_ROR_TIME) {
                autoTuneState = AUTOTUNE_STABILIZING;
                autoTuneCurrentStep = 1;
                autoTuneStepStartTime = now;
                DEBUG_PRINTLN(F("Auto-tune HEATING → STABILIZING via Path 1 (normal: at setpoint with low RoR)"));
            }
        } else {
            autoTuneReachedSetpoint = false;
            autoTuneRorStableStartTime = 0;
        }

        // --- Path 2 (Equilibrium): >= 75% of target AND low RoR for STABILIZATION_TIME ---
        if (autoTuneState == AUTOTUNE_HEATING && !autoTuneReachedSetpoint) {
            float equilMinTemp = AUTOTUNE_EQUIL_MIN_PCT * autoTuneTargetTemp;
            if (autoTuneFilteredTemp >= equilMinTemp && absRor <= AUTOTUNE_STABILITY_ROR) {
                if (autoTuneStabilizationStartTime == 0) autoTuneStabilizationStartTime = now;
            } else {
                autoTuneStabilizationStartTime = 0;
            }
            if (autoTuneStabilizationStartTime != 0 && (now - autoTuneStabilizationStartTime) >= AUTOTUNE_STABILIZATION_TIME) {
                autoTuneSetpoint = autoTuneFilteredTemp;
                autoTuneState = AUTOTUNE_STABILIZING;
                autoTuneCurrentStep = 1;
                autoTuneStepStartTime = now;
                DEBUG_PRINTF("Auto-tune HEATING → STABILIZING via Path 2 (equilibrium at %.1f°C)\n", autoTuneSetpoint);
            }
        }

        // --- Path 3 (Timeout): INITIAL_STEP_TIME elapsed → FAILED ---
        if (autoTuneState == AUTOTUNE_HEATING && (now - autoTuneStartTime >= AUTOTUNE_INITIAL_STEP_TIME)) {
            DEBUG_PRINTLN(F("Auto-tune HEATING → FAILED via Path 3 (timeout)"));
            autoTuneState = AUTOTUNE_FAILED;
            publishAutoTuneStatusMsg();
            handleAutoTuneStop("", 0);
            return;
        }
    }
    else if (autoTuneState == AUTOTUNE_STABILIZING) {
        state.heaterOutput = autoTuneOutputBias * 255 / 100;

        if (now - autoTuneStepStartTime >= AUTOTUNE_MIN_STEP_TIME) {
            autoTuneState = AUTOTUNE_RUNNING;
            autoTuneCurrentStep = 1;
            autoTuneStepStartTime = now;
            // Initialize running extrema to current filtered temperature
            autoTuneRunningMax = autoTuneFilteredTemp;
            autoTuneRunningMin = autoTuneFilteredTemp;
            if (autoTuneFilteredTemp <= autoTuneSetpoint) {
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
        // Use filtered temperature for relay decisions
        float error = autoTuneFilteredTemp - autoTuneSetpoint;

        // Track running extrema between relay toggles
        if (autoTuneOutputHigh) {
            if (autoTuneFilteredTemp > autoTuneRunningMax) autoTuneRunningMax = autoTuneFilteredTemp;
        } else {
            if (autoTuneFilteredTemp < autoTuneRunningMin) autoTuneRunningMin = autoTuneFilteredTemp;
        }

        bool toggled = false;
        float stepDurationSec = stepDuration / 1000.0f;
        if (autoTuneOutputHigh) {
            if (error >= autoTuneRelayHyst || stepDuration >= stepTimeout) {
                // Record actual peak (running max), not crossing temperature
                if (autoTunePeakCount < 10) {
                    autoTunePeaks[autoTunePeakCount].temperature = autoTuneRunningMax;
                    autoTunePeaks[autoTunePeakCount].time = now / 1000.0f;
                    autoTunePeakCount++;
                }
                autoTuneRunningMax = -1e9f;  // Reset for next cycle
                // Track heating half-period (output was high during this step)
                autoTuneHeatingTime += stepDurationSec;
                autoTuneHeatingCount++;
                autoTuneOutputHigh = false;
                toggled = true;
                state.heaterOutput = (autoTuneOutputBias - autoTuneOutputAmplitude) * 255 / 100;
            }
        } else {
            if (error <= -autoTuneRelayHyst || stepDuration >= stepTimeout) {
                // Record actual valley (running min), not crossing temperature
                if (autoTuneValleyCount < 10) {
                    autoTuneValleys[autoTuneValleyCount].temperature = autoTuneRunningMin;
                    autoTuneValleys[autoTuneValleyCount].time = now / 1000.0f;
                    autoTuneValleyCount++;
                }
                autoTuneRunningMin = 1e9f;  // Reset for next cycle
                // Track cooling half-period (output was low during this step)
                autoTuneCoolingTime += stepDurationSec;
                autoTuneCoolingCount++;
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
        publishAutoTuneResultsMsg();
        state.heaterEnabled = false;
        state.controlMode = MODE_AUTO;
        publishAutoTuneStatusMsg();
    }
    // --- Step response state machine ---
    else if (autoTuneState == AUTOTUNE_STEP_BASELINE) {
        // Run heater at bias output, measure baseline temperature
        state.heaterOutput = autoTuneOutputBias * 255 / 100;
        if (state.heaterOutput > 255) state.heaterOutput = 255;

        if (now - autoTuneStepStartTime >= AUTOTUNE_STEP_BASELINE_TIME) {
            autoTuneStepBaselineTemp = autoTuneFilteredTemp;
            autoTuneStepOutputPct = autoTuneOutputBias;
            autoTuneStepDataCount = 0;
            autoTuneStepStartTime = now;
            autoTuneState = AUTOTUNE_STEP_UP;
            DEBUG_PRINTF("Auto-tune STEP_BASELINE complete: baseline=%.1f°C, stepping up to %.0f%%\n",
                         autoTuneStepBaselineTemp, autoTuneOutputBias + autoTuneOutputAmplitude);
        }
    }
    else if (autoTuneState == AUTOTUNE_STEP_UP) {
        // Increase heater output, record data every ~1 second
        float stepOutput = autoTuneOutputBias + autoTuneOutputAmplitude;
        state.heaterOutput = stepOutput * 255 / 100;
        if (state.heaterOutput > 255) state.heaterOutput = 255;

        // Record data point (~1Hz, same rate as temp reads)
        if (autoTuneStepDataCount < AUTOTUNE_STEP_DATA_SIZE) {
            autoTuneStepData[autoTuneStepDataCount].time_ms = now - autoTuneStepStartTime;
            autoTuneStepData[autoTuneStepDataCount].temperature = autoTuneFilteredTemp;
            autoTuneStepDataCount++;
        }

        // Check if temperature has settled (RoR low enough)
        float absRor = fabs(getRateOfRise());
        if (absRor < AUTOTUNE_STEP_SETTLE_ROR) {
            if (autoTuneStepSettleStart == 0) autoTuneStepSettleStart = now;
        } else {
            autoTuneStepSettleStart = 0;
        }

        // Transition to SETTLE if RoR has been low enough, or buffer is filling up
        if ((autoTuneStepSettleStart != 0 && (now - autoTuneStepSettleStart) >= AUTOTUNE_STEP_SETTLE_TIME) ||
            autoTuneStepDataCount >= AUTOTUNE_STEP_DATA_SIZE - 10) {
            autoTuneState = AUTOTUNE_STEP_SETTLE;
            autoTuneStepSettleStart = 0;
            DEBUG_PRINTF("Auto-tune STEP_UP → STEP_SETTLE: %d data points, temp=%.1f°C\n",
                         autoTuneStepDataCount, autoTuneFilteredTemp);
        }
    }
    else if (autoTuneState == AUTOTUNE_STEP_SETTLE) {
        // Continue recording, wait for full settling
        float stepOutput = autoTuneOutputBias + autoTuneOutputAmplitude;
        state.heaterOutput = stepOutput * 255 / 100;
        if (state.heaterOutput > 255) state.heaterOutput = 255;

        if (autoTuneStepDataCount < AUTOTUNE_STEP_DATA_SIZE) {
            autoTuneStepData[autoTuneStepDataCount].time_ms = now - autoTuneStepStartTime;
            autoTuneStepData[autoTuneStepDataCount].temperature = autoTuneFilteredTemp;
            autoTuneStepDataCount++;
        }

        float absRor = fabs(getRateOfRise());
        if (absRor < AUTOTUNE_STEP_SETTLE_ROR) {
            if (autoTuneStepSettleStart == 0) autoTuneStepSettleStart = now;
        } else {
            autoTuneStepSettleStart = 0;
        }

        if ((autoTuneStepSettleStart != 0 && (now - autoTuneStepSettleStart) >= AUTOTUNE_STEP_SETTLE_TIME) ||
            autoTuneStepDataCount >= AUTOTUNE_STEP_DATA_SIZE) {
            autoTuneState = AUTOTUNE_STEP_ANALYZE;
            DEBUG_PRINTF("Auto-tune STEP_SETTLE → STEP_ANALYZE: %d data points\n", autoTuneStepDataCount);
        }
    }
    else if (autoTuneState == AUTOTUNE_STEP_ANALYZE) {
        // Placeholder: actual FOPDT computation implemented in US-009
        autoTuneState = AUTOTUNE_ANALYZING;
        DEBUG_PRINTLN(F("Auto-tune STEP_ANALYZE → ANALYZING (FOPDT computation)"));
    }
}

static bool calculateAutoTunePIDParameters() {
    float avgPeriod = 0;
    float avgAmplitude = 0;
    bool computed = false;
    autoTuneConsistencyPct = 0.0f;

    // Primary analysis: use peaks/valleys, discarding first 2 as transient
    const int transientSkip = 2;
    int usablePeaks = autoTunePeakCount - transientSkip;
    int usableValleys = autoTuneValleyCount - transientSkip;

    if (usablePeaks >= 2 && usableValleys >= 2) {
        // Compute average period from post-transient peaks
        float totalPeriod = 0; int periodCount = 0;
        for (int i = transientSkip + 1; i < autoTunePeakCount; i++) {
            float period = autoTunePeaks[i].time - autoTunePeaks[i-1].time;
            if (period > 0) { totalPeriod += period; periodCount++; }
        }
        if (periodCount > 0) avgPeriod = totalPeriod / periodCount;

        // Compute amplitudes from post-transient peak-valley pairs
        float totalAmplitude = 0; int amplitudeCount = 0;
        float amplitudes[10] = {0};
        for (int i = transientSkip; i < autoTunePeakCount; i++) {
            for (int j = transientSkip; j < autoTuneValleyCount; j++) {
                if (fabs(autoTunePeaks[i].time - autoTuneValleys[j].time) < (avgPeriod / 2 + 2)) {
                    float amplitude = fabs(autoTunePeaks[i].temperature - autoTuneValleys[j].temperature) / 2.0f;
                    if (amplitudeCount < 10) amplitudes[amplitudeCount] = amplitude;
                    totalAmplitude += amplitude;
                    amplitudeCount++;
                    break;
                }
            }
        }
        if (periodCount > 0 && amplitudeCount > 0) {
            avgAmplitude = totalAmplitude / amplitudeCount;

            // Consistency validation: check last 3 amplitudes within tolerance of their mean
            if (amplitudeCount >= 3) {
                float last3[3];
                for (int i = 0; i < 3; i++) last3[i] = amplitudes[amplitudeCount - 3 + i];
                float mean3 = (last3[0] + last3[1] + last3[2]) / 3.0f;

                float maxDev = 0.0f;
                for (int i = 0; i < 3; i++) {
                    float dev = fabs(last3[i] - mean3) / mean3;
                    if (dev > maxDev) maxDev = dev;
                }
                autoTuneConsistencyPct = 1.0f - maxDev;

                if (maxDev > AUTOTUNE_CONSISTENCY_TOLERANCE) {
                    DEBUG_PRINTF("Auto-tune consistency check FAILED: amplitudes=[%.2f, %.2f, %.2f], mean=%.2f, maxDev=%.1f%%\n",
                                 last3[0], last3[1], last3[2], mean3, maxDev * 100.0f);
                    computed = false;
                } else {
                    DEBUG_PRINTF("Auto-tune consistency OK: %.0f%% (amplitudes=[%.2f, %.2f, %.2f])\n",
                                 autoTuneConsistencyPct * 100.0f, last3[0], last3[1], last3[2]);
                    computed = true;
                }
            } else {
                computed = true;
            }
        }
    }

    // Fallback analysis with tightened thresholds
    if (!computed) {
        const float ref = autoTuneSetpoint;
        int crossings[10]; int crossCount = 0;
        for (int i = 1; i < autoTuneTempHistoryCount && crossCount < 10; i++) {
            int idxPrev = (autoTuneTempHistoryIndex - i - 1 + AUTOTUNE_TEMP_HISTORY_SIZE) % AUTOTUNE_TEMP_HISTORY_SIZE;
            int idxCur = (autoTuneTempHistoryIndex - i + AUTOTUNE_TEMP_HISTORY_SIZE) % AUTOTUNE_TEMP_HISTORY_SIZE;
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
            int idx = (autoTuneTempHistoryIndex - i - 1 + AUTOTUNE_TEMP_HISTORY_SIZE) % AUTOTUNE_TEMP_HISTORY_SIZE;
            float v = autoTuneTempHistory[idx];
            if (v < tmin) tmin = v; if (v > tmax) tmax = v;
        }
        float range = (tmax > tmin) ? (tmax - tmin) : 0;
        avgAmplitude = range / 2.0f;
        computed = (avgPeriod > AUTOTUNE_FALLBACK_MIN_PERIOD && avgAmplitude > AUTOTUNE_FALLBACK_MIN_AMPLITUDE);
        if (computed) {
            DEBUG_PRINTF("Auto-tune fallback analysis: period=%.1fs, amplitude=%.2f°C\n", avgPeriod, avgAmplitude);
        }
    }

    if (!computed) {
        DEBUG_PRINTLN(F("Insufficient oscillation data for PID calculation"));
        return false;
    }

    float outputSwing = autoTuneOutputAmplitude * 2;
    float Ku = (outputSwing * 4) / (avgAmplitude * 3.14159f);
    float rawKu = Ku;

    // Asymmetry correction: account for different heating/cooling dynamics
    autoTuneAsymmetryRatio = 0.0f;
    if (autoTuneHeatingCount > 0 && autoTuneCoolingCount > 0) {
        float avgHeatingHalf = autoTuneHeatingTime / autoTuneHeatingCount;
        float avgCoolingHalf = autoTuneCoolingTime / autoTuneCoolingCount;
        autoTuneAsymmetryRatio = avgHeatingHalf / avgCoolingHalf;

        float d_h = autoTuneOutputBias + autoTuneOutputAmplitude;
        float d_c = autoTuneOutputBias - autoTuneOutputAmplitude;
        float d = (float)autoTuneOutputAmplitude;
        if (d_c > 0 && avgAmplitude > 0) {
            Ku = (4.0f * d) / (3.14159f * avgAmplitude) * sqrtf(1.0f + (d_h / d_c) * (d_h / d_c)) / sqrtf(2.0f);
        }
        DEBUG_PRINTF("Auto-tune asymmetry: ratio=%.2f, heating_half=%.1fs, cooling_half=%.1fs\n",
                     autoTuneAsymmetryRatio, avgHeatingHalf, avgCoolingHalf);
    }

    // Hysteresis correction: compensate for relay hysteresis phase shift
    autoTuneHysteresisCorrection = 1.0f;
    float hyst = autoTuneRelayHyst;
    if (avgAmplitude > hyst) {
        float ratio = hyst / avgAmplitude;
        autoTuneHysteresisCorrection = 1.0f / sqrtf(1.0f - ratio * ratio);
        Ku *= autoTuneHysteresisCorrection;
        DEBUG_PRINTF("Auto-tune hysteresis correction: factor=%.3f\n", autoTuneHysteresisCorrection);
    } else {
        DEBUG_PRINTLN(F("Auto-tune: amplitude <= hysteresis, skipping hysteresis correction"));
    }

    DEBUG_PRINTF("Auto-tune Ku: raw=%.2f, asymmetry_factor=%.3f, hyst_factor=%.3f, corrected=%.2f\n",
                 rawKu, (autoTuneAsymmetryRatio > 0) ? Ku / (rawKu * autoTuneHysteresisCorrection) : 1.0f,
                 autoTuneHysteresisCorrection, Ku);

    // PID tuning method dispatch
    DEBUG_PRINTF("Auto-tune using tuning method: %s\n", autoTuneTuningMethod);
    if (strcmp(autoTuneTuningMethod, "zn_classic") == 0) {
        autoTuneRecommendedKp = 0.6f * Ku;
        autoTuneRecommendedKi = (2.0f * autoTuneRecommendedKp) / avgPeriod;
        autoTuneRecommendedKd = (autoTuneRecommendedKp * avgPeriod) / 8.0f;
    } else if (strcmp(autoTuneTuningMethod, "zn_some_overshoot") == 0) {
        autoTuneRecommendedKp = 0.33f * Ku;
        autoTuneRecommendedKi = (2.0f * autoTuneRecommendedKp) / avgPeriod;
        autoTuneRecommendedKd = (autoTuneRecommendedKp * avgPeriod) / 3.0f;
    } else if (strcmp(autoTuneTuningMethod, "zn_no_overshoot") == 0) {
        autoTuneRecommendedKp = 0.2f * Ku;
        autoTuneRecommendedKi = (2.0f * autoTuneRecommendedKp) / avgPeriod;
        autoTuneRecommendedKd = (autoTuneRecommendedKp * avgPeriod) / 3.0f;
    } else {
        // Default: Tyreus-Luyben (conservative, ~5% overshoot)
        autoTuneRecommendedKp = Ku / 3.2f;
        autoTuneRecommendedKi = autoTuneRecommendedKp / (2.2f * avgPeriod);
        autoTuneRecommendedKd = (autoTuneRecommendedKp * avgPeriod) / 6.3f;
    }

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

static void resetAutoTuneData() {
    autoTunePeakCount = 0;
    autoTuneValleyCount = 0;
    autoTuneTempHistoryCount = 0;
    autoTuneTempHistoryIndex = 0;
    autoTuneRecommendedKp = 0;
    autoTuneRecommendedKi = 0;
    autoTuneRecommendedKd = 0;
    autoTuneFilteredTemp = 0.0f;
    autoTuneFilterInitialized = false;
    autoTuneRunningMax = -1e9f;
    autoTuneRunningMin = 1e9f;
    autoTuneHeatingTime = 0.0f;
    autoTuneCoolingTime = 0.0f;
    autoTuneHeatingCount = 0;
    autoTuneCoolingCount = 0;
    autoTuneConsistencyPct = 0.0f;
    autoTuneAsymmetryRatio = 0.0f;
    autoTuneHysteresisCorrection = 0.0f;
    autoTuneTuningMethod = "tyreus_luyben";
    autoTuneOutputBias = AUTOTUNE_OUTPUT_BIAS;
    autoTuneOutputAmplitude = AUTOTUNE_OUTPUT_AMPLITUDE;
    autoTuneRelayHyst = AUTOTUNE_RELAY_HYST;
    autoTuneMode = "relay";
    autoTuneStepDataCount = 0;
    autoTuneStepBaselineTemp = 0.0f;
    autoTuneStepOutputPct = 0.0f;
    autoTuneStepSettleStart = 0;
    autoTuneStepAggressiveness = 1.0f;
    autoTuneFOPDT_K = 0.0f;
    autoTuneFOPDT_tau = 0.0f;
    autoTuneFOPDT_theta = 0.0f;
}
