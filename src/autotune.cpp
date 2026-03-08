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
static double autoTuneOriginalKp = 0.0, autoTuneOriginalKi = 0.0, autoTuneOriginalKd = 0.0;
static double autoTuneRecommendedKp = 0.0, autoTuneRecommendedKi = 0.0, autoTuneRecommendedKd = 0.0;
static bool autoTuneUsedFallback = false;
static bool autoTuneReachedSetpoint = false;
static float autoTuneStepTimeout = AUTOTUNE_MAX_STEP_TIME;
static unsigned long autoTuneRorStableStartTime = 0;
static unsigned long autoTuneFastTrackStartTime = 0;
static bool autoTuneOutputHigh = false;

// Auto-tune data storage
static PeakValley autoTunePeaks[10];
static PeakValley autoTuneValleys[10];
static int autoTunePeakCount = 0;
static int autoTuneValleyCount = 0;

// Auto-tune temperature history for oscillation detection
static float autoTuneTempHistory[20];
static unsigned long autoTuneTempTimeHistory[20];
static int autoTuneTempHistoryIndex = 0;
static int autoTuneTempHistoryCount = 0;

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
        autoTuneState != AUTOTUNE_HEATING && autoTuneState != AUTOTUNE_STABILIZING) {
        return;
    }

    unsigned long now = millis();

    // Check for timeout
    if (now - autoTuneStartTime > AUTOTUNE_MAX_DURATION) {
        DEBUG_PRINTLN(F("Auto-tune timeout"));
        autoTuneState = AUTOTUNE_FAILED;
        publishAutoTuneStatusMsg();
        handleAutoTuneStop("", 0);
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
        publishAutoTuneResultsMsg();
        state.heaterEnabled = false;
        state.controlMode = MODE_AUTO;
        publishAutoTuneStatusMsg();
    }
}

static bool calculateAutoTunePIDParameters() {
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

static void resetAutoTuneData() {
    autoTunePeakCount = 0;
    autoTuneValleyCount = 0;
    autoTuneTempHistoryCount = 0;
    autoTuneTempHistoryIndex = 0;
    autoTuneRecommendedKp = 0;
    autoTuneRecommendedKi = 0;
    autoTuneRecommendedKd = 0;
}
