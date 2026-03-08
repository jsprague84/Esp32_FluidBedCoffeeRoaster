#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "roaster_state.h"
#include "safety.h"
#include "temperature.h"

// Consecutive NaN counters per thermocouple
static int beanNanCount = 0;
static int envNanCount = 0;

// MQTT connection tracking
static unsigned long lastMqttConnectedTime = 0;

void initSafety() {
    beanNanCount = 0;
    envNanCount = 0;
    lastMqttConnectedTime = millis();
}

void triggerSafeShutdown() {
    state.heaterOutput = 0;
    ledcWrite(SSR_PIN, 0);
    state.fanPWM = 255;
    state.prevFanPWM = 255;
    ledcWrite(FAN_PIN, 255);
    state.heaterEnabled = false;
    DEBUG_PRINTLN(F("SAFETY: Safe shutdown triggered — heater OFF, fan MAX"));
}

void resetSafety() {
    beanNanCount = 0;
    envNanCount = 0;
    state.systemStatus = SYSTEM_OK;
}

// Call this from mqtt_handler or main when MQTT is connected
void safetyNotifyMqttConnected() {
    lastMqttConnectedTime = millis();
}

void runSafetyChecks() {
    // --- Over-temperature cutoff ---
    if (state.beanTemperature >= MAX_BEAN_TEMP) {
        DEBUG_PRINTF("SAFETY: Bean temp %.1f >= %.1f MAX\n",
                     state.beanTemperature, (double)MAX_BEAN_TEMP);
        state.systemStatus = SAFETY_ERROR;
        triggerSafeShutdown();
        return;
    }
    if (state.envTemperature >= MAX_ENV_TEMP) {
        DEBUG_PRINTF("SAFETY: Env temp %.1f >= %.1f MAX\n",
                     (double)state.envTemperature, (double)MAX_ENV_TEMP);
        state.systemStatus = SAFETY_ERROR;
        triggerSafeShutdown();
        return;
    }

    // --- Sensor failure detection ---
    if (isnan(state.beanTemperature)) {
        beanNanCount++;
        if (beanNanCount >= SAFETY_SENSOR_FAIL_COUNT) {
            DEBUG_PRINTF("SAFETY: Bean sensor failed %d consecutive reads\n", beanNanCount);
            state.systemStatus = SENSOR_ERROR;
            triggerSafeShutdown();
            return;
        }
    } else {
        beanNanCount = 0;
    }

    if (isnan(state.envTemperature)) {
        envNanCount++;
        if (envNanCount >= SAFETY_SENSOR_FAIL_COUNT) {
            DEBUG_PRINTF("SAFETY: Env sensor failed %d consecutive reads\n", envNanCount);
            state.systemStatus = SENSOR_ERROR;
            triggerSafeShutdown();
            return;
        }
    } else {
        envNanCount = 0;
    }

    // --- Rate-of-rise limit ---
    float ror = getRateOfRise();
    if (ror > SAFETY_MAX_ROR) {
        DEBUG_PRINTF("SAFETY: RoR %.2f > %.1f limit — zeroing heater\n",
                     ror, (double)SAFETY_MAX_ROR);
        state.systemStatus = ROR_ERROR;
        state.heaterOutput = 0;
        ledcWrite(SSR_PIN, 0);
        return;
    }

    // --- MQTT communication timeout ---
    // Track MQTT connected state
    // Note: Until US-012 replaces PubSubClient, we check via extern mqttClient
    // After US-012, this will use mqtt_handler's isConnected()/getLastConnectedTime()
    extern bool safetyMqttConnected;
    if (safetyMqttConnected) {
        lastMqttConnectedTime = millis();
    }
    if (millis() - lastMqttConnectedTime > SAFETY_MQTT_TIMEOUT_MS) {
        DEBUG_PRINTLN(F("SAFETY: MQTT timeout — entering safe state"));
        state.systemStatus = COMMS_TIMEOUT;
        triggerSafeShutdown();
        return;
    }
}
