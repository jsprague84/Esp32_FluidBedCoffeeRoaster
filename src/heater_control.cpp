#include <Arduino.h>
#include <PID_v1.h>
#include <Preferences.h>
#include "config.h"
#include "debug.h"
#include "roaster_state.h"
#include "heater_control.h"
#include "driver/gpio.h"

// PID controller uses pointers into state struct
static PID beanPID(&state.beanTemperature, &state.heaterOutput, &state.beanSetpoint,
                   DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);

// Preferences for NVS-based PID storage
static Preferences preferences;

// PID compute timing
static unsigned long lastPidCompute = 0;

void initHeaterControl() {
    // Initialize LEDC for PWM (Arduino Core 3.x API)
    ledcAttach(SSR_PIN, 5000, 8);
    ledcAttach(FAN_PIN, 5000, 8);
    gpio_set_drive_capability((gpio_num_t)FAN_PIN, GPIO_DRIVE_CAP_3);

    loadPIDParameters();

    // Initialize PID
    beanPID.SetMode(AUTOMATIC);
    beanPID.SetOutputLimits(0, 255);
}

void updateHeaterControl() {
    // Fan PWM — only write when value changes
    if (state.fanPWM != state.prevFanPWM) {
        ledcWrite(FAN_PIN, state.fanPWM);
        state.prevFanPWM = state.fanPWM;
    }

    // Heater control logic
    if (!state.heaterEnabled) {
        state.heaterOutput = 0;
        ledcWrite(SSR_PIN, 0);
    } else {
        if (state.controlMode == MODE_MANUAL) {
            // In manual mode, heaterOutput is set directly via MQTT
            if (state.fanPWM > SAFETY_MIN_FAN_PWM) {
                ledcWrite(SSR_PIN, static_cast<int>(state.heaterOutput));
            } else {
                ledcWrite(SSR_PIN, 0);
                DEBUG_PRINTLN(F("Failsafe: Fan too low, heater off."));
            }
        } else if (state.controlMode == MODE_AUTO) {
            if (!isnan(state.beanTemperature)) {
                if (millis() - lastPidCompute >= PID_COMPUTE_INTERVAL) {
                    lastPidCompute = millis();
                    beanPID.Compute();
                }
                if (state.fanPWM > SAFETY_MIN_FAN_PWM) {
                    ledcWrite(SSR_PIN, static_cast<int>(state.heaterOutput));
                } else {
                    ledcWrite(SSR_PIN, 0);
                    DEBUG_PRINTLN(F("Failsafe: Fan too low, heater off."));
                }
            } else {
                DEBUG_PRINTLN(F("Invalid bean temperature! Heater disabled."));
                ledcWrite(SSR_PIN, 0);
            }
        }
    }
}

void savePIDParameters() {
    preferences.begin("pid", false);
    preferences.putDouble("Kp", state.Kp);
    preferences.putDouble("Ki", state.Ki);
    preferences.putDouble("Kd", state.Kd);
    preferences.end();
    DEBUG_PRINTLN(F("PID Parameters Saved to NVS"));
}

void loadPIDParameters() {
    preferences.begin("pid", true);
    state.Kp = preferences.getDouble("Kp", DEFAULT_KP);
    state.Ki = preferences.getDouble("Ki", DEFAULT_KI);
    state.Kd = preferences.getDouble("Kd", DEFAULT_KD);
    preferences.end();

    DEBUG_PRINTLN(F("PID Parameters Loaded from NVS"));
    DEBUG_PRINTF("Loaded PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", state.Kp, state.Ki, state.Kd);

    beanPID.SetTunings(state.Kp, state.Ki, state.Kd);
}

void updatePIDTunings() {
    beanPID.SetTunings(state.Kp, state.Ki, state.Kd);
}
