#include <SPI.h>
#include "MAX6675Handler.h"
#include "config.h"
#include "debug.h"
#include "roaster_state.h"
#include "temperature.h"

// Hardware Pin Definitions
#define BT_CS BT_CS_PIN
#define ET_CS ET_CS_PIN

// Thermocouple instances (local to this module)
static MAX6675Handler beanThermocouple(BT_CS);
static MAX6675Handler envThermocouple(ET_CS);

// Rate of Rise calculation (static file-scope)
static float tempHistory[RATE_HISTORY_SIZE];
static unsigned long timeHistory[RATE_HISTORY_SIZE];
static int historyIndex = 0;
static int historyCount = 0;

// Timing
static unsigned long lastTempRead = 0;

void initTemperature() {
    SPI.begin();
    beanThermocouple.begin();
    envThermocouple.begin();
    DEBUG_PRINTLN(F("MAX6675 Initialized"));
}

void readTemperatures() {
    if (millis() - lastTempRead < TEMP_READ_INTERVAL) return;
    lastTempRead = millis();

    state.beanTemperature = beanThermocouple.readTemperature();
    state.envTemperature = envThermocouple.readTemperature();

    if (!isnan(state.beanTemperature) && !isnan(state.envTemperature)) {
        applyCalibration();
        updateRateOfRise(state.beanTemperature);
    }
}

void applyCalibration() {
    state.beanTemperature += state.beanTempOffset;
    state.envTemperature += state.envTempOffset;
}

void updateRateOfRise(float currentTemp) {
    tempHistory[historyIndex] = currentTemp;
    timeHistory[historyIndex] = millis();
    historyIndex = (historyIndex + 1) % RATE_HISTORY_SIZE;
    if (historyCount < RATE_HISTORY_SIZE) historyCount++;
}

float getRateOfRise() {
    if (historyCount < 3) return 0.0;

    int startIdx = (historyIndex - historyCount + RATE_HISTORY_SIZE) % RATE_HISTORY_SIZE;
    int endIdx = (historyIndex - 1 + RATE_HISTORY_SIZE) % RATE_HISTORY_SIZE;

    float tempDiff = tempHistory[endIdx] - tempHistory[startIdx];
    float timeDiff = (timeHistory[endIdx] - timeHistory[startIdx]) / 1000.0;

    if (timeDiff > 0) {
        return (tempDiff / timeDiff) * 60.0;
    }
    return 0.0;
}
