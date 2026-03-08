#pragma once

// Auto-tune state machine states
typedef enum {
    AUTOTUNE_IDLE = 0,
    AUTOTUNE_HEATING = 1,
    AUTOTUNE_STABILIZING = 2,
    AUTOTUNE_RUNNING = 3,
    AUTOTUNE_ANALYZING = 4,
    AUTOTUNE_COMPLETE = 5,
    AUTOTUNE_FAILED = 6
} AutoTuneState;

void initAutoTune();
void updateAutoTune();
void handleAutoTuneStart(const char* payload, size_t len);
void handleAutoTuneStop(const char* payload, size_t len);
void handleAutoTuneApply(const char* payload, size_t len);
bool isAutoTuneActive();
const char* getAutoTuneStateString(AutoTuneState atState);
