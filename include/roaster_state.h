#pragma once

// Control Modes
#define MODE_MANUAL 0
#define MODE_AUTO 1

// System Status
typedef enum {
    SYSTEM_OK = 0,
    WIFI_ERROR = 1,
    MQTT_ERROR = 2,
    SENSOR_ERROR = 3,
    SAFETY_ERROR = 4,
    ROR_ERROR = 5,
    COMMS_TIMEOUT = 6
} SystemStatus;

struct RoasterState {
    double beanSetpoint;
    double beanTemperature;
    double heaterOutput;
    float envTemperature;
    int fanPWM;
    int prevFanPWM;
    int controlMode;
    bool heaterEnabled;
    double Kp;
    double Ki;
    double Kd;
    float beanTempOffset;
    float envTempOffset;
    SystemStatus systemStatus;
};

extern RoasterState state;
