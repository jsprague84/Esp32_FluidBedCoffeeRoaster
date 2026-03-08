// filepath: //esp32CoffeeRoaster/include/config.example.h
#ifndef CONFIG_H
#define CONFIG_H

// Firmware Version
#define FIRMWARE_VERSION "3.0.0"

// OTA Configuration
#ifndef OTA_PASSWORD
#define OTA_PASSWORD "YourOTAPassword"
#endif

// WiFi Configuration
extern const char* ssid;
extern const char* password;

// MQTT Configuration
#define MQTT_BROKER "YourMQTTBrokerAddress"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32_roaster_01"
#define MQTT_CLEAN_SESSION true

// Topic structure
#define MQTT_BASE_TOPIC "roaster/"
#define MQTT_STATUS_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/status"
#define MQTT_TELEMETRY_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/telemetry"
#define MQTT_CONTROL_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/control"

// Auto-tune MQTT topics
#define MQTT_AUTOTUNE_STATUS_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/autotune/status"
#define MQTT_AUTOTUNE_START_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/autotune/start"
#define MQTT_AUTOTUNE_STOP_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/autotune/stop"
#define MQTT_AUTOTUNE_APPLY_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/autotune/apply"
#define MQTT_AUTOTUNE_RESULTS_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/autotune/results"

// Add debug topics
#define MQTT_DEBUG_TOPIC MQTT_BASE_TOPIC MQTT_CLIENT_ID "/debug"

// Hardware Pin Configuration
#define BT_CS_PIN   4
#define ET_CS_PIN   5
#define SSR_PIN     33
#define FAN_PIN     25

// Safety Parameters
#define SAFETY_MIN_FAN_PWM 100
#define MAX_BEAN_TEMP 240.0
#define MAX_ENV_TEMP 300.0
#define MIN_BEAN_TEMP 0.0
#define MAX_HEATER_PWM 100
#define MIN_HEATER_PWM 0
#define SAFETY_MQTT_TIMEOUT_MS 60000
#define SAFETY_MAX_ROR 30.0
#define SAFETY_SENSOR_FAIL_COUNT 3
#define SAFETY_MAX_HEATER_TEMP 260.0

// Temperature Calibration Offsets
#define TEMP_CALIBRATION_BEAN 0.0
#define TEMP_CALIBRATION_ENV 0.0

// NTP Configuration
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define NTP_GMT_OFFSET 0
#define NTP_DAYLIGHT_OFFSET 0

// Timing Configuration (milliseconds)
#define TEMP_READ_INTERVAL 1000
#define PID_COMPUTE_INTERVAL 100
#define SERIAL_OUTPUT_INTERVAL 1000
#define MQTT_PUBLISH_INTERVAL 1000
#define MQTT_RECONNECT_INTERVAL 5000
#define WIFI_CHECK_INTERVAL 30000

// System Configuration
#define WATCHDOG_TIMEOUT_SEC 30
#define WIFI_TIMEOUT_MS 20000
#define MQTT_BUFFER_SIZE 512
#define RATE_HISTORY_SIZE 10

// PID Default Values
#define DEFAULT_KP 15.0
#define DEFAULT_KI 1.0
#define DEFAULT_KD 25.0

// Auto-tune Configuration
// -- Timing --
#define AUTOTUNE_MIN_STEP_TIME 10000         // Min seconds per relay step (ms)
#define AUTOTUNE_MAX_STEP_TIME 90000         // Max seconds per relay step (ms)
#define AUTOTUNE_INITIAL_STEP_TIME 120000    // Heating phase timeout (ms)
#define AUTOTUNE_MAX_DURATION 1800000        // Max total auto-tune duration (ms)
#define AUTOTUNE_STABILIZATION_TIME 45000    // Equilibrium stabilization time (ms)
#define AUTOTUNE_STABILITY_ROR_TIME 8000     // RoR must be low for this long (ms)
// -- Thresholds --
#define AUTOTUNE_TOTAL_STEPS 12              // Relay oscillation steps (>= 6 post-transient)
#define AUTOTUNE_SETPOINT_TOLERANCE 2.5      // ±°C tolerance for "near setpoint"
#define AUTOTUNE_STABILITY_ROR 2.0           // Max abs RoR for stable (°C/min)
#define AUTOTUNE_EQUIL_MIN_PCT 0.75f         // Min fraction of target for equilibrium (0.0-1.0)
#define AUTOTUNE_MIN_PEAKS 3                 // Min peaks for analysis
#define AUTOTUNE_MIN_VALLEYS 3               // Min valleys for analysis
#define AUTOTUNE_FALLBACK_MIN_PERIOD 10.0f   // Min period for valid fallback (seconds)
#define AUTOTUNE_FALLBACK_MIN_AMPLITUDE 1.0f // Min amplitude for valid fallback (°C)
#define AUTOTUNE_CONSISTENCY_TOLERANCE 0.10f // Max relative deviation for consistency (0.0-1.0)
// -- Relay parameters (configurable via MQTT start command) --
#define AUTOTUNE_OUTPUT_BIAS 50              // Default relay bias (%, 10-90)
#define AUTOTUNE_OUTPUT_AMPLITUDE 25         // Default relay amplitude (%, 5-45)
#define AUTOTUNE_RELAY_HYST 1.0              // Default relay hysteresis (°C, 0.1-5.0)
// -- Filter --
#define AUTOTUNE_EMA_ALPHA 0.3f              // EMA noise filter (0.0-1.0, lower = smoother)
#define AUTOTUNE_TEMP_HISTORY_SIZE 120       // History buffer (samples at ~1Hz)
// -- Step response mode --
#define AUTOTUNE_STEP_BASELINE_TIME 60000    // Baseline measurement period (ms)
#define AUTOTUNE_STEP_SETTLE_ROR 0.5f        // Max RoR for settled (°C/min)
#define AUTOTUNE_STEP_SETTLE_TIME 30000      // Settle confirmation time (ms)
#define AUTOTUNE_STEP_DATA_SIZE 300          // Data buffer (5 minutes at 1Hz)

#endif
