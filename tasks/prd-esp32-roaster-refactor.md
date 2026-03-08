# PRD: ESP32 Fluid-Bed Coffee Roaster Controller — Optimized Refactor & Rewrite

## Introduction

Comprehensive refactor and rewrite of the ESP32-based fluid-bed coffee roaster controller firmware. The current firmware is a functional but monolithic 1300-line `main.cpp` with deprecated APIs, missing safety protections, and heap fragmentation risks during long roast sessions.

The ESP32 communicates with a Rust Axum web application (rustRoast) over MQTT. rustRoast is a multi-device platform supporting multiple protocols and device configurations. Any MQTT protocol changes must be coordinated across both systems.

This refactor addresses 20 identified issues spanning safety, reliability, maintainability, and modern API compliance. The target platform is Arduino Core 3.x via pioarduino with ESP-IDF 5.x.

## Goals

- Eliminate all safety gaps: over-temperature cutoff, sensor failure detection, MQTT timeout, rate-of-rise limiting
- Modularize into 6+ source files with a shared state struct for maintainability
- Migrate to Arduino Core 3.x APIs (LEDC, watchdog, Preferences) via pioarduino platform
- Upgrade ArduinoJson v6 to v7 (remove all deprecated Document types)
- Replace PubSubClient with ESP32MQTTClient (non-blocking, thread-safe MQTT)
- Add NTP time sync for real Unix timestamps in telemetry
- Eliminate heap fragmentation from Arduino String usage in hot paths
- Fix platformio.ini build configuration (lib_deps in correct env)
- Sync config.example.h with config.h (auto-tune topics and configuration)
- Remove dead code and fix auto-tune state machine bugs
- Update rustRoast TelemetryPayload to include `uptimeMs` field

## User Stories

### US-001: Fix platformio.ini build configuration
**Description:** As a developer, I want `pio run` against the default environment to compile successfully so that fresh clones build without errors.

**Acceptance Criteria:**
- [ ] `[env]` common section contains all shared settings (framework, board, monitor_speed, upload_speed, lib_deps)
- [ ] `[env:nodemcu-32s]` inherits common settings and builds via USB serial
- [ ] `[env:nodemcu-32s-ota]` inherits common settings and adds OTA upload configuration
- [ ] Platform pinned to pioarduino Arduino Core 3.x release URL
- [ ] lib_deps updated: `br3ttb/PID@^1.2.1`, `cyijun/ESP32MQTTClient@^1.1.1`, `bblanchon/ArduinoJson@^7.4.1`, `Wire`
- [ ] Clean `pio run` succeeds on both environments
- [ ] OTA password sourced from a build flag or config, not hardcoded

### US-002: Create modular file structure with shared state
**Description:** As a developer, I want the firmware split into logical modules so that I can navigate, test, and modify individual subsystems without reading 1300 lines.

**Acceptance Criteria:**
- [ ] `include/roaster_state.h` — shared `RoasterState` struct with all global control variables
- [ ] `include/debug.h` — debug macros (DEBUG_PRINT, DEBUG_PRINTLN, DEBUG_PRINTF) controlled by build flag
- [ ] `include/safety.h` — safety constants and function declarations
- [ ] `include/mqtt_handler.h` — MQTT function declarations
- [ ] `include/temperature.h` — thermocouple and rate-of-rise declarations
- [ ] `include/heater_control.h` — heater and PID declarations
- [ ] `include/autotune.h` — auto-tune state machine declarations
- [ ] `src/main.cpp` — slim setup()/loop() calling init and update functions from modules (~100-150 lines)
- [ ] `src/safety.cpp` — safety monitor implementation
- [ ] `src/mqtt_handler.cpp` — MQTT connection, subscriptions, publishing, message routing
- [ ] `src/temperature.cpp` — thermocouple reading, calibration, rate-of-rise calculation
- [ ] `src/heater_control.cpp` — heater control logic, PID management
- [ ] `src/autotune.cpp` — auto-tune state machine
- [ ] All modules access shared state via `extern RoasterState state;`
- [ ] Project compiles cleanly with no linker errors

### US-003: Implement comprehensive safety monitor
**Description:** As a roaster operator, I want multiple independent safety checks running every loop iteration so that hardware failures, communication loss, or control runaway cannot cause a fire.

**Acceptance Criteria:**
- [ ] Over-temperature cutoff: heater immediately disabled if `beanTemp >= MAX_BEAN_TEMP` OR `envTemp >= MAX_ENV_TEMP` (new config constant)
- [ ] Sensor failure: consecutive NaN counter per thermocouple; heater disabled after 3 consecutive failures; counter resets on valid reading
- [ ] MQTT communication timeout: if MQTT disconnected for > `SAFETY_MQTT_TIMEOUT_MS` (default 60000ms), enter safe state (heater off, fan max)
- [ ] Rate-of-rise limit: heater output zeroed if RoR > `SAFETY_MAX_ROR` (default 30.0 C/min)
- [ ] All safety checks execute BEFORE heater output write in loop()
- [ ] Safety events published to MQTT status topic when connected
- [ ] Safety events logged to Serial debug output
- [ ] `systemStatus` enum extended with `ROR_ERROR` and `COMMS_TIMEOUT` values
- [ ] Emergency stop handler preserved and working

### US-004: Migrate to Arduino Core 3.x LEDC API
**Description:** As a developer, I want the firmware to use the current LEDC API so that heater and fan PWM outputs work correctly on Arduino Core 3.x.

**Acceptance Criteria:**
- [ ] `ledcSetup()` and `ledcAttachPin()` replaced with `ledcAttach(pin, freq, resolution)`
- [ ] All `ledcWrite()` calls use GPIO pin numbers (SSR_PIN, FAN_PIN) instead of channel numbers (0, 1)
- [ ] `gpio_set_drive_capability()` call for FAN_PIN preserved
- [ ] Heater SSR: 5kHz, 8-bit resolution on SSR_PIN (GPIO 33)
- [ ] Fan: 5kHz, 8-bit resolution on FAN_PIN (GPIO 25)
- [ ] Fan PWM only written when value changes (track previous value)

### US-005: Replace EEPROM with Preferences (NVS)
**Description:** As a developer, I want PID parameters stored using the ESP32 Preferences library so that values survive power loss without flash corruption risk.

**Acceptance Criteria:**
- [ ] `#include <EEPROM.h>` replaced with `#include <Preferences.h>`
- [ ] `EEPROM.begin()` removed from setup
- [ ] `savePIDParameters()` uses `preferences.begin("pid", false)` / `preferences.putDouble()` / `preferences.end()`
- [ ] `loadPIDParameters()` uses `preferences.begin("pid", true)` / `preferences.getDouble(key, default)` / `preferences.end()`
- [ ] Default PID values returned automatically when key missing (no manual NaN validation needed)
- [ ] EEPROM_SIZE config constant removed
- [ ] PID values persist across reboot (manual verification)

### US-006: Migrate to ArduinoJson v7
**Description:** As a developer, I want all JSON serialization/deserialization using ArduinoJson v7 so that the firmware uses current, maintained APIs.

**Acceptance Criteria:**
- [ ] All `StaticJsonDocument<N>` and `DynamicJsonDocument(N)` replaced with `JsonDocument`
- [ ] All `doc.containsKey("key")` replaced with `doc["key"].is<T>()` pattern
- [ ] `serializeJson()` and `deserializeJson()` calls unchanged (same API in v7)
- [ ] `doc.overflowed()` checked after building large telemetry documents
- [ ] No `JSON_ARRAY_SIZE`, `JSON_OBJECT_SIZE`, or `JSON_STRING_SIZE` macros used
- [ ] Project compiles cleanly with `bblanchon/ArduinoJson@^7.4.1`

### US-007: Replace PubSubClient with ESP32MQTTClient
**Description:** As a roaster operator, I want MQTT reconnection to happen in a background FreeRTOS task so that heater control is never blocked during network issues.

**Acceptance Criteria:**
- [ ] `#include <PubSubClient.h>` replaced with `#include "ESP32MQTTClient.h"`
- [ ] MQTT client runs in background FreeRTOS task via `mqttClient.loopStart()`
- [ ] No `mqttClient.loop()` call needed in main loop
- [ ] Last Will Testament configured: `{"status":"offline"}` on status topic (retained)
- [ ] All control topic subscriptions registered with lambda callbacks
- [ ] Auto-tune topic subscriptions registered
- [ ] Telemetry publishing uses `mqttClient.publish()` with QoS 1
- [ ] Status publishing uses retained messages
- [ ] MQTT reconnection never blocks the main control loop
- [ ] `MQTT_MAX_PACKET_SIZE` / buffer size configured to 512+ bytes

### US-008: Eliminate heap fragmentation in MQTT message handling
**Description:** As a developer, I want MQTT message handling to use stack-allocated char buffers instead of Arduino String objects so that heap fragmentation doesn't cause crashes during long roast sessions.

**Acceptance Criteria:**
- [ ] `handleMQTTMessage()` uses `char[]` buffer with `memcpy()` instead of `String payloadStr`
- [ ] Topic matching uses `strncmp()` / `strcmp()` instead of `String::startsWith()` / `String::substring()`
- [ ] Control command handlers accept `const char*` instead of `const String&`
- [ ] Telemetry JSON serialized to a `static char jsonBuf[512]` instead of Arduino String
- [ ] No Arduino `String` objects created in any function called from `loop()` hot path
- [ ] Free heap and largest free block included in telemetry for monitoring

### US-009: Add NTP time sync and real Unix timestamps
**Description:** As a rustRoast backend developer, I want telemetry timestamps to be real Unix epoch seconds so that server-side time correlation is accurate across devices and reboots.

**Acceptance Criteria:**
- [ ] `configTime()` called after WiFi connection with `pool.ntp.org` and `time.nist.gov`
- [ ] NTP sync wait with timeout (max 10 retries, 1s each) — non-blocking after initial attempt
- [ ] Telemetry `timestamp` field contains Unix epoch seconds when NTP synced
- [ ] Telemetry `timestamp` field contains `0` when NTP not yet synced (server uses its own clock)
- [ ] New `uptimeMs` field added to telemetry containing `millis()` value
- [ ] NTP server addresses configurable in config.h

### US-010: Update rustRoast TelemetryPayload for new fields
**Description:** As a rustRoast developer, I want the TelemetryPayload struct to accept the new `uptimeMs` field and handle real Unix timestamps so that the backend correctly processes updated ESP32 telemetry.

**Acceptance Criteria:**
- [ ] `TelemetryPayload` struct in `crates/server/src/telemetry.rs` gains `uptime_ms: Option<u64>` field with `#[serde(default, rename = "uptimeMs")]`
- [ ] Existing `uptime` field (seconds) continues to work (backward compatible)
- [ ] Test `test_parse_esp32_payload` updated with realistic Unix timestamp and `uptimeMs` field
- [ ] Backward compatible: old ESP32 firmware without `uptimeMs` still parses successfully (field is `Option`)
- [ ] `session_telemetry` INSERT query unchanged (doesn't use timestamp or uptimeMs directly)

### US-011: Sync config.example.h with config.h
**Description:** As a developer cloning the repo, I want config.example.h to contain all required definitions so that copying it to config.h produces a compilable project.

**Acceptance Criteria:**
- [ ] Auto-tune MQTT topic definitions added (`MQTT_AUTOTUNE_STATUS_TOPIC`, `MQTT_AUTOTUNE_START_TOPIC`, `MQTT_AUTOTUNE_STOP_TOPIC`, `MQTT_AUTOTUNE_APPLY_TOPIC`, `MQTT_AUTOTUNE_RESULTS_TOPIC`)
- [ ] Full auto-tune configuration block added (all `AUTOTUNE_*` defines)
- [ ] Safety configuration constants added (`MAX_ENV_TEMP`, `SAFETY_MQTT_TIMEOUT_MS`, `SAFETY_MAX_ROR`, `SAFETY_SENSOR_FAIL_COUNT`)
- [ ] NTP server configuration added
- [ ] OTA password defined as `OTA_PASSWORD`
- [ ] ESP32MQTTClient-specific config if needed
- [ ] WiFi credentials use placeholder values
- [ ] MQTT broker uses placeholder value
- [ ] Copying config.example.h to config.h and filling in credentials produces a compilable build

### US-012: Fix auto-tune state machine bugs and remove dead code
**Description:** As a developer, I want the auto-tune state machine to correctly report failure states and not contain unreachable code.

**Acceptance Criteria:**
- [ ] `checkAutoTuneCrossedSetpoint()` function removed (dead code — never called)
- [ ] Auto-tune timeout: `AUTOTUNE_FAILED` state preserved and published before cleanup (not overwritten by stop handler)
- [ ] `handleAutoTuneStop()` checks for `AUTOTUNE_FAILED` state and preserves it
- [ ] Auto-tune status publish includes failure reason when in FAILED state
- [ ] Ziegler-Nichols calculation algorithm preserved unchanged
- [ ] Auto-tune peak/valley recording in RUNNING phase preserved unchanged

### US-013: Update ESP32 watchdog API for ESP-IDF 5.x
**Description:** As a developer, I want the watchdog timer using the ESP-IDF 5.x API so that it compiles on Arduino Core 3.x.

**Acceptance Criteria:**
- [ ] `esp_task_wdt_init(timeout, panic)` replaced with `esp_task_wdt_init(&config)` using `esp_task_wdt_config_t` struct
- [ ] Timeout specified in milliseconds (`WATCHDOG_TIMEOUT_SEC * 1000`)
- [ ] `idle_core_mask` set to monitor Core 0
- [ ] `trigger_panic = true` preserved
- [ ] `esp_task_wdt_add(NULL)` and `esp_task_wdt_reset()` calls unchanged

### US-014: Fix MAX6675Handler SPI initialization
**Description:** As a developer, I want SPI initialized once in setup rather than once per thermocouple handler so that initialization is clean and correct.

**Acceptance Criteria:**
- [ ] `SPI.begin()` called once in `setup()` before thermocouple `begin()` calls
- [ ] `MAX6675Handler::begin()` no longer calls `SPI.begin()` — only sets up CS pin
- [ ] Both thermocouples read correctly after the change
- [ ] Thermocouple error detection (open circuit NaN) still works

### US-015: Improve MQTT control message parsing robustness
**Description:** As a roaster operator, I want invalid MQTT control payloads rejected rather than silently setting values to zero so that accidental misconfiguration doesn't disrupt a roast.

**Acceptance Criteria:**
- [ ] Setpoint handler uses `strtof()` with end-pointer validation; rejects non-numeric input
- [ ] Fan PWM handler uses `strtol()` with end-pointer validation; rejects non-numeric input
- [ ] Heater PWM handler uses `strtol()` with end-pointer validation; rejects non-numeric input
- [ ] Invalid payloads logged to debug output with topic name
- [ ] Valid "0" values still accepted correctly for all handlers
- [ ] Heater PWM conversion uses proper rounding: `(value * 255 + 50) / 100`

### US-016: Standardize MQTT LWT and status messages
**Description:** As a rustRoast backend developer, I want consistent fields in online/offline status messages so that device registry information is always complete.

**Acceptance Criteria:**
- [ ] LWT (offline) message includes: `status`, `id`, `timestamp`
- [ ] Online status message includes: `status`, `id`, `ip`, `rssi`, `version`, `freeHeap`, `timestamp`
- [ ] Periodic status publish (via `publishMQTTStatus()`) includes same fields as online message
- [ ] `version` field uses a `FIRMWARE_VERSION` define in config.h
- [ ] All status messages use retained flag

## Functional Requirements

- FR-1: Safety monitor runs every loop iteration BEFORE heater output, checking: over-temperature, sensor validity, MQTT connectivity, rate-of-rise
- FR-2: Heater immediately disabled (PWM=0) when ANY safety condition triggers
- FR-3: Fan set to maximum (PWM=255) during safety-triggered shutdown for cooling
- FR-4: All MQTT communication is non-blocking; reconnection happens in background FreeRTOS task
- FR-5: Telemetry published at 1Hz with real Unix epoch timestamp (NTP) and uptimeMs
- FR-6: PID parameters stored in NVS via Preferences library with named keys
- FR-7: All JSON operations use ArduinoJson v7 `JsonDocument` type
- FR-8: LEDC PWM uses pin-based API (`ledcAttach`/`ledcWrite` with pin numbers)
- FR-9: Watchdog timer uses ESP-IDF 5.x config struct API
- FR-10: No Arduino `String` objects allocated in the main loop hot path
- FR-11: Auto-tune FAILED state persisted and published before cleanup
- FR-12: Invalid MQTT control payloads rejected with debug logging (not silently applied)
- FR-13: config.example.h contains all definitions needed for compilation
- FR-14: SPI bus initialized exactly once before thermocouple handlers
- FR-15: rustRoast TelemetryPayload accepts new `uptimeMs` field (backward compatible)

## Non-Goals

- No change to the auto-tune Ziegler-Nichols calculation algorithm
- No change to MQTT topic structure or naming
- No addition of new MQTT control commands
- No BLE or web server functionality on the ESP32
- No change to hardware pin assignments
- No change to PID library (br3ttb/PID stays at v1.2.1)
- No PSRAM support (NodeMCU-32S does not have PSRAM)
- No TLS/SSL for MQTT (local network only)
- No change to rustRoast API endpoints, WebSocket protocol, or database schema (beyond adding Optional telemetry field)
- No dashboard UI changes

## Technical Considerations

- **Platform:** pioarduino `55.03.37` (Arduino Core 3.3.7, ESP-IDF 5.5.2)
- **Board:** `nodemcu-32s` (ESP32-WROOM-32, no PSRAM, 520KB SRAM)
- **MQTT library change:** ESP32MQTTClient uses `std::string` internally (no Arduino String fragmentation). Callback signatures differ from PubSubClient — all message routing must be rewritten.
- **ArduinoJson v7 memory:** Allocates in 1KB heap blocks. On ESP32 with ~300KB free heap this is fine, but telemetry publishing creates a new JsonDocument every second. Consider reusing a global document with `doc.clear()`.
- **NTP dependency:** If WiFi is unavailable at boot, NTP won't sync. Telemetry timestamp will be 0 until sync occurs. The rustRoast backend should handle timestamp=0 gracefully (use server time).
- **Backward compatibility:** The `uptimeMs` field in rustRoast uses `#[serde(default)]` so old firmware without this field still deserializes correctly.
- **Testing:** No automated tests for ESP32 firmware. Verification is manual via serial monitor, MQTT broker inspection, and rustRoast dashboard observation.

## Success Metrics

- Clean `pio run` on fresh clone (after copying config.example.h)
- No Arduino `String` allocations in loop() hot path (verified by code inspection)
- Free heap stable (no downward trend) over 30+ minute roast sessions
- All 5 safety conditions trigger correctly when simulated
- MQTT reconnection does not cause any delay in heater control loop
- Telemetry timestamps are real Unix epoch seconds (verified in rustRoast DB)
- Auto-tune completes successfully with same quality as before refactor

## Open Questions

1. Should the ESP32MQTTClient library support QoS 2 for emergency stop commands, or is QoS 1 sufficient?
2. Should the safety monitor be able to be reset via MQTT after a safety shutdown, or require a physical reboot?
3. Is the 60-second MQTT timeout appropriate, or should it be shorter for active roast sessions?
4. Should the firmware version be auto-incremented from git tags via PlatformIO build flags?
