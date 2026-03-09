# Vision: Generic Temperature Control Platform

## Overview

Transform the current coffee-roaster-specific ESP32 firmware and rustRoast web application into a **generic temperature control platform** capable of managing multiple device types (coffee roasters, smokers, kilns, sous vide, fermentation chambers, etc.) from a single unified interface.

The ESP32 firmware becomes a **universal PID temperature controller** configured per device via `config.h`. The rustRoast application evolves into a **multi-device platform** with generic connection/configuration plumbing and pluggable device-type-specific feature modules.

---

## Part 1: Generic ESP32 Firmware

### Current State (Post-Refactor)
The firmware is already 90% generic — modular architecture with temperature reading, PID control, safety monitoring, MQTT communication, and auto-tune. The only device-specific elements are naming conventions and hardcoded assumptions.

### What Needs to Change

#### A. Naming & Terminology
| Current (Coffee-Specific) | Generic Replacement |
|---|---|
| `beanTemp` / `beanTemperature` | `primaryTemp` or `probeTemp[0]` |
| `envTemp` / `envTemperature` | `secondaryTemp` or `probeTemp[1]` |
| `beanSetpoint` | `setpoint` (already used in some places) |
| `beanPID` | `pid` |
| `beanThermocouple` | `probe[0]` or `primaryProbe` |
| `envThermocouple` | `probe[1]` or `secondaryProbe` |
| `MAX_BEAN_TEMP` | `MAX_PRIMARY_TEMP` |
| `MAX_ENV_TEMP` | `MAX_SECONDARY_TEMP` |
| MQTT topic root `roaster/` | Configurable `MQTT_TOPIC_ROOT` in config.h |

#### B. Configurable Sensor Count
Currently hardcoded to 2 thermocouples (bean + environment). Should support:
- 1 sensor (sous vide, simple smoker)
- 2 sensors (coffee roaster, smoker with food + chamber probes)
- 3+ sensors (multi-zone kiln, complex smoker)

```cpp
// config.h
#define NUM_TEMP_PROBES 2
#define PROBE_0_CS_PIN 4    // Primary (e.g., food/bean)
#define PROBE_0_LABEL "primary"
#define PROBE_1_CS_PIN 5    // Secondary (e.g., chamber/environment)
#define PROBE_1_LABEL "secondary"
```

#### C. Configurable Actuator Count
Currently hardcoded to 2 actuators (heater SSR + fan). Should support:
- 1 actuator (sous vide heater only)
- 2 actuators (coffee roaster: heater + fan, smoker: heater + damper)
- 3 actuators (kiln: heater + fan + damper)

```cpp
// config.h
#define NUM_ACTUATORS 2
#define ACTUATOR_0_PIN 33
#define ACTUATOR_0_TYPE "heater"   // SSR PWM
#define ACTUATOR_0_FREQ 5000
#define ACTUATOR_1_PIN 25
#define ACTUATOR_1_TYPE "fan"      // DC motor PWM
#define ACTUATOR_1_FREQ 5000
```

#### D. Configurable MQTT Topic Root
```cpp
// config.h
#define MQTT_TOPIC_ROOT "device/"  // or "roaster/", "smoker/", etc.
// Topics become: device/{client_id}/telemetry, device/{client_id}/control/*, etc.
```

#### E. Configurable Safety Thresholds
All safety limits already defined in config.h — just need renaming and per-probe support:
```cpp
#define SAFETY_MAX_TEMP_PROBE_0 240.0   // Coffee: 240°C, Smoker: 150°C
#define SAFETY_MAX_TEMP_PROBE_1 300.0
#define SAFETY_MAX_ROR 30.0             // °C/min
#define SAFETY_MIN_ACTUATOR_1 100       // Min fan PWM for heater operation
#define SAFETY_MQTT_TIMEOUT_MS 60000
#define SAFETY_SENSOR_FAIL_COUNT 3
```

#### F. Telemetry JSON Structure
Evolve from hardcoded field names to array-based:
```json
{
  "timestamp": 1709900000,
  "uptimeMs": 300000,
  "probes": [
    {"label": "primary", "temp": 185.5},
    {"label": "secondary", "temp": 145.2}
  ],
  "rateOfRise": 12.5,
  "actuators": [
    {"label": "heater", "value": 75},
    {"label": "fan", "value": 180}
  ],
  "setpoint": 200.0,
  "controlMode": 1,
  "pid": {"kp": 15.0, "ki": 1.0, "kd": 25.0},
  "systemStatus": 0,
  "freeHeap": 245760,
  "rssi": -45
}
```

**Backward compatibility:** Keep flat fields (`beanTemp`, `envTemp`, etc.) alongside array format during transition. Server accepts both.

#### G. Device Descriptor
On MQTT connect, device publishes a capability descriptor:
```json
{
  "deviceType": "coffee_roaster",
  "firmwareVersion": "3.1.0",
  "probes": [
    {"index": 0, "label": "bean", "type": "MAX6675"},
    {"index": 1, "label": "environment", "type": "MAX6675"}
  ],
  "actuators": [
    {"index": 0, "label": "heater", "type": "ssr_pwm", "range": [0, 100]},
    {"index": 1, "label": "fan", "type": "dc_motor", "range": [0, 255]}
  ],
  "capabilities": ["pid", "autotune", "manual", "auto"],
  "safetyLimits": {
    "maxTemp": [240.0, 300.0],
    "maxRoR": 30.0,
    "minFanForHeater": 100
  }
}
```

This lets rustRoast **auto-discover** device capabilities and render appropriate UI controls.

### What Stays the Same
- PID control algorithm (universal)
- Auto-tune Ziegler-Nichols (universal for any PID system)
- Safety architecture (over-temp, sensor fail, comms timeout, RoR limit)
- MQTT communication pattern (telemetry push, control subscribe)
- OTA updates
- NTP time sync
- Preferences-based PID storage
- Modular file structure

### Implementation Approach
1. Rename all coffee-specific variables/fields to generic names
2. Add `NUM_TEMP_PROBES` / `NUM_ACTUATORS` config with array-based probe/actuator management
3. Make MQTT topic root configurable
4. Add device descriptor publish on connect
5. Support both flat (legacy) and array-based telemetry JSON
6. Create device-specific config.h templates: `config.coffee_roaster.h`, `config.smoker.h`, `config.kiln.h`

---

## Part 2: rustRoast Multi-Device Platform

### Design Principle
**Generic plumbing, pluggable domain features.** The core platform handles device management, telemetry, sessions, and control. Device-type-specific features (coffee profiles, cupping notes, smoke ring analysis, etc.) are modular extensions.

### Architecture Layers

```
┌─────────────────────────────────────────────────────┐
│                 SvelteKit Dashboard                  │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐ │
│  │ Coffee   │ │ Smoker   │ │ Kiln     │ │ Custom │ │
│  │ Features │ │ Features │ │ Features │ │ Module │ │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └───┬────┘ │
│       └─────────┬──┴───────────┬┘            │      │
│          ┌──────┴──────────────┴──────────────┘      │
│          │    Generic Control & Monitoring UI        │
│          │  (device list, telemetry chart, sessions) │
│          └──────────────┬───────────────────┘        │
├─────────────────────────┼───────────────────────────┤
│                   Axum Backend                       │
│  ┌──────────────────────┴───────────────────────┐   │
│  │           Generic Device Platform             │   │
│  │  • Device registry & connections              │   │
│  │  • Telemetry ingestion & caching              │   │
│  │  • Session management (state machine)         │   │
│  │  • Generic control API                        │   │
│  │  • MQTT/WebSocket/Modbus adapters             │   │
│  └──────────────────────┬───────────────────────┘   │
│  ┌──────────┐ ┌──────────┴┐ ┌──────────┐           │
│  │ Coffee   │ │ Smoker    │ │ Kiln     │           │
│  │ Module   │ │ Module    │ │ Module   │           │
│  │• Profiles│ │• Smoke    │ │• Ramp    │           │
│  │• Cupping │ │  curves   │ │  sched   │           │
│  │• Events  │ │• Wood     │ │• Zone    │           │
│  │• AUC     │ │  types    │ │  control │           │
│  │• Phases  │ │• Stall    │ │• Cooling │           │
│  └──────────┘ │  detect   │ │  curves  │           │
│               └───────────┘ └──────────┘           │
├─────────────────────────────────────────────────────┤
│                    SQLite                            │
│  Generic: devices, telemetry, sessions, profiles    │
│  Per-type: coffee_*, smoker_*, kiln_* tables        │
└─────────────────────────────────────────────────────┘
```

### Core Changes Required

#### A. Device Type Abstraction
```sql
ALTER TABLE devices ADD COLUMN device_type TEXT NOT NULL DEFAULT 'coffee_roaster';
-- Values: 'coffee_roaster', 'smoker', 'kiln', 'sous_vide', 'generic'
```

#### B. Generic MQTT Topic Handling
- `topics.rs`: Make ROOT configurable per device (from device_connections config)
- Subscribe to multiple roots: `+/+/telemetry`, `+/+/status`
- Or use a single configurable root: `device/+/telemetry`

#### C. Generic Control API
```
POST /api/devices/{device_id}/control/{parameter}
  body: {"value": <number|string>}

# Replaces hardcoded:
#   /api/roaster/{device_id}/control/setpoint
#   /api/roaster/{device_id}/control/fan_pwm
#   etc.
```

Server validates `{parameter}` against the device's registered capabilities (from device descriptor or device profile).

#### D. Generic Telemetry
- Accept both flat (legacy) and array-based telemetry JSON
- Store as opaque JSON in telemetry table (already done)
- Device-type modules define how to extract/display fields

#### E. Pluggable Feature Modules (Rust Traits)
```rust
trait DeviceTypeModule {
    fn device_type(&self) -> &str;
    fn session_metadata_fields(&self) -> Vec<MetadataField>;
    fn event_types(&self) -> Vec<EventType>;
    fn profile_schema(&self) -> ProfileSchema;
    fn telemetry_field_labels(&self) -> Vec<FieldLabel>;
    fn validate_control_param(&self, param: &str, value: f64) -> Result<()>;
    fn compute_statistics(&self, session: &Session) -> serde_json::Value;
}
```

#### F. Dashboard Device-Type Routing
```svelte
<!-- Device page renders type-specific components -->
{#if device.device_type === 'coffee_roaster'}
  <CoffeeControls {device} />
  <PhaseStatsPanel {session} />
  <CuppingEditor {session} />
{:else if device.device_type === 'smoker'}
  <SmokerControls {device} />
  <SmokeLogPanel {session} />
{:else}
  <GenericControls {device} />
{/if}
```

### Preserved Coffee-Specific Features
All current coffee features stay intact, just scoped to `device_type = 'coffee_roaster'`:
- Roast profiles with first/second crack targets
- Phase detection (drying, Maillard, development)
- Cupping scores and notes
- AUC calculation
- Bean metadata (origin, variety, weight)
- Development time ratio
- Weight loss percentage
- Event detection (first crack, second crack, charge, drop)

### Example: Smoker Device Type
```
Sensors: food probe (primary), chamber probe (secondary), optional smoke box probe
Actuators: heater element, intake damper (or fan)
Events: StallStart, StallEnd, WrapTime, RestStart
Metadata: meat_type, wood_type, target_internal_temp, rub_notes
Profile: time-temp curve for chamber, target internal temp at completion
Statistics: total_cook_time, stall_duration, average_chamber_temp
```

### Example: Kiln Device Type
```
Sensors: multiple zone probes, exhaust probe
Actuators: heater elements (per zone), damper
Events: RampStart, HoldStart, CoolingStart
Metadata: clay_type, glaze_type, firing_type (bisque/glaze)
Profile: multi-segment ramp/hold/cool schedule
Statistics: peak_temp, total_firing_time, cooling_rate
```

---

## Implementation Roadmap

### Phase 1: Generic ESP32 Firmware (Near-term)
- Rename coffee-specific variables to generic names
- Array-based probe/actuator configuration
- Configurable MQTT topic root
- Device descriptor publish
- Create config templates per device type
- **No rustRoast changes needed** — backward-compatible telemetry

### Phase 2: rustRoast Generic Plumbing (Medium-term)
- Add `device_type` to devices table
- Generic control API (`/api/devices/{id}/control/{param}`)
- Configurable MQTT topic root per device
- Accept array-based telemetry alongside flat format
- Generic session management (type-agnostic state machine)

### Phase 3: rustRoast Coffee Module Isolation (Medium-term)
- Move coffee-specific models into a module/trait
- Coffee-specific session metadata, events, statistics scoped to type
- Profile schema parameterized by device type
- Dashboard components conditionally rendered by type

### Phase 4: New Device Type Modules (Long-term)
- Smoker module (meat types, wood types, stall detection)
- Kiln module (ramp/hold schedules, multi-zone)
- Generic module (any 2-probe, 2-actuator device)
- Each module: backend trait impl + frontend component set

---

## Key Design Decisions

1. **ESP32 firmware is ONE codebase** — config.h defines the device personality, not separate codebases
2. **Auto-tune stays on ESP32** — network resilience, real-time fidelity, simplicity
3. **rustRoast preserves ALL coffee features** — they're scoped to a module, not removed
4. **Telemetry is backward-compatible** — flat and array formats coexist during transition
5. **Device descriptor enables auto-discovery** — server learns device capabilities on connect
6. **Feature modules are additive** — adding a smoker module doesn't touch coffee code
