# ESP32 Coffee Roaster Control System

A streamlined ESP32-based hardware controller for a modified popcorn popper coffee roaster. This system provides real-time temperature monitoring, PID-based automatic control, and MQTT communication for integration with external roasting software like Artisan.

## Overview

This project transforms a popcorn popper into a precision coffee roaster by adding:
- **Dual temperature monitoring** using MAX6675 modules with K-type thermocouples
- **PWM-controlled heater** via solid-state relay (SSR)
- **Variable-speed fan control** via L298N motor driver
- **PID temperature control** for automated roasting profiles
- **MQTT communication** for remote monitoring and control
- **Safety features** including fan failsafe and emergency stop

## Hardware Requirements

### Core Components
- **ESP32 development board** (NodeMCU-32S recommended)
- **2x MAX6675 thermocouple amplifiers** for temperature sensing
- **2x K-type thermocouples** (one for bean temperature, one for environment)
- **Solid State Relay (SSR)** for heater control
- **L298N motor driver** for fan PWM control
- **Modified popcorn popper** as the base roasting chamber

### Wiring Configuration
```
ESP32 Pin  | Component        | Purpose
-----------|------------------|------------------
GPIO 4     | MAX6675 #1 CS    | Bean thermocouple
GPIO 5     | MAX6675 #2 CS    | Environment thermocouple
GPIO 33    | SSR Signal       | Heater control
GPIO 25    | L298N PWM        | Fan speed control
3.3V       | MAX6675 VCC      | Power supply
GND        | Common Ground    | Ground reference
```

## Software Architecture

### Core Features
- **Real-time temperature monitoring** at 1Hz sampling rate
- **Dual control modes**: Manual PWM control or automatic PID control
- **Rate of Rise (ROR) calculation** for roast profiling
- **MQTT telemetry publishing** every 1000ms
- **Safety systems** with watchdog timer and fan failsafe
- **OTA firmware updates** for remote maintenance
- **Persistent PID parameter storage** in EEPROM

### MQTT Communication Protocol

#### Published Topics

**Telemetry Data** (`roaster/{device_id}/telemetry`)
```json
{
  "timestamp": 123456,
  "beanTemp": 185.5,
  "envTemp": 145.2,
  "rateOfRise": 12.5,
  "heaterPWM": 75,
  "fanPWM": 180,
  "setpoint": 200.0,
  "controlMode": 1,
  "heaterEnable": 1,
  "uptime": 1234,
  "Kp": 15.0,
  "Ki": 1.0,
  "Kd": 25.0,
  "freeHeap": 245760,
  "rssi": -45,
  "systemStatus": 0
}
```

**Status Updates** (`roaster/{device_id}/status`)
```json
{
  "status": "online",
  "id": "esp32_roaster_01-ABC123",
  "ip": "192.168.1.100",
  "rssi": -45,
  "freeHeap": 245760,
  "version": "2.0.0-mqtt-only"
}
```

#### Subscribed Control Topics

| Topic | Payload | Description |
|-------|---------|-------------|
| `roaster/{device_id}/control/setpoint` | `200.0` | Target bean temperature (°C) |
| `roaster/{device_id}/control/fan_pwm` | `180` | Fan PWM value (0-255) |
| `roaster/{device_id}/control/heater_pwm` | `75` | Manual heater PWM percentage (0-100) |
| `roaster/{device_id}/control/mode` | `"auto"` or `"manual"` | Control mode selection |
| `roaster/{device_id}/control/heater_enable` | `"1"` or `"0"` | Enable/disable heater |
| `roaster/{device_id}/control/pid` | `{"kp":15.0,"ki":1.0,"kd":25.0}` | PID parameters |
| `roaster/{device_id}/control/emergency_stop` | `"1"` | Emergency stop trigger |

## Installation and Setup

### 1. Development Environment Setup

Install PlatformIO Core or PlatformIO IDE:
```bash
# Using Python pip
pip install platformio

# Or install PlatformIO IDE extension in VS Code
```

### 2. Clone and Configure

```bash
git clone <repository_url>
cd Esp32_FluidBedCoffeeRoaster
```

Copy and edit the configuration file:
```bash
cp include/config.example.h include/config.h
```

Edit `include/config.h` with your settings:
```cpp
// WiFi Configuration
const char* ssid = "your-wifi-network";
const char* password = "your-wifi-password";

// MQTT Configuration  
#define MQTT_BROKER "192.168.1.100"
#define MQTT_CLIENT_ID "esp32_roaster_01"
```

### 3. Hardware Pin Configuration

Verify pin assignments in `include/config.h` match your wiring:
```cpp
#define BT_CS_PIN   4    // Bean thermocouple CS
#define ET_CS_PIN   5    // Environment thermocouple CS  
#define SSR_PIN     33   // Heater SSR control
#define FAN_PIN     25   // Fan PWM control
```

### 4. Build and Flash

```bash
# Build the project
pio run

# Upload to ESP32 (connect via USB)
pio run --target upload

# Monitor serial output
pio device monitor
```

## Operation Guide

### Control Modes

#### Manual Mode
- Direct PWM control of heater and fan
- User sets heater PWM percentage (0-100%)
- User sets fan PWM value (0-255)
- No automatic temperature regulation

#### Automatic Mode (PID)
- PID controller maintains target bean temperature
- User sets desired setpoint temperature
- System automatically adjusts heater PWM
- Fan speed remains manually controlled

### Safety Features

#### Fan Failsafe
- Heater automatically disabled if fan PWM < 100
- Prevents overheating by ensuring adequate airflow
- Cannot be overridden via software

#### Temperature Limits
- Bean temperature range: 0-240°C
- Environment temperature range: 0-150°C  
- Invalid readings trigger sensor error state

#### Emergency Stop
- Immediately disables heater
- Sets fan to maximum speed (PWM 255)
- Can be triggered via MQTT or system failure

#### Watchdog Timer
- 30-second hardware watchdog
- System resets if main loop freezes
- Prevents unsafe stuck states

### Monitoring and Diagnostics

#### Serial Debug Output
When connected via USB, the system outputs diagnostic information:
```
System Status:
Mode: Auto, BT: 185.5°C, ET: 145.2°C, ROR: 12.50°C/min
Heater: 75, Fan: 180, Enabled: 1, MQTT: OK
```

#### System Status Codes
- `0`: SYSTEM_OK - All systems operational
- `1`: WIFI_ERROR - WiFi connection failed
- `2`: MQTT_ERROR - MQTT broker disconnected  
- `3`: SENSOR_ERROR - Thermocouple reading invalid
- `4`: SAFETY_ERROR - Safety system triggered

## Calibration and Tuning

### Temperature Calibration

Adjust temperature offsets in `include/config.h`:
```cpp
#define TEMP_CALIBRATION_BEAN 2.5   // Add 2.5°C to bean readings
#define TEMP_CALIBRATION_ENV -1.0   // Subtract 1.0°C from environment
```

### PID Tuning

Default PID parameters (good starting point):
```cpp
#define DEFAULT_KP 15.0  // Proportional gain
#define DEFAULT_KI 1.0   // Integral gain  
#define DEFAULT_KD 25.0  // Derivative gain
```

#### Tuning Process
1. Start with manual mode to understand system response
2. Set initial PID values and switch to auto mode
3. Observe setpoint tracking and adjust:
   - **Kp**: Increase for faster response, decrease if oscillating
   - **Ki**: Increase to eliminate steady-state error
   - **Kd**: Increase to reduce overshoot and oscillation

4. Fine-tune via MQTT during operation:
```bash
mosquitto_pub -h broker_ip -t "roaster/esp32_roaster_01/control/pid" \
  -m '{"kp":18.0,"ki":1.2,"kd":30.0}'
```

## Integration with External Software

### Artisan Coffee Roaster
This ESP32 system is designed to work with the separate `coffee-roaster-bridge` Rust application, which:
- Converts MQTT data to Modbus TCP for Artisan compatibility
- Provides web interface for monitoring
- Handles data logging and roast profiles

### Custom Applications
Direct MQTT integration allows custom software to:
- Subscribe to real-time telemetry data
- Send control commands 
- Implement advanced roasting algorithms
- Create custom monitoring dashboards

## Troubleshooting

### Common Issues

#### WiFi Connection Fails
1. Verify SSID and password in `config.h`
2. Check signal strength at ESP32 location
3. Monitor serial output for connection attempts
4. Try 2.4GHz network (ESP32 doesn't support 5GHz)

#### MQTT Connection Fails  
1. Verify broker IP address and port
2. Check network connectivity to broker
3. Ensure broker allows anonymous connections
4. Check firewall settings

#### Temperature Readings Invalid
1. Verify thermocouple connections
2. Check MAX6675 wiring and power supply
3. Ensure thermocouples are K-type
4. Test with multimeter for continuity

#### Heater Not Responding
1. Check SSR wiring and power supply
2. Verify `heaterEnable` is set to 1
3. Ensure fan PWM is above safety minimum
4. Check control mode (manual vs auto)

### Debug Commands

#### MQTT Test Commands
```bash
# Set manual mode
mosquitto_pub -h broker_ip -t "roaster/esp32_roaster_01/control/mode" -m "manual"

# Set heater PWM to 50%
mosquitto_pub -h broker_ip -t "roaster/esp32_roaster_01/control/heater_pwm" -m "50"

# Enable heater
mosquitto_pub -h broker_ip -t "roaster/esp32_roaster_01/control/heater_enable" -m "1"

# Emergency stop
mosquitto_pub -h broker_ip -t "roaster/esp32_roaster_01/control/emergency_stop" -m "1"
```

#### Monitor Telemetry
```bash
mosquitto_sub -h broker_ip -t "roaster/esp32_roaster_01/telemetry" -v
```

### Recovery Options

#### Factory Reset
To reset PID parameters to defaults:
1. Flash firmware with `pio run --target upload`
2. Parameters will be restored to `config.h` defaults

#### OTA Updates
For remote firmware updates:
1. Uncomment OTA settings in `platformio.ini`
2. Set correct IP address
3. Use `pio run --target upload` with ESP32 on network

## API Reference

### Configuration Constants

#### Timing Parameters (milliseconds)
- `TEMP_READ_INTERVAL`: 1000 - Temperature sensor reading frequency
- `PID_COMPUTE_INTERVAL`: 100 - PID calculation frequency  
- `MQTT_PUBLISH_INTERVAL`: 1000 - Telemetry publishing rate
- `MQTT_RECONNECT_INTERVAL`: 5000 - MQTT reconnection attempts

#### Safety Limits
- `SAFETY_MIN_FAN_PWM`: 100 - Minimum fan speed for heater operation
- `MAX_BEAN_TEMP`: 240.0 - Maximum allowable bean temperature
- `MAX_HEATER_PWM`: 100 - Maximum heater PWM percentage

### System Functions

#### Temperature Management
- `readTemperature()`: Read calibrated temperature from thermocouples
- `updateRateOfRise()`: Calculate temperature change rate
- `getRateOfRise()`: Get current rate of rise in °C/minute

#### Control System  
- `beanPID.Compute()`: Execute PID control algorithm
- `handleControlMode()`: Switch between manual/automatic modes
- `handleEmergencyStop()`: Execute emergency shutdown sequence

#### Communication
- `publishMQTTTelemetry()`: Send sensor data and system status
- `connectMQTT()`: Establish MQTT broker connection
- `mqttCallback()`: Process incoming control commands

## Advanced Features

### Roast Profiling Support
The system provides comprehensive data for roast profiling:
- **Bean Temperature (BT)**: Primary roasting metric
- **Environment Temperature (ET)**: Chamber air temperature  
- **Rate of Rise (ROR)**: Temperature change velocity
- **Time-stamped data**: Millisecond precision timing
- **System parameters**: Real-time PID and PWM values

### Data Logging Integration
MQTT telemetry can be captured by external systems for:
- Historical roast analysis
- Profile comparison and optimization
- Quality control tracking
- Equipment performance monitoring

### Custom Control Algorithms
The MQTT interface enables external control systems to:
- Implement advanced roasting profiles
- Coordinate multiple roaster units
- Apply machine learning algorithms
- Integrate with inventory management systems

## License and Contributing

This project is open source. Contributions are welcome for:
- Bug fixes and stability improvements
- Additional safety features
- Enhanced monitoring capabilities
- Integration with new roasting software

## Support

For technical support and questions:
1. Check the troubleshooting section above
2. Review serial debug output for error messages
3. Verify hardware connections and configuration
4. Test MQTT communication with external tools

---

**Version**: 2.0.0-mqtt-only  
**Last Updated**: 2025  
**Compatible Hardware**: ESP32 NodeMCU-32S, MAX6675, K-type thermocouples