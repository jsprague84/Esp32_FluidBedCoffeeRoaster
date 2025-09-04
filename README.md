# ESP32 Coffee Roaster - MQTT-Only Version

A streamlined ESP32-based hardware controller for a modified popcorn popper coffee roaster. This version focuses on core roasting functionality with MQTT communication for external system integration.

## Hardware Components
- **ESP32 Development Board** (NodeMCU-32S)
- **2x MAX6675 modules** with K-type thermocouples for bean and environment temperature monitoring
- **Solid State Relay (SSR)** with PWM for heater control
- **L298N motor driver** with PWM for DC fan control
- **Modified popcorn popper** as roasting chamber base

## Key Features
- **MQTT-only communication** - streamlined for integration with external systems
- **PID temperature control** with automatic and manual modes
- **Real-time telemetry** publishing at 1Hz
- **Safety systems** including fan failsafe and emergency stop
- **OTA firmware updates** for remote maintenance
- **Persistent configuration** stored in EEPROM

## Communication Protocol
Uses MQTT for all external communication, eliminating Modbus TCP and web server dependencies. Designed for integration with:
- Bridge applications for Artisan compatibility
- Node-RED for visual programming
- Custom dashboard applications
- IoT monitoring systems

## Recent Changes (MQTT Integration Branch)
- ✅ **Removed**: Modbus TCP server and AsyncTCP dependencies
- ✅ **Removed**: Built-in web server for simplified architecture  
- ✅ **Enhanced**: MQTT communication with comprehensive telemetry
- ✅ **Streamlined**: Core control functions only
- ✅ **Optimized**: Reduced memory footprint and improved reliability
