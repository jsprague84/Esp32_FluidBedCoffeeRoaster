#include "MAX6675Handler.h"
#include <SPI.h>

// Constructor: Initialize with the CS pin
MAX6675Handler::MAX6675Handler(int csPin) : _csPin(csPin) {}

// Initialize the MAX6675 module
void MAX6675Handler::begin() {
    pinMode(_csPin, OUTPUT);  // Set CS pin as output
    digitalWrite(_csPin, HIGH); // Deselect MAX6675 by default
}

// Read raw data from the MAX6675 module
uint16_t MAX6675Handler::readRawData() {
    uint16_t rawData;

    // Start SPI transaction
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW); // Select the MAX6675 module
    delayMicroseconds(1); // Short delay for stability

    rawData = SPI.transfer16(0x00); // Read 16 bits from the MAX6675
    digitalWrite(_csPin, HIGH); // Deselect the MAX6675 module
    SPI.endTransaction(); // End SPI transaction

    return rawData;
}

// Read temperature in Celsius from the MAX6675 module
float MAX6675Handler::readTemperature() {
    uint16_t rawData = readRawData();

    // Check for thermocouple connection error
    if (rawData & 0x4) {
        Serial.println("Error: Thermocouple not connected.");
        return NAN; // Return NAN if no thermocouple is connected
    }

    // Shift data to remove the 3 least significant bits and convert to °C
    rawData >>= 3;
    return rawData * 0.25;
}