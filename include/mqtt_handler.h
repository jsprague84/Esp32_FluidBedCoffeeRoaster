#pragma once

void initMQTT();
void publishTelemetry();
void publishStatus(const char* status);
void publishAutoTuneStatus();
void publishAutoTuneResults();
bool mqttIsConnected();
unsigned long getLastConnectedTime();
