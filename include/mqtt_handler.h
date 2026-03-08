#pragma once

void initMQTT();
void publishTelemetry();
void publishStatus(const char* status);
bool mqttIsConnected();
unsigned long getLastConnectedTime();
void mqttPublishAutoTuneStatus(const char* buf, size_t len);
void mqttPublishAutoTuneResults(const char* buf, size_t len);
