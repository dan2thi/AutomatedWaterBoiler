#ifndef CONNECTION_UTIL_H
#define CONNECTION_UTIL_H

#include <Arduino.h>

extern const char* bedSideLeakTopic;
extern const char* controllerStateTopic;
extern const char* operationStatusTopic;
extern const char* cycleTopic;
extern const char* requestTopic;
extern const char* authorizationTopic;

enum WaterSystemState {
  STATE_INITIALIZING, 
  STATE_CONNECTING_MQTT, STATE_CONNECTED_MQTT, STATE_ACTIVE,
  STATE_ASLEEP, STATE_FILLING, STATE_BOILING, STATE_COOLING,
  STATE_TRANSFERRING, STATE_WAITING, STATE_TANK_FULL, STATE_ERROR, SHUTDOWN, LEAK_DETECTED
};

void connectToWiFi();
void connectMQTT();
void mqttDelay(unsigned long duration);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void shutdownSystem();
void publishToMQTT(const char* topic, const char* payload, bool retain = false);
void publishDiscovery();
const char* getStateName(WaterSystemState state);
uint32_t getHACoolingSeconds();
uint32_t getHARetrySeconds();

// New Home Assistant authorization API
String requestHA(const char* action, const String& cycleID = "");
String lastRequestID();
uint32_t lastAuthorizationExpiry();
uint32_t lastWindowEnd();

extern bool emergencyShutdownTriggered;
#endif
