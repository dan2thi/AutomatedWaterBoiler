#include "connection_util.h"
#include "connection_secrets.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_system.h>

static WiFiClient espClient;
static PubSubClient mqttClient(espClient);

const char* bedSideLeakTopic = "mbr/bedside/leak";
const char* controllerStateTopic = "waterboiler/controllerstate";
const char* operationStatusTopic = "waterboiler/operationstatus";
const char* cycleTopic = "waterboiler/cyclecounter";  // MATCHES HOME ASSISTANT
const char* requestTopic = "waterboiler/request";
const char* authorizationTopic = "waterboiler/authorization";

static String pendingRequestID;
static String receivedDecision;
static bool responseReceived = false;
static uint32_t authorizationExpiresAt = 0;
static uint32_t windowEnd = 0;

uint32_t haCoolingSeconds = 0;
uint32_t haRetrySeconds = 0;

const char* getStateName(WaterSystemState state) {
  switch (state) {
    case STATE_INITIALIZING: return "Initializing";
    case STATE_CONNECTING_MQTT: return "Connecting to MQTT";
    case STATE_CONNECTED_MQTT: return "Connected to MQTT";
    case STATE_ACTIVE: return "Active";
    case STATE_ASLEEP: return "Asleep";
    case STATE_FILLING: return "Filling Kettle";
    case STATE_BOILING: return "Boiling Water";
    case STATE_COOLING: return "Cooling Water";
    case STATE_TRANSFERRING: return "Transferring Water to Glass Tank";
    case STATE_WAITING: return "Waiting for Off-Peak Window";
    case STATE_TANK_FULL: return "Glass Tank Full";
    case STATE_ERROR: return "Error";
    case SHUTDOWN: return "Shutdown";
    case LEAK_DETECTED: return "Leak Detected";
    default: return "Error";
  }
}

void publishToMQTT(const char* topic, const char* message, bool retain) {
  if (mqttClient.connected()) {
    mqttClient.publish(topic, message, retain);
    mqttClient.loop();
  }
}

void mqttDelay(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    if (mqttClient.connected()) mqttClient.loop();
    delay(10);
  }
}

void connectToWiFi() {
  delay(250);
  WiFi.mode(WIFI_STA);
  delay(250);
  WiFi.begin(ssid, password);
  delay(250);
  Serial.print("Connecting to Wi-Fi");
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; ++i) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connection failed. Resetting...");
    esp_sleep_enable_timer_wakeup(3 * 1000000ULL);
    esp_deep_sleep_start();
  }
  Serial.printf("\nWi-Fi connected: %s\n", WiFi.localIP().toString().c_str());
}

void connectMQTT() {
  delay(250);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(1024);
  mqttClient.setCallback(mqttCallback);
  for (int i = 0; i < 5 && !mqttClient.connected(); ++i) {
    if (mqttClient.connect("ESP32WaterHeaterClient", mqtt_user, mqtt_password)) break;
    delay(2000);
  }
  if (!mqttClient.connected()) {
    Serial.println("MQTT unavailable; sleeping for 3 seconds");
    esp_sleep_enable_timer_wakeup(3 * 1000000ULL);
    esp_deep_sleep_start();
  }
  if (!mqttClient.subscribe(authorizationTopic) || !mqttClient.subscribe(bedSideLeakTopic)) {
    Serial.println("MQTT subscription failed; sleeping for 3 seconds");
    esp_sleep_enable_timer_wakeup(3 * 1000000ULL);
    esp_deep_sleep_start();
  }
  mqttDelay(300);
  publishToMQTT(controllerStateTopic, getStateName(STATE_ACTIVE), true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, bedSideLeakTopic) == 0) {
    if (length == 2 && payload[0] == 'O' && payload[1] == 'N') {
      Serial.println("Water leak detected");
      delay(250);
      publishToMQTT(operationStatusTopic, getStateName(LEAK_DETECTED), true);
      delay(250);
      emergencyShutdownTriggered = true;
      shutdownSystem();
    }
    return;
  }
  if (strcmp(topic, authorizationTopic) != 0) return;

  JsonDocument doc;
  if (deserializeJson(doc, payload, length)) return;
  String id = doc["request_id"] | "";
  if (id.isEmpty() || id != pendingRequestID) return;
  receivedDecision = String(doc["decision"] | "");
  authorizationExpiresAt = doc["expires_at"] | 0UL;
  windowEnd = doc["window_end"] | 0UL;
  responseReceived = true;
  haCoolingSeconds = doc["sleep_seconds"] | 0;
  haRetrySeconds = doc["retry_seconds"] | 0;
}

String requestHA(const char* action, const String& cycleID) {
  if (!mqttClient.connected()) return "NO_CONNECTION";
  pendingRequestID = String((uint32_t)esp_random(), HEX) + "_" +
                     String((uint32_t)esp_random(), HEX);
  receivedDecision = "";
  responseReceived = false;
  authorizationExpiresAt = 0;
  windowEnd = 0;

  JsonDocument doc;
  doc["request_id"] = pendingRequestID;
  doc["action"] = action;
  if (cycleID.length()) doc["cycle_request_id"] = cycleID;
  String json;
  serializeJson(doc, json);
  if (!mqttClient.publish(requestTopic, json.c_str(), false)) return "PUBLISH_FAILED";

  unsigned long started = millis();
  while (!responseReceived && millis() - started < 10000UL) {
    if (!mqttClient.connected() || emergencyShutdownTriggered) return "DISCONNECTED";
    mqttClient.loop();
    delay(20);
  }
  return responseReceived ? receivedDecision : "TIMEOUT";
}

String lastRequestID() { return pendingRequestID; }
uint32_t lastAuthorizationExpiry() { return authorizationExpiresAt; }
uint32_t lastWindowEnd() { return windowEnd; }

void publishDiscovery() {
  const char* device =
    "\"device\":{"
      "\"identifiers\":[\"esp32_waterboiler_01\"],"
      "\"name\":\"Automated Water Boiling System\","
      "\"manufacturer\":\"DIY\","
      "\"model\":\"ESP32 Water Controller\""
    "}";
  char config[768];
  snprintf(config, sizeof(config),
    "{\"name\":\"Controller State\",\"unique_id\":\"waterboiler_controller_state\","
    "\"state_topic\":\"waterboiler/controllerstate\",\"icon\":\"mdi:kettle-steam\",%s}", device);
  publishToMQTT("homeassistant/sensor/waterboiler_controller_state/config", config, true);
  mqttDelay(300);
  snprintf(config, sizeof(config),
    "{\"name\":\"Operation Status\",\"unique_id\":\"waterboiler_operation_status\","
    "\"state_topic\":\"waterboiler/operationstatus\",\"icon\":\"mdi:power-sleep\",%s}", device);
  publishToMQTT("homeassistant/sensor/waterboiler_operation_status/config", config, true);
  mqttDelay(300);
  snprintf(config, sizeof(config),
    "{\"name\":\"Cycle Counter\",\"unique_id\":\"waterboiler_cycles_counter\","
    "\"state_topic\":\"waterboiler/cyclecounter\",\"icon\":\"mdi:counter\",%s}", device);
  publishToMQTT("homeassistant/sensor/waterboiler_cycles_counter/config", config, true);
  // Do not clear retained states or the HA-owned cycle counter here.
  mqttDelay(300);
  Serial.println("Published Home Assistant discovery messages");
}

uint32_t getHACoolingSeconds() {
  return haCoolingSeconds;
}

uint32_t getHARetrySeconds() {
  return haRetrySeconds;
}