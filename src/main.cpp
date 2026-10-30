#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <ESP32Servo.h>
#include "connection_util.h"

// Start with false. Change only after validating Home Assistant's
// fail-closed cooling logic, the hardware safeguards and the dry run.
constexpr bool ENABLE_ACTUATORS = true;

#define D4 4
#define D13 13
#define D14 14
#define D25 25
#define D26 26

#define MAX_SLEEP_SECONDS 10800

const int servoPin = D4;
const int waterPumpPin = D14;
const int solenoidPin = D13;
const int kettleWaterSensorPin = D25;
const int glassTankWaterSensorPin = D26;

// Preserve existing short test durations. Restore production values later.
constexpr uint32_t pumpTimeout = 135;
constexpr uint32_t solenoidTimeout = 60;
constexpr uint32_t waterBoilingTimePeriod = 500;
// constexpr uint32_t coolDownTimePeriod = 10800;  // Home Assistant uses 3 hours
// constexpr uint32_t coolDownTimePeriod = 8;  // testing
// constexpr uint32_t retrySleepSeconds = 900;
// constexpr uint32_t retrySleepSeconds = 7; // testing

bool emergencyShutdownTriggered = false;
Servo myServo;

void reportController(WaterSystemState state) {
  Serial.println("Reporting controller state: " + String(getStateName(state)));
  publishToMQTT(controllerStateTopic, getStateName(state), true);
}
void reportOperation(WaterSystemState state) {
  Serial.println("Reporting operation state: " + String(getStateName(state)));
  publishToMQTT(operationStatusTopic, getStateName(state), true);
}

uint32_t getCoolingDuration() {
  uint32_t seconds = getHACoolingSeconds();
  return seconds >= 1 && seconds <= MAX_SLEEP_SECONDS ? seconds : 0;
}

uint32_t getRetryDuration() {
  uint32_t seconds = getHARetrySeconds();
  return seconds >= 1 && seconds <= 3600 ? seconds : 3600;
}


void sleepForSeconds(uint32_t seconds) {
  // Always set actuators to their known inactive commands before sleeping.
  digitalWrite(waterPumpPin, LOW);
  digitalWrite(solenoidPin, LOW);
  if (ENABLE_ACTUATORS) myServo.write(0);
  mqttDelay(250);
  reportController(STATE_ASLEEP);
  mqttDelay(250);
  Serial.printf("Sleeping for %lu seconds\n", (unsigned long)seconds);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

void shutdownSystem() {
  Serial.println("Initiating emergency shutdown");
  delay(250);
  digitalWrite(waterPumpPin, LOW);
  delay(250);
  digitalWrite(solenoidPin, LOW);
  delay(250);
  if (ENABLE_ACTUATORS) myServo.write(0);
  delay(250);
  reportController(SHUTDOWN);
  delay(250);
  Serial.flush();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();
}

void initEsp32() {
  Serial.begin(115200);
  delay(250);
  pinMode(waterPumpPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(waterPumpPin, LOW);
  digitalWrite(solenoidPin, LOW);
  pinMode(kettleWaterSensorPin, INPUT);
  pinMode(glassTankWaterSensorPin, INPUT);
  if (ENABLE_ACTUATORS) {
    myServo.attach(servoPin);
    delay(500);
    myServo.write(0);
    delay(500);
  }
}

bool synchronizeTime() {
  configTime(-6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) return false;
  return time(nullptr) > 1700000000;
}

// Pump attempts transfer every authorized wake, including when kettle is empty.
// Timeout means attempt finished; it does not prove that water moved.
void transferWaterToTank() {
  reportController(STATE_ACTIVE);
  reportOperation(STATE_TRANSFERRING);
  if (!ENABLE_ACTUATORS) {
    Serial.println("DRY RUN: transfer attempt");
    mqttDelay(1000);
    return;
  }
  if (digitalRead(glassTankWaterSensorPin) == LOW) 
  {
    Serial.println("Glass tank is already full");
    return;
  }
  unsigned long start = millis();
  digitalWrite(waterPumpPin, HIGH);
  mqttDelay(250);
  while (millis() - start < pumpTimeout * 1000UL) {
    if (emergencyShutdownTriggered){
      Serial.println("Emergency shutdown triggered during transfer");
      break;
    }
    if (digitalRead(glassTankWaterSensorPin) == LOW) {
      Serial.println("Glass tank is now full");
      break;
    }
    mqttDelay(100);
  }
  digitalWrite(waterPumpPin, LOW);
  mqttDelay(250);
}

void fillWater() {
  reportController(STATE_ACTIVE);
  reportOperation(STATE_FILLING);
  mqttDelay(250);
  if (!ENABLE_ACTUATORS) {
    Serial.println("DRY RUN: filling kettle");
    mqttDelay(1000);
    return;
  }
  Serial.println("Filling kettle");
  mqttDelay(250);
  if (digitalRead(kettleWaterSensorPin) == LOW) {
    Serial.println("Kettle is already full");
    mqttDelay(250);
    return;
  }
  unsigned long start = millis();
  digitalWrite(solenoidPin, HIGH);
  mqttDelay(250);
  while (millis() - start < solenoidTimeout * 1000UL) {
    if (emergencyShutdownTriggered){
      Serial.println("Emergency shutdown triggered during water filling");
      break;
    }
    if (digitalRead(kettleWaterSensorPin) == LOW) {
      Serial.println("Kettle is now full");
      break;
    }
    mqttDelay(250);
  }
  digitalWrite(solenoidPin, LOW);
  mqttDelay(250);
  Serial.println("Water filling complete");
}

void boilWaterWithoutSleep() {
  reportController(STATE_ACTIVE);
  reportOperation(STATE_BOILING);
  mqttDelay(1000);
  if (!ENABLE_ACTUATORS) {
    Serial.println("DRY RUN: boiling kettle");
    mqttDelay(waterBoilingTimePeriod * 1000UL);
    return;
  }
  Serial.println("Boiling water in the kettle");
  delay(250);
  myServo.write(170);
  mqttDelay(1000);
  myServo.write(70);
  mqttDelay(waterBoilingTimePeriod * 1000UL);
  
  myServo.write(0);
  Serial.println("Boiling complete, kettle switched-off");
  mqttDelay(1000);
}

void runWaterBoiler() {
  String decision = requestHA("wake");
  delay(250);
  Serial.printf("Response for {wake} request: %s\n", decision.c_str());
  delay(250);
  if (decision == "WAIT") {
    reportOperation(STATE_COOLING);
    sleepForSeconds(getRetryDuration());
  }
  if (decision != "TRANSFER" &&
      decision != "READY" &&
      decision != "READY_SKIP_TRANSFER") {
    reportOperation(STATE_ERROR);
    sleepForSeconds(getRetryDuration());
  }
  // Skip transfer if HA confirms it was already completed
  if (decision == "READY_SKIP_TRANSFER") {
    Serial.println("Final transfer already completed. Skipping.");
  } else {
    transferWaterToTank();
    delay(250);
  }
  if (decision == "TRANSFER") {
    decision = requestHA("transfer_done");
    Serial.printf("Response for {transfer_done request}: %s\n", decision.c_str());
    if (decision != "READY") {
      reportOperation(STATE_ERROR);
      sleepForSeconds(getRetryDuration());
    }
  }
  bool glassTankFull = digitalRead(glassTankWaterSensorPin);
  delay(250);
  if (ENABLE_ACTUATORS && glassTankFull == LOW) {
    delay(250);
    Serial.println("Glass tank is full");
    delay(250);
    reportOperation(STATE_TANK_FULL);
    delay(250);
    sleepForSeconds(getRetryDuration());
  }
  delay(250);
  decision = requestHA("cycle");
  delay(250);
  Serial.printf("Response for {cycle} request: %s\n", decision.c_str());
  delay(250);
  if (decision != "APPROVED") {
    reportOperation(STATE_WAITING);
    sleepForSeconds(getRetryDuration());
  }
  const String cycleRequestID = lastRequestID();
  const uint32_t expiresAt = lastAuthorizationExpiry();
  const uint32_t closesAt = lastWindowEnd();


  fillWater();
  // if (!fillWater()) {
  //   reportOperation(STATE_ERROR);
  //   sleepForSeconds(retrySleepSeconds);
  // }

  // An approval is single-use and short-lived; check immediately before boiling.
  time_t nowEpoch = time(nullptr);
  if (nowEpoch < 1700000000 || expiresAt == 0 || closesAt == 0 ||
      (uint32_t)nowEpoch >= expiresAt ||
      ((uint64_t)nowEpoch + waterBoilingTimePeriod + 5ULL) >= closesAt) {
    Serial.println("Boiling permission expired / insufficient off-peak time");
    reportOperation(STATE_ERROR);
    sleepForSeconds(getRetryDuration());
  }

  boilWaterWithoutSleep(); // Always returns with kettle switch-off command sent.
  reportOperation(STATE_COOLING);

  decision = requestHA("boil_finished", cycleRequestID);
  Serial.printf("Cooling response: %s\n", decision.c_str());
  // Fail safely if HA unavailable. HA must NOT infer an actual switch-off
  // time from the original authorization timestamp.
  if (decision != "COOLING") reportOperation(STATE_ERROR);
  else reportOperation(STATE_COOLING);
  uint32_t coolingSeconds = getCoolingDuration();
  if (coolingSeconds == 0) {
    reportOperation(STATE_ERROR);
    sleepForSeconds(MAX_SLEEP_SECONDS);
  }

  sleepForSeconds(coolingSeconds);
}

void setup() {
  initEsp32();
  connectToWiFi();
  connectMQTT();
  publishDiscovery();
  reportController(STATE_ACTIVE);
  if (!synchronizeTime()) {
    Serial.println("Time sync failed; no heating authorized");
    reportOperation(STATE_ERROR);
    sleepForSeconds(getRetryDuration());
  }
  runWaterBoiler();
}

void loop() {} // Each wake is handled once from setup().

