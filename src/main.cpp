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
constexpr uint32_t pumpTimeout = 120;
constexpr uint32_t solenoidTimeout = 65;
constexpr uint32_t waterBoilingTimePeriod = 500;
// constexpr uint32_t coolDownTimePeriod = 10800;  // Home Assistant uses 3 hours
// constexpr uint32_t coolDownTimePeriod = 8;  // testing
// constexpr uint32_t retrySleepSeconds = 900;
// constexpr uint32_t retrySleepSeconds = 7; // testing

bool emergencyShutdownTriggered = false;
Servo myServo;

void reportController(WaterSystemState state) {
  RemoteLog.println("Reporting controller state: " + String(getStateName(state)));
  publishToMQTT(controllerStateTopic, getStateName(state), true);
}
void reportOperation(WaterSystemState state) {
  RemoteLog.println("Reporting operation state: " + String(getStateName(state)));
  publishToMQTT(operationStatusTopic, getStateName(state), true);
}

uint32_t getCoolingDuration() {
  uint32_t seconds = getHACoolingSeconds();
  return seconds >= 1 && seconds <= MAX_SLEEP_SECONDS ? seconds : 0;
}

uint32_t getRetryDuration() {
  uint32_t seconds = getHARetrySeconds();
  return seconds >= 1 && seconds <= MAX_SLEEP_SECONDS ? seconds : MAX_SLEEP_SECONDS;
}


void sleepForSeconds(uint32_t seconds) {
  // Always set actuators to their known inactive commands before sleeping.
  digitalWrite(waterPumpPin, LOW);
  digitalWrite(solenoidPin, LOW);
  if (ENABLE_ACTUATORS) myServo.write(0);
  mqttDelay(250);
  reportController(STATE_ASLEEP);
  mqttDelay(250);
  RemoteLog.printf("Sleeping for %lu seconds\n", (unsigned long)seconds);
  Serial.flush();
  flushRemoteLogs();
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

void shutdownSystem() {
  RemoteLog.println("Initiating emergency shutdown");
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
  flushRemoteLogs();
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
    RemoteLog.println("DRY RUN: transfer attempt");
    mqttDelay(1000);
    return;
  }
  if (digitalRead(glassTankWaterSensorPin) == LOW) 
  {
    RemoteLog.println("Glass tank is already full");
    return;
  }
  unsigned long start = millis();
  digitalWrite(waterPumpPin, HIGH);
  mqttDelay(250);
  while (millis() - start < pumpTimeout * 1000UL) {
    if (emergencyShutdownTriggered){
      RemoteLog.println("Emergency shutdown triggered during transfer");
      break;
    }
    if (digitalRead(glassTankWaterSensorPin) == LOW) {
      RemoteLog.println("Glass tank is now full");
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
    RemoteLog.println("DRY RUN: filling kettle");
    mqttDelay(1000);
    return;
  }
  RemoteLog.println("Filling kettle");
  mqttDelay(250);
  if (digitalRead(kettleWaterSensorPin) == LOW) {
    RemoteLog.println("Kettle is already full");
    mqttDelay(250);
    return;
  }
  unsigned long start = millis();
  digitalWrite(solenoidPin, HIGH);
  mqttDelay(250);
  while (millis() - start < solenoidTimeout * 1000UL) {
    if (emergencyShutdownTriggered){
      RemoteLog.println("Emergency shutdown triggered during water filling");
      shutdownSystem();
    }
    if (digitalRead(kettleWaterSensorPin) == LOW) {
      delay(100);
      RemoteLog.println("Kettle is now full");
      break;
    }
    mqttDelay(250);
  }
  digitalWrite(solenoidPin, LOW);
  mqttDelay(250);
  if (digitalRead(kettleWaterSensorPin) == LOW) {
    RemoteLog.println("Water filling complete");
  }
  else {
    RemoteLog.println("Error : Kettle is not full");
    delay(100);
    reportController(STATE_ASLEEP);
    reportOperation(ERROR_WATER_FILLING);
    sleepForSeconds(getRetryDuration());
  }
  
}

void boilWaterWithoutSleep() {
  reportController(STATE_ACTIVE);
  reportOperation(STATE_BOILING);
  mqttDelay(1000);
  if (!ENABLE_ACTUATORS) {
    RemoteLog.println("DRY RUN: boiling kettle");
    mqttDelay(waterBoilingTimePeriod * 1000UL);
    return;
  }
  RemoteLog.println("Boiling water in the kettle");
  delay(250);
  myServo.write(170);
  mqttDelay(1000);
  myServo.write(70);
  mqttDelay(waterBoilingTimePeriod * 1000UL);
  
  myServo.write(0);
  RemoteLog.println("Boiling complete, kettle switched-off");
  mqttDelay(1000);
}


void runWaterBoiler() {

  String decision = requestHA("wake");
  delay(250);
  RemoteLog.printf("Response for {wake} request: %s\n", decision.c_str());
  delay(250);

  // Cooling is not complete. Do not transfer water.
  if (decision == "WAIT") {
    reportOperation(STATE_COOLING);
    sleepForSeconds(getRetryDuration());
    return;
  }

  // Reject unexpected or invalid responses.
  if (decision != "TRANSFER" &&
      decision != "READY" &&
      decision != "READY_SKIP_TRANSFER") {

    reportOperation(ERROR_UNIDENTIFIED_RESPONSE);
    sleepForSeconds(getRetryDuration());
    return;
  }

  // Transfer water ONLY when HA explicitly authorizes it.
  if (decision == "TRANSFER") {

    RemoteLog.println("HA authorized water transfer.");

    transferWaterToTank();
    delay(250);

    // Notify Home Assistant that transfer has finished.
    decision = requestHA("transfer_done");

    RemoteLog.printf("Response for {transfer_done} request: %s\n",
                  decision.c_str());

    if (decision != "READY") {
      reportOperation(STATE_ERROR);
      sleepForSeconds(getRetryDuration());
      return;
    }

  } else if (decision == "READY" ||
             decision == "READY_SKIP_TRANSFER") {

    RemoteLog.println("No pending transfer. Skipping water pump.");
  }

  // Check whether the glass storage tank is full.
  bool glassTankFull = digitalRead(glassTankWaterSensorPin);
  delay(250);

  if (ENABLE_ACTUATORS && glassTankFull == LOW) {

    RemoteLog.println("Glass tank is full");
    delay(250);

    reportOperation(STATE_TANK_FULL);
    delay(250);

    sleepForSeconds(getRetryDuration());
    return;
  }

  // Request authorization for another boiling cycle.
  delay(250);
  decision = requestHA("cycle");
  delay(250);

  RemoteLog.printf("Response for {cycle} request: %s\n",
                decision.c_str());
  delay(250);

  if (decision != "APPROVED") {

    if (decision == "MAX_CYCLES_REACHED") {
      RemoteLog.println("Maximum cycles reached for current window.");
      publishToMQTT(operationStatusTopic, "Maximum Cycles Reached", true);
    } else {
      reportOperation(STATE_WAITING);
    }

    sleepForSeconds(getRetryDuration());
    return;
  }

  // Store authorization information for this cycle.
  const String cycleRequestID = lastRequestID();
  const uint32_t expiresAt = lastAuthorizationExpiry();
  const uint32_t closesAt = lastWindowEnd();

  // Fill the kettle.
  fillWater();

  // Ensure the kettle contains water before heating.
  if (ENABLE_ACTUATORS &&
      digitalRead(kettleWaterSensorPin) != LOW) {

    RemoteLog.println("ERROR: Kettle filling failed. Boiling cancelled.");

    reportOperation(ERROR_WATER_FILLING);
    sleepForSeconds(getRetryDuration());
    return;
  }

  // Authorization is single-use and short-lived.
  // Check it again immediately before boiling.
  time_t nowEpoch = time(nullptr);

  if (nowEpoch < 1700000000 ||
      expiresAt == 0 ||
      closesAt == 0 ||
      (uint32_t)nowEpoch >= expiresAt ||
      ((uint64_t)nowEpoch + waterBoilingTimePeriod + 5ULL) >= closesAt) {

    RemoteLog.println("Boiling permission expired / insufficient off-peak time");

    reportOperation(STATE_ERROR);
    sleepForSeconds(getRetryDuration());
    return;
  }

  // Begin boiling. This function must switch the
  // kettle off before returning.
  boilWaterWithoutSleep();

  reportOperation(STATE_COOLING);

  // Notify Home Assistant that boiling has finished.
  decision = requestHA("boil_finished", cycleRequestID);

  RemoteLog.printf("Cooling response: %s\n", decision.c_str());

  // Home Assistant must confirm the cooling period.
  if (decision != "COOLING") {

    RemoteLog.println("ERROR: Cooling confirmation unavailable.");

    reportOperation(ERROR_COOLING_CONFIRMATION_UNAVAILABLE);
    sleepForSeconds(MAX_SLEEP_SECONDS);
    return;
  }

  reportOperation(STATE_COOLING);

  // Obtain cooling duration from Home Assistant.
  uint32_t coolingSeconds = getCoolingDuration();

  if (coolingSeconds == 0) {

    RemoteLog.println("ERROR: Invalid cooling duration.");

    reportOperation(ERROR_INVALID_COOLING_DURATION);
    sleepForSeconds(MAX_SLEEP_SECONDS);
    return;
  }

  RemoteLog.printf("Cooling for %lu seconds\n",
                (unsigned long)coolingSeconds);

  sleepForSeconds(coolingSeconds);
}

void setup() {
  initEsp32();
  connectToWiFi();
  connectMQTT();
  publishDiscovery();
  reportController(STATE_ACTIVE);
  if (!synchronizeTime()) {
    RemoteLog.println("Time sync failed; no heating authorized");
    reportOperation(ERROR_TIME_SYNC_FAILURE);
    sleepForSeconds(getRetryDuration());
  }
  runWaterBoiler();
}

void loop() {} // Each wake is handled once from setup().

