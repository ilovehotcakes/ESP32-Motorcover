#include <Arduino.h>
#include <math.h>
#include <string>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Preferences.h>
#include "variables.h"
#include "my_preferences.h"
// #include "motor.h"

enum ConnectionState {
  initializing,
  reconnectingWifi,
  connectingMqtt,
  readingMqttMessage
};

bool VERBOSE = true;
TaskHandle_t C0;  // Dual core setup
void core0Task(void * parameter);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
ConnectionState connection;
void startWifi();
void readState();
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDR);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


void setup() {
  // Initialize hardware serial for debugging
  if (VERBOSE) Serial.begin(9600);

  // For the blue LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  
  SERIAL_PORT.begin(115200);  // Initialize hardware serial for hardware UART driver
  driver.pdn_disable(true);   // Enable UART on TMC2209
  driver.begin();             // Begin sending data
  driver.toff(4);             // Enables driver in software
  driver.rms_current(600);    // Motor RMS current "rms_current will by default set ihold to 50% of irun but you can set your own ratio with additional second argument; rms_current(1000, 0.3)."
  driver.pwm_autoscale(true);    // Needed for stealthChop
  driver.en_spreadCycle(false);  // Toggle spreadCycle on TMC2208/2209/2224
  driver.blank_time(24);
  driver.microsteps(microsteps);


  // Motor setup
  stepper.setEnablePin(EN_PIN);
  stepper.setMaxSpeed(velocity);
  stepper.setAcceleration(acceleration);
  stepper.setPinsInverted(false, false, true);
  stepper.setCurrentPosition(current_position);
  stepper.disableOutputs();

  // Core 0 setup for dual core operation
  disableCore0WDT();  // Disable watchdog timer
  xTaskCreatePinnedToCore(core0Task, "Core_0", 8192, NULL, 1, &C0, 0);

  startWifi();
  connection = connectingMqtt;

  // Load preferences from memory
  // preferences_local.begin("local", false);
  // load_preference();
  
  // sendPercentage();
}





















void updatePosition() {
  current_position = stepper.currentPosition();
  preferences_local.putInt("current_position", current_position);
  // open_percent = stepsToPercent(current_position);
}


// void sendPercentage() {
//   mqttClient.beginMessage(subtopic);
//   mqttClient.print(open_percent);
//   mqttClient.endMessage();
//   Serial.print("Sent MQTT message: ");
//   Serial.print(open_percent);
//   Serial.print("% = ");
//   Serial.print(stepper.currentPosition());
//   Serial.print(" / ");
//   Serial.println(max_steps);
// }


void moveToPosition(int position) {
  stepper.moveTo(position);
  if (stepper.distanceToGo() != 0) {
    motor = MOTOR_ENABLE;
    stepper.enableOutputs();
  }
}

void stopMotor() {
  motor = MOTOR_DISABLE;
  if (set_max == true) {
    set_max = false;
    max_steps = stepper.currentPosition();
    preferences_local.putInt("max_steps", max_steps);
  } else if (set_min == true) {
    set_min = false;
    int distance_traveled = 2147483646 - stepper.currentPosition();
    max_steps = max_steps + distance_traveled - previous_position;
    stepper.setCurrentPosition(0);
  }
  stepper.moveTo(stepper.currentPosition());
  stepper.disableOutputs();
  stepper.setMaxSpeed(velocity);
  updatePosition();
  // sendPercentage();
}


int percentToSteps(int percent) {
  float result = (float) percent * (float) max_steps / 100.0;
  return (int) round(result);
}


int stepsToPercent(int steps) {
  float result = (float) current_position / (float) max_steps * 100;
  return (int) round(result);
}


void setMax() {
  set_max = true;
  stepper.setMaxSpeed(velocity / 4);
  moveToPosition(2147483646);
}


void setMin() {
  set_min = true;
  stepper.setMaxSpeed(velocity / 4);
  previous_position = stepper.currentPosition();
  stepper.setCurrentPosition(2147483646);
  moveToPosition(0);
}






void loop() {
  switch(motor) {
    case MOTOR_ENABLE:
      if (stepper.distanceToGo()!= 0)
        stepper.run();
      else
        stopMotor();
    break;
  }
}
































void core0Task(void * parameter) {
  for (;;) readState();
}


// Todo: add timeout and restart
void startWifi() {
  Serial.println((String) "[E] Attempting to connect to WPA SSID: " + ssid);

  while (WiFi.begin(ssid, pass) != WL_CONNECTED)
    delay(5000);
  
  Serial.print("[E] You're connected to the WiFi! IP: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* buf, unsigned int len) {
  String message = "";
  for (int i = 0; i < len; i++) message += (char) buf[i];
  int command = message.toInt();

  Serial.println((String) "Received a command: " + command);

  if (command >= 0) {
    moveToPosition(percentToSteps(command));
  } else if (command == COVER_STOP) {
    cover = COVER_STOP;
  } else if (command == COVER_CLOSE) {
    moveToPosition(0);
    cover = COVER_CLOSE;
  } else if (command == COVER_OPEN) {
    moveToPosition(max_steps);
    cover = COVER_OPEN;
  } else if (command == COVER_SET_MAX) {
    setMax();
  } else if (command == COVER_SET_MIN) {
    setMin();
  }
}


// Todo: add timeout and restart
// Todo: add will message to send percentage
void connectMqtt() {
  mqttClient.setServer(brokerIP, brokerPort);

  Serial.println((String) "[E] Attempting to connect to MQTT broker: " + brokerIP);
  
  while (!mqttClient.connect(mqttID, mqttUser, mqttPass));

  mqttClient.subscribe(inTopic);

  mqttClient.setCallback(callback);

  Serial.println((String) "[E] You're connected to the MQTT broker! Topic: " + inTopic);
}


void readState() {
  switch (connection) {
    case reconnectingWifi:
      // Wait for reconnection
      if (WiFi.status() == WL_CONNECTED)
        connection = connectingMqtt;
    break;

    case connectingMqtt:
      connectMqtt();
      connection = readingMqttMessage;
    break;

    case readingMqttMessage:
      // Check wifi connection
      if (WiFi.status() == WL_NO_SSID_AVAIL)
        connection = reconnectingWifi;

      // Check mqtt
      if (!mqttClient.connected())
        connection = connectingMqtt;
      
      // Use non blocking pubsub to read messages
      mqttClient.loop();
    break;
  }
}