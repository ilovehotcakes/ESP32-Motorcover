/**
  Motorized Cover - ESP32
  Aurthor: Jason Chen

  A simple ESP32 program that let's user control powerful and quiet stepper
  motors via MQTT. You use it to open and close blinds/shades, etc. Can be
  used with Alexa or Home Assistant.

  This program utilizes TMC2209 drivers for the stepper motor and it drives
  NEMA stepper motors.
**/
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"
#include "motor.h"

// Defining states for the state machine
#define INITIALIZING      0
#define RECONNECTING_WIFI 1
#define CONNECTING_MQTT   2
#define READ_MQTT_MSG     3
// Defining commands recieved from MQTT
#define COVER_STOP       -1
#define COVER_OPEN       -2
#define COVER_CLOSE      -3
#define COVER_SET_MIN    -4
#define COVER_SET_MAX    -5
#define REBOOT_SYS       -99
// Defining LED pin, it's tied to GPIO2 on HiLetGo board
#define LED_PIN           2
#define VERBOSE           1


void core0Task(void * parameter);
void startWifi();
void connectMqtt();
void sendMqtt(String);


TaskHandle_t C0;  // For dual core setup
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
int      state = INITIALIZING;   // State machine to manage WiFi/MQTT
String   ssid  = secretSSID;     // SSID (name) for WiFi
String   pass  = secretPass;     // Network password for WiFi
String   mqttID     = secretMqttID;    // MQTT ID for PubSubClient
String   mqttUser   = secretMqttUser;  // MQTT server username (optional)
String   mqttPass   = secretMqttPass;  // MQTT server password (optional)
String   brokerIP   = secretBrokerIP;  // IP of MQTT server
uint16_t brokerPort = secretBrokerPort;
String   inTopic    = secretInTopic;   // MQTT inbound topic
String   outTopic   = secretOutTopic;  // MQTT outbound topic



void setup() {
  // Initialize hardware serial for debugging
  if (VERBOSE) Serial.begin(9600);
  
  // Initialized and turn on LED to indicate boot up
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Core 0 setup for dual core operation
  disableCore0WDT();  // Disable watchdog timer
  xTaskCreatePinnedToCore(core0Task, "Core_0", 8192, NULL, 1, &C0, 0);

  // Start the WiFi connection
  startWifi();
  state = CONNECTING_MQTT;

  motorSetup();
}


/**
  Keeps running the motor and checks if there is any messages avaiable to
  publish to MQTT server. The motor must run in the loop. For some unknown
  reasons, the stepper doesn't run smoothly on core0.
**/
void loop() {
  motorRun();
}


/**
  The state machine that tracks communication to MQTT/WiFi. I have tried to
  start the WiFi in the state machine but it doesn't work. Also, the delay at
  the of the function is required, else WiFi can't connect for unknown reason.
**/
void core0Task(void * parameter) {
  for (;;) {
    switch (state) {
      case RECONNECTING_WIFI:
        // Turn on LED to indicate disconnected
        digitalWrite(LED_PIN, HIGH);
        
        // Wait for reconnection
        if (WiFi.status() == WL_CONNECTED)
          state = CONNECTING_MQTT;
      break;

      case CONNECTING_MQTT:
        // Turn on LED to indicate disconnected
        digitalWrite(LED_PIN, HIGH);

        connectMqtt();

        // Update MQTT server of current shade opening position
        sendMqtt((String) motorCurrentPercent());

        // Turn off LED to indicate fully connected
        digitalWrite(LED_PIN, LOW);
        state = READ_MQTT_MSG;
      break;

      case READ_MQTT_MSG:
        // Check wifi connection
        if (WiFi.status() == WL_NO_SSID_AVAIL)
          state = RECONNECTING_WIFI;

        // Check mqtt connection
        if (!mqttClient.connected())
          state = CONNECTING_MQTT;
        
        // Use non blocking method to check for messages
        mqttClient.loop();
      break;
    }
    delay(100);
  }
}


// TODO: Add timeout and restart
void startWifi() {
  Serial.println("[E] Attempting to connect to WPA SSID: " + ssid);

  while (WiFi.begin(ssid.c_str(), pass.c_str()) != WL_CONNECTED)
    delay(5000);
  
  Serial.print("[E] You're connected to the WiFi! IP: ");
  Serial.println(WiFi.localIP());
}


void readMqtt(char* topic, byte* buf, unsigned int len) {
  String message = "";
  for (int i = 0; i < len; i++) message += (char) buf[i];
  int command = message.toInt();

  Serial.println("[I] Received message: " + message);

  if (command >= 0) motorMove(command);
  else if (command == COVER_STOP) motorStop();
  else if (command == COVER_OPEN) motorMin();
  else if (command == COVER_CLOSE) motorMax();
  else if (command == COVER_SET_MAX) motorSetMax();
  else if (command == COVER_SET_MIN) motorSetMin();
  else if (command == REBOOT_SYS) ESP.restart();
}


// TODO: add timeout and restart
void connectMqtt() {
  Serial.println("[E] Attempting to connect to MQTT broker: " + brokerIP);

  mqttClient.setServer(brokerIP.c_str(), brokerPort);
  
  while (!mqttClient.connect(mqttID.c_str(), mqttUser.c_str(), mqttPass.c_str()));

  mqttClient.subscribe(inTopic.c_str());
  mqttClient.setCallback(readMqtt);

  Serial.println("[E] You're connected to the MQTT broker! Topic: " + inTopic);
}


void sendMqtt(String message) {
  mqttClient.publish(secretOutTopic.c_str(), message.c_str());
  Serial.println((String) "[I] Sent message: " + message);
}