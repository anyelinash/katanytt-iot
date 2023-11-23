#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Arduino.h>

const char *ssid = "BJR";
const char *password = "@WOlf1998";
const char *mqtt_server = "34.73.226.103";
const int mqtt_port = 1883;
const char *mqtt_user = "admin";
const char *mqtt_password = "@Katanytt2023";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;

// Define the pin connected to the MQ135 sensor
const int pinMQ = 34;  // Pin A0 on the DOIT ESP32 DevKit
MQ135 gasSensor = MQ135(pinMQ);

// Define the pin connected to the relay
const int relayPin = 23;  // Change this to the GPIO pin you connected the relay's signal input to

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Error connecting to WiFi.");
  }
}

void reconnectWifi() {
  if (millis() - lastWifiReconnectAttempt > 30000) {
    lastWifiReconnectAttempt = millis();
    Serial.println("Attempting to reconnect WiFi...");
    WiFi.reconnect();
  }
}

void reconnectMqtt() {
  if (!client.connected() && WiFi.status() == WL_CONNECTED) {
    if (millis() - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = millis();
      Serial.print("Attempting MQTT connection...");
      if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
        Serial.println("Connected");

        // Subscribe to the MQTT topic for relay control
        client.subscribe("relay/control");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" Trying again in 5 seconds");
      }
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  // Handle MQTT messages received on the subscribed topic
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received on topic '");
  Serial.print(topic);
  Serial.print("': ");
  Serial.println(message);

  // Check the message and control the relay accordingly
  if (String(topic) == "relay/control") {
    if (message == "on") {
      digitalWrite(relayPin, LOW); // Turn on the relay
    } else if (message == "off") {
      digitalWrite(relayPin, HIGH); // Turn off the relay
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT); // Set the relay pin as an output
  digitalWrite(relayPin, LOW); // Initialize the relay in the off state

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected() || WiFi.status() != WL_CONNECTED) {
    reconnectMqtt(); // Attempt to reconnect MQTT
  } else {
    // Perform your normal operations here

    int valorSensor = analogRead(pinMQ);
    float resistencia = ((1023.0 / valorSensor) - 1) * 10000;
    float ppm = gasSensor.getPPM();
    String mensaje = "Sensor value: " + String(valorSensor) +
                     ", Resistance: " + String(resistencia) +
                     ", Gas concentration (PPM): " + String(ppm);

    client.publish("sensors/gas", mensaje.c_str());
    Serial.println("Message sent: " + mensaje);

    // Allow the client to process incoming messages
    client.loop();

    delay(2000);
  }
}

