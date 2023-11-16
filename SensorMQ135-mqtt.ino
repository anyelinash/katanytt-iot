#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>
#include <PubSubClient.h>
#include <WiFi.h>

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

// Define el pin al que está conectado el sensor MQ135
const int pinMQ = 34;  // Pin A0 en el DOIT ESP32 DevKit
MQ135 gasSensor = MQ135(pinMQ);

void setup_wifi() {
  delay(10);
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("Dirección IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Error al conectar a WiFi.");
  }
}

void reconnectWifi() {
  if (millis() - lastWifiReconnectAttempt > 30000) {
    lastWifiReconnectAttempt = millis();
    Serial.println("Intentando reconectar WiFi...");
    WiFi.reconnect();
  }
}

void reconnectMqtt() {
  if (!client.connected() && WiFi.status() == WL_CONNECTED) {
    if (millis() - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = millis();
      Serial.print("Intentando conexión MQTT...");
      if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
        Serial.println("Conectado");
      } else {
        Serial.print("falló, rc=");
        Serial.print(client.state());
        Serial.println(" Intentando de nuevo en 5 segundos");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  reconnectWifi(); // Intenta reconectar WiFi
  if (!client.connected() || WiFi.status() != WL_CONNECTED) {
    reconnectMqtt(); // Intenta reconectar MQTT
  } else {
    // Realiza tus operaciones normales aquí

    int valorSensor = analogRead(pinMQ);
    float resistencia = ((1023.0 / valorSensor) - 1) * 10000;
    float ppm = gasSensor.getPPM();
    String mensaje = "Valor del sensor: " + String(valorSensor) +
                     ", Resistencia: " + String(resistencia) +
                     ", Concentración de gas (PPM): " + String(ppm);
    client.publish("sensores/gas", mensaje.c_str());
    Serial.println("Mensaje enviado: " + mensaje);

    delay(2000);
  }
}
