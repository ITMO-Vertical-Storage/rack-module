#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define MODULE_TYPE "rack-module"

// --- WiFi ---
const char* ssid = "Beeline_MF";
const char* password = "$@ndr0nix";

// --- MQTT ---
const char* mqtt_server = "192.168.8.100"; // IP Orange Pi
WiFiClient espClient;
PubSubClient client(espClient);

// --- Мультиплексоры ---
#define MUX1_ADDR 0x70
#define MUX2_ADDR 0x77

// 10 датчиков по порядку: 0–4 на первом MUX, 0–4 на втором
Adafruit_VL53L0X sensor;
uint8_t muxAddr[] = { MUX2_ADDR, MUX2_ADDR, MUX2_ADDR, MUX2_ADDR, MUX2_ADDR,
                      MUX1_ADDR, MUX1_ADDR, MUX1_ADDR, MUX1_ADDR, MUX1_ADDR };
uint8_t channel[] = {4, 3, 2, 1, 0, 0, 1, 2, 3, 4};

void selectMux(uint8_t muxAddr, uint8_t channel);
void setup_wifi();
void reconnect();
void sendIdentity();
void initSensors();
void readAndSendSensors();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start monitor");
  Wire.begin();

  setup_wifi();
  client.setServer(mqtt_server, 1883);

  initSensors();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  readAndSendSensors();
  delay(1000);
}

// Функция выбора канала мультиплексора
void selectMux(uint8_t muxAddr, uint8_t channel) {
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Подключение к WiFi
void setup_wifi() {
  delay(1000);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("\nIP: " + String(WiFi.localIP().toString()));
  Serial.println("\nMAC: " + String(WiFi.macAddress()));
}

// Подключение к MQTT
void reconnect() {
  while (!client.connected()) {
    if (client.connect((WiFi.macAddress()).c_str())) {
      Serial.println("Connected to MQTT");
      sendIdentity();
    } else {
      delay(1000);
    }
  }
}

// Отправка информации о модуле
void sendIdentity() {
  String ip = WiFi.localIP().toString();
  String mac = WiFi.macAddress();

  client.publish(("module/" + mac + "/identity/type").c_str(), MODULE_TYPE);
  client.publish(("module/" + mac + "/identity/ip").c_str(), ip.c_str());
}

// Настройка датчиков
void initSensors() {
  for (int i = 0; i < 10; i++) {
    selectMux(muxAddr[i], channel[i]);
    if (!sensor.begin()) {
      Serial.printf("Sensor %d not found\n", i+1);
    } else {
      Serial.printf("Sensor %d OK\n", i+1);
    }
  }
}

// Отправка показаний
void readAndSendSensors() {
  for (int i = 0; i < 10; i++) {
    selectMux(muxAddr[i], channel[i]);

    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      char topic[64];
      snprintf(topic, sizeof(topic), "module/%s/sensor/vl53l0x/%d", (WiFi.macAddress()).c_str(), i+1);
      char payload[16];
      snprintf(payload, sizeof(payload), "%d", measure.RangeMilliMeter);
      client.publish(topic, payload);
    }

    delay(50);
  }
}