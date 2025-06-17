#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <VL53L0X.h>

#define MODULE_TYPE "rack-module"
#define MUX1_ADDR 0x70
#define MUX2_ADDR 0x77

const char* ssid = "Beeline_MF";
const char* password = "$@ndr0nix";
const char* mqtt_server = "192.168.8.100";

WiFiClient espClient;
PubSubClient client(espClient);

VL53L0X sensors[10];

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
  delay(1500);
}

void selectMux(uint8_t muxAddr, uint8_t channel) {
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(50);
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(WiFi.macAddress().c_str())) {
      sendIdentity();
    } else {
      delay(1000);
    }
  }
}

void sendIdentity() {
  String ip = WiFi.localIP().toString();
  String mac = WiFi.macAddress();

  client.publish(("module/" + mac + "/identity/type").c_str(), MODULE_TYPE);
  client.publish(("module/" + mac + "/identity/ip").c_str(), ip.c_str());
}

void initSensors() {
  for (int i = 0; i < 10; i++) {
    selectMux(muxAddr[i], channel[i]);
    delay(100);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.printf("Sensor %d not found\n", i + 1);
      continue;
    } else {
      // Включаем Long Range Mode
      sensors[i].setSignalRateLimit(0.1);
      sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    }
  }
}

void readAndSendSensors() {
  String mac = WiFi.macAddress();

  for (int i = 0; i < 10; i++) {
    selectMux(muxAddr[i], channel[i]);
    delay(100);
    uint16_t dist = sensors[i].readRangeSingleMillimeters();
    if (sensors[i].timeoutOccurred()) {
      Serial.printf("Sensor %d timeout\n", i + 1);
      continue;
    }

    char topic[64];
    snprintf(topic, sizeof(topic), "module/%s/sensor/vl53l0x/%d", mac.c_str(), i + 1);
    char payload[16];
    snprintf(payload, sizeof(payload), "%d", dist);
    client.publish(topic, payload);
  }
}
