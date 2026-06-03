#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <GyverDS18.h>
#include <Preferences.h>

Preferences preferences;

const char* mqtt_server    = "dev.rightech.io";
const char* mqtt_client_id = "mqtt-ergunxer-xtu5gl";

const char* topic_command_reset = "base/command/reset";
const char* topic_temp          = "base/state/temperature";
const char* topic_purity        = "base/state/water_purity";
const char* topic_capacity      = "base/state/water_capacity";

#define TDS_SENSOR_PIN   A0
#define DS_PIN           1
#define FLOW_SENSOR_PIN  2
#define SAMPLE_COUNT     30

GyverDS18Single ds(DS_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

int analogBuffer[SAMPLE_COUNT];
int analogBufferTemp[SAMPLE_COUNT];
float tdsValue    = 0;
float temperature = 25.0;

volatile uint32_t pulseCount = 0;
uint32_t lastPulseCount      = 0;
float flowRate               = 0;
float totalVolume            = 0;
bool waterFlowing            = false;

unsigned long lastFlowCheck = 0;
unsigned long lastMqttSend  = 0;
unsigned long lastSaveTime  = 0;
float lastSavedVolume       = 0;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

int medianFilter(int arr[], int len) {
  int buf[len];
  for (int i = 0; i < len; i++) buf[i] = arr[i];
  for (int j = 0; j < len - 1; j++) {
    for (int i = 0; i < len - j - 1; i++) {
      if (buf[i] > buf[i + 1]) {
        int tmp = buf[i];
        buf[i] = buf[i + 1];
        buf[i + 1] = tmp;
      }
    }
  }
  if (len & 1) return buf[len / 2];
  return (buf[len / 2] + buf[len / 2 - 1]) / 2;
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);

  if (!wm.autoConnect("WaterMonitor-AP")) {
    Serial.println("WiFi config timeout, restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("WiFi connected. IP: " + WiFi.localIP().toString());
}

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting... ");
    if (client.connect(mqtt_client_id)) {
      Serial.println("OK");
      client.subscribe(topic_command_reset);
    } else {
      Serial.print("fail, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

  if (String(topic) == topic_command_reset && message == "1") {
    totalVolume = 0.0;
    preferences.putFloat("totalVolume", 0.0);
    client.publish(topic_capacity, "0.00");
    Serial.println("Volume reset via MQTT command");
  }
}

void updateFlow(unsigned long now) {
  if (now - lastFlowCheck < 100) return;
  lastFlowCheck = now;

  noInterrupts();
  uint32_t currentPulses = pulseCount;
  interrupts();

  uint32_t delta = currentPulses - lastPulseCount;
  lastPulseCount = currentPulses;
  flowRate = delta / 7.3;

  if (flowRate > 0.1) {
    totalVolume += flowRate / 600.0;
    if (!waterFlowing) {
      waterFlowing = true;
      Serial.println(">>> Flow started");
    }
  } else if (waterFlowing) {
    waterFlowing = false;
    Serial.println("<<< Flow stopped");
  }
}

void publishSensors(unsigned long now) {
  if (now - lastMqttSend < 5000) return;
  lastMqttSend = now;

  if (ds.ready()) {
    if (ds.readTemp()) temperature = ds.getTemp();
    ds.requestTemp();
  }

  for (int i = 0; i < SAMPLE_COUNT; i++) analogBuffer[i] = analogRead(TDS_SENSOR_PIN);
  for (int i = 0; i < SAMPLE_COUNT; i++) analogBufferTemp[i] = analogBuffer[i];

  float voltage      = medianFilter(analogBufferTemp, SAMPLE_COUNT) * 3.3 / 4096.0;
  float compensation = 1.0 + 0.02 * (temperature - 25.0);
  float vCompensated = voltage / compensation;
  tdsValue = (133.42 * pow(vCompensated, 3)
            - 255.86 * pow(vCompensated, 2)
            + 857.39 * vCompensated) * 0.5;

  client.publish(topic_temp,     String(temperature, 1).c_str());


client.publish(topic_purity,   String(tdsValue, 0).c_str());
  client.publish(topic_capacity, String(totalVolume, 2).c_str());

  Serial.printf("T: %.1f | TDS: %.0f | Vol: %.2f L\n", temperature, tdsValue, totalVolume);
}

void persistVolume() {
  if (millis() - lastSaveTime > 60000 || abs(totalVolume - lastSavedVolume) > 0.5) {
    preferences.putFloat("totalVolume", totalVolume);
    lastSaveTime = millis();
    lastSavedVolume = totalVolume;
    Serial.println("Volume saved to flash");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TDS_SENSOR_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

  ds.requestTemp();
  setupWifi();

  preferences.begin("water-meter", false);
  totalVolume = preferences.getFloat("totalVolume", 0.0);
  lastSavedVolume = totalVolume;
  Serial.printf("Loaded volume: %.2f L\n", totalVolume);

  client.setServer(mqtt_server, 1883);
  client.setKeepAlive(60);
  client.setCallback(mqttCallback);
}

void loop() {
  if (!client.connected()) mqttReconnect();
  client.loop();

  unsigned long now = millis();
  updateFlow(now);
  publishSensors(now);
  persistVolume();
}