#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1015.h>
#include <time.h>
#include <FS.h>
#include <WiFiManager.h>

Adafruit_ADS1015 ads1(0x48);
Adafruit_ADS1015 ads2(0x49);

#define LED_PIN 2
#define CHANNELS 8

WiFiClient espClient;
PubSubClient mqtt(espClient);
Ticker ledTicker;
Ticker stateTicker;
Ticker mqttTicker;
Ticker telemetryTicker;
Ticker sendTicker;

double samples[CHANNELS];
double current[CHANNELS];
double calibration[CHANNELS];
uint8_t numOfSamples = 0;
unsigned long measureStart = 0;

double consumption[CHANNELS];
double consumption_today[CHANNELS];
double lastPublish[CHANNELS];

bool booted = false;
bool reconnecting = false;
bool willsend = false;
bool resetted_consumption = false;
bool willsendtelemetry = false;

char mqtt_server[20] = "10.0.2.1";

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  ledTicker.attach(0.5, ledtick);

  WiFi.mode(WIFI_STA);
  WiFi.hostname("Sicherungskasten");

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(5 * 60);
  if (!wifiManager.autoConnect("Sicherungskasten")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  ArduinoOTA.setHostname("Sicherungskasten");
  ArduinoOTA.begin();

  ads1.begin();
  ads1.setSPS(ADS1015_DR_3300SPS);
  ads2.begin();
  ads2.setSPS(ADS1015_DR_3300SPS);

  sendTicker.attach(30, sendtick);
  stateTicker.attach(60, statetick);
  ledTicker.attach(2, ledtick);
  telemetryTicker.attach(5 * 60, telemetrytick);

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/calibration.json")) {
      //file exists, reading and loading
      File configFile = SPIFFS.open("/calibration.json", "r");
      if (configFile) {
        StaticJsonDocument<512> doc;
        auto err = deserializeJson(doc, configFile);
        JsonObject json = doc.as<JsonObject>();
        for (uint8_t i = 0; i < CHANNELS; i++) {
          String channelName = "channel_" + String(i);
          if (json.containsKey(channelName)) {
            calibration[i] = json[channelName];
          }
        }
        configFile.close();
      }
    }

    if (SPIFFS.exists("/telemetry.json")) {
      //file exists, reading and loading
      File configFile = SPIFFS.open("/telemetry.json", "r");
      if (configFile) {
        StaticJsonDocument<512> doc;
        auto err = deserializeJson(doc, configFile);
        JsonObject json = doc.as<JsonObject>();
        for (uint8_t i = 0; i < CHANNELS; i++) {
          String channelName = "channel_" + String(i);
          if (json.containsKey(channelName)) {
            consumption[i] = json[channelName];
          }
        }
        configFile.close();
      }
    }
  }

  configTime(1 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  measureStart = millis();
}

void loop() {
  ArduinoOTA.handle();
  reconnectMQTT();
  mqtt.loop();
  measure();
  if (willsend) send();
  if (willsendtelemetry) sendTelemetry();
  yield();
}

void measure() {
  for (uint8_t i = 0; i < 4; i++) {
    ads1.startContinuous_SingleEnded(i);
    ads2.startContinuous_SingleEnded(i);

    unsigned long start = millis();
    for (int s = 0; s < 117; s++) { //with i2c communication and math operations it will take about 250 ms to process 117 samples per channel => 1 second for all channels
      delay(1);
      measured(i, ads1.getLastConversionResults());
      measured(i + 4, ads2.getLastConversionResults());
    }
  }

  for (uint8_t i = 0; i < CHANNELS; i++) {
    double average = sqrt(samples[i] / 117);
    average -= calibration[i];
    if (average < 0) average = 0;
    double power = average * 230.0;
    bool threshold = (average > lastPublish[i] * 1.5 || average < lastPublish[i] / 1.5) && (power > 20 || lastPublish[i]*230.0 > 20);
    if (threshold) willsend = true;
    current[i] = average;
    consumption[i] += average;
    consumption_today[i] += average;
    samples[i] = 0;
  }
}

void measured(uint8_t channel, int16_t value) {
  double voltage = (value * 3.0) / 1000.0;
  double current = (voltage - 2.511) * 10.0;
  samples[channel] += current * current;
}

void send() {
  willsend = false;

  DynamicJsonDocument doc;
  JsonObject root = doc.to<JsonObject>();

  int total_power = 0;
  for (uint8_t i = 0; i < CHANNELS; i++) {
    String channelName = "channel_" + String(i);
    JsonObject channel = root.createNestedObject(channelName);
    channel["current"] = current[i];
    channel["power"] = round(current[i] * 230.0);
    lastPublish[i] = current[i];
    total_power += round(current[i] * 230.0);
  }

  JsonObject total = root.createNestedObject("total");
  total["power"] = total_power;

  char jsonValue[1024];
  serializeJson(doc, jsonValue);
  Serial.println(jsonValue);
  mqtt.publish("Sicherungskasten/measure", jsonValue, false);
}

void sendTelemetry() {
  willsendtelemetry = false;

  {
    DynamicJsonDocument doc;
    JsonObject json = doc.to<JsonObject>();
    for (uint8_t i = 0; i < CHANNELS; i++) {
      String channelName = "channel_" + String(i);
      json[channelName] = consumption[i];
    }

    File configFile = SPIFFS.open("/telemetry.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJson(json, configFile);
    configFile.close();


    time_t now;
    struct tm * timeinfo;
    time(&now);
    timeinfo = localtime(&now);
    if (timeinfo->tm_hour == 0 && !resetted_consumption) {
      for (uint8_t i = 0; i < CHANNELS; i++) {
        consumption_today[i] = 0;
      }
    }
    resetted_consumption = timeinfo->tm_hour == 0;
  }

  DynamicJsonDocument doc;
  JsonObject root = doc.to<JsonObject>();

  for (uint8_t i = 0; i < CHANNELS; i++) {
    String channelName = "channel_" + String(i);
    JsonObject channel = root.createNestedObject(channelName);
    channel["consumption_today"] = (consumption_today[i] / 3600.0 / 1000.0) * 230.0;
    channel["consumption_total"] = (consumption[i] / 3600.0 / 1000.0) * 230.0;
    lastPublish[i] = current[i];
  }

  char jsonValue[1024];
  serializeJson(doc, jsonValue);
  Serial.println(jsonValue);
  mqtt.publish("Sicherungskasten/telemetry", jsonValue, false);
}

void reconnectMQTT() {
  if (mqtt.connected() || reconnecting || WiFi.status() != WL_CONNECTED) return;

  Serial.print("Attempting MQTT connection");
  String clientId = "Sicherungskasten-" + String(random(0xffffff), HEX);

  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(receivedMessage);

  WiFi.mode(WIFI_STA);
  if (mqtt.connect(clientId.c_str())) {
    Serial.println("connected");
    mqtt.subscribe("Sicherungskasten/calibrate");
    if (!booted) {
      booted = true;
      mqtt.publish("Sicherungskasten/state", "booted", false);
    } else {
      mqtt.publish("Sicherungskasten/state", "reconnected", false);
    }
    ledTicker.detach();
    pinMode(LED_PIN, INPUT);
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqtt.state());
    reconnecting = true;
    mqttTicker.once(10, reconnectMQTTTimeout);
    if (!ledTicker.active()) {
      pinMode(LED_PIN, OUTPUT);
      ledTicker.attach(2, ledtick);
    }
  }
}

void reconnectMQTTTimeout() {
  reconnecting = false;
}

void ledtick() {
  //toggle state
  int state = digitalRead(LED_PIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_PIN, !state);     // set pin to the opposite state
}

void sendtick() {
  willsend = true;
}

void telemetrytick() {
  willsendtelemetry = true;
}

void statetick() {
  if (!mqtt.connected()) return;
  mqtt.publish("Sicherungskasten/state", "online", false);
}

void receivedMessage(char* topic_p, byte* payload, unsigned int length) {
  payload[length] = '\0';
  if (strcmp(topic_p, "Sicherungskasten/calibrate") == 0) {
    int channel = atoi((char*)payload);
    calibrate(channel);
  }
}

void calibrate(int channel) {
  if (channel >= CHANNELS) {
    for (uint8_t i = 0; i < CHANNELS; i++) calibration[i] = 0;
  } else calibration[channel] = 0;
  measure();
  if (channel >= CHANNELS) {
    for (uint8_t i = 0; i < CHANNELS; i++) calibration[channel] = current[channel];
  } else calibration[channel] = current[channel];


  DynamicJsonDocument doc;

  JsonObject json = doc.to<JsonObject>();
  for (uint8_t i = 0; i < CHANNELS; i++) {
    String channelName = "channel_" + String(i);
    json[channelName] = calibration[i];
  }

  File configFile = SPIFFS.open("/calibration.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  serializeJson(json, configFile);
  configFile.close();
}
