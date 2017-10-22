#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <MQTTClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266httpUpdate.h>
#include <SPI.h>
#include "main.h"
#include "secrets.h"
#include "esp8266_defaults.h"
#include "devices.h"

bool WiFiConnected (void);
bool MQTTConnected (void);

#ifdef RELAY0
#define RELAY
#endif
#ifdef RELAY1
#define RELAY
#endif
#ifdef RELAY2
#define RELAY
#endif
#ifdef RELAY3
#define RELAY
#endif

#ifdef BUTTON1
#define BUTTON
#endif
#ifdef BUTTON2
#define BUTTON
#endif
#ifdef BUTTON3
#define BUTTON
#endif
#ifdef BUTTON4
#define BUTTON
#endif

#ifdef DHT_SENSOR
#include <DHT.h>
DHT dht(DHTPIN, DHTTYPE);
#endif

#ifdef SOIL_HUMIDITY_SENSOR
unsigned int soilHumidityRaw;
double soilHumidity;
#endif

#ifdef LIGHT_SENSOR
unsigned int lightIntensityRaw;
double lightIntensity;
#endif

#ifdef LIGHT_SENSOR_PIN
int lightPulseCnt = 0;
bool increaseLightPulseCnt = false;
#endif

#ifdef CURRENT_COUNTER
float currentEnergy = 0.0;
float currentPower = 0.0;
int previousPulseCnt = 0;
int currentPulseCnt = 0;
#endif

#ifdef ONE_WIRE_SENSOR
#include <OneWire.h>
OneWire ds(ONE_WIRE_BUS);
#endif

#ifdef OLED
#include <Wire.h>  // Include Wire if you're using I2C
#include "SH1106.h"
//SH1106 display(0x3c, D3, D5);
SH1106 display(0x3c, D1, D2); // Settings for WROOM-02
#endif

#ifdef ENV_DISPLAY
String displayBuffer[DISPLAY_COUNT];
int i;
int displayCur=0;
String currentTime;
String currentDate;

#endif

#ifdef BME_SENSOR
#include <Wire.h>  // Include Wire if you're using I2C
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
Adafruit_BME280 bme; // I2C
#endif

#ifdef SONOFF
int sonoffState = 0;
bool updateSONOFF = false;
#endif

#ifdef BUTTON1
int button1State = 0;
bool updateBUTTON1 = false;
#endif
#ifdef BUTTON2
int button2State = 0;
bool updateBUTTON2 = false;
#endif
#ifdef BUTTON3
int button3State = 0;
bool updateBUTTON3 = false;
#endif
#ifdef BUTTON4
int button4State = 0;
bool updateBUTTON4 = false;
#endif

#ifdef TWO_WAY_SWITCH
int TWO_WAY_SWITCHState = 0;
bool updateTWO_WAY_SWITCH = false;
#endif

#ifdef COVER
bool updateCOVER = true;
int coverState = 0; // 0 - not closed, 1 - closed
#endif

#ifdef RELAY0
int relay0State = 0;
#endif
#ifdef RELAY1
int relay1State = 0;
#endif
#ifdef RELAY2
int relay2State = 0;
#endif
#ifdef RELAY3
int relay3State = 0;
#endif



#ifdef FORCE_DEEPSLEEP
extern "C" {
#include "user_interface.h"
}
ADC_MODE(ADC_VCC);
int vdd;
#endif

const char* ROOM = DEVICE_NAME;
const char* HOST = HOST_NAME;



const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;



// Port configuration



// Standard MQTT topics
const char* MQTT_TEMP_TOPIC = "/temperature";
const char* MQTT_DS18_TEMP_TOPIC = "/ds18_temperature";
const char* MQTT_HUMIDITY_TOPIC = "/humidity";
const char* MQTT_BME_HUMIDITY_TOPIC = "/bme_humidity";
const char* MQTT_BME_TEMPERATURE_TOPIC = "/bme_temperature";
const char* MQTT_BME_PRESSURE_TOPIC = "/bme_pressure";
const char* MQTT_SENSOR_FAILURE_TOPIC = "/sensor_failure";

const char* MQTT_SOIL_HUMIDITY_TOPIC = "/soil_humidity";

const char* MQTT_WIFI_RSSI_TOPIC = "/rssi";
const char* MQTT_WIFI_CHANNEL_TOPIC = "/channel";
const char* MQTT_STARTUP_TOPIC = "/startup";
const char* MQTT_IP_TOPIC = "/ip";

const char* MQTT_FLASH_TOPIC = "/flash";

#ifdef LIGHT_SENSOR
const char* MQTT_LIGHT_INTENSITY_TOPIC = "/light_intensity";
const char* MQTT_LIGHT_TRIGGER_TOPIC = "/light_trigger";
#endif

#ifdef LIGHT_SENSOR_PIN
const char* MQTT_LIGHT_PULSE_COUNTER_TOPIC = "/light_pulse_cnt";
const char* MQTT_LIGHT_PULSE_BIAS_TOPIC = "/light_pulse_bias";
#endif

#ifdef CURRENT_SENSOR
const char* MQTT_CURRENT_SENSOR_CUR_ENERGY_TOPIC = "/current_energy";
const char* MQTT_CURRENT_SENSOR_CUR_POWER_TOPIC = "/current_power";
#endif


#ifdef RELAY0
const char* MQTT_RELAY_0_STATE_TOPIC = "/relay0";
const char* MQTT_RELAY_0_SET_TOPIC = "/relay0/set";
#endif
#ifdef RELAY1
const char* MQTT_RELAY_1_STATE_TOPIC = "/relay1";
const char* MQTT_RELAY_1_SET_TOPIC = "/relay1/set";
#endif
#ifdef RELAY2 
const char* MQTT_RELAY_2_STATE_TOPIC = "/relay2";
const char* MQTT_RELAY_2_SET_TOPIC = "/relay2/set";
#endif
#ifdef RELAY3 
const char* MQTT_RELAY_3_STATE_TOPIC = "/relay3";
const char* MQTT_RELAY_3_SET_TOPIC = "/relay3/set";
#endif

#ifdef COVER
const char* MQTT_COVER_STATE_TOPIC = "/cover";
const char* MQTT_COVER_SET_TOPIC = "/cover/set";
#endif

#ifdef BUTTON1
char* MQTT_BUTTON1_STATE_TOPIC = BUTTON1_STATE_TOPIC;
char* MQTT_BUTTON1_SET_TOPIC = BUTTON1_SET_TOPIC;
#endif
#ifdef BUTTON2
char* MQTT_BUTTON2_STATE_TOPIC = BUTTON2_STATE_TOPIC;
char* MQTT_BUTTON2_SET_TOPIC = BUTTON2_SET_TOPIC;
#endif
#ifdef BUTTON3
char* MQTT_BUTTON3_STATE_TOPIC = BUTTON3_STATE_TOPIC;
char* MQTT_BUTTON3_SET_TOPIC = BUTTON3_SET_TOPIC;
#endif
#ifdef BUTTON4
char* MQTT_BUTTON4_STATE_TOPIC = BUTTON4_STATE_TOPIC;
char* MQTT_BUTTON4_SET_TOPIC = BUTTON4_SET_TOPIC;
#endif
#ifdef BUTTON5
const char* MQTT_BUTTON5_STATE_TOPIC = "/button5";
const char* MQTT_BUTTON5_SET_TOPIC = "/button5/set";
#endif
#ifdef BUTTON6
const char* MQTT_BUTTON6_STATE_TOPIC = "/button6";
const char* MQTT_BUTTON6_SET_TOPIC = "/button6/set";
#endif
#ifdef BUTTON7
const char* MQTT_BUTTON7_STATE_TOPIC = "/button7";
const char* MQTT_BUTTON7_SET_TOPIC = "/button7/set";
#endif
#ifdef BUTTON8
const char* MQTT_BUTTON8_STATE_TOPIC = "/button8";
const char* MQTT_BUTTON8_SET_TOPIC = "/button8/set";
#endif

#ifdef SONOFF
const char* MQTT_SONOFF_STATE_TOPIC = "/sonoff";
const char* MQTT_SONOFF_SET_TOPIC = "/sonoff/set";
#endif

const char* MQTT_BAT_VOLT_TOPIC = "/battery_voltage";

const char* MQTT_DEEPSLEEP_TOPIC = "/deepsleep";
const char* MQTT_PING_TOPIC = "/ping";
const char* MQTT_PONG_TOPIC = "/pong";
const char* MQTT_HEARTBEAT_TOPIC = "/heartbeat";
const long MQTT_SENSOR_TIMER = TIME_SENSOR_UPDATE; // in seconds
const long MQTT_STATE_TIMER = TIME_STATE_UPDATE; // in seconds
const long MQTT_DISPLAY_TIMER = TIME_DISPLAY_UPDATE; // in seconds
unsigned long MQTT_SENSOR_TIMER_previousMillis = 0;
unsigned long MQTT_STATE_TIMER_previousMillis = 0;
unsigned long MQTT_DISPLAY_TIMER_previousMillis = 0;

WiFiClient net;
MQTTClient mqtt;

bool wifiRunning;
bool mqttRunning;

char char_buffer[84];

int heartbeat = 0;
int wifiTries = 0;

void messageReceived(String &topic, String &payload) {
  Serial.print("incoming: ");
  Serial.print(topic);
  Serial.print(" - ");
  Serial.print(payload);
  Serial.println();

  if (topic == buildTopic(MQTT_PING_TOPIC)) {
    mqttPong(payload);
  }
  if (topic == buildTopic(MQTT_FLASH_TOPIC)) {
    mqttSend(buildTopic(MQTT_FLASH_TOPIC), "Trying to update");
//    t_httpUpdate_return ret = ESPhttpUpdate.update(DEFAULT_FLASH_SERVER, DEFAULT_FLASH_PORT, payload);
    t_httpUpdate_return ret = ESPhttpUpdate.update(payload);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.println("[update] Update failed.");
        mqttSend(buildTopic(MQTT_FLASH_TOPIC), "Update failed");
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[update] Update no Update.");
        mqttSend(buildTopic(MQTT_FLASH_TOPIC), "No Update");
        break;
      case HTTP_UPDATE_OK:
        Serial.println("[update] Update ok."); // may not called we reboot the ESP
        mqttSend(buildTopic(MQTT_FLASH_TOPIC), "Update ok");
        break;
    }
  }
#ifdef RELAY
  if (topic == buildTopic(MQTT_RELAY_0_SET_TOPIC)) {
    mqttRelaySet(payload, 0);
  }
  if (topic == buildTopic(MQTT_RELAY_1_SET_TOPIC)) {
    mqttRelaySet(payload, 1);
  }
  if (topic == buildTopic(MQTT_RELAY_2_SET_TOPIC)) {
    mqttRelaySet(payload, 2);
  }
  if (topic == buildTopic(MQTT_RELAY_3_SET_TOPIC)) {
    mqttRelaySet(payload, 3);
  }
#endif
#ifdef COVER
  if (topic == buildTopic(MQTT_COVER_SET_TOPIC)) {
    mqttCoverTrigger();
    Serial.print(" cover triggered ");
  }
#endif

#ifdef SONOFF
  if (topic == buildTopic(MQTT_SONOFF_SET_TOPIC)) {
    mqttSONOFFSet(payload);
  }
#endif
#ifdef BUTTON1
  if (topic == MQTT_BUTTON1_STATE_TOPIC) {
    mqttBUTTONState(payload, 1);
  }
#endif
#ifdef BUTTON2
  if (topic == MQTT_BUTTON2_STATE_TOPIC) {
    mqttBUTTONState(payload, 2);
  }
#endif
#ifdef BUTTON3
  if (topic == MQTT_BUTTON3_STATE_TOPIC) {
    mqttBUTTONState(payload, 3);
  }
#endif
#ifdef BUTTON4
  if (topic == MQTT_BUTTON4_STATE_TOPIC) {
    mqttBUTTONState(payload, 4);
  }
#endif
#ifdef TWO_WAY_SWITCH
  if (topic == TWO_WAY_SWITCH_STATE_TOPIC) {
    if (payload == "on") {
      digitalWrite (TWO_WAY_SWITCH_LED_PIN,TWO_WAY_SWITCH_LED_ON_LEVEL);
    }
    if (payload == "off") {
      digitalWrite (TWO_WAY_SWITCH_LED_PIN,TWO_WAY_SWITCH_LED_OFF_LEVEL);      
    }
  }
#endif
#ifdef ENV_DISPLAY
  if (topic == DISPLAY_TOPIC_1) {
    displayBuffer[0] = payload;
  }
  if (topic == DISPLAY_TOPIC_2) {
    displayBuffer[1] = payload;
  }
  if (topic == DISPLAY_TOPIC_3) {
    displayBuffer[2] = payload;
  }
  if (topic == DISPLAY_TOPIC_4) {
    displayBuffer[3] = payload;
  }
  if (topic == DISPLAY_TOPIC_5) {
    displayBuffer[4] = payload;
  }
  if (topic == DISPLAY_TOPIC_6) {
    displayBuffer[5] = payload;
  }
  if (topic == DISPLAY_TOPIC_7) {
    displayBuffer[6] = payload;
  }
  if (topic == DISPLAY_TOPIC_8) {
    displayBuffer[7] = payload;
  }  
  if (topic == DISPLAY_TIME_TOPIC) {
    currentTime = payload;
  }
  if (topic == DISPLAY_DATE_TOPIC) {
    currentDate = payload;
  }
  
  
#endif

}

void setup(void) {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Booting Sketch...");

#ifdef GPIO_STATIC_OUTPUT
  pinMode(GPIO_STATIC_OUTPUT, OUTPUT);
  digitalWrite(GPIO_STATIC_OUTPUT, GPIO_STATIC_OUTPUT_LEVEL);
#endif

#ifdef OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
#endif

#ifdef DHT_SENSOR
  dht.begin();
#endif
#ifdef DHT_SUPPLY_PIN
  pinMode(DHT_SUPPLY_PIN, OUTPUT);
  digitalWrite(DHT_SUPPLY_PIN, HIGH);
#endif

#ifdef SOIL_HUMIDITY_SENSOR
  pinMode(SOIL_HUMIDITY_SUPPLY_PIN, OUTPUT);
#endif

#ifdef LIGHT_SENSOR_PIN
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LIGHT_SENSOR_PIN), LIGHTInterruptHandler, RISING);
#endif

#ifdef SONOFF
  pinMode(SONOFF_PIN, OUTPUT);
  delay (10);
  digitalWrite(SONOFF_PIN, LOW);
  delay (10);
  pinMode(SONOFF_LED_PIN, OUTPUT);
  delay (10);
  digitalWrite(SONOFF_LED_PIN, HIGH);
  delay (10);
  pinMode(SONOFF_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SONOFF_BUTTON_PIN), SONOFFInterruptHandler, FALLING);
  delay (10);
#endif

#ifdef BUTTON1
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), BUTTON1InterruptHandler, BUTTON1_DIR);
#ifdef BUTTON1_LED_PIN
  pinMode(BUTTON1_LED_PIN, OUTPUT);
#endif
#endif
#ifdef BUTTON2
  pinMode(BUTTON2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), BUTTON2InterruptHandler, BUTTON2_DIR);
#ifdef BUTTON2_LED_PIN
  pinMode(BUTTON2_LED_PIN, OUTPUT);
#endif
#endif
#ifdef BUTTON3
  pinMode(BUTTON3_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON3_PIN), BUTTON3InterruptHandler, BUTTON3_DIR);
#ifdef BUTTON3_LED_PIN
  pinMode(BUTTON3_LED_PIN, OUTPUT);
#endif
#endif
#ifdef BUTTON4
  pinMode(BUTTON4_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON4_PIN), BUTTON4InterruptHandler, BUTTON4_DIR);
#ifdef BUTTON4_LED_PIN
  pinMode(BUTTON4_LED_PIN, OUTPUT);
#endif
#endif
#ifdef BUTTON5
  pinMode(BUTTON5_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON5_PIN), BUTTON5InterruptHandler, BUTTON5_DIR);
#endif
#ifdef BUTTON6
  pinMode(BUTTON6_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON6_PIN), BUTTON6InterruptHandler, BUTTON6_DIR);
#endif
#ifdef BUTTON7
  pinMode(BUTTON7_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON7_PIN), BUTTON7InterruptHandler, BUTTON7_DIR);
#endif
#ifdef BUTTON8
  pinMode(BUTTON8_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON8_PIN), BUTTON8InterruptHandler, BUTTON8_DIR);
#endif

#ifdef TWO_WAY_SWITCH
  pinMode(TWO_WAY_SWITCH_ON_PIN, INPUT_PULLUP);
  pinMode(TWO_WAY_SWITCH_OFF_PIN, INPUT_PULLUP);
  pinMode(TWO_WAY_SWITCH_LED_PIN,OUTPUT);
  digitalWrite(TWO_WAY_SWITCH_LED_PIN,TWO_WAY_SWITCH_LED_OFF_LEVEL);
  attachInterrupt(digitalPinToInterrupt(TWO_WAY_SWITCH_ON_PIN), TWO_WAY_SWITCHInterruptHandler, TWO_WAY_SWITCH_DIR);
  attachInterrupt(digitalPinToInterrupt(TWO_WAY_SWITCH_OFF_PIN), TWO_WAY_SWITCHInterruptHandler, TWO_WAY_SWITCH_DIR);
#endif

#ifdef COVER
  pinMode(COVER_STATE1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COVER_STATE1_PIN), COVERInterruptHandler, COVER_STATE1_DIR);
  pinMode(COVER_TRIGGER1_PIN, OUTPUT);
  pinMode(COVER_TRIGGER2_PIN, OUTPUT);
  
#endif

#ifdef RELAY0
  pinMode(RELAY0_PIN, OUTPUT);
  digitalWrite(RELAY0_PIN, LOW);
#endif
#ifdef RELAY1
  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
#endif
#ifdef RELAY2
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW);
#endif
#ifdef RELAY3
  pinMode(RELAY3_PIN, OUTPUT);
  digitalWrite(RELAY3_PIN, LOW);
#endif

#ifdef BME_SENSOR
  Wire.begin(SDA, SCL);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
#endif




  Serial.println();
  Serial.println("Setting up WiFi");

  WiFi.hostname(HOST);
  WiFi.mode(WIFI_STA);

  connectWifi();

  MDNS.begin(ROOM);
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", ROOM, update_path, update_username, update_password);

  if (mqttRunning) {
    mqttSend(buildTopic(MQTT_STARTUP_TOPIC), "-1");
  }

#ifdef SONOFF
  if (mqttRunning) {
    mqttSONOFFSet("off");
  }
#endif

  if (mqttRunning) {
    mqttPublishSensorValues();
  }

#ifdef FORCE_DEEP_SLEEP
  delay(TIME_REPROGRAM * 1000); // time to make reprogramming possible
#endif

#ifdef ENV_DISPLAY
for (i=0;i<DISPLAY_COUNT;i++) {
  displayBuffer[i] = "4.20";
}
#endif

#ifdef OLED

#endif
}

void loop(void) {
  unsigned long currentMillis = millis();
//  Serial.println("Loop...");
  wifiRunning = WiFiConnected();

  if (!wifiRunning) {
    mqttRunning = false;
    Serial.println();
    Serial.println("WiFi not connected, trying to reconnect");
    delay (5000);
    connectWifi();
  }

  mqttRunning = MQTTConnected();

  if ((wifiRunning) && (!mqttRunning)) {
    Serial.println();
    Serial.println("MQTT not connected, trying to reconnect");
    delay (5000);
    connectMQTT();
  }

  if (wifiRunning) {
    httpServer.handleClient();
  }

#ifdef FORCE_DEEPSLEEP
  if (mqttRunning) {
    mqttSend(buildTopic(MQTT_DEEPSLEEP_TOPIC), String(TIME_DEEPSLEEP));
  }
  ESP.deepSleep(TIME_DEEPSLEEP * 60 * 1000000);
  delay(100);
#endif

  if (mqttRunning) {
    mqtt.loop();
  }

#ifdef SONOFF
  if (updateSONOFF) {
    if (sonoffState == 0) {
      mqttSONOFFSet("off");
    }
    else {
      mqttSONOFFSet("on");
    }
    updateSONOFF = false;
  }
#endif

#ifdef BUTTON1
  if (updateBUTTON1) {
    delay (250);
    if (button1State == 0) {
      mqttSendBUTTON("off", 1);
    }
    else {
      mqttSendBUTTON("on", 1);
    }
    updateBUTTON1 = false;
  }
#endif

#ifdef BUTTON2
  if (updateBUTTON2) {
    delay (250);
    if (button2State == 0) {
      mqttSendBUTTON("off", 2);
    }
    else {
      mqttSendBUTTON("on", 2);
    }
    updateBUTTON2 = false;
  }
#endif

#ifdef BUTTON3
  if (updateBUTTON3) {
    delay (250);
    if (button3State == 0) {
      mqttSendBUTTON("off", 3);
    }
    else {
      mqttSendBUTTON("on", 3);
    }
    updateBUTTON3 = false;
  }
#endif

#ifdef BUTTON4
  if (updateBUTTON4) {
    delay (250);
    if (button4State == 0) {
      mqttSendBUTTON("off", 4);
    }
    else {
      mqttSendBUTTON("on", 4);
    }
    updateBUTTON4 = false;
  }
#endif

#ifdef TWO_WAY_SWITCH
  if (updateTWO_WAY_SWITCH) {
    delay (250);
    if (digitalRead(TWO_WAY_SWITCH_ON_PIN) == TWO_WAY_SWITCH_ON_LEVEL) {
      TWO_WAY_SWITCHState = 1;
    }
    else if (digitalRead(TWO_WAY_SWITCH_OFF_PIN) == TWO_WAY_SWITCH_ON_LEVEL) {
      TWO_WAY_SWITCHState = 2;
    }
    else {
      TWO_WAY_SWITCHState = 0;
    }
    mqttSendTWO_WAY_SWITCH();
    updateTWO_WAY_SWITCH = false;
  }
#endif

#ifdef COVER
  if (updateCOVER) {
    delay (250);
    if (digitalRead(COVER_STATE1_PIN) == LOW) {
      coverState = 1; // closed
    }
    else {
      coverState = 0; // not closed
    }
    mqttSendCOVER();
    updateCOVER = false;
  }
#endif

#ifdef LIGHT_SENSOR_PIN
  if (increaseLightPulseCnt) {
    lightPulseCnt++;
    Serial.println(String(lightPulseCnt));
    mqttSend(buildTopic(MQTT_LIGHT_PULSE_COUNTER_TOPIC), String(lightPulseCnt));  
    increaseLightPulseCnt = false;
    delay (1000);
  }
#endif
//  Serial.println(String(currentMillis - MQTT_SENSOR_TIMER_previousMillis));
  if (currentMillis - MQTT_SENSOR_TIMER_previousMillis >= MQTT_SENSOR_TIMER * 1000) {
    // save the last time you blinked the LED
    MQTT_SENSOR_TIMER_previousMillis = currentMillis;
    Serial.println("Counter for Sensors...");
    if (mqttRunning) {
      mqttPublishSensorValues();
    }
  }
//  Serial.println(String(currentMillis - MQTT_STATE_TIMER_previousMillis));
  if (currentMillis - MQTT_STATE_TIMER_previousMillis >= MQTT_STATE_TIMER * 1000) {
    // save the last time you blinked the LED
    MQTT_STATE_TIMER_previousMillis = currentMillis;
    Serial.println("Counter for States...");
    if (mqttRunning) {
      mqttPublishStateValues();
    }
  }
//  Serial.println(String(currentMillis - MQTT_DISPLAY_TIMER_previousMillis));
  if (currentMillis - MQTT_DISPLAY_TIMER_previousMillis >= 10 * 1000) {
    // save the last time you blinked the LED
    MQTT_DISPLAY_TIMER_previousMillis = currentMillis;
    Serial.println("Counter for Display...");
    if (mqttRunning) {
      mqttPublishDisplayValues();
    }
  }
}

void connectWifi() {
  Serial.println();
  Serial.println("Connecting to Wifi...");
  Serial.println(WIFI_SSID);

#ifdef SONOFF
  digitalWrite(SONOFF_LED_PIN, HIGH);
  delay (200);
  digitalWrite(SONOFF_LED_PIN, HIGH);
#endif

  do { 
    wifiTries++;
    delay (1000);
    Serial.print("Trying to connect:");
    Serial.println(String(wifiTries));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  while (WiFi.waitForConnectResult() != WL_CONNECTED);

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    Serial.println("ok");
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiRunning = true;
    connectMQTT();
  }
  else {
    Serial.println("Wifi connection failed.");
  }
}

void connectMQTT() {
  Serial.println();
  Serial.println("Connecting to MQTT...");
  mqtt.begin(MQTT_BROKER, net);
  mqtt.onMessage(messageReceived);
  mqtt.connect(HOST);

  if (MQTTConnected()) {
    Serial.println("ok");
    Serial.println();
    Serial.println(buildTopic(MQTT_PING_TOPIC));
    mqtt.subscribe(buildTopic(MQTT_PING_TOPIC));
    mqtt.subscribe(buildTopic(MQTT_FLASH_TOPIC));
    
#ifdef RELAY
    mqtt.subscribe(buildTopic(MQTT_RELAY_0_SET_TOPIC));
    mqtt.subscribe(buildTopic(MQTT_RELAY_1_SET_TOPIC));
    mqtt.subscribe(buildTopic(MQTT_RELAY_2_SET_TOPIC));
    mqtt.subscribe(buildTopic(MQTT_RELAY_3_SET_TOPIC));
#endif
#ifdef SONOFF
    mqtt.subscribe(buildTopic(MQTT_SONOFF_SET_TOPIC));
#endif
#ifdef COVER
    mqtt.subscribe(buildTopic(MQTT_COVER_SET_TOPIC));
#endif
#ifdef BUTTON1
    mqtt.subscribe(MQTT_BUTTON1_STATE_TOPIC);
#endif
#ifdef BUTTON2
    mqtt.subscribe(MQTT_BUTTON2_STATE_TOPIC);
#endif
#ifdef BUTTON3
    mqtt.subscribe(MQTT_BUTTON3_STATE_TOPIC);
#endif
#ifdef BUTTON4
    mqtt.subscribe(MQTT_BUTTON4_STATE_TOPIC);
#endif
#ifdef TWO_WAY_SWITCH
    mqtt.subscribe(TWO_WAY_SWITCH_STATE_TOPIC);
#endif
#ifdef ENV_DISPLAY
    mqtt.subscribe(DISPLAY_TOPIC_1);
    mqtt.subscribe(DISPLAY_TOPIC_2);
    mqtt.subscribe(DISPLAY_TOPIC_3);
    mqtt.subscribe(DISPLAY_TOPIC_4);    
    mqtt.subscribe(DISPLAY_TOPIC_5);
    mqtt.subscribe(DISPLAY_TOPIC_6);
    mqtt.subscribe(DISPLAY_TOPIC_7);
    mqtt.subscribe(DISPLAY_TOPIC_8);    
    mqtt.subscribe(DISPLAY_TIME_TOPIC); 
    mqtt.subscribe(DISPLAY_DATE_TOPIC);            
#endif
    mqttRunning = true;
  }
  if (mqttRunning) {
    mqttSend(buildTopic(MQTT_STARTUP_TOPIC), String(wifiTries));
    delay (500);
    mqttSend(buildTopic(MQTT_STARTUP_TOPIC), "0");
    mqttSend(buildTopic(MQTT_IP_TOPIC), WiFi.localIP().toString());     
  }
}


// build topics with device id on the fly
char* buildTopic(const char *topic) {
  sprintf(char_buffer, "%s/%s%s", LOCATION, ROOM, topic);
  return char_buffer;
}



double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  temp = (int) (4 * temp + .5);
  return (double) temp / 4;
}

#ifdef BUTTON
void mqttBUTTONState(String payload, int button) {
  switch (button) {
#ifdef BUTTON1
    case 1:
      if (payload == "on") {
        button1State = 1;
#ifdef BUTTON1_LED_PIN
        digitalWrite(BUTTON1_LED_PIN, BUTTON1_LED_ACTIVE);
#endif
      }
      if (payload == "off") {
        button1State = 0;
#ifdef BUTTON1_LED_PIN
        digitalWrite(BUTTON1_LED_PIN, BUTTON1_LED_N_ACTIVE);
#endif
      }
      break;
#endif
#ifdef BUTTON2
    case 2:
      if (payload == "on") {
        button2State = 1;
#ifdef BUTTON2_LED_PIN
        digitalWrite(BUTTON2_LED_PIN, BUTTON2_LED_ACTIVE);
#endif
      }
      if (payload == "off") {
        button2State = 0;
#ifdef BUTTON2_LED_PIN
        digitalWrite(BUTTON2_LED_PIN, BUTTON2_LED_N_ACTIVE);
#endif
      }
      break;
#endif
#ifdef BUTTON3
    case 3:
      if (payload == "on") {
        button3State = 1;
#ifdef BUTTON3_LED_PIN
        digitalWrite(BUTTON3_LED_PIN, BUTTON3_LED_ACTIVE);
#endif
      }
      if (payload == "off") {
        button3State = 0;
#ifdef BUTTON3_LED_PIN
        digitalWrite(BUTTON3_LED_PIN, BUTTON3_LED_N_ACTIVE);
#endif
      }
      break;
#endif
#ifdef BUTTON4
    case 4:
      if (payload == "on") {
        button4State = 1;
#ifdef BUTTON4_LED_PIN
        digitalWrite(BUTTON4_LED_PIN, BUTTON4_LED_ACTIVE);
#endif
      }
      if (payload == "off") {
        button4State = 0;
#ifdef BUTTON4_LED_PIN
        digitalWrite(BUTTON4_LED_PIN, BUTTON4_LED_N_ACTIVE);
#endif
      }
      break;
#endif
  }
}
#endif

void mqttSendBUTTON(String payload, int button) {
  switch (button) {
#ifdef BUTTON1
    case 1:
      if (button1State == 0) {
        mqttSend(MQTT_BUTTON1_SET_TOPIC, "on");
      }
      else {
        mqttSend(MQTT_BUTTON1_SET_TOPIC, "off");
      }
      updateBUTTON1 = false;
      break;
#endif
#ifdef BUTTON2
    case 2:
      if (button2State == 0) {
        mqttSend(MQTT_BUTTON2_SET_TOPIC, "on");
      }
      else {
        mqttSend(MQTT_BUTTON2_SET_TOPIC, "off");
      }
      updateBUTTON2 = false;
      break;
#endif
#ifdef BUTTON3
    case 3:
      if (button3State == 0) {
        mqttSend(MQTT_BUTTON3_SET_TOPIC, "on");
      }
      else {
        mqttSend(MQTT_BUTTON3_SET_TOPIC, "off");
      }
      updateBUTTON3 = false;
      break;
#endif
#ifdef BUTTON4
    case 4:
      if (button4State == 0) {
        mqttSend(MQTT_BUTTON4_SET_TOPIC, "on");
      }
      else {
        mqttSend(MQTT_BUTTON4_SET_TOPIC, "off");
      }
      updateBUTTON4 = false;
      break;
#endif
  }
}

#ifdef BUTTON1
void BUTTON1InterruptHandler() {
  Serial.println("BUTTON 1 pressed: ");
  if (digitalRead(BUTTON1_PIN) == LOW) {
    updateBUTTON1 = true;
  }
  Serial.println(String(button1State));
}
#endif

#ifdef BUTTON2
void BUTTON2InterruptHandler() {
  Serial.println("BUTTON 2 pressed: ");
  if (digitalRead(BUTTON2_PIN) == LOW) {
    updateBUTTON2 = true;
  }
  Serial.println(String(button2State));
}

#endif

#ifdef BUTTON3
void BUTTON3InterruptHandler() {
  Serial.println("BUTTON 3 pressed: ");
  if (digitalRead(BUTTON3_PIN) == LOW) {
    updateBUTTON3 = true;
  }
  Serial.println(String(button3State));
}
#endif

#ifdef BUTTON4
void BUTTON4InterruptHandler() {
  Serial.println("BUTTON 4 pressed: ");
  if (digitalRead(BUTTON4_PIN) == LOW) {
    updateBUTTON4 = true;
  }
  Serial.println(String(button4State));
}
#endif

#ifdef TWO_WAY_SWITCH
void TWO_WAY_SWITCHInterruptHandler(void) {
  updateTWO_WAY_SWITCH = true;
}

void mqttSendTWO_WAY_SWITCH (void) {
  if (TWO_WAY_SWITCHState == 2) {
    mqttSend(TWO_WAY_SWITCH_SET_TOPIC, "shutdown");
  } 
  else if (TWO_WAY_SWITCHState == 1) {
    mqttSend(TWO_WAY_SWITCH_SET_TOPIC, "shutdown_revoke");
    mqttSend(TWO_WAY_SWITCH_SET_TOPIC, "on");
  }
  else if (TWO_WAY_SWITCHState == 0) {
    mqttSend(TWO_WAY_SWITCH_SET_TOPIC, "shutdown_revoke");
    mqttSend(TWO_WAY_SWITCH_SET_TOPIC, "off");
  }
  delay (500);
}

#endif


#ifdef COVER
void COVERInterruptHandler() {
  Serial.println("COVER changed: ");
  updateCOVER = true;
}
void mqttSendCOVER(void) {
  if (coverState == 1) {
    mqttSend(buildTopic(MQTT_COVER_STATE_TOPIC), "0");
  }
  else {
    mqttSend(buildTopic(MQTT_COVER_STATE_TOPIC), "1");
  }
}

#endif


#ifdef LIGHT_SENSOR_PIN
void LIGHTInterruptHandler() {
  Serial.println("LIGHT pulse detected: ");
  if (digitalRead(LIGHT_SENSOR_PIN) == HIGH) {
    increaseLightPulseCnt = true;
  }
}
#endif

#ifdef SONOFF
void SONOFFInterruptHandler() {
  Serial.println("SONOFF button pressed: ");
  if (digitalRead(SONOFF_BUTTON_PIN) == LOW) {
    if (sonoffState == 0) {
      sonoffState = 1;
    }
    else {
      sonoffState = 0;
    }
    updateSONOFF = true;
  }
  Serial.println(String(sonoffState));
}

void mqttSONOFFSet(String payload) {
  if ((payload == "on") && (sonoffState != 2)) {
    sonoffState = 1;
    digitalWrite(SONOFF_PIN, HIGH);
    digitalWrite(SONOFF_LED_PIN, LOW);
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "on");
  }
  if (payload == "off") {
    sonoffState = 0;
    digitalWrite(SONOFF_PIN, LOW);
    digitalWrite(SONOFF_LED_PIN, HIGH);
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "off");
  }
  if (payload == "shutdown") {
    sonoffState = 2;
    digitalWrite(SONOFF_PIN, LOW);
    digitalWrite(SONOFF_LED_PIN, HIGH);
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "shutdown");
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "off");
  }
  if (payload == "shutdown_revoke") {
    sonoffState = 0;
    digitalWrite(SONOFF_PIN, LOW);
    digitalWrite(SONOFF_LED_PIN, HIGH);
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "off");
  }
  delay (500);
}
#endif

#ifdef COVER
void mqttCoverTrigger(void) {
  Serial.println("triggering relay");
  digitalWrite(COVER_TRIGGER1_PIN, HIGH);
  delay (1000);
  digitalWrite(COVER_TRIGGER1_PIN, LOW);
}
#endif

#ifdef RELAY
void mqttRelaySet(String payload, int relay) {
  switch (relay) {
#ifdef RELAY0
    case 0:
      if (payload == "on") {
        digitalWrite(RELAY0_PIN, HIGH);
        mqttSend(buildTopic(MQTT_RELAY_0_STATE_TOPIC), "on");
        relay0State = 1;
      }
      if (payload == "off") {
        digitalWrite(RELAY0_PIN, LOW);
        mqttSend(buildTopic(MQTT_RELAY_0_STATE_TOPIC), "off");
        relay0State = 0;
      }
      break;
#endif
#ifdef RELAY1
    case 1:
      if (payload == "on") {
        digitalWrite(RELAY1_PIN, HIGH);
        mqttSend(buildTopic(MQTT_RELAY_1_STATE_TOPIC), "on");
        relay1State = 1;
      }
      if (payload == "off") {
        digitalWrite(RELAY1_PIN, LOW);
        mqttSend(buildTopic(MQTT_RELAY_1_STATE_TOPIC), "off");
        relay1State = 0;
      }
      break;
#endif
#ifdef RELAY2
    case 2:
      if (payload == "on") {
        digitalWrite(RELAY2_PIN, HIGH);
        mqttSend(buildTopic(MQTT_RELAY_2_STATE_TOPIC), "on");
        relay2State = 1;
      }
      if (payload == "off") {
        digitalWrite(RELAY2_PIN, LOW);
        mqttSend(buildTopic(MQTT_RELAY_2_STATE_TOPIC), "off");
        relay2State = 0;
      }
      break;
#endif
#ifdef RELAY3
    case 3:
      if (payload == "on") {
        digitalWrite(RELAY3_PIN, HIGH);
        mqttSend(buildTopic(MQTT_RELAY_3_STATE_TOPIC), "on");
        relay3State = 1;
      }
      if (payload == "off") {
        digitalWrite(RELAY3_PIN, LOW);
        mqttSend(buildTopic(MQTT_RELAY_3_STATE_TOPIC), "off");
        relay3State = 0;
      }
      break;
#endif
  }
}
#endif


#ifdef ONE_WIRE_SENSOR
float getDsTemp() { // https://blog.silvertech.at/arduino-temperatur-messen-mit-1wire-sensor-ds18s20ds18b20ds1820/
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -999;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -888;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -777;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}
#endif


bool WiFiConnected (void) {
  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    return (true);
  }
  else {
    return (false);
  }
}

bool MQTTConnected (void) {
  if (mqtt.connected()) {
    return (true);
  }
  else {
    return (false);
  }
}

void mqttSend (char *topic, String message) {
  if (mqttRunning) {
    Serial.println("Sending...");
    mqtt.publish(topic, message);
  }
}

void mqttPublishSensorValues() {
  float t;
  float h;
  Serial.println("Sensors...");

  mqttSend(buildTopic(MQTT_WIFI_RSSI_TOPIC), String(WiFi.RSSI()));
  mqttSend(buildTopic(MQTT_WIFI_CHANNEL_TOPIC), String(WiFi.channel()));
 
#ifdef SOIL_HUMIDITY_SENSOR
  digitalWrite(SOIL_HUMIDITY_SUPPLY_PIN, HIGH);
  delay (1000);
  soilHumidityRaw = analogRead(A0);
  soilHumidity = (double) (1024 - soilHumidityRaw) / 1024 * 100;
  digitalWrite(SOIL_HUMIDITY_SUPPLY_PIN, LOW);
  if ((soilHumidityRaw > 10) || (soilHumidityRaw < 1000)) {
    mqttSend(buildTopic(MQTT_SOIL_HUMIDITY_TOPIC), String(soilHumidity));
  }
#endif

#ifdef LIGHT_SENSOR
  lightIntensityRaw = analogRead(A0);
  lightIntensity = (double) (1024 - lightIntensityRaw) / 1024 * 100;
//  if ((lightIntensityRaw > 10) || (lightIntensityRaw < 1000)) {
    mqttSend(buildTopic(MQTT_LIGHT_INTENSITY_TOPIC), String(lightIntensity));
//  }
#endif

#ifdef DHT_SENSOR
  h = dht.readHumidity() + DHT_HUM_OFFSET;
  t = dht.readTemperature() + DHT_TEMP_OFFSET;
#ifdef FORCE_DEEPSLEEP
  digitalWrite(DHT_SUPPLY_PIN, LOW);
#endif
  // Check if any reads failed and exit early (to try again).
  if (!isnan(h) || !isnan(t)) {
    mqttSend(buildTopic(MQTT_TEMP_TOPIC), String(t));
    mqttSend(buildTopic(MQTT_HUMIDITY_TOPIC), String(h));
  }
#endif

#ifdef ONE_WIRE_SENSOR
  t = getDsTemp();
  if ((t < 85.0) && (t > -40.0)) {
    mqttSend(buildTopic(MQTT_DS18_TEMP_TOPIC), String(t));
  }
#endif

#ifdef BME_SENSOR
  if ((bme.readTemperature() > -50.0) && (bme.readPressure() > 0.0)) {
    mqttSend(buildTopic(MQTT_BME_TEMPERATURE_TOPIC), String(bme.readTemperature()));
    mqttSend(buildTopic(MQTT_BME_HUMIDITY_TOPIC), String(bme.readHumidity()));
    mqttSend(buildTopic(MQTT_BME_PRESSURE_TOPIC), String(bme.readPressure() / 100.0F));
  }
  else {
    mqttSend(buildTopic(MQTT_SENSOR_FAILURE_TOPIC), "failure detected");
    mqttSend(buildTopic(MQTT_SENSOR_FAILURE_TOPIC), "");
  }
#endif

#ifdef FORCE_DEEPSLEEP
  mqttSend(buildTopic(MQTT_BAT_VOLT_TOPIC), String(ESP.getVcc() / 1000.0));
#endif

#ifdef CURRENT_SENSOR
  currentEnergy = lightPulseCnt/500.0;
  currentPower = currentEnergy*1000.0*3600.0/TIME_SENSOR_UPDATE;
  mqttSend(buildTopic(MQTT_CURRENT_SENSOR_CUR_ENERGY_TOPIC),String(currentEnergy));
  mqttSend(buildTopic(MQTT_CURRENT_SENSOR_CUR_POWER_TOPIC),String(currentPower));  
  lightPulseCnt = 0;
#endif

}

void mqttPublishStateValues() {
  
  Serial.println("States...");

  Serial.println("Button");
  
  if (digitalRead(SONOFF_BUTTON_PIN) == LOW) {
    Serial.println("Button low");
  }
  if (digitalRead(SONOFF_BUTTON_PIN) == HIGH) {
    Serial.println("Button high");
  }

  if (heartbeat == 0) {
    mqttSend(buildTopic(MQTT_HEARTBEAT_TOPIC), "0");
    heartbeat = 1;
  }
  else {
    mqttSend(buildTopic(MQTT_HEARTBEAT_TOPIC), "1");
    heartbeat = 0;    
  }
    
#ifdef SONOFF
  if (sonoffState == 0) {
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "off");
  }
  if (sonoffState == 1) {
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "on");
  }
  if (sonoffState == 2) {
    mqttSend(buildTopic(MQTT_SONOFF_STATE_TOPIC), "off");
  }
#endif

#ifdef TWO_WAY_SWITCH
//  mqttSendTWO_WAY_SWITCH();
#endif

#ifdef RELAY0
  if (relay0State == 0) {
    mqttSend(buildTopic(MQTT_RELAY_0_STATE_TOPIC), "off");
  }
  if (relay0State == 1) {
    mqttSend(buildTopic(MQTT_RELAY_0_STATE_TOPIC), "on");
  }
#endif

#ifdef RELAY1
  if (relay1State == 0) {
    mqttSend(buildTopic(MQTT_RELAY_1_STATE_TOPIC), "off");
  }
  if (relay1State == 1) {
    mqttSend(buildTopic(MQTT_RELAY_1_STATE_TOPIC), "on");
  }
#endif

#ifdef RELAY2
  if (relay2State == 0) {
    mqttSend(buildTopic(MQTT_RELAY_2_STATE_TOPIC), "off");
  }
  if (relay2State == 1) {
    mqttSend(buildTopic(MQTT_RELAY_2_STATE_TOPIC), "on");
  }
#endif

#ifdef RELAY3
  if (relay3State == 0) {
    mqttSend(buildTopic(MQTT_RELAY_3_STATE_TOPIC), "off");
  }
  if (relay3State == 1) {
    mqttSend(buildTopic(MQTT_RELAY_3_STATE_TOPIC), "on");
  }
#endif

#ifdef COVER
  if (digitalRead(COVER_STATE1_PIN) == LOW) {
    coverState = 1; // closed
  }
  else {
    coverState = 0; // not closed
  }
  mqttSendCOVER();
#endif
}

void mqttPublishDisplayValues() {
  float t;
  float h;
  Serial.println("Display...");

#ifdef ENV_DISPLAY
  display.clear();
  // draw the current demo method
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  switch (displayCur) {
    case 0: 
      display.drawString(0,0,DISPLAY_HEAD_1);
      break;
    case 1: 
      display.drawString(0,0,DISPLAY_HEAD_2); 
      break;
    case 2: 
      display.drawString(0,0,DISPLAY_HEAD_3); 
      break;
    case 3: 
      display.drawString(0,0,DISPLAY_HEAD_4); 
      break;
    case 4: 
      display.drawString(0,0,DISPLAY_HEAD_5); 
      break;
    case 5: 
      display.drawString(0,0,DISPLAY_HEAD_6); 
      break;
    case 6: 
      display.drawString(0,0,DISPLAY_HEAD_7); 
      break;
    case 7: 
      display.drawString(0,0,DISPLAY_HEAD_8); 
      break;
  }
  
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_24);
  display.drawString(64,17,displayBuffer[displayCur]);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  switch (displayCur) {
    case 0: 
      display.drawString(66,17,DISPLAY_UNIT_1);
      break;
    case 1: 
      display.drawString(66,17,DISPLAY_UNIT_2); 
      break;
    case 2: 
      display.drawString(66,17,DISPLAY_UNIT_3); 
      break;
    case 3: 
      display.drawString(66,17,DISPLAY_UNIT_4); 
      break;
    case 4: 
      display.drawString(66,17,DISPLAY_UNIT_5); 
      break;
    case 5: 
      display.drawString(66,17,DISPLAY_UNIT_6); 
      break;
    case 6: 
      display.drawString(66,17,DISPLAY_UNIT_7); 
      break;
    case 7: 
      display.drawString(66,17,DISPLAY_UNIT_8); 
      break;
  }
  
  displayCur++;
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(64,42,displayBuffer[displayCur]);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  switch (displayCur) {
    case 0: 
      display.drawString(66,42,DISPLAY_UNIT_1);
      break;
    case 1: 
      display.drawString(66,42,DISPLAY_UNIT_2); 
      break;
    case 2: 
      display.drawString(66,42,DISPLAY_UNIT_3); 
      break;
    case 3: 
      display.drawString(66,42,DISPLAY_UNIT_4); 
      break;
    case 4: 
      display.drawString(66,42,DISPLAY_UNIT_5); 
      break;
    case 5: 
      display.drawString(66,42,DISPLAY_UNIT_6); 
      break;
    case 6: 
      display.drawString(66,42,DISPLAY_UNIT_7); 
      break;
    case 7: 
      display.drawString(66,42,DISPLAY_UNIT_8); 
      break;
  }  
  
  displayCur++;
  if (displayCur > (DISPLAY_COUNT-1)) displayCur = 0;
  
  //display.drawString(64, 12, "Hello World!"); 
  //display.drawString(64, 24, String(millis()));
  // write the buffer to the display
  display.display();
#endif

}

void mqttPong(String payload) {
  mqttSend(buildTopic(MQTT_PONG_TOPIC), payload);
}

