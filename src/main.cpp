//////////////////////////////////////////
//                                      //
//  CREATED BY  : Jérémy Noverraz       //
//  CREATED ON  : 31.01.2022            //
//  VERSION     : 1.0                   //                                      
//  DESCRIPTION : BME680 data over MQTT //
//                                      //
//////////////////////////////////////////

#include <Arduino.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SPI.h>

#define WIFI_SSID "SwissLabsBox2"  // WiFi SSID
#define WIFI_PASS "JP3YMhAdx4rbvyru3S"  // WPA2 password

//Mosquitto broker on HomeAssistant
#define MQTT_HOST IPAddress(192,168,1,4)
#define MQTT_PORT 1883
#define MQTT_USER "helitekmqttuser"
#define MQTT_PASS "W3lc0m32h3l1t3k"

//Temperature MQTT topics
#define TEMP_TOPIC "helitek/sensor/temperature/temperature"
#define TEMP_STATE_TOPIC "helitek/sensor/temperature/state"

//Humidity MQTT topics
#define HUM_TOPIC "helitek/sensor/humidity/humidity"
#define HUM_STATE_TOPIC "helitek/sensor/humidity/state"

//Pressure MQTT topics
#define PRESS_TOPIC "helitek/sensor/pressure/pressure"
#define PRESS_STATE_TOPIC "helitek/sensor/pressure/state"

//GasResistance MQTT topics
#define GAS_TOPIC "helitek/sensor/gas/gas"
#define GAS_STATE_TOPIC "helitek/sensor/gas/state"


Adafruit_BME680 bme;

//BME680 sensor address
#define BME680_I2C_ADDR 0x77

//BME680 sensor settings
#define BME680_I2C_BUS 0
#define BME680_I2C_SPEED 400000
#define BME680_I2C_TIMEOUT 1000

//BME680 sensor calibration data
#define BME680_CAL_DATA_SIZE 32

//BME680


// Variables to hold sensor readings
float temperature;
float humidity;
float pressure;
float gas_resistance;


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0; // will store last time temperature was updated
const long interval = 10000; // interval at which to publish a temperature reading (in ms)

void getBMEReadings(){
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println("Failed to begin reading !");
    return;
  }
  if (!bme.endReading()) {
    Serial.println("Failed to end reading !");
    return;
  }
  temperature = bme.temperature;
  humidity = bme.humidity;
  pressure = bme.pressure / 100.0F;
  gas_resistance = bme.gas_resistance;
}


void connectToWifi() {
  // Connect to WiFi access point.
  // If you want to connect to a network with an existing name, try
  //     WiFi.begin(ssid, pass);
  // If you want to connect to a network with an unknown name, try
  //     WiFi.begin(ssid);
  // If the network is unknown, it will be created with the supplied
  //     password.
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void connectToMqtt(){
  Serial.println("Connecting to MQTT...");
  while (!mqttClient.connected()) {
    Serial.print(".");
    delay(500);
    mqttClient.connect();
  }

}

void WifiEvent(WiFiEvent_t  event){
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event){
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  //Serial begin 9600
  Serial.begin(9600);
  Serial.println();

  if(!bme.begin()){
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while(1);
  }

  mqttReconnectTimer = xTimerCreate("mqttReconnectTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiReconnectTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WifiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);

  

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  
  connectToWifi();

  //Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

}

void loop() {
  unsigned long currentMillis = millis();
  //Every X number of seconds (inverval = 10 seconds)
  //It published a new MQTT message
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;

    getBMEReadings();
    Serial.println();
    Serial.printf("Temperature: %0.2f °C\n", temperature);
    Serial.printf("Humidity: %0.2f %%\n", humidity);
    Serial.printf("Pressure: %0.2f hPa\n", pressure);
    Serial.printf("Gas Resistance: %0.2f Ohms\n", gas_resistance);

    //Publish an MQTT message on the topic temperature
    uint16_t packetIdPub1 = mqttClient.publish(TEMP_TOPIC, 1, true, String(temperature).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", TEMP_TOPIC, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperature);

    //Publish an MQTT message on the topic humidity
    uint16_t packetIdPub2 = mqttClient.publish(HUM_TOPIC, 1, true, String(humidity).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", HUM_TOPIC, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity);

    //Publish an MQTT message on the topic pressure
    uint16_t packetIdPub3 = mqttClient.publish(PRESS_TOPIC, 1, true, String(pressure).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", PRESS_TOPIC, packetIdPub3);
    Serial.printf("Message: %.2f \n", pressure);

    //Publish an MQTT message on the topic gas_resistance
    uint16_t packetIdPub4 = mqttClient.publish(GAS_TOPIC, 1, true, String(gas_resistance).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", GAS_TOPIC, packetIdPub4);
    Serial.printf("Message: %.2f \n", gas_resistance);


  }
}