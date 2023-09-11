// ESP32 WROOM Pinout - https://www.wemos.cc/en/latest/d32/d32_pro.html#pin
// libraries:           https://github.com/espressif/arduino-esp32/tree/master/libraries
// sensor humidity -    https://arduino-tutorials.net/tutorial/capacitive-soil-moisture-sensor-arduino
// espressif -          https://docs.espressif.com/projects/arduino-esp32/en/latest/api/deepsleep.html
// BMP280 sensor        https://arduino-projekte.info/produkt/bmp280-5v-temperatur-und-luftdrucksensor/
// PubSub / mqtt        https://github.com/knolleary/pubsubclient

#include "config.h"
#include <dummy.h>
#include <WiFi.h>

#include "WiFiClientSecure.h" // for TLS support (example: https://github.com/knolleary/pubsubclient/pull/851/files)
#include <PubSubClient.h>

#include <Wire.h>   // I2C
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>  // https://github.com/adafruit/Adafruit_BME280_Library/

#define LED_BUILTIN 5   // GPIO5

#define uS_TO_S_FACTOR 1000000ULL         /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30*uS_TO_S_FACTOR  /* Time ESP32 will go to sleep (in seconds) */


#define SOIL_SENSOR 36  // ADC1-CH0 (Sensor VP)


#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_ADDR_BME280 0x76
Adafruit_BME280 bme; // BME280 over I2C
//SCL -> GPIO22
//SDA -> GPIO21

RTC_DATA_ATTR int idx = 1;
struct Humidity {
  uint32_t value = 0;
  float percentage = 0.;
  // calibrare
  uint32_t min = 2500;  // uscat
  uint32_t max = 880;   // ud (in apa)
};
Humidity hu;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Received from mqtt...");
}
const char* mqtt_server = MQTT_URL;
//WiFiClient wifiClient;
WiFiClientSecure wifiClient;
PubSubClient mqtt_client(mqtt_server, MQTT_PORT, callback, wifiClient);
const char* CA_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"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\n" \
"-----END CERTIFICATE-----";


// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(9600);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  //set ADC resolution to 12 bits (0-4096)
  analogReadResolution(12);  // (optional) 12 is default

  initBME280(); // init BME280 lib

  connectToWifi();

  connectToMQTT();
}

// the loop function runs over and over again forever
void loop() {

  //blinkLed(3);

  Serial.print(idx); Serial.println(".");
  readSoilHumidity(SOIL_SENSOR, hu);
  readBME280();  // air temp & pressure

  if (mqtt_client.connected()) {
    Serial.print("sending to mqtt...");

    char buffer[8];
    
    // sending soil humidity
    snprintf (buffer, sizeof(buffer), "%d", hu.value);
    mqtt_client.publish("sensors/experiment/soil_humidity/value", buffer, true);
    snprintf (buffer, sizeof(buffer), "%f", hu.percentage);
    mqtt_client.publish("sensors/experiment/soil_humidity/percent", buffer, true);

    // sending other data
    snprintf (buffer, sizeof(buffer), "%f", bme.readPressure()/ 100.0F);
    mqtt_client.publish("sensors/experiment/air_pressure/value", buffer, true);
    snprintf (buffer, sizeof(buffer), "%f", bme.readTemperature());
    mqtt_client.publish("sensors/experiment/air_temperature/value", buffer, true);
    snprintf (buffer, sizeof(buffer), "%f", bme.readHumidity());
    mqtt_client.publish("sensors/experiment/air_humidity/value", buffer, true);
    snprintf (buffer, sizeof(buffer), "%f", bme.readAltitude(SEALEVELPRESSURE_HPA));
    mqtt_client.publish("sensors/experiment/altitude/value", buffer, true);

    Serial.println("done.");
  } else {
    Serial.println("Skip mqtt: not connected");
  }
  
  Serial.println();
  idx++;
  //delay(15000);  // wait 

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " us");
  //Serial.flush(); 
  esp_deep_sleep_start();
}

//
void connectToWifi() {
  // We start by connecting to a WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("\nWaiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
}

void connectToMQTT() {

  Serial.print("\nConnecting to MQTT... ");
  //mqtt_client.setServer(mqtt_server, 1883);
  wifiClient.setCACert(CA_cert);          // add Root CA certificate
  if (mqtt_client.connect("esp32", MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println("Connected to MQTT");
    //client.publish("outTopic","hello world");
    //client.setCallback(callback);
  } else {
    Serial.println("Failed to connect to MQTT");
    Serial.print("mqtt_client state: ");Serial.println(mqtt_client.state());

    char lastError[100];
    wifiClient.lastError(lastError,100);  //Get the last error for WiFiClientSecure
    Serial.print("TLS state"); Serial.print(lastError);
    //block();
  }
}

/*
  // subscribe to mqtt
  void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}*/

void initBME280() {
  Serial.println(F("BME280 test"));
  unsigned status = bme.begin(I2C_ADDR_BME280);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      block();
  }
}

void block() {
  while (1) delay(10);
}

void readBME280() {
  //Get pressure value
  float pressure = bme.readPressure();
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();

  //Print the results
  Serial.print("Temperature: "); Serial.print(temperature);Serial.println("°C");
  Serial.print("Pressure: ");Serial.print(pressure / 100.0F); Serial.println("hPa");
  Serial.print("Approx. Altitude = "); Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));Serial.println(" m");
  Serial.print("Air Humidity = "); Serial.print(humidity); Serial.println(" %");

  Serial.println();
}


void readSoilHumidity(int gpio, Humidity& hum) {
  hum.value = analogRead(gpio);
  if (hum.value > hum.min) hum.min = hum.value;                                // update min
  if (hum.value < hum.max) hum.max = hum.value;                                // update max
  hum.percentage = (float)(hum.min - hum.value) * 100 / (hum.min - hum.max);  // humidity in percent

  Serial.print("Soil humidity: "); Serial.print(hu.percentage);Serial.print("%");
  Serial.print("\t("); Serial.print(hu.value); Serial.println(")");
}

void blinkLed(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}
