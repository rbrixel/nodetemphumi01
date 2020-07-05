/*
 * EN: ESP-Temperature- and Humidity-Module
 * DE: ESP-Temperatur- und Luftfeuchtigkeits-Modul
 * 
 * Hardware: 
 * - ESP-12F (ESP8266, "Wemos D1 Mini"-Clone)
 * - BME280 (temperature, humidity and air pressure)
 * - DS18B20 (waterproof for outdoor measurement)
 * - ublox Neo-6M GPS-module
 * 
 * Software:
 * - wifi (integrated in esp8266)
 * - EspMQTTClient (aka PubSubClient) 1.8.0
 * - Over-The-Air-Update (ArduinoOTA)
 * - OneWire (OneWire - use 2.3.0 for GPIO16/D0!)
 * - TinyGPSPlus-1.0.2b (http://arduiniana.org/libraries/tinygpsplus/)
 * - Cactus.io BME280 (http://cactus.io/projects/weather/arduino-weather-station-bme280-sensor)
 * 
 * Setup:
 * - Change SSID of your wifi (STASSID)
 * - Change wifi-password (STAPSK)
 * - Change hostname of your module (host)
 * - Change ip-address of your MQTT-Broker (MQTT_BROKER)
 * - Change DS18B20-Pin (ONE_WIRE_BUS)
 * - Change Software-Serial-Pins (RXPin, TXPin)
 * 
 * Pinmapping Wemos D1 Mini:
 * Board  Arduino/GPIO  Special     In use
 * D0     16                        DS18B20 onewire
 * D1     5             SCL - i2c   SCL - i2c
 * D2     4             SDA - i2c   SDA - i2c
 * D3     0                         RX - Neo6m GPS
 * D4     2                         TX - Neo6m GPS
 * D5     14            SCK - SPI
 * D6     12            MISO - SPI
 * D7     13            MOSI - SPI
 * D8     15            SS - SPI
 * TX     1
 * RX     3
 * 
 * Author: René Brixel <mail@campingtech.de>
 * Date: 2020-07-02
 */

// WIFI
#include <ESP8266WiFi.h> // ESP8266
#ifndef STASSID
#define STASSID "*****" // your wifi-name
#define STAPSK  "*****" // your wifi-password
#endif
const char* host = "nodetemphumi01"; // hostname of module
const char* ssid     = STASSID;
const char* password = STAPSK;

// MQTT-Client
#include <PubSubClient.h>
const char* MQTT_BROKER = "192.168.111.199"; // ip-address of your mqtt-broker
WiFiClient espClient;
PubSubClient client(espClient);
//long lastMsg = 0;
char msg[50];
//int value = 0;

// OTA-Update
#include <ESP8266mDNS.h> // ESP8266
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// BME280
// SCL - D1 - GPIO5
// SDA - D2 - GPIO4
#include <cactus_io_BME280_I2C.h>
BME280_I2C bme(0x76); // uint i2c-address

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 16 // GPIO16 - D0
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int sensorCount;

// GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 0, TXPin = 2; // RXPin = D3, TXPin = D4;
static const uint32_t GPSBaud = 9600; // Standard 9600, alternative 4800
float gpsLat = 0.0; // Latitude
float gpsLon = 0.0; // Longitude
float gpsSat = 0.0; // Satellites
float gpsAlt = 0.0; // Altitude
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// Interval-values (for non blocking code)
unsigned long nbcPreviousMillis = 0; // holds last timestamp
const long nbcInterval = 5000; // interval in milliseconds (1000 milliseconds = 1 second)

void setup() {
  // Start serial-connection
  Serial.begin(115200);

  // Start BME280
  if (!bme.begin()) {
    Serial.println("Could not find BME280 sensor, check wiring");
    while (1);
  }
  bme.setTempCal(-1);// Temp was reading high so subtract 1 degree

  // Start DS18B20-sensor
  sensors.begin();
  sensorCount = sensors.getDS18Count();

  // Start wifi and connect to access point
  setup_wifi();

  // Start wifi and connect to access point
  setup_otau();

  // Start MQTT
  client.setServer(MQTT_BROKER, 1883); // ip-address, port 1883
  //client.setCallback(callback); // only needed for subscription of topic

  // Start Software-serial for gps
  ss.begin(GPSBaud); // connect gps sensor (standard 9600)
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA); // set explicit as wifi-client - important that it not acts as an ap AND client!
  WiFi.hostname(host); // set hostname of module
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
}

void setup_otau() {
  // ArduinoOTA.setPort(8266); // Port defaults to 8266
  ArduinoOTA.setHostname(host); // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setPassword("campingtech"); // No authentication by default
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

/*
// Callback for subscribe MQTT-topic:
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message [");
  Serial.print(topic);
  Serial.print("] ");
  char msg[length + 1];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg[i] = (char)payload[i];
  }
  Serial.println();
  
  msg[length] = '\0';
  Serial.println(msg);
 
  if(strcmp(msg,"on")==0) {
    //digitalWrite(13, HIGH);
  }
  else if(strcmp(msg,"off")==0) {
    //digitalWrite(13, LOW);
  }
}
*/

/*
// MQTT-Reconnect (for subscribe and publish):
void reconnect() {
  while (!client.connected()) {
    Serial.println("Reconnecting MQTT...");
    if (!client.connect(host)) {
      // To connect with credetials: boolean connect (clientID, [username, password], [willTopic, willQoS, willRetain, willMessage], [cleanSession]) // https://pubsubclient.knolleary.net/api
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
  client.subscribe("/home/data");
  Serial.println("MQTT Connected!");
}
*/

// MQTT-Reconnect (for publishing only):
void reconnect() {
  while (!client.connected()) {
    Serial.print("Reconnecting...");
    if (!client.connect(host)) {
      // To connect with credetials: boolean connect (clientID, [username, password], [willTopic, willQoS, willRetain, willMessage], [cleanSession])
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
  Serial.println("MQTT Connected!");
}

void loop() {
  // --- realtime code ---

  // Arduino-OTA:
  ArduinoOTA.handle();
  
  // MQTT:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        gpsLat = gps.location.lat();
        gpsLon = gps.location.lng();
      }
      if (gps.satellites.isValid()) {
        gpsSat = gps.satellites.value();
      }
      if (gps.altitude.isValid()) {
        gpsAlt = gps.altitude.meters();
      }
    }
  }

  // --- non-blocking-code ---
  unsigned long nbcCurrentMillis = millis(); // get actual timestamp
  if (nbcCurrentMillis - nbcPreviousMillis >= nbcInterval) {
    nbcPreviousMillis = nbcCurrentMillis;

    Serial.println("- - - - -");
    Serial.println("Uptime (ms): " + String(millis()));

    // DS18B20
    if (sensorCount == 0) {
      Serial.println("ds18b20 not found.");
      client.publish("/climate/outdoor/temperature", "error");
    } else {
      sensors.requestTemperatures(); // Send the command to get temperature readings 
      float outTemp = sensors.getTempCByIndex(0); // read first sensor for outdoor temperature
      
      snprintf (msg, 50, "%.2f", outTemp);
      client.publish("/climate/outdoor/temperature", msg);
      Serial.println("/climate/outdoor/temperature: " + String(outTemp) + " (Celsius)");
    }

    // BME280
    bme.readSensor();
    float bTemp = bme.getTemperature_C(); // Temperature in Celsius
    float bHumi = bme.getHumidity(); // Humidity in Precent
    float bHp = bme.getPressure_HP(); // pressure in hectapascals
    float bMb = bme.getPressure_MB(); // pressure in millibars

    snprintf (msg, 50, "%.2f", bTemp);
    client.publish("/climate/indoor/floor/temperature", msg);
    snprintf (msg, 50, "%.2f", bHumi);
    client.publish("/climate/indoor/floor/humidity", msg);
    snprintf (msg, 50, "%.2f", bHp);
    client.publish("/climate/indoor/floor/pressure-hp", msg);
    snprintf (msg, 50, "%.2f", bMb);
    client.publish("/climate/indoor/floor/pressure-mb", msg);

    Serial.println("/climate/indoor/floor/temperature: " + String(bTemp) + " (Celsius)");
    Serial.println("/climate/indoor/floor/humidity: " + String(bHumi) + " (%)");
    Serial.println("/climate/indoor/floor/pressure-hp: " + String(bHp) + " (hectapascals)");
    Serial.println("/climate/indoor/floor/pressure-mb: " + String(bMb) + " (millibars)");

    // GPS
    if (gpsLat == 0.0) {
      client.publish("/position/latitude", "error");
    } else {
      snprintf (msg, 50, "%.2f", gpsLat);
      client.publish("/position/latitude", msg);
    }
    if (gpsLon == 0.0) {
      client.publish("/position/latitude", "error");
    } else {
      snprintf (msg, 50, "%.2f", gpsLon);
      client.publish("/position/longitude", msg);
    }
    if (gpsSat == 0.0) {
      client.publish("/position/latitude", "error");
    } else {
      snprintf (msg, 50, "%.0f", gpsSat);
      client.publish("/position/satellites", msg);
    }
    if (gpsAlt == 0.0) {
      client.publish("/position/latitude", "error");
    } else {
      snprintf (msg, 50, "%.2f", gpsAlt);
      client.publish("/position/altitude", msg);
    }
    
    Serial.println("/position/latitude: " + String(gpsLat) + " (N, Breitengrad)");
    Serial.println("/position/longitude: " + String(gpsLon) + " (E, Längengrad)");
    Serial.println("/position/satellites: " + String(gpsSat) + " (number)");
    Serial.println("/position/altitude: " + String(gpsAlt) + " (meters)");
  }
}

/*
 * Appendix - sources for tutorials:
 * 
 * MQTT publish/subscribe: https://smarthome-blogger.de/tutorial/esp8266-mqtt-tutorial/ (german)
 * MQTT PubSubClient-API: // https://pubsubclient.knolleary.net/api (english)
 * DHT22: https://funduino.de/anleitung-dht11-dht22 (german)
 */
