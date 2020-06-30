/*
 * EN: ESP-Temperature- and Humidity-Module
 * DE: ESP-Temperatur- und Luftfeuchtigkeits-Modul
 * 
 * Hardware: 
 * - ESP-12F (ESP8266, "D1 Mini")
 * - DHT22 (or DHT11)
 * - DS18B20
 * 
 * Software:
 * - wifi
 * - mqtt (pubsubclient)
 * 
 * Author: René Brixel <mail@campingtech.de>
 * Version: 1.0
 * Date: 2020-06-30
 */

// WIFI
#include <ESP8266WiFi.h>
#ifndef STASSID
#define STASSID "wlan-name" // your wifi-name
#define STAPSK  "wlan-passwort" // your wifi-password
#endif
const char* host = "nodetemphumi01"; // hostname of module
const char* ssid     = STASSID;
const char* password = STAPSK;

// MQTT-Client
#include <PubSubClient.h>
const char* MQTT_BROKER = "BROKER_IP"; // ip-address of your mqtt-broker
WiFiClient espClient;
PubSubClient client(espClient);
//long lastMsg = 0;
char msg[50];
//int value = 0;

// OTA-Update
/*
#include <ESP8266WebServer.h>
ESP8266WebServer server(80);
const char* serverIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
*/

// DHT
#include <DHT.h>
#define DHTPIN 2  
#define DHTTYPE DHT22 // DHT11, DHT22
DHT dht(DHTPIN, DHTTYPE);

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 3  //Sensor DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int sensorCount;

// Interval-values (for non blocking code)
unsigned long nbcPreviousMillis = 0; // holds last timestamp
const long nbcInterval = 2000; // interval in milliseconds (1000 milliseconds = 1 second)

void setup() {
  // Start serial-connection
  Serial.begin(115200);

  // Start DHT-sensor
  dht.begin();

  // Start DS18B20-sensor
  sensors.begin();
  sensorCount = sensors.getDS18Count();

  // Start wifi and connect to access point
  setup_wifi();

  // Start MQTT
  client.setServer(MQTT_BROKER, 1883); // ip-address, port 1883
  //client.setCallback(callback); // only needed for subscription of topic
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
// MQTT-Reconnect (for subscribe):
void reconnect() {
  while (!client.connected()) {
    Serial.println("Reconnecting MQTT...");
    if (!client.connect("ESP8266Client")) {
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
    if (!client.connect("ESP8266Client")) {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
  Serial.println("MQTT Connected!");
}

void loop() {
  // Non-blocking-Code; only executed when interval is reached
  unsigned long nbcCurrentMillis = millis(); // get actual timestamp
  if (nbcCurrentMillis - nbcPreviousMillis >= nbcInterval) {
    nbcPreviousMillis = nbcCurrentMillis;

    Serial.println("- - - - -");
    Serial.println("Publishing sensor-data!");

    // Check, if ds18b20 is reachable
    if (sensorCount == 0) {
      Serial.println("ds18b20 not found.");
      client.publish("/climate/outdoor/temperature", "error");
    } else {
      float outTemp = sensors.getTempCByIndex(0); // read outdoor temperature
      
      snprintf (msg, 50, "%.2f", outTemp);
      client.publish("/climate/outdoor/temperature", msg);
      Serial.println("/climate/outdoor/temperature: " + String(outTemp));
    }

    float inHumi  = dht.readHumidity(); // read indoor humidity
    float inTemp  = dht.readTemperature(); // read indoor temperature

    snprintf (msg, 50, "%.2f", inTemp);
    client.publish("/climate/indoor/floor/temperature", msg);
    snprintf (msg, 50, "%.2f", inHumi);
    client.publish("/climate/indoor/floor/humidity", msg);

    Serial.println("/climate/indoor/floor/temperature: " + String(inTemp));
    Serial.println("/climate/indoor/floor/humidity: " + String(inHumi));
    Serial.println("- - - - -");
  }

  // --- realtime code starts here ---
  
  // MQTT:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

/*
 * Appendix - sources for tutorials:
 * 
 * MQTT publish/subscribe: https://smarthome-blogger.de/tutorial/esp8266-mqtt-tutorial/ (german)
 * DHT22: https://funduino.de/anleitung-dht11-dht22 (german)
 */
