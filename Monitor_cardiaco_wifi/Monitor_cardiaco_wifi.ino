#include <UbidotsESPMQTT.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
/****************************************
   Define Constants
 ****************************************/
#define TOKEN "BBFF-futcmgmyVLvU4ZR4jEpRQox0KecPZc" // Your Ubidots TOKEN
#define WIFINAME "JESSENIA" //Your SSID
#define WIFIPASS "rosita2000@" // Your Wifi Pass
#define DEVICE_LABEL  "ESP8266_Vital_Monitor"  // Put here your Ubidots device label
#include "MAX30100_PulseOximeter.h"
#define REPORTING_PERIOD_MS     1000

Ubidots client(TOKEN);

/****************************************
   Auxiliar Functions
 ****************************************/
uint32_t tsLastReport = 0;
void callback(char* topic, byte* payload, unsigned int length) {
}

// cast from an array of chars to float value.

/****************************************
   Main Functions
 ****************************************/
// Objeto pulsometro oximetro ************
PulseOximeter pox;

void setup() {
  // put your setup code here, to run once:
  client.ubidotsSetBroker("industrial.api.ubidots.com"); // Sets the broker properly for the business account
  //client.setDebug(true); // Pass a true or false bool value to activate debug messages
  Serial.begin(115200);
  client.wifiConnection(WIFINAME, WIFIPASS);
  client.begin(callback);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  if (!pox.begin()) {
    Serial.println("FAILED MAX30100");
    for (;;);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    client.reconnect();
  }
  // Make sure to call update as fast as possible
  pox.update();
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    // to computer Serial Monitor
    Serial.print("BPM: ");
    Serial.print(pox.getHeartRate());
    //blue.println("\n");
    Serial.print("    SpO2: ");
    Serial.print(pox.getSpO2());
    Serial.print("%");
    Serial.println("\n");


    client.add("bpm", pox.getHeartRate());
    client.ubidotsPublish(DEVICE_LABEL);
    client.loop();
    tsLastReport = millis();
  }
}
