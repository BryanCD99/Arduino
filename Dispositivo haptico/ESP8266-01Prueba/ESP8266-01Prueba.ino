#define adress 10
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

const char* ssid = "JESSENIA";
const char* password = "rosita2000@";
// Host y port para la comunicación con el socket del ESP8266 receptor ***
const char* host = "192.168.2.119";
const uint16_t port = 5050;
//Contador par saber que se est comunicando con el modulo wifi
int count = 0;
// Wifi Client class *****
WiFiClient client;
void setup() {
  // put your setup code here, to run once:
  ///pinMode(2, OUTPUT);
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  if (WiFi.status() == WL_CONNECTED) {
    SendWIFIConfirm = true; // Si se logra conectar a la red, se indica al sistema que hay conexion y que debe conectarse al servidor
  }
}
void SerialRequest()
{
  if (Serial.available() > 0)
  {
    char data = Serial.read();
    switch (data)
    {
      case 'S':
        Serial.print(count); // Read sensor data ***************
        count++;
        break;
      case 'R':
        count = 0;
        Serial.print(count); // Read sensor data ***************
        break;
    }
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  SerialRequest();
}
