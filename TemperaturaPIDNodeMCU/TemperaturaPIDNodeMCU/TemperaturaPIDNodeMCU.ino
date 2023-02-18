
#include <OneWire.h>
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <DallasTemperature.h>

// Definiciones ************************************
#define FIREBASE_HOST "pruebaconexiones-cfe46-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "YKIazTPMaBLaOTkoQUQdxMCTEkAce4r4pYUgM4kY"

#define WIFI_SSID "CNT-SHERMAN"
#define WIFI_PASSWORD "@sus2020"

// Variables de uso general *************************
const int SensorPin = 2;
const int Fan = 13;
const int Light = 15;
const int Led_red = 12;
const int Led_blue = 16;
int AnalogData = 0;
int DataPWM = 0;
float Sensor = 0;
float voltageConsigna = 0;


unsigned char *Set_point_Data = (unsigned char *)&voltageConsigna;
unsigned char *DS18B20Data = (unsigned char *)&Sensor;
unsigned char data = ' ';

// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(SensorPin);
DallasTemperature sensorDS18B20(&oneWireObjeto);

void setup() {
  // put your setup code here, to run once:
  // Inicializa comunicación Serial *******************************************
  Serial.begin(115200);
  // Inicializar pines ********************************************************
  pinMode(SensorPin, INPUT);
  pinMode(Fan, OUTPUT);
  pinMode(Light, OUTPUT);
  pinMode(Led_red, OUTPUT);
  pinMode(Led_blue, OUTPUT);

  // Inicializar salidas ******************************************************
  digitalWrite(Fan, LOW);
  digitalWrite(Light, LOW);
  digitalWrite(Led_red, LOW);
  digitalWrite(Led_blue, LOW);

  // frecuencia PWM para el control del motor ************
  analogWriteFreq(200); //200Hz
  analogWriteRange(255);// Maximo 255

  // Set WIFI Settings ****************************
  // Cliente WiFi
  WiFiClientSecure client;
  // Conexion con la red *********
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }
  // Start Firebase conection *********************
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}
void PWM_Fan()
{
  while (Serial.available() == 0)
  {
    data = ' ';
  }
  data = Serial.read();
  digitalWrite(Led_red, LOW);
  digitalWrite(Led_blue, HIGH);
  analogWrite(Fan, (int)data);
  analogWrite(Light, 0);
}
void PWM_Light()
{
  while (Serial.available() == 0)
  {
    data = ' ';
  }
  data = Serial.read();
  digitalWrite(Led_blue, LOW);
  digitalWrite(Led_red, HIGH);
  analogWrite(Light, (int)data);
  analogWrite(Fan, 0);
}
void SenToNube()
{
  String DataToSend;
  DataToSend = String(Sensor) + " °C";
  Firebase.setString("Feedback", DataToSend);
  DataToSend = String(voltageConsigna) + " V";
  Firebase.setString("Consigna", DataToSend);
}
void LeerPotenciometro()
{
  //Leer consigna ****************
  AnalogData = analogRead(A0); //Lectura del ADC
  voltageConsigna = AnalogData * (3.3 / 1023.0); //escalar voltaje
  voltageConsigna = round(voltageConsigna * 100) / 100; // Redondear a 2 cifras
}
void LeerTemperatura()
{
  sensorDS18B20.requestTemperatures();
  Sensor = sensorDS18B20.getTempCByIndex(0);
  Sensor = round(Sensor * 100) / 100; // Redondear a 2 cifras

}
void SendData()
{
  for (int i = 3; i >= 0; i--)
    Serial.write(Set_point_Data[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(DS18B20Data[i]);
}
void SerialRequest()
{
  if (Serial.available() > 0)
  {
    data = Serial.read();
    switch (data)
    {
      case 'S':
        Serial.print('s'); // Read sensor data ***************
        SendData();
        break;
      case 'L':
        Serial.print('l'); // Write PWM to light *******************
        PWM_Light();
        break;
      case 'F':
        Serial.print('f'); // Write PWM to Fan *******************
        PWM_Fan();
        break;
      case 'W':
        SenToNube();
        Serial.print('w'); // Subir dato wifi *******************
        break;
      default:
        Serial.print('E'); // In case of error send E ********
        break;

    }
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  LeerPotenciometro();
  LeerTemperatura();
  SerialRequest();

}
