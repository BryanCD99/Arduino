#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
#include <UbidotsESPMQTT.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

// Definiciones del programa **********
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Definicion para la comunición IOT
#define UBIDOTS_TOKEN "BBFF-SsPDCEdunSJVxwWZxfvYZsDatRB4Wi"    // Put here your Ubidots TOKEN
#define WIFI_SSID "JESSENIA"   // Put here your Wi-Fi SSID
#define WIFI_PASS "rosita2000@"        // Put here your Wi-Fi password
#define DEVICE_LABEL  "ESP8266_Vital_Monitor"  // Put here your Ubidots device label

// Constantes del programa ************
int const D0 = 16;
int const D3 = 0;
int const D5 = 14;
int const D6 = 12;
int const D7 = 13;    // VALVE
int const D8 = 15;     // PUMP

// Constantes de tiempo de ejecución del programa
const unsigned long Sampling = 10;          // Tiempo de ejecución del filtro pasa alto
unsigned long const Mode1Time = 1000;
unsigned long const Mode2Time = 2000;
unsigned long const Mode3Time = 4000;
unsigned long const SendData = 5000;

// Variables de control de ejecución *******************
unsigned long tsLastReport = 0;           // Control del tiempo de ejecución del monitor cardiaco
unsigned long TemperatureMeasureTime = 0; // Control del tiempo de ejecución del monitor de temperatura
unsigned long PresureTimeWait = 0;        // Espera antes de medir la presión
unsigned long MeasurePresureTime = 0;
unsigned long SendDataTime = 0;           // Intervalo de envio de datos a la plataforma Ubidots

// Constantes para la medición de la presión
const float ADC_mV = 3.3 / 1024;      // convesion multiplier from ESP ADC value to voltage in mV
const float SensorOffset = 0.18;     // in V measure from sensor
const float Vs = 5;                   // Voltage source
const float Kpa2mmHg = 7.5006;    // convesion multiplier from mmH2O to mmHg

// Variables para el filtro pasa alto ******
const float RC = 0.32985;
const float K = 1.15;
float RealPresure = 0;
float LastPresure = 0;
float HighOutput = 0;
float Output = 0;
float LastOutput = 0;

// Filtro EMA ********************
float LastAn = 0;
const float a = 0.15;

// Variables de uso general ***********
int SelectMode = 0;         // Inicia en modo Oximetro
float sensorValue = 0;
float SIS = 0;
float DIA = 0;
float HeartRate, SpO2;
double temp_amb;       // Variables para el modo de medición de temperatura **********
double temp_obj;
bool FlagSwitch1 = false;
bool FlagSwitch2 = false;
bool FlagSwitch3 = false;
bool UseMode1 = true;
bool UseMode2 = false;
bool UseMode3 = false;
bool Sendonce3 = false;
bool WriteBipMap = false;
bool ResetPump = false;
bool MeasurePresure = false;
bool StopMeasure = false;


// Variables y objetos para el modo monitor cardiaco *************
// Bitmap logo corazon *******
const unsigned char heartBitmap [] PROGMEM =
{
  0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x18, 0x00, 0x0f, 0xe0, 0x7f, 0x00, 0x3f, 0xf9, 0xff, 0xc0,
  0x7f, 0xf9, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xf0,
  0xff, 0xf7, 0xff, 0xf0, 0xff, 0xe7, 0xff, 0xf0, 0xff, 0xe7, 0xff, 0xf0, 0x7f, 0xdb, 0xff, 0xe0,
  0x7f, 0x9b, 0xff, 0xe0, 0x00, 0x3b, 0xc0, 0x00, 0x3f, 0xf9, 0x9f, 0xc0, 0x3f, 0xfd, 0xbf, 0xc0,
  0x1f, 0xfd, 0xbf, 0x80, 0x0f, 0xfd, 0x7f, 0x00, 0x07, 0xfe, 0x7e, 0x00, 0x03, 0xfe, 0xfc, 0x00,
  0x01, 0xff, 0xf8, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x7f, 0xe0, 0x00, 0x00, 0x3f, 0xc0, 0x00,
  0x00, 0x0f, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// Objeto pulsometro oximetro ************
PulseOximeter pox;

// Objeto de medición de temperatura *****
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Configuración de la pantalla OLED ****
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Objeto para la comunición con la plataforma Ubidots **************
Ubidots client(UBIDOTS_TOKEN);
// Objeto para responder a la plataforma **************
void callback(char* topic, byte* payload, unsigned int length) {
}

// Controlador de los pulsadores *******************
void SwitchHandler()
{
  // Cambio al modo de oximetro y frecuencia cardiaca ***********
  if ((digitalRead(D3) == LOW) && (UseMode1 == false))
  {
    if (FlagSwitch1 == false)
    {
      // Apaga la valvula y el motor del medidor de presión. Puede usarse como paro de emergencia
      digitalWrite(D7, LOW);
      digitalWrite(D8, LOW);
      // Realiza la configuración para el inicio del modo del pulsometro/Oximetro
      Mode1Setup();
      SelectMode = 0;
      // Bandera que indica que el pulsador esta presionado. Evita multiples ejecuciones de esta sección
      FlagSwitch1 = true;
      // Banderas para indicar que esta seleccionado el modo 1. Evita que se reinicie el modo si se vuelve a presionar el pulsador
      UseMode1 = true;
      UseMode2 = false;
      UseMode3 = false;
    }
  } else
  {
    FlagSwitch1 = false;
  }

  if ((digitalRead(D5) == LOW) && (UseMode2 == false))
  {
    if (FlagSwitch2 == false)
    {
      // Apaga la valvula y el motor del medidor de presión. Puede usarse como paro de emergencia
      digitalWrite(D7, LOW);
      digitalWrite(D8, LOW);
      // Realiza la configuracion del modo sensor de temperatura
      Mode2Setup();
      SelectMode = 1;
      // Bandera que indica que el pulsador esta presionado. Evita multiples ejecuciones de esta sección
      FlagSwitch2 = true;
      UseMode1 = false;
      UseMode2 = true;
      UseMode3 = false;

    }
  } else
  {
    FlagSwitch2 = false;
  }

  if ((digitalRead(D6) == LOW) && (UseMode3 == false))
  {
    if (FlagSwitch3 == false)
    {
      // Realiza la configuracion del modo medidor de presión
      Mode3Setup();
      SelectMode = 2;
      // Bandera que indica que el pulsador esta presionado. Evita multiples ejecuciones de esta sección
      FlagSwitch3 = true;
      UseMode1 = false;
      UseMode2 = false;
      UseMode3 = true;
    }
  } else
  {
    FlagSwitch3 = false;
  }
}
//Funciones del Modo de medicion del ritmo cardiaco **************
void onBeatDetected()
{
  WriteBipMap = true;
}
void Mode1Setup()
{
  // Mensaje de inicio del modo pulsometro-oximetro ******************************
  Serial.println("Initializing pulse oximeter.."); // Send a ACK on monitor serial
  // Clear the buffer *******
  display.clearDisplay();
  // Configuración de color y tamaño de texto *****
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 0);
  display.println("Initializing pulse oximeter..");
  display.display();
  delay(1000);
}
void Mode1()
{
  pox.update();
  if (millis() - tsLastReport > Mode1Time) {
    HeartRate = pox.getHeartRate();
    SpO2 = pox.getSpO2();
    Serial.print("PUL Y OXI:");
    Serial.print(HeartRate);
    Serial.print("-----");
    Serial.print("Oxygen Percent:");
    Serial.print(SpO2);
    Serial.println("\n");

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 16);
    display.println(HeartRate);
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 0);
    display.println("PUL Y OXI");
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 30);
    display.println("Spo2");
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 45);
    display.println(SpO2);
    if (WriteBipMap)
    {
      Serial.println("Beat!");
      display.drawBitmap( 60, 20, heartBitmap, 28, 28, 1);
      display.display();
      //delay(100);
      display.fillRect ( 60, 20, 28, 28, BLACK);
      display.display();
      WriteBipMap = false;
    } else
    {
      display.display();
    }
    tsLastReport = millis();
  }
}
// Modo de medicion de temperatura **************
void Mode2Setup()
{
  Serial.println("Starting Temperature Sensor MLX90614...");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(25, 15);
  display.println(" Universidad");
  display.setCursor(10, 35);
  display.println("tecnica del norte");
  display.display();
  delay(1000);
}
void Mode2()
{
  if (millis() - TemperatureMeasureTime > Mode2Time) {
    temp_amb = mlx.readAmbientTempC();
    temp_obj = mlx.readObjectTempC();
    // Ecuación de corrección lineal ***
    temp_obj = 1.2122 * temp_obj - 3.8337;
    // Limites para mostrar la temperatura de un objeto solo dentro del rango
    if ((temp_obj < 25) || (temp_obj > 50))
    {
      temp_obj = 0;
    }

    //Serial Monitor
    Serial.print("Room Temp = ");
    Serial.println(temp_amb);
    Serial.print("Object temp = ");
    Serial.println(temp_obj);

    display.clearDisplay();
    display.setCursor(25, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println(" Temperatura");

    display.setCursor(10, 20);
    display.setTextSize(1);
    display.print("Ambient: ");
    display.print(temp_amb);
    display.print((char)247);
    display.print("C");

    display.setCursor(10, 40);
    display.setTextSize(1);
    display.print("Object: ");
    display.print(temp_obj);
    display.print((char)247);
    display.print("C");

    display.display();
    TemperatureMeasureTime = millis();
  }
}
// Modo de medicion de la presión **************
void Mode3Setup()
{
  Serial.println("Medición de presión");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Medicion de presion");
  display.display();
  delay(1000);

  // Activa la valvula
  digitalWrite(D7, HIGH);
  // Activa la bomba
  digitalWrite(D8, HIGH);
  ResetPump = false;
  RealPresure = 0;
  MeasurePresure = false;
  StopMeasure = false;
  LastPresure = 0;
  LastAn = 0;
  // Bandera para realizar un solo envío a Ubidots de la presión medida
  Sendonce3 = false;
  MeasurePresureTime = millis();
}
void Mode3()
{
  // Función de pre-procesamiento de la señal digital muestreada del sensor ********
  if (millis() - MeasurePresureTime > Sampling)
  {
    // Lectura del sensor de presión e mmHg
    sensorValue = ((20 / 9) * (((analogRead(A0) * ADC_mV - SensorOffset) * 25) / Vs) - 1) * Kpa2mmHg;
    // Filtro EMA para la eliminación de ruido
    sensorValue = a * sensorValue + (1 - a) * LastAn;
    LastAn = sensorValue;
    // Filtro pasa alto para la determinación de la señal pulsante
    Output =  (K * (sensorValue - LastPresure) + LastOutput) / (1 + (0.01 / RC));
    LastOutput = Output;
    LastPresure = sensorValue;

    MeasurePresureTime = millis();
  }

  // Cuando se alcanza o supera la presión de 180 mmHg procede al apagado de la bomba y se realiza las mediciones
  if (sensorValue >= 180)
  {
    MeasurePresure = true;
  }
  // Metodo de medición ****************
  if (MeasurePresure == true) {
    if (ResetPump == false) // Apaga la bomba y reestablece el contador de espera para estabilizar la señal pulsante
    {
      digitalWrite(D8, LOW);
      ResetPump = true;
      HighOutput = -100;  // Valor inicial para asegurar la detección de la mayor amplitud de la señal pulsante
      PresureTimeWait = millis();
    } else
    {
      if (millis() - PresureTimeWait > Mode3Time) {
        if (sensorValue <= 90)  // Si la presion decae por debajo de 90 mmHg entonces se libera la valvula y el proceso termina
        {
          StopMeasure = true;
        }
        if (StopMeasure == true) // Cuando el proceso de medición termina se calcula la presion sistolica y diastolica y se muestra en el display
        {
          SIS = RealPresure * 0.55 + RealPresure / 1.3;
          DIA = RealPresure * 0.85;
          digitalWrite(D7, LOW);
          UseMode3 = false;
          display.clearDisplay();
          display.setCursor(0, 0);
          display.print("SIS: ");
          display.print(SIS);
          display.println(" mmHg");
          display.print("DIA: ");
          display.print(DIA);
          display.println(" mmHg");
          display.println("Pressure sensing");
          display.println("complete Complete");
          display.display();

        } else {
          // Si la medición esta en progreso, se determina el mayor valor de salida del filtro pasa alta
          // el cual determinara la presión media del paciente
          if (HighOutput < Output)
          {
            HighOutput = Output;         // Variable para almacenar la mayor amplitud actual del filtro (se actualiza si se detecta un mayor valor)
            RealPresure = sensorValue;   // Variable que almacena el valor de la presion correspondiente a la mayor amplitud del filtro pasa alta
          }

          // Se muestra en el display el valor de la presión actual
          display.clearDisplay();
          display.setCursor(0, 0);
          display.print("Pressure: ");
          display.print(sensorValue);
          display.println("mmHg");
          display.display();

          Serial.println(Output, 2);
        }
      }
    }
  }
}
// Variables a enviar a UBIDOTS *****************************
void SendMode1Data()
{
  client.add("bpm", HeartRate);
  client.add("oxigeno", SpO2);
  client.ubidotsPublish(DEVICE_LABEL);
}
void SendMode2Data()
{
  client.add("temp_ambiente", temp_amb);
  client.add("temp_objeto", temp_obj);
  client.ubidotsPublish(DEVICE_LABEL);
}
void SendMode3Data()
{
  client.add("P_SIS", SIS);
  client.add("P_DIA", DIA);
  client.ubidotsPublish(DEVICE_LABEL);
}
// Configuración general del programa ******************
void setup() {
  // Configuración de entradas y salidas
  pinMode(D3, INPUT);  // Pinout del pulsador 1
  pinMode(D5, INPUT);  // Pinout del pulsador 2
  pinMode(D6, INPUT);  // Pinout del pulsador 3
  pinMode(D7, OUTPUT); // Pin para la valvula
  pinMode(D8, OUTPUT); // Pin para la bomba

  // Estado inicial de las salidas
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  // Configuración puerto serial ***********************
  Serial.begin(115200);

  // Configuacion e inicializacion de la pantalla OLED ************************
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {   // Inicia el puerto I2C para la pantalla OLED
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever                    // En caso de fallo, se indica en el monitor serial y se congela el programa
  }
  display.display();                                            // Carga pantalla de inicio Adafruit
  delay(1000); // Pause for 1 second

  // Clear the display **********
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 0);
  // Visualización de estado de conexión **********
  display.print("Conectando... ");
  display.display();
  // Conexion con ubidots
  client.ubidotsSetBroker("industrial.api.ubidots.com"); // Sets the broker properly for the business account
  client.wifiConnection(WIFI_SSID, WIFI_PASS);
  client.begin(callback);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // Muestra dirección IP *****************
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Conectado con: ");
  display.println(WiFi.localIP());
  display.display();
  delay(1000); // Pause for 1 second
  display.clearDisplay();
  // Inicialización de los dispositivos de medición I2C *****************
  // Inicialización del pulsometro-oximetro ***************************************
  if (!pox.begin()) {
    Serial.println("FAILED MAX30100");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 0);
    display.println("FAILED MAX30100");
    display.display();
    for (;;);
  }
  Serial.println("SUCCESS MAX30100 STARTING");
  // Codigo para la detección del beat
  pox.setOnBeatDetectedCallback(onBeatDetected);
  //Initialize  sensor de temperatura  ************************************
  mlx.begin();
  Serial.println("SUCCESS MLX90614 STARTING");
  // Inicio modo 1
  Mode1Setup(); // Inicializa solo el modo uno
}

void loop() {
  if (!client.connected()) {
    client.reconnect();
  }
  // Controlador de los pulsadores **********************
  SwitchHandler();
  // Seleccion de modo de funcionamiento ****************
  switch (SelectMode)
  {
    case 0:
      Mode1();
      break;
    case 1:
      Mode2();
      break;
    case 2:
      Mode3();
      break;
  }
  // Envio de datos a ubidots ***********************
  if (millis() - SendDataTime > SendData) {
    switch (SelectMode)
    {
      case 0:
        Serial.println("Enviando datos del pulsometro");
        SendMode1Data();
        pox.begin();
        break;
      case 1:
        Serial.println("Enviando datos de temperatura");
        SendMode2Data();
        mlx.readAmbientTempC();
        mlx.readObjectTempC();
        break;
      case 2:
        if (StopMeasure == true)
        {
          if (!Sendonce3)
          {
            Serial.println("Enviando datos de presión");
            SendMode3Data();
            Sendonce3 = true;
          }
        }
        break;
    }
    SendDataTime = millis();
  }
  client.loop();
}
