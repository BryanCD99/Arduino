#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// Definiciones del programa
#define WIFIComunicationTime 10 // Refresco para el envio de datos
#define Refresh 20              // Refresco del puerto serial
#define WIFITime 200            // Intervalo de verificación para el control de la conexion WIFI


//access point network credentials
const char* ssid = "DH_01";
const char* password = "123456789";
//const char* ssid = "JessyNetwork";
//const char* password = "JN1234567";

// Host y port para la comunicación con el socket del ESP8266 receptor ***
const char* host = "192.168.137.1";
const uint16_t port = 5050;

// Variables de control de ejecución **********
unsigned long CurrentTime = 0;
unsigned long SerialTime = 0;
unsigned long WIFIControlTime = 0;
unsigned long WIFIDataTime = 0;

// Variables de uso general *******************
bool WIFIConnect = false;
bool TryConnect = true;
bool SendWIFIConfirm = false;
bool SystemStarted = false;

// Variables Pitch, Roll and Yaw para la obtención de angulos de inclinación
float pitch = 0;    // Incremento en Y
float roll = 0;     // Incremento en X
float yaw = 0;      // Incremento en Z
float BatteryVoltage = 0;
bool ChargerConnect = false;

typedef union {
  float val;
  uint8_t bytes[4];
} floatval;
floatval Bytes2Float;

//// Wifi Client class *****
//WiFiClient client;

void setup() {
  // put your setup code here, to run once:
  // Inicializacion del puerto serial
  Serial.begin(9600);
  // Modo de estacion ************
  WiFi.mode(WIFI_OFF);

  pinMode(LED_BUILTIN, OUTPUT);

}
void SerialHandler() {
  char data;
  if (Serial.available() > 0) {
    data = Serial.read();
    switch (data) {
      case 'C': // Solicitud de conectarse con el receptor
        digitalWrite(LED_BUILTIN, LOW);
        if (WiFi.status() == WL_DISCONNECTED) { // Si el WIFI esta apagado, lo enciende
          //Serial.println("Turn on");
          WiFi.mode(WIFI_STA);
        }
        SendWIFIConfirm = true;
        ConnectWIFI();
        break;
      case 'S':
        void ReadBytes();
        break;
      case 'O': // Apaga el WIFI
        SendWIFIConfirm = false;
        WiFi.mode(WIFI_OFF);
        WIFIConnect = false;
        TryConnect = true;
        break;
      case 'Y':
        ChargerConnect = true;
        break;
      case 'N':
        ChargerConnect = false;
        break;
    }
  }
}
void ReadBytes() {
  for (int i = 3; i >= 0; i--)
    Bytes2Float.bytes[i] = Serial.read();
  pitch = Bytes2Float.val;
  for (int i = 3; i >= 0; i--)
    Bytes2Float.bytes[i] = Serial.read();
  roll = Bytes2Float.val;
  for (int i = 3; i >= 0; i--)
    Bytes2Float.bytes[i] = Serial.read();
  yaw = Bytes2Float.val;
  for (int i = 3; i >= 0; i--)
    Bytes2Float.bytes[i] = Serial.read();
  BatteryVoltage = Bytes2Float.val;
}
void ConnectWIFI() {
  if (WiFi.status() != WL_CONNECTED) {  // En caso de que este conectado, no se vuelve a conectar
    if ((WiFi.status() == WL_CONNECT_FAILED) && (TryConnect == false)) {
      TryConnect = true;  // En caso de que haya fallado el intento de conexión, lo vuelve a intentar
      //Serial.println("reConectando");
    }
    if (TryConnect == true) { // Intenta conectar una vez hasta que haya transcurrido el numero de intentos de conección de la función
      TryConnect = false;
      // Serial.println("Conectando");
      WiFi.begin(ssid, password); // Intenta iniciar la conexión WIFI
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    SendWIFIConfirm = true; // Si se logra conectar a la red, se indica al sistema que hay conexion y que debe conectarse al servidor
  }
}
void WIFIConnectionHandler() {
  // Si se ha conectado correctamente se envia confirmación al ARDUINO PRO MINI
  if ((SendWIFIConfirm == true) && (WiFi.status() == WL_CONNECTED)) {
    if (client.connect(host, port)) { // Intenta realizar la conexión con el servidor por medio de web socket
      Serial.print('C');              // Si lo conseigue, se envia confirmación al arduino Pro MINI
      SendWIFIConfirm = false;
    }
    SystemStarted = true;
  }
  // Si la conexión falla se vuelve a conectar ******************************
  if ((SystemStarted == true) && (WiFi.status() == WL_CONNECT_FAILED)) {
    ConnectWIFI();
  }
}
void WIFIClientHandler() {
  WiFiClient client;
  if (client.available()) {
    char c = client.read();
    switch (c) {
      case 'S':
        client.println(pitch);
        client.println(roll);
        client.println(yaw);
        client.println(BatteryVoltage);
        break;
      case 'M':
        Serial.print('M');
        client.print("Motor ON!");
        break;
    }
  }
}
void loop() {
  // Main **************
  digitalWrite(LED_BUILTIN, HIGH);
  CurrentTime = millis();
  // Controlador de comunicación serial **************
  if ((CurrentTime - SerialTime) > Refresh) {
    SerialHandler();
    SerialTime = CurrentTime;
  }
  // Controlador de conexion WIFI **************
  if ((CurrentTime - WIFIControlTime) > WIFITime) {
    WIFIConnectionHandler();
    WIFIControlTime = CurrentTime;
  }
  // Controlador de flujo de datos en red ******
  if ((CurrentTime - WIFIDataTime) > WIFIComunicationTime) {
    WIFIClientHandler();
    WIFIDataTime = CurrentTime;
  }
}
