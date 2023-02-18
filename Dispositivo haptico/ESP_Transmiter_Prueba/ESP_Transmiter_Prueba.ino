#include <ESP8266WiFi.h>
//#include <WiFiClient.h>

// Definiciones del programa
#define WIFIComunicationTime 1 // Refresco para el envio de datos
#define Refresh 5              // Refresco del puerto serial
#define WIFITime 500            // Intervalo de verificación para el control de la conexion WIFI
#define ESPSerialTimeOUT 1000   // Si no se reciben datos por el puerto serial dentro de 1 segundo, automaticamente se cierra la conexion con el servidor y se limpia el buffer serial

//access point network credentials
const char* ssid = "DH_01";
const char* password = "123456789";

// Host y port para la comunicación con el socket del ESP8266 receptor ***
const char* host = "192.168.2.220";
const uint16_t port = 8080;

// Variables de control de ejecución **********
unsigned long CurrentTime = 0;
unsigned long SerialTime = 0;
unsigned long WIFIControllerTime = 0;
unsigned long WIFIDataTime = 0;
unsigned long SerialTimeout = 0;


// Variables de uso general *******************
bool WIFIConnect = false;
bool TryConnect = false;
int ClientMode = 0; // 0 = enviar datos, 1 = pedir datos
bool ServeConnect = false;
bool WaitForDataBytes = false;
bool SerialTimeoutOnce = false;

// Variables Pitch, Roll and Yaw para la obtención de angulos de inclinación
float pitch = 0;    // Incremento en Y
float roll = 0;     // Incremento en X
float yaw = 0;      // Incremento en Z
float BatteryVoltage = 0;
bool ReadMotor = false;
bool MotorStateBuffer = false;
bool ChargerConnect = false;

// Variables para la comunicación con el ESP *****
unsigned char *pitchData = (unsigned char *)&pitch;
unsigned char *rollData = (unsigned char *)&roll;
unsigned char *yawData = (unsigned char *)&yaw;
unsigned char *BatteryVoltageData = (unsigned char *)&BatteryVoltage;

void WIFIClientHandler() {
  WiFiClient client;
  if (client.connect(host, port)) {
    //client.setNoDelay(true);
    //Serial.println("Conectado!");
    ClientMode = 0;
    digitalWrite(LED_BUILTIN, LOW);
    while (client.connected()) {
      ServeConnect = true; // Server conectado (Conexión exitosa a la RED)
      if ((WiFi.status() == WL_DISCONNECTED)) {
        //ClientMode = 1;
        client.stop();
        break;// Rompe el ciclo ************
      }
      if (ClientMode == 0) {
        // Envia los datos de la memoria ***************
        client.print('R');
        for (int i = 0; i <= 3; i++)
          client.write(pitchData[i]);
        for (int i = 0; i <= 3; i++)
          client.write(rollData[i]);
        for (int i = 0; i <= 3; i++)
          client.write(yawData[i]);
        for (int i = 0; i <= 3; i++)
          client.write(BatteryVoltageData[i]);
        // Envia el estado actual del cargador **********
        if (ChargerConnect) {
          client.print('Y');
        } else {
          client.print('N');
        }
        // Envia solicitud de estado para el motor ********
        client.print('M');
        // Cambia a modo de recepción ******
        ClientMode = 1;
      }
      if (ClientMode == 1) {
        if (client.available() > 0) {
          char data = client.read();
          if (data == 'M') {
            MotorStateBuffer = true;
            ReadMotor = true;
          } else {
            if (!ReadMotor)
              MotorStateBuffer = false;
          }
          //Serial.println("No END");
          client.stop();
        }
      }
    }
    //Serial.println("Desconectado!");
  }
}
void ReadBytes() {
  while (Serial.available() < 16) {
    continue;
  }
  for (int i = 3; i >= 0; i--)
    pitchData[i] = Serial.read();
  for (int i = 3; i >= 0; i--)
    rollData[i] = Serial.read();
  for (int i = 3; i >= 0; i--)
    yawData[i] = Serial.read();
  for (int i = 3; i >= 0; i--)
    BatteryVoltageData[i] = Serial.read();
  //WaitForDataBytes = false;

}
void SerialHandler() {
  char data;
  if (Serial.available() > 0) {
    SerialTimeout = CurrentTime;  // Si pasa un segundo sin recibir datos, se detiene la conexión con el servidor ******
    SerialTimeoutOnce = true;    // Ejecuta el timeout una vez para evitar borrar datos futuros
    data = Serial.read();
    switch (data) {
      case 'C': // Solicitud de conectarse con el receptor
        WIFIConnect = true;
        if (ServeConnect) {
          Serial.print('C');
        } else {
          Serial.print('D');
        }
        break;
      case 'S':
        ReadBytes();
        //WaitForDataBytes = true;
        break;
      case 'O': // Apaga el WIFI
        WIFIConnect = false;
        break;
      case 'Y':
        ChargerConnect = true;
        break;
      case 'N':
        ChargerConnect = false;
        break;
      case 'M':
        ReadMotor = false;
        if (MotorStateBuffer) {
          Serial.print("M");
        } else {
          Serial.print("m");
        }
        break;
    }
  }
}
void WIFIConectionController() {
  if (WIFIConnect) {
    if (WiFi.status() == WL_DISCONNECTED) { // Si el WIFI esta apagado, lo enciende
      WiFi.mode(WIFI_STA);
    }
    if (WiFi.status() != WL_CONNECTED) { //Si no esta conectado intenta conectarse *****************
      ServeConnect = false; // Server desconectado (Conexión perdida o intentando conectar WIFI)
      if (!TryConnect) {
        //Serial.print("Connecting to ");
        //Serial.println(ssid);
        WiFi.begin(ssid, password);
        TryConnect = true;
      } else {
        if ((WiFi.status() == WL_CONNECT_FAILED) || (WiFi.status() == WL_NO_SSID_AVAIL) ) {
          //Serial.println("Failed to connect. Trying again");
          TryConnect = false;
        }
      }
    }
    if (WiFi.status() == WL_CONNECTED) {
      if (TryConnect) {
        //Serial.println("");
        //Serial.println("WiFi connected");
        //Serial.println("IP address: ");
        //Serial.println(WiFi.localIP());
        TryConnect = false;
      }
    }
  } else {
    ServeConnect = false; // Server desconectado (WIFI apagado)
    WiFi.mode(WIFI_OFF);
  }
}
void setup() {
  // Inicializacion del puerto serial
  Serial.begin(9600);
  // Modo inicial WIFI apagado ************
  WiFi.mode(WIFI_OFF);
  // LED para indicar el envio de datos
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // Client Handler
  if (WiFi.status() == WL_CONNECTED) {
    WIFIClientHandler();
  }
  // put your main code here, to run repeatedly:
  CurrentTime = millis();
  // Controlador de comunicación serial **************
  if ((CurrentTime - SerialTime) > Refresh) {
    SerialHandler();
    SerialTime = CurrentTime;
  }
  // Timeout del puerto serie ******
  if ((CurrentTime - SerialTimeout) > ESPSerialTimeOUT) {
    if (SerialTimeoutOnce) {
      // Limpia el puerto serie *******
      while (Serial.available() > 0) {
        Serial.read();
      }
      // Apaga el WIFI *************
      WIFIConnect = false;
      // Evita que se ejecute este codigo más de una vez ***
      SerialTimeoutOnce = false;
      // Desactiva el motor en caso de que esta variable haya quedado activada al desconectar el server
      MotorStateBuffer = false;
    }
  }
  // Controlador de conexión a red ******
  if ((CurrentTime - WIFIControllerTime) > WIFITime) {
    WIFIConectionController();
    WIFIControllerTime = CurrentTime;
  }
}
