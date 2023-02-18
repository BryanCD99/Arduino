#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

// Definiciones del programa
#define Refresh 10              // Refresco del puerto serial
#define WIFISendData 50         // Refresco entre envio de los datos
#define WIFITime 500            // Intervalo de verificación para el control de la conexion WIFI
#define InputDataTimeOut 50     // Timeout para cuando no se reciban datos por el websocket

//access point network credentials
const char* ssid = "DH_01";
const char* password = "123456789";

// Host y port para la comunicación con el socket del ESP8266 receptor ***
const char* host = "192.168.2.220";
const uint16_t port = 8080;

// Variables de uso general *******************
bool WIFIConnect = false;
bool TryConnect = false;
bool ServeConnect = false;
bool reciveMotorOn = false;

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

// Variables de control de ejecución **********
unsigned long CurrentTime = 0;
unsigned long SerialTime = 0;
unsigned long WIFIControllerTime = 0;
unsigned long WIFISendUDP = 0;
unsigned long DataTimeout = 0;

// Configuración de red para el ESP8266 **********************
IPAddress local_IP(192, 168, 2, 220);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

// Puerto para el web socket *********************************
unsigned int UDPPort = 8080;      // local port to listen on
WiFiUDP Udp;

void WIFIClientHandler() {
  digitalWrite(LED_BUILTIN, LOW);
  ServeConnect = true;
  // Envio de los datos ***********************
  Udp.beginPacket(local_IP, UDPPort);
  for (int i = 0; i <= 3; i++)
    Udp.write(pitchData[i]);
  for (int i = 0; i <= 3; i++)
    Udp.write(rollData[i]);
  for (int i = 0; i <= 3; i++)
    Udp.write(yawData[i]);
  for (int i = 0; i <= 3; i++)
    Udp.write(BatteryVoltageData[i]);
  Udp.endPacket();                                              // this will automatically send the data
  //delay(10);
  // Envio de los comandos *********************
  Udp.beginPacket(local_IP, UDPPort);
  // Envia el estado actual del cargador **********
  if (ChargerConnect) {
    Udp.write('Y');
  } else {
    Udp.write('N');
  }
  Udp.endPacket();
  //delay(10);
  // Envia solicitud de estado para el motor ********
  Udp.beginPacket(local_IP, UDPPort);
  if (!reciveMotorOn) {
    Udp.write('m');
  } else {
    Udp.write('M');
  }
  Udp.endPacket();

  // Recibe el estado del motor *********************
  DataTimeout = millis();
  while (true) {
    int packetSize = Udp.parsePacket();
    //Serial.println( packetSize);
    if (packetSize == 1) {
      // read the packet
      char data = Udp.read();
      if (data == 'M') {
        if (!ReadMotor) {
          reciveMotorOn = true;
          ReadMotor = true;
          MotorStateBuffer = true;
        }
      }
      if (data == 'm') {
        reciveMotorOn = false;
        ReadMotor = false;
      }
      break;                                                    // exit the while-loop
    }
    if ((millis() - DataTimeout) > InputDataTimeOut) {          // Si transcurren 50 mS sin recibir nada, se sale del bucle
      break;                                                    // exit
    }
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
}
void SerialHandler() {
  char data;
  if (Serial.available() > 0) {
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
          Serial.print('M');
          MotorStateBuffer = false;
        } else {
          Serial.print('m');
        }
        break;
    }
  }
}
void WIFIConectionController() {
  if (WIFIConnect) {
    if (WiFi.status() == WL_DISCONNECTED) { // Si el WIFI esta apagado, lo enciende
      WiFi.mode(WIFI_AP_STA);
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
        //        Serial.println("");
        //        Serial.println("WiFi connected");
        //        Serial.println("IP address: ");
        //        Serial.println(WiFi.localIP());
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
  Udp.begin(UDPPort);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // Client Handler
  if (WiFi.status() == WL_CONNECTED) {
    // Controlador de envio de datos ******
    if ((CurrentTime - WIFISendUDP) > WIFISendData) {
      WIFIClientHandler();
      WIFISendUDP = CurrentTime;
    }
  }
  CurrentTime = millis();
  // Controlador de comunicación serial **************
  if ((CurrentTime - SerialTime) > Refresh) {
    SerialHandler();
    SerialTime = CurrentTime;
  }
  // Controlador de conexión a red ******
  if ((CurrentTime - WIFIControllerTime) > WIFITime) {
    WIFIConectionController();
    WIFIControllerTime = CurrentTime;
  }
}
