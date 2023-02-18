#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

// Definiciones del programa
#define Refresh 10              // Refresco del puerto serial
#define ReconnectTime 1000          // Si ningun cliente se conecta en 1S se indica dispositivo desconectado
#define ClientTimeOut 1000      // Timout para el cliente en caso de desconexión durante la comunicación

// Set your access point network credentials
const char* ssid = "DH_01";
const char* password = "123456789";

// Variables de uso general *****************
float BatteryVoltage = 0;
bool ChargerConnect = false;
bool SendMotorOn = false;
bool SendDisconectClient = true;
char packetBuffer[16]; //buffer to hold incoming packet
byte PWMData = 0;

// Variables Pitch, Roll and Yaw para la obtención de angulos de inclinación
float pitch = 0;    // Incremento en Y
float roll = 0;     // Incremento en X
float yaw = 0;      // Incremento en Z

// Variables para la comunicación con el ESP *****
unsigned char *pitchData = (unsigned char *)&pitch;
unsigned char *rollData = (unsigned char *)&roll;
unsigned char *yawData = (unsigned char *)&yaw;
unsigned char *BatteryVoltageData = (unsigned char *)&BatteryVoltage;

// Variables de control de ejecución *************
unsigned long CurrentTime = 0;
unsigned long SerialTime = 0;
unsigned long Reconnect = 0;

// Configuración de red para el ESP8266 **********************
IPAddress local_IP(192, 168, 2, 220);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

// Puerto para el web socket *********************************
unsigned int UDPPort = 8080;      // local port to listen on
WiFiUDP Udp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  // Setting the WiFi mode
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  IPAddress IP = WiFi.softAPIP();

  Udp.begin(UDPPort);
}
void SenData() {
  for (int i = 3; i >= 0; i--)
    Serial.write(pitchData[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(rollData[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(yawData[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(BatteryVoltageData[i]);
}
void SerialHandler() {
  char data;
  if (Serial.available() > 0) {
    data = Serial.read();
    switch (data) {
      case 'C':
        if (SendDisconectClient) {
          Serial.print('D');  // Cliente desconectado **************
        } else {
          Serial.print('C');  // Cliente conectado **************
        }
        break;
      case 'R': // si recibe la R (Read), el ESP envia los datos que se han recibido
        SenData();
        break;
      case 'B':
        if (ChargerConnect) {
          Serial.print('Y');  // Envia Y (yes) si el cargador esta conectado
        } else {
          Serial.print('N');  // Envia N (No) si el cargador esta desconectado
        }
        break;
      case 'M':
        GetPWM();
        SendMotorOn = true;
        break;
    }
  }
}
void GetPWM() {
  while (Serial.available() == 0) {
    continue;
  }
  PWMData = (byte)Serial.read();
}
void ServerHandler() {
  // Revisa si se han recibido paquetes ***************
  int packetSize = Udp.parsePacket(); // almacena el tamaño del paquete recibido, si no ha datos, vale 0
  if (packetSize) {
    digitalWrite(LED_BUILTIN, LOW);
    // Si se dejan de recibir paquetes, entonces empieza a correr un contador que al pasar un segundo indica que se perdio la conexion *****
    SendDisconectClient = false; // Desactiva el envio de desconexion de cliente al arduino nano
    Reconnect = millis(); // Resetea el contador

    // Paquetes de 1 byte indican comando ***********
    if (packetSize == 1) {
      char data = Udp.read();
      //Serial.println("Contents:");
      //Serial.println(data);
      switch (data) {
        case 'Y':
          ChargerConnect = true;
          break;
        case 'N':
          ChargerConnect = false;
          break;
        case 'm':
          if (SendMotorOn) {
            Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
            Udp.write('M');
            Udp.write(PWMData);
            Udp.endPacket();
          } else {
            Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
            Udp.write('m');
            Udp.endPacket();
          }
          break;
        case 'M':
          SendMotorOn = false;
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write('m');
          Udp.endPacket();
          break;
      }
    }
    // Paquetes de 16 bytes indican datos *************+
    if (packetSize == 16) {
      Udp.read(packetBuffer, 16);
      for (int i = 0; i <= 3; i++)
      {
        pitchData[i] = (byte)packetBuffer[i];
      }
      for (int i = 4; i <= 7; i++)
      {
        rollData[i - 4] = (byte)packetBuffer[i];
      }
      for (int i = 8; i <= 11; i++)
      {
        yawData[i - 8] = (byte)packetBuffer[i];
      }
      for (int i = 12; i <= 15; i++)
      {
        BatteryVoltageData[i - 12] = (byte)packetBuffer[i];
      }
    }
    // Paquetes de otros tamaños quedan descartados ***************
  }
}
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // ServerHandler ***********************************
  ServerHandler();

  // Actualización de tiempo de sistema **************
  CurrentTime = millis();

  // Controlador de comunicación serial **************
  if ((CurrentTime - SerialTime) > Refresh) {
    SerialHandler();
    SerialTime = CurrentTime;
  }
  // Controlador de conexion activa **************
  if ((CurrentTime - Reconnect) > ReconnectTime) {
    if (SendDisconectClient == false) {
      SendDisconectClient = true;
    }
  }
}
