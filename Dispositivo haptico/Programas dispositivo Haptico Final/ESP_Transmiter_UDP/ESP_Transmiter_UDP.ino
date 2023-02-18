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
byte PWMData = 0;

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
  /*
   * Normalmente se recibe la 'm' minuscula para indicar que no hay que encender el motor, esto se hace debido a la 
   * posibilidad de que se pierdan paquetes de información, por lo que se mantiene siempre esta confirmación
   * Cuando se recibe un paquete de dos bytes, con 'M' como prefijo, indica al sistema que se ha recibido un valor para
   * el PWM, y el valor no se puede alterar hasta que el receptor vuelva a enviar 'm' minuscula. De igual forma solo se 
   * envia el valor del PWM una sola vez cuando se lee el puerto serie del ESP8266. Para leer un nuevo valor, este debe llegar
   * por el cliente UDP
   */
  DataTimeout = millis(); // Reset para el timout. Si se superan los 50mS, se sale del bucle
  while (true) {
    int packetSize = Udp.parsePacket();
    //Serial.println( packetSize);
    if (packetSize == 1) {  // Cuando no hay que encender el motor se recibe la m y sirve tambien para resetear
      // read the packet    // las variables correspondientes
      char data = Udp.read();
      if (data == 'm') {
        reciveMotorOn = false;
        ReadMotor = false;
      }
      break;                                                    // exit the while-loop
    }
    if (packetSize == 2) {  // Cuando hay que encender el motor, se recibe como prefijo la M seguido del valor
      // read the packet    // del PWM
      char data = Udp.read();
      if (data == 'M') {
        if (!ReadMotor) {
          reciveMotorOn = true;
          ReadMotor = true; // Habilita para enviar el valor de PWM
          MotorStateBuffer = true;
          PWMData = (byte)Udp.read(); // Valor del PWM
        }
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
          Serial.write(PWMData);
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
