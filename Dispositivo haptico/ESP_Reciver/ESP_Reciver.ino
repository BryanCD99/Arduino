#include <ESP8266WiFi.h>

// Definiciones del programa
#define Refresh 20              // Refresco del puerto serial
#define ReconnectTime 1000          // Si ningun cliente se conecta en 1S se indica dispositivo desconectado
#define ClientTimeOut 1000      // Timout para el cliente en caso de desconexión durante la comunicación


// Set your access point network credentials
const char* ssid = "DH_01";
const char* password = "123456789";

// Configuración de red para el ESP8266 **********************
IPAddress local_IP(192, 168, 2, 220);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

// Puerto para el web socket *********************************
WiFiServer wifiServer(8080);

// Variables de uso general *****************
float BatteryVoltage = 0;
bool ChargerConnect = false;
bool SendMotorOn = false;
bool SendDisconectClient = true;
bool GetClientCommand = false;

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
unsigned long CTimeOut = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  // Setting the ESP as an access point
  //  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);
  //  Serial.print("Estableciendo configuración Soft-AP... ");
  //  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Listo" : "Falló!");
  WiFi.softAPConfig(local_IP, gateway, subnet);
  IPAddress IP = WiFi.softAPIP();
  //  Serial.print("AP IP address: ");
  //  Serial.println(IP);

  // Inicia el servidor *******************************
  wifiServer.begin();
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
        SendMotorOn = true;
        break;
    }
  }
}
void ServerHandler() {
  WiFiClient client = wifiServer.available();
  if (client) {
    //client.setNoDelay(true);
    char data;
    //    Serial.println("Cliente Conectado");
    CTimeOut = millis();
    GetClientCommand = false;
    SendDisconectClient = false; // Desactiva el envio de desconexion de cliente al arduino nano
    Reconnect = CurrentTime; // Resetea el contador
    while (client.connected() || client.available() > 0) {
      // Timout Control *******************
      if ((millis() - CTimeOut) > ClientTimeOut) {
        //        Serial.println("Timeout");
        break;
      }
      // START CODE SERVER *****************************
      if (!GetClientCommand) {
        if (client.available() > 0) {
          data = client.read();
          //          Serial.print("Comando recibido: ");
          //          Serial.println(data);
          GetClientCommand = true;
        }
      }
      if (GetClientCommand) {
        switch (data) {
          case 'R': // Read(Lee) todos los datos de rotación(X;Y;Z) y el voltaje de la batería
            digitalWrite(LED_BUILTIN, LOW);
            if (client.available() >= 16) {
              for (int i = 0; i <= 3; i++)
              {
                pitchData[i] = (byte)client.read();
              }
              for (int i = 0; i <= 3; i++)
              {
                rollData[i] = (byte)client.read();
              }
              for (int i = 0; i <= 3; i++)
              {
                yawData[i] = (byte)client.read();
              }
              for (int i = 0; i <= 3; i++)
              {
                BatteryVoltageData[i] = (byte)client.read();
              }
              GetClientCommand = false;
            }
            break;
          case 'Y':
            ChargerConnect = true;
            GetClientCommand = false;
            break;
          case 'N':
            ChargerConnect = false;
            GetClientCommand = false;
            break;
          case 'M':
            if (SendMotorOn) {
              client.print('M');
              SendMotorOn = false;
            } else {
              client.print('m');
            }
            GetClientCommand = false;
            break;
        }
      }
      //      Serial.print("Cliente estado: ");
      //      Serial.println(client.connected());
      //      Serial.print("Buffer estado: ");
      //      Serial.println(client.available());
      // END CODE SERVER ********************************
    }
    //    Serial.println("Cliente desconectado");
    client.stop();
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
    Reconnect = CurrentTime;
  }
}
