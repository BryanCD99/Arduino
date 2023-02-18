#include <SoftwareSerial.h>

#define BlinkLED_BLUE 2000       // Parpadeo led azul modo normal
#define BlinkLED_RED 200        // Parpadeo led rojo modo batería baja
#define BlinkCONNECT 200         // Parpadeo para indicar que se esta conectando el dispositivo
#define CONECTION 10

// Constantes del programa ******************
const int TX = 3;
const int RX = 2;
const int LED_BLUE = 4;
const int LED_RED = 5;

// Variables de uso general *****************
bool LedBlueState = false;
bool LedRedState = false;
float BatteryVoltage = 0;
bool ChargerConnect = false;
char LastData = 'D';
byte BufferSerialData[4];
int SystemState = 0;        // Indica el estado del sistema. 0: sistema conectandose, 1: sistema conectado. 2: sistema con batería baja
int GetESPDataMode = 0;     // 0 indica que va a pedir datos del acelerometro, 1 indica que se va a preguntar por el cargador, 2 indica que se va preguntar si hay un cliente conectado
bool WaitForIncomingData = false;
bool SendMotorOn = false;
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
unsigned long LED_REDTime = 0;
unsigned long LED_BLUETime = 0;
unsigned long ConexionTime = 0;

// Puerto serial virtual para el ESP8266
SoftwareSerial ESPSerial(RX, TX); // RX, TX

// Funcion de union para la conversión de array de 4 bytes a float32
union u_tag {
  byte b[4];
  float fval;
} u;

unsigned long LEDS_TIME(bool LED) {  // LED = 0: Led azul, // LED = 1: Led red
  switch (SystemState) {
    case 0:
      return BlinkCONNECT;
      break;
    case 1:
      if (LED) {
        return 999999999;   // El led no debe parpadear, por lo que se envia un valor muy alto para que no tenga tiempo para parpadear
      } else {
        return BlinkLED_BLUE;
      }
      break;
    case 2:
      if (LED) {
        return BlinkLED_RED;
      } else {

        return 999999999;   // El led no debe parpadear, por lo que se envia un valor muy alto para que no tenga tiempo para parpadear
      }
      break;
  }
}

void PCSerialHandler()
{
  char data;
  if (Serial.available() > 0)
  {
    data = Serial.read();
    switch (data)
    {
      case 'R':
        Send2PC();
        break;
      case 'M':
        SendMotorOn = true;
        GetPWM();
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
void ESPSerialHandler()
{
  char data;
  data = ESPSerial.read();
  switch (data)
  {
    case 'C':
      if (LastData == 'D') {
        LastData = 'C';
        SystemState = 1;  // Pasa a modo de conexión
        // Apaga el LED rojo debido a que podria haberse quedado encendido durante el modo de conexión
        LedRedState = false;
        digitalWrite(LED_RED, LedRedState);
        //Serial.println("Cliente Conectado");
      }
      break;
    case 'D':
      if (LastData == 'C') {
        LastData = 'D';
        //Serial.println("Cliente desconectado");
        LedBlueState = true;
        LedRedState = false;
        digitalWrite(LED_BLUE, LedBlueState);
        digitalWrite(LED_RED, LedRedState);
        LED_BLUETime = CurrentTime;
        LED_REDTime = CurrentTime;
        SystemState = 0;
      }
      break;
  }
}
float GetESPSerialData() {
  for (int i = 3; i >= 0; i--)
  {
    BufferSerialData[i] = (byte)ESPSerial.read();
  }
  u.b[3] = BufferSerialData[3];
  u.b[2] = BufferSerialData[2];
  u.b[1] = BufferSerialData[1];
  u.b[0] = BufferSerialData[0];
  return u.fval;
}
void Send2PC() {
  for (int i = 0; i <= 3; i++)
    Serial.write(pitchData[i]);
  for (int i = 0; i <= 3; i++)
    Serial.write(rollData[i]);
  for (int i = 0; i <= 3; i++)
    Serial.write(yawData[i]);
  for (int i = 0; i <= 3; i++)
    Serial.write(BatteryVoltageData[i]);
  if (ChargerConnect) {
    Serial.print('Y');
  } else {
    Serial.print('N');
  }
  if (SystemState == 1) {
    Serial.print('C');
  }
  if (SystemState == 0) {
    Serial.print('D');
  }
}
void ReadData() {
  // PRegunta por todos los datos de movimiento y el voltaje de la batería **
  if (GetESPDataMode == 0) {
    if (!WaitForIncomingData) {
      WaitForIncomingData = true;
      //Serial.println("Datos");
      ESPSerial.print('R');
    }
    if (ESPSerial.available() >= 16) {
      //Serial.println("Datos recibidos");
      pitch = GetESPSerialData();
      roll = GetESPSerialData();
      yaw = GetESPSerialData();
      BatteryVoltage = GetESPSerialData();
      WaitForIncomingData = false;
      GetESPDataMode = 1;
    }
  }
  // Pregunta si el cargador esta conectado ****
  if (GetESPDataMode == 1) {
    if (!WaitForIncomingData) {
      WaitForIncomingData = true;
      ESPSerial.print('B');
      //Serial.println("Cargador");
    }
    if (ESPSerial.available() == 1) {
      char Data = ESPSerial.read();
      if (Data == 'Y') {
        //Serial.println("Cargador conectado");
        ChargerConnect = true;
      }
      if (Data == 'N') {
        //Serial.println("Cargador desconectado");
        ChargerConnect = false;
      }
      WaitForIncomingData = false;
      GetESPDataMode = 2;
    }
  }
  // Pregunta si esta conectado un cliente
  if (GetESPDataMode == 2) {
    if (!WaitForIncomingData) {
      WaitForIncomingData = true;
      //Serial.println("Conexion");
      ESPSerial.print('C');
    }
    if (ESPSerial.available() == 1) {
      ESPSerialHandler();
      //Serial.println("Conexion establecida");
      WaitForIncomingData = false;
      GetESPDataMode = 3;
    }
  }
  if (GetESPDataMode == 3) {
    if (SendMotorOn) {
      ESPSerial.print('M');
      ESPSerial.write(PWMData);
    }
    SendMotorOn = false;
    GetESPDataMode = 0;
  }
}
void setup() {
  // Configuración serial para la comunicación con el ordenador
  Serial.begin(115200);
  // Configuración de pines de entrada y salida ******
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Estado inicial de las salidas ******************
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);

  // Configuración serial para la comunicación con la antena ESP8266
  ESPSerial.begin(9600);
  // Estado inicial de los LEDS
  LedBlueState = true;
  LedRedState = false;
  SystemState = 0;

  // Espera a que el ESP8266 inicie correctamente ****
  delay(300);
  // Limpia el puerto serie del ESP8266 al encender *******
  while (ESPSerial.available() > 0) {
    ESPSerial.read();
  }
}

void loop() {
  // Actualización de tiempo del sistema
  CurrentTime = millis();

  // Envio y recepción de datos a consola serial *************************
  if ((CurrentTime - SerialTime) >= 10)
  {
    PCSerialHandler();
    SerialTime = CurrentTime;
  }
  // Controlador del parpadeo del led azul *************
  if ((CurrentTime - LED_BLUETime) > LEDS_TIME(false)) {
    LedBlueState = !LedBlueState;
    digitalWrite(LED_BLUE, LedBlueState);
    LED_BLUETime = CurrentTime;
  }
  // Controlador del parpadeo del led rojo *************
  if ((CurrentTime - LED_REDTime) > LEDS_TIME(true)) {
    LedRedState = !LedRedState;
    digitalWrite(LED_RED, LedRedState);
    LED_REDTime = CurrentTime;
  }
  //  Controlador de la conexión con el ESP *************
  if ((CurrentTime - ConexionTime) > CONECTION) {
    ReadData();
    ConexionTime = CurrentTime;
  }
}
