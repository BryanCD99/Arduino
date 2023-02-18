#include <Wire.h>
#include <MPU6050.h>

// Definiciones del programa
#define Refresh 20              // Refresco del puerto serial
#define MPUReadTime 50          // Lectura del Giroscopio
#define SendDataTime 100        // Tiempo de envio de los datos al ESP8266
#define MotorTimeON 500           // Tiempo de encendido del motor
#define BatteryReadTime 1000    // Tiempo de lectura de la batería
#define ESPCONNECT 1000         // Tiempo de reconexion con el ESP
#define TimeoutButton 3000
#define BlinkLED_BLUE 2000       // Parpadeo led azul modo normal
#define BlinkLED_RED 200        // Parpadeo led rojo modo batería baja
#define BlinkCONNECT 200         // Parpadeo para indicar que se esta conectando el dispositivo


// Constantes del programa ******************
const int BUTTON = 2;
const int Vibrator = 10;
const int LED_RED = 11;
const int LED_BLUE = 12;
const int CHARGER = 13;
const int Battery_Monitor = A0;
const float Conversion = 1.1 / 1023;
const float BatteryConversion = 4.193;

// Variables de uso general *****************
bool EnableModule = false;  // Variable que se utiliza para activar o desactivar todos los modulos
bool TimeoutON = false;     // Variable empleada para conocer si el pulsador se mantuvo presionado dentro del tiempo establecido
int SystemState = 0;        // Indica el estado del sistema. 0: sistema conectandose, 1: sistema conectado. 2: sistema con batería baja
bool LedBlueState = false;
bool LedRedState = false;
float BatteryVoltage = 0;
bool ChargerConnect = false;
bool ConnectESP = false;
bool TurnOnMotor = false;
int BatteryPorcent = 0;

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
volatile unsigned long CurrentTime = 0;
volatile unsigned long ButtonTime = 0;
unsigned long LED_REDTime = 0;
unsigned long LED_BLUETime = 0;
unsigned long SerialTime = 0;
unsigned long BatteryTime = 0;
unsigned long MPUTime = 0;
unsigned long ESPCONNECTTime = 0;
unsigned long ESPSendDataTime = 0;
unsigned long MotorTime = 0;

// Instancia del sensor MPU 6050
MPU6050 mpu;

void setup() {
  // Configuración del puerto serial *****************
  Serial.begin(115200);

  // Configuración de pines de entrada y salida ******
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(Vibrator, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(CHARGER, INPUT);

  // Estado inicial de las salidas ******************
  digitalWrite(Vibrator, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);

  // Configuración de referencia de voltaje
  analogReference(INTERNAL);

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Configuración de umbral giroscopio ************
  mpu.setThreshold(3);
  // Configuración de interrupción externa *********
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(BUTTON), ButtonISRHandler, FALLING);
  interrupts();

}
void ButtonISRHandler() {
  TimeoutON = true;
  ButtonTime = CurrentTime;
}

void SystemON_OFF() {
  EnableModule = !EnableModule;
  if (EnableModule) {
    LedBlueState = true;
    LedRedState = false;
    SystemState = 0;
    mpu.calibrateGyro();        // Calibra el giroscopio **************************************
    TurnOnMotor = false;
    ConnectESP = false;         // Indica al sistema que debe conectarse al ESP8266
    Serial.print('C');          // Envia el caracter C (confirma mijin) para indicar al ESP que se debe conectar ***********
    ESPCONNECTTime = CurrentTime; // Reestablece el contador de tiempo para evitar un doble envio del caracter en el loop
    mpu.setSleepEnabled(false);
  } else {
    digitalWrite(Vibrator, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
    Serial.print('O');          // Apaga WIFI en el ESP8266
    mpu.setSleepEnabled(true);
  }
}
void SerialHandler() {
  char data;
  if (Serial.available() > 0) {
    data = Serial.read();
    switch (data) {
      case 'C': // si recibe la C, el ESP ha confirmado xD
        SystemState = 1;  // Pasa a modo de conexión
        ConnectESP = true;
        // Apaga el LED rojo debido a que podria haberse quedado encendido durante el modo de conexión
        LedRedState = false;
        digitalWrite(LED_RED, LedRedState);
        break;
      case 'D':
        break;
      case 'M':
        TurnOnMotor = true;
        MotorTime = CurrentTime;
        digitalWrite(Vibrator, HIGH);
        break;
    }
  }
}
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
void ReadMPU() {
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis;// * (float)MPUReadTime / 1000;
  roll = roll + norm.XAxis;// * (float)MPUReadTime / 1000;
  yaw = yaw + norm.ZAxis;// * (float)MPUReadTime / 1000;
}
void SenData() {
  Serial.print('S');     // Indica que se van a enviar datos ************
  for (int i = 3; i >= 0; i--)
    Serial.write(pitchData[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(rollData[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(yawData[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(BatteryVoltageData[i]);
  if (ChargerConnect) {
    Serial.print('Y');  // Envia Y (yes) si el cargador esta conectado
  } else {
    Serial.print('N');  // Envia N (No) si el cargador esta desconectado
  }
}
void loop() {
  // Actualización de tiempo de sistema **************
  CurrentTime = millis();

  // Controlador de comunicación serial **************
  if (((CurrentTime - SerialTime) > Refresh) && (EnableModule == true)) {
    SerialHandler();
    SerialTime = CurrentTime;
  }
  // Controlador de lectura del giroscopio **************
  if (((CurrentTime - MPUTime) > MPUReadTime) && (EnableModule == true)) {
    ReadMPU();
    MPUTime = CurrentTime;
  }
  // Controlador de activación o desactivación del modulo
  if (((CurrentTime - ButtonTime) > TimeoutButton) && (TimeoutON == true)) {
    TimeoutON = false;
    if (digitalRead(BUTTON) == 0) {
      SystemON_OFF();
    }
  }
  // Controlador de conección con el ESP **************
  if (((CurrentTime - ESPCONNECTTime) > ESPCONNECT) && (EnableModule == true) && (ConnectESP == false)) {
    Serial.print('C');          // Envia el caracter C para indicar al ESP que se debe conectar ***********
    ESPCONNECTTime = CurrentTime;
  }
  // Controlador de envio de datos al ESP8266 **************
  if (((CurrentTime - ESPSendDataTime) > SendDataTime) && (EnableModule == true) && (SystemState == 1)) {
    SenData();
    ESPSendDataTime = CurrentTime;
  }
  // Lectura de la batería y detección del cargador *************
  if (((CurrentTime - BatteryTime) > BatteryReadTime) && (EnableModule == true)) {
    BatteryVoltage = analogRead(Battery_Monitor) * Conversion * BatteryConversion;
    BatteryPorcent = map(BatteryVoltage, 3.5, 4.10, 0, 100);
    if (digitalRead(CHARGER) == 1) {
      ChargerConnect = true;
    } else {
      ChargerConnect = false;
    }
    BatteryTime = CurrentTime;
  }
  // Controlador del parpadeo del led azul *************
  if (((CurrentTime - LED_BLUETime) > LEDS_TIME(false)) && (EnableModule == true)) {
    LedBlueState = !LedBlueState;
    digitalWrite(LED_BLUE, LedBlueState);
    LED_BLUETime = CurrentTime;
  }
  // Controlador del parpadeo del led rojo *************
  if (((CurrentTime - LED_REDTime) > LEDS_TIME(true)) && (EnableModule == true)) {
    LedRedState = !LedRedState;
    digitalWrite(LED_RED, LedRedState);
    LED_REDTime = CurrentTime;
  }
  // Controlador de encendido del motor **************
  if (((CurrentTime - MotorTime) > MotorTimeON) && (EnableModule == true) && (TurnOnMotor == true)) {
    digitalWrite(Vibrator, LOW);
    TurnOnMotor = false;
  }
}
