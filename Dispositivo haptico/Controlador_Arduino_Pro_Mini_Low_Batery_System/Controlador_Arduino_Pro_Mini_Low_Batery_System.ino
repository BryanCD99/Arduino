#include <Wire.h>
#include <MPU6050.h>

// Definiciones del programa
#define MPUReadTime 10          // Lectura del Giroscopio
#define SerialTime 20           // Refresco del puerto serial

#define MotorTimeON 200         // Tiempo de encendido del motor
#define BatteryReadTime 1000    // Tiempo de lectura de la batería

#define BlinkLED_RED 200        // Parpadeo led rojo modo batería baja
#define TimeoutButton 3000      // Periodo para la detección del boton de encendido o apagado
#define BlinkLED_BLUE 2000      // Parpadeo led azul modo normal

#define BlinkBatteryLow 3       // Veces que parpadeara el Led antes de apagar el sistema en batería baja;

#define BlinkCONNECT 200        // Parpadeo para indicar que se esta conectando el dispositivo
#define RAD_A_DEG = 57.295779   // Conversion de radianes a grados 180/PI

#define BatteryLow 3.60         // Voltaje de batería baja, el dispositivo entra en modo de bajo consumo
// al llegar a este voltaje. Se desactiva al conectar el cargador...

// Constantes del programa ******************
const int BUTTON = 2;
const int Vibrator = 10;
const int LED_RED = 11;
const int LED_BLUE = 12;
const int CHARGER = 13;
const int Battery_Monitor = A0;
const float Conversion = 1.1 / 1023;
const float BatteryConversion = 4.193;

// Variables Pitch, Roll and Yaw para la obtención de angulos de inclinación y voltaje de batería
float pitch = 0;    // Incremento en Y
float roll = 0;     // Incremento en X
float yaw = 0;      // Incremento en Z
float BatteryVoltage = 0;

// Variables para la comunicación con el ESP *****
unsigned char *pitchData = (unsigned char *)&pitch;
unsigned char *rollData = (unsigned char *)&roll;
unsigned char *yawData = (unsigned char *)&yaw;
unsigned char *BatteryVoltageData = (unsigned char *)&BatteryVoltage;

// Variables de uso general **********************
int SystemState = 0;        // Indica el estado del sistema. 0: sistema conectandose, 1: sistema conectado. 2: sistema con batería baja
bool TimeoutON = false;     // Variable empleada para conocer si el pulsador se mantuvo presionado dentro del tiempo establecido
bool EnableModule = false;  // Variable que se utiliza para activar o desactivar todos los modulos
bool LedBlueState = false;  // Estado del Led azul
bool LedRedState = false;   // Estado del Led rojo
bool ChargerConnect = false; // Estado de conexión del cargador
bool TurnOnMotor = false;          // Estado del motor
bool SendCommandOnce = false; // Envio de comando una unica vez
int SerialMode = 0;           // 0: Solicitud de conexion de red, 1: envio de datos y del estado del cargador, 2: Lectura del motor.
char LastData = 'D';
bool LowBatteryDetect = false;  // 0: batería con carga, 1: batería baja
int CountBeforeTurnOff = 0;

// Variables para la deteccion del movimiento *****
bool detect = false;
bool Go_Zero = false;

// Variables de control de ejecución *************
volatile unsigned long CurrentTime = 0;
volatile unsigned long ButtonTime = 0;
unsigned long LED_REDTime = 0;
unsigned long LED_BLUETime = 0;
unsigned long ESPSerial = 0;
unsigned long BatteryTime = 0;
unsigned long MPUTime = 0;

unsigned long ESPSendDataTime = 0;
unsigned long MotorTime = 0;

// Instancia del sensor MPU 6050
MPU6050 mpu;

void setup() {
  // Configuración del puerto serial *****************
  Serial.begin(9600);

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
  while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
  {
    //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    digitalWrite(LED_RED, HIGH);
    delay(100);
    digitalWrite(LED_RED, LOW);
    delay(400);
  }
  // Desactiva el modulo en su estado inicial
  mpu.setSleepEnabled(true);
  // Configuración de interrupción externa *********
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(BUTTON), ButtonISRHandler, CHANGE);
  interrupts();
}

void ButtonISRHandler() {
  ButtonTime = CurrentTime;
  if (digitalRead(BUTTON) == 0) {
    TimeoutON = true;
  } else {
    TimeoutON = false;
  }
}
void SystemON_OFF() {
  EnableModule = !EnableModule;
  if (EnableModule) {
    CountBeforeTurnOff = 0; // Reinicia el contador de batería baja
    // Solo enciende si hay cargador o si la batería no esta baja ***************
    if (!LowBatteryDetect || (digitalRead(CHARGER) == 1)) {
      LowBatteryDetect = false;
      LedBlueState = true;
      LedRedState = false;
      SystemState = 0;
      mpu.setSleepEnabled(false);
      mpu.calibrateGyro();        // Calibra el giroscopio **************************************
      pitch = 0;
      roll = 0;
      yaw = 0;
      SerialMode = 0;
      detect = false;
      Go_Zero = false;
      SendCommandOnce = false;
    } else {
      // Si hay batería baja, entonces indica con los leds usando SystemState = 2
      SystemState = 2;
      LedBlueState = false;
      LedRedState = false;
      EnableModule = false;
      digitalWrite(LED_BLUE, LedRedState);
      digitalWrite(LED_RED, LedRedState);
    }
  } else {
    digitalWrite(Vibrator, LOW);
    digitalWrite(LED_RED, LOW);
    if (SystemState != 2) {
      if (LedBlueState) {
        digitalWrite(LED_BLUE, LOW);
        delay(250);
        digitalWrite(LED_BLUE, HIGH);
        delay(250);
        digitalWrite(LED_BLUE, LOW);
        delay(250);
        digitalWrite(LED_BLUE, HIGH);
        delay(250);
        digitalWrite(LED_BLUE, LOW);
      } else {
        delay(250);
        digitalWrite(LED_BLUE, HIGH);
        delay(250);
        digitalWrite(LED_BLUE, LOW);
        delay(250);
        digitalWrite(LED_BLUE, HIGH);
        delay(250);
        digitalWrite(LED_BLUE, LOW);
      }
      delay(250);
    }
    Serial.print('O');  // Apaga el WIFI
    // Limpia el puerto serie *******
    while (Serial.available() > 0) {
      Serial.read();
    }
    SystemState = 0; // Para apagar el sistema se deja el modo 0
    mpu.setSleepEnabled(true);
  }
}
void ReadMPU(float dt) {
  dt = dt / 1000;
  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();
  // Read normalized values
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll
  pitch = atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * RAD_TO_DEG;
  roll = atan2(normAccel.YAxis, sqrt(normAccel.XAxis * normAccel.XAxis + normAccel.ZAxis * normAccel.ZAxis)) * RAD_TO_DEG;

  //Aplicar el Filtro Complementario
  pitch = round(0.98 * (pitch + normGyro.XAxis * dt) + 0.02 * pitch);
  roll = round(0.98 * (roll + normGyro.YAxis * dt) + 0.02 * roll);

  // Se cambio el parametro YAW para detectar un movimiento rápido en Y ****
  /*
    //Integración respecto del tiempo paras calcular el YAW
    if (abs((normGyro.ZAxis * dt)) > 1) {
      yaw = round(yaw + normGyro.ZAxis * dt);
    }
  */

  // Si hay conexion entonces, se activa el modo de detección de movimiento rápido ******
  if (SystemState == 1) {
    // Si se detecta un movimiento rápido y además el angulo es menor a -30, entonces
    if ((normGyro.YAxis < -200) && (pitch < -30)) {
      if (!Go_Zero) {
        Go_Zero = true;
        detect = !detect;
      }
    }
    if (abs(normGyro.YAxis) < 10) {
      Go_Zero = false;
    }
    if (detect) {
      yaw = 100;  // Indica que se ha detectado un movimiento de sujetar ****
    } else {
      yaw = 0;    // Indica que se ha detectado un movimiento de soltar *****
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
void SerialData() {
  if (SerialMode == 0) {
    if (!SendCommandOnce) {
      // Solicita el estado actual de la conexión
      Serial.print('C');
      SendCommandOnce = true;
    }
    if (Serial.available() > 0) {
      SerialHandler();
      SerialMode = 1;
      SendCommandOnce = false;
    }
  }
  if (SerialMode == 1) {
    if (!TurnOnMotor)   // No se envian datos durante el encendido del motor para evitar enviar falsos movimientos
      SendData();       // por las vibraciones
    SerialMode = 2;
  }
  if (SerialMode == 2) {
    if (!SendCommandOnce) {
      // Solicita el estado actual del motor
      Serial.print('M');
      SendCommandOnce = true;
    }
    if (Serial.available() > 0) {
      SerialHandler();
      SerialMode = 0;
      SendCommandOnce = false;
    }
  }
}
void SendData() {
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
void SerialHandler() {
  char data;
  data = Serial.read();
  switch (data) {
    case 'C':
      if (LastData == 'D') {
        SystemState = 1;            // Pasa a modo de conexión
        LedRedState = false;        // Apaga el LED rojo debido a que podria haberse quedado encendido durante el modo de conexión
        digitalWrite(LED_RED, LedRedState);
      }
      break;
    case 'D':
      if (LastData == 'C') {
        SystemState = 0;            // Pasa a modo de desconexión
        LedBlueState = true;
        LedRedState = false;
      }
      break;
    case 'M':
      TurnOnMotor = true;
      MotorTime = CurrentTime;
      digitalWrite(Vibrator, HIGH);
      break;
  }
}
void loop() {
  // Actualización de tiempo de sistema **************
  CurrentTime = millis();
  // Controlador de activación o desactivación del modulo
  if (((CurrentTime - ButtonTime) > TimeoutButton) && (TimeoutON == true)) {
    TimeoutON = false;
    if (digitalRead(BUTTON) == 0) {
      SystemON_OFF();
    }
  }
  // Controlador de lectura del giroscopio **************
  if (((CurrentTime - MPUTime) > MPUReadTime) && (EnableModule == true)) {
    ReadMPU(float(CurrentTime - MPUTime));
    MPUTime = CurrentTime;
  }
  // Lectura de la batería y detección del cargador *************
  if (((CurrentTime - BatteryTime) > BatteryReadTime) && (EnableModule == true)) {
    BatteryVoltage = analogRead(Battery_Monitor) * Conversion * BatteryConversion;
    // Detección del cargador ***************
    if (digitalRead(CHARGER) == 1) {
      ChargerConnect = true;
    } else {
      ChargerConnect = false;
      // Modo batería baja ********************
      if (BatteryVoltage <= BatteryLow) { // Solo se comprueba batería baja si no hay cargador conectado
        SystemState = 2;          // Modo batería baja
        LowBatteryDetect = true;  // Indica que se ha detectado batería baja para que en el siguiente reinicio, el sistema responda con modo baterpia baja
        CountBeforeTurnOff = 0;   // Contador para el parpadeo del led rojo de batería baja
        EnableModule = false;     // apaga el modulo
        // Apaga los dos leds
        LedBlueState = false;
        LedRedState = false;
        digitalWrite(LED_BLUE, LedRedState);
        digitalWrite(LED_RED, LedRedState);
      }
    }
    BatteryTime = CurrentTime;
  }
  // Controlador del flujo de datos con ESP8266 **************
  if (((CurrentTime - ESPSerial) > SerialTime) && (EnableModule == true)) {
    SerialData();
    ESPSerial = CurrentTime;
  }
  // Controlador del parpadeo del led azul *************
  if (((CurrentTime - LED_BLUETime) > LEDS_TIME(false)) && (EnableModule == true)) {
    LedBlueState = !LedBlueState;
    digitalWrite(LED_BLUE, LedBlueState);
    LED_BLUETime = CurrentTime;
  }
  // Controlador del parpadeo del led rojo *************
  if (((CurrentTime - LED_REDTime) > LEDS_TIME(true)) && ((EnableModule == true) || (SystemState == 2))) {
    LedRedState = !LedRedState;
    digitalWrite(LED_RED, LedRedState);
    if (SystemState == 2) {    // Si esta en modo de batería baja se realiza el conteo antes de apagar el sistema
      CountBeforeTurnOff += 1;
      if (CountBeforeTurnOff == (BlinkBatteryLow * 2)) {    // 3*2 debido a que hay que contar tambien las veces que se apaga
        CountBeforeTurnOff = 0;
        EnableModule = true;// Solo se habilita para ejecutar el apagado general
        SystemON_OFF();
      }
    }
    LED_REDTime = CurrentTime;
  }
  // Controlador de encendido del motor **************
  if (((CurrentTime - MotorTime) > MotorTimeON) && (EnableModule == true) && (TurnOnMotor == true)) {
    digitalWrite(Vibrator, LOW);
    TurnOnMotor = false;
  }
}
