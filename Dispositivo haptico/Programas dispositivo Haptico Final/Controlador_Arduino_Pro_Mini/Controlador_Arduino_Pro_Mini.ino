#include <Wire.h>
#include <MPU6050.h>

// Definiciones del programa
#define SerialTime 20           // Refresco del puerto serial
#define MPUReadTime 10          // Lectura del Giroscopio

#define MotorTimeON 200         // Tiempo de encendido del motor
#define BatteryReadTime 1000    // Tiempo de lectura de la batería

#define TimeoutButton 3000
#define BlinkLED_BLUE 2000      // Parpadeo led azul modo normal
#define BlinkLED_RED 200        // Parpadeo led rojo modo batería baja
#define BlinkCONNECT 200        // Parpadeo para indicar que se esta conectando el dispositivo
#define RAD_A_DEG = 57.295779   // Conversion de radianes a grados 180/PI

// Constantes del programa ******************
const int BUTTON = 2;
const int Vibrator = 10;
const int LED_RED = 11;
const int LED_BLUE = 9;
const int CHARGER = 4;
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
byte PWMData = 0;

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

  // Initialize Timer1
  // fast PWM mode with top=0xff
  // OC1B output, non inverted PWM
  // Freq_PWM = Fosc / (prescale * TOP)
  // Freq_PWM = 8Mhz / (1*256) = 31.25 KHz

  // OC1B = D10 Pin del motor para el PWM
  TCCR1A = B00100001; //COM1A1:COM1A0 = b00 | COM1B1:COM1B0 = b10 (PWM clear on Match, "Normal mode") | WGM11:WGM10 = b01
  TCCR1B = B00001001; //FOC2A,FOC2B unused on this mode; WGM13 = b0 ; WGM12 = b0 ; CS12:CS11:CS10 = b001 (Prescaler = 1);
  // WGM13:WGM12:WGM11:WGM10 = b0101 (Fast PWM 8bit mode, TOP = 0xff)

  // ClearTimer, Clear match register
  TCNT1H = 0x00;
  TCNT1L = 0x00;

  OCR1BH = 0x00;
  OCR1BL = 0x00;
  // Configuración de interrupción externa *********
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(BUTTON), ButtonISRHandler, FALLING);
  interrupts();
}
void SetPWM(byte PWM) {
  OCR1BL = (byte)PWM;
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
    //digitalWrite(Vibrator, LOW);
    SetPWM(0x00); // Apaga el PWM
    digitalWrite(LED_RED, LOW);
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
    Serial.print('O');  // Apaga el WIFI
    // Limpia el puerto serie *******
    while (Serial.available() > 0) {
      Serial.read();
    }
    mpu.setSleepEnabled(true);
  }
}
void ReadMPU(float dt) {
  dt = dt / 1000;
  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();
  // Read normalized values
  Vector normGyro = mpu.readNormalizeGyro();

  // Traslacion de los ejes para la utilización de otro plano cartesiano
  Vector translate = normAccel;
  normAccel.XAxis = translate.ZAxis;
  normAccel.ZAxis = -translate.XAxis;

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
    /*
      if ((normGyro.YAxis > 200) && (pitch < -40)) {
      if (!Go_Zero) {
        Go_Zero = true;
        detect = !detect;
      }
      }
      if ((abs(normGyro.YAxis) < 10) && (abs(pitch) < 10)) {
      Go_Zero = false;
      }
      if (detect) {
      yaw = 100;  // Indica que se ha detectado un movimiento de sujetar ****
      } else {
      yaw = 0;    // Indica que se ha detectado un movimiento de soltar *****
      }
    */

    yaw = round(normGyro.YAxis);
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
    if (!TurnOnMotor)
      SendData();
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
      GetPWM();
      TurnOnMotor = true;
      MotorTime = CurrentTime;
      SetPWM(PWMData); // Configura el PWM
      //digitalWrite(Vibrator, HIGH);
      break;
  }
}
void GetPWM() {
  while (Serial.available() == 0) {
    continue;
  }
  PWMData = (byte)Serial.read();
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
    if (!TurnOnMotor)
      ReadMPU(float(CurrentTime - MPUTime));
    MPUTime = CurrentTime;
  }
  // Lectura de la batería y detección del cargador *************
  if (((CurrentTime - BatteryTime) > BatteryReadTime) && (EnableModule == true)) {
    BatteryVoltage = analogRead(Battery_Monitor) * Conversion * BatteryConversion;
    if (digitalRead(CHARGER) == 1) {
      ChargerConnect = true;
    } else {
      ChargerConnect = false;
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
  if (((CurrentTime - LED_REDTime) > LEDS_TIME(true)) && (EnableModule == true)) {
    LedRedState = !LedRedState;
    digitalWrite(LED_RED, LedRedState);
    LED_REDTime = CurrentTime;
  }
  // Controlador de encendido del motor **************
  if (((CurrentTime - MotorTime) > MotorTimeON) && (EnableModule == true) && (TurnOnMotor == true)) {
    SetPWM(0x00);  // Apaga el PWM *****
    //digitalWrite(Vibrator, LOW);
    TurnOnMotor = false;
  }
}
