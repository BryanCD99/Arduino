#include <Servo.h>

#define DeltaTime 0.1f    // Define el tiempo minimo por iteracion para el calculo de la ecuacion diferencial


#define LEDArbol1 2       // Led verde 1
#define LEDArbol2 3       // Led verde 2
#define LEDArbol3 4       // Led verde 3
#define LEDArbol4 5       // Led verde 4

#define LAMP1 6
#define LAMP2 7

#define SerialRefresh 10
#define ServoRefresh 15

#define MaxServo 70
#define MinServo 0
#define MaquetaAlto 10.0f
#define ArbolReal 50.0f


// Variables de uso general
bool Move = false;
bool LampState = true;
int angle = 0;
int LightPos = 0;

// Variables para el calculo numerico de la ecuacion diferencial
volatile float I_Accumulator;    // Memoria de la integral
volatile float H;                // Altura actual
volatile float dh;               // Razon de cambio de la altura
volatile float Hf;               // Altura final

// Variables que definen el comportamiento de la ecuación diferencial ****
volatile float alpha = 0;
volatile float betha = 0;
volatile float gamma = 0;
volatile float H0 = 0;             // Altura inicial del arbol
volatile float T = 0;              // Tiempo de crecimiento total


// Variables para la comunicación con el ordenador ***********************
unsigned char *alphaData = (unsigned char *)&alpha;
unsigned char *bethaData = (unsigned char *)&betha;
unsigned char *gammaData = (unsigned char *)&gamma;
unsigned char *H0Data = (unsigned char *)&H0;
unsigned char *TData = (unsigned char *)&T;
unsigned char *HData = (unsigned char *)&H;

byte BufferSerialData[4];

// Variables de control de tiempo
unsigned long CurrentTime = 0;
unsigned long ServoTime = 0;
unsigned long SerialTime = 0;
unsigned long LightTime = 0;
unsigned long LigthRefresh = 200;

Servo ServoTree;  // create servo object to control a servo

void SetServo(int Angle) {
  if (Angle < MinServo) {
    Angle = MinServo;
  }
  if (Angle > MaxServo) {
    Angle = MaxServo;
  }
  ServoTree.write(Angle);
}
// Funcion de union para la conversión de array de 4 bytes a float32
union u_tag {
  byte b[4];
  float fval;
} u;
// Obtiene los bytes para ajustar la ecuación diferencial desde el computador
float GetPCData() {
  for (int i = 3; i >= 0; i--)
  {
    BufferSerialData[i] = (byte)Serial.read();
  }
  u.b[3] = BufferSerialData[3];
  u.b[2] = BufferSerialData[2];
  u.b[1] = BufferSerialData[1];
  u.b[0] = BufferSerialData[0];
  return u.fval;
}
// Realiza los calculos de la ecuacion diferencial para la edad establecida del arbol
float Estimator() {
  H = H0;
  I_Accumulator = H0;
  for (float t = 0; t < T; t += DeltaTime) {
    Send2PC();
    dh = pow(H, betha) * alpha - gamma * H;
    H = Integrator(dh, DeltaTime);
  }
  return H;
}

void Send2PC() {
  for (int i = 0; i <= 3; i++)
    Serial.write(HData[i]);
}
// Funcion de calculo de la integral *********
float Integrator(float Input, float Time) {
  I_Accumulator += Input * Time;
  return I_Accumulator;
}
// Funcion para la comunicación con el ordenador *********
void SerialHalndler() {
  if (Serial.available() > 0)
  {
    char data = Serial.read(); // Leer el dato del puerto serial
    switch (data)
    {
      case 'A':
        // Espera 20 bytes correspondientes a la configuración de la ecuación
        while (Serial.available() < 20) {
          continue;
        }
        alpha = GetPCData();    // Obtiene el alpha
        betha = GetPCData();    // Obtiene el betha
        gamma = GetPCData();    // Obtiene el gamma
        H0 = GetPCData();       // Obtiene la altura inicial
        T = GetPCData();        // Obtiene el tiempo de crecimiento
        break;
      case 'B':
        Hf = Estimator();
        angle = (int)(asin(Hf / ArbolReal) * 180 / PI);
        digitalWrite(LEDArbol1, HIGH);
        digitalWrite(LEDArbol2, HIGH);
        digitalWrite(LEDArbol3, HIGH);
        digitalWrite(LEDArbol4, HIGH);
        delay(250);
        digitalWrite(LEDArbol1, LOW);
        digitalWrite(LEDArbol2, LOW);
        digitalWrite(LEDArbol3, LOW);
        digitalWrite(LEDArbol4, LOW);
        delay(250);
        digitalWrite(LEDArbol1, HIGH);
        digitalWrite(LEDArbol2, HIGH);
        digitalWrite(LEDArbol3, HIGH);
        digitalWrite(LEDArbol4, HIGH);
        delay(250);
        digitalWrite(LEDArbol1, LOW);
        digitalWrite(LEDArbol2, LOW);
        digitalWrite(LEDArbol3, LOW);
        digitalWrite(LEDArbol4, LOW);
        delay(250);
        Move = true;
        break;
      case 'C':
        LampState = !LampState;
        digitalWrite(LAMP1, LampState);
        digitalWrite(LAMP2, LampState);
        break;
    }
  }
}

void setup() {
  // Configuracion de la comunicación con el ordenador ************
  Serial.begin(115200);

  // Configura los pines de entrada y salida
  pinMode(LEDArbol1, OUTPUT);
  pinMode(LEDArbol2, OUTPUT);
  pinMode(LEDArbol3, OUTPUT);
  pinMode(LEDArbol4, OUTPUT);
  pinMode(LAMP1, OUTPUT);
  pinMode(LAMP2, OUTPUT);

  // Configuracion inicial de las salidas
  digitalWrite(LEDArbol1, HIGH);
  digitalWrite(LEDArbol2, HIGH);
  digitalWrite(LEDArbol3, HIGH);
  digitalWrite(LEDArbol4, HIGH);
  digitalWrite(LAMP1, LampState);
  digitalWrite(LAMP2, LampState);

  // Configura el servo para el control de la altura del arbol
  ServoTree.attach(9);  // attaches the servo on pin 9 to the servo object
  ServoTree.write(0);

}

void loop() {
  // Actualización de tiempo del sistema
  CurrentTime = millis();
  // Envio y recepción de datos a consola serial *************************
  if ((CurrentTime - SerialTime) >= SerialRefresh)
  {
    SerialHalndler();
    SerialTime = CurrentTime;
  }

  if ((CurrentTime - ServoTime) >= ServoRefresh)
  {
    if (Move) {
      int CurrentServo = ServoTree.read();
      if (abs(angle - CurrentServo) > 0) {
        if ((angle - CurrentServo) > 0) {
          SetServo(CurrentServo + 1);
        } else {
          SetServo(CurrentServo - 1);
        }
      } else {
        Move = false;
      }
    }
    ServoTime = CurrentTime;
  }
  if ((CurrentTime - LightTime) >= LigthRefresh)
  {
    switch (LightPos) {
      case 0:
        digitalWrite(LEDArbol1, HIGH);
        digitalWrite(LEDArbol2, LOW);
        digitalWrite(LEDArbol3, LOW);
        digitalWrite(LEDArbol4, LOW);
        break;
      case 1:
        digitalWrite(LEDArbol1, LOW);
        digitalWrite(LEDArbol2, HIGH);
        digitalWrite(LEDArbol3, LOW);
        digitalWrite(LEDArbol4, LOW);
        break;
      case 2:
        digitalWrite(LEDArbol1, LOW);
        digitalWrite(LEDArbol2, LOW);
        digitalWrite(LEDArbol3, HIGH);
        digitalWrite(LEDArbol4, LOW);
        break;
      case 3:
        digitalWrite(LEDArbol1, LOW);
        digitalWrite(LEDArbol2, LOW);
        digitalWrite(LEDArbol3, LOW);
        digitalWrite(LEDArbol4, HIGH);
        break;
    }
    if (LightPos < 3) {
      LightPos ++;
    } else {
      LightPos = 0;
    }
    LightTime = CurrentTime;
  }

}
