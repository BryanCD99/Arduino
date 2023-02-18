#include "Controladores.h"

// Constantes del programa ******************************
float const SpeedConversionEncoder = 45;    // Factor de convercion de frecuencia a RPS para el encoder
int const uPin = 10;                        // Pin de salida del motor
int const LED = 4;                          // Pin del LED            
int const EncoderPin = 2;                   // Pin de entrada para el encoder

// Constantes de control P discreto
const float P_Kp = 70;
const unsigned long P_T = 300; //Tiempo de ejecución, Tiempo de muestreo*** Milisegundos
const float TP = P_T / 1000;

// Constantes de PI discreto **********
const float PI_Kp = 30;
const float PI_Ki = 50;
const unsigned long PI_T = 300; //Tiempo de ejecución, Tiempo de muestreo*** Milisegundos
const float TPI = PI_T / 1000;

// Constantes de PID discreto **********
const float PID_Kp = 30;
const float PID_Ki = 50;
const float PID_Kd = 0.01;
const unsigned long PID_T = 300; //Tiempo de ejecución, Tiempo de muestreo*** Milisegundos
const float TPID = PID_T / 1000;

// Variables de uso general ****************
volatile int Count = 0;
float SpeedEncoder;
float SetPoint = 0;
byte BufferSerialData[4];
unsigned char *SpeedEncoderData = (unsigned char *)&SpeedEncoder;
int Control = 0;

// Variables de control de ejecución
unsigned long CurrentTime = 0;
unsigned long Read_EncoderTime = 0;
unsigned long Control_Time = 0;
unsigned long SerialConsole = 0;
unsigned long Control_Time_Set = 0;

// Crea controladores Proporcional, Proporcional Integral y PID ******************
Controladores ControlProporcional, ControlPI, ControlPID;

// Funcion de union para la conversión de array de 4 bytes a float32
union u_tag {
  byte b[4];
  float fval;
} u;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Configuración de entradas y salidas
  pinMode(uPin, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(EncoderPin, INPUT);
  digitalWrite(uPin, LOW);
  digitalWrite(LED, LOW);

  noInterrupts(); // disable all interrupts

  // Initialize Timer2
  // fast PWM mode with top=0xff
  // OC2B output, non inverted PWM
  // Freq_PWM = Fosc / (prescale * TOP)
  // Freq_PWM = 16Mhz / (1*256) = 62.5 KHz

  TCCR2A = B10000011; //COM1A1:COM1A0 = b00 | COM1B1:COM1B0 = b10 (PWM clear on Match, "Normal mode") | WGM21:WGM20 = b11
  TCCR2B = B00000010; //FOC2A,FOC2B unused on this mode; WGM22 = b0 ; CS12:CS11:CS10 = b001 (Prescaler = 1);
  // WGM22:WGM21:WGM20 = b011 (Fast PWM 8bit mode, TOP = 0xff)

  // ClearTimer, Clear match register
  TCNT2 = 0x00;
  OCR2A = 0x00;

  // Establece las constantes de los controladores para el control del motor
  ControlProporcional.Constants(P_Kp, 0, 0, TP);
  ControlPI.Constants(PI_Kp, PI_Ki, 0, TPI);
  ControlPID.Constants(PID_Kp, PID_Ki, PID_Kd, TPID);
  Control_Time_Set = PI_T;
  attachInterrupt(digitalPinToInterrupt(EncoderPin), Encoder, CHANGE);
  interrupts(); // enable all interrupts
}

void SetSetPoint()
{
  while (Serial.available() < 4) {
    continue;
  }
  for (int i = 3; i >= 0; i--)
  {
    BufferSerialData[i] = (byte)Serial.read();
  }
  u.b[3] = BufferSerialData[3];
  u.b[2] = BufferSerialData[2];
  u.b[1] = BufferSerialData[1];
  u.b[0] = BufferSerialData[0];
  SetPoint = u.fval;
  Serial.write('S');
}
void ReadData()
{
  for (int i = 3; i >= 0; i--)
    Serial.write(SpeedEncoderData[i]);
}

void Motor(int Control)
{
  OCR2A = (byte)(Control);
}
void SerialHandler()
{
  char data;
  if (Serial.available() > 0)
  {
    data = Serial.read();
    switch (data)
    {
      case 'S':
        SetSetPoint();
        break;
      case 'R':
        ReadData();
        break;
      case 'P':
        Control_Time_Set =  P_T;
        Control = 1;
        break;
      case 'I':
        Control_Time_Set =  PI_T;
        Control = 0;
        break;
      case 'D':
        Control_Time_Set =  PID_T;
        Control = 2;
        break;
    }
  }
}

void Encoder()
{
  Count ++;
  digitalWrite(LED, digitalRead(2));
}
void Read_Encoder()
{
  SpeedEncoder = (float)(Count / SpeedConversionEncoder);
  Count = 0;
}
void loop() {
  // put your main code here, to run repeatedly:
  CurrentTime = millis();

  // Envio y recepción de datos a consola serial *************************
  if ((CurrentTime - SerialConsole) >= 20)
  {
    SerialHandler();
    SerialConsole = CurrentTime;
  }
  // Lectura del encoder ******************
  if ((CurrentTime - Read_EncoderTime) >= 300)
  {
    Read_Encoder();
    Read_EncoderTime = CurrentTime;
  }

  // Control de motor ******************
  if ((CurrentTime - Control_Time) >= Control_Time_Set)
  {
    if (Control == 0)
    {
      Motor(ControlPI.Get_PI(SetPoint, SpeedEncoder));
    } else if (Control == 1)
    {
      Motor(ControlProporcional.Get_P(SetPoint, SpeedEncoder));
    } else if (Control == 2)
    {
      Motor(ControlPID.Get_P(SetPoint, SpeedEncoder));
    }
    Control_Time = CurrentTime;
  }
}
