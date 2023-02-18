
#define SensorPuls 2          // Entrada del sensor de posicion del motor
#define SensorMax 3           // Entrada del sensor de cerrado del techo
#define EnvironmentLigth 5    // Salida para el control de las luces
#define MotorA 6              // Control de giro para cerrar el techo
#define MotorB 7              // Control de giro para abrir el techo
#define Pump 10               // Control de velocidad de la bomba(OC1B)
#define MotorSpeed 9          // Control de velocidad del motor(OC1A)

#define PulsMax 25            // Pulsos correspondientes a la apertura maxima del techo

#define SerialRefresh 10      // Tiempo de refresco para el procesamiento de la comunicacion con el ordenador en mS
#define MotorRefresh 10       // Tiempo de refresco para el control del motor en mS

#define RiegoMin 5            // Tiempo minimo real de riego en segundos
#define RiegoMax 20           // Tiempo maximo real de riego en segundos
#define TiempoMin 1           // Tiempo minimo de riego en la simulación en min
#define TiempoMax 120         // Tiempo maximo de riego en la simulacion en min
#define Const1 (RiegoMax-RiegoMin)      // Constante 1 para el calculo del riego
#define Const2 (TiempoMax-TiempoMin)    // Constante 2 para el calculo del riego

// Variables de uso general
volatile bool Move = false;                   // activa el motor
volatile bool Direction = false;     // Indica la direccion a la que esta girando el motor
bool Home = false;                   // Indica que hay que ir a la posición de inicio
volatile int PulsCount = 0;          // Pulsos de la posicion del motor
volatile int MotorPosition = 0;      // Posición deseada del motor de la persiana
byte SpeedMotor = 160;               // Velocidad del motor (SpeedMotor = 0, Velocidad 0%; SpeedMotor = 255, Velocidad 100%
byte SpeedPump = 70;                // Velocidad de la bomba (SpeedPump = 0, Fuerza 0%; SpeedPump = 255, Fuerza 100%
bool PumpEnable = false;
byte LightDimmer = 255;              // Intensidad de la luz
byte BufferSerialData[4];            // Obtiene los datos del puerto serial

// Variables de control de tiempo
unsigned long CurrentTime = 0;
unsigned long SerialTime = 0;
unsigned long MotorTime = 0;
unsigned long RiegoTime = 0;
unsigned long PumpTime = 0;

// Funcion de union para la conversión de array de 4 bytes a float32
union u_tag {
  byte b[4];
  float fval;
} u;
// Obtiene los bytes de la informacion enviada desde el computador
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
// Controla el movimiento del motor de la persiana, llevandola a la posicion deseada
void MoveMotor() {
  if (Move) {
    // Recoge la persiana. Si se toca el sensor se detiene
    if (((MotorPosition - PulsCount) < 0) || Home) {
      Direction = true;       // True: Disminuir la cuenta de pulsos
      // Recoge la persiana del techo
      digitalWrite(MotorA, HIGH);
      digitalWrite(MotorB, LOW);
      OCR1AL = SpeedMotor;
      // Si se toca el sensor de tope, entonces se detiene el motor
      if (digitalRead(SensorMax) == 0) {
        Move = false;
        Home = false;
        PulsCount = 0;
        digitalWrite(MotorA, LOW);
        digitalWrite(MotorB, LOW);
        OCR1AL = 0;
      }
    }
    // Expande la persiana, si se llega al maximo se detiene
    if (((MotorPosition - PulsCount) > 0) && !Home) {
      Direction = false;      // False: Incrementar la cuenta de pulsos
      // Expande la persiana del techo
      digitalWrite(MotorA, LOW);
      digitalWrite(MotorB, HIGH);
      OCR1AL = SpeedMotor;
      // Si se alcanza la posición máxima, se detiene el motor
      if (PulsCount >= PulsMax) {
        Move = false;
        digitalWrite(MotorA, LOW);
        digitalWrite(MotorB, LOW);
        OCR1AL = 0;
      }
    }
    // Si se llega a la posicion deseada se detiene
    if (((PulsCount - MotorPosition) == 0) && !Home) {
      Move = false;
      digitalWrite(MotorA, LOW);
      digitalWrite(MotorB, LOW);
      OCR1AL = 0;
    }
  }
}
// Funcion para la comunicación con el ordenador *********
void SerialHalndler() {
  if (Serial.available() > 0)
  {
    char data = Serial.read(); // Leer el dato del puerto serial
    switch (data)
    {
      case 'A':   // Envia la persiana a una posicion deseada
        while (Serial.available() < 4) {
          continue;
        }
        MotorPosition = (int)((GetPCData() / 100) * PulsMax ); // Convierte un valor entre 0 a 100 a la posicion adecuada del motor
        if (MotorPosition == 0) {
          Home = true;
        } else {
          Home = false;
        }
        Move = true;
        break;
      case 'B':   // Controla el tiempo de encendido de la bomba
        while (Serial.available() < 4) {
          continue;
        }
        RiegoTime = (unsigned long)(RiegoMin + (GetPCData() - TiempoMin) * Const1 / Const2) * 1000;
        OCR1BL = SpeedPump;
        PumpEnable = true;
        break;
      case 'C':   // Controla la intensidad de las luces del invernadero
        while (Serial.available() < 1) {
          continue;
        }
        LightDimmer = (byte)Serial.read();
        analogWrite(EnvironmentLigth, LightDimmer);
        break;
    }
  }
}

// Cuenta pulsos para conocer la posicion del motor
void PositionMotor() {
  if (Move) {
    if (Direction) {
      PulsCount --;
    } else {
      PulsCount ++;
    }
  }
}

// Envia a la posicion de inicio
void GoStart() {
  Home = true;
  Move = true;
}
void setup() {
  // Configuracion de la comunicación con el ordenador ************
  Serial.begin(115200);

  // Configura los pines de entrada y salida
  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);
  pinMode(EnvironmentLigth, OUTPUT);
  pinMode(Pump, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);

  pinMode(SensorPuls, INPUT);
  pinMode(SensorMax, INPUT);

  //Configuración inicial de las salidas
  digitalWrite(EnvironmentLigth, LOW);
  digitalWrite(MotorA, LOW);
  digitalWrite(MotorB, LOW);
  digitalWrite(Pump, HIGH);
  digitalWrite(MotorSpeed, LOW);

  // Initialize Timer1
  // fast PWM mode with top=0xff
  // OC1B, OC2B output, non inverted PWM
  // Freq_PWM = Fosc / (prescale * TOP)
  // Freq_PWM = 16Mhz / (1*256) = 62.5 KHz

  // OC1B = D10 Pin del motor para el PWM
  TCCR1A = B10110001; //COM1A1:COM1A0 = b10 | COM1B1:COM1B1 = b11 (PWM clear on Match, "Normal mode") | WGM11:WGM10 = b01
  TCCR1B = B00001001; //FOC2A,FOC2B unused on this mode; WGM13:WGM12 = b01 ; CS12:CS11:CS10 = b001 (Prescaler = 1);
  // WGM13:WGM12:WGM11:WGM10 = b0101 (Fast PWM 8bit mode, TOP = 0xff)

  // ClearTimer, Clear match register
  TCNT1H = 0x00;
  TCNT1L = 0x00;

  OCR1BH = 0x00;
  OCR1BL = 0x00;

  OCR1AH = 0x00;
  OCR1AL = 0x00;

  // Interrupciones para el manejo de la velocidad del motor
  attachInterrupt(digitalPinToInterrupt(SensorPuls), PositionMotor, RISING);

  // Se envia a la posicion de inicio el techo (Cerrar techo)
  GoStart();
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

  // Controla el movimiento del motor de la persiana del techo *************
  if ((CurrentTime - MotorTime) >= MotorRefresh)
  {
    MoveMotor();
    MotorTime = CurrentTime;
  }
  // Controla la activación de la bomba *************
  if (PumpEnable) {
    if ((CurrentTime - PumpTime) >= RiegoTime)
    {
      OCR1BL = 0;
      PumpEnable = false;
    }
  } else {
    PumpTime = CurrentTime;
  }
}
