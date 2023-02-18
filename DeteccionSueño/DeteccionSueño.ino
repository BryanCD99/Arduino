#define LED 2      // Led de activación
#define Buzzer 12  // Buzzer

#define SerialRefresh 10
#define PulsometroRefresh 20

// Variables de uso general
bool EnableAlarm = false;        // Variable de activacion de la alarma
volatile float Pulsometro = 60;  // Variable que almacena el valor del pulsometro
volatile float Send = 0;         // Variable que sirve de buffer para enviar datos tipo float al PC
int pulsePin = 2;                // Sensor de Pulso conectado al puerto A2

// Estas variables son volatiles porque son usadas durante la rutina de interrupcion en la segunda Pestaña
volatile int BPM;                // Pulsaciones por minuto
volatile int Signal;             // Entrada de datos del sensor de pulsos
volatile int IBI = 600;          // tiempo entre pulsaciones
volatile boolean Pulse = false;  // Verdadero cuando la onda de pulsos es alta, falso cuando es Baja
volatile boolean QS = false;     // Verdadero cuando el Arduino Busca un pulso del Corazon

// Variables para la comunicación con el ordenador ***********************
unsigned char *SenData = (unsigned char *)&Send;

// Variables de control de tiempo
unsigned long CurrentTime = 0;
unsigned long SerialTime = 0;
unsigned long PulsTime = 0;
unsigned long AlarmPeriod = 0;
unsigned long AlarmTime = 0;

// Funcion de union para la conversión de array de 4 bytes a float32
union u_tag {
  byte b[4];
  float fval;
} u;
// Obtiene los bytes para obtener los datos del computador
float GetPCData() {
  for (int i = 3; i >= 0; i--) {
    u.b[i] = (byte)Serial.read();
  }
  return u.fval;
}
void Send2PC(float Data) {
  Send = Data;
  for (int i = 3; i >= 0; i--)
    Serial.write(SenData[i]);
}
void SerialHalndler() {
  if (Serial.available() > 0) {
    char data = Serial.read();  // Leer el dato del puerto serial
    switch (data) {
      case 'A':
        // Espera a que llegue el tiempo de encendido de la alarma
        while (Serial.available() < 4) {
          continue;
        }
        // Obtiene el tiempo de activacion de la alarma
        AlarmPeriod = GetPCData();
        // Reinicia el contador de tiempo de la alarma
         AlarmTime = CurrentTime;
        // Activa la alarma
        EnableAlarm = true;
        // Enciende el LED
        digitalWrite(LED, HIGH);
        //tone(Buzzer, 2000);
        OCR1BH = 0x0F;
        OCR1BL = 0x9F;
        break;
      case 'B':
        // Envia los datos del pulsometro a la computadora
        Send2PC(Pulsometro);
        break;
    }
  }
}
void setup() {
  // Configuracion de la comunicación con el ordenador ************
  Serial.begin(115200);
  // Configuración de entradas y salidas
  pinMode(LED, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  // Estado inicial de las salidas
  digitalWrite(LED, LOW);
  digitalWrite(Buzzer, LOW);

  // Initialize Timer1
  // fast PWM mode with top=ICR1=8000
  // OC1B output, non inverted PWM
  // Freq_PWM = Fosc / (prescale * TOP)
  // Freq_PWM = 16Mhz / (1*8000) = 2KHz

  // OC1B = D10 Pin del motor para el PWM
  TCCR1A = B00100010;  //COM1A1:COM1A0 = b00 | COM1B1:COM1B0 = b10 (PWM clear on Match, "Normal mode") | WGM11:WGM10 = b10
  TCCR1B = B00011001;  //FOC2A,FOC2B unused on this mode; WGM13:WGM12 = b11 ; CS12:CS11:CS10 = b001 (Prescaler = 1);
  // WGM13:WGM12:WGM11:WGM10 = b0101 (Fast PWM 8bit mode, TOP = 0xff)

  // ClearTimer, Clear match register
  TCNT1H = 0x00;
  TCNT1L = 0x00;

  OCR1BH = 0x00;
  OCR1BL = 0x00;

  // Configura PWM a 2KHz (Sonido de frecuencia de 2KHz)
  ICR1H = 0x1F;
  ICR1L = 0x3F;

  interruptSetup();
}

void loop() {
  CurrentTime = millis();
  // Envio y recepción de datos a consola serial *************************
  if ((CurrentTime - SerialTime) >= SerialRefresh) {
    SerialHalndler();
    SerialTime = CurrentTime;
  }

  // Lectura del pulsometro
  if ((CurrentTime - PulsTime) >= PulsometroRefresh) {
    PulsTime = CurrentTime;
  }

  // Control de la desactivación de la alarma
  if (EnableAlarm) {
    if ((CurrentTime - AlarmTime) >= AlarmPeriod) {
      EnableAlarm = false;
      digitalWrite(LED, LOW);
      //noTone(Buzzer);
      OCR1BH = 0x00;
      OCR1BL = 0x00;
    }
  } else {
    AlarmTime = CurrentTime;
  }
  //Serial.print(Pulsometro); //Habilitar estas linea para ver BPM en el monitor serial pero deshabilitar la siguiente
  //Serial.print(50);
  //Serial.print(",");
  //Serial.print(Signal);  // envia el valor del pulso por el puerto serie  (desabilitarla si habilita la anterior linea)
  //Serial.print(",");
  //Serial.println(-50);
  if (QS == true) {  // Bandera del Quantified Self es verdadera cuando el Arduino busca un pulso del corazon
    if (!(BPM > 130 || BPM < 0))
      Pulsometro = (float)BPM;
    QS = false;  // Reset a la bandera del Quantified Self
  }
}