
float P = 0;
volatile float encoder = 0;
float prom = 0;
float RPS = 0;
unsigned char data;
unsigned char u;
unsigned char *pointer_RPS = (unsigned char *)&RPS;
unsigned char *pointer_P = (unsigned char *)&P;

// Variables to time operations **************************
unsigned long currentMillis = 0;
unsigned long state1 = 0;
unsigned long state2 = 0;


void setup() {
  // put your setup code here, to run once:
  //******************* Serial Comunication *****************************************************************
  Serial.begin(115200); // Used to development

  // Co*nfiguración para interrupción (Empleada para contar los pulsos del encoder) **************************
  attachInterrupt(digitalPinToInterrupt(2), Count, RISING);

  noInterrupts(); // disable all interrupts while do configure
  // ******************************* Set PWM frecuency **********************************************
  // Freq_PWM = 16Mhz /(255*8) = 7.84 Khz
  TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (0 << COM2B0) | (1 << WGM21) | (1 << WGM20);
  // Select Pin9 as output, fast PWM
  TCCR2B =  (0 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20);
  // Select Preescaler Fosc/8
  TCNT2 = 0x00; //Reset LSM of the timer
  OCR2B = 0x00; //Set LSB zero to start the system OFF, it register is used to change dute cycle

  interrupts(); // enable all interrupts

  // Salida de control para el motor
  pinMode(9, OUTPUT); //PWM output (OC2B)
}
void ReadStates()
{
  for (int i = 3; i >= 0; i--)
    Serial.write(pointer_P[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(pointer_RPS[i]);
}
void WriteControl()
{
  while (Serial.available() == 0)
  {
    u = 0;
  }
  u = Serial.read();
  OCR2B = (unsigned char)(u);
  Serial.write(OCR2B);
}
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  //Ejecución cada 333 milisegundos *****************************
  if ((unsigned long)(currentMillis - state1) >= 333) {
    Estado1();
    state1 = millis();
  }
  //Ejecucion cada 20 milisegundos *************************************
  if ((unsigned long)(currentMillis - state2) >= 20) {
    Estado2();
    state2 = millis();
  }
}
void Count()
{
  encoder += 1;

}

void Estado1()
{
  // Conversión de pulsos a revoluciones por segundo ****************
  noInterrupts();
  RPS = (encoder * 5 / 100) ;
  encoder = 0;
  interrupts();
  
  // **********************************************
  // Lectura analogica del potenciometro
  P = analogRead(A0);
}
void Estado2()
{

  if (Serial.available() > 0)
  {
    data = Serial.read();
    switch (data)
    {
      case 'A':
        Serial.write('a');
        ReadStates();
        break;
      case 'B':
        Serial.write('b');
        WriteControl();
        break;
      default:
        Serial.write('Z');
        break;

    }
  }
}
