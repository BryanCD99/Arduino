
float P = 0;
float LM35 = 0;
unsigned char data;

unsigned char *pointer_LM35 = (unsigned char *)&LM35;
unsigned char *pointer_P = (unsigned char *)&P;

// Variables to time operations **************************
unsigned long currentMillis = 0;
unsigned long state1 = 0;
unsigned long state2 = 0;


void setup() {
  // put your setup code here, to run once:
  
  //******************* Serial Comunication *****************************************************************
  Serial.begin(115200); // Used to development
  //******************** Set all inputs and outputs ********************************************************
  pinMode(9, OUTPUT); //Fan
  pinMode(10, OUTPUT); //Light
  //******************** Set and Clear initial ***************************************************
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  //******************** Changing the internal reference *****************************************
  analogReference(INTERNAL1V1);
  
  noInterrupts(); // disable all interrupts while do configure
  
  // ******************************* Set PWM frecuency **********************************************
  // Freq_PWM = 16Mhz /(255*8) = 7.84 Khz
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (0 << COM2B0) | (1 << WGM21) | (1 << WGM20);
  TCCR2B =  (0 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20);
  // Select Preescaler Fosc/8
  TCNT2 = 0x00; //Reset LSM of the timer
  OCR2B = 0x00; //it register is used to change dute cycle
  OCR2A = 0x00; //it register is used to change dute cycle

  interrupts(); // enable all interrupts
}
void Sensors()
{
  for (int i = 3; i >= 0; i--)
    Serial.write(pointer_P[i]);
  for (int i = 3; i >= 0; i--)
    Serial.write(pointer_LM35[i]);
}
void FanControl()
{
  Input();
  OCR2B = (unsigned char)(data);
  Serial.write(OCR2B);
}
void LightControl()
{
  Input();
  OCR2A = (unsigned char)(data);
  Serial.write(OCR2A);
}
void Input()
{
  while (Serial.available() == 0)
  {
    data = ' ';
  }
  data = Serial.read();
}
void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  //Ejecución cada 500 milisegundos *****************************
  if ((unsigned long)(currentMillis - state1) >= 500) {
    Estado1();
    state1 = millis();
  }
  //Ejecucion cada 20 milisegundos *************************************
  if ((unsigned long)(currentMillis - state2) >= 20) {
    Estado2();
    state2 = millis();
  }
}
void Estado1()
{
  // **********************************************
  // Lectura analogica del potenciometro
  P = analogRead(A0);
  delay(1);
  // Lectura analogica del LM35 ****************
  LM35 = analogRead(A1);
  LM35 = LM35/10;
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
        Sensors();
        break;
      case 'B':
        Serial.write('b');
        LightControl();
        break;
      case 'C':
        Serial.write('c');
        FanControl();
        break;
      default:
        Serial.write('Z');
        break;

    }
  }
}
