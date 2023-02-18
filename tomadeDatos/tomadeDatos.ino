int lm35 = A1;
int foco = 9; // Pin 9 corresponde a OCR2B
int ventilador = 8; // Ventilador
float temperatura = 0;
unsigned char *pointer_temperatura = (unsigned char *)&temperatura;
unsigned char data;


void setup() {
  // put your setup code here, to run once:
  // CONFIGURO PUERTOS
  pinMode(lm35, INPUT);
  pinMode(foco, OUTPUT);
  pinMode(ventilador, OUTPUT);
  digitalWrite(ventilador,LOW);
  //CONFIGURO LA CONEXION SERIAL
  Serial.begin(9600);

  // Changing frecuency of the PWM *****
  noInterrupts(); // disable all interrupts while do configure

  // ******************************* Set PWM frecuency **********************************************
  // Freq_PWM = 16Mhz /(255*8) = 7.84 Khz
  TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (0 << COM2B0) | (1 << WGM21) | (1 << WGM20);
  TCCR2B =  (0 << WGM22) | (0 << CS22) | (0 << CS21) | (1 << CS20);
  // Select Preescaler Fosc/8
  TCNT2 = 0x00; //Reset LSM of the timer
  OCR2B = 0x00; //it register is used to change dute cycle
 
  interrupts(); // enable all interrupts
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    data = Serial.read();
    switch (data)
    {
      case 'S':
        Serial.write('s');
        Sensor();
        break;
      case 'F':
        Serial.write('f');
        LightControl();
        break;
      case 'V':
        Serial.write('v');
        VentiladorControl();
        break;
    }
  }
}
void Sensor()
{
  //realizo lectura del lm35
  temperatura = analogRead(lm35);
  //pausa para estabilizar medicion
  //envio lectura al puerto serie
  temperatura = ((temperatura * 5000) / 1023) / 10;
  for (int i = 3; i >= 0; i--)
    Serial.write(pointer_temperatura[i]);
}
void LightControl()
{
  // Espera a que llegue el control desde Matlab
  while (Serial.available() == 0)
  {
    data = ' ';
  }
  data = Serial.read();
  // Escribe el dato 
  OCR2B = (unsigned char)(data); // Equivale a AnalogWrite
  Serial.write(OCR2B);
}
void VentiladorControl()
{
  while (Serial.available() == 0)
  {
    data = ' ';
  }
  data = Serial.read();
  switch (data)
    {
      case 'E':
        Serial.write('e');
        digitalWrite(ventilador,HIGH);
        break;
      case 'A':
        Serial.write('a');
        digitalWrite(ventilador,LOW);
        break;
    }
}
