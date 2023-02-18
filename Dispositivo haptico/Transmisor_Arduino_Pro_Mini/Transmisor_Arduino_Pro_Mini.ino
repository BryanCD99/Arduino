#include <ArduinoLowPower.h>

#include <Wire.h>
#include <MPU6050.h>

// Constantes del programa ***************************
const int BUTTON = 2;
const int VIBRATOR = 10;
const int LED_RED = 11;
const int LED_BLUE = 12;
const int CHARGER_CONNECT = 13;

const int BATTERY_MONITOR = A0;

const float ADC_CONV = 1.1 / 1023;

// Variables de uso general ***************************
float CurrentBatterVoltage = 0;
bool ON_BUTTON_STATE = 0;

// Variables de control de ejecución de programa ******
volatile unsigned long CurrentTime = 0;
volatile unsigned long ButtonTime = 0;

void setup() {
  // Configuración puerto serial **********************
  Serial.begin(115200);

  // Configuración de entradas y salidas **************
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(VIBRATOR, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(CHARGER_CONNECT, INPUT);

  // Estado inicial de las salidas ********************
  digitalWrite(VIBRATOR, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);

  // Configuración del ADC ****************************
  analogReference(INTERNAL);

  // Interrpción externa para el boton ****************
  attachInterrupt(digitalPinToInterrupt(BUTTON), ButtonHandler, FALLING);

}
void ButtonHandler(){
  ButtonTime = CurrentTime;
}
void ReadBatteryVoltage()
{
  CurrentBatterVoltage = analogRead(BATTERY_MONITOR) * ADC_CONV;
}
void loop() {
  // put your main code here, to run repeatedly:
  CurrentTime = millis();
}
