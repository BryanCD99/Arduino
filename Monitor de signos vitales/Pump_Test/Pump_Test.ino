/*
 * Connect Air punp to G and D8
 * Connect Solenoid Valve to G and D7
 * Connect the MPX5050 DP sensor to VU(Pin between A0 and G),G and A0
 */
#define REPORTING_PERIOD_MS     5000
int Button_Pump = 12;  // D6
int Valve = 13;     // D7
int Pump = 2;     // D4
int temp3 = 0;    // Push button
int Pump_Sys = 0;
uint32_t tsLastReport = 0;

const float ADC_mV = 43.22266;       // convesion multiplier from ESP ADC value to voltage in mV
const float SensorOffset = 200;     // in mV taken from datasheet
const float sensitivity = 0.88257;      // in mV/mmH2O taken from datasheet
const float mmh2O_mmGh = 0.07356;      // convesion multiplier from mmH2O to mmHg

void setup() {
  pinMode(Pump, OUTPUT);
  digitalWrite(Pump, LOW);
  pinMode(Valve, OUTPUT);
  digitalWrite(Valve, LOW);
  Serial.begin(115200);
  Serial.println("Air pump Testing"); 
}

void loop() {
  temp3 = digitalRead(Button_Pump); 
   
  if (temp3 == LOW) {
    Pump_Sys = 1;
    digitalWrite(Valve, HIGH);
    tsLastReport = millis();
  }

  if (Pump_Sys == 1){
    digitalWrite(Pump, HIGH);

    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      Pump_Sys = 0;
      digitalWrite(Pump, LOW);
      digitalWrite(Valve, LOW);
      float sensorValue = ((analogRead(A0) * ADC_mV - SensorOffset) / sensitivity * mmh2O_mmGh);
      Serial.print("Pressure: ");
      Serial.print(sensorValue, 2);
      Serial.println("mmHg");
      Serial.println("Pressure sensing complete Complete");
    }
  }
  
}
