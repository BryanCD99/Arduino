//Regulador de carga solar
//Performed by Bryan Steven Cañarte Delgado
//*************************** Change these values at base of your battery *****************************
#define VoltageR 14.4f                // Voltage absorcion-bulk (V)
#define VoltageFloat 13.7f            // Voltage float battery  (V)
#define VoltageR24 29.6f              // Voltage absorcion-bulk (V)
#define VoltageFloat24 27.4f          // Voltage float battery  (V)
#define VoltageMinBattery 12.2f       // Voltage minimal to desconect load  (V)
#define VoltageMinBattery24 23.7f     // Voltage minimal to desconect load  (V)
#define limCurrent 2.0f               // Define limite current  (A)
#define minCurrent 0.2f               // Minimal current to change to float mode  (A)

// Setup of mainboard **********************************************************
#define minDVoltage 2.0f            // Define a minimal delta Voltage to Regulate
#define changeMode 0.7f             // Minimal delta Voltage to change mode
#define convertion 0.09191f         // Conversion factor
#define VinC 10.0f                  // Conversion to input voltage
#define CurrentConv 0.0302f         // Current by unit of ADC


// Definition of inputs and outputs ¨********************
const int uPin = 9;    // Control voltage, this is the pin called OC1A
const int LC = 2;      // Load connect pin
const int RL = 3;      // Red-Led
const int GL = 4;      // Green-Led
const int YL = 5;      // Yellow-Led
const int BP = 6;      // Battery Protection
const int x0Pin = A0;  // Voltage at the battery
const int x1Pin = A1;  // Current sensor
const int x2Pin = A2;  // Voltage input

// Variables to signal feedback *************************
float FB = 0; // Feedback signal
float RV = 0; // Reference Voltage
float BA = 0; // Bulk-Absorbcion volatge
float VF = 0; // Float Voltage
float VO = 0; // Voltage to turn off the load
float VI = 0; // Voltage input
float VM = 0; // Voltage Min
float VD = 0; // Delta Voltage ChangeMode
float CI = 0; // Current data
float CM = 0; // Current measure

// Variables to control Output **************************
int e = 0;              // Error variable
float Switch_cycle = 0; // Dute Cycle
float eL = 0;           // Current error limit

// Variables to time operations **************************
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long state1 = 0;
unsigned long state2 = 0;
unsigned long state3 = 0;

//  Variables to general use
bool VoltageRegulateEnable = false;
bool FloatBattery = false;
bool loadOff = false;

String dato = "";
String dato1 = "";
String dato2 = "";

void setup() {
  // put your setup code here, to run once:
  //******************** Set all inputs and outputs **********************************************************
  pinMode(uPin, OUTPUT);                            // Set pin OC1A as output
  pinMode(LC, OUTPUT);                              // Set Pin 2 as output
  pinMode(RL, OUTPUT);                              // Set Red Led as output
  pinMode(GL, OUTPUT);                              // Set Green Led as output
  pinMode(YL, OUTPUT);                              // Set Yellow Led as output
  pinMode(BP, OUTPUT);                              // Set Output Battery as output
  //******************** Set and Clear initial ***************************************************
  digitalWrite(LC, LOW);                            // Turn off load when system started
  digitalWrite(RL, LOW);                            // Turn off Red Led
  digitalWrite(GL, LOW);                            // Turn off Green Led
  digitalWrite(YL, LOW);                            // Turn off Yellow Led
  digitalWrite(BP, LOW);                            // Turn off battery protection
  //******************* Serial Comunication *****************************************************************
  Serial.begin(115200); // Used to development
  noInterrupts(); // disable all interrupts while do configure
  // ******************************* Set PWM frecuency **********************************************
  // Freq_PWM = 16Mhz /256 = 125 Khz
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (0 << WGM10);
  //       COM1A1 = 1  ; COM1A0 = 0 ; COM1B1 = 0  ; COM1B0 = 0  ;         Note: When use a inverted output, you must set COM1A1 & COM1A0, if you use a normal output, you only have to must COM1A1
  //       Clear output OC1A when match happen, Output OC1B disconnected
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);

  //        ICNC1 & ICES1 are used when Timer had a external signal. Unused in Fast PWM mode
  //        CS12 = 0; CS11 = 0; CS10 = 0; Select no prescaler
  //        WGM13 = 1 ; WGM12 = 1;  WGM11 = 1 ; WGM10 = 0; Select Fast 8 bit PWM with TOP value on ICR1

  ICR1H  = 0x00;  //Set ICR1 register to 80h (#128) MSB = 0x00, LSB = 80h
  ICR1L  = 0x80;  //This value is TOP of the counter. Timer reach to zero when count is equal to it.

  TCNT1H = 0x00; //Reset MSB of the timer
  TCNT1L = 0x00; //Reset LSM of the timer
  OCR1AH = 0x00; //Clear MSB of the match register, unused in Fast PWM 8 bit
  OCR1AL = 0x00; //Set LSB zero to start the system OFF, it register is used to change dute cycle
  interrupts(); // enable all interrupts
  BattConnect();
  inicio();
}

void loop() {
  unsigned long currentMillis = millis();

  //Ejecución continua *****************************************
  ReadStates();
  CycleCharge();
  BatteryProtection();

  //actualización al desborde *****************************
  if ((unsigned long)(currentMillis < previousMillis)) {
    state1 = millis();
    state2 = millis();
    state3 = millis();
    previousMillis = currentMillis;
  } else
  {
    previousMillis = currentMillis;
  }
  //Ejecución cada 3 milisegundos *****************************
  if ((unsigned long)(currentMillis - state1) >= 3) {
    Estado1();
    state1 = millis();
  }
  //Ejecucion cada medio ||||segundo *************************************
  if ((unsigned long)(currentMillis - state2) >= 500) {
    Estado2();
    state2 = millis();
  }
  //Ejecucion cada dos segundos *************************************
  if ((unsigned long)(currentMillis - state3) >= 2000) {
    Estado3();
    state3 = millis();
  }
}
//**********************************************************************************

void BatteryProtection()
{
  if (VM < 105)
  {
    digitalWrite(BP, LOW);                            // Turn off battery protection
    dato1 = "Panel desconectado";
  } else
  {
    digitalWrite(BP, HIGH);                            // Turn on battery protection
  }
  //**************** Connect or disconnect load at base of minimal battery voltage ****************
  if ((FB < VO) & (loadOff == false))
  {
    loadOff = true;

  }


}
void CycleCharge()
{
  //*************** Cycle of charge controler ***********************************
  if ((FB < BA) & (FloatBattery == false))
  {
    RV = BA;
    dato1 = "Modo Bulk";
  } else
  {
    FloatBattery = true;
  }
  if ((FB > VD) & (FloatBattery == true))
  {
    RV = VF;
    dato1 = "Modo de flotación";

  } else
  {
    FloatBattery = false;
  }
}
void Estado1()
{
  if (VM < VI) {
    OCR1AL = (unsigned char)(0);
    VoltageRegulateEnable = false;
  } else
  {
    Control();
    CalculateCurrent();
    VoltageRegulateEnable = true;
    if (loadOff == true)
    {
      loadOff = false;

    }
  }
}
void Estado2()
{
  if (loadOff == true)
  {
    digitalWrite(GL, LOW);                            // Turn off Green Led
    digitalWrite(RL, LOW);                            // Turn off Red Led
    digitalWrite(YL, HIGH);                            // Turn on yellow Led
    digitalWrite(LC, LOW);                            // Turn off load when system started
    dato2 = "Batería baja";
  } else
  {
    digitalWrite(YL, LOW);                            // Turn off yellow Led
    digitalWrite(LC, HIGH);                            // Turn on load
    if (VoltageRegulateEnable == true)
    {
      digitalWrite(RL, LOW);                            // Turn off Red Led
      digitalWrite(GL, HIGH);                            // Turn on Green Led
      dato2 = "Cargando batería";
    } else
    {
      digitalWrite(RL, HIGH);                           // Turn on Red Led
      digitalWrite(GL, LOW);                            // Turn off Green Led
      dato2 = "Batería en uso";
    }
  }
}
void Estado3()
{
  Serial.println("*********************************************");
  Serial.print("Tipo de batería: ");
  Serial.println(dato);
  Serial.print("Voltaje actual de batería: ");
  Serial.println(FB / (convertion * 1024 / 5));
  Serial.print("Voltaje de entrada: ");
  Serial.println(VinC * 5 * VM / 1024);
  Serial.print("Corriente de carga actual: ");
  Serial.println(CI * CurrentConv);
  Serial.print("Modo actual: ");
  Serial.println(dato1);
  Serial.print("Estado de la batería: ");
  Serial.println(dato2);
}

void BattConnect()
{
  ReadStates();
  Calculate();
}
void ReadStates()
{
  FB = (float)(analogRead(x0Pin));    //Feedback signal
  CI = (float)(analogRead(x1Pin));    //Current sensor
  VM = (float)(analogRead(x2Pin));    //Voltage input
}

void Calculate()
{
  //******************** Calculate all conversions **********************************************************
  if (FB < 19 * convertion * 1024 / 5)
  {
    BA = (VoltageR * convertion * 1024 / 5);          // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VF = (VoltageFloat * convertion * 1024 / 5);      // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VD = (VoltageFloat - changeMode) * 0.1 * 1024 / 5; // Convert voltage to conversion ADC value
    VO = (VoltageMinBattery * convertion * 1024 / 5); // Convert voltage to conversion ADC value
    VI = (VoltageR + minDVoltage) * 0.1 * 1024 / 5; // Convert voltage to conversion ADC value
    Serial.println("12V");
    dato = "12V";
  } else
  {
    BA = (VoltageR24 * convertion * 1024 / 5);          // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VF = (VoltageFloat24 * convertion * 1024 / 5);      // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VD = (VoltageFloat24 - changeMode) * convertion * 1024 / 5; // Convert voltage to conversion ADC value
    VO = (VoltageMinBattery24 * convertion * 1024 / 5); // Convert voltage to conversion ADC value
    VI = (VoltageR24 + minDVoltage) * 0.1 * 1024 / 5;   // Convert voltage to conversion ADC value
    Serial.println("24V");
    dato = "24V";
  }
  RV = BA;                                          // Start in mode Bulk-Absorcion
}

void Control() {
  float k = 0.08;                           //  Gain in close loop
  e = RV - FB;                              //  Error
  Switch_cycle = Switch_cycle + e * k - eL; //  Control
  if (Switch_cycle <= 0)                    //  Min dute Cycle = 0
    Switch_cycle = 0;
  if (Switch_cycle >= 125)                  //  Máx duye Cycle = 98% ;(122/128)*100 = 97.66% real value
    Switch_cycle = 125;

  OCR1AL = (unsigned char)(Switch_cycle);   //  Write dute Cycle
}
void CalculateCurrent()
{
  CM = CI * CurrentConv;
  if (CM > limCurrent)
  {
    eL = 50 * CM;
  } else
  {
    eL = 0;
  }

}
void inicio()
{
  for (int x = 0; x < 3; x++)
  {
    digitalWrite(GL, HIGH);                            // Turn on Green Led
    delay(250);
    digitalWrite(GL, LOW);                            // Turn off Green Led
    delay(250);
  }
  delay(500);
}
