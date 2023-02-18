//*************************** Change these values at base of your battery *****************************
#define VoltageR 14.4f                // Voltage absorcion-bulk (V)
#define VoltageFloat 13.7f            // Voltage float battery  (V)
#define VoltageR24 29.6f              // Voltage absorcion-bulk (V)
#define VoltageFloat24 27.4f          // Voltage float battery  (V)
#define VoltageMinBattery 12.2f       // Voltage minimal to desconect load  (V)
#define VoltageMinBattery24 23.7f       // Voltage minimal to desconect load  (V)
#define limCurrent 2.0f               // Define limite current  (A)
#define minCurrent 0.2f               // Minimal current to change to float mode  (A)

// Setup of mainboard **********************************************************
#define minDVoltage 2.0f            // Define a minimal delta Voltage to Regulate
#define changeMode 0.7f             // Minimal delta Voltage to change mode
#define convertion 0.17595f         // Conversion factor
#define VinC 0.02783f               // Conversion to input voltage
#define CurrentConv 0.0302f         // Current by unit of ADC


// Definition of inputs and outputs ¨********************
const int uPin = 9;    // Control voltage, this is the pin called OC1A
const int LC = 2;      // Load connect pin
const int RL = 3;      // Red-Led
const int GL = 4;      // Green-Led
const int YL = 5;      // Yellow-Led
const int BP = 6;      // Battery Protection
const int x0Pin = A0;  // Voltage at the output of the second integrator
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
unsigned long softStart = 500;
unsigned long time_ = 0;

//  Variables to general use
bool VoltageRegulateEnable = false;
bool FloatBattery = false;
bool loadOff = false;

void setup() {
  
  //******************** Set all inputs and outputs **********************************************************
  pinMode(uPin,OUTPUT);                             // Set pin OC1A as output 
  pinMode(LC,OUTPUT);                               // Set Pin 2 as output
  pinMode(RL,OUTPUT);                               // Set Red Led as output
  pinMode(GL,OUTPUT);                               // Set Green Led as output
  pinMode(YL,OUTPUT);                               // Set Yellow Led as output
  pinMode(BP,OUTPUT);                               // Set Output Battery as output
  //******************** Set and Clear initial ***************************************************
  digitalWrite(LC,LOW);                             // Turn off load when system started
  digitalWrite(RL,LOW);                             // Turn off Red Led
  digitalWrite(GL,LOW);                             // Turn off Green Led
  digitalWrite(YL,LOW);                             // Turn off Yellow Led
  digitalWrite(BP,LOW);                             // Turn off Yellow Led
    //******************* Serial Comunication *****************************************************************
  Serial.begin(115200); // Used to development
  
  //******************* Delay before start operation ********************************************************
  BattConnect();
  SoftStartDelay();
  noInterrupts(); // disable all interrupts while do configure
  // ******************************* Set PWM frecuency **********************************************
  // Freq_PWM = 16Mhz /128 = 125 Khz 
  TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);   
  //       COM1A1 = 1  ; COM1A0 = 0 ; COM1B1 = 0  ; COM1B0 = 0  ;         Note: When use a inverted output, you must set COM1A1 & COM1A0, if you use a normal output, you only have to must COM1A1
  //       Clear output OC1A when match happen, Output OC1B disconnected
  TCCR1B = (0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);

  //        ICNC1 & ICES1 are used when Timer had a external signal. Unused in Fast PWM mode
  //        CS12 = 0; CS11 = 0; CS10 = 0; Select no prescaler
  //        WGM13 = 1 ; WGM12 = 1;  WGM11 = 1 ; WGM10 = 0; Select Fast 8 bit PWM with TOP value on ICR1
  
  ICR1H  = 0x00;  //Set ICR1 register to 80h (#128) MSB = 0x00, LSB = 80h
  ICR1L  = 0x80;  //This value is TOP of the counter. Timer reach to zero when count is equal to it.
  
  TCNT1H = 0x00; //Reset MSB of the timer
  TCNT1L = 0x00; //Reset LSM of the timer
  OCR1AH = 0x00; //Clear MSB of the match register, unused in Fast PWM 8 bit
  OCR1AL = 0x00; //Set LSB zero to start the system OFF, it register is used to change dute cycle

  // ****************************** Set interrup by timer 2 ******************************************
  // T = (1/16MHz)*(2^8-00)*8 = 128 uS
  TCCR2A = (0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);   
  //       COM1A1 = 0  ; COM1A0 = 0  ; COM1B1 = 0  ; COM1B0 = 0  ;  
  //       OC12A and OC2B both are disconnected
  
  TCCR2B = (0<<WGM22) | (0<<CS22) | (1<<CS21) | (0<<CS20);
  //        CS22 = 0; CS21 = 1; CS20 = 0; Select prescaler Fosc/8
  //        WGM13 = 0 ; WGM12 = 0;  WGM11 = 0 ; WGM10 = 0; Select Normal Mode operation

  TCNT2 = 0x00; //This register have a pre-load of the timer. However, in this case, it's no have pre-load
  OCR2A = 0x00; //Clear match register. It's unused
  OCR2B = 0x00; //Clear match register. It's unused

  TIMSK2 = (0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2); // Enable timer2 overflow interrupt
  // ****************************** End of the configurate *******************************************
  interrupts(); // enable all interrupts
     
} 
void BattConnect()
{
  ReadStates();
  Calculate();
}
void Calculate()
{
  //******************** Calculate all conversions **********************************************************
  if(FB<19*convertion*1024/5)
  {
    BA = (VoltageR*convertion*1024/5);                // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VF = (VoltageFloat*convertion*1024/5);            // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VD = (VoltageFloat-changeMode)*convertion*1024/5; // Convert voltage to conversion ADC value
    VO = (VoltageMinBattery*convertion*1024/5);       // Convert voltage to conversion ADC value
    VI = (VoltageR+minDVoltage)*convertion*1024/5;    // Convert voltage to conversion ADC value
    Serial.println("12V");
  }else
  {
    BA = (VoltageR24*convertion*1024/5);                // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VF = (VoltageFloat24*convertion*1024/5);            // Convert voltage Bulk-Absorcion mode to conversion ADC value
    VD = (VoltageFloat24-changeMode)*convertion*1024/5; // Convert voltage to conversion ADC value
    VO = (VoltageMinBattery24*convertion*1024/5);       // Convert voltage to conversion ADC value
    VI = (VoltageR24+minDVoltage)*convertion*1024/5;    // Convert voltage to conversion ADC value
    Serial.println("24V");  
  }
    RV = BA;                                          // Start in mode Bulk-Absorcion
}
ISR(TIMER2_OVF_vect) // interrupt service routine 
{   
  
    ReadStates();
    if(VoltageRegulateEnable == true)
    {
      Control();
      CalculateCurrent();
    }     
    Serial.print(FB/(convertion*1024/5)); 
    Serial.print("-");   
    Serial.println(VO);  
}
void Control(){
  float k = 0.08;                           //  Gain in close loop
  e = RV-FB;                                //  Error
  Switch_cycle = Switch_cycle + e*k-eL;     //  Control
  if(Switch_cycle<=0)                       //  Min dute Cycle = 0
    Switch_cycle = 0;
  if(Switch_cycle>=125)                     //  Máx duye Cycle = 98% ;(122/128)*100 = 97.66% real value
    Switch_cycle = 125;

  //Serial.print("-");
  OCR1AL = (unsigned char)(Switch_cycle);   //  Write dute Cycle
}
void CalculateCurrent()
{
  CM = CI*CurrentConv;
  if(CM>limCurrent)
  {
    eL = 18*CM;
  }else
  {
    eL = 0;  
  }
  
}
void ReadStates()
{
  FB = (float)(analogRead(x0Pin));    //Feedback signal
  CI = (float)(analogRead(x1Pin));    //Current sensor
  VM = (float)(analogRead(x2Pin));    //Voltage input
  
}
void SoftStartDelay() // Delay with interval of 500ms before start regulation
{
  Serial.println("Starting system...");
  delay(softStart);
  Serial.println("System Started!");
  delay(1);
  ReadStates();  
  digitalWrite(LC,HIGH);
}
void loop() {
  //**************** Turn on/off swicht regulator at base of minimal voltage input  ***************
  if(VM<VI){
      VoltageRegulateEnable = false;
      OCR1AL = (unsigned char)(0);  
      digitalWrite(RL,HIGH);                            // Turn on Red Led
      digitalWrite(GL,LOW);                             // Turn off Green Led
      digitalWrite(BP,LOW);
  }else{
      VoltageRegulateEnable = true;
      
      if(loadOff==true)
      {
        loadOff = false;
        digitalWrite(LC,HIGH);
      } 
      digitalWrite(RL,LOW);                             // Turn off Red Led
      digitalWrite(GL,HIGH);                             // Turn off Green Led
      digitalWrite(BP,HIGH);
  }
  
  //**************** Connect or disconnect load at base of minimal battery voltage ****************
  if((FB<VO)&(loadOff==false)){
      loadOff = true;
      digitalWrite(LC,LOW);
      digitalWrite(GL,LOW);                             // Turn off Green Led
      digitalWrite(RL,LOW);                             // Turn off Red Led
      digitalWrite(YL,HIGH);                             // Turn on yellow Led
  }  

  //*************** Cycle of charge controler ***********************************
  if((FB<BA)&(FloatBattery == false))
  {
     RV=BA; 
  }else
  {
     FloatBattery = true;  
  }
  if((FB>VD)&(FloatBattery == true))
  {
     RV=VF; 
     
  }else
  {
    FloatBattery = false;
  }
  
}
