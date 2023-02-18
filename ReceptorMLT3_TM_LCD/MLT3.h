#define Par true
#define Impar false

#include <Arduino.h>
#define RX_Buffer 64

// Indica el voltage del bit. 0:0V, 1:2.5V, 2:5V
static int MLT3StarBit = 1;
// Indica la Transicion del MLT3. True: indica un semiciclo ascendente 0V -> 2.5V,  2.5V -> 5V, 
// False: indica un semiciclo descendente 0V -> 2.5V,  2.5V -> 5V
static bool Transition = true;

// Almacena el periodo actual
static volatile int TS = 0;
// Contador para la recepción del bit. Inicializa en -1 porque al entrar a la subrutina de recepción se le suma 1 automaticamente
static volatile int BitCounter = -1;
// Variable de verificación de paridad
static volatile bool CheckInputParity;
// Indica si se desea usar paridad en la transmisión. False implica que no se desea usar paridad
static volatile bool UseParidad = false;
// Almacena la paridad calculada de la transmision
static volatile bool Paridad;

// Variables de lectura del ADC. inicializan en 512 porque aprox 2.5V
static volatile int ADCBuffer = 512;
static volatile int ADCData = 512;

// Buffer de recepción de 64 bytes
static volatile uint16_t MLT3Buffer[RX_Buffer]; //Buffer
static volatile uint16_t BufferTail = 0; // Cola de lectura
static volatile uint16_t BufferHead = 0; // Cabeza de escritura
static volatile uint16_t index = 0; // Indice de guardado del dato leido

static volatile uint16_t MLT3RXData;  // Variable del dato leido
static uint16_t MLT3RXBuffer = 0;     // Variable de entrada del dato leido
static volatile uint16_t MLT3ReadBuffer = 0; // Buffer de rececpcion de los bits del MLT3

class MLT3 : public Print           // Crea la clase y ademas llama a la libreria print
{ private:
    int MLT3WriteBuffer[9];         // Buffer de transmision. Almacena los bits a ser enviados
    void SetOutput(int type);       // Configura la salida
    void MLT3Encode(uint8_t Dato);  // Codifica los bits
    void ClockGenerator();          // Genera la señal de reloj

  public:
    // Constructor de la clase
    MLT3() {
      // Baudrate por defecto
      TS = 1000000 / 9600;
      // Sin paridad
      UseParidad = false;
      // Paridad por defecto Par
      Paridad = false;
      // Inicializa la variable para comprobar la paridad
      CheckInputParity = false;

      // Configura señal de reloj para el transmisor ************+
      // D12 como salida
      DDRB |= B00010000;

      // Configuracion del puerto analogico A0 como entrada para el MLT3
      // Configuración del puerto analogico para alta velocidad ***********
      //REFS1:0,REFS0:1 -> Referencia a VCC, ADLAR:0 -> Alinea a la derecha. 16 bits
      //MUX3:0 MUX2:0 MUX1:0 MUX0:0 -> Selecciona canal 0
      ADMUX = B01000000;

      ADCSRB = 0; // Modo Free Running. No utilizado en este caso
      // ADEN:1 -> Habilita el modulo de ADC, ADATE:1 -> auto trigger disable, ADPS2:0:b100 -> Preescaler = 16
      // Clock = 16Mhz/64 = 250Khz
      // Sampling = 250Khz/(13 clocks) = 19,23 Khz
      ADCSRA = B10000110;

      // Realiza una primera conversión
      ADCSRA |= B01000000;
      while ((ADCSRA && B01000000) == B01000000) {  // Espera a que termine la captura del dato
        continue;
      }

      // Configura señal de reloj para el receptor ************
      // A1 como entrada digital
      DDRC &= B11111101;
      // Configuracion de la interrupcion de A1
      PCICR =  B00000010; //PORTC
      // Habilitacion del pin A1 para la interrupcion
      PCMSK1 = B00000010; //PCINT9
    }
    void begin(int Baudrate);
    void begin(int Baudrate, bool Paridad);
    uint8_t read();
    virtual size_t write(uint8_t Data);
    int available();
    static void MLT3Decode();
};
