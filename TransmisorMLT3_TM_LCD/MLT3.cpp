#include <Arduino.h>
#include "MLT3.h"

void MLT3::SetOutput(int type) {
  // D10(PB2) pin que permite salidas de 5V y 0V
  // D11(PB3) pin que permite salidas de 2.5V
  switch (type) {
    // Salida de 0V *************
    case 0:
      // Pone el pin D10 (PORTB.2) a 0V
      PORTB &= ~B00000100;

      // D10 como salida (PORTB.2)
      DDRB   |=    B00000100;
      // D11 como entrada (PORTB.3)
      DDRB   &=   ~B00001000;
      break;
    // Salida de 2.5V *************
    case 1:
      // Pone el pin D11 (PORTB.3) a 5V. Por el divisor de voltaje llega a 2.5V
      PORTB |= B00001000;

      // D11 como salida (PORTB.3)
      DDRB   |=    B00001000;
      // D10 como entrada (PORTB.2)
      DDRB   &=   ~B00000100;
      break;
    // Salida de 5V *************
    case 2:
      // Pone el pin D10 (PORTB.2) a 5V
      PORTB |= B00000100;
      
      // D10 como salida (PORTB.2)
      DDRB   |=    B00000100;
      // D11 como entrada (PORTB.3)
      DDRB   &=   ~B00001000;
      break;
  }
}
// Inicia la libreria pero sin paridad
void MLT3::begin(int Baudrate) {
  SetOutput(1);
  TS = 1000000 / Baudrate;
  UseParidad = false;
}
// Inicial la libreria usando paridad
void MLT3::begin(int Baudrate, bool P) {
  SetOutput(1);
  TS = 1000000 / Baudrate;
  UseParidad = true;
  Paridad = P;
}
// Escribe un byte usando el procolo MLT3
size_t MLT3::write(uint8_t Data) {
  MLT3Encode(Data);
  // NOTA:(UseParidad ? 9 : 8) es equivalente a un if, donde se usa el valor 9 cuando es UseParidad = true y viceversa
  for (int BitIndex = 0; BitIndex < (UseParidad ? 9 : 8); BitIndex++) {
    SetOutput(MLT3WriteBuffer[BitIndex]);
    ClockGenerator();
  }
}
// Retorna un dato del buffer de recepcion. Si se leyo todo el buffer retorna el ultimo dato leido
uint8_t MLT3::read() {
  if (BufferTail == BufferHead) {
    // Retorna el ultimo valor almacenado
    return MLT3RXBuffer;
  } else {  
    MLT3RXBuffer = MLT3Buffer[BufferTail];     // Obtiene el dato del buffer serial
    BufferTail = (BufferTail + 1) % RX_Buffer; // Actualiza la cola de lectura
    return (uint8_t)MLT3RXBuffer;              // Retorna los datos del buffer serial
  }
}
// Decodifica un byte entrante por la recepción.
// Utiliza el ADC del Atmega como pin de entrada. Para saber cuando es un "1" se toma como base que debe ocurrir una transicion de voltaje.
// Es decir, el valor del ADC del bit anterior y el actual son diferentes, por lo que al restarlos el resultado debe ser diferente de 0.
// Los "0" se identifican usando el mismo metodo con la diferencia que al restar los valores del ADC el resultado siempre sera un valor cercano a 0 
void MLT3::MLT3Decode() {
  // Si existe una diferencia considerable en el voltaje de la linea de datos, es porque ocurrio una transicion y el bit es 1
  //ADCBufferxD[BitCounter] = abs(ADCData - ADCBuffer);
  if (UseParidad) {       // Permite recibir datos con paridad
    if (abs(ADCData - ADCBuffer) > 50) { // Se compara con 50 porque si :'v se escogio randomicamente y funciono xD
      // Almacena un 1
      MLT3ReadBuffer |= 0x200;                  // Coloca un bit 1 en la posicion 9 
      MLT3ReadBuffer = MLT3ReadBuffer >> 1;     // Desplaza el contenido del buffer hacia la derecha
      CheckInputParity = CheckInputParity ^ 1;  // Verifica la paridad a medida que recibe un bit
    } else {
      // Almacena un 0
      MLT3ReadBuffer &= 0x1FF;                  // Coloca un 0 en la posicion 9        
      MLT3ReadBuffer = MLT3ReadBuffer >> 1;     // Desplaza el contenido del buffer hacia la derecha
    }
  } else { // Para recibir datos sin paridad
    if (abs(ADCData - ADCBuffer) > 50) { // LO mismo que lo de arriba solo que ahora pone un "1" o un "0" en la posicion 8
      // Almacena un 1
      //ADCBufferxD[BitCounter] = 1;
      MLT3ReadBuffer |= 0x100;
      MLT3ReadBuffer = MLT3ReadBuffer >> 1;
    } else {
      // Almacena un 0
      MLT3ReadBuffer &= 0x0FF;
      MLT3ReadBuffer = MLT3ReadBuffer >> 1;
    }
  }
}
// Retorna la cantidad de bytes disponibles en el buffer
int MLT3::available() {
  //Serial.println(((RX_Buffer + BufferHead - BufferTail)%RX_Buffer));
  return ((RX_Buffer + BufferHead - BufferTail) % RX_Buffer);
}
// Genera la señal de reloj de acuerdo con el baudrate
void MLT3::ClockGenerator() {
  // Pone el pin D12 (PORTB.4) a HIGH
  PORTB |= B00010000;
  delayMicroseconds(TS);
  // Pone el pin D12 (PORTB.4) a LOW
  PORTB &= ~B00010000;
  delayMicroseconds(TS);
}

// Codifica el byte a ser escrito por el puerto MLT3
void MLT3::MLT3Encode(uint8_t Dato) {
  uint16_t Data = Dato & 0x00FF;
  uint16_t CheckBit = 1;
  uint16_t CheckParity = 0;

  for (int BitIndex = 0; BitIndex < 9; BitIndex++) {
    // Si el bit es uno se cambia de estado
    if ((Data & CheckBit) == CheckBit) {

      // Si llega al valor alto, entonces debe bajar
      if (MLT3StarBit == 2) {
        Transition = false;
      }
      // Si llega al valor bajo, entonces debe subir
      if (MLT3StarBit == 0) {
        Transition = true;
      }
      // Incrementa o decrementa segun la transicion
      if (Transition) {
        MLT3StarBit += 1;
      } else {
        MLT3StarBit -= 1;
      }
      // Verifica paridad para todo el byte del dato
      if (BitIndex <= 7) {
        // Cada vez que se detecta un uno, entonces se realiza un xor para llevar la cuenta de la paridad
        CheckParity = CheckParity ^ 0x100;
      }
    }
    if (BitIndex == 7) {
      // Calcula el bit de paridad
      if (Paridad) { // Paridad = true, Paridad par
        if (CheckParity == 0x100) { // Implica que hay numero impar de 1
          Data |= 0x100; // Se aumenta un uno para hacer que haya un numero par de 1
        }
      } else { // Paridad = false, Paridad impar
        if (CheckParity == 0) { // Implica que hay numero par de 1
          Data |= 0x100; // Se aumenta un uno para hacer que haya un numero impar de 1
        }
      }
    }
    // Guarda en el buffer MLT3 el valor de voltaje a transmitir
    // 0V = 0
    // 2.5V = 1
    // 5V = 2
    MLT3WriteBuffer[BitIndex] = MLT3StarBit;
    CheckBit = CheckBit << 1; //Desplaza el bit a la izquierda
  }
}

// Se utiliza una interrupcion porque esto asegura que los bytes del MLT3 sean leidos asyncronicamente y no haya que estar comprobando de forma manual
// Se lee el pin durante el estado logico LOW de la señal del reloj mientras que se incrementa el contador de bits durante el esto logico alto
ISR(PCINT1_vect) {
  // Obtiene el estado del pin RX *********************
  // Inicia la conversión
  ADCSRA |= B01000000;
  while ((ADCSRA && B01000000) == B01000000) {  // Espera a que termine la captura del dato
    continue;
  }

  // Si esta en estado logico alto llama a la rutina que almacena el dato
  if ((PINC & B00000010) == B00000010) {
    BitCounter ++; // Incrementa el contador de bits
    //ADCBufferxD[BitCounter] = ADC;
  } else {

    ADCData = ADCBuffer;
    // Si es estado logico bajo, se almacena el estado del pin para posteriormente compararlo
    ADCBuffer = (int)ADC;
    // Guarda el Bit leido
    MLT3::MLT3Decode();

    // Si ya se recibio todo el dato, entonces se almacena
    if (BitCounter == (UseParidad ? 8 : 7)) {
      bool SaveData = true;
      // Si se usa paridad entonces se determina si se ha recibido de forma correcta
      if (UseParidad) {
        // Compara la paridad. Paridad es la "Paridad" seleccionada por el usuario y "CheckInputParity" es la del byte leido.
        // Cuando "Paridad" = true, hablamos de Par *********** Cuando "Paridad" = false, hablamos de Inpar
        // Cuando "CheckInputParity" = false, hablamos de un dato Par *********** Cuando "CheckInputParity" = true, hablamos de un dato impar
        // Por lo que la paridad es correcta cuando "Paridad" y "CheckInputParity" no son iguales xD (Tremenda fumada xD)
        // Operacion logica que da true cuando las entradas no son iguales, XOR (Simbolo) -> ^
        if (!(Paridad ^ CheckInputParity)) { // Verifica si las entradas son iguales. Si lo son, la paridad es erronea y no se guarda el dato
          SaveData = false;
        }
        CheckInputParity = false; // Reinicia la variable de control de paridad para el siguiente dato
      }

      // Obtiene el dato del buffer de entrada
      MLT3RXData = MLT3ReadBuffer;
      // Reinicia el buffer para recibir un nuevo dato
      MLT3ReadBuffer = 0;

      // Almacena el dato en el buffer de recpecion ****************
      if (SaveData) {
        // Actualiza el dato una vez se ha recibido completamente
        index = (uint16_t)(BufferHead + 1) % RX_Buffer;
        if (index != BufferTail) { // Si el buffer esta lleno entonces se descarta
          MLT3Buffer[BufferHead] = MLT3RXData;
          BufferHead = index;
        }
      }
      // Reinicia el contador para recibir un nuevo dato
      BitCounter = -1;
    }
  }
}
