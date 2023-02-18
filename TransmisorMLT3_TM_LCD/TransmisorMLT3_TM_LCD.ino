#include "MLT3.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>


// Definiciones del programa
#define LowCharacter 32
#define HighCharacter 126
#define MaxStringLength 4

// Variables de uso general *****
char key;
bool WriteLCD = false;
bool NoCharacter = false;
bool Send = false;
byte Character = 65; // Variable que contiene el caracter actual a mostrar en el LCD. Inicia con la 'A'
char Caracter = ' '; // Almacena el caracter a enviar al LCD
byte CursorLocation = 0; // Posicion del cursor del LCD
String Data = "";

// Constantes del programa
const byte ROWS = 4;
const byte COLS = 4;

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// Objeto de la pantalla LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

MLT3 MLT3;
Keypad myKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  MLT3.begin(4800, Par);

  lcd.init();
  lcd.setCursor(0, 0);
  lcd.print("Message: ");
  lcd.setCursor(0, 1);
  lcd.backlight();
  lcd.blink();
}
void GetKeys() {
  key = myKeypad.getKey();
  if (key != NO_KEY) {
    WriteLCD = true;
  }
}
void LCDControl() {
  if (WriteLCD) {
    WriteLCD = false;
    // Borra el dato ************
    if (key == '#') {
      CursorLocation = 0;
      NoCharacter = false;
      Data = "";
      lcd.clear();
      // Mostrar mensaje borrando ***
      lcd.setCursor(0, 0);
      lcd.print("Borrando");
      delay(300);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Message: ");
      lcd.setCursor(CursorLocation, 1);
      lcd.blink();
      // Envia el dato *************
    } else if (key == '*') {
      if (Caracter != ' ') {
        Data += Caracter;
      }
      Send = true;
      CursorLocation = 0;
      NoCharacter = false;
      Character = 65;
      lcd.clear();
      // Mostrar mensaje enviando ***
      lcd.setCursor(0, 0);
      lcd.print("Enviando");
      delay(300);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Message: ");
      lcd.setCursor(CursorLocation, 1);
      lcd.blink();
      // Incrementa el contador de caracter ************
    } else if (key == 'A') {
      if (!NoCharacter) {
        Caracter = GetCharacter(true);
        lcd.print(Caracter);
        lcd.setCursor(CursorLocation, 1);
      }
    } else if (key == 'B') {
      if (!NoCharacter) {
        Caracter = GetCharacter(false);
        lcd.print(Caracter);
        lcd.setCursor(CursorLocation, 1);
      }
    } else if (key == 'C') {
      CursorLocation ++;
      if (CursorLocation < MaxStringLength) {

        lcd.setCursor(CursorLocation, 1);
        Data += Caracter;
        Caracter = ' ';
      } else {
        lcd.noBlink();
        NoCharacter = true;
      }
      // Almacena datos numericos ****
    } else {
      if (!NoCharacter) {
        lcd.print(key);
        lcd.setCursor(CursorLocation, 1);
        Caracter = key;
      }
    }
  }
}
void SendData() {
  if (Send) {
    Send = false;
    // Envia los datos binarios correspondiente a la cadena ******
    for (int i = 0; i < Data.length(); i++) {
      Serial.write(Data[i]);
      MLT3.write(Data[i]);
    }
    // Imprime caracter de finalizacion
    MLT3.write(0x00);
    Data = "";
  }
}
char GetCharacter(bool UpDownControl) {
  char CurrentChar = (char)Character;
  if (UpDownControl) {
    if (Character < 126) {
      Character++;
    } else {
      Character = 32;
    }
  } else {
    if (Character > 32) {
      Character--;
    } else {
      Character = 126;
    }
  }
  return CurrentChar;
}

void loop() {
  GetKeys();
  LCDControl();
  SendData();
}
