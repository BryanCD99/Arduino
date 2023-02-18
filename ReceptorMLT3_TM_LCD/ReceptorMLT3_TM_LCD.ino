#include "MLT3.h"
MLT3 MLT3;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  MLT3.begin(4800, Par);
  Serial.println("Iniciando: ");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (MLT3.available() > 0) {
    uint8_t Dato = MLT3.read();
    char c = (char)Dato;
    if (Dato == 0x00) {
      Serial.println(" :Recibido");
    } else {
      Serial.print(c);
    }
  }
}
