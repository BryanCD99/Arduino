void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for(float a = 0;a<= 100;a+=0.1){
    Serial.print("Tangente de: ");
    Serial.print(a);
    Serial.print(" = ");  
    float tang = tan(a*PI/180);
    Serial.print(tang);
    Serial.print(", artang: ");
    Serial.println(atan(tang)*180/PI);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
//  float dato = 11;
//  dato = dato/0;
//  Serial.print("dat: ");
//  Serial.print(dato);
//  Serial.print(", atan: ");
//  Serial.println(atan(dato));

}
