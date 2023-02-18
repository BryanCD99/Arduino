#include <Wire.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>
#define adress 11

// Objeto Giroscopio-Acelerometro
MPU6050 mpu;

// serial r232
SoftwareSerial mySerial(7, 8); // RX, TX
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Data recibido
String Dato = "";

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}
void ReadMPU()
{
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  Serial.print(" Yaw = ");
  Serial.print(yaw);

  // Wait to full timeStep period
  delay((timeStep * 1000) - (millis() - timer));
}
void ReadESP() {
  mySerial.print('S');
  delay(10);
  Dato = "";
  if (mySerial.available() > 0) {
    while (mySerial.available() > 0) {
      char inByte = mySerial.read();
      Dato += inByte;
    }
    Serial.print(" espdata: ");
    Serial.println(Dato);
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  ReadMPU();
  ReadESP();

}
