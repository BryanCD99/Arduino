
#include <Wire.h>
#include <MPU6050.h>

//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
#define MPUReadTime 10

MPU6050 mpu;
float pitch, roll, yaw;
unsigned long CurrentTime, MPUDT_Time;

void setup() {
  Serial.begin(115200);

  Serial.println("Initialize MPU6050");

  while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

}
void ReadMPU(float dt) {
  dt = dt/1000;
  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();
  
  // Read normalized values
  Vector normGyro = mpu.readNormalizeGyro();
  // Traslacion de los ejes para la utilización de otro plano cartesiano
  Vector translate = normAccel;
  normAccel.XAxis = translate.ZAxis;
  normAccel.ZAxis = -translate.XAxis;
  // Calculate Pitch & Roll
  pitch = atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * RAD_TO_DEG;
  roll = atan2(normAccel.YAxis, sqrt(normAccel.XAxis * normAccel.XAxis + normAccel.ZAxis * normAccel.ZAxis)) * RAD_TO_DEG;

  //dt = (millis() - tiempo_prev) / 1000.0;
  //tiempo_prev = millis();

  //Aplicar el Filtro Complementario
  pitch = round(0.98 * (pitch + normGyro.XAxis * dt) + 0.02 * pitch);
  roll = round(0.98 * (roll + normGyro.YAxis * dt) + 0.02 * roll);
  //Integración respecto del tiempo paras calcular el YAW
  /*
  if (abs((normGyro.ZAxis * dt)) > 1) {
    yaw = round(yaw + normGyro.ZAxis * dt);
  }
  */
  yaw = normGyro.YAxis;
}
void loop() {
  // Actualización de tiempo de sistema **************
  CurrentTime = millis();
  // Controlador de lectura del giroscopio **************
  if ((CurrentTime - MPUDT_Time) > MPUReadTime) {
    ReadMPU(float(CurrentTime - MPUDT_Time));
    Serial.print(" XAng = ");
    Serial.print(pitch);
    Serial.print(" YAng = ");
    Serial.print(roll);
    Serial.print(" ZAng = ");
    Serial.println(yaw);
    MPUDT_Time = CurrentTime;
  }
}
