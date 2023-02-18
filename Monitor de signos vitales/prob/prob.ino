#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

 
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
 
double temp_amb;
double temp_obj;
//double calibration = 4.36;
double calibration = (calibration * 3.3 * 100)/1024;
/*char auth[] = "bOTyO9tRJyp8d0FLY8CoFyW115buIiZC";    // You should get Auth Token in the Blynk App.
char ssid[] = "Alexahome";                       // Your WiFi credentials.
char pass[] = "loranthus";
 */
void setup()
{
  Serial.begin(9600);
  mlx.begin();         //Initialize MLX90614
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //initialize with the I2C addr 0x3C (128x64)
  
  Serial.println("Temperature Sensor MLX90614");
 
  display.clearDisplay();
  display.setCursor(25,15);  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(" Universidad tecnica");
  display.setCursor(25,35);
  display.setTextSize(1);
  display.print("del norte");
  display.display();
  delay(2500);
}
 
void loop()
{
  //Reading room temperature and object temp
  //for reading Fahrenheit values, use
  //mlx.readAmbientTempF() , mlx.readObjectTempF() )
  temp_amb = mlx.readAmbientTempC();
  temp_obj = mlx.readObjectTempC();
 
  //Serial Monitor
  Serial.print("Room Temp = ");
  Serial.println(temp_amb);
  Serial.print("Object temp = ");
  Serial.println(temp_obj);
 
  display.clearDisplay();
  display.setCursor(25,0);  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(" Temperatura");
  
  display.setCursor(10,20);
  display.setTextSize(1);
  display.print("Ambient: ");
  display.print(temp_amb);
  display.print((char)247);
  display.print("C");
 
  display.setCursor(10,40);
  display.setTextSize(1);
  display.print("Object: ");
  display.print(temp_obj + calibration);
  display.print((char)247);
  display.print("C");
  
  display.display();
  delay(1000);
}
