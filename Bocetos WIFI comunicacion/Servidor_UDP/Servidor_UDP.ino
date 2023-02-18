//-- Libraries Included --------------------------------------------------------------
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

//------------------------------------------------------------------------------------
  // Define I/O Pins
  #define     LED0        2         // WIFI Module LED

//------------------------------------------------------------------------------------
  // Authentication Variables
  char*       ssid;              // SERVER WIFI NAME
  char*       password;          // SERVER PASSWORD

//------------------------------------------------------------------------------------
  #define     MAXSC     6           // MAXIMUM NUMBER OF CLIENTS
    
  IPAddress APlocal_IP(192, 168, 4, 1);
  IPAddress APgateway(192, 168, 4, 1);
  IPAddress APsubnet(255, 255, 255, 0);

  unsigned int UDPPort = 2390;      // local port to listen on

  WiFiUDP Udp;
//------------------------------------------------------------------------------------
  // Some Variables
  char packetBuffer[255]; //buffer to hold incoming packet
  char result[10];


void setup() {
  Serial.begin(115200);

  // Setting Up A Wifi Access Point
  SetWifi("DataTransfer","");
}

void loop() {

  HandleClients();

}



void HandleClients(){
unsigned long tNow;
  
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    
    Serial.print("From ");
    Serial.print(Udp.remoteIP());
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    
    tNow=millis();
    dtostrf(tNow, 8, 0, result);
    
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(result);
    Udp.endPacket();
  }
}


void SetWifi(char* Name, char* Password){
  // Stop active previous WIFI
  WiFi.disconnect();

  // Setting the WiFi mode
  WiFi.mode(WIFI_AP_STA);
  Serial.println("WIFI Mode : AccessPoint Station");

  ssid      = Name;
  password  = Password;

  // Starting the AccessPoint
  WiFi.softAPConfig(APlocal_IP, APgateway, APsubnet);                   // softAPConfig (local_ip, gateway, subnet)
  WiFi.softAP(ssid, password, 1 ,0, MAXSC);                             // WiFi.softAP(ssid, password, channel, hidden, max_connection)                         
  Serial.println("WIFI < " + String(ssid) + " > ... Started");

  // Wait a bit
  delay(50);

  // Getting server IP
  IPAddress IP = WiFi.softAPIP();
   
  // Printing the server IP Address
  Serial.print("AccessPoint IP : ");
  Serial.println(IP);

  // Starting UDP Server
  Udp.begin(UDPPort);
  
  Serial.println("Server Started");
}





