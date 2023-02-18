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
  const String  Devicename = "Device_1";

//------------------------------------------------------------------------------------
  // WIFI Module Role & Port    
  IPAddress APlocal_IP(192, 168, 4, 1);
  //IPAddress    apIP(10, 10, 10, 1);
  IPAddress APgateway(192, 168, 4, 1);
  IPAddress APsubnet(255, 255, 255, 0);

  unsigned int UDPPort = 2390;      // local port to send and listen to  

  WiFiUDP Udp;

//------------------------------------------------------------------------------------
  // Some Variables
  char result[16];              // Buffer big enough for 7-character float
  char packetBuffer[255];       // buffer for incoming data

//====================================================================================

void setup() {
  
  // Setting The Serial Port ----------------------------------------------
  Serial.begin(115200);
  
  // WiFi Connect ----------------------------------------------------  
  Check_WiFi_and_Connect();
}

//====================================================================================

void loop() {

  Send_Data_To_Server();

}

//====================================================================================

void Send_Data_To_Server() {
unsigned long tNow;
    
  tNow=millis();                                                // get the current runtime
  dtostrf(tNow, 8, 0, result);                                  // translate it to a char array.

  Udp.beginPacket(APlocal_IP, UDPPort);                         // the IP Adress must be known 
  // Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());         // this can be used, to answer to a peer, if data was received first
  Udp.write(result);
  Udp.endPacket();                                              // this will automatically send the data

  while(1){
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      Serial.print("Contents:");
      Serial.println(packetBuffer);
      break;                                                    // exit the while-loop
    }
    if((millis()-tNow)>1000){                                   // if more then 1 second no reply -> exit
      Serial.println("timeout");
      break;                                                    // exit
    }
  }
}

//====================================================================================

void Check_WiFi_and_Connect(){
  if (WiFi.status() != WL_CONNECTED){
    WiFi.disconnect();                                          // probably not necessary due to WiFi.status() != WL_CONNECTED
    WiFi.begin("DataTransfer");                                 // reconnect to the Network
    Serial.println();
    Serial.print("Wait for WiFi");

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());

    Udp.begin(UDPPort);
  }
}

