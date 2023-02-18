//-- Libraries Included --------------------------------------------------------------
  #include <ESP8266WiFi.h>

//------------------------------------------------------------------------------------
  // Define I/O Pins
  #define     LED0        2         // WIFI Module LED

//------------------------------------------------------------------------------------
  // Authentication Variables
  char*       ssid;              // SERVER WIFI NAME
  char*       password;          // SERVER PASSWORD

//------------------------------------------------------------------------------------
  // WiFi settings
  #define     MAXSC     6           // MAXIMUM NUMBER OF CLIENTS

  IPAddress APlocal_IP(192, 168, 4, 1);
  IPAddress APgateway(192, 168, 4, 1);
  IPAddress APsubnet(255, 255, 255, 0);

  unsigned int TCPPort = 2390;
 
  WiFiServer  TCP_SERVER(TCPPort);      // THE SERVER AND THE PORT NUMBER
  WiFiClient  TCP_Client[MAXSC];        // THE SERVER CLIENTS Maximum number
//------------------------------------------------------------------------------------
  // Some Variables
  char result[10];


void setup(){
  
  // Setting the serial port
  Serial.begin(115200);           // Computer Communication
    
  // Setting the mode of the pins
  pinMode(LED0, OUTPUT);          // WIFI OnBoard LED Light
    
  // setting up a Wifi AccessPoint
  SetWifi("DataTransfer","");
}

//====================================================================================
 
void loop(){
  
  HandleClients(); 

}

//====================================================================================
 
void SetWifi(char* Name, char* Password){
  // Stop any previous WIFI
  WiFi.disconnect();

  // Setting The Wifi Mode
  WiFi.mode(WIFI_AP_STA);
  Serial.println("WIFI Mode : AccessPoint");
  
  // Setting the AccessPoint name & password
  ssid      = Name;
  password  = Password;
   
  // Starting the access point
  WiFi.softAPConfig(APlocal_IP, APgateway, APsubnet);                 // softAPConfig (local_ip, gateway, subnet)
  WiFi.softAP(ssid, password, 1, 0, MAXSC);                           // WiFi.softAP(ssid, password, channel, hidden, max_connection)     
  Serial.println("WIFI < " + String(ssid) + " > ... Started");
   
  // wait a bit
  delay(50);
   
  // getting server IP
  IPAddress IP = WiFi.softAPIP();
  
  // printing the server IP address
  Serial.print("AccessPoint IP : ");
  Serial.println(IP);

  // starting server
  TCP_SERVER.begin();                                                 // which means basically WiFiServer(TCPPort);
  
  Serial.println("Server Started");
}

//====================================================================================

void HandleClients(){
unsigned long tNow;
       
  if(TCP_SERVER.hasClient()){
    WiFiClient TCP_Client = TCP_SERVER.available();
    TCP_Client.setNoDelay(1);                                          // enable fast communication
    while(1){
      //---------------------------------------------------------------
      // If clients are connected
      //---------------------------------------------------------------
      if(TCP_Client.available()){
        // read the message
        String Message = TCP_Client.readStringUntil('\r');
          
        // print the message on the screen
        Serial.print("Received packet of size ");
        Serial.println(sizeof(Message));
          
        // print who sent it
        Serial.print("From ");
        Serial.print(TCP_Client.remoteIP());
        Serial.print(", port ");
        Serial.println(TCP_Client.remotePort());

        // content
        Serial.print("Content: ");
        Serial.println(Message);

        // generate a response - current run-time -> to identify the speed of the response
        tNow=millis();
        dtostrf(tNow, 8, 0, result);
          
        // reply to the client with a message     
        TCP_Client.println(result);                             // important to use println instead of print, as we are looking for a '\r' at the client
        TCP_Client.flush();
      }
       
      //---------------------------------------------------------------
      // If clients are disconnected                                            // does not realy work....
      //---------------------------------------------------------------     
      if(!TCP_Client || !TCP_Client.connected()){
        // Here We Turn Off The LED To Indicated The Its Disconnectted
        digitalWrite(LED0, LOW);
        break;
      }
      
    }   
  }
  else{
    // the LED blinks if no clients are available
    digitalWrite(LED0, HIGH);
    delay(250);
    digitalWrite(LED0, LOW);
    delay(250);
  }
}


