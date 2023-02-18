#include <ESP8266WiFi.h>
#include <WebSocketClient.h>


//char path[] = "/";
//char host[] = "echo.websocket.org";

//const char* ssid = "JESSENIA";
//const char* password = "rosita2000@";
// Host y port para la comunicación con el socket del ESP8266 receptor ***
const char* host = "192.168.137.1";
const uint16_t port = 5050;
const char* ssid = "JessyNetwork";
const char* password = "JN1234567";

// Use WiFiClient class to create TCP connections
WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(5000);


  // Connect to the websocket server
  if (client.connect(host, port)) {
    Serial.println("Connected");
  } else {
    Serial.println("Connection failed.");
    while (1) {
      // Hang on failure
    }
  }
}


void loop() {
  String data;

  if (client.connected()) {

    if (client.available()) {
      char c = client.read();
      switch (c) {
        case 'S':
          client.print("Send Data");
          break;
        case 'M':
          Serial.print('M');
          client.print("Motor ON!");
          break;
      }
    }

  } else {
    Serial.println("Client disconnected.");
    while (1) {
      // Hang on disconnect.
    }
  }

  // wait to fully let the client disconnect
  delay(100);

}
