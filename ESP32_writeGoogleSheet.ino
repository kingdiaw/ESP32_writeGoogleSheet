//(1)-Include Library
#include <WiFiClientSecure.h>
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager

//API:https://script.google.com/macros/s/AKfycbxS6VbbiK0jXnQ2XliuQm1tFwilDfKfNNEtgn3OgvUfYqW2794tokXeW9Z1rGvar74_/exec?temp=24&humi=55.8
//(2)-Define Constant Value
const char* ssid     = "4G-CPE-141B";     // your network SSID (name of wifi network)
const char* password = "king@535382";     // your network password
const char* host = "script.google.com";
const int port = 443;

//(3)-Object Mapping
WiFiClientSecure client;
WiFiManager wm;

//(4)-I/O Mapping
const unsigned char temp_sensor = 35;

//(5)-Global Variable Declaration
float suhu=27.0;
float humi=56.7;
uint32_t sysTick = 0;

//6.1 Function - setup_wifi()
void setup_wifi(void) {
  
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(60);
  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name
  if (wm.autoConnect("AutoConnectAP")) {
    Serial.println("connected...yeey :)");
  }
  else {
    Serial.println("Configportal running");
  }
}

//6.2 Function - update_gssheet()
void update_gsheet(void) {
  Serial.println("\nStarting connection to server...");

  if (!client.connect(host, port))
    Serial.println("Connection failed!");
  else {
    Serial.println("Connected to server!");
    //Prepare HTTP header
    String header = "";
    header = "GET https://script.google.com/macros/s/AKfycbxS6VbbiK0jXnQ2XliuQm1tFwilDfKfNNEtgn3OgvUfYqW2794tokXeW9Z1rGvar74_/exec?temp=" + (String(suhu)) + "&humi=" + (String(humi))+ " HTTP/1.0\n";
    header += "Host: script.google.com\n";
    header += "Connection: close\n\n";
    Serial.print(header);
    // Make a HTTP request:
    client.print(header);

    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        Serial.println("headers received");
        break;
      }
    }
    // if there are incoming bytes available
    // from the server, read them and print them:
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
    client.stop();
  }
}

void setup() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  Serial.begin(115200);
  setup_wifi();

}

void loop() {
  wm.process();
  // put your main code here, to run repeatedly:
  if(sysTick < millis()){
    sysTick = millis()+5000;
    update_gsheet();
  }
}
