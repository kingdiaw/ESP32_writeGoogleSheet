#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
WiFiManager wm;

void setup() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  Serial.begin(115200);
  setup_wifi();

}

void loop() {
  wm.process();
  // put your main code here, to run repeatedly:
}

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
