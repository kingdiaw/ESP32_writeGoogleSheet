//(1)-Include Library
#include <WiFiClientSecure.h>
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager
#include <ESP32Servo.h>

//API:https://script.google.com/macros/s/AKfycbzIbiJlAI4H4iHRTwueYlhXXUDJF1X5E7MWmgbowVBhgbRGkr4dJoFCWyIX0zYFAHAr/exec?temp=24&humi=55.8
//(2)-Define Constant Value
const char* HOST = "script.google.com";
const int PORT = 443;
const uint8_t SERVO_PIN = 18;
const uint32_t INTERVAL_TASK1 = 60000;  //update gsheet
const uint16_t INTERVAL_TASK2 = 200;    //update servo position

//(3)-Object Mapping
WiFiClientSecure client;
WiFiManager wm;
Servo myservo;  

//(4)-I/O Mapping
const unsigned char temp_sensor = 35;

//(5)-Global Variable Declaration
float suhu = 27.0;
float humi = 56.7;
uint32_t timeUpdate_task1 = 0;
uint32_t timeUpdate_task2 = 0;

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

//6.2 Function - setup servo
void setup_servo(void) {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(SERVO_PIN, 500, 2400);   // attaches the servo on pin 18 to the servo object
                                          // using SG90 servo min/max of 500us and 2400us
                                          // for MG995 large servo, use 1000us and 2000us,
                                          // which are the defaults, so this line could be
                                          // "myservo.attach(SERVO_PIN);"
}
//6.3 Function - update_gssheet()
void update_gsheet(void) {
  Serial.println("\nStarting connection to server...");
  client.setInsecure();//skip verification
  if (!client.connect(HOST, PORT))
    Serial.println("Connection failed!");
  else {
    Serial.println("Connected to server!");
    //Prepare HTTP header
    String header = "";
    header = "GET https://script.google.com/macros/s/AKfycbzIbiJlAI4H4iHRTwueYlhXXUDJF1X5E7MWmgbowVBhgbRGkr4dJoFCWyIX0zYFAHAr/exec?temp=" + (String(suhu)) + "&humi=" + (String(humi)) + " HTTP/1.0\n";
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
  if (timeUpdate_task1  < millis()) {
    timeUpdate_task1 = millis() + INTERVAL_TASK1;
    //Application for Task 1
    update_gsheet();
  }
  if(timeUpdate_task2 < millis()){
    static uint8_t pos;
    timeUpdate_task2 = millis() + INTERVAL_TASK2;
    //Application Task 2
    if(++pos > 180)pos = 0;
    Serial.println(pos);
    myservo.write(pos);    
  }
}
