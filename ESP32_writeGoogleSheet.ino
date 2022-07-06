//(1)-Include Library
#include <WiFiClientSecure.h>
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager
#include <ESP32Servo.h>       // by Kevin Harrington Version 0.11.0
#include <ESP32Time.h>        //https://github.com/fbiego/ESP32Time
#include <Wire.h>
#include <Adafruit_INA219.h>  // by Adafruit
#include "RTClib.h"           //https://github.com/adafruit/RTClib


//(2)-Define Constant Value
const char* HOST = "script.google.com";
const int PORT = 443;
const uint8_t SERVO_PIN = 14;
const uint32_t INTERVAL_TASK1 = 60000;  //update gsheet
const uint16_t INTERVAL_TASK2 = 2000;   //update servo position
const uint16_t INTERVAL_TASK3 = 1000;   //update Displaying Time
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 3600 * 8;  //GMT+8
const int   DAYLIGHT_OFFSET_SEC = 0;
const uint8_t PV_POSITION_ARRAY[9] = {25, 35, 45, 55, 65, 75, 85, 95, 100};
const uint8_t HOUR_START = 9;  //(9)9AM
const uint8_t HOUR_STOP = 17;   //(17)5PM
const uint8_t INDEX_OFFSET = HOUR_START;
const uint8_t ADD_INA219_PV = 0x40;
const uint8_t ADD_INA219_BAT = 0x41;
const uint8_t SHUNT_VOLTAGE_INDEX = 0x00;
const uint8_t BUS_VOLTAGE_INDEX = 0x01;
const uint8_t CURRENT_MA_INDEX = 0x02;
const uint8_t POWER_MW_INDEX = 0x03;
const uint8_t LOAD_VOLTAGE_INDEX = 0x04;
const uint8_t LED = 2;

//(3)-Object Mapping
WiFiClientSecure client;
WiFiManager wm;
Servo myservo;
RTC_DS3231 ds3231;
ESP32Time rtc;

Adafruit_INA219 pv(ADD_INA219_PV);
Adafruit_INA219 battery(ADD_INA219_BAT);

//(4)-I/O Mapping
const unsigned char temp_sensor = 35;

//(5)-Global Variable Declaration
float Vpv = 0.0;
float Ipv = 0.0;
float Vbat = 0.0;
float Ibat = 0.0;
float pv_array[5];
float battery_array[5];

uint32_t timeUpdate_task1 = 0;
uint32_t timeUpdate_task2 = 0;
uint32_t timeUpdate_task3 = 0;
bool ina219_isOK = false;
bool ds3231_isOK = false;

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
//API:https://script.google.com/macros/s/AKfycbyUBV3986c6CS_XWKnquRPZBZfNgR1dETGzt6IH8QseepRtoNaTV3yN0STdfQAko-T4/exec?Vpv=13.2&Ipv=0.33&Vbat=11.2&Ibat=0.22
void update_gsheet(void) {
  Serial.println("\nStarting connection to server...");
  client.setInsecure();//skip verification
  if (!client.connect(HOST, PORT))
    Serial.println("Connection failed!");
  else {
    Serial.println("Connected to server!");
    //Prepare HTTP header
    String header = "";
    header = "GET https://script.google.com/macros/s/AKfycbyUBV3986c6CS_XWKnquRPZBZfNgR1dETGzt6IH8QseepRtoNaTV3yN0STdfQAko-T4/exec?Vpv=" + (String(Vpv)) + "&Ipv=" + (String(Ipv)) + "&Vbat=" + (String(Vbat)) + "&Ibat=" + (String(Ibat)) + " HTTP/1.0\n";
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

//6.4 Setup RTC using DS3231
void setup_rtc(void) {
  ds3231_isOK = true;
  if (! ds3231.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    ds3231_isOK = false;
  }
  if (ds3231.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    ds3231.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = ds3231.now();
  rtc.setTime(now.second(), now.minute(), now.hour(), now.day(), now.month(), now.year());
}

//6.5 Setup INA219
void setup_ina219(void) {
  Serial.println("Initialize INA219...");
  ina219_isOK = true;
  if (!pv.begin()) {
    Serial.print("Fail to find INA219 at address:0x");
    Serial.println(ADD_INA219_PV, HEX);
    ina219_isOK = false;
  }
  if (!battery.begin()) {
    Serial.print("Fail to find INA219 at address:0x");
    Serial.println(ADD_INA219_BAT, HEX);
    ina219_isOK = false;
  }
}

//6.6 Read Data from INA219 (PV)
void read_pv_data(void) {
  pv_array[SHUNT_VOLTAGE_INDEX] = pv.getShuntVoltage_mV();
  pv_array[BUS_VOLTAGE_INDEX] = pv.getBusVoltage_V();
  pv_array[CURRENT_MA_INDEX] = pv.getCurrent_mA();
  pv_array[POWER_MW_INDEX] = pv.getPower_mW();
  pv_array[LOAD_VOLTAGE_INDEX] = pv_array[BUS_VOLTAGE_INDEX] + (pv_array[SHUNT_VOLTAGE_INDEX] / 1000);
}

//6.7 Read Data from INA219 (BATTERY)
void read_battery_data(void) {
  battery_array[SHUNT_VOLTAGE_INDEX] = battery.getShuntVoltage_mV();
  battery_array[BUS_VOLTAGE_INDEX] = battery.getBusVoltage_V();
  battery_array[CURRENT_MA_INDEX] = battery.getCurrent_mA();
  battery_array[POWER_MW_INDEX] = battery.getPower_mW();
  battery_array[LOAD_VOLTAGE_INDEX] = battery_array[BUS_VOLTAGE_INDEX] + (battery_array[SHUNT_VOLTAGE_INDEX] / 1000);
}

//6.8 setup_digital_IO
void setup_digital_IO(void) {
  pinMode (LED, OUTPUT);
}

//6.9 blinking LED
void blink_LED(void) {
  static uint32_t kLedTick;
  if (kLedTick < millis()) {
    kLedTick = millis() + 200;
    digitalWrite (LED, digitalRead(LED) ^ 1);
  }
}

//=========================== SETUP =================================
void setup() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  Serial.begin(115200);
  setup_wifi();
  setup_servo();
  setup_rtc();
  setup_ina219();
  setup_digital_IO();
}

//========================== LOOP ===================================
void loop() {
  wm.process();
  blink_LED();
  // put your main code here, to run repeatedly:
  if (timeUpdate_task1  < millis()) {
    timeUpdate_task1 = millis() + INTERVAL_TASK1;
    //Application for Task 1
    if (ina219_isOK) {
      read_pv_data();
      read_battery_data();
      Vpv = pv_array[LOAD_VOLTAGE_INDEX];
      Ipv = pv_array[CURRENT_MA_INDEX];
      Vbat = battery_array[LOAD_VOLTAGE_INDEX];
      Ibat = battery_array[CURRENT_MA_INDEX];
      update_gsheet();
    }
  }
  if (timeUpdate_task2 < millis()) {
    static uint8_t pos;
    static uint8_t kHour;
    timeUpdate_task2 = millis() + INTERVAL_TASK2;
    //Application Task 2
    kHour = rtc.getHour(true);
    if (kHour >= HOUR_START && kHour <= HOUR_STOP) {
      pos = PV_POSITION_ARRAY[kHour - INDEX_OFFSET];
    }
    else if (kHour >= 0 && kHour < HOUR_START) {
      pos = PV_POSITION_ARRAY[0];
    }
    else if (kHour > HOUR_STOP && kHour <= 23) {
      pos = PV_POSITION_ARRAY[8];
    }
    Serial.println(pos);
    myservo.write(pos);
  }
  if(timeUpdate_task3 < millis()){
    timeUpdate_task3 = millis() + 1000;
    Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));   // (String) returns time with specified format 
  }
}
//=========================== END LOOP =============================
