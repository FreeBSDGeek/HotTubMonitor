/*
 * HOT TUB TEMPERATURE CONTROLLER
 * 
 * This program reads the temperature of a hot tub and then relays both it's internal
 * curcuit board tempature as well as the hot tub water temperarture to an MQTT server.
 * 
 * Written By; Kirk Davis
 * Date: 15 Aug 2018
 * 
 * Version: 1.x
 *  - Inital version that sent the tempature data to the Adafruilt IO Cloud.
 *  - Built on the Adafruit MQTT library.
 * 
 * Version 2.x
 *  - Moved the MQTT code ovdr to a genewric MQTT library so that we can push the
 *  data directly ot a local MQTT server.
 *  - Allowed for setting the MQTT topics that get updated.
 *  - Allowed for subscribing to a control channel (although not used yet)
 *  
 * Version 3.x
 *  - Added in WiFiManager to allow setting of WiFi connection on startup.
 *  - Detects boardSensor on startup (if external sensor is not plugged in) and
 *  stores this sensor address in EEPROM. 
 *  - Added in the code for Double Reset so that you can factory reset the board
 *  without loading in new code.
 * 
 * Version 3.3.x
 *  - Move the config variables into a Json object that is stored in Flash.
 *  - Move the parameters for the MQTT connection into WiFiManager so it can 
 *  be configured on the main portal setup screen.
 *  - More code cleanup.
 * 
 *  Patch 3.3.3 This is a bugfix release
 *    - Fixed portal AP mode getting stuck and advertising AP SSID always
 *    - Stopped the DoubleReset form detecting when we reset after the AP timeout
 *    - Added debug macros so that code is smaller when debugging is not needed.
 *    - The MQTT Topics are now settable (and change for debugging).
 *    
 * Version 4.0.0
 *    - RE-write to create two one wire bus'.  One for each sensor.  This will allow
 *    for easier handling of sensor detection and no need for a sensor setup.
 *    
 */

#define __DEBUG__ // Comment out to turn off debuging

#ifdef __DEBUG__
#define DEBUG(s)   { Serial.print(F(s)); }
#define DEBUGVAR(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define DEBUGHEX(s,v) { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); }
#else
#define DEBUG(s)
#define DEBUGVAR(s,v)
#define DEBUGHEX(s,v)
#endif
 
#include <FS.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Base64.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MQTT.h"                 // arduino-mqtt 2.5.0
#include <EEPROM.h>
#include <DoubleResetDetector.h>
#include <stdlib.h>

// function prototype (only required for ESP8266 boards)
void connect(void);


/************************* Status LED(s)  ***********************************/
#define STATUS_PIN  0

/************************* Double Reset  ************************************/
#define DRD_TIMEOUT 2   // Timeout in seconds for a double reset
#define DRD_ADDRESS 0   // location in EEPROM to store the Double Reset flag

/************************* One Wire Temp DS18B20 ****************************/
#define ONEWIRE_BOARD_BUS 2
#define ONEWIRE_WATER_BUS 12

/****************************************************************************/
/****************** (you don't need to change this!) ************************/

// The sensor address is depermined at startup. This is just a place holder.
DeviceAddress waterSensorAddr = {0x28, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
DeviceAddress boardSensorAddr = {0x28, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/**************************** MQTT Setup ***********************************/
#ifdef __DEBUG__
#define BOARD_TOPIC "debug/in/hottub_board_temp/state"
#define WATER_TOPIC "debug/in/hottub_water_temp/state"
#else
#define BOARD_TOPIC "sensors/in/hottub_board_temp/state"
#define WATER_TOPIC "sensors/in/hottub_water_temp/state"
#endif

char mqtt_server[40] = "mqtt.example.com";
char mqtt_port[6] = "1883";
char mqtt_user[20] = "";
char mqtt_pass[32] = "";

// Setup the MQTT client.  The MQTT client handles all MQTT traffic.
MQTTClient mqtt;

/**************************** WiFi Setup ***********************************/
//     Setup the WiFi client.
WiFiClient client;
//     Flag to know when the config was updates and should be saved.
bool shouldSaveConfig = false;

/************************ Double Reset Setup *******************************/
// Setup the double reset client
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

/********** Callback routing from WiFiManager to save Config  **************/
void saveConfigCallback() {
  DEBUG("Should save config....\n");
  shouldSaveConfig = true;
}

/************** Call back when entering AP/Config mode *********************/
void configModeCallback() {
  DEBUG("Entering Captive Portal AP Mode\n")
  Serial.println(F("Captive Portal Setup mode..."));
  Serial.println(WiFi.softAPIP());
}

/*********** Save the config to the Flash File System  *********************/
void saveConfig() {

  DEBUG("Saving Config from SPIFFS\n");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_pass"] = mqtt_pass;    
  
  JsonArray& boardSensorValues = json.createNestedArray("board_sensor");
  for (uint8_t i = 0; i < 8; i++) {
    boardSensorValues.add(boardSensorAddr[i]);
  }
  
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println(F("failed to open config file to writing."));
  }

  // json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();

}

/*********** Restore the config from the Flash File System  *******************/
void readConfig() {
  
  if (SPIFFS.begin()) {
    
    // mounted file system
    if (SPIFFS.exists("/config.json")) {
      DEBUG("Reading Config from SPIFFS\n");
      
      //file exists, reading and loading
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
  
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          // json.printTo(Serial);
  
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
    
          for (uint8_t i = 0; i < 8; i++) {
            boardSensorAddr[i] = json["board_sensor"][i];
          }

        } 
        configFile.close();
      }
    }
  } else {
    DEBUG("failed to mount SPIFFS\n");
  }
    //end read
}

  
/*********** Connect to the wireless AP and then the MQTT server *************/
void connect() {
  char ClientID[32];
  
  Serial.print(F("Checking wifi connection..."));
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(F("connected!"));
  
  Serial.print(F("Connected to "));
  Serial.print(WiFi.SSID());
  Serial.print(F(" with IP address "));
  Serial.println(WiFi.localIP());

  strcpy(ClientID, "HotTub_");
  strcat(ClientID, WiFi.localIP().toString().c_str());

  Serial.print(F("Checking MQTT connection..."));
  while (!mqtt.connect( ClientID, mqtt_user, mqtt_pass )) {
    Serial.print(F("."));
    delay(500);
  }
  Serial.println(F("connected!"));
}

/**********  Callback routine when a message is received.  **************/
void messageReceived(String &topic, String &payload) {
  Serial.println("Incoming: " + topic + " - " + payload);
}

/**********  Print a one wire device address ****************************/
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

/********** Compare two one-wire addresses to see if they match. ********/
bool compareAddress(DeviceAddress targetAddress, DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (targetAddress[i] != deviceAddress[i]) {
      return false;
    }
  }
  return true;
}

/*********** Check and see if the sensor is connected to the system. *****/
bool checkForAddress(DallasTemperature sensors, DeviceAddress checkAddr) {
  int found = -1;
  DeviceAddress tempAddr;
  for (uint8_t i = 0; i < sensors.getDeviceCount(); i++) {
    sensors.getAddress(tempAddr,i);
    if ( compareAddress( checkAddr, tempAddr )) {
      return true;  // True
    }
  }
  return false;  // False
}

/*********** Stop the program on an error *******************************/
void stop() {

  while (1) {
    drd.loop(); 
    drd.stop();  
    digitalWrite(STATUS_PIN, HIGH);
    delay(500);
    digitalWrite(STATUS_PIN, LOW);
    delay(500);
  }
}

/***********************************************************************/
// Setup OneWire Instance to communicate to the DS18B20 sensor.
OneWire oneWireBoard(ONEWIRE_BOARD_BUS);
OneWire oneWireWater(ONEWIRE_WATER_BUS);

// Create the sensor by passing a reference to the oneWire instance we created.
DallasTemperature boardSensor(&oneWireBoard);
DallasTemperature waterSensor(&oneWireWater);

/************** Setup Routing Starts Here *****************************/
void setup() {
  // Serial port Setup
  Serial.begin(115200);
  
  // Statrus LED Setup
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);
  delay(50);

  DEBUG("\n\n")
  Serial.println();
  Serial.println(F("ESP8266 HotTub MQTT Controller"));
  Serial.println(F("Version: 4.0.0-alpha   August 2022"));

  // Grab the config stored in the SPIFFS File system.
  readConfig();
  
  // Setup the hostname and connect to the wifi
  WiFi.mode(WIFI_STA);
  WiFi.hostname("hottub-controller-test");

  // Setup the WiFi Manager client. This is responsable for the web portal to setup Wireless.
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt username", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt password", mqtt_pass, 32);
  
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  // Allow for a pin to reset all the WiFi Setup information.
  // The is similar to factory reset as it will boot into AP mode. 
  if ( drd.detectDoubleReset() ) {
    Serial.println(F("Double Reset Detected..."));
    Serial.println(F("Reseting the Board Config back to factory."));
    
    strcpy(mqtt_server, "mqtt.example.com");
    strcpy(mqtt_port, "1883");
    strcpy(mqtt_user, "");
    strcpy(mqtt_pass, "");
    saveConfig();
    
    delay(100);
    Serial.println(F("Reseting the WiFi Settings."));
    wifiManager.resetSettings();
    delay(500);
    drd.stop();
  }

  // Start and try to connect to the network.
  // If not connected, then wait in AP mode for 180 seconds then reset

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  DEBUG("Entering Config Portal Mode.")
  wifiManager.setTimeout(300);
  if(!wifiManager.autoConnect("HotTub-AP")) {
    DEBUG("Config Portal Times out.  Resetting...\n")
    drd.stop();
    ESP.reset();
    delay(1000);
  }

  // If HERE then we are connected up to ATA mode to a wifi AP
  DEBUG("Configured and connected to a WiFi AP\n")
  
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());

  DEBUGVAR( "\nMQTT Server : ", mqtt_server ); DEBUG("\n");
  DEBUGVAR( "MQTT Port   : ", mqtt_port ); DEBUG("\n");
  DEBUGVAR( "MQTT User   : ", mqtt_user ); DEBUG("\n");
  DEBUGVAR( "MQTT Pass   : ", mqtt_pass ); DEBUG("\n")

   
  Serial.print(F("Board Sensor : "));
  printAddress(boardSensorAddr);
  Serial.println();

  if (shouldSaveConfig) {
    saveConfig();
  }
  
  // Setup the MQTT connection.
  mqtt.begin(mqtt_server, client);
  mqtt.onMessage(messageReceived);

  DEBUG("\n\n")
  DEBUG("Publishing Board Temparature on : ")
  DEBUG(BOARD_TOPIC)
  DEBUG("\n")
  DEBUG("Publishing Water Temparature on : ")
  DEBUG(BOARD_TOPIC)
  DEBUG("\n\n")
  
  connect();

  // Setup the Tempature Sensor
  boardSensor.begin();
  waterSensor.begin();

  int sensorCount = boardSensor.getDeviceCount();
  DEBUGVAR("Found ", sensorCount);
  DEBUG(" devices on the board sensor bus.\n")

  // Check to see if the board sensor is responding.
  if (! checkForAddress(boardSensor, boardSensorAddr)) {
    if ( sensorCount > 1 ) {
      Serial.println(F("\nFound more than one sensor on the Board Sensor Bus. This is an error."));
      Serial.println(F("There should only be one board sensor. This may be an earlier version "));
      Serial.println(F("of the monitor board."));
      Serial.println(F("This needs to be fixed before we can continue. Stopping the Monitor."));
      stop();
    }
    

    if ( sensorCount == 1 ) {
      boardSensor.getAddress(boardSensorAddr, 0);
      Serial.print(F("\nFound the board sensor with the following address: "));
      printAddress(boardSensorAddr);
      Serial.println();
      
      // Found a new board sensor.  Update the saved config.
      saveConfig();
      
      DEBUG("Updating the config in flash... ");
    }
    
    // Opps, wasn't able to find any sensors.  Display an error and halt.
    if ( sensorCount < 1 ) {
      Serial.println(F("\nERROR: Unable to locate a board sensor.  There should be one on the board so this is a big problem."));
      Serial.println(F("The sensor board is in need of repair. Stopping the Monitor."));
      stop();
    }
  }

  sensorCount = waterSensor.getDeviceCount();
  DEBUGVAR("Found ", sensorCount);
  DEBUG(" devices on the water sensor bus.\n")
  
  if ( sensorCount < 1 ) {
    Serial.println(F("\nERROR: Unable to locate a water sensor.  Did you plug it in?"));
    Serial.println(F("If the water sensor is plugged in then there is a hardware error"));
    Serial.println(F("and the sendor board is in need of repair.  If you just fogot to"));
    Serial.println(F("plug in the water sensor, then turn the monitor off, plug the sensor"));
    Serial.println(F("and turn it back on.  Stopping the Monitor!"));
    stop();
  }
  Serial.println(F("Checking for board and water Sensors..." ));  
  // We know what the boardSensor is so we can now figure out what the
  // water sensor is.
  DeviceAddress tempAddr;
  for (int i=0; i < sensorCount; i++) {
    waterSensor.getAddress(tempAddr,i);
    if ( ! compareAddress( boardSensorAddr, tempAddr )) {
      memcpy(waterSensorAddr, tempAddr, sizeof(tempAddr));
    }
  }

  if ( boardSensor.validAddress(boardSensorAddr) ) {
    Serial.print(F("Board Sensor Address: "));
    printAddress(boardSensorAddr);
    Serial.println();
  }

  if ( waterSensor.validAddress(waterSensorAddr) ) {
    Serial.print(F("Water Sensor Address: "));
    printAddress(waterSensorAddr);
    Serial.println();
  }
    
  // sensors.setResolution(tempSensor, 12);
  // sensors.setWaitForConversion(false);

  DEBUG("\n\n")
  delay(100);  
}

/************** The MAIN execution loop starts here *************************/
char  buff[6];
long  lastMsg = 0;
float oldTemp0 = 0;
float oldTemp1 = 0;
float temp = 0;

void loop() {

  // Loop to check for Double Reset Timeout.
  drd.loop();
  
  // Start the MQTT loop to monitor the MQTT connection.
  mqtt.loop();
  delay(10);
  
  if (!mqtt.connected()) {
    connect();
  }

  // Get the current time in milliseconds. We only want to update every min.
  long now = millis();
  if (now - lastMsg > 60000) {        // Only send an update every Min.
    lastMsg = now;

    // ******** Board Sensor Polling *******
    boardSensor.requestTemperatures();

    // Record the board temperature but only if it is available.
    if ( checkForAddress(boardSensor, boardSensorAddr) ) {
      
      temp = boardSensor.getTempF(boardSensorAddr); 
      dtostrf( temp, 5, 2, buff);
      if (! mqtt.publish(BOARD_TOPIC, buff)) { // Publish the value and check if OK
        Serial.println(F("Failed update"));
      } else {
        DEBUGVAR("Board Temp : ", temp)
        Serial.println(F(" update successful!"));
      }
      temp = 0;
    } else {
      Serial.println(F("Unable to detect the boardSensor."));
    }

    // ******** Water Sensor Polling *******
    waterSensor.requestTemperatures();

    // Record the water sensor but only if it is available.
    if ( checkForAddress(waterSensor, waterSensorAddr)) {
      
      temp = waterSensor.getTempF(waterSensorAddr);  

      dtostrf( temp, 5, 2, buff);        
      if (! mqtt.publish(WATER_TOPIC, buff)) { // Publish the value and check if OK
        Serial.println(F("Failed update"));
      } else {
        DEBUGVAR("Water Temp : ", temp)
        Serial.println(F(" update successful!"));
      }

      temp = 0;
    } else {
      Serial.println(F("Unable to detect the waterSensor."));
    }
  }

}
