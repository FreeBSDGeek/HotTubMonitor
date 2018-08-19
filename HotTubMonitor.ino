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
 */

#include <FS.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Base64.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MQTT.h"
#include <EEPROM.h>
#include <DoubleResetDetector.h>
#include <stdlib.h>

// function prototype (only required for ESP8266 boards)
void connect(void);


/************************* Status LED(s)  ************************************/
#define STATUS_PIN  0

/************************* Double Reset  ************************************/
#define DRD_TIMEOUT 2   // Timeout in seconds for a double reset
#define DRD_ADDRESS 0   // location in EEPROM to store the Double Reset flag

/************************* One Wire Temp DS18B20 *****************************/
#define ONE_WIRE_BUS 2

// do not edit the water sensor.  It is depermined at startup. This is just a place holder.
DeviceAddress waterSensor = {0x28, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
DeviceAddress boardSensor = {0x28, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/************ Global State (you don't need to change this!) ******************/

char mqtt_server[40] = "mqtt.example.com";
char mqtt_port[6] = "1883";
char mqtt_user[20] = "";
char mqtt_pass[32] = "";

bool shouldSaveConfig = false;

// Setup the WiFi client.  The WiFi client handles all the wireless communication.
WiFiClient client;

// Setup the MQTT client.  The MQTT client handles all MQTT traffic.
MQTTClient mqtt;

// Setup the double reset client
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

/*********** Callback routing from WiFiManager to save Config  **************/
void saveConfigCallback() {
  Serial.println("Should save config....");
  shouldSaveConfig = true;
}

/*********** Save the config to the Flash File System  *********************/
void saveConfig() {

  Serial.println("saving the custom config parameters...");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_pass"] = mqtt_pass;    
  
  JsonArray& boardSensorValues = json.createNestedArray("board_sensor");
  for (uint8_t i = 0; i < 8; i++) {
    boardSensorValues.add(boardSensor[i]);
  }

  Serial.print("DEBUG: Saved the boardSensor Address ");
  printAddress(boardSensor);
  Serial.println(" to json config.");
  
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file to writing.");
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();

}

/*********** Restore the config from the Flash File System  *******************/
void readConfig() {
  
  if (SPIFFS.begin()) {
    
    // mounted file system
    if (SPIFFS.exists("/config.json")) {
      
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
  
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          json.printTo(Serial);
  
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
    
          // JsonArray& boardSensorValues - json.parseNestedArray("board_sensor");
          for (uint8_t i = 0; i < 8; i++) {
            boardSensor[i] = json["board_sensor"][i];
          }
          // memcpy(boardSensor, json["board_sensor"], sizeof(boardSensor));

          Serial.print("DEBUG: Read the boardSensor Address  : ");
          printAddress(boardSensor);
          Serial.println(" from the Json config file.");
        } 
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
    //end read
}

  
/*********** Connect to the wireless AP and then the MQTT server *************/
void connect() {
  Serial.print("Checking wifi connection...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("connected!");
  
  Serial.print("Connected to ");
  Serial.print(WiFi.SSID());
  Serial.print(" with IP address ");
  Serial.println(WiFi.localIP());

  Serial.print("Checking MQTT connection...");
  while (!mqtt.connect( mqtt_server, mqtt_user, mqtt_pass )) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("connected!");
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

/*********** Check abnd see if the sensor is connected to the system. *****/
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
    digitalWrite(STATUS_PIN, HIGH);
    delay(500);
    digitalWrite(STATUS_PIN, LOW);
    delay(500);
  }
}

/***********************************************************************/
// Setup OneWire Instance to communicate to the DS18B20 sensor.
OneWire oneWire(ONE_WIRE_BUS);

// Create the sensor by passing a reference to the oneWire instance we created.
DallasTemperature sensors(&oneWire);

/************** Setup Routing Starts Here *****************************/
void setup() {
  // Serial port Setup
  Serial.begin(115200);
  // Statrus LED Setup
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);
  delay(500);
  
  Serial.println(F("ESP8266 HotTub MQTT Controller"));
  Serial.println(F("Version: 3.3.2   August 2018"));

  // Grab the config stored in the SPIFFS File system.
  readConfig();
  
  // Setup the hostname and connect to the wifi
  WiFi.hostname("hottub-controller-test");

  // Setup the WiFi Manager client. This is responsable for the web portal to setup Wireless.
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt username", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt password", mqtt_pass, 32);
  
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
  
  // Allow for a pin to reset all the WiFi Setup information.
  // The is similar to factory reset as it will boot into AP mode. 
  if ( drd.detectDoubleReset() ) {
    Serial.println("Double Reset Detected...");
    Serial.println("Reseting the Board Config back to factory.");
    
    strcpy(mqtt_server, "mqtt.example.com");
    strcpy(mqtt_port, "1883");
    strcpy(mqtt_user, "");
    strcpy(mqtt_pass, "");
    saveConfig();
    
    delay(100);
    Serial.println("Reseting the WiFi Settings.");
    wifiManager.resetSettings();
    delay(500);
    drd.stop();
  }

  // Start and try to connect to the network.
  // If not connected, then wait in AP mode for 180 seconds then reset

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  
  wifiManager.setTimeout(180);
  if(!wifiManager.autoConnect("AutoConnectAP")) {
    ESP.reset();
    delay(1000);
  }

  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());

  Serial.print( "MQTT Server : " );
  Serial.println( mqtt_server );
  Serial.print( "MQTT port   : ");
  Serial.println( mqtt_port );
  Serial.print( "MQTT user   : ");
  Serial.println( mqtt_user );
  Serial.print( "MQTT pass   : ");
  Serial.println( mqtt_pass );
  Serial.print("Board Sensor : ");
  printAddress(boardSensor);
  Serial.println();

  if (shouldSaveConfig) {
    saveConfig();
  }
  
  // Setup the MQTT connection.
  mqtt.begin(mqtt_server, client);
  mqtt.onMessage(messageReceived);

  connect();

  // Setup the Tempature Sensor
  sensors.begin();

  int sensorCount = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(sensorCount, DEC);
  Serial.println(" devices.");

  // Check to see if the board sensor is responding.
  if (! checkForAddress(sensors, boardSensor)) {
    Serial.println("\nERROR: Unable to locate the board sensor.");
    if ( sensorCount > 1 ) {
      Serial.println("Unplug any external sensors and restart the device.");
      stop();
    }
    
    // We only have one sensor so it must be the board sensor.
    if ( sensorCount == 1 ) {
      sensors.getAddress(boardSensor, 0);
      Serial.print("\nFound one sensor with the following address: ");
      printAddress(boardSensor);
      Serial.println();
      
      // Found a new board sensor.  Update the saved config.
      saveConfig();
      
      Serial.print("Updating the config in flash... ");
      Serial.println("You can now connect the remote sensor and reboot."); 
      stop();
    }
    
    // Opps, wasn't able to find any sensors.  Display an error and halt.
    if ( sensorCount < 1 ) {
      Serial.println("\nERROR: Unable to locate any sensors.  There should be one on the board so this is a big problem.");
      Serial.println("The sensor board is in need of repair.");
      stop();
    }
  }

  Serial.println("Checking for board and water Sensors..." );  
  // We know what the boardSensor is so we can now figure out what the
  // water sensor is.
  DeviceAddress tempAddr;
  for (int i=0; i < sensorCount; i++) {
    sensors.getAddress(tempAddr,i);
    if ( ! compareAddress( boardSensor, tempAddr )) {
      memcpy(waterSensor, tempAddr, sizeof(tempAddr));
    }
  }

  if ( sensors.validAddress(boardSensor) ) {
    Serial.print("Board Sensor Address: ");
    printAddress(boardSensor);
    Serial.println();
  }

  if ( sensors.validAddress(waterSensor) ) {
    Serial.print("Water Sensor Address: ");
    printAddress(waterSensor);
    Serial.println();
  }
    
  // sensors.setResolution(tempSensor, 12);
  // sensors.setWaitForConversion(false);

  delay(500);  
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
    
    sensors.requestTemperatures();

    // Record the board temperature but only if it is available.
    if ( checkForAddress(sensors, boardSensor) ) {
      
      temp = sensors.getTempF(boardSensor); 
      dtostrf( temp, 5, 2, buff);
      if (! mqtt.publish("sensors/in/hottub_board_temp/state", buff)) { // Publish the value and check if OK
        Serial.println(F("Failed update"));
      } else {
        Serial.println(F("Successful update!"));
      }
      temp = 0;
    } else {
      Serial.println("Unable to detect the boardSensor.");
    }

    // Record the water sensor but only if it is available.
    if ( checkForAddress(sensors, waterSensor)) {
      
      temp = sensors.getTempF(waterSensor);  

      dtostrf( temp, 5, 2, buff);        
      if (! mqtt.publish("sensors/in/hottub_water_temp/state", buff)) { // Publish the value and check if OK
        Serial.println(F("Failed update"));
      } else {
        Serial.println(F("Successful update!"));
      }

      temp = 0;
    } else {
      Serial.println("Unable to detect the waterSensor.");
    }
  }

}

