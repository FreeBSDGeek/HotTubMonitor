# HotTubMonitor
ESP8266 Arduino Wireless sketch to monitor the temperature of a HotTub and report via MQTT

I started this project so I could have a way to monitor the temperature of my Hot Tub.  I live in Canada and it gets cold here in the winter.  If something happened then things could freeze up before I noticed any issues.  This project monitors the temperature and sends the updates (via MQTT) to a Home Assistant server. I have alerts setup in Home Assistant so it can alert me if the tempature is not correct and I can take some action. I use ([Home Assistant](https://www.home-assistant.io/)) for all my smart home devices but any MQTT enabled backend will work, (for example OpenHAB).

The temperature monitor is created using a ESP8266 Arduino board ([Adafruit 8266 Hazzah](https://www.adafruit.com/product/2471)) and two Dallas OneWire (DS18B20) temperature sensors. I use one of the sensors to monitor the board temperature (inside the enclosure) an another one as a probe that sits in the hot tub.

**Note:** as of Version 4.x the board design has changed.  It now puts the sensors of different One Wire bus' so that we no longer need to probe the sensors on setup.  This is a small hardware design change to maek the usability a lot easier.  If you have a version 3.x (or before) monitor, make sure to change the cercuit before using this code.

### Dependancies

There are a few library dependencies that need to be installed:
* [Dallas Temperature Sensor Library](https://github.com/milesburton/Arduino-Temperature-Control-Library)
* [OneWire Library](https://www.pjrc.com/teensy/td_libs_OneWire.html)
* [ArduinoJson Library](https://arduinojson.org/?utm_source=meta&utm_medium=library.properties)
* [DoubleResetDetector Library](https://github.com/datacute/DoubleResetDetector)
* [WiFiManager Library](https://github.com/tzapu/WiFiManager)
* [MQTT Library](https://github.com/256dpi/arduino-mqtt)

All the dependancies can be installed from the Library Manager in the Arduino 1.8.19 client.

### Setup

##### Wireless
Once the sketch is complied and loaded into your ESP8266, it will boot into AP mode.  Take a wireless device (like a cell phone) and connect to it's wireless access point portal (SSID: HotTub-AP).  It should redirect you automatically to a web page.  Click on "WiFi" and it will scan for access points that are close and display the SSIDs at the top on the page.  Click on the SSID and then enter the wifi password.  On the same screen, it also asks for the MQTT server (this is the hostname for my Home Assistant server), MQTT username, and MQTT password.  Leave the username and password blank if you do not have authentication enabled on your MQTT server.  Once you save this configuration, it will turn off the access point and connect up to the wireless you selected.

##### Sensors
There is no longer a need to do any sensor setups.  As of version 4.x, the monitor code will know what sensor is the board sensor and what one is for the water. Make sure the water sensor is plugged in when the monitor is turned on. 

### Operation

The monitor will send both the board and water temperature values every minute.  The MQTT Topics are hard coded into the sketch right now (I hope to make them configurable in a future version), and are as follows:

* Board Temperature Topic: sensors/in/hottub_board_temp/state
* Water Temperature Topic: sensors/in/hottub_water_temp/state

