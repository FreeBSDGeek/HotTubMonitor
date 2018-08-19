# HotTubMonitor
ESP8266 Arduino Wireless sketch to monitor the temperature of a HotTub and report via MQTT

I started this project so I could have a way to monitor the temperature of my Hot Tub.  I live in Canada and it gets cold here in the winter.  If something happened then things could freeze up before I noticed any issues.  This project monitors the temperature and updates me (through an OpenHAB server) if there is an issue.

The temperature monitor is created using a ESP8266 Arduino board (I used a Adafruit 8266 Hazzah) and two Dallas OneWire (DS18B20) temperature sensors. I use one of the sensors to monitor the board temperature (inside the enclosure) an another one as a probe that sits in the hot tub.

### Dependancies

There are a few library dependencies that need to be installed:
* [Dallas Temperature Sensor Library](https://github.com/milesburton/Arduino-Temperature-Control-Library)
* [OneWire Library](https://www.pjrc.com/teensy/td_libs_OneWire.html)
* [ArduinoJson Library](https://arduinojson.org/?utm_source=meta&utm_medium=library.properties)
* [DoubleResetDetector Library](https://github.com/datacute/DoubleResetDetector)
* [WiFiManager Library](https://github.com/tzapu/WiFiManager)
* [MQTT Library](https://github.com/256dpi/arduino-mqtt)

All the dependancies can be installed from the Library Manager in the Arduino 1.8.5 client.

### Setup

##### Wireless
Once the sketch is complied and loaded into your ESP8266, it will boot into AP mode.  Take a wireless device (like a cell phone) and connect to it's wireless access point portal.  It should redirect you automatically to a web page.  Click on "WiFi" and it will scan for access points that are close and display the SSIDs at the top on the page.  Click on the SSID and then enter the wifi password.  On the same screen, it also asks for the MQTT server (this is the hostname for my OpenHAB server), MQTT username, and MQTT password.  Leave the username and password blank if you do not have authentication enabled on your MQTT server.  Once you save this configuration, it will turn off the access point and connect up to the wireless you selected.

##### Sensors
The Monitor needs to detect what sensor is used for the board and what one is used for the remote water sensor.  This is a little more difficult with the onewire bus as there is no way to determine where the sensors are.  If it detects only one sensor, then the sketch will assume that is the board sen sensor and record that sensors address in the flash.  That way it will always know.

After setting up the wifi, unplug the external sensor (so that only the board sensor is connected) and reset the ESP8266 board.  Wait until the red light on the board starts flashing.  This happens after is has detected the board sensor.  Once it has done that, you can plug back in the remote water sensor and reset the board again.  You are ready to go.

### Operation

The monitor will send both the board and water temperature values every minute.  The MQTT Topics are hard coded into the sketch right now (I hope to make them configurable in a future version), and are as follows:

* Board Temperature Topic: sensors/in/hottub_board_temp/state
* Water Temperature Topic: sensors/in/hottub_water_temp/state

