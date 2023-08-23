# esp8266_weatherstation
NCode is used with an ESP8266.
Intension is to have a Weatherstation with multiple Sensors, 
Additionally i have many indoor Sensors (per room in my house).

This Arduino Project is a NodeMCU (ESP8266) based Weatherstation, with the following features:

- imlemented Sensors:
  - DHT22 (Temp & Humidity)
  - Sensirion SHT31 (Temp & Humidity) My Favorite
  - MCP9808 (Temp)
  - Bosch BMP388 (Air Pressure & Temp)
  - MH-Z19B (CO2 Sensor) <20$ AliExpress
  - DS18B20 (Temp) Be careful, this sensor has Problems with self-heating, needs to be soldered with very short legs
  - ST VL53L0 proximitySensor
  - ST VL6180 proximitySensor
  - Wind Speed Sensor
  - AMS BH1750 (light Sensor)
 
- Implemented Dispaly (for Indoor use)
  -   SSD1306 (OLED 128x64 pixel)
 
-   Implemented Air condition control (via IR LED)
  -     Midea
  
- Implemented Services:
  - Home Assistant Auto Discovery
  - Wunderground (upload Data to your personal Weatherstation)
  - Pushingbox (sent Push Notifications to your mobile device
  - MQTT (Publish your Measurements to your MQTT Broker)
  - NTP CLient (get current time from a NTP Server)

  
- Implemented features:
  - Automatic Summertime / Wintertime change
  - Code works also if no Sensor is connected, or only a few
  - Calculate Dew Point
  - Sent automatic Status Information via Pushingbox
  - Automatically select WiFi with best signal strenght
      
    
You will need the following Arduino Libraries:
- ESP8266WiFi.h
- DHT.h
- OneWire.h
- DallasTemperature.h
- WiFiAutoSelector.h //https://gist.github.com/AndiSHFR/e9c46890af7cddff6cb5ea7d4f1c5c49
- Wire.h
- Adafruit_SHT31.h
- Adafruit_BMP280.h
- Adafruit_MCP9808.h
- NTPClient.h
- WiFiUdp.h
- TimeLib.h
- ArduinoJson.h
- PubSubClient.h
- mhz19.h
- SoftwareSerial.h


Pinning of your NodeMCU pins where your sensors are connected:
D1    //Connect Speed Sensor (Wind Sensor)
D2    //Connect Pulse Sensor (Energy Sensor)
D4    // this Connects DHT22 to NodeMCU Pin D4
D5    //defines the i2C Clock Pin on D5
D6    //defines the i2C Data Pin on D6
D7    //this is PWM input for MHZ19-B Co2 Sensor to NodeMCU Pin D7
D8    // this Connects MAXIM DS18B20 to NodeMCU Pin D8


No Power Saving Modes are implemented, so Powering without batteries is recommended.

Have fun
