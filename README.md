# esp8266_weatherstation
NodeMCU esp8266 Weatherstation with multiple Sensors, outdoor and indoor.

This Arduino Project is a NodeMCU (ESP8266) based Weatherstation, with the following features:

- imlemented Sensors:
  - DHT22 (Temp & Humidity)
  - Sensirion SHT31 (Temp & Humidity) My Favorite
  - MCP9808 (Temp)
  - Bosch BMP280 (Air Pressure & Temp)
  - MH-Z19B (CO2 Sensor) <20$ AliExpress
  - DS18B20 (Temp) Be careful, this sensor has Problems with self-heating, needs to be soldered with very short legs
  
- Implemented Services:
  - Wunderground (upload Data to your personal Weatherstation)
  - Wunderground (download Data from any personal Weatherstation)
  - Pushingbox (sent Push Notifications to your mobile device
  - MQTT (Publish your Measurements to your MQTT Broker)
  - NTP CLient (get current time from a NTP Server)
  
- Implemented features:
  - Automatic Summertime / Wintertime change
  - Code works also, if no Sensor is connected, or only a few
  - Calculate Dew Point
  - Sent automatic Status Information via Pushingbox
  - Automatically select WiFi with best signal strenght
      
    
You will need the following Arduino Libraries (no BETA!):
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
- D1 PIN_RX       Connect MH-Z19B TX Pin to NodeMCU Pin D1 (Supply: 5V)
- D2 PIN_TX       Connect MH-Z19B RX Pin to NodeMCU Pin D2 (Supply: 5V) 
- D4 DHTPIN       Connect DHT22 to NodeMCU Pin D4          (Supply: 3.3V; do not try 5V)
- D5 SCL_PIN      Connect i2C Clock Pin to NodeMCU Pin D5  (All Connected Sensors: Supply: 3.3V)
- D6 SDA_PIN      Connect i2C Data Pin to NodeMCU Pin D6   (All Connected Sensors: Supply: 3.3V)
- D7 ONE_WIRE_PIN Connect MAXIM DS18B20 to NodeMCU Pin D7  (Supply: 3.3V)
