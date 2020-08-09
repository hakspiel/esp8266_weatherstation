
//Included Libraries
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "WiFiAutoSelector.h" //https://gist.github.com/AndiSHFR/e9c46890af7cddff6cb5ea7d4f1c5c49
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_BMP3XX.h"
#include <Adafruit_MCP9808.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adafruit_MCP4725.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <BH1750.h>
//#include "Adafruit_VL6180X.h"
//#include "Adafruit_VL53L0X.h"
#include <VL53L0X.h>
#include <VL6180X.h>

// Define the Numer of this Client, every Client needs its own unique number, 
  #define Client_Number "10"


  // 0 = Outside Temperature 
  // 1 = Inside Sensor 1 Sleep
  // 2 = Inside Sensor 2 Office
  // 3 = Inside Sensor 3 Living
  // 4 = Inside Sensor 4 Lia
  // 5 = Inside Sensor 5 Fabio
  // 6 = Inside Sensor 6 rooflake
  // 7 = Inside Sensor 7 roofhill
  // 8 = Inside Sensor 8 Cellar
  // 9 = Inside Sensor 9 Garage
  //10 = Inside Sensor 10 Technique
  //11 = Inside Sensor 11 Bathroom
  //12 = Inside Sensor 12

// Define Updaterate of the whole loop
  #define REPORT_INTERVAL 29 // in sec; Cycle Time is approx 2 sec...
  
//WiFi Access:
  const char *ssid1     = "*****";
  const char *password1 = "*****";
  
  const char *ssid2     = "*****";
  const char *password2 = "*****";
  
  const char *ssid3     = "*****";
  const char *password3 = "*****";
    
// Wunderground (Download Data from Wundergound)
  #define WU_API_KEY "*****"                         // Create Account to download Data from Wunderground.com: https://www.wunderground.com/weather/api/d/pricing.html
  #define WUNDERGROUND "api.wunderground.com"       // DOcumentation of API Calls: https://www.wunderground.com/weather/api/d/docs
  #define WU_LOCATION "pws:IMECKENB4"               // Station ID you want to download data from

// Wunderground (Upload Data to your PWS at Wundergound)
  #define WUNDERGROUND2 "rtupdate.wunderground.com" //Create your own Weatherstation to upload Data to Wunderground.com: https://www.wunderground.com/personal-weather-station/signup.asp
  #define WUNDERGROUND_STATION_ID "*****"
  #define WUNDERGROUND_STATION_PASSWORD "*****"

// PushingBox scenario DeviceId code and API
  String deviceId = "*****";             // Anleitung Pushingbox: http://www.geekstips.com/android-push-notifications-esp8266-arduino-tutorial/
  const char *logServer = "api.pushingbox.com";

// MQTT: ID, server IP, port, username and password
  const char *MQTT_CLIENT_ID = "";
  const char *MQTT_CLIENT_ID0 = "Outdoor";
  const char *MQTT_CLIENT_ID1 = "Sleep";
  const char *MQTT_CLIENT_ID2 = "Office";
  const char *MQTT_CLIENT_ID3 = "Living";
  const char *MQTT_CLIENT_ID4 = "Lia";
  const char *MQTT_CLIENT_ID5 = "Fabio";
  const char *MQTT_CLIENT_ID6 = "Rooflake";
  const char *MQTT_CLIENT_ID7 = "Roofhill";
  const char *MQTT_CLIENT_ID8 = "Cellar";
  const char *MQTT_CLIENT_ID9 = "Garage";
  const char *MQTT_CLIENT_ID10 = "Technique";
  const char *MQTT_CLIENT_ID11 = "Bathroom";
  const char *MQTT_CLIENT_ID12 = "Sensor12";
  
  const char *MQTT_SERVER_IP = "*****"; //Hass.io
  uint16_t MQTT_SERVER_PORT = 1883;
  const char *MQTT_USER = "*****";
  const char *MQTT_PASSWORD = "*****";

// MQTT: topic
  const char *MQTT_SENSOR_TOPICS0 = "outdoor/sensor1";
   const char *MQTT_SENSOR_TOPICS1 = "indoor/sensor1";
  const char *MQTT_SENSOR_TOPICS2 = "indoor/sensor2";
  const char *MQTT_SENSOR_TOPICS3 = "indoor/sensor3";
  const char *MQTT_SENSOR_TOPICS4 = "indoor/sensor4";
  const char *MQTT_SENSOR_TOPICS5 = "indoor/sensor5";
  const char *MQTT_SENSOR_TOPICS6 = "indoor/sensor6";
  const char *MQTT_SENSOR_TOPICS7 = "indoor/sensor7";
  const char *MQTT_SENSOR_TOPICS8 = "indoor/sensor8";
  const char *MQTT_SENSOR_TOPICS9 = "indoor/sensor9";
  const char *MQTT_SENSOR_TOPICS10 = "indoor/sensor10";
  const char *MQTT_SENSOR_TOPICS11 = "indoor/sensor11";
  const char *MQTT_SENSOR_TOPICS12 = "indoor/sensor12";

// MQTT Version   
  #define MQTT_VERSION MQTT_VERSION_3_1_1

//for wifi auto selector
  #define WIFI_CONNECT_TIMEOUT 8000 
  WiFiAutoSelector wifiAutoSelector(WIFI_CONNECT_TIMEOUT);  

//Definition of your Node MCU pins where your sensors are connected
  #define input_pin1 D1    //Connect Speed Sensor (Wind Sensor)
  #define input_pin2 D2    //Connect Pulse Sensor (Energy Sensor)
  #define DHTPIN D4       // this Connects DHT22 to NodeMCU Pin D4
  #define SCL_PIN D5      //defines the i2C Clock Pin on D5
  #define SDA_PIN D6      //defines the i2C Data Pin on D6
  #define pwm_pin D7      //this is PWM input for MHZ19-B Co2 Sensor to NodeMCU Pin D7
  #define ONE_WIRE_PIN D8 // this Connects MAXIM DS18B20 to NodeMCU Pin D8

//Definition of Wunderground Page
  const char WUNDERGROUND_REQ[] =
    "GET /api/" WU_API_KEY "/conditions/q/" WU_LOCATION ".json HTTP/1.1\r\n"
    "User-Agent: ESP8266/0.1\r\n"
    "Accept: */*\r\n"
    "Host: " WUNDERGROUND "\r\n"
    "Connection: close\r\n"
    "\r\n";
      
//Data that we want to extract from the Wunderground page
  struct UserData {
    char temp_c[32];
    char relative_humidity[32];
    char dewpoint_c[32];
    char observation_time_rfc822[32];
  };

//The callback function header needs to
//  be declared before the PubSubClient constructor and the
//  actual callback defined afterwards.
//  This ensures the client reference in the callback function
//  is valid.

// This is the "Callback function header"
void callback(char* topic, byte* payload, unsigned int length);

// this defines the Type of DHTxx Sensor
  #define DHTTYPE DHT22 
  const int temperaturePrecision = 12;// resolution for all DS18B20 devices (9, 10, 11 or 12 bits)

//initialize an instance of the each class
  WiFiClient  wificlient2; 
  WiFiClient  client2;
  OneWire oneWire(ONE_WIRE_PIN);
  DallasTemperature DS18B20(&oneWire); // pass oneWire reference to DallasTemperature
  DallasTemperature sensors(&oneWire);
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
  Adafruit_BMP3XX bme;
  DHT dht(DHTPIN, DHTTYPE);
  Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, "0.de.pool.ntp.org", 3600, 60000); // Die letzten 2 Werte: 1) Zeitversatz der aktuellen Zeitzone in sekunden; 2) Updateintervall in Sekunden 
  PubSubClient client(wificlient2);
  Adafruit_MCP4725 dac; 
  BH1750 lightMeter(0x23);
  BH1750 lightMeter2(0x5C);
    // ADDR Pin: If it has voltage greater or equal to 0.7*VCC voltage the sensor address will be0x5C. 
    // if ADDR voltage is less than 0.7 * VCC the sensor address will be 0x23 (by default).

  
//  Adafruit_VL6180X vl = Adafruit_VL6180X();
//  Adafruit_VL53L0X lox = Adafruit_VL53L0X();
  //VL53L0X VL53L0;
  VL6180X VL6180;
 
//Globale Variablen definieren
  float Temp2;
  float feuchte;
  float taupunktTmp;
  float Taupunkt_alt;
  float minh = 100;
  float maxh = 0;
  float mint = 100;
  float maxt = -100;
  float minsoilt = 100;
  String minh_time;
  String maxh_time;
  String mint_time;
  String maxt_time;
  String minsoilt_time;
  unsigned long time_new=0;
  unsigned long time_old=0;
  unsigned long time_new2=0;
  unsigned long time_old2=0;
  unsigned long time_new3=0;
  unsigned long time_old3=0;
  unsigned long time_new4=0;
  unsigned long time_old4=0;
  unsigned long time_new5=0;
  unsigned long time_old5=0;
  float temp_average=0;
  float hum_average=0;
  float Taupunkt_mittel = 0;
  float Absolute_Feuchte_mittel = 0;
  float atmospheric_pressure = 0;
  float compare_temp = 0;
  float maximtemp1 = 0;
  float maximtemp2 = 0;
  float dht_temp = 0;
  float sht31_temp = 0;
  float mcp9808_temp = 0;
  bool Mitteilungssperre = false;
  bool morgenwerte_uebermittelt = false;
  bool abendwerte_uebermittelt = false;
  bool fresh_resetted = true;
  bool mcp = true;
  int ausfaelle = 0;
  float lux = 0;
  float lux2 = 0;
  volatile unsigned long i_wind = 1;
  volatile unsigned long i_wind2 = 1;
  float wind_gust = 0;
  float wind_avg = 0;
  float wind_gust_max = 0;
  float wind_avg1 = 0;
  float wind_avg_avg = 0;
  int wind_collection=0;
  volatile unsigned long first_micros;
  volatile unsigned long last_micros;
  long debouncing_time = 900; //in micros
  long Pulse_Array[1000];
  long co2_pwm = 0; 
  float lux_vl61=0;
  long range_vl61=0;
  long range_vl53=0;
  float p_temperature_last = 22; 
  float p_humidity_last = 50;
  float p_pressure_last = 1024;
  float p_taupunkt_last = 15;
  float p_Absolute_Feuchte_last = 10;
  float p_lux_last = 100;
  long p_co2_last = 411;
  float p_wind_gust_last = 2;
  float p_wind_avg_avg_last = 2;
  float offset_temp=0;
  float offset_hum=0;
  float power_avg =0.1;
  float power_max =0.1;
  float power_max2 =1;
  float energy_avg =0; //4437000;
  float energy_day =0; //9000;
  float energy_month =0; //95000;
  const char *publishtopic = "1";
  char daytopic[14];
  char monthtopic[14];
  char overalltopic[14];
  bool reset_day = false;
  bool reset_month = false;
  bool energy_received = false;
  bool day_received = false;
  bool month_received = false;
  


//Interrupt for Speed Sensor
void ICACHE_RAM_ATTR Interrupt()
{
  if((long)(micros() - last_micros) >= debouncing_time) {
    if(i_wind==1){
    first_micros = micros();  
      }
    Pulse_Array[i_wind]=micros()-first_micros;
    //last_micros = micros();
   
    i_wind++;
    
    if(i_wind>=999){
       if (Client_Number == "10"){
        stromzaehler();
      }else{
        wind();  
        wind_sum_up();
      }
      
    }
    last_micros = micros();
  }
}



void setup() {
  
  //Start der Seriellen Datenübertragung zum PC  
    Serial.begin(115200);
    Serial.println();
  
 //Auflisten der verfügbaren WLAN Netzwerke
   listNetworks();
 
  //Angabe der Netzwerkdaten
    wifiAutoSelector.add(ssid1, password1);
    wifiAutoSelector.add(ssid2, password2);
    wifiAutoSelector.add(ssid3, password3);

  //Connect to Wifi
    wifiConnect();
    
//Enable "Over the Air" WiFi Update (OTA Updates)

  //SetHostname aktepziert nur dirkte Namen keine Variablen
    if (Client_Number == "0") ArduinoOTA.setHostname("Outdoor");
    if (Client_Number == "1") ArduinoOTA.setHostname("Sleep");
    if (Client_Number == "2") ArduinoOTA.setHostname("Office");
    if (Client_Number == "3") ArduinoOTA.setHostname("Living");
    if (Client_Number == "4") ArduinoOTA.setHostname("Lia");
    if (Client_Number == "5") ArduinoOTA.setHostname("Fabio");
    if (Client_Number == "6") ArduinoOTA.setHostname("Rooflake");
    if (Client_Number == "7") ArduinoOTA.setHostname("Roofhill");
    if (Client_Number == "8") ArduinoOTA.setHostname("Cellar");
    if (Client_Number == "9") ArduinoOTA.setHostname("Garage");
    if (Client_Number == "10")ArduinoOTA.setHostname("Technique");
    if (Client_Number == "11")ArduinoOTA.setHostname("Bathroom");
    if (Client_Number == "12")ArduinoOTA.setHostname("Sensor12");

//Temperature_offset
    if (Client_Number == "0") offset_temp=-1.5;
    if (Client_Number == "1") offset_temp=-2.2;
    if (Client_Number == "2") offset_temp=-1.25;
    if (Client_Number == "3") offset_temp=-1.5;
    if (Client_Number == "4") offset_temp=-1.25;
    if (Client_Number == "5") offset_temp=-1.5;
    if (Client_Number == "6") offset_temp=-1.25;
    if (Client_Number == "7") offset_temp=0;
    if (Client_Number == "8") offset_temp=-0.66;
    if (Client_Number == "9") offset_temp=-0.60;
    if (Client_Number == "10")offset_temp=0;
    if (Client_Number == "11")offset_temp=-1.00;
    if (Client_Number == "12")offset_temp=0;

    //Humidity_offset
    if (Client_Number == "0") offset_hum=4.0;
    if (Client_Number == "1") offset_hum=9.0;
    if (Client_Number == "2") offset_hum=4.0;
    if (Client_Number == "3") offset_hum=4.5;
    if (Client_Number == "4") offset_hum=3.7;
    if (Client_Number == "5") offset_hum=4.5;
    if (Client_Number == "6") offset_hum=5.0;
    if (Client_Number == "7") offset_hum=0;
    if (Client_Number == "8") offset_hum=2.2;
    if (Client_Number == "9") offset_hum=3.8;
    if (Client_Number == "10")offset_hum=0;
    if (Client_Number == "11")offset_hum=0.75;
    if (Client_Number == "12")offset_hum=0;
    
    
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_FS
        type = "filesystem";
      }
     // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      Serial.println("Start updating " + type);
    });
   

    
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
    ArduinoOTA.begin();
    Serial.println("OTA Update Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Hostname: ");
    Serial.println(ArduinoOTA.getHostname());
    


  //Bibliotheken starten
    Wire.begin(SDA_PIN, SCL_PIN);  //I2C Bus
    Wire.setClock(100000); //I2C Bus frequency in Hz
    dht.begin();        //Asong2302 DHT22
    sht31.begin(0x44); //Sensirion SHT31
    bme.begin();  //Bosch BMP388
    DS18B20.begin(); // DS18B20
    DS18B20.setResolution(temperaturePrecision);
    mcp9808.begin();  //MCP9808 0x18
    if(mcp9808.begin()==false) mcp9808_temp = -127; 
    timeClient.begin();
    dac.begin(0x62);
      // For Adafruit MCP4725A0 the address is 0x60 or 0x61
      // For Adafruit MCP4725A1 the address is 0x62 or 0x63 (ADDR pin tied to VCC)
      // For Adafruit MCP4725A2 the address is 0x64 or 0x65
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2); //BH1750 Light Sensor
    lightMeter.setMTreg(254);
    lightMeter2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2); //BH1750 Light Sensor2
    lightMeter2.setMTreg(254);
    pinMode(pwm_pin, INPUT);
    pinMode(input_pin1, INPUT_PULLUP);//
    pinMode(input_pin2, INPUT);//
    if (Client_Number != "10"){
      attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,RISING);
    }
    else attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING);
    
    
    //vl.begin();
    //lox.begin();
    
    //VL53L0.init();
    //VL53L0.setMeasurementTimingBudget(200000);
    VL6180.init();
    VL6180.configureDefault();
    VL6180.setScaling(3);// Valid scaling factors are 1, 2, or 3.


  //Scan i2C Bus for Devices
    scan_i2c();  
  
  //Scan for Maxim 1-Wire Bus Devices
    lookUp_for_DS18B20_Sensors();
  


    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Quality: ");
    Serial.println(WiFi.RSSI()); 
    
  // Sending a notification to your mobile phone; function takes the message as a parameter
    //sendNotification("Restarted");

  //get data from Wunderground
    //Wunderground_read_station();

  // init the MQTT connection
    client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    client.setCallback(callback);
    if (Client_Number == "0") MQTT_CLIENT_ID = MQTT_CLIENT_ID0;
    if (Client_Number == "1") MQTT_CLIENT_ID = MQTT_CLIENT_ID1;
    if (Client_Number == "2") MQTT_CLIENT_ID = MQTT_CLIENT_ID2;
    if (Client_Number == "3") MQTT_CLIENT_ID = MQTT_CLIENT_ID3;
    if (Client_Number == "4") MQTT_CLIENT_ID = MQTT_CLIENT_ID4;
    if (Client_Number == "5") MQTT_CLIENT_ID = MQTT_CLIENT_ID5;
    if (Client_Number == "6") MQTT_CLIENT_ID = MQTT_CLIENT_ID6;
    if (Client_Number == "7") MQTT_CLIENT_ID = MQTT_CLIENT_ID7;
    if (Client_Number == "8") MQTT_CLIENT_ID = MQTT_CLIENT_ID8;
    if (Client_Number == "9") MQTT_CLIENT_ID = MQTT_CLIENT_ID9;
    if (Client_Number == "10")MQTT_CLIENT_ID = MQTT_CLIENT_ID10;
    if (Client_Number == "11")MQTT_CLIENT_ID = MQTT_CLIENT_ID11;
    if (Client_Number == "12")MQTT_CLIENT_ID = MQTT_CLIENT_ID12;

  
   
  //Print Headline of Measurements
    Serial.println();
    Serial.println("MCP9808\tDS18B20\tDHT22\tSHT31\tBMP280\t\tAverage\tDHT22\tSHT31\tAverage\tTaupunkt\tAbs_Feuchte\tBMP280\tBMP280\tBMP280\tCycletime\tBodentemp\tCO2\tWiFi\tMQTT\tTime");


    
}   // End of Void Setup() !!!





void loop() {
  
  
  //Check if WIFI Connection is alive
    if(WiFi.status() != WL_CONNECTED) {
      //Connect to Wifi
        wifiConnect(); //This is a sub routine
        
      //ESP.restart();                //restart ESP8266
    }

  // Check for OTA Updates, first deattach interrupts, to avaoid failurs.
    detachInterrupt(digitalPinToInterrupt(input_pin1));
    //detachInterrupt(digitalPinToInterrupt(input_pin2));

    ArduinoOTA.handle();
    
    attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,RISING);
    //attachInterrupt(digitalPinToInterrupt(input_pin2),Interrupt,FALLING);
    

 
  //Connect to MQTT Service and check in each loop if its connected
    if (!client.connected()) {
      MQTT_connect();
    }
    client.loop();
//   if (digitalRead(input_pin2) != HIGH) i_wind = i_wind + 10;
//   if (digitalRead(input_pin2) != LOW) i_wind = i_wind + 100;

    //declaration of variables and reading out temperatures and humidity from all sensors
    DS18B20.requestTemperatures();
    maximtemp1 = DS18B20.getTempCByIndex(0);
    maximtemp2 = DS18B20.getTempCByIndex(1);
    dht_temp = dht.readTemperature();
    sht31_temp = sht31.readTemperature() + offset_temp;
    if (mcp == true && mcp9808_temp >= -100) mcp9808_temp = mcp9808.readTempC();
    float dht_hum = dht.readHumidity();
    float sht31_humidity = sht31.readHumidity() + offset_hum;
    
    bme.setTemperatureOversampling(BMP3_OVERSAMPLING_32X);
    bme.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bme.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    
    float bmp_pressure = bme.pressure/100.0; //aktueller Luftdruck in mbar
    float bmp_hoehe = bme.readAltitude(1024.00);
    float bmp_temp = bme.temperature;
    lux = lightMeter.readLightLevel();
    lux2 = lightMeter2.readLightLevel();

    if (lux2 <= 0) lux2 = 0;
    lux = lux + lux2;
    
    //lux_vl61 = vl.readLux(VL6180X_ALS_GAIN_5);
    //range_vl61 = vl.readRange();
    //VL53L0X_RangingMeasurementData_t measure;
    //range_vl53= measure.RangeMilliMeter;
    
    //range_vl53 = VL53L0.readRangeSingleMillimeters();
    range_vl61 = VL6180.readRangeSingleMillimeters();





    if (Client_Number == "10"){
         
     //Get current Time from NTP Server
        timeClient.update();
        time_t utcCalc = timeClient.getEpochTime();
        int summer_offset = 0;  //offset in Wintetime
        if (summertime(year(utcCalc), month(utcCalc), day(utcCalc), hour(utcCalc), 1)) summer_offset=3600; //create offset in summertime
        timeClient.setTimeOffset(3600 + summer_offset);
        //timeClient.update();
        utcCalc = timeClient.getEpochTime();
        
       //Reset durchführen einmal pro Tag
       if (hour(utcCalc) == 0){         //Wenn es ein uhr nachts ist
     
          if (reset_day == false){
            energy_day = 0;
            reset_day =true;
          }
            
       }
    
    
       //Reset durchführen einmal pro Monat
       if (day(utcCalc) == 1){ 
        if (hour(utcCalc) == 0){ 
          if (reset_month == false){
            energy_month = 0;
            reset_month == true;
          }
        }
       }
    }


  
    if (millis() >= 10800000){       //Wenn die Laufzeit des Geräts größer ist als 66,66 Minuten (4.000.000 Milliseconds)
      ESP.restart();                //restart ESP8266
      }
  // }

 client.loop();     
      
  // loop through each Maxim DS18B20 device, print out temperature
    /*
    for(int i=0; i<temperatureNumDevices; i++) {
      // print temperature
      float temperatureC = temperatureSensors.getTempC(deviceAddressArray[i]);
      dtostrf(temperatureC, 5, 1, temperatureArrayChar[i]);  // write to char array
    }*/

  //Bestimmung der Druchschnittstemperatur aller Temperatursensoren
     ausfaelle = 0;    
    //if(maximtemp2 <= -100) maximtemp2 = 0;
    
    if(maximtemp1 <= -100) {
      ++ausfaelle;
      maximtemp1 = 0;
    }
   
    if (mcp == false) mcp9808_temp = -127;
    if(mcp9808_temp <= -100){
      ++ausfaelle;
      mcp9808_temp = 0;
      mcp = false;
    }
   
    if (isnan(dht_temp)){
      ++ausfaelle;
      dht_temp = 0;
    }
   
    if (isnan(sht31_temp)) {
      ++ausfaelle;
      sht31_temp = 0;
    }
   
    if (ausfaelle == 4){
      temp_average =0.01;
    }
    else temp_average = (mcp9808_temp + maximtemp1 + dht_temp + sht31_temp )/(4 - ausfaelle);


    //Statusmitteilung nach einem Restart senden
   if (millis() >= 25000){
    if (millis() <= 37000){
      restart_status();
    }
   }
 
   
  //Bestimmung der Druchschnittsfeuchte aller Luftfeuchitgkeitssensoren
    hum_average = (dht_hum + sht31_humidity)/2;
    if (isnan(dht_hum)) {
      hum_average = sht31_humidity;
      } 
    if (isnan(sht31_humidity)) {
      hum_average = dht_hum;
      } 

  //Bestimmung der Min Max werte:
   //morgens
    maxh = _max(maxh, hum_average);
    mint = _min(mint, temp_average);  
    minsoilt = _min(minsoilt, maximtemp2);  

   //abends
    maxt = _max(maxt, temp_average);
    minh = _min(minh, hum_average); //Funktioniert nur mit underscore, aber warum???
        
    //morgens
//    if(maxh == hum_average)maxh_time = timeClient.getFormattedTime();
//    if(mint == temp_average)mint_time = timeClient.getFormattedTime();
//    if(minsoilt == maximtemp2)minsoilt_time = timeClient.getFormattedTime();

    //abends
//    if(maxt == temp_average)maxt_time = timeClient.getFormattedTime();
//    if(minh == hum_average)minh_time = timeClient.getFormattedTime();

  //Jeden morgen die Min Werte übertragen
//     if (hour(utcCalc) == 7){         //Wenn es 7 uhr morgens ist
//      if (Client_Number == "0"){ 
//        if (morgenwerte_uebermittelt == false){
//          //AIO_Min_Max_Temp_outdoor_1.publish(mint);
//          delay(2000);
//          //AIO_Min_Max_hum_outdoor_1.publish(maxh);
//          delay(2000);
//          //AIO_Min_Soil_Temp_outdoor_1.publish(minsoilt);
//          delay(2000);
////            String Morgentemp = String(mint);
////            String Morgenfeuchte = String(maxh);
////            String morgentemp_time = String(mint_time);
////            String morgenfeuchte_time = String(maxh_time);
//          String Morgenwerte = "Die Mintemp betrug " +String(mint) +"°C, um " +String(mint_time) +"Uhr; Die Mintemp am Boden betrug " +String(minsoilt) +"°C, um " +String(minsoilt_time) +"Uhr. Die Maxfeuchte betrug " +String(maxh) +"% um " +String(maxh_time) +"Uhr";
//          //sendNotification(Morgenwerte);
//          morgenwerte_uebermittelt = true;
//        }
//      }
//    }
//
//  //jeden Abend die max Werte übertragen
//     if (hour(utcCalc) == 18){         //Wenn es 18 uhr abends ist
//      if (Client_Number == "0"){ 
//        if (abendwerte_uebermittelt == false){
//          //AIO_Min_Max_Temp_outdoor_1.publish(maxt);
//          delay(2000);
//          //AIO_Min_Max_hum_outdoor_1.publish(minh);
//          delay(2000);
//          String Abendtemp = String(maxt);
//          String Abendfeuchte = String(minh);
//          String Abendtemp_time = String(maxt_time);
//          String Abendfeuchte_time = String(minh_time);
//          String Abendwerte = "Maxtemp: " +Abendtemp +"°C, um " +Abendtemp_time +"Uhr; Minfeuchte: " +Abendfeuchte +"% um " +Abendfeuchte_time +"Uhr";
//          //sendNotification(Abendwerte);
//          abendwerte_uebermittelt = true;
//          
//        }
//      }
//    }
  

  //Bestimmung des Taupunkts
    Taupunkt_mittel = Taupunkt_berechnen(temp_average, hum_average);

  //Bestimmung der absoluten Luftfeuchigkeit
  Absolute_Feuchte_mittel = Absolute_Feuchte_berechnen(temp_average, hum_average);
client.loop();
// Bestimmung des CO2 Werts
    if (Client_Number != "0"){ 
      if (millis() >= 25000){       //Wenn die Laufzeit des Geräts größer ist als 22sec MH Z19b CO2 Sensor needs 20sec boot time
        read_CO2_PWM();
      }
      else {
        co2_pwm = 410;
      }
      
    }
  
  //Ermittlung des aktuellen Internet Verbindungsstatus
    //CurrentWifiStatus();

  //Bestimmung des Atmosphärischen Drucks bezogen auf Meereshöhe.
    
client.loop();    

    //Bodenwetterkarte > QFF: Der Ortsluftdruck wird mit Hilfe der vor Ort herrschenden Temperatur auf Meereshöhe runter gerechnet.
    //Fliegerei > QNH: Der Ortsluftdruck wird mit Hilfe der ICAO-Standardatmosphäre auf Meerehöhe runter gerechnet.
    //
    //QFF:
    //- Vorteil; Näher an der Realität, da es ja nicht immer ICAO-Standartatmösphären Wetter (u. a. 15°C auf NN) hat.
    //- Nachteil; Man benötigt eine Temperaturmessung vor Ort.
    //=> Für Bodenwetterkarte geeignet.
    //
    //QNH:
    //- Vorteil; Man braucht nur den Luftdruck zu messen und rechnet mit Standardwerten auf Meereshöhe runter. Jeder (z. B. Flugzeug) erhält bei der Reduktion den gleichen Fehler, so ist es möglich die Flugzeuge in verschiedenen Höhen zu ordnen.
    //- Nachteil; Die runter gerechneten Luftdruckwerte entsprechen nicht der Wahrheit, da das "echte" Wetter meist von der ICAO-Standardatmosphäre abweicht.
    //=> Für georndete Flughöhen geeignet.
   
    float Sea_level = 510.1;  //Meereshöhe + Plattformhöhe des Spielturms: 508,6m + 1,5m = 510.1m
    float T = temp_average ;  //Außentemperatur nötig zur Berechnung des korrekten Drucks bezogen auf Meereshöhe
    float Th = T + 273.15;
    
    float Ch_E = (((Absolute_Feuchte_mittel/1000)* 461.5 * Th)/100)*0.12;
    float a_h_half = 0.0065 * Sea_level * 0.5;
    float nenner = (Th + Ch_E + a_h_half)*287.05;
    float x_pressure = (Sea_level * 9.80665)/nenner;
    
    atmospheric_pressure = bmp_pressure * exp (x_pressure); //QFF detailed (for weatherforecast)
    //atmospheric_pressure = (bmp_pressure * exp (Sea_level / (29.3 * (T + 273.15)))) ; //QFF old
    float Sea_level_pressure = (bmp_pressure / pow((1 - (Sea_level / 44330)),5.255)); //QNH (for heith measurement)
    float bmp_hoehe2 = 44330.0 * (1.0 - pow(bmp_pressure / Sea_level_pressure, 0.1903));
   
  //Bestimmen der Programmlaufzeit
    time_new = millis();
    float cycle_time = time_new - time_old;
    time_old = millis();

  //Messen der Windgeschwindigkeit
  if (Client_Number == "0") {
    wind();
    wind_sum_up();
  }
    
 client.loop();
 
  //Print all Data to Serial Monitor
    Serial.print(mcp9808_temp); Serial.print("°C"); Serial.print("\t");
    Serial.print(maximtemp1); Serial.print("°C"); Serial.print("\t");
    Serial.print(dht_temp); Serial.print("°C"); Serial.print("\t");
    Serial.print(sht31_temp); Serial.print("°C"); Serial.print("\t");
    Serial.print(bmp_temp); Serial.print("°C"); Serial.print("\t");
    Serial.print(temp_average); Serial.print("°C"); Serial.print("\t");
    
    Serial.print(dht_hum); Serial.print("%"); Serial.print("\t");
    Serial.print(sht31_humidity); Serial.print("%"); Serial.print("\t");// "\n\r" is NewLine; \t prints a tab
    Serial.print(hum_average); Serial.print("%"); Serial.print("\t");
    Serial.print(Taupunkt_mittel); Serial.print("°C"); Serial.print("\t");
    Serial.print(Absolute_Feuchte_mittel); Serial.print("g/m³"); Serial.print("\t");
     
    Serial.print(bmp_pressure); Serial.print("hpa"); Serial.print("\t");
    Serial.print(atmospheric_pressure);Serial.print("hpa"); Serial.print("\t");
    Serial.print(Sea_level_pressure);Serial.print("hpa"); Serial.print("\t");
    //Serial.print(p0);Serial.print("hpa"); Serial.print("\t");
    Serial.print(bmp_hoehe); Serial.print("m"); Serial.print("\t");
    Serial.print(bmp_hoehe2); Serial.print("m"); Serial.print("\t");
    Serial.print(cycle_time); Serial.print("ms"); Serial.print("\t");
    
    Serial.print(maximtemp2); Serial.print("°C"); Serial.print("\t");
    Serial.print(co2_pwm); Serial.print("ppm"); Serial.print("\t");
    Serial.print(WiFi.status()); Serial.print(" "); Serial.print("\t");
    Serial.print(ausfaelle); Serial.print(""); Serial.print("\t");
    Serial.print(lux); Serial.print("lx"); Serial.print("\t");
    Serial.print(lux2); Serial.print("lx2"); Serial.print("\t");
    Serial.print(wind_gust); Serial.print(" gust km/h"); Serial.print("\t");
    Serial.print(wind_avg); Serial.print(" avg km/h"); Serial.print("\t");
    Serial.print(i_wind2); Serial.print("Pulse"); Serial.print("\t");
    //Serial.println();
//    Serial.print(lux_vl61); Serial.print("lx_ST"); Serial.print("\t");
//    Serial.print(range_vl61); Serial.print("mm"); Serial.print("\t");
//    Serial.print(range_vl53); Serial.print("mm"); Serial.print("\t");
//    Serial.print(MQTT_CLIENT_ID); Serial.print(""); Serial.print("\t");
   
    
  //long too_long = ((REPORT_INTERVAL * 1000) + 10000);
  //if (cycle_time >= too_long){
  //  ESP.restart();
  //}
  
  
  //Analogwerte Ausgeben
    //write_analog_output(100,0);
    //write_analog_output(200,1);
    //write_analog_output(300,2);
    //write_analog_output(400,3);
    

    
 
      time_new5 = millis();
      float cycle_time4 = time_new5 - time_old5;

      
      if (cycle_time4 >= 1000 * REPORT_INTERVAL){     //Diese Befehle alle x sekunden durchführen
        
        if (Client_Number != "10"){
          if (millis() >= 40000){ 
            publishData(temp_average, hum_average, atmospheric_pressure, Taupunkt_mittel, Absolute_Feuchte_mittel, lux, co2_pwm, wind_gust_max, wind_avg_avg);    // Publish Data to MQTT Broker for all sensors expect technique
          }
        }
        else {
          stromzaehler();
          publishData(temp_average, hum_average, atmospheric_pressure, Taupunkt_mittel, Absolute_Feuchte_mittel, lux, co2_pwm, power_avg, energy_avg);         // Publish Data to MQTT Broker for technique Sensor
        }
        
      data_published();    
        
      time_old5 = time_new5;
      }
      else{
          Serial.print("No_MQTT_Transfer"); Serial.print("\t");
      }
   
 
 Serial.println(); 
 
// Post measurement Data to Wunderground Station
    if (Client_Number == "0"){ 

      //Diese Befehle alle 3 minuten durchführen
      time_new4 = millis();
      float cycle_time3 = time_new4 - time_old4;
      if (cycle_time3 >= 3 * 60000){ 
          
          Wunderground_post_data();
        //Wunderground Stationsvergleich
        //Wunderground_read_station();
        Serial.print("Wunderground"); Serial.print("\t");
        time_old4 = time_new4;
      }
    }

  
  
//Diese Befehle alle 6 Stunden durchführen
    time_new3 = millis();
    float cycle_time2 = time_new3 - time_old3;
    if (cycle_time2 >= 360 * 60000){ 
     
      //Pushingbox soll den aktuellen Status erhalten
      //current_status();
      Serial.print("current_status"); Serial.print("\t");
      time_old3 = time_new3;
    }
  
    
    //Serial.print(cycle_time); Serial.print("ms"); Serial.print("\t");
//    Serial.println(timeClient.getFormattedTime());


  //abwarten des angegebenen Updateintervalls
   if (cycle_time < 6000) delay(4100);
    //int cnt = REPORT_INTERVAL; //- (cycle_time - REPORT_INTERVAL);
    //while(cnt--) delay(1000);
  
   
}     // End of void loop() !!!




//Check the current WiFi Status
void CurrentWifiStatus()  
{
  
//    WL_NO_SHIELD        = 255   // for compatibility with WiFi Shield library
//    WL_IDLE_STATUS      = 0
//    WL_NO_SSID_AVAIL    = 1
//    WL_SCAN_COMPLETED   = 2
//    WL_CONNECTED        = 3
//    WL_CONNECT_FAILED   = 4
//    WL_CONNECTION_LOST  = 5
//    WL_DISCONNECTED     = 6

 // WL_CONNECTED: assigned when connected to a WiFi network; :
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected to a WiFi network");
    }
  
   // WL_NO_SHIELD: check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.print("WiFi shield not present");
    }
 
  // WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED); 
  if (WiFi.status() == WL_IDLE_STATUS) {
    Serial.print("WL_IDLE_STATUS");
    } 

  // WL_NO_SSID_AVAIL: assigned when no SSID are available; 
  if (WiFi.status() == WL_NO_SSID_AVAIL) {
    Serial.print("No SSID are available");
    } 
    
  // WL_SCAN_COMPLETED: assigned when the scan networks is completed;  
  if (WiFi.status() == WL_SCAN_COMPLETED) {
    Serial.print("scan of networks is completed");
    } 

   // WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;   
  if (WiFi.status() == WL_CONNECT_FAILED) {
    Serial.print("the connection fails for all the attempts");
    } 

   // WL_CONNECTION_LOST: assigned when the connection is lost;   
  if (WiFi.status() == WL_CONNECTION_LOST) {
    Serial.print("the connection is lost");
    } 

   // WL_DISCONNECTED: assigned when disconnected from a network;   
  if (WiFi.status() == WL_DISCONNECTED) {
    Serial.print("disconnected from a network");
    } 
}


//erreichbare Netzwerke Auflisten
void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    while (true);
  }

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    //known_SSIDs[sizeof(known_SSIDs)] = WiFi.SSID(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));
  }
}


//Anzeige der Verschlüsselung
void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.println("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.println("Auto");
      break;
  }
}


void scan_i2c()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  //delay(5000);           // wait 5 seconds for next scan
}

//making wifi connection
void wifiConnect()
{
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.print("Connecting to wifi: ");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid1, password1);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Connection Failed! Rebooting...");
      delay(5000);
      ESP.restart();
    }
 }   
 
//    if(-1 < wifiAutoSelector.scanAndConnect()) {
//      int connectedIndex = wifiAutoSelector.getConnectedIndex();
//      Serial.print(" '");
//      Serial.print(wifiAutoSelector.getSSID(connectedIndex));
//      Serial.println("'. Done.");
//    }else{
//      Serial.println("failed.");
//    }
  
 
if(WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println();
    Serial.print("Connecting to wifi: ");
    if(-1 < wifiAutoSelector.scanAndConnect()) {
      int connectedIndex = wifiAutoSelector.getConnectedIndex();
      Serial.print(" '");
      Serial.print(wifiAutoSelector.getSSID(connectedIndex));
      Serial.println("'. Done.");
    }else{
      Serial.println("failed.");
    }
  }
  delay(500);
 }

  //taupunkt berechnen
  // See: http://www.wetterochs.de/wetter/feuchte.html
  
float Taupunkt_berechnen(float Temp2, float feuchte)
{
float a;
float b;
float Saettigungsdampfdruck;
float Dampfdruck;
float v;
float Taupunkt;
float absolute_Feuchte;

float molekulargewicht_wasserdampf;
float universelle_Gaskonstante;
float aTp;
float bTp;

  
  
universelle_Gaskonstante = 8314.3; //J/(kmol*K) (universelle Gaskonstante)
molekulargewicht_wasserdampf = 18.016; //kg/kmol (Molekulargewicht des Wasserdampfes)
 
//Temp = -20.0;
//feuchte= 20.0;

if(Temp2 >= 0) {
    a = 7.5;
    b = 237.3;
}else{
    a = 9.5; 
    b = 265.5;
    }
  
      
    Saettigungsdampfdruck = 6.1078 * pow(10,((a*Temp2)/(b+Temp2))); //(T)
    Dampfdruck = feuchte/100 * Saettigungsdampfdruck;        //(r,T)
    v = log10(Dampfdruck/6.1078);                            //(r,T)
    Taupunkt = (b*v)/(a-v);                                    //(r,T)
    absolute_Feuchte = pow(10,5) * molekulargewicht_wasserdampf/universelle_Gaskonstante * Dampfdruck/(Temp2+273.15);           //(r,TK)
    //absolute_Feuchte = 10^5 * mw/R* * SDD(TD)/TK            //(TD,TK)

//vergleich
 aTp = 17.271;         // fuer Formel Taupunkt
bTp = 237.7;          // fuer Formel Taupunkt
taupunktTmp = ((aTp * Temp2) / (bTp + Temp2)) + log(feuchte / 100);
Taupunkt_alt = (bTp * taupunktTmp) / (aTp - taupunktTmp);
return Taupunkt ; 
  }


float Absolute_Feuchte_berechnen(float Temp2, float feuchte)
{
float a;
float b;
float Saettigungsdampfdruck;
float Dampfdruck;
float v;
float Taupunkt;
float absolute_Feuchte;

float molekulargewicht_wasserdampf;
float universelle_Gaskonstante;
float aTp;
float bTp;

  
  
universelle_Gaskonstante = 8314.3; //J/(kmol*K) (universelle Gaskonstante)
molekulargewicht_wasserdampf = 18.016; //kg/kmol (Molekulargewicht des Wasserdampfes)
 
//Temp = -20.0;
//feuchte= 20.0;

if(Temp2 >= 0) {
    a = 7.5;
    b = 237.3;
}else{
    a = 9.5; 
    b = 265.5;
    }
  
      
    Saettigungsdampfdruck = 6.1078 * pow(10,((a*Temp2)/(b+Temp2))); //(T)
    Dampfdruck = feuchte/100 * Saettigungsdampfdruck;        //(r,T)
    v = log10(Dampfdruck/6.1078);                            //(r,T)
    Taupunkt = (b*v)/(a-v);                                    //(r,T)
    absolute_Feuchte = pow(10,5) * molekulargewicht_wasserdampf / universelle_Gaskonstante * Dampfdruck / (Temp2+273.15);           //(r,TK)
    //absolute_Feuchte = 10^5 * mw/R* * SDD(TD)/TK            //(TD,TK)

//vergleich
 aTp = 17.271;         // fuer Formel Taupunkt
bTp = 237.7;          // fuer Formel Taupunkt
taupunktTmp = ((aTp * Temp2) / (bTp + Temp2)) + log(feuchte / 100);
Taupunkt_alt = (bTp * taupunktTmp) / (aTp - taupunktTmp);

float neue_abs_feuchte =(13.2471488 * feuchte * exp((17.67 * Temp2)/(Temp2 + 243.5))) / (273.15+Temp2);
//absolute_Feuchte = absolute_Feuchte - neue_abs_feuchte;

return absolute_Feuchte ; 




  }


void lookUp_for_DS18B20_Sensors()
{
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  
  Serial.print("Looking for 1-Wire devices...\n\r");    // "\n\r" is NewLine; \t prints a tab
  Serial.print(oneWire.search(addr));
  while(oneWire.search(addr)) {
    Serial.print("\n\rFound \'1-Wire\' device with address:\n\r");
    for( i = 0; i < 8; i++) {
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');
      }
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.print("CRC is not valid!\n");
        return;
    }
  }
  Serial.print("\n\r\n\rThat's it.\r\n");
  oneWire.reset_search();
  return;
}


void sendNotification(String message){

  //Serial.println("- connecting to pushingbox.com server: " + String(logServer));
  if (client2.connect(logServer, 80)) {
    //Serial.println("- succesfully connected");
    
    String postStr = "devid=";
    postStr += String(deviceId);
    postStr += "&parameter=";   //Wert bei "Pushingbox -> Scenario -> Action" zwischen 2 Dollar Zeichen steht, im Feld "Message" oder "titel", oder in beiden; Anleitung: http://www.geekstips.com/android-push-notifications-esp8266-arduino-tutorial/
    postStr += String(message);
    postStr += "\r\n\r\n";
    
    //Serial.println("- sending data...");
    
    client2.print("POST /pushingbox HTTP/1.1\n");
    client2.print("Host: api.pushingbox.com\n");
    client2.print("Connection: close\n");
    client2.print("Content-Type: application/x-www-form-urlencoded\n");
    client2.print("Content-Length: ");
    client2.print(postStr.length());
    client2.print("\n\n");
    client2.print(postStr);
  }
client2.stop();
 }

 
boolean summertime(int year, byte month, byte day, byte hour, byte tzHours)
// European Daylight Savings Time calculation by "jurs" for German Arduino Forum
// input parameters: "normal time" for year, month, day, hour and tzHours (0=UTC, 1=MEZ)
// return value: returns true during Daylight Saving Time, false otherwise
{ 
 if (month<3 || month>10) return false; // keine Sommerzeit in Jan, Feb, Nov, Dez
 if (month>3 && month<10) return true; // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
 if (month==3 && (hour + 24 * day)>=(1 + tzHours + 24*(31 - (5 * year /4 + 4) % 7)) || month==10 && (hour + 24 * day)<(1 + tzHours + 24*(31 - (5 * year /4 + 1) % 7))) 
   return true; //Summertime
 else 
   return false; //Wintertime
}



void Wunderground_read_station() {

  // Use WiFiClient class to create TCP connections
  //const int httpPort = 80;
  if (!client2.connect(WUNDERGROUND, 80)) {
    Serial.println(F("connection failed"));
    return;
  }

  Serial.println();
  Serial.print(WUNDERGROUND_REQ);
  client2.print(WUNDERGROUND_REQ);
  client2.flush();

 // Skip HTTP headers; They end with an empty line
  char endOfHeaders[] = "\r\n\r\n";
  client2.setTimeout(10000); //http Timeout
  client2.find(endOfHeaders);

  
  UserData userData;
  if (readReponseContent(&userData)) {
    printUserData(&userData);
  }


client2.stop();
}


// extract Data from the JSON File
bool readReponseContent(struct UserData* userData) {
  // Compute optimal size of the JSON buffer according to what we need to parse.
  // See https://bblanchon.github.io/ArduinoJson/assistant/
  const size_t BUFFER_SIZE = 
      JSON_OBJECT_SIZE(8)    // the root object has 8 elements
      + JSON_OBJECT_SIZE(5)  // the "address" object has 5 elements
      + JSON_OBJECT_SIZE(2)  // the "geo" object has 2 elements
      + JSON_OBJECT_SIZE(3)  // the "company" object has 3 elements
      + 512;                 // additional space for strings

  // Allocate a temporary memory pool
  DynamicJsonBuffer jsonBuffer(BUFFER_SIZE);

  JsonObject& root = jsonBuffer.parseObject(client2);

  if (!root.success()) {
    Serial.println("JSON parsing failed!");
    return false;
  }

  // Here were copy the strings we're interested in
  strcpy(userData->temp_c, root["current_observation"]["temp_c"]);
  strcpy(userData->relative_humidity, root["current_observation"]["relative_humidity"]);
  strcpy(userData->dewpoint_c, root["current_observation"]["dewpoint_c"]);
  strcpy(userData->observation_time_rfc822, root["current_observation"]["observation_time_rfc822"]);
  // It's not mandatory to make a copy, you could just use the pointers
  // Since, they are pointing inside the "content" buffer, so you need to make
  // sure it's still in memory when you read the string

  return true;
}

 //Print the data extracted from the JSON
void printUserData(const struct UserData* userData) {
  Serial.print("temp_c = ");
  Serial.println(userData->temp_c);
  Serial.print("relative_humidity = ");
  Serial.println(userData->relative_humidity);
  Serial.print("dewpoint_c = ");
  Serial.println(userData->dewpoint_c);
  Serial.print("observation_time_rfc822 = ");
  Serial.println(userData->observation_time_rfc822);

         
  
  String Sting_Userdata_temp_c = (char *)userData->temp_c;
  float float_Userdata_temp_c =  Sting_Userdata_temp_c.toFloat();
  compare_temp = float_Userdata_temp_c - temp_average;
//  AIO_Berger_Halde.publish(compare_temp);
  delay(2000);
          
        
  
}

void Wunderground_post_data(){
  
String dew ="&dewptf="; 
String tem="&tempf="; 
String tem2="&temp2f="; 
String tem3="&temp3f="; 
String tem4="&temp4f="; 
String tem5="&temp5f="; 
String soiltem="&soiltempf="; 
String bar ="&baromin=";

String taupunkt = String(Taupunkt_mittel * 1.8 + 32);
String temp = String(temp_average * 1.8 + 32);
String temp2 = String(mcp9808_temp * 1.8 + 32);
String temp3 = String(maximtemp1 * 1.8 + 32);
String temp4 = String(dht_temp * 1.8 + 32);
String temp5 = String(sht31_temp * 1.8 + 32);
String soiltemp = String(maximtemp2 * 1.8 + 32);
String pressure = String(atmospheric_pressure *0.02954);
String humidity = String(hum_average);

String WUNDERGROUND_end =
      "&realtime=1&rtfreq=2.5&action=updateraw HTTP/1.1\r\n"
      "User-Agent: ESP8266/0.1\r\n"
      "Accept: */*\r\n"
      "Host: " WUNDERGROUND2 "\r\n"
      "Connection: close\r\n"
      "\r\n";
      
String WUNDERGROUND_all =         // Create Link: /weatherstation/updateweatherstation.php?ID="STATION_ID"&PASSWORD="PASSWORD"&dateutc=now&humidity=84.4&dewptf=60.1&tempf=64.4&baromin=29.5001&realtime=1&rtfreq=2.5&action=updateraw
      "GET /weatherstation/updateweatherstation.php?ID=" WUNDERGROUND_STATION_ID 
      "&PASSWORD=" WUNDERGROUND_STATION_PASSWORD 
      "&dateutc=now&humidity=" +humidity 
      +dew +taupunkt 
      +tem +temp 
      +tem2 +temp2 
      +tem3 +temp3 
      +tem4 +temp4 
      +tem5 +temp5 
      +soiltem +soiltemp 
      +bar +pressure 
     +String("&realtime=1&rtfreq=2.5&action=updateraw HTTP/1.1\r\n"
      "User-Agent: ESP8266/0.1\r\n"
      "Accept: */*\r\n"
      "Host: " WUNDERGROUND2 "\r\n"
      "Connection: close\r\n"
      "\r\n"); 


//Serial.println();
//Serial.println(WUNDERGROUND_all);
 
client2.connect(WUNDERGROUND2, 80);
client2.print(WUNDERGROUND_all);
client2.stop();
// http://rtupdate.wunderground.com/weatherstation/updateweatherstation.php?ID=IMECKENB22&PASSWORD=suggypas&dateutc=now&humidity=84.4&dewptf=60.1&tempf=64.4&baromin=29.5001&realtime=1&rtfreq=2.5&action=updateraw
 
}

void restart_status(){
  long rssi = WiFi.RSSI();
  String current_SSID = WiFi.SSID();
 // String current_time = timeClient.getFormattedTime();
  
//  
//  String Statuswerte = "Es erfolgte ein Restart um " +current_time 
//  +"Uhr mit dem Netzwerk " +current_SSID 
//  + ".  \r\nEmpfangsqualität: " +rssi 
//  +"db.  \r\nAktuell " +ausfaelle 
//  + " Sensorausfälle.  \r\nAvg. Temp: " +temp_average 
//  +"°C.  \r\nMCP9808: " +mcp9808_temp 
//  +"°C.  \r\nDS18b20: " +maximtemp1  
//  +"°C.  \r\nDHT21: "+dht_temp  
//  + "°C.  \r\nSHT31: " +sht31_temp 
//  +"°C.  \r\nBodentemp: " +maximtemp2  
//  +"°C.  \r\nAvg. Feuchte: " +hum_average 
//  +"%.  \r\nLuftdruck: "+atmospheric_pressure  
//  +"mbar.  \r\nTaupunkt: "   +Taupunkt_mittel  +"°C." ; 
//  
  //sendNotification(Statuswerte);
  
}

void current_status(){
  long rssi = WiFi.RSSI();
  String current_SSID = WiFi.SSID();
//  String current_time = timeClient.getFormattedTime();
  
  
//  String Statuswerte = "Statusreport um " +current_time 
//  +"Uhr mit dem Netzwerk " +current_SSID 
//  + ".  \r\nEmpfangsqualität: " +rssi 
//  +"db.  \r\nAktuell " +ausfaelle 
//  + " Sensorausfälle.  \r\nAvg. Temp: " +temp_average 
//  +"°C.  \r\nMCP9808: " +mcp9808_temp 
//  +"°C.  \r\nDS18b20: " +maximtemp1  
//  +"°C.  \r\nDHT21: "+dht_temp  
//  + "°C.  \r\nSHT31: " +sht31_temp 
//  +"°C.  \r\nBodentemp: " +maximtemp2  
//  +"°C.  \r\nAvg. Feuchte: " +hum_average 
//  +"%.  \r\nLuftdruck: "+atmospheric_pressure  
//  +"mbar.  \r\nTaupunkt: "   +Taupunkt_mittel  +"°C." ; 
  
  //sendNotification(Statuswerte);
  
}

void MQTT_connect() {
  // Loop until we're connected
  Serial.print("Attempting MQTT connection...");
  if (!client.connected()) {
     // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("MQTT connected");
       MQTT_subscribe();
       
    } else {
      Serial.print("ERROR: Connection failed, rc=");
      Serial.print(client.state());
        //int state ()
        //
        //Returns the current state of the client. If a connection attempt fails, this can be used to get more information about the failure.
        //Returns
        //
        //    int - the client state, which can take the following values (constants defined in PubSubClient.h):
        //        -4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
        //        -3 : MQTT_CONNECTION_LOST - the network connection was broken
        //        -2 : MQTT_CONNECT_FAILED - the network connection failed
        //        -1 : MQTT_DISCONNECTED - the client is disconnected cleanly
        //        0 : MQTT_CONNECTED - the client is connected
        //        1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
        //        2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
        //        3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
        //        4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
        //        5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect


      Serial.println("try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      //sendNotification("MQTT Connection Failed");
      if (client.state()==-2){
        //ESP.restart();                //restart ESP8266
        
      }
    }
  }
}


// function called to publish the temperature and the humidity
void publishData(float p_temperature, float p_humidity, float p_pressure, float p_taupunkt, float p_Absolute_Feuchte, float p_lux, long p_co2,float p_wind_gust ,float p_wind_avg_avg) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  
// /* 
//  if (p_temperature >= 1.1*p_temperature_last) p_temperature = 1.1*p_temperature_last;
//  if (p_temperature <= 0.9*p_temperature_last) p_temperature = 0.9*p_temperature_last;
//
//  if (p_pressure >= 1.1*p_pressure_last) p_pressure = 1.1*p_pressure_last;
//  if (p_pressure <= 0.9*p_pressure_last) p_pressure = 0.9*p_pressure_last;
//
//  if (p_taupunkt >= 1.1*p_taupunkt_last) p_taupunkt = 1.1*p_taupunkt_last;
//  if (p_taupunkt <= 0.9*p_taupunkt_last) p_taupunkt = 0.9*p_taupunkt_last;
//
//  if (p_Absolute_Feuchte >= 1.1*p_Absolute_Feuchte_last) p_Absolute_Feuchte = 1.1*Absolute_Feuchte_last;
//  if (p_Absolute_Feuchte <= 0.9*p_Absolute_Feuchte_last) p_Absolute_Feuchte = 0.9*Absolute_Feuchte_last;
//
//  if (p_lux >= 1.5*p_lux_last) p_lux = 1.5*p_lux_last;
//  if (p_lux <= 0.5*p_lux_last) p_lux = 0.5*p_lux_last;
//
//  if (p_co2 >= 1.1*p_co2_last) p_co2 = 1.1*p_co2_last;
//  if (p_co2 <= 0.9*p_co2_last) p_co2 = 0.9*p_co2_last;
//
//  if (p_wind_gust >= 1.1*p_wind_gust_last) p_wind_gust = 1.1*p_wind_gust_last;
//  if (p_wind_gust <= 0.9*p_wind_gust_last) p_wind_gust = 0.9*p_wind_gust_last;
//
//  if (p_wind_avg_avg >= 1.1*p_wind_avg_avg_last) p_wind_avg_avg = 1.1*p_wind_avg_avg_last;
//  if (p_wind_avg_avg <= 0.9*p_wind_avg_avg_last) p_wind_avg_avg = 0.9*p_wind_avg_avg_last;
//
//  p_humidity_last = p_humidity;
//  p_pressure_last = p_pressure;
//  p_taupunkt_last = p_taupunkt;
//  p_Absolute_Feuchte_last = p_Absolute_Feuchte;
//  p_lux_last = p_lux;
//  p_co2_last = p_co2;
//  p_wind_gust_last = p_wind_gust;
//  p_wind_avg_avg_last = p_wind_avg_avg;
//  
//*/ 
 
  if (p_co2 <= 350) p_co2 = 350;
  
  uint16_t p_lux2;
  uint16_t p_wind_gust2;
  uint16_t p_wind_avg_avg2;

//  p_temperature =-12.23;
//  p_lux = 9999.91;
//  p_wind_gust = 23.91;
//  p_wind_avg_avg = 34.91;
//  p_co2 = 1485;
//  p_pressure = 1004.65;
  
 if (p_lux >= 100) p_lux2 = (int)(p_lux);
 if (p_wind_gust >= 100) p_wind_gust2 = (int)(p_wind_gust);
 if (p_wind_avg_avg >= 100) p_wind_avg_avg2 = (int)(p_wind_avg_avg);
    Serial.println();
    
    Serial.print(p_lux); Serial.print("lx"); Serial.print("\t");
    Serial.print(p_lux2); Serial.print("lx"); Serial.print("\t");
    
  StaticJsonBuffer<270> jsonBuffer;
  
  // Get the root object in the document
  JsonObject& root = jsonBuffer.createObject();
  
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["T"] = (String)p_temperature;
  root["H"] = (String)p_humidity;
  if (Client_Number == "0")root["P"] = (String)p_pressure;
  root["D"] = (String)p_taupunkt;
  root["A"] = (String)p_Absolute_Feuchte;
  if (p_lux < 100) root["L"] = (String)p_lux;
  if (p_lux >= 100) root["L"] = (String)p_lux2;
  if (Client_Number != "0") root["C"] = (String)p_co2;
 // if (Client_Number != "0") root["G"] = (String)p_wind_gust;
 // if (Client_Number != "0") root["W"] = (String)p_wind_avg_avg;
  if (p_wind_gust < 100) root["G"] = (String)p_wind_gust;
  if (p_wind_avg_avg < 100) root["W"] = (String)p_wind_avg_avg;
  if (p_wind_gust2 >= 9999) p_wind_gust2=9999;
  if (p_wind_gust >= 100) root["G"] = (String)p_wind_gust2;
  if (p_wind_avg_avg >= 999) p_wind_avg_avg2=999;
  if (p_wind_avg_avg >= 100) root["W"] = (String)p_wind_avg_avg2;
  
  //root.prettyPrintTo(Serial);
  //Serial.println("");


  dtostrf(energy_day, 10, 2, daytopic);//convert float to char
  dtostrf(energy_month, 10, 2, monthtopic);//convert float to char
  dtostrf(energy_avg, 10, 2, overalltopic);//convert float to char
  
  char data[270];
  root.printTo(data, root.measureLength() + 1);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPICS0, data, true);
    if (Client_Number == "1") client.publish(MQTT_SENSOR_TOPICS1, data, true);    
    if (Client_Number == "2") client.publish(MQTT_SENSOR_TOPICS2, data, true);    
    if (Client_Number == "3") client.publish(MQTT_SENSOR_TOPICS3, data, true);   
    if (Client_Number == "4") client.publish(MQTT_SENSOR_TOPICS4, data, true);   
    if (Client_Number == "5") client.publish(MQTT_SENSOR_TOPICS5, data, true);   
    if (Client_Number == "6") client.publish(MQTT_SENSOR_TOPICS6, data, true);   
    if (Client_Number == "7") client.publish(MQTT_SENSOR_TOPICS7, data, true);    
    if (Client_Number == "8") client.publish(MQTT_SENSOR_TOPICS8, data, true);    
    if (Client_Number == "9") client.publish(MQTT_SENSOR_TOPICS9, data, true);   
    if (Client_Number == "10") client.publish(MQTT_SENSOR_TOPICS10, data, true);
    if (Client_Number == "10") client.publish("Ventilation_Office/switch/status",publishtopic);
    if (Client_Number == "10") client.publish("Ventilation_Sleep/switch/status",publishtopic);
    if (Client_Number == "10") client.publish("Ventilation_Kids/switch/status",publishtopic);   
    if (Client_Number == "10") client.publish("homeassistant/sensor/energy/day",daytopic);
    if (Client_Number == "10") client.publish("homeassistant/sensor/energy/month",monthtopic);
    if (Client_Number == "10") client.publish("homeassistant/sensor/energy/all",overalltopic);
    if (Client_Number == "11") client.publish(MQTT_SENSOR_TOPICS11, data, true);   
    if (Client_Number == "12") client.publish(MQTT_SENSOR_TOPICS12, data, true);

  yield();
  Serial.print("MQTT-Published"); Serial.print("\t");
  //MQTT_subscribe();
  

 
}


void MQTT_subscribe(){
  client.subscribe("LED");
  
  if (Client_Number == "0"){
    client.subscribe("Ventilation/Power");
  }
   if (Client_Number == "10"){
     client.subscribe("Ventilation_Office/Power");
     client.publish("Ventilation_Office/switch/status",publishtopic);
     client.subscribe("Ventilation_Sleep/Power");
     client.publish("Ventilation_Sleep/switch/status",publishtopic);
     client.subscribe("Ventilation_Kids/Power");
     client.publish("Ventilation_Kids/switch/status",publishtopic);
     client.subscribe("homeassistant/sensor/energy/state");
     client.subscribe("homeassistant/sensor/energy_day/state");
     client.subscribe("homeassistant/sensor/energy_month/state");
   }
}

// function called when a MQTT message arrived
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived; TOPIC: [");
  Serial.print(topic);
  
  Serial.print("] ");
  Serial.print(topic[4]);
  Serial.print("Payload [");
  
  
 if (topic[0] =='L' ) ESP.restart();

 
  
  payload[length] = '\0'; // Add a NULL to the end of the char* to make it useable as string https://www.arduino.cc/reference/de/language/variables/data-types/string/
 
  int intPayload = atoi((char *)payload); //Returs '0' when not numeric
  double doublePayload = atof((char *)payload); //Returs '0' when not numeric
 
  if (topic[12] !='t' ){
    if (intPayload >= 9) intPayload = 9;
  }

  
  Serial.print(intPayload); Serial.print(" is Integer; "); Serial.print("] ");  


  if (topic[12] =='O' ) write_analog_output((intPayload*100)+25, 0); //   Ziel: Büro & Dach           alt: Küche & Dach
  if (topic[12] =='S' ) write_analog_output((intPayload*100)+25, 1); //   Ziel: Schlafzimmer & Küche  alt: Schlafzimmer & Büro
  if (topic[12] =='K' ) write_analog_output((intPayload*100)+25, 2); //   Ziel: Fabio & Lia           alt: Fabio & Lia
  
  if (millis() <= 40000){
    if (topic[12] =='t' ){
      if (energy_received ==false) {
       energy_avg = energy_avg + doublePayload; //   Energiemessung
       energy_received = true;
      }
    }
    if (topic[28] =='d' ){
      if (day_received ==false) {
       energy_day = energy_day + doublePayload; //   Energiemessung
       day_received = true;
      }
    } 
    if (topic[28] =='m' ){
      if (month_received ==false) {
       energy_month = energy_month + doublePayload; //   Energiemessung
       month_received = true;
      }
    }  
  }

      //Funktion Steuerspannung        Min    Nenn   Max [V DC]
      //Manuelle Steuerung            0,00 ≤ 0,25 ≤ 0,50
      //Pausen-Funktion               1,00 ≤ 1,25 ≤ 1,50
      //Wärmerückgewinnung Stufe 1    2,00 ≤ 2,25 ≤ 2,50
      //Wärmerückgewinnung Stufe 2    3,00 ≤ 3,25 ≤ 3,50
      //Wärmerückgewinnung Stufe 3    4,00 ≤ 4,25 ≤ 4,50
      //Unknown                       5,00 ≤ 5,25 ≤ 5,50
      //Durchlüftung Stufe 1          6,00 ≤ 6,25 ≤ 6,50
      //Durchlüftung Stufe 2          7,00 ≤ 7,25 ≤ 7,50
      //Durchlüftung Stufe 3          8,00 ≤ 8,25 ≤ 8,50
      //Unknown                       9,00 ≤ 9,25 ≤ 9,50

    
//  int int_first = intPayload / 10;
//  int int_second = intPayload - (int_first*10);
//
//  int first_bit = int_first / 4;        //HIGH-Byte berechnen
//  int second_bit = (int_first - (first_bit * 4)) / 2;
//  int third_bit = int_first - (first_bit * 4) - (second_bit *2);  //LOW-Byte berechnen

//  if (first_bit == 1) write_analog_output((int_second*100)+25, 2); 
//  if (second_bit == 1) write_analog_output((int_second*100)+25, 1); 
//  if (third_bit == 1) write_analog_output((int_second*100)+25, 0); 
  

 
  
//  String stringOne =  String(*payload); //Returns ASCII Value of first character
  
  
//    
//    Serial.print((char *)payload);  //whole Message
//    Serial.print(" is CharString; ");
//    Serial.print(stringOne);
//    Serial.print(" is String; ");
//  
//  for (int i = 0; i < length; i++) {    //Umwandeln von Byte in Character bzw String
//    Serial.print((char)payload[i]); //Only one character 
//
//    Serial.print(" is Char with lenght; ");
//  }
  
  
  
// 
  char chars_added[200];
  char* part1 = topic;
  char* part2 = "/status";
   
  strcpy(chars_added, part1);
  strcpy(chars_added + strlen(part1), part2);

 char char_payload[10];
  String string_to_convert;
  string_to_convert = String(intPayload);
  string_to_convert.toCharArray(char_payload,10); 


  client.publish(chars_added,char_payload);
  //client.publish("Ventilation/switch/status";1)
   // client.publish("office/light/brightness",(char *)payload);
   Serial.print(chars_added);
   Serial.print(" is Topic & /status; ");
   Serial.print(char_payload);
   Serial.print(" is Char_Payload.");
  
   Serial.println();


  // Switch on the LED if an 1 was received as first character
  
  if ((char)payload[0] == '0') {
  
    //digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    Serial.print("detected: 0");
//    write_analog_output(100,0);
//    write_analog_output(100,1);
//    write_analog_output(100,2);
//    write_analog_output(100,3);
  }
  
  if ((char)payload[0] == '1') {
    //digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    Serial.print("detected: 1");
//    write_analog_output(200,0);
//    write_analog_output(200,1);
//    write_analog_output(200,2);
//    write_analog_output(200,3);
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
    //ESP.restart();                //restart ESP8266
  } 
  
  
}

void read_CO2_PWM()
  {
  long th, tl, periode, p1, p2;

  th = pulseIn(pwm_pin, HIGH, 3000000);
     tl = pulseIn(pwm_pin, LOW, 3000000);
    periode = th + tl;
    p1 = periode/502; // start pulse width
    p2 = periode/251; // start and end pulse width combined 
    if (periode >0 ) co2_pwm = 5000 * (th - p1) / (periode - p2);
    if (periode ==0 ) co2_pwm = 411;
    //Serial.println();
    //Serial.print("th: "); Serial.print(th);Serial.print("\t"); Serial.print("tl: "); Serial.print(tl); Serial.print("\t");     Serial.print("ppm_pwm: "); Serial.print(ppm_pwm);Serial.print("\t");
    
    }


 void read_energy_PWM()
  {
  long th, tl, periode, p1, p2;

  th = pulseIn(input_pin2, HIGH, 30000000); //1) wait for rising edge & start timer. 2) Wait for falling edge & stop timer.
  //tl = pulseIn(input_pin2, LOW, 20000000);  //1) wait for falling edge & start timer. 2) Wait for rising edge & stop timer.
    tl=4340;
    periode = th + tl;
    //periode = periode * 0.000001;
    
    power_avg = 360000000 / periode ; //convert from "pulses per second" to "W" (10000 Pulses per kWh)
    //power_max2 = tl/10;
    energy_avg = power_avg;
    //energy_avg = (i_wind-2) / 10000; //convert from "pulses" to "kWh" (10000 Pulses per kWh)

    
    }

void write_analog_output(int Analog_Value, int Analog_Channel){
Serial.println();
Serial.print("Value: ");
Serial.print(Analog_Value);
Serial.print("0mV Channel: ");
Serial.print(Analog_Channel);
Serial.println();

    //int Analog_Value;  //Wert  0.. 1023 = 10,23V
    //int Analog_Channel; //Kanal 0..3
    
    //#define I2C_IN_ADDR   16 >> 1 // I2C-INPUT-Addresse als 7 Bit
    //#define I2C_OUT_ADDR 176 >> 1 // I2C-OUTPUT-Addresse als 7 Bit
    //const int16_t I2C_SLAVE = 0x08;
    
    //int  SEL=32;         // serielle Datenlänge
    //char Empfang[32];  // Epfangsdaten der seriellen Schnittstelle
    byte HBy;
    byte LBy;

    HBy = Analog_Value / 256;        //HIGH-Byte berechnen
    LBy = Analog_Value - HBy * 256;  //LOW-Byte berechnen
  
    Wire.beginTransmission(0x6F);         //(I2C_OUT_ADDR); // Start Übertragung zur ANALOG-OUT Karte
    Wire.write(Analog_Channel);                    // Kanal schreiben
    Wire.write(LBy);                      // LOW-Byte schreiben
    Wire.write(HBy);                      // HIGH-Byte schreiben
    Wire.endTransmission();               // Ende
}

void wind() {

    detachInterrupt(digitalPinToInterrupt(input_pin1));
    Pulse_Array[0]=1;
    Pulse_Array[1]=1;
    float rps_avg = 0.001;
    float duration_sec_avg = (Pulse_Array[i_wind-1]);
    //Serial.print("i_wind-1: "); Serial.print(i_wind-1);Serial.print("\t"); Serial.print("Pulse_Array[i_wind-1]: "); Serial.print(Pulse_Array[i_wind-1]); Serial.print("\t");

    
    //Calculate average "Rotations per Second"
    if (i_wind >=3){
      
      long rps_avg1 = (duration_sec_avg / (i_wind-2) ); //computing average time per pulse (inµs)
          
      float rps_avg2 = rps_avg1 * 0.000002; //Convert from µs to seconds, and multiply by 2 ---> because there are 2 pulses per rotation. SAme like X / 500000
      rps_avg = 1/ rps_avg2; //Kehrwert; conversionfrom "Seconds per Rotation" in "rotation per second"
      
     //Serial.println(); Serial.print("rps_avg1: "); Serial.print(rps_avg1); Serial.print("\t");Serial.print("rps_avg2: "); Serial.print(rps_avg2,4); Serial.print("\t"); Serial.print("rps_avg2: "); Serial.print(rps_avg2,4);
    
    

  //Calculate maximum "Rotations per Second"
  
   
      float rps_max = 0.1;  //Initiate maximum Speed variable with a very low number.
      for (int a=1; a <= i_wind-3; a=a+2){  //Stepsize 2--> because there are 2 pulses per rotation. We always wat to trigger the same mechanical pulse, to eliminate mechanical tolerances
    
      //Bestimmen der Einzeldrehzahlen aus dem Array
      long array_diff = (Pulse_Array[a+2]-Pulse_Array[a]);
      float duration_sec_2 = array_diff*0.000001;
      float rps_2 = 1 /(duration_sec_2);
      //Serial.print("array_diff: "); Serial.print(array_diff); Serial.print("\t"); Serial.print("rps_2: "); Serial.print(rps_2); Serial.print("\t");
        //Speichern des Maxwerts
        if (rps_max <= rps_2){
          if (rps_2 <= 100){
            rps_max = rps_2;
          }
        }
        //Serial.println(); Serial.print("rps_max: "); Serial.print(rps_max); Serial.print("\t");
        //Serial.println(); Serial.print("duration_sec_2: "); Serial.print(duration_sec_2); Serial.print("\t");
      }
    
    

    //Convert from "rotaional speed" to "wind speed" in km/h 
    
    wind_gust = 1.761 / (1 + rps_max) + 3.013 * rps_max;  // found here: https://www.amazon.de/gp/customer-reviews/R3C68WVOLJ7ZTO/ref=cm_cr_getr_d_rvw_ttl?ie=UTF8&ASIN=B0018LBFG8 (in German)
    wind_avg = 1.761 / (1 + rps_avg) + 3.013 * rps_avg; // found here: https://www.amazon.de/gp/customer-reviews/R3C68WVOLJ7ZTO/ref=cm_cr_getr_d_rvw_ttl?ie=UTF8&ASIN=B0018LBFG8 (in German)
    }else{
      wind_gust = 1.90;
      wind_avg = 1.90;
    }
    //Serial.println();Serial.print(wind_avg); Serial.print(wind_gust); Serial.print("\t");
    //Serial.print(Pulse_Array[0]); Serial.print("\t");Serial.print(Pulse_Array[1]);Serial.print("\t");Serial.print(Pulse_Array[2]);Serial.print("\t");Serial.print(Pulse_Array[3]);Serial.print("\t");Serial.print(Pulse_Array[4]);Serial.print("\t");Serial.print(Pulse_Array[5]);Serial.print("\t");Serial.print(Pulse_Array[6]);Serial.print("\t");Serial.print(Pulse_Array[7]);Serial.print("\t");Serial.print(Pulse_Array[8]);Serial.print("\t");Serial.print(Pulse_Array[9]);
    //Serial.println();



        
    //Clear Array
    for(int b=0; b<=i_wind+1; b++) Pulse_Array[b] = 0;
    i_wind2 = i_wind;
    i_wind = 1;
    Pulse_Array[0]=1;
    Pulse_Array[1]=1;
    //Serial.println();
    //Serial.print(wind_avg); Serial.print(wind_gust); Serial.print("\t");
    //Serial.print(Pulse_Array[0]); Serial.print("\t");Serial.print(Pulse_Array[1]);Serial.print("\t");Serial.print(Pulse_Array[2]);Serial.print("\t");Serial.print(Pulse_Array[3]);Serial.print("\t");Serial.print(Pulse_Array[4]);Serial.print("\t");Serial.print(Pulse_Array[5]);Serial.print("\t");Serial.print(Pulse_Array[6]);Serial.print("\t");Serial.print(Pulse_Array[7]);Serial.print("\t");Serial.print(Pulse_Array[8]);Serial.print("\t");Serial.print(Pulse_Array[9]);
    //Serial.println();
    
    attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,RISING);

  }

void stromzaehler() {
  
  long test_start = micros();
    detachInterrupt(digitalPinToInterrupt(input_pin1));

    Pulse_Array[0]=1;
    Pulse_Array[1]=1;
    float rps_avg = 0.001;
    float duration_sec_avg = (Pulse_Array[i_wind-1]);
    //Serial.print("i_wind-1: "); Serial.print(i_wind-1);Serial.print("\t"); Serial.print("Pulse_Array[i_wind-1]: "); Serial.print(Pulse_Array[i_wind-1]); Serial.print("\t");

    
    //Calculate average "Power consumption"
    if (i_wind >=3){
      
      long rps_avg1 = (duration_sec_avg / (i_wind-2) ); //computing average time per pulse (in µs)
          
      float rps_avg2 = rps_avg1 * 0.000001; //Convert from µs to seconds
      rps_avg = 1/ rps_avg2; //Kehrwert; conversionfrom "Seconds per Pulse" in "Pulses per second"
      power_avg = rps_avg * 360; //convert from "pulses per second" to "W" (10000 Pulses per kWh)
      energy_avg = energy_avg + ((i_wind-2) * 0.1); //convert from "pulses" to "Wh" (10 Pulses per Wh)
      energy_day = energy_day + ((i_wind-2) * 0.1);
      energy_month = energy_month + ((i_wind-2) * 0.1);
      
    // Serial.println(); Serial.print("rps_avg1: "); Serial.print(rps_avg1); Serial.print("\t");Serial.print("rps_avg2: "); Serial.print(rps_avg2,4); Serial.print("\t"); Serial.print("rps_avg2: "); Serial.print(rps_avg2,4);
    
    }else{
//      power_max=0.2;
//      power_max2=0.2;
//      power_avg=0.2;
//      energy_avg=0.2
        
        read_energy_PWM();
    }
    

  //Calculate maximum "Power consumption"
    power_max = 0.1;  //Initiate maximum Power variable with a very low number.
    if (i_wind >=3){
  
      for (int a=1; a <= i_wind-3; a=a+1){  
    
      //Bestimmen der Einzeldrehzahlen aus dem Array
      long array_diff = (Pulse_Array[a+1]-Pulse_Array[a]);
      float duration_sec_2 = array_diff*0.000001;         //Time from 1 Pulse to the next
      float rps_2 = 1  / (duration_sec_2);                //Kehrwert; conversionfrom "Seconds per Pulse" in "Pulses per second"
      power_max = rps_2 * 360;                                //convert from "pulses per second" to "W" (10000 Pulses per Watt)
      //Serial.println(); Serial.print("power_max: "); Serial.print(power_max); Serial.print("\t");Serial.print(power_max2);
      
   //Serial.print("array_diff: "); Serial.print(array_diff); Serial.print("\t"); Serial.print("rps_2: "); Serial.print(rps_2); Serial.print("\t");
        
     //Speichern des Maxwerts
        if (power_max2 <= power_max){
          power_max2 = power_max;
        }
        //Serial.println(); Serial.print("duration_sec_2: "); Serial.print(duration_sec_2); Serial.print("\t");
      }
    }
    
   
    //Clear Array
    for(int b=0; b<=i_wind+1; b++) Pulse_Array[b] = 0;
    i_wind2 = i_wind;
    i_wind = 1;
    Pulse_Array[0]=1;
    Pulse_Array[1]=1;
    
    
    attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING);

//long test_end = micros() - test_start;
//Serial.println(); Serial.print("dauer Stromzaehler[µs] "); Serial.print(test_end); Serial.print("\t");

}

void wind_sum_up(){
wind_collection++;
  //save maximum wind speed over multiple "void Loop()" runs
  if (wind_gust_max <= wind_gust) wind_gust_max = wind_gust;

  //save average wind speeds over multiple "void Loop()" runs
  wind_avg1 = wind_avg1+wind_avg;

  wind_avg_avg = wind_avg1/wind_collection;
  //Serial.println();
  //Serial.print("wind_avg1:"); Serial.print(wind_avg1); Serial.print("\t"); Serial.print("wind_collection:"); Serial.print(wind_collection); Serial.print("\t");
}   

  void data_published(){
  wind_avg1 = 0;
  wind_collection = 0;
  wind_gust_max = 0;
  }



//String float2str(float f, int n) {
//// Float in String umwandeln mit x Kommastellen
//// 1234.5678 max. 5 Vorkomma und 0...n Nachkommastellen
//// Eingabe 
////   float f: zahl max. 32000,0 und min. 0.00000000001 ? ggf. kleiner
////   int n: Nachkommastellen 0...10 ggf. mehr
//// Ausgabe
////   String z.B. "10.04"
//// 
//// Matthias Busse 20.5.2014 Version 2.0
//
//String s="12345.67890";
//int i, k, st=0;
//float teiler;
//double d,e;
//char c;
//
//  s = "";
//  d = f;
//  for(i=10000; i >0; i/=10) {
//    // 10000er 1000er 100er 10er 1er
//    k=(int)d/i; 
//    if((k>0) | st) {
//      st=1;
//      c=k+48; // ASCII
//      s.concat(c);
//    }
//    d = d - (k*i);
//  }
//  if(st==0) s.concat("0"); // wenigstens 0 ausgeben
//  if(n>0) s.concat("."); // Dezimalpunkt
//  teiler = 0.1;
//  for(i=0; i< n; i++) {
//    e = d / teiler; // 0.1er 0.01er 0.001er 0.0001er
//    k=(int)e;
//    c=k+48; // ASCII
//    s.concat(c);
//    d = d - (k*teiler);
//    teiler = teiler / 10.0;
//  }
//  return s;
//} // float2str
