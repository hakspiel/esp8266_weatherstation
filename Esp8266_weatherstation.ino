

//   : Code um Feuchtigkeit und Temperatur Daten auf IOT hochzuladen
//Purpose: Sending Temperature online from DHT22/AM2302 Humidity/Temperatur Sensor and
//         Maxim DB18b20 Temperature sensor


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
#include <StreamUtils.h>
#include <VL53L0X.h>
#include <VL6180X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSansOblique9pt7b.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Midea.h>

//#include <Effortless_SPIFFS.h>




// Define the Numer of this Client, every Client needs its own unique number, 
  #define Client_Number "0"





  // 0 =Outside Sensor 0 Out
  // 1 = Inside Sensor 1 Sleep
  // 2 = Inside Sensor 2 Office
  // 3 = Inside Sensor 3 Living
  // 4 = Inside Sensor 4 Lia
  // 5 = Inside Sensor 5 Fabio
  // 6 = Inside Sensor 6 roof
  // 7 = Inside Sensor 7 tank
  // 8 = Inside Sensor 8 Cellar
  // 9 = Inside Sensor 9 Garage
  //10 = Inside Sensor 10 Tech
  //11 = Inside Sensor 11 Bath
  //12 = Inside Sensor 12 Check
  //13 = Inside Sensor 13 Water
  //14 = Inside Sensor 14 Clima
  //15 = Inside Sensor 15 Tank2
  //16 = Inside Sensor 16 Friwa2
  //17 = Inside Sensor 17 Friwa1
  

// Define Updaterate of the whole loop
  #define REPORT_INTERVAL 17 // in sec; Cycle Time is approx 2 sec...
  
//WiFi Access:
  const char *ssid1     = "************";
  const char *password1 = "************";
  
  const char *ssid2     = "************";
  const char *password2 = "************";
  
  const char *ssid3     = "************";
  const char *password3 = "************";
    
// Wunderground (Download Data from Wundergound)
  #define WU_API_KEY "************"            // Create Account to download Data from Wunderground.com: https://www.wunderground.com/weather/api/d/pricing.html
  #define WUNDERGROUND "api.wunderground.com"       // DOcumentation of API Calls: https://www.wunderground.com/weather/api/d/docs
  #define WU_LOCATION "************""                 // Station ID you want to download data from

// Wunderground (Upload Data to your PWS at Wundergound)
  #define WUNDERGROUND2 "rtupdate.wunderground.com" //Create your own Weatherstation to upload Data to Wunderground.com: https://www.wunderground.com/personal-weather-station/signup.asp
  #define WUNDERGROUND_STATION_ID "************"         // Station ID you want to upload data to
  #define WUNDERGROUND_STATION_PASSWORD "************"

// PushingBox scenario DeviceId code and API
  String deviceId = "************";             // Anleitung Pushingbox: http://www.geekstips.com/android-push-notifications-esp8266-arduino-tutorial/
  const char *logServer = "api.pushingbox.com";

// MQTT: server IP, port, username and password
  const char *MQTT_SERVER_IP = "192.168.178.39"; //Hass.io
  uint16_t MQTT_SERVER_PORT = 1883;
  const char *MQTT_USER = "************";
  const char *MQTT_PASSWORD = "************";

// MQTT Version   
  #define MQTT_VERSION MQTT_VERSION_3_1_1

// MQTT define Max packet size
  #define MQTT_MAX_PACKET_SIZE 256

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

//  The callback function header needs to
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
  Adafruit_SHT31 sht31_2 = Adafruit_SHT31();
  Adafruit_BMP3XX bme;
  DHT dht(DHTPIN, DHTTYPE);
  Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, "0.de.pool.ntp.org", 3600, 600000); // Die letzten 2 Werte: 1) Zeitversatz der aktuellen Zeitzone in sekunden; 2) Updateintervall in Millisekunden (600.000 = 10min)
  PubSubClient client(wificlient2);

  
  Adafruit_MCP4725 dac; 
  BH1750 lightMeter(0x23);
  BH1750 lightMeter2(0x5C);
    // ADDR Pin: If it has voltage greater or equal to 0.7*VCC voltage the sensor address will be0x5C. 
    // if ADDR voltage is less than 0.7 * VCC the sensor address will be 0x23 (by default).
  ADC_MODE(ADC_TOUT);
  Adafruit_SSD1306 display(128, 64, &Wire, 4);  

  
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
  unsigned long time_new6=0;
  unsigned long time_old6=0;
  unsigned long offset_restart=0;
  float temp_average=0;
  float hum_average=0;
  float Taupunkt_mittel = 0;
  float Absolute_Feuchte_mittel = 0;
  float atmospheric_pressure = 0;
  float compare_temp = 0;
  float maximtemp1 = 0;
  float maximtemp2 = 0;
  float dht_temp = 0;
  float dht_hum = 0; 
  float sht31_temp = 0;
  float sht31_humidity =0;
  float sht31_2_temp = 0;
  float sht31_2_humidity =0;
  float sht31_3_temp = 0;
  float sht31_4_temp =0;  
  float sht31_3_humidity =0;
  float sht31_4_humidity =0;
  float mcp9808_temp = 0;
  bool Mitteilungssperre = false;
  bool morgenwerte_uebermittelt = false;
  bool abendwerte_uebermittelt = false;
  bool fresh_resetted = true;
  bool mcp = true;
  int ausfaelle = 0;
  float lux = 0;
  float lux1 = 0;
  float lux2 = 0;
  volatile unsigned long i_wind = 1;
  volatile unsigned long i_wind2 = 1;
  float wind3 = 1;
  float wind_gust = 0;
  float wind_gust_max = 0;
  float wind_mittelwert = 0;
  
  int wind_collection=0;
  volatile unsigned long first_micros = 0;
  volatile unsigned long last_micros = 0;
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
  float energy_avg =0; //4464000;
  float energy_day =0; //5000;
  float energy_month =0; //130000;
  const char *publishtopic1 = "1";
  const char *publishtopic0 = "0";  
  char daytopic[14];
  char monthtopic[14];
  char overalltopic[14];
  char temptopic[14];
  char comparetopic[14];
  char temptopic2[14];
  char humtopic2[14];
  char temptopic3[14];
  char humtopic3[14];  
  char lux1topic[14];
  char lux2topic[14];
  char wind_gustmaxtopic[14];
  char wind_gusttopic[14];
  char wind_topic[14];
 // char wind_mediantopic[14];
 // char wind_mediantopic2[14];
  char water_time_topic[14];
  char climate_temp_topic[14];
  char climate_mode_topic[14];
  char climate_power_topic[14];
  char climate_fan_topic[14];
  char vent_office_topic[14];
  char vent_kids_topic[14];
  char vent_sleep_topic[14];
  bool reset_day = false;
  bool reset_month = false;
  bool energy_received = false;
  bool day_received = false;
  bool month_received = false;
  double reset_energy = 4400000;
  int today = 5;
  int daylight=0;
  char daylighttopic[4];
  bool new_daylight_possible = true;
  long current_runtime =0;
  int published_data = 0;
  bool data_transfered = false;
  int Numberofpoints=0;
  int factor = 1000;
  int number_of_runs=0;
  int index_wind =0;
  int vent_office = 0;
  int vent_kids = 0;
  int vent_sleep = 0;  
float wind_array[244];
  int analog_readings;
  uint8_t climate_temp = 24;
  String climate_power = "OFF";
  String climate_mode = "off";
  String climate_fan = "auto";
  String climate_swing = "off";  
  uint8_t fan_speed = 1;
  char analog_readingstopic[14];
  time_t utcCalc;
  //char utcCalc2[10];
  String utcCalc2;  
  float room_volume = 1;
  float room_humidity = 1;
  char room_humiditytopic[14];
  const char *room_name;
const char *MQTT_CLIENT_ID; 
const char *MQTT_BASE_TOPIC;
const char *MQTT_SENSOR_TOPIC;
const char *MQTT_SENSOR_TOPIC_temp2;
const char *MQTT_SENSOR_TOPIC_temp3;
const char *MQTT_SENSOR_TOPIC_hum2;
const char *MQTT_SENSOR_TOPIC_hum3;
const char *MQTT_SENSOR_TOPIC_lux1;
const char *MQTT_SENSOR_TOPIC_lux2;
const char *MQTT_SENSOR_TOPIC_gust;
const char *MQTT_SENSOR_TOPIC_wind;
const char *MQTT_SENSOR_TOPIC_gustmax;
//const char *MQTT_SENSOR_TOPIC_wind_median;
//const char *MQTT_SENSOR_TOPIC_wind_median2;
const char *MQTT_SENSOR_TOPIC_rain;
const char *MQTT_SENSOR_TOPIC_room_humidity;
const char *MQTT_SENSOR_TOPIC_daylight;
String mqtt_topic;
String mqtt_topic_temp2;
String mqtt_topic_temp3;
String mqtt_topic_hum2;
String mqtt_topic_hum3;
String mqtt_topic_lux1;
String mqtt_topic_lux2;
String mqtt_topic_gust;
String mqtt_topic_wind;
String mqtt_topic_gustmax;
//String mqtt_topic_wind_median;
//String mqtt_topic_wind_median2;
String mqtt_topic_rain;
String mqtt_topic_room_humidity;
String mqtt_topic_daylight;
int water_time = 10000; 
int water_counter = 0;
int start_water = 0;
int start_water_override = 0;
bool sht1_found = false;
bool sht2_found = false;
bool light1_found = false;
bool light2_found = false;
bool display_found = false;
float flow_rate = 0;
float cycle_time5 = 30000;
float cycle_time = 0;
int ct5 = 30;
int wind_factor = 3;


    //if (Client_Number == "14"){
      const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
      IRMideaAC ac(kIrLed); //, false, true);  // Set the GPIO to be used to sending the message
    //}


// Class constructor
// @param[in] pin GPIO to be used when sending.
// @param[in] inverted Is the output signal to be inverted?
// @param[in] use_modulation Is frequency modulation to be used?
//IRMideaAC::IRMideaAC(const uint16_t pin, const bool inverted, const bool use_modulation)


//Interrupt for Speed Sensor
void ICACHE_RAM_ATTR Interrupt() {


  if (Client_Number == "13") debouncing_time = 500000;
  if((long)(micros() - last_micros) >= debouncing_time) {
    if (Client_Number == "13") {
      //start_water = 1;         
    }else{       
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
    



  // Room name
    if (Client_Number == "0") room_name="Out";
    if (Client_Number == "1") room_name="Sleep";
    if (Client_Number == "2") room_name="Office";
    if (Client_Number == "3") room_name="Living";
    if (Client_Number == "4") room_name="Lia";
    if (Client_Number == "5") room_name="Fabio";
    if (Client_Number == "6") room_name="Roof";
    if (Client_Number == "7") room_name="Tank";
    if (Client_Number == "8") room_name="Cellar";
    if (Client_Number == "9") room_name="Garage";
    if (Client_Number == "10")room_name="Tech";
    if (Client_Number == "11")room_name="Bath";
    if (Client_Number == "12")room_name="Check";
    if (Client_Number == "13")room_name="Water";    
    if (Client_Number == "14") room_name="Clima";
    if (Client_Number == "15") room_name="Tank2";  
    if (Client_Number == "16") room_name="Friwa2";  
    if (Client_Number == "17") room_name="Friwa1";  
      

  // MQTT: ID
    MQTT_CLIENT_ID = room_name;
    MQTT_BASE_TOPIC  = "S" ;

  // MQTT: topics
    
    mqtt_topic ="";
    mqtt_topic = String(MQTT_BASE_TOPIC) + String(Client_Number);
    MQTT_SENSOR_TOPIC= mqtt_topic.c_str();
    
    
    mqtt_topic_temp2 = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/temp2";
    MQTT_SENSOR_TOPIC_temp2= mqtt_topic_temp2.c_str();

    
    mqtt_topic_temp3 = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/temp3";
    MQTT_SENSOR_TOPIC_temp3= mqtt_topic_temp3.c_str();

    
    mqtt_topic_hum2 = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/hum2";
    MQTT_SENSOR_TOPIC_hum2= mqtt_topic_hum2.c_str();      
    
   
    mqtt_topic_hum3 = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/hum3";
    MQTT_SENSOR_TOPIC_hum3= mqtt_topic_hum3.c_str();    

   
    mqtt_topic_lux1 = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/lux1";
    MQTT_SENSOR_TOPIC_lux1= mqtt_topic_lux1.c_str();
    
   
    mqtt_topic_lux2 = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/lux2";
    MQTT_SENSOR_TOPIC_lux2= mqtt_topic_lux2.c_str();     

  
    mqtt_topic_gust = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/gust";
    MQTT_SENSOR_TOPIC_gust= mqtt_topic_gust.c_str();
    
  
    mqtt_topic_wind = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/wind";
    MQTT_SENSOR_TOPIC_wind= mqtt_topic_wind.c_str();     

  
    mqtt_topic_gustmax = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/gust_max";
    MQTT_SENSOR_TOPIC_gustmax= mqtt_topic_gustmax.c_str();
    
 
 //   mqtt_topic_wind_median = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/wind_median";
 //   MQTT_SENSOR_TOPIC_wind_median= mqtt_topic_wind_median.c_str();        

   
//    mqtt_topic_wind_median2 = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/wind_median2";
 //   MQTT_SENSOR_TOPIC_wind_median2= mqtt_topic_wind_median2.c_str();
    
    
    mqtt_topic_rain = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/rain";
    MQTT_SENSOR_TOPIC_rain= mqtt_topic_rain.c_str();    
    
   
    mqtt_topic_room_humidity = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/room_humidity";
    MQTT_SENSOR_TOPIC_room_humidity= mqtt_topic_room_humidity.c_str();
    
   
    mqtt_topic_daylight = String(MQTT_BASE_TOPIC) + String(Client_Number) + "/daylight";
    MQTT_SENSOR_TOPIC_daylight= mqtt_topic_daylight.c_str();
    

 
  //Temperature_offset
    offset_temp=0;                                //Outdoor
    if (Client_Number == "1") offset_temp=-2.7;   //Sleep; Updated: 31-01-21 old: -2.2
    if (Client_Number == "2") offset_temp=-1.0;   //Office; Updated: 21-01-05 old: -1.25
    if (Client_Number == "3") offset_temp=-1.30;  //Living; Updated: 21-01-21 old: -1.5
    if (Client_Number == "4") offset_temp=-1.25;  //Lia
    if (Client_Number == "5") offset_temp=-1.5;   //Fabio
    if (Client_Number == "6") offset_temp=-1.25;  //Rooflake
    if (Client_Number == "7") offset_temp=0;      //Tank
    if (Client_Number == "8") offset_temp=-0.66;  //Cellar
    if (Client_Number == "9") offset_temp=-0.60;  //Garage
    if (Client_Number == "10")offset_temp=0;      //Technique
    if (Client_Number == "11")offset_temp=-1.50;  //Bathroom Updated: 18-07-23 old: -1.0
    if (Client_Number == "12")offset_temp=0;      //Check; Do not change

  //Humidity_offset
    offset_hum=0;                               //Outdoor
    if (Client_Number == "1") offset_hum=11.5;  //Sleep; Updated: 31-01-21 old: 9.0
    if (Client_Number == "2") offset_hum=5.0;   //Office; Updated: 21-01-05 old: 4.0
    if (Client_Number == "3") offset_hum=5.5;   //Living; Updated: 21-01-21 old: 4.5
    if (Client_Number == "4") offset_hum=3.7;   //Lia
    if (Client_Number == "5") offset_hum=4.5;   //Fabio
    if (Client_Number == "6") offset_hum=5.0;   //Rooflake
    if (Client_Number == "7") offset_hum=0;     //Tank
    if (Client_Number == "8") offset_hum=2.2;   //Cellar
    if (Client_Number == "9") offset_hum=3.8;   //Garage
    if (Client_Number == "10")offset_hum=0;     //Technique
    if (Client_Number == "11")offset_hum=5.2;  //Bathroom  Updated: 18-07-23 old: 0.75
    if (Client_Number == "12")offset_hum=0;     //Check; Do not change
    
  //Room Volume
    room_volume=1;                                 //Outdoor
    if (Client_Number == "1") room_volume=41.36;   //Sleep;
    if (Client_Number == "2") room_volume=35.07;   //Office
    if (Client_Number == "3") room_volume=251.42;  //Living;
    if (Client_Number == "4") room_volume=48.22;   //Lia
    if (Client_Number == "5") room_volume=46.83;   //Fabio
    if (Client_Number == "6") room_volume=90.58;   //Rooflake
    if (Client_Number == "7") room_volume=1;       //Tank
    if (Client_Number == "8") room_volume=52.91;   //Cellar
    if (Client_Number == "9") room_volume=132.99;  //Garage
    if (Client_Number == "10")room_volume=32.89;   //Technique
    if (Client_Number == "11")room_volume=35.34;   //Bathroom
    if (Client_Number == "12")room_volume=35.07;   //Check; Value Office
   
  //Enable "Over the Air" WiFi Update (OTA Updates)
  //SetHostname
    ArduinoOTA.setHostname(room_name);
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
    
//if (Client_Number == "10") write_analog_output(50, 3);  //Heizung ausschalten 0-1024 ---> 50 = 0,5V

    
    EEPROM.begin(512);
    

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();

    ac.begin();         //IR Air Conditioning IR LED Output
    dht.begin();        //Asong2302 DHT22
    sht31.begin(0x44); //Sensirion SHT31
    sht31_2.begin(0x45); //Sensirion SHT31
    bme.begin_I2C();  //Bosch BMP388
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
        // CONTINUOUS_HIGH_RES_MODE_2: Factor: 2
        // Other Modes: Factor: 1
        // Max Value 16 bit : 65536 /(1.2 * Factor *(MTREG /69))
        // Calculation Here: 65536 /(1.2 * 2 *(254 /69)) = 7417 lux

    
    if (Client_Number == "13") {
      pinMode(D2, OUTPUT);    
      digitalWrite(D2, LOW);    
    }


    pinMode(pwm_pin, INPUT);
    pinMode(input_pin1, INPUT_PULLUP);//
     if (Client_Number != "14"){
       pinMode(input_pin1, INPUT_PULLUP);
     }   //Check; Value OfficepinMode(input_pin2, INPUT);//
    
    if (Client_Number != "10"){
      attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING); //was RISING
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

 //Print Headline of Measurements
    Serial.print("My Client number is "); Serial.println(Client_Number);
    Serial.println();
    Serial.println("MCP9808\tDS18B20\tDHT22\tSHT31\tBMP280\t\tAverage\tDHT22\tSHT31\tAverage\tTaupunkt\tAbs_Feuchte\tBMP280\tBMP280\tBMP280\tCycletime\tBodentemp\tCO2\tWiFi\tMQTT\tTime");

  // init the MQTT connection
    client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    client.setCallback(callback);

   //Connect to MQTT
    MQTT_connect();
    client.loop();
   
   //Sent MQTT Auto Discovery Topic to MQTT Broker, to realize Auto Discovery in HomeAssistant
  //sendMQTTDiscoveryMsg(String Sensor_kind ,  String unit ,  String Device_class ,  String Value_template ,  String State_Topic); 
    sendMQTTDiscoveryMsg("Temperature" ,  "°C" ,  "TEMPERATURE" ,  "{{value_json.T}}" ,  String(MQTT_SENSOR_TOPIC)); 
    sendMQTTDiscoveryMsg("Humidity" ,  "%" ,  "HUMIDITY" ,  "{{value_json.H}}" ,  String(MQTT_SENSOR_TOPIC)); 
    sendMQTTDiscoveryMsg("DewPoint" ,  "°C" ,  "TEMPERATURE" ,  "{{value_json.D}}" ,  String(MQTT_SENSOR_TOPIC)); 
    sendMQTTDiscoveryMsg("Abs_Hum" ,  "g/m³" ,  "" ,  "{{value_json.A}}" ,  String(MQTT_SENSOR_TOPIC)); 
    sendMQTTDiscoveryMsg("Brightness" ,  "lx" ,  "ILLUMINANCE" ,  "{{value_json.L}}" ,  String(MQTT_SENSOR_TOPIC)); 
    sendMQTTDiscoveryMsg("Temperature2" ,  "°C" ,  "TEMPERATURE" ,  "" ,  String(MQTT_SENSOR_TOPIC_temp2)); 
    sendMQTTDiscoveryMsg("Humidity2" ,  "%" ,  "HUMIDITY" ,  "" ,  String(MQTT_SENSOR_TOPIC_hum2));     
    if (Client_Number != "0") sendMQTTDiscoveryMsg("Room_Hum" ,  "g" ,  "WEIGHT" ,  "" ,  String(MQTT_SENSOR_TOPIC_room_humidity));   
    if (Client_Number != "0") sendMQTTDiscoveryMsg("CO2_Room" , "ppm" ,  "CARBON_DIOXIDE" ,  "{{value_json.C}}" ,  String(MQTT_SENSOR_TOPIC)); 
    
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Gustmax" , "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_gustmax)); 
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Gust" , "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_gust)); 
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Wind" , "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_wind)); 
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Temp3" , "°C" ,  "TEMPERATURE" ,  "" ,  String(MQTT_SENSOR_TOPIC_temp3)); 
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Hum3" ,  "%" ,  "HUMIDITY" ,  "" ,  String(MQTT_SENSOR_TOPIC_hum3)); 
    if (Client_Number == "0") sendMQTTDiscoveryMsg("AirPressure" ,  "mbar" ,  "PRESSURE" ,  "{{value_json.P}}" ,  String(MQTT_SENSOR_TOPIC));     
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Lux1" , "lx" ,  "ILLUMINANCE" ,  "" ,  String(MQTT_SENSOR_TOPIC_lux1)); 
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Lux2" , "lx" ,  "ILLUMINANCE" ,  "" ,  String(MQTT_SENSOR_TOPIC_lux2)); 
    if (Client_Number == "0") sendMQTTDiscoveryMsg("Rain" , "mm" ,  "" ,  "{{(((value|float/12)-86)*(-1))|round(0)}}" ,  String(MQTT_SENSOR_TOPIC_rain));   
 //   if (Client_Number == "0") sendMQTTDiscoveryMsg("Gust" , "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_gust));     
 //   if (Client_Number == "0") sendMQTTDiscoveryMsg("Wind" , "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_wind));     
//    if (Client_Number == "0") sendMQTTDiscoveryMsg("W_med" , "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_wind_median));    
//    if (Client_Number == "0") sendMQTTDiscoveryMsg("W_med2" , "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_wind_median2));     
   // if (Client_Number != "0") sendMQTTDiscoveryMsg("W_med" ,  "km/h" ,  "WIND_SPEED" ,  "" ,  String(MQTT_SENSOR_TOPIC_wind_median));                
    
  if (Client_Number == "13") client.publish("Sprudler/switch", publishtopic0, true);  
  if (Client_Number == "13") client.publish("Sprudler/available", publishtopic1, true);  



    
 //   sendMQTTDiscoveryMsg("Pressure" ,  "mbar" ,  "pressure" ,  "{{ value_json.P }}" ,  String(MQTT_SENSOR_TOPIC)); 
 //   sendMQTTDiscoveryMsg("Pressure" ,  "mbar" ,  "pressure" ,  "{{ value_json.P }}" ,  String(MQTT_SENSOR_TOPIC)); 


   start_water = 0;
   start_water_override = 0;
   
    
}   // End of Void Setup() !!!





void loop() {
  

unsigned long time1 = millis();

  //Check if WIFI Connection is alive
    if(WiFi.status() != WL_CONNECTED) {
      //Connect to Wifi
        wifiConnect(); //This is a sub routine
        
      //ESP.restart();                //restart ESP8266
    }

  // Check for OTA Updates, first deattach interrupts, to avaoid failurs.
    if (Client_Number != "0"){
      detachInterrupt(digitalPinToInterrupt(input_pin1));
      //detachInterrupt(digitalPinToInterrupt(input_pin2));

      ArduinoOTA.handle();
  
      attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING); // was RISING
        //attachInterrupt(digitalPinToInterrupt(input_pin2),Interrupt,FALLING);
    }
 
  //Connect to MQTT Service and check in each loop if its connected
    if (!client.connected()) {
      MQTT_connect();
    }
    client.loop();
  //   if (digitalRead(input_pin2) != HIGH) i_wind = i_wind + 10;
  //   if (digitalRead(input_pin2) != LOW) i_wind = i_wind + 100;


 unsigned long time2 = millis();
 
//falls Interrupt geschehen ist, wasser starten, und Interrupt zurücksetzen              
 if (Client_Number == "13") {
    digitalWrite(D2, LOW);  
    client.publish("Sprudler/available", publishtopic1, true); 
    if(analogRead(A0) <= 200)start_water = 1;
    if(start_water_override == 1) {
      water_control();
        start_water_override = 0;
    }
    if(start_water == 1) {
      if(analogRead(A0) >= 200) {
        water_control();
        start_water = 0;
      }
    }
    
 }

    //declaration of variables and reading out temperatures and humidity from all sensors
  if (Client_Number != "13") {
    DS18B20.requestTemperatures();
    maximtemp1 = DS18B20.getTempCByIndex(0);
    maximtemp2 = DS18B20.getTempCByIndex(1);
    dht_temp = dht.readTemperature();
    dht_hum = dht.readHumidity();
    if (mcp == true && mcp9808_temp >= -100) mcp9808_temp = mcp9808.readTempC();


  }
  
  unsigned long time3 = millis();
  
    if (sht1_found == true) sht31_temp = sht31.readTemperature() + offset_temp;
    if (sht1_found == true) sht31_humidity = sht31.readHumidity() + offset_hum;
    if (sht2_found == true) sht31_2_temp = sht31_2.readTemperature() + offset_temp - 0.15;
    if (sht2_found == true) sht31_2_humidity = sht31_2.readHumidity() + offset_hum;
    if (sht2_found == false) sht31_2_temp = 0.0 / 0.0;
    if (sht2_found == false) sht31_2_humidity = 0.0 / 0.0;
  unsigned long time4 = millis();

    analog_readings = analogRead(A0);
    
  unsigned long time5 = millis();
  
      
      if(sht31_humidity >= sht31_2_humidity) {
        sht31_3_humidity = sht31_humidity;
      }else{
        sht31_3_humidity = sht31_2_humidity;
      }
  
    if (isnan(sht31_2_temp)){
      sht31_3_temp = sht31_temp;    
      sht31_2_temp = 0;
    }else{
      if(sht31_temp >= sht31_2_temp) {
        sht31_3_temp = sht31_2_temp;
      }else{
        sht31_3_temp = sht31_temp;     
      }
    }

    
    bme.setTemperatureOversampling(BMP3_OVERSAMPLING_32X);
    bme.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bme.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    
    float bmp_pressure = bme.pressure/100.0; //aktueller Luftdruck in mbar
    float bmp_hoehe = bme.readAltitude(1024.00);
    float bmp_temp = bme.temperature;

   
    
    // Change resolution of BH1750 Light Sensor, to fit better into 16 values
    // Formula measurement Range: 65536 / ( 2.4 *(MTreg / 69))
    // Example MTreg= 254: 65536 / 8.835 = 7417
    // Example MTreg= 32: 65536 / 1.113 = 58880
    
    if (lux1 >=1000.0) {
      if (light1_found == true) lightMeter.setMTreg(32);
    }else{
      if (light1_found == true) lightMeter.setMTreg(254);
    }
    
    if (lux2 >=1000.0) {
      if (light2_found == true) lightMeter2.setMTreg(32);
    }else{
      if (light2_found == true) lightMeter2.setMTreg(254);
    }


if (light1_found == true) lux1 = lightMeter.readLightLevel();
if (light2_found == true) lux2 = lightMeter2.readLightLevel();

    if (lux1 >= 58500.0) lux1 = 58500.0;
    if (lux2 >= 58500.0) lux2 = 58500.0;
    
    // Schmitt Trigger for Daylight
    if (lux1 >=0.1 && lux2 >=0.1) {
      if (daylight == 1 && new_daylight_possible == true) {
          current_runtime = millis();
          //new_daylight_possible = false;
          daylight = 2;
          daylighttopic[0] = 'O';  // einfache Anfürhungszeichen verwenden
          daylighttopic[1] = 'N';  // einfache Anfürhungszeichen verwenden
          daylighttopic[2] = 'N';  // einfache Anfürhungszeichen verwenden
          daylighttopic[3] = '\0';
          //daylighttopic[4] ---> Last char is invalid. see: https://www.arduino.cc/reference/de/language/variables/data-types/array/
      }
      
      if (daylight ==2 || daylight ==0) {
          //new_daylight_possible = false;
          daylight = 2;
          daylighttopic[0] = 'O';  // einfache Anfürhungszeichen verwenden
          daylighttopic[1] = 'N';  // einfache Anfürhungszeichen verwenden
          daylighttopic[2] = 'N';  // einfache Anfürhungszeichen verwenden
          daylighttopic[3] = '\0';
          //daylighttopic[4] ---> Last char is invalid. see: https://www.arduino.cc/reference/de/language/variables/data-types/array/
      }
      
    }else{
    
      if (daylight == 2 && new_daylight_possible == true) {
          current_runtime = millis();
          //new_daylight_possible = false;
          daylight = 1;
          daylighttopic[0] = 'O';  // einfache Anfürhungszeichen verwenden
          daylighttopic[1] = 'F';  // einfache Anfürhungszeichen verwenden
          daylighttopic[2] = 'F';  // einfache Anfürhungszeichen verwenden
          daylighttopic[3] = '\0';
          //daylighttopic[4] ---> Last char is invalid. see: https://www.arduino.cc/reference/de/language/variables/data-types/array/
      }
      if (daylight <=1) {
          //new_daylight_possible = false;
          daylight = 1;
          daylighttopic[0] = 'O';  // einfache Anfürhungszeichen verwenden
          daylighttopic[1] = 'F';  // einfache Anfürhungszeichen verwenden
          daylighttopic[2] = 'F';  // einfache Anfürhungszeichen verwenden
          daylighttopic[3] = '\0';
          //daylighttopic[4] ---> Last char is invalid. see: https://www.arduino.cc/reference/de/language/variables/data-types/array/
      }
    }

 //   if (millis() - current_runtime >= 21000000) new_daylight_possible = true;
    //if (lux2>=0.2) new_daylight_possible = true;
       
    if (isnan(lux1)) lux1 = 0;
    if (isnan(lux2)) lux2 = 0;
    if (lux1 <= 0) lux1 = 0;
    if (lux2 <= 0) lux2 = 0;
    //lux = (lux1 + lux2)/2;
   
    if(lux1 >= lux2) {
        lux = lux1;
      }else{
        lux = lux2;
      }
    
    
    //lux_vl61 = vl.readLux(VL6180X_ALS_GAIN_5);
    //range_vl61 = vl.readRange();
    //VL53L0X_RangingMeasurementData_t measure;
    //range_vl53= measure.RangeMilliMeter;
    
    //range_vl53 = VL53L0.readRangeSingleMillimeters();
    range_vl61 = VL6180.readRangeSingleMillimeters();




 
         
     //Get current Time from NTP Server
        timeClient.update();
        utcCalc = timeClient.getEpochTime();
        today = day(utcCalc);
        int summer_offset = 0;  //offset in Wintetime
        if (summertime(year(utcCalc), month(utcCalc), day(utcCalc), hour(utcCalc), 1)) summer_offset=3600; //create offset in summertime
        timeClient.setTimeOffset(3600 + summer_offset); 
        utcCalc = timeClient.getEpochTime();
        utcCalc2 = timeClient.getFormattedTime();
        
       unsigned long time6 = millis();
        
    //Resets:
    if (Client_Number == "10"){
             
       //Reset durchführen einmal pro Tag
       if (hour(utcCalc) == 0){         //Wenn es ein uhr nachts ist
     
          if (reset_day == false){
            
            client.publish("homeassistant/sensor/energy/last_day",daytopic ,true);
            energy_day = 0;
            reset_day =true;
          }
            
       }

  
      if (hour(utcCalc) == 1) reset_day = false;
    
       //Reset durchführen einmal pro Monat
       if (day(utcCalc) == 1){ 
        if (hour(utcCalc) == 0){ 
          if (reset_month == false){
            client.publish("homeassistant/sensor/energy/last_month",monthtopic ,true);
            energy_month = 0;
            reset_month == true;
          }
        }
       }
       
       if (day(utcCalc) == 1) reset_month = false;
       
    }else{
      if (millis() >= 21600000 + offset_restart){       //Wenn die Laufzeit des Geräts größer ist als 6 Stunden dann Reset
        ESP.restart();                //restart ESP8266
      }
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
   
    if (isnan(sht31_3_temp)) {
      ++ausfaelle;
      sht31_3_temp = 0;
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
    hum_average = (dht_hum + sht31_3_humidity) / 2;
    if (isnan(dht_hum)) {
      hum_average = sht31_3_humidity;
      } 
    if (isnan(sht31_3_humidity)) {
      hum_average = dht_hum;
      } 
    if (isnan(sht31_2_humidity)) {
      sht31_2_humidity = 0;
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

  //calculate absolute humidity roomspecific
  room_humidity = room_volume * Absolute_Feuchte_mittel;


client.loop();

// Bestimmung des CO2 Werts
    if (Client_Number != "0"){ 
      if (millis() >= 25000){       //Wenn die Laufzeit des Geräts größer ist als 22sec MH Z19b CO2 Sensor needs 20sec boot time
        if (Client_Number == "17") {
          read_periode();
        }
        else {
        read_CO2_PWM();
        }
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
   
    float Sea_level = 510.0;  //Meereshöhe + Plattformhöhe des Spielturms: 508,6m + 1,4m = 510.0m
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

  if (atmospheric_pressure <=960.0) atmospheric_pressure = 1000.0;
   
  //Bestimmen der Programmlaufzeit
    time_new = millis();
    cycle_time = time_new - time_old;
    time_old = millis();

 
    
 client.loop();
 unsigned long time7 = millis();
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
    Serial.print("ADC Value: "); Serial.print(analogRead(A0)); Serial.print("\t");
   // Serial.print(WiFi.status()); Serial.print(" "); Serial.print("\t");
    //Serial.print(ausfaelle); Serial.print(""); Serial.print("\t");
    Serial.print(lux); Serial.print("lx"); Serial.print("\t");
    Serial.print(lux2); Serial.print("lx2"); Serial.print("\t");
    Serial.print(wind_gust); Serial.print(" gust km/h"); Serial.print("\t");
    Serial.print(wind_mittelwert); Serial.print(" avg km/h"); Serial.print("\t");
    Serial.print(i_wind2); Serial.print("Pulse"); Serial.print("\t");
    //Serial.println();
    Serial.print(utcCalc);  Serial.print("utc "); Serial.print("\t");
    Serial.print(utcCalc2);  Serial.print("utc2 "); Serial.print("\t");
  //  Serial.print(utcCalc3);  Serial.print("utc3 "); Serial.print("\t");
    Serial.print(millis());  Serial.print("millis"); Serial.print("\t");

    Serial.print(lux_vl61); Serial.print("lx_ST"); Serial.print("\t");


    unsigned long time8 = millis();
 

    
//    Serial.print(range_vl61); Serial.print("mm"); Serial.print("\t");
//    Serial.print(range_vl53); Serial.print("mm"); Serial.print("\t");
//    Serial.print(MQTT_CLIENT_ID); Serial.print(""); Serial.print("\t");
   
   
    Serial.println(); 

   client.loop();


  //falls Interrupt geschehen ist, wasser starten, und Interrupt zurücksetzen
  if (Client_Number == "13"){
    if(analogRead(A0) <= 200)start_water = 1; 
    if(start_water_override == 1) {
      water_control();
      start_water_override = 0;
    }
    if(start_water == 1) {
      if(analogRead(A0) >= 200) {
        water_control();
        start_water = 0;
      }
    }
  }

  if (display_found = true){
      // FreeMono12pt7b.h		
      // FreeSansBoldOblique12pt7b.h
      // FreeMono18pt7b.h		
      // FreeSansBoldOblique18pt7b.h
      // FreeMono24pt7b.h		
      // FreeSansBoldOblique24pt7b.h
      // FreeMono9pt7b.h			
      // FreeSansBoldOblique9pt7b.h
      // FreeMonoBold12pt7b.h		
      // FreeSansOblique12pt7b.h
      // FreeMonoBold18pt7b.h		
      // FreeSansOblique18pt7b.h
      // FreeMonoBold24pt7b.h		
      // FreeSansOblique24pt7b.h
      // FreeMonoBold9pt7b.h		
      // FreeSansOblique9pt7b.h
      // FreeMonoBoldOblique12pt7b.h	
      // FreeSerif12pt7b.h
      // FreeMonoBoldOblique18pt7b.h	
      // FreeSerif18pt7b.h
      // FreeMonoBoldOblique24pt7b.h	
      // FreeSerif24pt7b.h
      // FreeMonoBoldOblique9pt7b.h	
      // FreeSerif9pt7b.h
      // FreeMonoOblique12pt7b.h		
      // FreeSerifBold12pt7b.h
      // FreeMonoOblique18pt7b.h		
      // FreeSerifBold18pt7b.h
      // FreeMonoOblique24pt7b.h		
      // FreeSerifBold24pt7b.h
      // FreeMonoOblique9pt7b.h		
      // FreeSerifBold9pt7b.h
      // FreeSans12pt7b.h		
      // FreeSerifBoldItalic12pt7b.h
      // FreeSans18pt7b.h		
      // FreeSerifBoldItalic18pt7b.h
      // FreeSans24pt7b.h		
      // FreeSerifBoldItalic24pt7b.h
      // FreeSans9pt7b.h			            <-------    <------
      // FreeSerifBoldItalic9pt7b.h
      // FreeSansBold12pt7b.h		
      // FreeSerifItalic12pt7b.h
      // FreeSansBold18pt7b.h		
      // FreeSerifItalic18pt7b.h
      // FreeSansBold24pt7b.h		
      // FreeSerifItalic24pt7b.h
      // FreeSansBold9pt7b.h		
      // FreeSerifItalic9pt7b.h
      
      // write Data to display
       
        // display.clearDisplay();
        // //display.setFont(&FreeSans9pt7b);
        // display.setTextSize(1);                       // Normal 1:1 pixel scale
        // display.setTextColor(SSD1306_WHITE);          // Draw white text
        // display.setCursor(0,0);                       // first line starting here
        // display.print("T: "); display.print(sht31_3_temp);//(temp_average); * //*  
        // display.print(char(247)); display.print("C");
        // display.setCursor(0,0+7+3);                   // after first line = 7 pixel( size of first line) + 3 pixel gap
        // display.print("H: "); display.print(hum_average);  display.print("%");
        // display.setCursor(0,7+7+3+3);                 // after 2nd line = 2x7 pixel + 2x3 pixel gap
        // //display.print("Time: "); display.print(utcCalc2); 
        // display.print("Dew: "); display.print(Taupunkt_mittel); display.print(char(247)); display.print("C");
        // display.setCursor(0,7+7+7+3+3+3);             // after 3rd line =  3x7 pixel + 3x3 pixel gap
        //   // display.print("Abs Hum: "); display.print(Absolute_Feuchte_mittel); display.print("g/m"); display.print(char(252));
       
        // utcCalc2 = timeClient.getFormattedTime();  
       
        // display.setCursor(0,7+7+7+7+3+3+3+3);         // after 4th line =  4x7 pixel + 4x3 pixel gap
        // display.print("Room Hum: "); display.print(room_humidity); display.print("ml"); 
        //    //display.print("Time: "); display.print(hour(utcCalc));   display.print(":");  display.print(minute(utcCalc)); display.print(":");  display.print(second(utcCalc)); 
        // display.setCursor(0,7+7+7+7+7+3+3+3+3+3);         // after 5th line =  5x7 pixel + 5x3 pixel gap
        // display.print("Time: "); display.print(utcCalc2); 
        // display.setCursor(0,7+7+7+7+7+7+3+3+3+3+3+3);         // after 5th line =  5x7 pixel + 5x3 pixel gap
        //     //display.print("test/Z7");
        // display.display();
          
       display.clearDisplay();
        display.setFont(&FreeSans9pt7b);
        //display.setFont(&FreeSerifOblique9pt7b);
        display.setTextSize(1);                       // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);          // Draw white text
        display.setCursor(0,12);                       // first line starting here
        display.print(""); display.print(sht31_4_temp);//(temp_average); * //*  
        display.print(char(247)); display.print("°C");
        display.setCursor(0,0+24+3);                   // after first line = 7 pixel( size of first line) + 3 pixel gap
        display.print(""); display.print(sht31_4_humidity);  display.print("%");
        display.setCursor(0,36+3+3);                 // after 2nd line = 2x7 pixel + 2x3 pixel gap
        //display.print("Time: "); display.print(utcCalc2); 
        //display.print("Dew: "); display.print(Taupunkt_mittel); display.print(char(247)); display.print("C");
        //display.setCursor(0,14+14+14+3+3+3);             // after 3rd line =  3x7 pixel + 3x3 pixel gap
          // display.print("Abs Hum: "); display.print(Absolute_Feuchte_mittel); display.print("g/m"); display.print(char(252));
       
        //utcCalc2 = timeClient.getFormattedTime();  
       //int currentminutes = timeClient.getMinutes();
        //display.setCursor(0,7+7+7+7+3+3+3+3);         // after 4th line =  4x7 pixel + 4x3 pixel gap
       // display.print("Room Hum: "); display.print(room_humidity); display.print("ml"); 
           //display.print("Time: "); display.print(hour(utcCalc));   display.print(":");  display.print(minute(utcCalc)); display.print(":");  display.print(second(utcCalc)); 
        //display.setCursor(0,7+7+7+7+7+3+3+3+3+3);         // after 5th line =  5x7 pixel + 5x3 pixel gap
      if (timeClient.getMinutes() <= 9) {
        display.print(""); display.print(timeClient.getHours()); display.print(":0"); display.print(timeClient.getMinutes()); 
      } 
      if (timeClient.getMinutes() >= 10){
        display.print(""); display.print(timeClient.getHours()); display.print(":"); display.print(timeClient.getMinutes()); 
      }  
        display.setCursor(0,48+3+3+3);         // after 5th line =  5x7 pixel + 5x3 pixel gap
            display.print(room_name);
        display.display();

  }



  //long too_long = ((REPORT_INTERVAL * 1000) + 10000);
  //if (cycle_time >= too_long){
  //  ESP.restart();
  //}
  
 
  //Analogwerte Ausgeben
    //write_analog_output(100,0);
    //write_analog_output(200,1);
    //write_analog_output(300,2);
    //write_analog_output(400,3);
    

    client.loop();
   
 
      time_new5 = millis();
      time_new6 = millis();
      float cycle_time4 = time_new5 - time_old5;
      cycle_time5 = time_new6 - time_old6;
      
      factor = 1000;
      if (Client_Number == "0")  factor = 5000;
      if (Client_Number == "1")  factor = 1000;
      if (Client_Number == "2")  factor = 1000;
      if (Client_Number == "3")  factor = 1000;
      if (Client_Number == "4")  factor = 1000;
      if (Client_Number == "5")  factor = 1000;
      if (Client_Number == "6")  factor = 1000;
      if (Client_Number == "7")  factor = 1000;
      if (Client_Number == "8")  factor = 1000;
      if (Client_Number == "9")  factor = 1000;
      if (Client_Number == "10") factor = 1000;
      if (Client_Number == "11") factor = 1000;
      if (Client_Number == "12") factor = 1000;
      if (Client_Number == "13") factor = 1000;
      if (Client_Number == "14") factor = 500;
      if (Client_Number == "15") factor = 1000;
      if (Client_Number == "16") factor = 1000;
      if (Client_Number == "17") factor = 100;
      if (Client_Number == "18") factor = 1000;





 //Messen der Windgeschwindigkeit

 
      if (cycle_time5 >= wind_factor *1000 ){               //Wind gusts definition: need at least 3sec sum up time
        if (Client_Number == "0"){
          wind();
          wind_sum_up();
          time_old6 = time_new6;
           
            
            //data_transfered = true;

        }
      }  









      
      if (cycle_time4 >= factor * REPORT_INTERVAL){     //wenn Factor =1000 wird Dieser Befehle alle 18 sekunden ausgeführt; 
        
        if (Client_Number != "10"){
          if (millis() >= 30000){ 
            if (Client_Number != "0") detachInterrupt(digitalPinToInterrupt(input_pin1));
            if (Client_Number != "0") publishData(temp_average, hum_average, atmospheric_pressure, Taupunkt_mittel, Absolute_Feuchte_mittel, lux, co2_pwm, wind_gust, wind_mittelwert);    // Publish Data to MQTT Broker for all sensors expect technique
            if (Client_Number != "0") attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING); //was RISING
            //data_transfered = true;

          }
        }else{
          stromzaehler();
          publishData(temp_average, hum_average, atmospheric_pressure, Taupunkt_mittel, Absolute_Feuchte_mittel, lux, co2_pwm, power_avg, power_avg);         // Publish Data to MQTT Broker for technique Sensor
        }
        
         
      
      time_old5 = time_new5;
      }
      else{
          Serial.print("No_MQTT_Transfer"); Serial.print("\t");
      }



    
//    if (cycle_time5 >= 1000 * REPORT_INTERVAL){
//        if (Client_Number == "0"){
//          if (millis() >= 30000){ 
//            if (data_transfered == false){
//              detachInterrupt(digitalPinToInterrupt(input_pin1));
//              publishData2();    // Publish Data to MQTT Broker 
//              data_published(); 
//              attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING);  //was RISING
//            }
//          data_transfered = false;
//          time_old6 = time_new6;
//        }
//      }
//    }  

           
 client.loop();
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
  if (cycle_time < 6000) {
    //delay(1100);   //Never change this 4100ms adder!!!
    client.loop();  
    //delay(1000);   //Never change this 4100ms adder!!!
    //client.loop();    
    //client.publish("Test/Test",publishtopic0);     
    //delay(1000);   //Never change this 4100ms adder!!!
    //client.loop();  
    //delay(1000);   //Never change this 4100ms adder!!!
    //client.loop();   
    //delay(1000);   //Never change this 4100ms adder!!!
    //client.loop();      
    //client.publish("Test/Test",publishtopic1);      
    //int cnt = REPORT_INTERVAL; //- (cycle_time - REPORT_INTERVAL);
    //while(cnt--) delay(1000);
}

    unsigned long time9 = millis();
    Serial.print(" time1-2: "); Serial.print(time2-time1); Serial.print("\t");
    Serial.print(" time2-3: "); Serial.print(time3-time2); Serial.print("\t");
    Serial.print(" time3-4: "); Serial.print(time4-time3); Serial.print("\t");
    Serial.print(" time4-5: "); Serial.print(time5-time3); Serial.print("\t");
    Serial.print(" time5-6: "); Serial.print(time6-time5); Serial.print("\t");
    Serial.print(" time6-7: "); Serial.print(time7-time6); Serial.print("\t");
    Serial.print(" time7-8: "); Serial.print(time8-time6); Serial.print("\t");
    Serial.print(" time8-9: "); Serial.print(time9-time7); Serial.print("\t");
    Serial.print(" time_all: "); Serial.print(time9-time1); Serial.print("\t");
    Serial.println();
    
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
      if (address == 68) sht1_found = true;
      if (address == 69) sht2_found = true;
      if (address == 35) light1_found = true;
      if (address == 92) light2_found = true;
      if (address == 60) display_found = true;
      
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

//   // Use WiFiClient class to create TCP connections
//   //const int httpPort = 80;
// //  if (!client2.connect(WUNDERGROUND, 80)) {
// //    Serial.println(F("connection failed"));
// //    return;
// //  }

//   Serial.println();
//   Serial.print(WUNDERGROUND_REQ);
  
//   client2.connect(WUNDERGROUND, 80);
//   client2.print(WUNDERGROUND_REQ);
//   client2.flush();

//  // Skip HTTP headers; They end with an empty line
//   char endOfHeaders[] = "\r\n\r\n";
//   client2.setTimeout(10000); //http Timeout
//   client2.find(endOfHeaders);

  
//   UserData userData;
//   if (readReponseContent(&userData)) {
//     printUserData(&userData);
//   }


client2.stop();
}


// // extract Data from the JSON File
// bool readReponseContent(struct UserData* userData) {
//   // Compute optimal size of the JSON buffer according to what we need to parse.
//   // See https://bblanchon.github.io/ArduinoJson/assistant/
//   const size_t BUFFER_SIZE = 
//       JSON_OBJECT_SIZE(8)    // the root object has 8 elements
//       + JSON_OBJECT_SIZE(5)  // the "address" object has 5 elements
//       + JSON_OBJECT_SIZE(2)  // the "geo" object has 2 elements
//       + JSON_OBJECT_SIZE(3)  // the "company" object has 3 elements
//       + 512;                 // additional space for strings

//   // Allocate a temporary memory pool
//   DynamicJsonBuffer jsonBuffer(BUFFER_SIZE);

//   JsonObject& root = jsonBuffer.parseObject(client2);

//   if (!root.success()) {
//     Serial.println("JSON parsing failed!");
//     return false;
//   }

//   // Here were copy the strings we're interested in
//   strcpy(userData->temp_c, root["current_observation"]["temp_c"]);
//   strcpy(userData->relative_humidity, root["current_observation"]["relative_humidity"]);
//   strcpy(userData->dewpoint_c, root["current_observation"]["dewpoint_c"]);
//   strcpy(userData->observation_time_rfc822, root["current_observation"]["observation_time_rfc822"]);
//   // It's not mandatory to make a copy, you could just use the pointers
//   // Since, they are pointing inside the "content" buffer, so you need to make
//   // sure it's still in memory when you read the string

//   return true;
// }

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
  compare_temp = float_Userdata_temp_c;    // - temp_average;

  dtostrf(compare_temp, 10, 2, comparetopic);//convert float to char
  client.publish("homeassistant/sensor/comparetemp",comparetopic, true);

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
String temp = String(sht31_3_temp * 1.8 + 32);
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

  if (!client.connected()) {
     // Attempt to connect
    Serial.print("Not MQTT connected, try to connect...");     
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


void sendMQTTDiscoveryDeviceMsg(String Sensor_kind ,  String unit ,  String Device_class ,  String Value_template ,  String State_Topic) {
 
  // This is the topic this program will send the state of this device to.
  //String stateTopic = "home/plants/" + String(sensorNumber) + "/state";

  // This is the discovery topic for this specific sensor
  
  

//  String room_name2 = String(room_name);
//  room_name2.toLowerCase();
// 
//  client.setBufferSize(512);   //IMPORTANT: defines the lenght of JSON message
//  
//  String discoveryTopic = "homeassistant/sensor/" + room_name2 +"/config";
//  
//  DynamicJsonDocument doc(512);
//  char buffer[512];
//  
//  JsonObject device = doc.createNestedObject("device");                          // create substructure in JSON
//  device["name"] = String(room_name);                                                // define Device name where entity is part of it
//  device["sa"] = room_name2;                                                  // define Device name where entity is part of it
//  device["mdl"] = String("DIY");                                                  // define Device name where entity is part of it    
//  device["ids"][0] = room_name2;                                                  // define Device name where entity is part of it
//                                              
//  
//serializeJson(doc, buffer);
//
// client.publish(discoveryTopic.c_str(), buffer, true);             //This Message needs to RETAIN too
//
//delay(500);

}

void sendMQTTDiscoveryMsg(String Sensor_kind ,  String unit ,  String Device_class ,  String Value_template ,  String State_Topic) {
 
/*
  // String Sensor_kind = "Temp";
  // String unit = "°C";
  // String Device_class = "temperature";
  // String Value_template = "{{ value_json.T }}";
  // String State_Topic = MQTT_SENSOR_TOPIC;
all:
- temperature T
- humidity H
- Pressure P
- absolute_humidity A
- lux L
- co2 C
- dew D
- Gust G
- Wind W
 
Special:

- lux2
*/

  // This is the topic this program will send the state of this device to.
  //String stateTopic = "home/plants/" + String(sensorNumber) + "/state";

  // This is the discovery topic for this specific sensor
  
  

 String room_name2 = String(room_name);
 String add_string = String("Room_Sensor");
 room_name2.toLowerCase();
 

  //String discoveryTopic = "homeassistant/sensor/" + String(State_Topic) + "/" + Sensor_kind +"/config";
  // String discoveryTopic = "homeassistant/sensor/" + String(State_Topic) + "/config"; MQTT_SENSOR_TOPIC
  
  client.setBufferSize(512);   //IMPORTANT: defines the lenght of JSON message
  
  String discoveryTopic = "homeassistant/sensor/" + String(MQTT_SENSOR_TOPIC) + "/" + Sensor_kind +"/config";
  
  DynamicJsonDocument doc(512);
  char buffer[512];

  doc["name"] = Sensor_kind +" " + String(room_name);    //Name of the Entity in Home Assistant
  
  Sensor_kind.toLowerCase();
  doc["obj_id"] = room_name2 +"_" + Sensor_kind;   //Entity ID in Home Assistant
  doc["uniq_id"] = room_name2 +"_" + Sensor_kind;   // Should be identical to obj_id
  doc["unit_of_meas"] = unit;
  if (Value_template != "") doc["val_tpl"] = Value_template;                    //Value Template; Here: JSON
  doc["stat_t"] = State_Topic;                                                    // = State Topic  
  doc["frc_upd"] = true;                                                          // Force Update
  doc["ret"] = true;                                                              //Retain
  doc["stat_cla"] = "measurement";                                                //needed for Long term statistics  https://developers.home-assistant.io/docs/core/entity/sensor/#long-term-statistics
  if (Device_class != "") doc["dev_cla"] = Device_class;                          // Device Class needed for Long term statistics 
//  doc["state_class"] = "measurement";                                                //needed for Long term statistics  https://developers.home-assistant.io/docs/core/entity/sensor/#long-term-statistics
//  if (Device_class != "") doc["device_class"] = Device_class;                          // Device Class needed for Long term statistics 

  
  JsonObject device = doc.createNestedObject("device");                          // create substructure in JSON
  device["name"] = add_string +"_" + String(room_name);                                                // define Device name where entity is part of it
  device["sa"] = add_string +"_" + room_name2;                                                  // define Device name where entity is part of it
  device["mdl"] = add_string +"_" + String("DIY");                                                  // define Device name where entity is part of it    
  device["mf"] = add_string +"_" + String("DIY");                                                  // define Device name where entity is part of it   
  device["sw"] = add_string +"_" + String("1.0");                                                  // define Device name where entity is part of it   
  device["ids"][0] = add_string +"_" + room_name2;                                                  // define Device name where entity is part of it
 
  
  //JsonArray ids = doc.createNestedArray("ids");
  //ids.add(room_name2);                                                  // define Device name where entity is part of it
                                                   
  
  
    //doc["stat_tpl"]                                         // = stateTemplate; 
  //doc["sa"] = room_name;                                    //suggested_area  --> does not show up in Home Assistant

//EepromStream eepromStream(0, 512);

serializeJson(doc, buffer);

 // size_t n = serializeJson(doc, eepromStream);
//buffer = eepromStream.readString().c_str();

 // client.publish(discoveryTopic.c_str(), eepromStream.c_str());
  client.publish(discoveryTopic.c_str(), buffer, true);             //This Message needs to RETAIN too

//EEPROM.commit();
//eepromStream.flush();  // (for ESP)

delay(500);

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
 
  //if (p_co2 <= 350) p_co2 = 350;
  
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
    
  //StaticJsonBuffer<270> jsonBuffer;
  

  // Get the root object in the document
 // alt JsonObject& root = jsonBuffer.createObject();
     
     DynamicJsonDocument root(1024); //neu
     char data[270];//neu
  
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
 // if (p_wind_gust < 100) root["G"] = (String)p_wind_gust;
  //if (p_wind_avg_avg < 100) root["W"] = (String)p_wind_avg_avg;
  //if (p_wind_gust2 >= 9999) p_wind_gust2=9999;
  //if (p_wind_gust >= 100) root["G"] = (String)p_wind_gust2;
  //if (p_wind_avg_avg >= 699) p_wind_avg_avg2=699;
  //if (p_wind_avg_avg >= 100) root["W"] = (String)p_wind_avg_avg2;
  
  //root.prettyPrintTo(Serial);
  //Serial.println("");


  dtostrf(energy_day, 10, 2, daytopic);//convert float to char
  dtostrf(energy_month, 10, 2, monthtopic);//convert float to char
  dtostrf(energy_avg, 10, 2, overalltopic);//convert float to char
  
  dtostrf(sht31_2_temp, 10, 2, temptopic2);//convert float to char
  dtostrf(sht31_2_humidity, 10, 2, humtopic2);//convert float to char  
  dtostrf(sht31_3_humidity, 10, 2, humtopic3);//convert float to char
  dtostrf(sht31_3_temp, 10, 2, temptopic3);//convert float to char
  dtostrf(lux1, 10, 2, lux1topic);//convert float to char
  dtostrf(lux2, 10, 2, lux2topic);//convert float to char
  
  //dtostrf(wind_gust_max, 10, 2, wind_gustmaxtopic);//convert float to char
  //dtostrf(wind_gust, 10, 2, wind_gusttopic);//convert float to char
  //dtostrf(p_wind_avg_avg, 10, 2, wind_topic);//convert float to char
  //dtostrf(wind_mittelwert, 10, 2, wind_mediantopic);//convert float to char
  //dtostrf(wind_mittelwert2, 10, 2, wind_mediantopic2);//convert float to char
  
  dtostrf(room_humidity, 10, 2, room_humiditytopic);//convert float to char
  dtostrf(analog_readings, 10,2, analog_readingstopic);
  
  itoa(climate_temp, climate_temp_topic, 10);//convert int to char
  itoa(vent_kids, vent_kids_topic, 10);//convert int to char
  itoa(vent_sleep, vent_sleep_topic, 10);//convert int to char
  itoa(vent_office, vent_office_topic, 10);//convert int to char
  itoa(water_time/1000, water_time_topic, 10);//convert int to char    
 
  climate_mode.toCharArray(climate_mode_topic, 10); //convert string to char
  climate_power.toCharArray(climate_power_topic, 10); //convert string to char
  climate_fan.toCharArray(climate_fan_topic, 10); //convert string to char
  
  //char data[270];
  //root.printTo(data, root.measureLength() + 1);//alt
  
  client.setBufferSize(512);   //IMPORTANT: defines the lenght of JSON message
  
  size_t n = serializeJson(root, data);//neu

   //client.publish(discoveryTopic.c_str(), buffer, n);
  
   //client.publish("homeassistant/sensor/temp",temptopic, true);
   //client.publish("homeassistant/sensor/hum",humtopic, true);
   client.publish(MQTT_SENSOR_TOPIC, data, true);
   
 //  String stopic;
 //  stopic = MQTT_SENSOR_TOPIC;
   //strcpy(stopic, MQTT_SENSOR_TOPIC);

//char buf2[20];

  // String testst = stopic + "/temp2";
  // testst.toCharArray(buf2, 20);
   
// if (Client_Number == "0") sendMQTTMsg("/temp2", temptopic)
 //if (Client_Number == "0") sendMQTTMsg("/temp3", temptopic3)  
 
 //if (Client_Number == "0") client.publish(buf2,temptopic, true);
  
  // testst = stopic + "/temp3";
   //testst.toCharArray(buf2, 20);
   

   // if (Client_Number == "0") client.publish(buf2,temptopic3, true);

client.publish(MQTT_SENSOR_TOPIC_temp2,temptopic2, true);
client.publish(MQTT_SENSOR_TOPIC_hum2,humtopic2, true);    

    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_temp3,temptopic3, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_hum3,humtopic3, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_lux1,lux1topic, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_lux2,lux2topic, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_daylight,daylighttopic, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_rain,analog_readingstopic, true);
    
    // if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_gustmax,wind_gustmaxtopic, true);
    // if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_gust,wind_gusttopic, true);
    // if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_wind,wind_topic, true);
    // if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_wind_median,wind_mediantopic, true);
    // if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_wind_median2,wind_mediantopic2, true);

    
    if (Client_Number != "0") client.publish(MQTT_SENSOR_TOPIC_room_humidity,room_humiditytopic, true);  
    
    if (Client_Number == "10") client.publish("Ventilation_Office/switch/status",publishtopic1 ,true);
    if (Client_Number == "10") client.publish("Ventilation_Sleep/switch/status",publishtopic1 ,true);
    if (Client_Number == "10") client.publish("Ventilation_Kids/switch/status",publishtopic1 ,true); 
    
    if (Client_Number == "10") client.publish("Ventilation_Office/Power/status",vent_office_topic ,true);
    if (Client_Number == "10") client.publish("Ventilation_Sleep/Power/status",vent_sleep_topic ,true);
    if (Client_Number == "10") client.publish("Ventilation_Kids/Power/status",vent_kids_topic ,true);  
     
    if (Client_Number == "10") client.publish("Ventilation_Office/available", publishtopic1);   
    if (Client_Number == "10") client.publish("Ventilation_Sleep/available", publishtopic1);
    if (Client_Number == "10") client.publish("Ventilation_Kids/available", publishtopic1);
    
    if (Client_Number == "10") client.publish("Heating/Switch/status",publishtopic1); 
    if (Client_Number == "10") client.publish("Heating/available", publishtopic1);
    if (Client_Number == "10") client.publish("Heating/Switch",publishtopic1); 
 
    if (Client_Number == "10") client.publish("Ventilation_Office/switch",publishtopic1 ,true);
    if (Client_Number == "10") client.publish("Ventilation_Sleep/switch",publishtopic1 ,true);
    if (Client_Number == "10") client.publish("Ventilation_Kids/switch",publishtopic1 ,true); 
             
    if (Client_Number == "10") client.publish("homeassistant/sensor/energy/day",daytopic ,true);
    if (Client_Number == "10") client.publish("homeassistant/sensor/energy/month",monthtopic ,true);
    if (Client_Number == "10") client.publish("homeassistant/sensor/energy/all",overalltopic ,true);

    if (Client_Number == "13") client.publish("Sprudler/switch/status",publishtopic1);     
    if (Client_Number == "13") client.publish("Sprudler/available", publishtopic1); 
    if (Client_Number == "13") client.publish("Sprudler/Amount/status", water_time_topic, true);  
   
    if (Client_Number == "14"){
      if (millis() >= 60000){
        client.publish("s14/available", publishtopic1); 
        client.publish("s14/mode/state", climate_mode_topic, true);
        client.publish("s14/temperature/state", climate_temp_topic, true);
       // if (climate_power == "OFF") client.publish("s14/power/set/status", publishtopic0, true);
        //if (climate_power == "ON") client.publish("s14/power/set/status", publishtopic1, true);
        client.publish("s14/fan/state", climate_fan_topic, true);
      }
    }
    

  yield();
  //Serial.print("MQTT-Published to"); Serial.print("\t");Serial.print(MQTT_SENSOR_TOPIC); Serial.print("\t");Serial.print(testst); Serial.print("\t");
  //MQTT_subscribe();


}


// void sendMQTTMsg(String data_topic ,  char unit){
//    String sensor_topic;
//    char buffer2[20];
//    String testst;
   
//    sensor_topic = MQTT_SENSOR_TOPIC; 
//    testst = sensor_topic + data_topic;
//    testst.toCharArray(buffer2, 20);   

// client.publish(buffer2, unit, true);
    

// }



// function called to publish the temperature and the humidity
void publishData2() {
 
  
  dtostrf(wind_gust_max, 10, 2, wind_gustmaxtopic);//convert float to char
  dtostrf(wind_gust, 10, 2, wind_gusttopic);//convert float to char
  dtostrf(wind_mittelwert, 10, 2, wind_topic);//convert float to char  
//  dtostrf(wind_mittelwert, 10, 2, wind_mediantopic);//convert float to char
 // dtostrf(wind_mittelwert2, 10, 2, wind_mediantopic2);//convert float to char
  dtostrf(analog_readings, 10,2, analog_readingstopic);
 
  
  //char data[270];
 // root.printTo(data, root.measureLength() + 1);
  //if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPICS0, data, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_gustmax,wind_gustmaxtopic, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_gust,wind_gusttopic, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_wind,wind_topic, true);
//    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_wind_median,wind_mediantopic, true);
//    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_wind_median2,wind_mediantopic2, true);
    if (Client_Number == "0") client.publish(MQTT_SENSOR_TOPIC_rain,analog_readingstopic, true);

    yield();
  Serial.print("MQTT-Published"); Serial.print("\t");
  //MQTT_subscribe();



}




void MQTT_subscribe(){
  client.subscribe("LED");
  
  if (Client_Number == "0"){                        //outdoor
    client.subscribe("Ventilation/Power");
  }

   if (Client_Number == "10"){                      //technique
    client.subscribe("Ventilation_Office/Power");
    client.publish("Ventilation_Office/switch/status",publishtopic1);
    client.publish("Ventilation_Office/switch",publishtopic1);    
    
    client.subscribe("Ventilation_Sleep/Power");
    client.publish("Ventilation_Sleep/switch/status",publishtopic1);
    client.publish("Ventilation_Sleep/switch",publishtopic1);    
    
    client.subscribe("Ventilation_Kids/Power");
    client.publish("Ventilation_Kids/switch/status",publishtopic1);
    client.publish("Ventilation_Kids/switch",publishtopic1);
      
    //client.subscribe("homeassistant/sensor/energy/state");
    client.subscribe("homeassistant/sensor/energy/all");
    //client.subscribe("homeassistant/sensor/energy/retain");
    client.subscribe("homeassistant/sensor/energy_day/state");
    client.subscribe("homeassistant/sensor/energy_month/state");
    //client.subscribe("homeassistant/sensor/energy/last_day");
    client.subscribe("homeassistant/sensor/energy/last_month");
    //client.subscribe("homeassistant/sensor/energy_pre");
    
    client.publish("Heating/Switch/status ",publishtopic1);    
    client.publish("Heating/Switch",publishtopic1);      
    client.subscribe("Heating/Power");    
   
  }
  
 if (Client_Number == "12"){                  //Display
    client.subscribe("homeassistant/sensor/temp3");
    client.subscribe("homeassistant/sensor/hum");
  } 
  
  
   if (Client_Number == "13"){                //Sprudler
    client.subscribe("Sprudler/switch");
    client.subscribe("Sprudler/Amount");
    client.subscribe("Sprudler/Counter");  
    client.publish("Sprudler/switch/status",publishtopic1);     
           
   }

 if (Client_Number == "14"){                   //AC_control
   //client.subscribe("s14/power/set");
   client.subscribe("s14/mode/set");
   client.subscribe("s14/temperature/set");
   client.subscribe("s14/fan/set");
   client.subscribe("s14/swing/set");
 
 }



}






void ac_control(){
if (Client_Number == "14"){
  
  //uint8_t climate_temp = 24;
  //uint8_t fan_speed = 1;


    // modes:
    //   - "off"
    //   - "cool"
    //   - "fan_only"
    //   - "auto"
    //   - "heat"
    //   - "dry"
    // swing_modes:
    //   - "on"
    //   - "off"
    // fan_modes:
    //   - "auto"
    //   - "high"
    //   - "medium"
    //   - "low"

  
    // power_command_topic: "s14/power/set" ----> ON OFF (not needed)
    // mode_command_topic: "s14/mode/set" ----> off  heat auto  cool  dry  fan_only
    // temperature_command_topic: "s14/temperature/set"   -----> 17  21  ...
    // fan_mode_command_topic: "s14/fan/set"    -----> high  medium  low
    // swing_mode_command_topic: "s14/swing/set"  ----->off  on
  
  // Set up what we want to send. See ir_Daikin.cpp for all the options.
  ac.on();
 //   ac.off();
  ac.setFan(fan_speed); // 0 = Auto; 1-3 = Fan Speed modes 1 = low; 2 = mid; 3 = high

// const uint8_t kMideaACCool = 0;     // 0b000
// const uint8_t kMideaACDry = 1;      // 0b001
// const uint8_t kMideaACAuto = 2;     // 0b010
// const uint8_t kMideaACHeat = 3;     // 0b011
// const uint8_t kMideaACFan = 4;      // 0b100

// const uint8_t kMideaACFanAuto = 0;  // 0b00
// const uint8_t kMideaACFanLow = 1;   // 0b01
// const uint8_t kMideaACFanMed = 2;   // 0b10
// const uint8_t kMideaACFanHigh = 3;  // 0b11  

  ac.setMode(kMideaACHeat);
 //kMideaACAuto
 //kMideaACCool
 //kMideaACHeat
 //kMideaACDry
 //kMideaACFan

  ac.setUseCelsius(true);
  ac.setTemp(climate_temp, true);
  //ac.setSwingVertical(false);
  //ac.setSwingHorizontal(false);

  
  // set Turn off timer in minutes)
  //ac.setOffTimer(30);

  // Display what we are going to send.
 // Serial.println(ac.toString());
 
// Reset the state of the remote to a known good state 
// Power On, Mode Auto, Fan Auto, Temp = 25C
 // ac.stateReset();
 
  // Now send the IR signal.
  ac.send();

delay(5000);

}

}





// function called when a MQTT message arrived
void callback(char* topic, byte* payload, unsigned int length) {
  
  char chars_added[200];
  char* part1 = topic;
  char* part2 = "/status";
  
  strcpy(chars_added, topic);
  strcat(chars_added, part2);  
  
  Serial.println();
  Serial.print("MQTT Message arrived! TOPIC: [");
  Serial.print(topic);
  
  Serial.print("]; ");
  //Serial.print(topic[4]);
  Serial.print("Payload: [");
  
  
 if (topic[0] =='L' ) ESP.restart();


  
  payload[length] = '\0'; // Add a NULL to the end of the char* to make it useable as string https://www.arduino.cc/reference/de/language/variables/data-types/string/
 
  int intPayload = atoi((char *)payload); //Returs '0' when not numeric
  double doublePayload = atof((char *)payload); //Returs '0' when not numeric
  String stringPayload = String((char *)payload);
  float test12 = 0;
  
if (Client_Number == "12"){             //Only Check
  if (topic[21] =='t' ){
    sht31_4_temp = doublePayload;
  }
  if (topic[21] =='h' ){
    sht31_4_humidity = doublePayload;
  }
  } 

  if (Client_Number == "13"){
    if (topic[9] =='s' ){ 
      if (intPayload == 1) start_water_override = 1;
      //client.publish("Sprudler/switch", publishtopic0, true);  
    }  
    //client.subscribe("Sprudler/switch");
   // client.subscribe("Sprudler/Amount ");
   //client.subscribe("Sprudler/Counter");   
     
  if (topic[9] =='A' ){    
    water_time = intPayload *1000;
    
    } 
if (topic[9] =='C' ){    
    water_counter = intPayload;
    }             
   }



if (Client_Number == "14"){               //Conrtol Air COnditioning (Only via IR LED connected to ESP8266)
    
   // client.publish("s14/available", publishtopic1); 
    
  if (topic[4] =='p' ){
//    climate_power = stringPayload;
//    climate_power.toCharArray(climate_power_topic, 10); //convert string to char
//    if (climate_power == "OFF") client.publish("s14/power/set/status", publishtopic0, true);
//    if (climate_power == "ON") client.publish("s14/power/set/status", publishtopic1, true);
//       if (stringPayload == "ON");
//       if (stringPayload == "OFF");
  }
 
  if (topic[4] =='m' ){                     //if climate Mode was changed
    if (stringPayload != climate_mode) { 
      //if (millis() >= 30000) {   
        if (stringPayload == "off") {
          ac.off();
          offset_restart = 0;
          //climate_power = "OFF";
          
        }
                
        if (stringPayload == "heat"){
          ac.on();
          ac.setMode(kMideaACHeat);
          ac.setFan(fan_speed);
          offset_restart = 36000000; //10 hours
          //climate_power = "ON";
          
        }
  
        if (stringPayload == "auto"){
          ac.on();
          ac.setMode(kMideaACAuto);
          ac.setFan(fan_speed);
          offset_restart = 21600000; //6hours
         // climate_power = "ON";
          
        }       
  
        if (stringPayload == "cool"){
          ac.on();
          ac.setMode(kMideaACCool);
          ac.setFan(fan_speed);
          offset_restart = 21600000;//6hours
          climate_power = "ON";
          
        }
  
        if (stringPayload == "dry"){
          ac.on();
          ac.setMode(kMideaACDry);
          ac.setFan(fan_speed);
          offset_restart = 21600000; //6hours
          //climate_power = "ON";
          
        }
  
        if (stringPayload == "fan_only"){
          ac.on();
          ac.setMode(kMideaACFan);
          ac.setFan(fan_speed);
          offset_restart = 21600000; //6hours
          //climate_power = "ON";
          
        }
  
          ac.setUseCelsius(true);
          ac.setTemp(climate_temp, true);
          
          climate_mode = stringPayload;
          ac.send(); 
      //} 
         // client.publish("s14/mode/state", (char*) stringPayload.c_str(), true);
         // if (climate_power == "OFF") client.publish("s14/power/set/status", publishtopic0, true);
         // if (climate_power == "ON") client.publish("s14/power/set/status", publishtopic1, true);   
    }
 
  }
 
  if (topic[4] =='t' ){                   //If AC Temp was changed
    if (intPayload != climate_temp) {   
      climate_temp = intPayload;
      if (climate_mode != "off"){
        //if (millis() >= 30000) {   
          ac.setUseCelsius(true);
          ac.setTemp(climate_temp, true);
          ac.setFan(fan_speed);
          ac.send(); 
         // client.publish("s14/temperature/state", (char*) stringPayload.c_str(), true);
        //}        
      }    
    }   
  }
  
  if (topic[4] =='f' ){                   //If Fan Speed was changed
  if (stringPayload != climate_fan){
    if (climate_mode != "off"){     
      if (stringPayload == "auto") {
        ac.setFan(0);
        fan_speed = 0;
      }
      if (stringPayload == "low") {
        ac.setFan(1);
        fan_speed = 1;
      }
      if (stringPayload == "medium") {
        ac.setFan(2);
        fan_speed = 2;
      }
      if (stringPayload == "high") {
        ac.setFan(3);
        fan_speed = 3;
      }
      ac.setUseCelsius(true);
      ac.setTemp(climate_temp, true);
      ac.send(); 
    }      
    climate_fan = stringPayload;      
    //client.publish("s14/fan/state", (char*) stringPayload.c_str(), true);
 }   
 }

  // if (topic[4] =='s' ){
  //   if (stringPayload != climate_swing){   
  //     if (stringPayload == "on") ac.stateReset();
  //     //if (stringPayload == "off") ac.stateReset();
      
  //     climate_swing = stringPayload;  
  //     ac.send();   
  //   }
  // }
}



if (Client_Number == "10"){               //Only techniqe
  
  
  if (topic[8] =='P' ){
    test12 = intPayload + 5 ;
    test12 = test12 / 10;
    intPayload = (int)test12;   //schneidet Nachkommastellen ab
    if (intPayload >= 7) intPayload = 7;     //Heizung hat nur 7 Stufen
    if (intPayload <= 0) intPayload = 0;     //Heizung hat nur 7 Stufen    
  }

if (topic[12] !='t' ){
    if (intPayload >= 9) intPayload = 9;   //Ventilation hat nur 9 Stufen
  }
  


  if (topic[12] =='O' ) {
    write_analog_output((intPayload*100)+25, 0);       //   Ziel: Büro & Dach           "Ventilation_Office/Power"  Wertebereich: 0 - 1024 --> 0-10V  
    vent_office = intPayload;
  }    
  if (topic[12] =='S' ) {
    write_analog_output((intPayload*100)+25, 1);   //   Ziel: Schlafzimmer & Küche  "Ventilation_Sleep/Power"   Wertebereich: 0 - 1024 --> 0-10V  
    vent_sleep = intPayload;  
}  
  if (topic[12] =='K' ) {
    write_analog_output((intPayload*100)+25, 2);   //   Ziel: Fabio & Lia           "Ventilation_Kids/Power"    Wertebereich: 0 - 1024 --> 0-10V  
    vent_kids = intPayload;    
  }    
  if (topic[8] =='P' )  {
  write_analog_output((intPayload*125)+50, 3);   //   Ziel: Heizstab              "Heating/Power"             Wertebereich: 0 - 1024 --> 0-10V   
  }  
  
}

  Serial.print(intPayload); Serial.print(" is Integer; "); Serial.print("] ");  Serial.print(test12); Serial.print(" and "); Serial.print(stringPayload); Serial.print(" is String; ");
  
  Serial.println();  
// Heizstab:
// Leistungsstufe	Von bis	Mittelwert
      // 0	      0,00	    0,50
      // 	        1,00	
      // 1	      1,25	    1,75
      // 	        2,25	
      // 2	      2,50	    3,00
      // 	        3,50	
      // 3	      3,75	    4,25
      // 	        4,75	
      // 4	      5,00	    5,50
      // 	        6,00	
      // 5	      6,25	    6,75
      // 	        7,25	
      // 6	      7,50	    8,00
      // 	        8,50	
      // 7	      8,75	    9,25
      // 	        9,75	


//if (Client_Number == "13") client.publish("Sprudler/switch", publishtopic0, true);  
  
  if (millis() <= 40000){
    if (topic[28] =='a' ){
      if (energy_received ==false) {
       energy_avg = energy_avg + doublePayload; //   Energiemessung
       energy_received = true;
       reset_energy = energy_avg;
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
  }else{
    if (topic[28] =='a' ){
      if (reset_energy <= doublePayload) {
       energy_avg = doublePayload; //   Energiemessung
       energy_month = (today * 12000) - (12000) + energy_day; 
       //4014kWh per year = 11kWh per Day
       //4380kWh per year = 12kWh per Day 
       //4745kWh per year = 13kWh per Day 
       //5110kWh per year = 14kWh per Day    
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
   Serial.println();
   Serial.print(topic); 
   Serial.println();     
// 
  
  //strcpy(chars_added + strlen(part1), part2);

 char char_payload[10];
  String string_to_convert;
  
  if (topic[8] =='P' ) intPayload = intPayload * 10; //Heizung 
   string_to_convert = String(intPayload);           //Heizung 
  string_to_convert.toCharArray(char_payload,10);    //Heizung 


  client.publish(chars_added,char_payload);
  //client.publish("Ventilation/switch/status";1)
   // client.publish("office/light/brightness",(char *)payload);
   Serial.println();
   Serial.print(topic); 
   Serial.println();        
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

void water_control(){  

  if (Client_Number == "13"){  
    if (millis() <= 20000){  
      client.publish("Sprudler/switch",publishtopic0, true); 
      }else{
         char Countertopic[14];
        float gesamtdauer = (water_time / 1000) + water_counter;
        float water_time_2 = water_time / 3;
        
        delay(250);   
       
        int time_now_water = millis();

        
        digitalWrite(D2, HIGH);
        delay(water_time_2); 
        client.loop();   
        digitalWrite(D2, HIGH);
        delay(water_time_2); 
        client.loop();   
        digitalWrite(D2, HIGH);
        delay(water_time_2);   
        client.loop();   
        digitalWrite(D2, LOW); 
        
        int time_now_water2 = millis() - time_now_water;
        Serial.println();
        Serial.print(time_now_water2);
        Serial.println();

        dtostrf(gesamtdauer, 10,2, Countertopic);
        client.publish("Sprudler/Counter",Countertopic, true);

        client.publish("Sprudler/switch",publishtopic0, true); 
        client.publish("Sprudler/available", publishtopic1); 
        client.publish("Sprudler/switch/status",publishtopic1, true);
        client.publish("Sprudler/Amount/status", water_time_topic, true); 

        attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING); //was RISING
      
      }  
  }
}  


void read_CO2_PWM() {
  long th, tl, periode, p1, p2;
  
  if (co2_pwm != 411){
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
  }

void read_periode2() {
  long tl;
  
     // th = pulseIn(pwm_pin, HIGH, 3000000);
      tl = pulseIn(pwm_pin, LOW, 3000000);
      
      // 26 ml/pulse --> 3.000.000µsec low time = 520ml/min
      // 26 ml/pulse -->    52.000µsec low time = 30.000ml/min
      if (tl > 0 ) {
        flow_rate =(1000000/tl)*26*60;  // unit: ml/min
      }
      if (tl == 0 ) flow_rate = 0;
      co2_pwm = flow_rate;
  }


void read_periode() {
  long tl, th, periode;
  
      th = pulseIn(pwm_pin, HIGH, 2000000);
      if (th != 0) {
        tl = pulseIn(pwm_pin, LOW, 2000000);
      }else{
        tl =0;
      }
      
      // 26 ml/pulse --> 3.000.000µsec low time = 520ml/min
      // 26 ml/pulse -->    52.000µsec low time = 30.000ml/min
      periode = tl + th;
      if (periode > 0 ) {
        flow_rate =(2000000/periode)*26*60;  // unit: ml/min
      }
      if (periode == 0 ) flow_rate = 0;
      co2_pwm = flow_rate;
  }

 void read_energy_PWM()
  {
  long th, tl, periode, p1, p2;

  //th = pulseIn(input_pin2, HIGH, 30000000); //1) wait for rising edge & start timer. 2) Wait for falling edge & stop timer.
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

    
    Pulse_Array[0]=1;
    Pulse_Array[1]=1;     //will create an error of 1µs but no 1/0 division
    float rps_avg = 0.001;

    detachInterrupt(digitalPinToInterrupt(input_pin1));
    
    float duration_sec_avg = (Pulse_Array[i_wind-1]);
    //Serial.print("i_wind-1: "); Serial.print(i_wind-1);Serial.print("\t"); Serial.print("Pulse_Array[i_wind-1]: "); Serial.print(Pulse_Array[i_wind-1]); Serial.print("\t");

    
    //Calculate average "Rotations per Second"
    if (i_wind >=3){
      
      long rps_avg1 = (duration_sec_avg / (i_wind-2) ); //computing average time per pulse (inµs)
          
      float rps_avg2 = rps_avg1 * 0.000002; //Convert from µs to seconds, and multiply by 2 ---> because there are 2 pulses per rotation. Same like X / 500000
      rps_avg = 1/ rps_avg2; //Kehrwert; conversion from "Seconds per Rotation" in "rotation per second"
      
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
    wind_gust_max = 1.761 / (1 + rps_max) + 3.013 * rps_max;  // found here: https://www.amazon.de/gp/customer-reviews/R3C68WVOLJ7ZTO/ref=cm_cr_getr_d_rvw_ttl?ie=UTF8&ASIN=B0018LBFG8 (in German)
    wind_gust = 1.761 / (1 + rps_avg) + 3.013 * rps_avg; // found here: https://www.amazon.de/gp/customer-reviews/R3C68WVOLJ7ZTO/ref=cm_cr_getr_d_rvw_ttl?ie=UTF8&ASIN=B0018LBFG8 (in German)
    }else{
      wind_gust_max = 0.01;
      wind_gust = 0.01;     // this function is called every 15 secs Wind_Gust is defined as "Median Value betwwen 3-20sec"
    }

    //Serial.print(Pulse_Array[0]); Serial.print("\t");Serial.print(Pulse_Array[1]);Serial.print("\t");Serial.print(Pulse_Array[2]);Serial.print("\t");Serial.print(Pulse_Array[3]);Serial.print("\t");Serial.print(Pulse_Array[4]);Serial.print("\t");Serial.print(Pulse_Array[5]);Serial.print("\t");Serial.print(Pulse_Array[6]);Serial.print("\t");Serial.print(Pulse_Array[7]);Serial.print("\t");Serial.print(Pulse_Array[8]);Serial.print("\t");Serial.print(Pulse_Array[9]);
    //Serial.println();


  //Clear Array
    for(int b=0; b<=i_wind+1; b++) Pulse_Array[b] = 0;
    i_wind2 = i_wind;
    i_wind = 1;
    Pulse_Array[0]=1;
    Pulse_Array[1]=1;
    //Serial.println();

    //Serial.print(Pulse_Array[0]); Serial.print("\t");Serial.print(Pulse_Array[1]);Serial.print("\t");Serial.print(Pulse_Array[2]);Serial.print("\t");Serial.print(Pulse_Array[3]);Serial.print("\t");Serial.print(Pulse_Array[4]);Serial.print("\t");Serial.print(Pulse_Array[5]);Serial.print("\t");Serial.print(Pulse_Array[6]);Serial.print("\t");Serial.print(Pulse_Array[7]);Serial.print("\t");Serial.print(Pulse_Array[8]);Serial.print("\t");Serial.print(Pulse_Array[9]);
    //Serial.println();


    dtostrf(wind_gust_max, 10, 2, wind_gustmaxtopic);//convert float to char
    dtostrf(wind_gust, 10, 2, wind_gusttopic);//convert float to char

    client.publish(MQTT_SENSOR_TOPIC_gustmax, wind_gustmaxtopic, true);
    client.publish(MQTT_SENSOR_TOPIC_gust, wind_gusttopic, true);
    
    published_data_counter();


    ArduinoOTA.handle();

    if (published_data >= 20 ){     //nach 20 mal "publishing wind" (~3,85sec) wird folgender befehl ausgeführt --> ~1,33min; 
      if (millis() >= 30000) publishData(temp_average, hum_average, atmospheric_pressure, Taupunkt_mittel, Absolute_Feuchte_mittel, lux, co2_pwm, wind_gust, wind_mittelwert);    // Publish Data to MQTT Broker for all sensors expect technique    
      
    }

    attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING); //was RISING
      
}


void wind_sum_up(){

  //detachInterrupt(digitalPinToInterrupt(input_pin1));
  //wind_collection++;
  
  
  
    // float ct5000 = 600000 / cycle_time5 ;  //max Number of runs = 10mins = 600sec = 600.000msec
    // ct5=(int)ct5000;
  
   
    //Numberofpoints = 240; //600 / ((factor/1000) * REPORT_INTERVAL); // always 10 minutes
    
   
  //  if (number_of_runs <= 240) {
  //    Numberofpoints = number_of_runs;
  //  }else{
  //    Numberofpoints = 240;
  //  }
    
    //wind_mittelwert = ((wind_mittelwert * Numberofpoints) + wind_gust) / (Numberofpoints +1);  // calculate median value of 10 minutes
  
  float ten_min_max_value = 600 / wind_factor;       //"wind_factor" Value should be = 3 ---> alle 3 sec wird eine Böe gemessen windgeschwindigkeit wird über 10min gemittelt
  int max_value = round(ten_min_max_value);
  
  number_of_runs++;  //do not reset to 0
  
  if (index_wind >= max_value) index_wind = 0;  //start again as soon as index wind is >=40
  
  wind_array[index_wind] = wind_gust;
  
  wind_mittelwert = 0;
  
  if (number_of_runs < max_value) {                                                                    //do this at startup until number_of_runs =40
    for(int c=0; c<number_of_runs; c++) wind_mittelwert = wind_mittelwert + wind_array[c] ;
    wind_mittelwert = wind_mittelwert / (number_of_runs);
   }else{                                                                                       //do this as soon as number_of_runs =40 or higher
    for(int c=0; c<max_value; c++) wind_mittelwert = wind_mittelwert + wind_array[c] ;           
    wind_mittelwert = wind_mittelwert / max_value;
   }
  
    
  index_wind++;
  
  
  dtostrf(wind_mittelwert, 10, 2, wind_topic);//convert float to char
  client.publish(MQTT_SENSOR_TOPIC_wind, wind_topic, true);

  //FiltVal= ((FiltVal * FF) + NewVal) / (FF +1);  
  
 
//attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING); //was rising

}   





  void published_data_counter(){
  published_data++;
  //Numberofpoints = 600 / ((factor/1000) * REPORT_INTERVAL);
  
//if (number_of_runs >= 240)  ESP.restart();
  
  if (published_data >= 21){     // all 10 min ---> 30sec * 20 = 600sec = 10min
 
    published_data = 0;
  }
 
  }

void stromzaehler() {
      
      Pulse_Array[0]=1;
      Pulse_Array[1]=1;
      
    if (i_wind >=3){ 
      
      detachInterrupt(digitalPinToInterrupt(input_pin1)); 
      
      i_wind2 = i_wind;
      //long test_start = micros();
      float duration_sec_avg = (Pulse_Array[i_wind-1]);
          
      //Clear Array
      for(int b=0; b<=i_wind+1; b++) Pulse_Array[b] = 0;
      //i_wind2 = i_wind;
      
      i_wind = 1;
         
      attachInterrupt(digitalPinToInterrupt(input_pin1),Interrupt,FALLING);
  
      Pulse_Array[0]=1;
      Pulse_Array[1]=1;
    
    
    
    //Serial.print("i_wind-1: "); Serial.print(i_wind-1);Serial.print("\t"); Serial.print("Pulse_Array[i_wind-1]: "); Serial.print(Pulse_Array[i_wind-1]); Serial.print("\t");

    
    //Calculate average "Power consumption"
    
      
      float average_lost_pulses_per_minute = 0.46; 
      wind3 = i_wind2 - 1 + average_lost_pulses_per_minute;
      
      //long rps_avg1 = (duration_sec_avg / (i_wind2-1) ); //computing average time per pulse (in µs)
      long rps_avg1 = (duration_sec_avg / wind3 ); //computing average time per pulse (in µs)
          
      float rps_avg2 = rps_avg1 * 0.000001; //Convert from µs to seconds
      float rps_avg = 1/ rps_avg2; //Kehrwert; conversionfrom "Seconds per Pulse" in "Pulses per second"
      power_avg = rps_avg * 360; //convert from "pulses per second" to "W" (10000 Pulses per kWh)
      
      //energy_avg = energy_avg + ((i_wind2-1) * 0.1); //convert from "pulses" to "Wh" (10 Pulses per Wh)
      energy_avg = energy_avg + (wind3 * 0.1); //convert from "pulses" to "Wh" (10 Pulses per Wh)
      
      //energy_day = energy_day + ((i_wind2-1) * 0.1);
      energy_day = energy_day + (wind3 * 0.1);
      
      //energy_month = energy_month + ((i_wind2-1) * 0.1);
      energy_month = energy_month + (wind3 * 0.1);
      
    // Serial.println(); Serial.print("rps_avg1: "); Serial.print(rps_avg1); Serial.print("\t");Serial.print("rps_avg2: "); Serial.print(rps_avg2,4); Serial.print("\t"); Serial.print("rps_avg2: "); Serial.print(rps_avg2,4);
    
    }else{
        //read_energy_PWM();
        
        if (power_avg <= 0.05) {
          power_avg = 0.1;
        }else{
          power_avg = 0.01;
      }
    }


   
   if (i_wind2 >=999) publishData(temp_average, hum_average, atmospheric_pressure, Taupunkt_mittel, Absolute_Feuchte_mittel, lux, co2_pwm, power_avg, power_avg); 
   


//long test_end = micros() - test_start;
//Serial.println(); Serial.print("dauer Stromzaehler[µs] "); Serial.print(test_end); Serial.print("\t");

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
