//Included Libraries
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "WiFiAutoSelector.h" //https://gist.github.com/AndiSHFR/e9c46890af7cddff6cb5ea7d4f1c5c49
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_MCP9808.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "mhz19.h"
#include "SoftwareSerial.h"


// Define the Numer of this Client, every Client needs its own unique number, 
  #define Client_Number "0"

  // 0 = Ouside Sensor 
  // 1 = Inside Sensor 1 
  // 2 = Inside Sensor 2
  // 3 = Inside Sensor 3


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
  #define WU_API_KEY "*****"             // Create Account to download Data from Wunderground.com: https://www.wunderground.com/weather/api/d/pricing.html
  #define WUNDERGROUND "api.wunderground.com"       // DOcumentation of API Calls: https://www.wunderground.com/weather/api/d/docs
  #define WU_LOCATION "pws:*****"               // Station ID you want to download data from

// Wunderground (Upload Data to your PWS at Wundergound)
  #define WUNDERGROUND2 "rtupdate.wunderground.com" //Create your own Weatherstation to upload Data to Wunderground.com: https://www.wunderground.com/personal-weather-station/signup.asp
  #define WUNDERGROUND_STATION_ID "*****"
  #define WUNDERGROUND_STATION_PASSWORD "*****"

// PushingBox scenario DeviceId code and API
  String deviceId = "*****";             // Anleitung Pushingbox: http://www.geekstips.com/android-push-notifications-esp8266-arduino-tutorial/
  const char* logServer = "api.pushingbox.com";

// MQTT: ID, server IP, port, username and password
  char* MQTT_CLIENT_ID = "office";
  char* MQTT_SERVER_IP = "*****"; //Hass.io ip in local network
  uint16_t MQTT_SERVER_PORT = 1883;
  char* MQTT_USER = "*****";
  char* MQTT_PASSWORD = "*****";

// MQTT: topic
  char* MQTT_SENSOR_TOPICS0 = "outdoor/sensor1";
  char* MQTT_SENSOR_TOPICS1 = "indoor/sensor1";
  char* MQTT_SENSOR_TOPICS2 = "indoor/sensor2";
  char* MQTT_SENSOR_TOPICS3 = "indoor/sensor3";

// MQTT Version   
  #define MQTT_VERSION MQTT_VERSION_3_1_1

//for wifi auto selector
  #define WIFI_CONNECT_TIMEOUT 8000 
  WiFiAutoSelector wifiAutoSelector(WIFI_CONNECT_TIMEOUT);  

//Definition of your Node MCU pins where your sensors are connected
  #define PIN_RX  D1  //Connect to MH-Z19B TX Pin - Communication with MH-Z19B  
  #define PIN_TX  D2  //Connect to MH-Z19B RX Pin - Communication with MH-Z19B 
  #define DHTPIN D4  // this Connects DHT22 to NodeMCU Pin D4
  #define SCL_PIN D5  //defines the i2C Clock Pin on D5
  #define SDA_PIN D6  //defines the i2C Data Pin on D6
  #define ONE_WIRE_PIN D7 // this Connects MAXIM DS18B20 to NodeMCU Pin D7

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


// this defines the Type of DHTxx Sensor
  #define DHTTYPE DHT22 
  const int temperaturePrecision = 12;// resolution for all DS18B20 devices (9, 10, 11 or 12 bits)

//initialize an instance of the each class
  WiFiClient  wificlient; 
  WiFiClient  client2;
  OneWire oneWire(ONE_WIRE_PIN);
  DallasTemperature DS18B20(&oneWire); // pass oneWire reference to DallasTemperature
  DallasTemperature sensors(&oneWire);
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
  Adafruit_BMP280 bme;
  DHT dht(DHTPIN, DHTTYPE);
  Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, "0.de.pool.ntp.org", 3600, 60000); // Die letzten 2 Werte: 1) Zeitversatz der aktuellen Zeitzone in sekunden; 2) Updateintervall in Sekunden 
  PubSubClient client(wificlient);
  SoftwareSerial mhz19b(PIN_RX, PIN_TX); //Communication with MH-Z19B
 
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
  float temp_average=0;
  float hum_average=0;
  float Taupunkt_mittel = 0;
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
  bool mcp = true;
  int ausfaelle = 0;
  int co2 = 1; 
  int temp = 1;



void setup() {
  
  //Start der Seriellen Datenübertragung zum PC  
    Serial.begin(115200);
    Serial.println();
  
  //Auflisten der verfügbaren WLAN Netzwerke
   listNetworks();

  //Bibliotheken starten
    Wire.begin(SDA_PIN, SCL_PIN);  //I2C Bus
    dht.begin();        //Asong2302 DHT22
    sht31.begin(0x44); //Sensirion SHT31
    bme.begin(0x77);  //Bosch BMP280
    DS18B20.begin(); // DS18B20
    DS18B20.setResolution(temperaturePrecision);
    mcp9808.begin();  //MCP9808 0x18
    if(mcp9808.begin()==false) mcp9808_temp = -127; 
    mhz19b.begin(9600);   //Communication with MH-Z19B 
    timeClient.begin();

  //Angabe der Netzwerkdaten
    wifiAutoSelector.add(ssid1, password1);
    wifiAutoSelector.add(ssid2, password2);
    wifiAutoSelector.add(ssid3, password3);

  
  //Scan i2C Bus for Devices
    scan_i2c();  
  
  //Scan for Maxim 1-Wire Bus Devices
    lookUp_for_DS18B20_Sensors();
  
  //Connect to Wifi
    wifiConnect();

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

    Serial.println();
    Serial.println("MCP9808\tDS18B20\tDHT22\tSHT31\tBMP280\tAverage\tDHT22\tSHT31\tAverage\tTaupunkt\tBMP280\tBMP280\tBMP280\tCycletime\tBodentemp\tMQTT\tTime");
    
}   // End of Void Setup() !!!






void loop() {
  
  
  //Check if WIFI Connection is alive
    if(WiFi.status() != WL_CONNECTED) {
      ESP.restart();                //restart ESP8266
    }

  //Connect to MQTT Service and check in each loop if its connected
    if (!client.connected()) {
      MQTT_connect();
    }
    client.loop();
  
  //declaration of variables and reading out temperatures and humidity from all sensors
    DS18B20.requestTemperatures();
    maximtemp1 = DS18B20.getTempCByIndex(0);
    maximtemp2 = DS18B20.getTempCByIndex(1);
    dht_temp = dht.readTemperature();
    sht31_temp = sht31.readTemperature();
    if (mcp == true && mcp9808_temp >= -100) mcp9808_temp = mcp9808.readTempC();
    float dht_hum = dht.readHumidity();
    float sht31_humidity = sht31.readHumidity();
    float bmp_pressure = bme.readPressure()/100; //aktueller Luftdruck in mbar
    float bmp_hoehe = bme.readAltitude(1017.00);
    float bmp_temp = bme.readTemperature();
      
      

  //Get current Time from NTP Server
    timeClient.update();
    time_t utcCalc = timeClient.getEpochTime();
    int summer_offset = 0;  //offset in Wintetime
    if (summertime(year(utcCalc), month(utcCalc), day(utcCalc), hour(utcCalc), 1)) summer_offset=3600; //create offset in summertime
    timeClient.setTimeOffset(3600 + summer_offset);
    //timeClient.update();
    utcCalc = timeClient.getEpochTime();
    
   //Reset durchführen einmal pro Tag
   if (hour(utcCalc) == 1){         //Wenn es ein uhr nachts ist
    if (millis() >= 4000000){       //Wenn die Laufzeit des Geräts größer ist als 66,66 Minuten (4.000.000 Milliseconds)
      ESP.restart();                //restart ESP8266
      }
   }

   
    
      
  // loop through each Maxim DS18B20 device, print out temperature
    /*
    for(int i=0; i<temperatureNumDevices; i++) {
      // print temperature
      float temperatureC = temperatureSensors.getTempC(deviceAddressArray[i]);
      dtostrf(temperatureC, 5, 1, temperatureArrayChar[i]);  // write to char array
    }*/

  //Bestimmung der Druchschnittstemperatur aller Temperatursensoren
       
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
   ausfaelle = 0;
    
  //Bestimmung der Druchschnittsfeuchte aller Luftfeuchutgkeutssensoren
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
    if(maxh == hum_average)maxh_time = timeClient.getFormattedTime();
    if(mint == temp_average)mint_time = timeClient.getFormattedTime();
    if(minsoilt == maximtemp2)minsoilt_time = timeClient.getFormattedTime();

    //abends
    if(maxt == temp_average)maxt_time = timeClient.getFormattedTime();
    if(minh == hum_average)minh_time = timeClient.getFormattedTime();

  //Jeden morgen die Min Werte übertragen
     if (hour(utcCalc) == 7){         //Wenn es 7 uhr morgens ist
      if (Client_Number == "0"){ 
        if (morgenwerte_uebermittelt == false){
          //AIO_Min_Max_Temp_outdoor_1.publish(mint);
          delay(2000);
          //AIO_Min_Max_hum_outdoor_1.publish(maxh);
          delay(2000);
          //AIO_Min_Soil_Temp_outdoor_1.publish(minsoilt);
          delay(2000);
//            String Morgentemp = String(mint);
//            String Morgenfeuchte = String(maxh);
//            String morgentemp_time = String(mint_time);
//            String morgenfeuchte_time = String(maxh_time);
          String Morgenwerte = "Die Mintemp betrug " +String(mint) +"°C, um " +String(mint_time) +"Uhr; Die Mintemp am Boden betrug " +String(minsoilt) +"°C, um " +String(minsoilt_time) +"Uhr. Die Maxfeuchte betrug " +String(maxh) +"% um " +String(maxh_time) +"Uhr";
          //sendNotification(Morgenwerte);
          morgenwerte_uebermittelt = true;
        }
      }
    }

  //jeden Abend die max Werte übertragen
     if (hour(utcCalc) == 18){         //Wenn es 18 uhr abends ist
      if (Client_Number == "0"){ 
        if (abendwerte_uebermittelt == false){
          //AIO_Min_Max_Temp_outdoor_1.publish(maxt);
          delay(2000);
          //AIO_Min_Max_hum_outdoor_1.publish(minh);
          delay(2000);
          String Abendtemp = String(maxt);
          String Abendfeuchte = String(minh);
          String Abendtemp_time = String(maxt_time);
          String Abendfeuchte_time = String(minh_time);
          String Abendwerte = "Maxtemp: " +Abendtemp +"°C, um " +Abendtemp_time +"Uhr; Minfeuchte: " +Abendfeuchte +"% um " +Abendfeuchte_time +"Uhr";
          //sendNotification(Abendwerte);
          abendwerte_uebermittelt = true;
          
        }
      }
    }
  

  //Bestimmung des Taupunkts
    Taupunkt_mittel = Taupunkt_berechnen(temp_average, hum_average);

// Bestimmung des CO2 Werts
    if (Client_Number != "0"){ 
    read_temp_co2();
    }
  
  //Ermittlung des aktuellen Internet Verbindungsstatus
    //CurrentWifiStatus();

  //Bestimmung des Atmosphärischen Drucks bezogen auf Meereshöhe.
    float Sea_level = 439.0;  //Meereshöhe + Plattformhöhe des Spielturms: 437.7m + 1,5m = 439.2m
    float T = temp_average ;  //Außentemperatur nötig zur Berechnung des korrekten Drucks bezogen auf Meereshöhe
    atmospheric_pressure = (bmp_pressure * exp (Sea_level / (29.3 * (T + 273.15)))) ;

   
  //Bestimmen der Programmlaufzeit
    time_new = millis();
    float cycle_time = time_new - time_old;
    time_old = millis();

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
     
    Serial.print(bmp_pressure); Serial.print("hpa"); Serial.print("\t");
    Serial.print(atmospheric_pressure);Serial.print("hpa"); Serial.print("\t");
    Serial.print(bmp_hoehe); Serial.print("m"); Serial.print("\t");
    Serial.print(cycle_time); Serial.print("ms"); Serial.print("\t");
    
    Serial.print(maximtemp2); Serial.print("°C"); Serial.print("\t");
    Serial.print(co2); Serial.print("ppm"); Serial.print("\t");
    Serial.print(WiFi.status()); Serial.print(" "); Serial.print("\t");
    //    Serial.print(maxt); Serial.print("°C"); Serial.print("\t");
    //    Serial.print(minh); Serial.print("%"); Serial.print("\t");
    //    Serial.print(maxh); Serial.print("%"); Serial.print("\t");
    //    Serial.print(mint_time); Serial.print(""); Serial.print("\t");
    //    Serial.print(maxt_time); Serial.print(""); Serial.print("\t");
    //    Serial.print(minh_time); Serial.print(""); Serial.print("\t");
    //    Serial.print(maxh_time); Serial.print(""); Serial.print("\t");

//long too_long = ((REPORT_INTERVAL * 1000) + 10000);
//if (cycle_time >= too_long){
//  ESP.restart();
//}

  // Publish Data to MQTT Broker
    publishData(temp_average, hum_average, atmospheric_pressure, Taupunkt_mittel, co2);
      

// Post measurement Data to Wunderground Station
    if (Client_Number == "0"){ 

      //Diese Befehle alle 3 minuten durchführen
      time_new4 = millis();
      float cycle_time3 = time_new4 - time_old4;
      if (cycle_time3 >= 3 * 60000){ 
          
          Wunderground_post_data();
        //Wunderground Stationsvergleich
        //Wunderground_read_station();
        
        time_old4 = time_new4;
      }
    }

  
  
//Diese Befehle alle 6 Stunden durchführen
    time_new3 = millis();
    float cycle_time2 = time_new3 - time_old3;
    if (cycle_time2 >= 360 * 60000){ 
     
      //Pushingbox soll den aktuellen Status erhalten
      current_status();
 
      time_old3 = time_new3;
    }
  
    
    //Serial.print(cycle_time); Serial.print("ms"); Serial.print("\t");
    Serial.println(timeClient.getFormattedTime());


  //abwarten des angegebenen Updateintervalls
    int cnt = REPORT_INTERVAL; //- (cycle_time - REPORT_INTERVAL);
    while(cnt--) delay(1000);
  
   
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
    if(-1 < wifiAutoSelector.scanAndConnect()) {
      int connectedIndex = wifiAutoSelector.getConnectedIndex();
      Serial.print(" '");
      Serial.print(wifiAutoSelector.getSSID(connectedIndex));
      Serial.println("'. Done.");
    }else{
      Serial.println("failed.");
    }
  }
 
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

  //taupunkt berechnen
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
    absolute_Feuchte = pow(10,5 * molekulargewicht_wasserdampf/universelle_Gaskonstante * Dampfdruck/(Temp2+273.15));           //(r,TK)
    //absolute_Feuchte = 10^5 * mw/R* * SDD(TD)/TK            //(TD,TK)

//vergleich
 aTp = 17.271;         // fuer Formel Taupunkt
bTp = 237.7;          // fuer Formel Taupunkt
taupunktTmp = ((aTp * Temp2) / (bTp + Temp2)) + log(feuchte / 100);
Taupunkt_alt = (bTp * taupunktTmp) / (aTp - taupunktTmp);
return Taupunkt ; 
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


void MQTT_connect() {
  // Loop until we're connected
  Serial.print("Attempting MQTT connection...");
  if (!client.connected()) {
     // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("MQTT connected");
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
        ESP.restart();                //restart ESP8266
        
      }
    }
  }
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
  String current_time = timeClient.getFormattedTime();
  
  
  String Statuswerte = "Es erfolgte ein Restart um " +current_time 
  +"Uhr mit dem Netzwerk " +current_SSID 
  + ".  \r\nEmpfangsqualität: " +rssi 
  +"db.  \r\nAktuell " +ausfaelle 
  + " Sensorausfälle.  \r\nAvg. Temp: " +temp_average 
  +"°C.  \r\nMCP9808: " +mcp9808_temp 
  +"°C.  \r\nDS18b20: " +maximtemp1  
  +"°C.  \r\nDHT21: "+dht_temp  
  + "°C.  \r\nSHT31: " +sht31_temp 
  +"°C.  \r\nBodentemp: " +maximtemp2  
  +"°C.  \r\nAvg. Feuchte: " +hum_average 
  +"%.  \r\nLuftdruck: "+atmospheric_pressure  
  +"mbar.  \r\nTaupunkt: "   +Taupunkt_mittel  +"°C." ; 
  
  //sendNotification(Statuswerte);
  
}

void current_status(){
  long rssi = WiFi.RSSI();
  String current_SSID = WiFi.SSID();
  String current_time = timeClient.getFormattedTime();
  
  
  String Statuswerte = "Statusreport um " +current_time 
  +"Uhr mit dem Netzwerk " +current_SSID 
  + ".  \r\nEmpfangsqualität: " +rssi 
  +"db.  \r\nAktuell " +ausfaelle 
  + " Sensorausfälle.  \r\nAvg. Temp: " +temp_average 
  +"°C.  \r\nMCP9808: " +mcp9808_temp 
  +"°C.  \r\nDS18b20: " +maximtemp1  
  +"°C.  \r\nDHT21: "+dht_temp  
  + "°C.  \r\nSHT31: " +sht31_temp 
  +"°C.  \r\nBodentemp: " +maximtemp2  
  +"°C.  \r\nAvg. Feuchte: " +hum_average 
  +"%.  \r\nLuftdruck: "+atmospheric_pressure  
  +"mbar.  \r\nTaupunkt: "   +Taupunkt_mittel  +"°C." ; 
  
  //sendNotification(Statuswerte);
  
}

// function called to publish the temperature and the humidity
void publishData(float p_temperature, float p_humidity, float p_pressure, float p_taupunkt, int p_co2) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  
  // Get the root object in the document
  JsonObject& root = jsonBuffer.createObject();
  
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = (String)p_temperature;
  root["humidity"] = (String)p_humidity;
  root["pressure"] = (String)p_pressure;
  root["taupunkt"] = (String)p_taupunkt;
  root["CO2"] = (String)p_co2;
  
  //root.prettyPrintTo(Serial);
  //Serial.println("");
  /*
     {
        "temperature": "23.20" ,
        "humidity": "43.70"
     }
  */
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  if (Client_Number == "0"){ 
    client.publish(MQTT_SENSOR_TOPICS0, data, true);
  }
  if (Client_Number == "1"){ 
    client.publish(MQTT_SENSOR_TOPICS1, data, true);    
    }
  if (Client_Number == "2"){ 
    client.publish(MQTT_SENSOR_TOPICS2, data, true);    
    }
  if (Client_Number == "3"){ 
    client.publish(MQTT_SENSOR_TOPICS0, data, true);   
    }
  yield();
  Serial.print("MQTT-Published"); Serial.print("\t");
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
}

static bool exchange_command(uint8_t cmd, uint8_t data[], unsigned int timeout)
{
    // create command buffer
    uint8_t buf[9];
    int len = prepare_tx(cmd, data, buf, sizeof(buf));

    // send the command
    mhz19b.write(buf, len);

    // wait for response
    unsigned long start = millis();
    while ((millis() - start) < timeout) {
        if (mhz19b.available() > 0) {
            uint8_t b = mhz19b.read();
            if (process_rx(b, cmd, data)) {
                return true;
            }
        }
        yield();
    }

    return false;
}

void read_temp_co2()
{
    uint8_t data[] = {0, 0, 0, 0, 0, 0};
    bool result = exchange_command(0x86, data, 3000);
    if (result) {
        co2 = (data[0] << 8) + data[1];
        temp = data[2] - 40;
    }
//#if 1
//        char raw[32];
//        sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4], data[5]);
//        Serial.println(raw);
//#endif
//    }
//    return result;
}
