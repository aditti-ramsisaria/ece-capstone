
/***************************************************************************
  ENS160 - Digital Air Quality Sensor
  
  This is an example for ENS160 basic reading in standard mode
    
  Updated by Sciosense / 25-Nov-2021
 ***************************************************************************/
//#include <ArduinoJson.h>
#include <SoftwareSerial.h>                        
SoftwareSerial esp8266(10,11);                   
#define serialCommunicationSpeed 115200               
#define DEBUG true 

#include <Wire.h>
int ArduinoLED = 13;

#include "ScioSense_ENS160.h"  // ENS160 library
//ScioSense_ENS160      ens160(ENS160_I2CADDR_0);
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

#include "Multichannel_Gas_GMXXX.h"

// Settings
#define BTN_START           0                         // 1: press button to start, 0: loop
#define BTN_PIN             WIO_5S_PRESS              // Pin that button is connected to
#define SAMPLING_FREQ_HZ    10                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         8                         // 8 samples at 4 Hz is 2 seconds

// Global objects
GAS_GMXXX<TwoWire> gas;               // Multichannel gas sensor v2


/*--------------------------------------------------------------------------
  SETUP function
  initiate sensor
 --------------------------------------------------------------------------*/
void setup() {
  //StaticJsonDocument<200> doc;
  //Serial.begin(serialCommunicationSpeed);           
  //esp8266.begin(serialCommunicationSpeed);     
  //InitWifiModule(); 

  Serial.begin(115200);

  while (!Serial) {}
  unsigned status;
  bool label = true;
  gas.begin(Wire, 0x08);
  status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    //Serial.println("-- Default Test --");

  //Switch on LED for init
  pinMode(ArduinoLED, OUTPUT);
  digitalWrite(ArduinoLED, LOW);

  /*Serial.println("------------------------------------------------------------");
  Serial.println("ENS160 - Digital air quality sensor");
  Serial.println();
  Serial.println("Sensor readout in standard mode");
  Serial.println();
  Serial.println("------------------------------------------------------------");
  delay(1000);*/

  //Serial.print("ENS160...");
  ens160.begin();
  if (ens160.available()) {
    ens160.setMode(ENS160_OPMODE_STD);
    // Print ENS160 versions
    //Serial.print("\tRev:  "); Serial.print(ens160.getMajorRev());
    //Serial.print("."); Serial.print(ens160.getMinorRev());
    //Serial.print("."); Serial.println(ens160.getBuild());
  
    //Serial.print("\tStandard mode ");
    //Serial.println( ? "done." : "failed!");
  }

  while(label) {
    Serial.print("grove_voc1");
    Serial.print(",");
    Serial.print("grove_no2");
    Serial.print(",");
    Serial.print("grove_eth");
    Serial.print(",");
    Serial.print("grove_co");
    Serial.print(",");
    Serial.print("ens_tvoc");
    Serial.print(",");
    Serial.print("ens_co2");
    Serial.print(",");
    Serial.print("temperature");
    Serial.print(",");
    Serial.print("pressure");
    Serial.print(",");
    Serial.print("altitude");
    Serial.print(",");
    Serial.print("humidity");
    Serial.print(",");
    Serial.println("label");
    label=false;
  }

}

/*--------------------------------------------------------------------------
  MAIN LOOP FUNCTION
  Cylce every 1000ms and perform measurement
 --------------------------------------------------------------------------*/

void loop() {
  float gm_no2_v;
  float gm_eth_v;
  float gm_voc_v;
  float gm_co_v;
  unsigned long timestamp;
  float ens_TVOC;
  float ens_CO2;
  float bme_temp;
  float bme_pressure;
  float bme_altitude;
  float bme_humidity;

  //Serial.println("timestamp,voc1,no2,eth,co");

  // Transmit samples over serial port
  for (int i = 0; i < NUM_SAMPLES; i++) {

    // Take timestamp so we can hit our target frequency
    timestamp = millis();

    // Read from GM-X02b sensors (multichannel gas)
    gm_no2_v = gas.calcVol(gas.getGM102B());
    gm_eth_v = gas.calcVol(gas.getGM302B());
    gm_voc_v = gas.calcVol(gas.getGM502B());
    gm_co_v = gas.calcVol(gas.getGM702B());

    //Read from ENS-160
    if (ens160.available()) {
      ens160.measure(0);
      ens_TVOC = ens160.getTVOC();
      ens_CO2 = ens160.geteCO2();
    }

    //Read from BME-280
    bme_temp = bme.readTemperature();
    bme_pressure = bme.readPressure();
    bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    bme_humidity = bme.readHumidity();
    
    Serial.print(gm_voc_v);
    Serial.print(",");
    Serial.print(gm_no2_v);
    Serial.print(",");
    Serial.print(gm_eth_v);
    Serial.print(",");
    Serial.print(gm_co_v);
    Serial.print(",");
    Serial.print(ens_TVOC);
    Serial.print(",");
    Serial.print(ens_CO2);
    Serial.print(",");
    Serial.print(bme_temp);
    Serial.print(",");
    Serial.print(bme_pressure);
    Serial.print(",");
    Serial.print(bme_altitude);
    Serial.print(",");
    Serial.print(bme_humidity);
    Serial.print(",");
    Serial.println("ambient");
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }
  delay(1000);
 }

/*
  if(esp8266.available())                                           
 {    
    if(esp8266.find("+IPD,"))
    {
     delay(1000);
 
     int connectionId = esp8266.read()-48;                                                
     String webpage = "<h1>Hello World!</h1>";
     String cipSend = "AT+CIPSEND=";
     cipSend += connectionId;
     cipSend += ",";
     cipSend +=webpage.length();
     cipSend +="\r\n";
     
     sendData(cipSend,1000,DEBUG);
     sendData(webpage,1000,DEBUG);
 
     String closeCommand = "AT+CIPCLOSE="; 
     closeCommand+=connectionId; // append connection id
     closeCommand+="\r\n";    
     sendData(closeCommand,3000,DEBUG);
    }
  }
}

String sendData(String command, const int timeout, boolean debug)
{
    String response = "";                                             
    esp8266.print(command);                                          
    long int time = millis();                                      
    while( (time+timeout) > millis())                                 
    {      
      while(esp8266.available())                                      
      {
        char c = esp8266.read();                                     
        response+=c;                                                  
      }  
    }    
    if(debug)                                                        
    {
      Serial.print(response);
    }    
    return response;                                                  
}

void InitWifiModule()
{
  sendData("AT+RST\r\n", 2000, DEBUG);                                                  
  sendData("AT+CWJAP=\"CMU-DEVICE\",\"\"\r\n", 2000, DEBUG);        
  delay (5000);
  sendData("AT+CWMODE=1\r\n", 1500, DEBUG);                                             
  delay (1500);
  sendData("AT+CIFSR\r\n", 1500, DEBUG);                                             
  delay (1500);
  sendData("AT+CIPMUX=1\r\n", 1500, DEBUG);                                             
  delay (1500);
  sendData("AT+CIPSERVER=1,80\r\n", 1500, DEBUG);                                     

}*/
