#include <math.h>
#include <SoftwareSerial.h>                      
#include <Wire.h>
#include "Multichannel_Gas_GMXXX.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ScioSense_ENS160.h"
#include "MultiClassification_linear.h"
#include <LiquidCrystal_I2C.h>

// FOR SENSORS

#define serialCommunicationSpeed 115200               
#define SEALEVELPRESSURE_HPA (1013.25)
#define SAMPLING_FREQ_HZ    10                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
unsigned long prevSensorMillis;

// Global objects
GAS_GMXXX<TwoWire> gas;               // Multichannel gas sensor v2

// Initialize BME
Adafruit_BME280 bme; 

// initialize ens
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

// initialize lcd
LiquidCrystal_I2C lcd(0x3F, 16, 2); 


Eloquent::ML::Port::SVM classifier;
float X[35];

struct SensorReading
{
    float gm_no2_v = 0.0;
    float gm_eth_v = 0.0;
    float gm_voc_v = 0.0;
    float gm_co_v = 0.0;
    unsigned long timestamp = 0;
    float ens_TVOC = 0.0;
    float ens_CO2 = 0.0;
    float bme_temp = 0.0;
    float bme_pressure = 0.0;
    float bme_altitude = 0.0;
    float bme_humidity = 0.0;
};

float gm_voc[SAMPLING_FREQ_HZ];
float gm_no2[SAMPLING_FREQ_HZ];
float gm_eth[SAMPLING_FREQ_HZ];
float gm_co[SAMPLING_FREQ_HZ];
float ens_tvoc[SAMPLING_FREQ_HZ];
float ens_co2[SAMPLING_FREQ_HZ];
float bme_temp[SAMPLING_FREQ_HZ];


SensorReading sensor_reading;
SensorReading sensor_readings_per_second[SAMPLING_FREQ_HZ];
int sample_count = 0; // goes from 0 to SAMPLING_FREQ_HZ - 1


// sensor setup
void sensorSetup() {
    unsigned status;

    // grove initialization
    gas.begin(Wire, 0x08);

    // ens160 initialization
    ens160.begin();
    if (ens160.available()) {
        ens160.setMode(ENS160_OPMODE_STD);
    }

    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); 
        Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
    }

}

// main setup
void setup() {
    lcd.begin(); //Defining 16 columns and 2 rows of lcd display
    lcd.clear();
    lcd.backlight(); //To Power ON the back light
    Serial.begin(115200);
    sensorSetup();
    Serial.println("starting now");
}

// main loop
void loop() {
    readSensors();
}

// compute mean of array
float getMean(float* val, int arrayCount) {
  long total = 0;
  for (int i = 0; i < arrayCount; i++) {
    total = total + val[i];
  }
  float avg = total / (float)arrayCount;
  return avg;
}

// compute standard deviation of array
float getStdDev(float* val, int arrayCount) {
    float avg = getMean(val, arrayCount);
    long total = 0;
    for (int i = 0; i < arrayCount; i++) {
        total = total + (val[i] - avg) * (val[i] - avg);
    }

    float variance = total / (float)arrayCount;
    float stdDev = sqrt(variance);
    return stdDev;
}

// return min of array of floats
float getMin(float* val, int arrayCount) {
    float min = val[0];
    for (int i = 0; i < arrayCount; i++) {
        if (val[i] < min) {
            min = val[i];
        }
    }
    return min;
}

// return max of array of floats
float getMax(float* val, int arrayCount) {
    float max = val[0];
    for (int i = 0; i < arrayCount; i++) {
        if (val[i] > max) {
            max = val[i];
        }
    }
    return max;
}

// return rms of array of floats
float getRms(float* val, int arrayCount) {
    float total_square = 0.0;
    for (int i = 0; i < arrayCount; i++) {
        total_square = total_square + val[i] * val[i];
    }
    float rms = total_square / (float) arrayCount;
    return sqrt(rms);
}

// read from sensor every SAMPLING_PERIOD_MS
void readSensors() {
    // Take timestamp so we can hit our target frequency
    sensor_reading.timestamp = millis();

    if (sensor_reading.timestamp > prevSensorMillis + SAMPLING_PERIOD_MS) {
        
        // Read from GM-X02b sensors (multichannel gas)
        gm_eth[sample_count] = gas.calcVol(gas.getGM302B());
        gm_voc[sample_count] = gas.calcVol(gas.getGM502B());
        gm_no2[sample_count] = gas.calcVol(gas.getGM102B());
        gm_co[sample_count] = gas.calcVol(gas.getGM702B());

        sensor_reading.gm_eth_v = gas.calcVol(gas.getGM302B());
        sensor_reading.gm_voc_v = gas.calcVol(gas.getGM502B());
        sensor_reading.gm_no2_v = gas.calcVol(gas.getGM102B());
        sensor_reading.gm_co_v = gas.calcVol(gas.getGM702B());

        // Read from ENS-160
        if (ens160.available()) {
            ens160.measure(0);
            ens_tvoc[sample_count] = ens160.getTVOC();
            ens_co2[sample_count] = ens160.geteCO2();
            sensor_reading.ens_TVOC = ens160.getTVOC();
            sensor_reading.ens_CO2 = ens160.geteCO2();
        }

        // Read from BME-280
        bme_temp[sample_count] = bme.readTemperature();
        sensor_reading.bme_temp = bme.readTemperature();
        
//        Serial.print(sensor_reading.gm_voc_v);
//        Serial.print(",");
//        Serial.print(sensor_reading.gm_no2_v);
//        Serial.print(",");
//        Serial.print(sensor_reading.gm_eth_v);
//        Serial.print(",");
//        Serial.print(sensor_reading.gm_co_v);
//        Serial.print(",");
//        Serial.print(sensor_reading.ens_TVOC);
//        Serial.print(",");
//        Serial.print(sensor_reading.ens_CO2);
//        Serial.print(",");
//        Serial.print(sensor_reading.bme_temp);
//        Serial.print(",");
//        Serial.print(sensor_reading.bme_pressure);
//        Serial.print(",");
//        Serial.print(sensor_reading.bme_altitude);
//        Serial.print(",");
//        Serial.print(sensor_reading.bme_humidity);
//        Serial.print(",");

        if (sample_count == (SAMPLING_FREQ_HZ - 1)) {
            // calculate min, max, std, rms, avg for each
            // grove voc
            X[0] = getMin(gm_voc, SAMPLING_FREQ_HZ);
            X[1] = getMax(gm_voc, SAMPLING_FREQ_HZ);
            X[2] = getStdDev(gm_voc, SAMPLING_FREQ_HZ);
            X[3] = getRms(gm_voc, SAMPLING_FREQ_HZ);
            X[4] = getMean(gm_voc, SAMPLING_FREQ_HZ);
            // grove eth
            X[5] = getMin(gm_eth, SAMPLING_FREQ_HZ);
            X[6] = getMax(gm_eth, SAMPLING_FREQ_HZ);
            X[7] = getStdDev(gm_eth, SAMPLING_FREQ_HZ);
            X[8] = getRms(gm_eth, SAMPLING_FREQ_HZ);
            X[9] = getMean(gm_eth, SAMPLING_FREQ_HZ);
            // grove no2
            X[10] = getMin(gm_no2, SAMPLING_FREQ_HZ);
            X[11] = getMax(gm_no2, SAMPLING_FREQ_HZ);
            X[12] = getStdDev(gm_no2, SAMPLING_FREQ_HZ);
            X[13] = getRms(gm_no2, SAMPLING_FREQ_HZ);
            X[14] = getMean(gm_no2, SAMPLING_FREQ_HZ);
            // grove co
            X[15] = getMin(gm_co, SAMPLING_FREQ_HZ);
            X[16] = getMax(gm_co, SAMPLING_FREQ_HZ);
            X[17] = getStdDev(gm_co, SAMPLING_FREQ_HZ);
            X[18] = getRms(gm_co, SAMPLING_FREQ_HZ);
            X[19] = getMean(gm_co, SAMPLING_FREQ_HZ);
            // ens_tvoc
            X[20] = getMin(ens_tvoc, SAMPLING_FREQ_HZ);
            X[21] = getMax(ens_tvoc, SAMPLING_FREQ_HZ);
            X[22] = getStdDev(ens_tvoc, SAMPLING_FREQ_HZ);
            X[23] = getRms(ens_tvoc, SAMPLING_FREQ_HZ);
            X[24] = getMean(ens_tvoc, SAMPLING_FREQ_HZ);
            // ens_co2
            X[25] = getMin(ens_co2, SAMPLING_FREQ_HZ);
            X[26] = getMax(ens_co2, SAMPLING_FREQ_HZ);
            X[27] = getStdDev(ens_co2, SAMPLING_FREQ_HZ);
            X[28] = getRms(ens_co2, SAMPLING_FREQ_HZ);
            X[29] = getMean(ens_co2, SAMPLING_FREQ_HZ);
            // bme temp
            X[30] = getMin(bme_temp, SAMPLING_FREQ_HZ);
            X[31] = getMax(bme_temp, SAMPLING_FREQ_HZ);
            X[32] = getStdDev(bme_temp, SAMPLING_FREQ_HZ);
            X[33] = getRms(bme_temp, SAMPLING_FREQ_HZ);
            X[34] = getMean(bme_temp, SAMPLING_FREQ_HZ);

            char* pred = classifier.predictLabel(X);

            lcd.clear();
            lcd.setCursor(4, 0);
            lcd.print(pred);

            Serial.print("Classified Label: ");
            Serial.println(pred);
        }
        sample_count = (sample_count + 1) % SAMPLING_FREQ_HZ;
        prevSensorMillis = sensor_reading.timestamp;
    }
}
