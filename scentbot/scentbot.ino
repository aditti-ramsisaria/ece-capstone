#include <math.h>
#include <SoftwareSerial.h>                      
#include <Wire.h>
#include <SPI.h>
#include "Multichannel_Gas_GMXXX.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ScioSense_ENS160.h"
#include "MultiClassification_linear.h"
#include <Adafruit_NeoPixel.h>

// FOR NEOPIXEL
#define PIXEL_PIN   45   // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 12   // number of neopixel (change this accordingly)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// FOR SENSORS

#define trigPin_front 12
#define echoPin_front 13
#define trigPin_left 14
#define echoPin_left 15
#define trigPin_right 16
#define echoPin_right 17
const int DISTANCE_THRESHOLD = 10; // cm

// Settings
#define serialCommunicationSpeed 115200               
#define DEBUG true 
#define SEALEVELPRESSURE_HPA (1013.25)
#define BTN_START           0                         // 1: press button to start, 0: loop
#define BTN_PIN             WIO_5S_PRESS              // Pin that button is connected to
#define SAMPLING_FREQ_HZ    10                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
unsigned long prevSensorMillis;

// Global objects

// Initialize Grove
GAS_GMXXX<TwoWire> gas;               // Multichannel gas sensor v2
// Initialize BME
Adafruit_BME280 bme; 
// Initialize ENS
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

LiquidCrystal_I2C lcd(0x3F, 16, 2); 
               
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

SensorReading sensor_reading;
SensorReading sensor_readings_per_second[SAMPLING_FREQ_HZ];
int sample_count = 0; // goes from 0 to SAMPLING_FREQ_HZ - 1

// For Classifier
// CLASSIFICTION_DEBUG
Eloquent::ML::Port::SVM classifier;
float X[35];
float gm_voc[SAMPLING_FREQ_HZ];
float gm_no2[SAMPLING_FREQ_HZ];
float gm_eth[SAMPLING_FREQ_HZ];
float gm_co[SAMPLING_FREQ_HZ];
float ens_tvoc[SAMPLING_FREQ_HZ];
float ens_co2[SAMPLING_FREQ_HZ];
float bme_temp[SAMPLING_FREQ_HZ];
float bme_humidity[SAMPLING_FREQ_HZ];
float bme_pressure[SAMPLING_FREQ_HZ];
float bme_altitude[SAMPLING_FREQ_HZ];

// dataset dependent values
const float voc_min = 0.92, voc_max = 4.84;
const float eth_min = 0.89, eth_max = 4.78;
const float no2_min = 0.77, no2_max = 4.67;
const float co_min = 1.06, co_max = 4.94;
const float tvoc_min = 1.0, tvoc_max = 25634.0;
const float co2_min = 400.0, co2_max = 12119.0;
const float temp_min = 21.79, temp_max = 27.37;


// FOR MOTORS

// Motor 1
#define IN1 4
#define IN2 5
#define enA 6
#define M1_ENCA 2
#define M1_ENCB 10

// Motor 2
#define IN3 7
#define IN4 8
#define enB 9
#define M2_ENCA 3
#define M2_ENCB 11

#define FORWARD 1
#define STOP 0
#define BACKWARD -1

// Constants
const int COUNTS_PER_ROTATION = 14;
const float GEAR_RATIO = 100.0F;
const float WHEEL_DIAMETER = 0.068;
const float WHEEL_CIRCUMFERENCE = 0.2136;
const float WHEEL_BASE = 0.142;

// MAP SIZE
const float MAP_X = 1.5;
const float MAP_Y = 1.5;

const unsigned long PERIOD = 50;

// Global variables
unsigned long currentMotorMillis;
unsigned long prevMotorMillis;

long M1_encoder_val = 0;
long M2_encoder_val = 0; 

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

float t = 0.0F;
float w_left = 0.0F;
float w_right = 0.0F;
float left_speed = 0.0F;
float right_speed = 0.0F;
float w_robot = 0.0F;
float robot_speed = 0.0F;
float k00, k01, k10, k11, k30, k31;

float state[3] = {0.0, 0.0, 0.0};
float last_state[3] = {0.0, 0.0, 0.0};

float distFromTarget = 0;
float prevDistFromTarget = 100;

// Parameters
int wheelSpeedLeft = 150;
int wheelSpeedRight = 140;
const float SCALE = 1.7;

float target_X = 0.0;
float target_Y = 0.0; 
float target_angle = 0.0;
int rotation_complete = 1;
int translation_complete = 1;
bool reverse_mode = false;

// right - M2 - B
// left - M1 - A

// FOR SCANNING

const float CONFIRMATION_THRESHOLD_TVOC = 600.0;
const float CONFIRMATION_THRESHOLD_GROVE_ETH = 2.0;
const float SLOPE_THRESHOLD_ETH = 0.02;
const float SLOPE_THRESHOLD_TVOC = 20.0;

const float TRANSLATION = 0.15; // Translation (m)
const float BACK_TRANSLATION = 0.1; // (m)

const int NUM_SCANS = 6; // Number of scans per rotation
const int SCAN_TIME = 3; // Time stopped to sample (s)
int scan_angle = 180 / NUM_SCANS; // Angle between samples (deg)
const SensorReading EMPTY_SCAN_READINGS[NUM_SCANS];

bool scent_detected = false;
bool scent_confirmed = false;
SensorReading scan_readings[NUM_SCANS];
bool scan_mode = false;

int curr_scan = -1; // [0, NUM_SCANS)
int num_samples = 0; // [0, SCAN_TIME * SAMPLING_FREQ_HZ) 

/* MATH FUNCTIONS */

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

// compute slope of best fit line over array of sensor readings
float bestFitSlope(SensorReading *sensor_readings_set, int num_readings, int sensor_used) {
    float sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0, sum_y2 = 0.0, sum_xy = 0.0, slope = 0.0; 
    if (num_readings == 10) {
        sum_x = 450.0;
        sum_x2 = 28500.0;
    }
    if (num_readings == 30) {
        sum_x = 4350.0;
        sum_x2 = 85550000.0;
    }
    float y = 0.0;
    for (int i = 0; i < num_readings; i++) {
        if (sensor_used == 2) {
            // grove voc
            y = (100.0 * sensor_readings_set[i].gm_voc_v);
        }
        else if (sensor_used == 3) {
            // grove ethanol
            y = (100.0 * sensor_readings_set[i].gm_eth_v);
        }
        else if (sensor_used == 4) {
            y = (100.0 * sensor_readings_set[i].ens_TVOC);
        }
        sum_y += y;
        sum_y2 += (y * y);
        sum_xy += (SAMPLING_PERIOD_MS * i * y / 10.0);
    }
   
    slope = (num_readings * sum_xy - sum_x * sum_y) / (num_readings * sum_x2 - sum_x * sum_x);
    return slope;
}

// get the l2 distance between 2 points
float getDistance(float x1, float y1, float x2, float y2) {
    return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
}

// convert from radians to degrees
float rad2deg(float theta) {
    return theta * (180.0) / PI;
}

// get target orientation to get from one coordiante to the other
float getTheta(float x1, float y1, float x2, float y2) {
    // put into local coordinate system
    // returns radians
    x2 = x2 - x1;
    y2 = y2 - y1;
    float theta = 0.0;
    
    if ((fabs(x2) < 1e-7) && (y2 < 0)) {
        theta = 1.5 * PI;
    }
    else if ((x2 == 0) and (y2 > 0)) {
        theta = 0.5 * PI;
    }
    else {
        theta = atan2(y2, x2);
    } 
    if (theta < 0) {
        theta = theta + 2 * PI;
    }
    theta = fmod(theta, 2 * PI);
    if (theta > PI) {
        theta = theta - 2 * PI;
    }
    return theta;
}

// normalize an array
void normalize_array(float* val, float min_val, float max_val, int arrayCount) {
    for (int i = 0; i < arrayCount; i++) {
        val[i] = (val[i] - min_val) / (max_val - min_val);
    }
}

/* SENSOR FUNCTIONS */

// sensor setup
void sensorSetup() {
    unsigned status;

    pinMode(trigPin_front, OUTPUT);
    pinMode(echoPin_front, INPUT);
    pinMode(trigPin_left, OUTPUT);
    pinMode(echoPin_left, INPUT);
    pinMode(trigPin_right, OUTPUT);
    pinMode(echoPin_right, INPUT);

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

// set up feature vector for classification
void featureSetup() {
    // calculate min, max, std, rms, avg for each
    // do normalization first
    normalize_array(gm_voc, voc_min, voc_max, SAMPLING_FREQ_HZ);
    normalize_array(gm_eth, eth_min, eth_max, SAMPLING_FREQ_HZ);
    normalize_array(gm_co, co_min, co_max, SAMPLING_FREQ_HZ);
    normalize_array(gm_no2, no2_min, no2_max, SAMPLING_FREQ_HZ);
    normalize_array(ens_tvoc, tvoc_min, tvoc_max, SAMPLING_FREQ_HZ);
    normalize_array(ens_co2, co2_min, co2_max, SAMPLING_FREQ_HZ);
    normalize_array(bme_temp, temp_min, temp_max, SAMPLING_FREQ_HZ);

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
    // currently not including humidity, pressure or altitude
}

// read from ultrasonic sensor
float readUltrasonic(int trigPin, int echoPin) {
    long duration;
    float distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2.0) / 29.1;
    return distance;
}

// read from sensor every SAMPLING_PERIOD_MS
void readSensors() {
    // Take timestamp so we can hit our target frequency
    sensor_reading.timestamp = millis();

    if (sensor_reading.timestamp > prevSensorMillis + SAMPLING_PERIOD_MS) {
        
        // Read from GM-X02b sensors (multichannel gas)
        sensor_readings_per_second[sample_count].gm_eth_v = gas.calcVol(gas.getGM302B());
        sensor_readings_per_second[sample_count].gm_voc_v = gas.calcVol(gas.getGM502B());
        sensor_readings_per_second[sample_count].gm_no2_v = gas.calcVol(gas.getGM102B());
        sensor_readings_per_second[sample_count].gm_co_v = gas.calcVol(gas.getGM702B());

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
            sensor_readings_per_second[sample_count].ens_TVOC = ens160.getTVOC();
            sensor_readings_per_second[sample_count].ens_CO2 = ens160.geteCO2();
            ens_tvoc[sample_count] = ens160.getTVOC();
            ens_co2[sample_count] = ens160.geteCO2();
            sensor_reading.ens_TVOC = ens160.getTVOC();
            sensor_reading.ens_CO2 = ens160.geteCO2();
        }

        // Read from BME-280
        sensor_readings_per_second[sample_count].bme_temp = bme.readTemperature();
        sensor_readings_per_second[sample_count].bme_pressure = bme.readPressure();
        sensor_readings_per_second[sample_count].bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        sensor_readings_per_second[sample_count].bme_humidity = bme.readHumidity();

        bme_temp[sample_count] = bme.readTemperature();
        bme_humidity[sample_count] = bme.readHumidity();
        bme_pressure[sample_count] = bme.readPressure();
        bme_altitude[sample_count] = bme.readAltitude(SEALEVELPRESSURE_HPA);

        sensor_reading.bme_temp = bme.readTemperature();
        sensor_reading.bme_pressure = bme.readPressure();
        sensor_reading.bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        sensor_reading.bme_humidity = bme.readHumidity();
        
        lcd.setCursor(0, 1);
        lcd.print("T:");
        lcd.print(sensor_reading.ens_TVOC);
        lcd.setCursor(9, 1);
        lcd.print("E:");
        lcd.print(sensor_reading.gm_eth_v);
       
        if (sample_count == (SAMPLING_FREQ_HZ - 1)) {
        // DEBUG: TESTING AND INSTEAD OF OR
            if ((sensor_reading.gm_eth_v > CONFIRMATION_THRESHOLD_GROVE_ETH || 
                 sensor_reading.ens_TVOC > CONFIRMATION_THRESHOLD_TVOC) && 
                 (scent_detected == true)) {
                Serial.println("CONFIRMATION eth, tvoc");
                Serial.print(sensor_reading.gm_eth_v);
                Serial.println(sensor_reading.ens_TVOC);
            
                scent_confirmed = true;
                scent_detected = false;
                
                // CLASSIFICATION_DEBUG confirmed that there is a scent there, do classification
                featureSetup();

                int* predicted = classifier.predict(X);
                int idx = predicted[0];
                char* pred = classifier.idxToLabel(idx);
                
                lcd.clear();
                lcd.setCursor(0, 1); 
                lcd.print(pred);
                confirmedScent(pred);

            } 

            else {
                // find best fit slope of ethanol readings over a period of a second
                float slope_grove_eth = bestFitSlope(sensor_readings_per_second, SAMPLING_FREQ_HZ, 3);
                float slope_ens_tvoc = bestFitSlope(sensor_readings_per_second, SAMPLING_FREQ_HZ, 4);
                Serial.print("slopes eth, tvoc: ");
                Serial.println(slope_grove_eth);
                Serial.println(slope_ens_tvoc);
                if ((scent_detected == false) && 
                    (((slope_grove_eth > SLOPE_THRESHOLD_ETH) || 
                    (slope_ens_tvoc > SLOPE_THRESHOLD_TVOC)))) {
                    scent_detected = true;
                    strip.fill(strip.Color(0,0,0), 0, 12);
                    translation_complete = 1;
                }
            }
        }

        // Save values
        
        if (scan_mode && (rotation_complete == 1) && (num_samples < (SCAN_TIME * SAMPLING_FREQ_HZ))) {
            scan_readings[curr_scan].gm_eth_v += (sensor_reading.gm_eth_v / (SCAN_TIME * SAMPLING_FREQ_HZ));
            scan_readings[curr_scan].ens_TVOC += (sensor_reading.ens_TVOC / (SCAN_TIME * SAMPLING_FREQ_HZ));
            num_samples++;
        }
        sample_count = (sample_count + 1) % SAMPLING_FREQ_HZ;
        prevSensorMillis = sensor_reading.timestamp;
    }
}

/* MOTOR FUNCTIONS */

// motor setup 
void motorSetup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    
    pinMode(M1_ENCA, INPUT);
    pinMode(M1_ENCB, INPUT);
    attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoderM1, RISING);
    
    pinMode(M2_ENCA,INPUT);
    pinMode(M2_ENCB,INPUT);
    attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoderM2, RISING);
  
    Serial.println("Starting in 5s");
    //delay(5000);
    readySetGo();
    randomSeed(analogRead(0));
}

// compute random x, y, orientation
void computeRandomConfig(float current_theta, float current_x, float current_y) {
    // compute a new orientation to turn to in random exploration within 180 deg of current
    // computes target cooridnates using new theta
    target_angle = random(current_theta - PI / 2.0, current_theta + PI / 2.0);
    float new_distance = random(15.0, 40.0) / 100.0;
    float new_x = current_x + new_distance * cos(target_angle);
    float new_y = current_y + new_distance * sin(target_angle);
    target_X = constrain(new_x, 0.0, MAP_X);
    target_Y = constrain(new_y, 0.0, MAP_Y);
}

// odometry computations and main loop commands
void checkEncoders() {
  currentMotorMillis = millis();
  if (currentMotorMillis > prevMotorMillis + PERIOD) {

    countsLeft += M1_encoder_val;
    if (abs(M1_encoder_val) > 1000) {
        Serial.print("M1_encoder_val: ");
        Serial.println(M1_encoder_val);
        Serial.print("M2_encoder_val: ");
        Serial.println(M2_encoder_val);
        countsLeft = prevLeft;
        Serial.println("fuck this, jank issue with left");
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
    }
    M1_encoder_val = 0;
    
    countsRight += M2_encoder_val;
    if (abs(M2_encoder_val) > 1000) {
        Serial.print("M1_encoder_val: ");
        Serial.println(M1_encoder_val);
        Serial.print("M2_encoder_val: ");
        Serial.println(M2_encoder_val);
        countsRight = prevRight;
        Serial.println("fuck this, jank issue with right");
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
    }
    
    M2_encoder_val = 0;


    t = (currentMotorMillis - prevMotorMillis) / 1000.0;

    w_left = SCALE * 2.0 * PI * ((countsLeft - prevLeft) / (COUNTS_PER_ROTATION * GEAR_RATIO)) / t;
    w_right = SCALE * 2.0 * PI * ((countsRight - prevRight) / (COUNTS_PER_ROTATION * GEAR_RATIO)) / t;

    left_speed = w_left * WHEEL_DIAMETER / 2.0F;
    right_speed =  -1.0 * w_right * WHEEL_DIAMETER / 2.0F;

    robot_speed = (left_speed + right_speed) / 2.0; // in m/s

    w_robot = -1.0 * (right_speed - left_speed) / WHEEL_BASE;  // in rad/s

    k00 = robot_speed * cos(last_state[2]);  // last_state[2]; // in radians
    k01 = robot_speed * sin(last_state[2]);  // last_state[2]; // in radians

    k10 = (robot_speed) * cos(last_state[2] + ((t * w_robot) / 2));
    k11 = (robot_speed) * sin(last_state[2] + ((t * w_robot) / 2));
    // note that k20 = k10, k21 = k11

    k30 = robot_speed * cos(last_state[2] + (t * w_robot));
    k31 = robot_speed * sin(last_state[2] + (t * w_robot));

    state[0] = last_state[0] + ((t / 6) * (k00 + 4 * k10 + k30)); // x
    state[1] = last_state[1] + ((t / 6) * (k01 + 4 * k11 + k31)); // y
    state[2] = fmod(last_state[2] + (t * w_robot), 2 * PI);

    float distance_front;
    float distance_left;
    float distance_right;
    distance_front = readUltrasonic(trigPin_front, echoPin_front);
    distance_left = readUltrasonic(trigPin_left, echoPin_left);
    distance_right = readUltrasonic(trigPin_right, echoPin_right);

    if ((distance_front < DISTANCE_THRESHOLD) && (scent_detected == false)) {
        Serial.println("STOP FROM ULTRASONIC");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WALL   DETECTED");
        lcd.setCursor(0, 1);
        lcd.print("BACKING UP...");
        // SET NEOPIXEL TO RED
        wallDetected();
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        delay(100);
        lcd.clear();//Clean the screen
        lcd.setCursor(0, 0); 
        rotation_complete = 1;
        translation_complete = 0;
        target_angle = fmod(state[2] - PI, 2*PI);
        target_X = state[0] + BACK_TRANSLATION * cos(target_angle);
        target_Y = state[1] + BACK_TRANSLATION * sin(target_angle);
        reverse_mode = true;
    } else if (((distance_left < DISTANCE_THRESHOLD) || (distance_right < DISTANCE_THRESHOLD))) {
        // If in scan_mode and rotating
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SIDE   DETECTED");
        lcd.setCursor(0, 1);
        lcd.print("BACKING UP...");
        wallDetected();
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        delay(100);
        lcd.clear();//Clean the screen
        lcd.setCursor(0, 0); 

        if (scan_mode) { // Reverse
            rotation_complete = 1;
            translation_complete = 0;
            float angle = fmod(state[2] - PI, 2*PI);
            reverse_mode = true;
            target_X = state[0] + BACK_TRANSLATION * cos(angle);
            target_Y = state[1] + BACK_TRANSLATION * sin(angle);
        } else {
            float dir = (distance_left < DISTANCE_THRESHOLD) ? 1.0 : -1.0;
            rotation_complete = 0;
            translation_complete = 0;
            target_angle = fmod(state[2] - (dir * PI/2), 2*PI);
            reverse_mode = false;
            target_X = state[0] + TRANSLATION * cos(target_angle);
            target_Y = state[1] + TRANSLATION * sin(target_angle);
        }
    }

    if (rotation_complete == 0) {
        // rotate till theta reached
        rotation(state[2], target_angle);
    }
    else if (rotation_complete == 1 && translation_complete == 0) {
        // translate till x, y, reached
        translation(state[0], state[1], target_X, target_Y);
    }

    last_state[0] = state[0];
    last_state[1] = state[1];
    last_state[2] = state[2];
    
    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMotorMillis = currentMotorMillis;
  }
}

// translate from current coordinate to target coordinate
void translation(float current_x, float current_y, float target_x, float target_y) {
    wheelSpeedLeft = 150;
    wheelSpeedRight = 140;
    
    distFromTarget = getDistance(current_x, current_y, target_x, target_y);
    Serial.print("dist: ");
    Serial.println(distFromTarget);
    Serial.print("prev dist: ");
    Serial.println(prevDistFromTarget);
    if (distFromTarget <= prevDistFromTarget) {
       Serial.println("regular translation: ");
       Serial.println(reverse_mode);
       // Approaching target, continue
       prevDistFromTarget = distFromTarget;
       if (getDistance(current_x, current_y, target_x, target_y) > 0.03) {
            if (reverse_mode) {
                setMotor(BACKWARD, wheelSpeedLeft, enA, IN1, IN2);
                setMotor(BACKWARD, wheelSpeedRight, enB, IN3, IN4);
            } else {
                setMotor(FORWARD, wheelSpeedLeft, enA, IN1, IN2);
                setMotor(FORWARD, wheelSpeedRight, enB, IN3, IN4);
            }
        }
        else { // Target reached, stop
            reverse_mode = false;
            setMotor(STOP, 0, enA, IN1, IN2);
            setMotor(STOP, 0, enB, IN3, IN4);
            Serial.println("translation reached");
            translation_complete = 1;
        }
    } else if (reverse_mode == false) {
      // Target missed, stop
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        Serial.println("HARD STOP: > 5cm away from the target");

        // Corrective action
        prevDistFromTarget = 100;
        rotation_complete = 0; // Reset the turn - rotate and translate again to meet the target
        target_angle = getTheta(current_x, current_y, target_X, target_Y);
    }
    else {
        // no hard stopping in reverse mode
        Serial.println("reverse mode with hard stop");
        translation_complete = 1;
        rotation_complete = 0;
        reverse_mode = false;
        prevDistFromTarget = 100;
    }
}    

// rotate from current orientation to target orientation
void rotation(float current_theta, float target_theta) {
    wheelSpeedLeft = 155;
    wheelSpeedRight = 155;
    float diff = rad2deg(target_theta) - rad2deg(current_theta);

    if (diff > 3) {
        // Clockwise
        setMotor(BACKWARD, wheelSpeedLeft, enA, IN1, IN2);
        setMotor(FORWARD, wheelSpeedRight, enB, IN3, IN4);
    }
    else if ((diff < -3)) {
        // Counterclockwise
        setMotor(FORWARD, wheelSpeedLeft, enA, IN1, IN2);
        setMotor(BACKWARD, wheelSpeedRight, enB, IN3, IN4);
    }
    else {
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        Serial.println("rotation reached");
        prevDistFromTarget = 100;
        rotation_complete = 1;
    }
}

// send PWM signal to motors to go forward, backward, or turn
void setMotor(int dir, int pwmVal, int pwm_pin, int in1, int in2){
    analogWrite(pwm_pin, pwmVal);
    if(dir == 1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else if(dir == -1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }  
}
 
// read from encoder on motor 1
void readEncoderM1(){
    int b = digitalRead(M1_ENCB);
    if(b > 0) {
        M1_encoder_val++;
    }
    else {
        M1_encoder_val--;
    }
}

// read from encoder on motor 2
void readEncoderM2(){
    int b = digitalRead(M2_ENCB);
    if(b > 0) {
        M2_encoder_val++;
    }
    else {
        M2_encoder_val--;
    }
}

/* UTILITY FUNCTIONS */
// reset state
void reset() {
    setMotor(STOP, 0, enA, IN1, IN2);
    setMotor(STOP, 0, enB, IN3, IN4);
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
    last_state[0] = 0.0;
    last_state[1] = 0.0;
    last_state[2] = 0.0;
    countsLeft = 0;
    countsRight = 0;
    rotation_complete = 1;
    translation_complete = 1;
    
    scent_detected = false;
    scent_confirmed = false;
    scan_mode = false;
    for (int i = 0; i < NUM_SCANS; i++) {
        scan_readings[i] = EMPTY_SCAN_READINGS[i];
    }
    curr_scan = -1;
    num_samples = 0;
    lcd.clear();
}

/* NEOPIXEL FUNCTIONS */

void readySetGo() {
  //reset Neopixel to off state
  strip.fill(strip.Color(0,0,0), 0, 12);
  // set first 4 to red
  for(uint16_t i=0; i<4; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
    strip.show();
  }
  delay(1666);
  // setting next 4 pixels to yellow
  for(uint16_t i=4; i<8; i++) {
    strip.setPixelColor(i, strip.Color(204, 204, 0));
    strip.show();
  }
  delay(1666);
  // setting last 4 pixels to green
  for(uint16_t i=8; i<12; i++) {
    strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
  }
  delay(1666);
}

void rainbowCycle() {
  strip.fill(strip.Color(0,0,0), 0, 12);
  uint16_t i;
  for(i=0; i< strip.numPixels(); i++) {
    strip.setPixelColor(i, RGBWheel(((i * 256 / strip.numPixels())) & 255));
  }
  strip.show();
}

void wallDetected() {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
    strip.show();
  }
}

void confirmedScent(char* label) {
  strip.fill(strip.Color(0,0,0), 0, 12);
  if(label == "alcohol") {
    for (int ii = 0; ii < 12; ++ii) {  
      strip.setPixelColor(ii, 0, 255, 0);  
    }
  } else if(label == "paint thinner") {
    for (int ii = 0; ii < 12; ++ii) {  
      strip.setPixelColor(ii, 76, 0, 153);  
    }
  } else {
    for (int ii = 0; ii < 12; ++ii) {  
      strip.setPixelColor(ii, 0, 0, 255);  
    }
  }
  strip.show();  
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t RGBWheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

/* MAIN FUNCTIONS */
// main setup
void setup() {
    lcd.begin(); //Defining 16 columns and 2 rows of lcd display
    lcd.clear();
    lcd.backlight(); //To Power ON the back light
    strip.begin();
    strip.show();
    strip.setBrightness(30);
    reset();
    Serial.begin(115200);
    sensorSetup();
    motorSetup();
}

// main loop
void loop() {
    if (scent_confirmed) {
        /* scent confirmed after detetction and scanning, stop motors */
        lcd.setCursor(0,0); 
        lcd.print("SCENT  CONFIRMED");
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        delay(10000);
        reset();
    } 
    else if (translation_complete == 1 && !scent_detected) {
        Serial.print("---------------------- RANDOM EXPLORATION MODE ----------------------");
        /* RANDOM SEARCH MODE 
        Compute next random x, y to translate to */
        // NEOPIXEL RAINBOW CYCLE
        rainbowCycle();
        computeRandomConfig(state[2], state[0], state[1]);
               
    //    target_X = ARRAY_X[CURRENT_POINT];
    //    target_Y = ARRAY_Y[CURRENT_POINT];
    //    target_angle = getTheta(state[0], state[1], target_X, target_Y);
    //    CURRENT_POINT++;
        
        rotation_complete = 0;
        translation_complete = 0;
    } 
    else if (translation_complete == 1 && scent_detected) {
        /* TARGETED SEARCH MODE 
        Scent has been detected, scan different angles in position*/
        Serial.println("\n\n\n-------***-------TARGETED SEARCH MODE-------***-------\n\n\n");        
        if (curr_scan < NUM_SCANS) { // SCANNING IN-PLACE
            scan_mode = true;
            // If sufficient samples have been taken, rotate to next position
            if (curr_scan == -1) {
                // first scan, initialize
                curr_scan++;
                num_samples = 0;
                scan_readings[0] = EMPTY_SCAN_READINGS[0];
                target_angle = fmod(state[2] + ((-90) * PI / 180.0), 2*PI);
                rotation_complete = 0;
                translation_complete = 1;
                lcd.clear();
            }

            lcd.setCursor(0, 0); 
            lcd.print("SCANNING: ");
            lcd.print(curr_scan);
            strip.setPixelColor(curr_scan*2, strip.Color(255, 128, 0));
            strip.setPixelColor((curr_scan*2) + 1, strip.Color(255, 128, 0));
            strip.show();

            if (num_samples == SCAN_TIME * SAMPLING_FREQ_HZ) {
                // completed samples for this scan, set to next scan
                num_samples = 0;
                curr_scan++;
                target_angle = fmod(state[2] + ((scan_angle) * PI / 180.0), 2*PI);

                rotation_complete = 0;
                translation_complete = 1;
            }
        } 
        else { // MOVING TOWARDS SCENT
            scan_mode = false;
            lcd.clear();
            lcd.setCursor(0, 0);
            // Find direction of highest ethanol concentration
            int max_i = 0;
            float max_val = 0.0;
            float val_eth = 0.0;
            float prev_eth = 0.0;
            float val_tvoc = 0.0;
            float prev_tvoc = 0.0;
            float slope_eth = 0.0;
            float slope_tvoc = 0.0;
                
            int low_detection_count = 0;
            for (int i = 0; i < NUM_SCANS; i++) {
                val_eth = scan_readings[i].gm_eth_v;
                val_tvoc = scan_readings[i].ens_TVOC;
                prev_eth = scan_readings[i - 1].gm_eth_v;
                prev_tvoc = scan_readings[i - 1].ens_TVOC;
                if (i > 0) {
                    slope_eth = (val_eth - prev_eth) / (SCAN_TIME);
                    slope_tvoc = (val_tvoc - prev_tvoc) / (SCAN_TIME);
                    Serial.println("slopes");
                    Serial.println(slope_eth);
                    Serial.println(slope_tvoc);
                    if ((slope_eth <= 0.0 && slope_tvoc <= 5.0)) {
                        // TODO: test and tune slope_tvoc threshold for exit 
                        low_detection_count++;
                    }
                }
                if (val_eth > max_val) {
                    max_i = i;
                    max_val = val_eth;
                }
            }
            
            // Reverts back to RANDOM mode if low detection levels
            if (low_detection_count == (NUM_SCANS - 1)) {
                scent_detected = false;
                rainbowCycle();
            } 
            else {
                // Set new coordinates in direction of highest concentration
                lcd.print("MAX VALUE AT: ");
                lcd.print(max_i);
                target_angle = state[2] - (scan_angle * (NUM_SCANS - 1 - max_i)) * PI / 180;
                target_X = state[0] + TRANSLATION * cos(target_angle);
                target_Y = state[1] + TRANSLATION * sin(target_angle);
                strip.fill(strip.Color(0,0,0), 0, 12);
            }
            
            // reset for next time targeted search mode is reached
            for (int i = 0; i < NUM_SCANS; i++) {
                scan_readings[i] = EMPTY_SCAN_READINGS[i];
            }
            curr_scan = -1;
            num_samples = 0;

            rotation_complete = 0;
            translation_complete = 0;
        }
    }
    checkEncoders(); 
    readSensors();
}
