#include <math.h>
#include <SoftwareSerial.h>                      
#include <Wire.h>
// include "ScioSense_ENS160.h"  // ENS160 library
#include <SPI.h>
// include <Adafruit_Sensor.h>
// include <Adafruit_BME280.h>
#include "Multichannel_Gas_GMXXX.h"

// FOR SENSORS

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
GAS_GMXXX<TwoWire> gas;               // Multichannel gas sensor v2

// Initialize BME
// Adafruit_BME280 bme; // I2C

// ScioSense_ENS160      ens160(ENS160_I2CADDR_0);
// ScioSense_ENS160      ens160(ENS160_I2CADDR_1);
               
struct SensorReading
{
    // float gm_no2_v = 0.0;
    float gm_eth_v = 0.0;
    float gm_voc_v = 0.0;
    // float gm_co_v = 0.0;
    unsigned long timestamp = 0;
    // float ens_TVOC = 0.0;
    // float ens_CO2 = 0.0;
    // float bme_temp = 0.0;
    // float bme_pressure = 0.0;
    // float bme_altitude = 0.0;
    // float bme_humidity = 0.0;
};

SensorReading sensor_reading;
SensorReading sensor_readings_per_second[SAMPLING_FREQ_HZ];
int sample_count = 0; // goes from 0 to SAMPLING_FREQ_HZ - 1

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
int wheelSpeedLeft = 170;
int wheelSpeedRight = 150;
const float SCALE = 1.7;

float target_X = 0.0;
float target_Y = 0.0; 
float target_angle = 0.0;
int rotation_complete = 1;
int translation_complete = 1;

// right - M2 - B
// left - M1 - A

// FOR SCANNING

const float DETECTION_THRESHOLD = 3.3;
const float CONFIRMATION_THRESHOLD = 4;
const float TRANSLATION = 0.15; // Translation (m)

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

// generate a random x, y coordinate
// compute theta between current x, y and generated
// rotate to new orientation
// translate to new coordinate

// sensor setup
void sensorSetup() {
    unsigned status;
    bool label = true;

    // grove initialization
    gas.begin(Wire, 0x08);
    
    // status = bme.begin();  
    // if (!status) {
    //     Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    //     Serial.print("SensorID was: 0x"); 
    //     Serial.println(bme.sensorID(),16);
    //     Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    //     Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    //     Serial.print("        ID of 0x60 represents a BME 280.\n");
    //     Serial.print("        ID of 0x61 represents a BME 680.\n");
    // }

    // ens160 initialization
    // ens160.begin();
    // if (ens160.available()) {
    //     ens160.setMode(ENS160_OPMODE_STD);
    // }
  
    if (label) {
        Serial.print("grove_voc1");
        Serial.print(",");
        // Serial.print("grove_no2");
        // Serial.print(",");
        Serial.print("grove_eth");
        Serial.print(",");
        // Serial.print("grove_co");
        // Serial.print(",");
        // Serial.print("ens_tvoc");
        // Serial.print(",");
        // Serial.print("ens_co2");
        // Serial.print(",");
        // Serial.print("temperature");
        // Serial.print(",");
        // Serial.print("pressure");
        // Serial.print(",");
        // Serial.print("altitude");
        // Serial.print(",");
        // Serial.print("humidity");
        // Serial.print(",");
        Serial.println("label");
        label=false;
    }
}

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
    delay(5000);
    randomSeed(analogRead(0));
}

// main setup
void setup() {
    reset();
    Serial.begin(115200);
    sensorSetup();
    motorSetup();
}

// main loop
void loop() {
    if (scent_confirmed) {
        /* scent confirmed after detetction and scanning, stop motors */
        Serial.println("SCENT CONFIRMED");
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        delay(10000);
        reset();
    } 
    else if (translation_complete == 1 && !scent_detected) {
        Serial.print("---------------------- RANDOM EXPLORATION MODE ----------------------");
        /* RANDOM SEARCH MODE 
        Compute next random x, y to translate to */

        float x, y;
        
        float a = random(target_X * 100 + 10, target_X * 100 + 20);
        float b = random(target_X * 100 - 20, target_X * 100 - 10);
        float c = random(target_Y * 100 + 10, target_Y * 100 + 20);
        float d = random(target_Y * 100 - 20, target_Y * 100 - 10);
        
        x = random(2) ? a : b;
        y = random(2) ? c : d;
        
        target_X = constrain(x, 0, 75) / 100.0;
        target_Y = constrain(y, 0, 75) / 100.0;
        Serial.print("new x: ");
        Serial.println(target_X);
        Serial.print("new y: ");
        Serial.println(target_Y);
        Serial.println(c);
        Serial.println(d);
        target_angle = getTheta(state[0], state[1], target_X, target_Y);
        rotation_complete = 0;
        translation_complete = 0;
    } 
    else if (translation_complete == 1 && scent_detected) {
        /* TARGETED SEARCH MODE 
        Scent has been detected, scan different angles in position*/
        Serial.println("\n\n\n-------***-------TARGETED SEARCH MODE-------***-------\n\n\n");        
        if (curr_scan < NUM_SCANS) { // SCANNING IN-PLACE
            scan_mode = true;
            Serial.print("------------------------------\n");
            Serial.print("Scan: ");
            Serial.println(curr_scan);
            Serial.print("------------------------------\n");
            // If sufficient samples have been taken, rotate to next position
            
            if (curr_scan == -1) {
                // first scan, initialize
                curr_scan++;
                num_samples = 0;
                scan_readings[0] = EMPTY_SCAN_READINGS[0];
                target_angle = fmod(state[2] + ((-90) * PI / 180.0), 2*PI);
                rotation_complete = 0;
                translation_complete = 1;
            }

            if (num_samples == SCAN_TIME * SAMPLING_FREQ_HZ) {
                // completed samples for this scan, set to next scan
                // Serial.print("------------------------------\n");
                // Serial.print("Num samples before reset: ");
                // Serial.println(num_samples);
                // Serial.print("Prev scan number: ");
                // Serial.println(curr_scan);


                num_samples = 0;
                curr_scan++;
                target_angle = fmod(state[2] + ((scan_angle) * PI / 180.0), 2*PI);

                // Serial.print("New scan number: ");
                // Serial.println(curr_scan);
                // Serial.print("current_angle (deg): ");
                // Serial.println(rad2deg(state[2]));
                // Serial.print("target_angle (deg): ");
                // Serial.println(rad2deg(target_angle));
                // Serial.print("------------------------------\n");
                rotation_complete = 0;
                translation_complete = 1;
            }
        } 
        else { // MOVING TOWARDS SCENT
            scan_mode = false;
            
            // Find direction of highest concentration
            int max_i = 0;
            int max_val = 0;
            int low_detection_count = 0;
            for (int i = 0; i < NUM_SCANS; i++) {
                int val = scan_readings[i].gm_voc_v;
                if (val < DETECTION_THRESHOLD){
                  low_detection_count++;
                }
                if (val > max_val) {
                    max_i = i;
                    max_val = val;
                }
            }
            
            // Reverts back to RANDOM mode if low detection levels
            if (low_detection_count > (NUM_SCANS - 1)) {
                scent_detected = false;
            } 
            else {
                // Set new coordinates in direction of highest concentration
                target_angle = state[2] - (scan_angle * (NUM_SCANS - 1 - max_i)) * PI / 180;
                target_X = state[0] + TRANSLATION * cos(target_angle);
                target_Y = state[1] + TRANSLATION * sin(target_angle);
    
                // Serial.print("_____!!!!!-------------TRANSLATION MODE-----------------!!!!\n");
                // Serial.print("max i value (best scan): ");
                // Serial.println(max_i);
                // Serial.print("curr angle: ");
                // Serial.println(rad2deg(state[2]));
                // Serial.print("target angle: ");
                // Serial.println(rad2deg(target_angle));
                // Serial.print("target_X: ");
                // Serial.println(target_X);
                // Serial.print("target_Y: ");
                // Serial.println(target_Y);
                // Serial.print("------------------------------\n");
    
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

float bestFitSlope(SensorReading *sensor_readings_set, int sensor_used) {
    float sum_x = 450.0, sum_y = 0.0, sum_x2 = 28500.0, sum_y2 = 0.0, sum_xy = 0.0, slope = 0.0; 
    float y = 0.0;
    for (int i = 0; i < SAMPLING_FREQ_HZ; i++) {
        //sum_x += (SAMPLING_PERIOD_MS * i);
        //sum_x2 += (SAMPLING_PERIOD_MS * SAMPLING_PERIOD_MS * i * i);
        // if (sensor_used == 0) {
        //     // ens TVOC
        //     y = sensor_readings_set[i].ens_TVOC;
        // }
        // else if (sensor_used == 1) {
        //     // ens CO2
        //     y = sensor_readings_set[i].ens_CO2;
        // }
        if (sensor_used == 2) {
            // grove voc
            y = (100.0 * sensor_readings_set[i].gm_voc_v);
        }
        else if (sensor_used == 3) {
            // grove ethanol
            y = sensor_readings_set[i].gm_eth_v;
        }
        sum_y += y;
        sum_y2 += (y * y);
        sum_xy += (SAMPLING_PERIOD_MS * i * y / 10.0);
    }
    Serial.println(sum_x);
    Serial.println(sum_x2);
    Serial.println(sum_y);
    Serial.println(sum_y2);
    Serial.println(sum_xy);

    slope = (SAMPLING_FREQ_HZ * sum_xy - sum_x * sum_y) / (SAMPLING_FREQ_HZ * sum_x2 - sum_x * sum_x);
    return slope;
}

// read from sensor every SAMPLING_PERIOD_MS
void readSensors() {
    // Take timestamp so we can hit our target frequency
    sensor_reading.timestamp = millis();

    if (sensor_reading.timestamp > prevSensorMillis + SAMPLING_PERIOD_MS) {
        
         // Read from GM-X02b sensors (multichannel gas)
        // sensor_readings_per_second[sample_count].gm_no2_v = gas.calcVol(gas.getGM102B());
        sensor_readings_per_second[sample_count].gm_eth_v = gas.calcVol(gas.getGM302B());
        sensor_readings_per_second[sample_count].gm_voc_v = gas.calcVol(gas.getGM502B());
        // sensor_readings_per_second[sample_count].gm_co_v = gas.calcVol(gas.getGM702B());
        // sensor_reading.gm_no2_v = gas.calcVol(gas.getGM102B());
        sensor_reading.gm_eth_v = gas.calcVol(gas.getGM302B());
        sensor_reading.gm_voc_v = gas.calcVol(gas.getGM502B());
        // sensor_reading.gm_co_v = gas.calcVol(gas.getGM702B());

        //Read from ENS-160
        // if (ens160.available()) {
        //     ens160.measure(0);
        //     sensor_readings_per_second[sample_count].ens_TVOC = ens160.getTVOC();
        //     sensor_readings_per_second[sample_count].ens_CO2 = ens160.geteCO2();
        //     sensor_reading.ens_TVOC = ens160.getTVOC();
        //     sensor_reading.ens_CO2 = ens160.geteCO2();
        // }

        // Serial.print("\nTVOC: ");
        // Serial.println(sensor_reading.ens_TVOC);
        // Serial.print("\nCO2:");
        // Serial.println(sensor_reading.ens_CO2);
        Serial.print("\nGrove Ethanol:");
        Serial.println(sensor_reading.gm_eth_v);
        Serial.print("\nGrove VOC:");
        Serial.println(sensor_reading.gm_voc_v);

        if (sensor_reading.gm_voc_v > CONFIRMATION_THRESHOLD) {
            scent_confirmed = true;
            scent_detected = false;
        } 
        // else if (sensor_reading.ens_TVOC > DETECTION_THRESHOLD) {
        //     scent_detected = true;
        // }
        if (sample_count == (SAMPLING_FREQ_HZ - 1)) {
            float slope = bestFitSlope(sensor_readings_per_second, 2);
            Serial.print("slope over a second: ");
            Serial.println(slope);
            if (slope > 0.02) {
                Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
                scent_detected = true;
            }
        }

        // Save values
        // Serial.println("__--------------------+++++");
        // Serial.print("Num samples: ");
        // Serial.println(num_samples);
        // Serial.print("Scan mode?: ");
        // Serial.println(scan_mode);
        // Serial.print("Rotation complete?: ");
        // Serial.println(rotation_complete);
        // Serial.println("__--------------------+++++");
        if (scan_mode && (rotation_complete == 1) && (num_samples < (SCAN_TIME * SAMPLING_FREQ_HZ))) {
            // scan_readings[curr_scan].ens_TVOC += (sensor_reading.ens_TVOC / (SCAN_TIME * SAMPLING_FREQ_HZ));
            // scan_readings[curr_scan].ens_CO2 += (sensor_reading.ens_CO2 / (SCAN_TIME * SAMPLING_FREQ_HZ));
            scan_readings[curr_scan].gm_eth_v += (sensor_reading.gm_eth_v / (SCAN_TIME * SAMPLING_FREQ_HZ));
            scan_readings[curr_scan].gm_voc_v += (sensor_reading.gm_voc_v / (SCAN_TIME * SAMPLING_FREQ_HZ));
            num_samples++;
        }
        sample_count = (sample_count + 1) % SAMPLING_FREQ_HZ;
        prevSensorMillis = sensor_reading.timestamp;
    }
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

    // Serial.print("countsleft: ");
    // Serial.println(countsLeft);
    // Serial.print("countsright: ");
    // Serial.println(countsRight);
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

    // Serial.print("w_robot: ");
    // Serial.println(w_robot); 
    // Serial.print("x: ");
    // Serial.println(state[0]);
    // Serial.print("y: ");
    // Serial.println(state[1]);
    // Serial.print("theta: ");
    // Serial.println(rad2deg(state[2]));

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
}

// translate from current coordinate to target coordinate
void translation(float current_x, float current_y, float target_x, float target_y) {
    wheelSpeedLeft = 130;
    wheelSpeedRight = 110;

    distFromTarget = getDistance(current_x, current_y, target_x, target_y);
    Serial.println("distance from target: ");
    Serial.println(distFromTarget);

    if (distFromTarget <= prevDistFromTarget) {
       // Approaching target, continue
       prevDistFromTarget = distFromTarget;
       if (getDistance(current_x, current_y, target_x, target_y) > 0.02) {
            Serial.println("forward");
            setMotor(FORWARD, wheelSpeedLeft, enA, IN1, IN2);
            setMotor(FORWARD, wheelSpeedRight, enB, IN3, IN4);
        }
        else { // Target reached, stop
            setMotor(STOP, 0, enA, IN1, IN2);
            setMotor(STOP, 0, enB, IN3, IN4);
            Serial.println("translation reached");
            translation_complete = 1;
        }
    } else {
      // Target missed, stop
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        Serial.println("HARD STOP: > 2cm away from the target");

        // Corrective action
        prevDistFromTarget = 100;
        rotation_complete = 0; // Reset the turn - rotate and translate again to meet the target
        target_angle = getTheta(current_x, current_y, target_X, target_Y);
        Serial.print("new theta: ");
        Serial.println(target_angle);
    }
}    

// rotate from current orientation to target orientation
void rotation(float current_theta, float target_theta) {
    wheelSpeedLeft = 130;
    wheelSpeedRight = 110;

    Serial.print("rotation - current_theta: ");
    Serial.println(rad2deg(current_theta));
    Serial.print("rotation - target_theta: ");
    Serial.println(rad2deg(target_theta));

    if (rad2deg(target_theta) - rad2deg(current_theta) > 3) {
        // Clockwise
        setMotor(BACKWARD, wheelSpeedLeft, enA, IN1, IN2);
        setMotor(FORWARD, wheelSpeedRight, enB, IN3, IN4);
    }
    else if (rad2deg(target_theta) - rad2deg(current_theta) < -3) {
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
    return fmod(theta, 2 * PI);
}

// get the l2 distance between 2 points
float getDistance(float x1, float y1, float x2, float y2) {
    return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
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
