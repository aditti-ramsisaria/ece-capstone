#include <math.h>
#include <SoftwareSerial.h>                      
#include <Wire.h>
#include <SPI.h>
#include "Multichannel_Gas_GMXXX.h"
#include <LiquidCrystal_I2C.h>

// FOR SENSORS

#define trigPin 12
#define echoPin 13
const int DISTANCE_THRESHOLD = 5; // cm

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
LiquidCrystal_I2C lcd(0x3F, 16, 2); 
               
struct SensorReading
{
    float gm_eth_v = 0.0;
    float gm_voc_v = 0.0;
    unsigned long timestamp = 0;
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

// const int NUM_POINTS = 1;
// const float ARRAY_X[NUM_POINTS] = {0.5};
// const float ARRAY_Y[NUM_POINTS] = {0.0};
// int CURRENT_POINT = 0;

// right - M2 - B
// left - M1 - A

// FOR SCANNING

const float CONFIRMATION_THRESHOLD_GROVE_ETH = 2.0;
const float SLOPE_THRESHOLD = 0.02;
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

// sensor setup
void sensorSetup() {
    unsigned status;

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // grove initialization
    gas.begin(Wire, 0x08);
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
    lcd.begin(); //Defining 16 columns and 2 rows of lcd display
    lcd.clear();
    lcd.backlight(); //To Power ON the back light
    reset();
    Serial.begin(115200);
    sensorSetup();
    motorSetup();
}

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

// main loop
void loop() {
    if (scent_confirmed) {
        /* scent confirmed after detetction and scanning, stop motors */
        lcd.clear(); //Clean the screen
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
            }

            lcd.setCursor(0, 0); 
            lcd.print("ALCOHOL SCAN: ");
            lcd.print(curr_scan);

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
            float val = 0.0;
            int low_detection_count = 0;
            for (int i = 0; i < NUM_SCANS; i++) {
                float slope;
                val = scan_readings[i].gm_eth_v;
                float prev_val = scan_readings[i - 1].gm_eth_v;
                if (i > 0) {
                    slope = (val - prev_val) / (SCAN_TIME);
                    if (slope <= 0.0) {
                        low_detection_count++;
                    }
                }
                if (val > max_val) {
                    max_i = i;
                    max_val = val;
                }
            }
            
            // Reverts back to RANDOM mode if low detection levels
            if (low_detection_count == (NUM_SCANS - 1)) {
                scent_detected = false;
            } 
            else {
                // Set new coordinates in direction of highest concentration
                lcd.print("MAX VALUE AT: ");
                lcd.print(max_i);
                target_angle = state[2] - (scan_angle * (NUM_SCANS - 1 - max_i)) * PI / 180;
                target_X = state[0] + TRANSLATION * cos(target_angle);
                target_Y = state[1] + TRANSLATION * sin(target_angle);
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
        sum_y += y;
        sum_y2 += (y * y);
        sum_xy += (SAMPLING_PERIOD_MS * i * y / 10.0);
    }
   
    slope = (num_readings * sum_xy - sum_x * sum_y) / (num_readings * sum_x2 - sum_x * sum_x);
    return slope;
}

// read from sensor every SAMPLING_PERIOD_MS
void readSensors() {
    // Take timestamp so we can hit our target frequency
    sensor_reading.timestamp = millis();

    if (sensor_reading.timestamp > prevSensorMillis + SAMPLING_PERIOD_MS) {
        
         // Read from GM-X02b sensors (multichannel gas)
        sensor_readings_per_second[sample_count].gm_eth_v = gas.calcVol(gas.getGM302B());
        sensor_readings_per_second[sample_count].gm_voc_v = gas.calcVol(gas.getGM502B());
        sensor_reading.gm_eth_v = gas.calcVol(gas.getGM302B());
        sensor_reading.gm_voc_v = gas.calcVol(gas.getGM502B());

        lcd.setCursor(0, 1);
        lcd.print("E: ");
        lcd.print(sensor_reading.gm_eth_v);
        lcd.setCursor(8, 1);
        lcd.print("V: ");
        lcd.print(sensor_reading.gm_voc_v);

        if ((sensor_reading.gm_eth_v > CONFIRMATION_THRESHOLD_GROVE_ETH) && (scent_detected == true)) {
            scent_confirmed = true;
            scent_detected = false;
        } 
       
        if (sample_count == (SAMPLING_FREQ_HZ - 1)) {
            // find best fit slope of ethanol readings over a period of a second
            float slope = bestFitSlope(sensor_readings_per_second, SAMPLING_FREQ_HZ, 3);
            Serial.println(slope);
            if (slope > SLOPE_THRESHOLD && scent_detected == false) {
                scent_detected = true;
                translation_complete = 1;
                
            }
        }

        // Save values
        
        if (scan_mode && (rotation_complete == 1) && (num_samples < (SCAN_TIME * SAMPLING_FREQ_HZ))) {
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
//    Serial.print("M1_encoder_val: ");
//    Serial.println(M1_encoder_val);
//    Serial.print("M2_encoder_val: ");
//    Serial.println(M2_encoder_val);

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

//    lcd.setCursor(0, 0);
//    lcd.print("X: ");
//    lcd.print(state[0]);
//    lcd.print(" Y: ");
//    lcd.print(state[1]);
//    Serial.print("x: ");
//    Serial.println(state[0]);
//    Serial.print("y: ");
//    Serial.println(state[1]);
//    Serial.print("theta: ");
//    Serial.println(state[2]);

    long duration;
    float distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2.0) / 29.1;

    Serial.print("distance: ");
    Serial.println(distance);

    if (distance < DISTANCE_THRESHOLD) {
        Serial.println("STOP FROM ULTRASONIC");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WALL   DETECTED");
        lcd.setCursor(0, 1);
        lcd.print("BACKING UP...");
        setMotor(STOP, 0, enA, IN1, IN2);
        setMotor(STOP, 0, enB, IN3, IN4);
        delay(500);
        lcd.clear();//Clean the screen
        lcd.setCursor(0, 0); 
        rotation_complete = 0;
        translation_complete = 0;
        target_angle = fmod(state[2] - PI, 2*PI);
        Serial.println("TARGET ANGLE");
        Serial.println(rad2deg(state[2]));
        Serial.println(rad2deg(target_angle));
        target_X = state[0] + TRANSLATION * cos(target_angle);
        Serial.println(target_X);
        target_Y = state[1] + TRANSLATION * sin(target_angle);
        Serial.println(target_Y);
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

// translate from current coordinate to target coordinate
void translation(float current_x, float current_y, float target_x, float target_y) {
    wheelSpeedLeft = 150;
    wheelSpeedRight = 140;
    
    distFromTarget = getDistance(current_x, current_y, target_x, target_y);
    
    if (distFromTarget <= prevDistFromTarget) {
       // Approaching target, continue
       prevDistFromTarget = distFromTarget;
       if (getDistance(current_x, current_y, target_x, target_y) > 0.03) {
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
        Serial.println("HARD STOP: > 5cm away from the target");

        // Corrective action
        prevDistFromTarget = 100;
        rotation_complete = 0; // Reset the turn - rotate and translate again to meet the target
        target_angle = getTheta(current_x, current_y, target_X, target_Y);
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
