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

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 50;

long M1_encoder_val = 0;
long M2_encoder_val = 0; 

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const int COUNTS_PER_ROTATION = 14;
const float GEAR_RATIO = 100.0F;
const float WHEEL_DIAMETER = 6.8;
const float WHEEL_CIRCUMFERENCE = 21.36;

float Sl = 0.0F;
float Sr = 0.0F;

// right - M2 - B
// left - M1 - A
 
void setup() {
  Serial.begin(115200);
  
  pinMode(M1_ENCA,INPUT);
  pinMode(M1_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoderM1, RISING);

  pinMode(M2_ENCA,INPUT);
  pinMode(M2_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoderM2, RISING);
  Serial.println("target pos");
}
 
void loop() {
 /*
  // set target position
  int target = 12000;
 
  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;
 
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
 
  // error
  int e = pos-target;
 
  // derivative
  float dedt = (e-eprev)/(deltaT);
 
  // integral
  eintegral = eintegral + e*deltaT;
 
  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;
 
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
 
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
 
  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);
 
  // store previous error
  eprev = e;
 
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  */
  checkEncoders();
}

void checkEncoders() {
  currentMillis = millis();
  if (currentMillis > prevMillis + PERIOD) {
    countsLeft += M1_encoder_val;
    M1_encoder_val = 0;
    countsRight += M2_encoder_val;
    M2_encoder_val = 0;

    Sl += ((countsLeft - prevLeft) / (COUNTS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (COUNTS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    int wheelSpeed = 200;
    if (Sr < 30) {
      setMotor(FORWARD, wheelSpeed, enA, IN1, IN2);
      setMotor(FORWARD, wheelSpeed, enB, IN3, IN4);
    }
    else {
      setMotor(STOP, 0, enA, IN1, IN2);
      setMotor(STOP, 0, enB, IN3, IN4);
    }
   
    Serial.print("Left: ");
    Serial.println(Sl);
    Serial.print("Right: ");
    Serial.println(Sr);
    
    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
  }
}
 
void setMotor(int dir, int pwmVal, int pwm_pin, int in1, int in2){
  analogWrite(pwm_pin, pwmVal);
  if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }  
}
 
void readEncoderM1(){
  int b = digitalRead(M1_ENCB);
  if(b > 0){
    M1_encoder_val++;
  }
  else{
    M1_encoder_val--;
  }
}

void readEncoderM2(){
  int b = digitalRead(M2_ENCB);
  if(b > 0){
    M2_encoder_val++;
  }
  else{
    M2_encoder_val--;
  }
}
