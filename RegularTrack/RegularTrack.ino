#include <ECE3.h>

uint16_t sensorValues[8];
const uint16_t MAX_VALUES[8] = { 1911, 1911, 1911, 1911, 1910, 1933, 1888, 1883 };
const uint16_t MIN_VALUES[8] = {  635,  589,  612,  635,  589,  589,  589,  544 };
const int8_t WEIGHTS[8] =      {  -15,  -14,  -12,   -8,    8,   12,   14,   15 };

//Best Drag Track: BASE_SPD = 200, Kd = 1.00, Kp = 0.1
//Best CoA Track: BASE_SPD = 20, Kd = 0.015, Kp = 0.05
const int BASE_SPD = 20;
int error = 0;
int prevError = 0;
int RSpeed;
int LSpeed;
float kp = 0.1;
float kd = 10;
//float ki = 0.000004;

const int left_nslp_pin=31;
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11;
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int YLED_Front_Right=41; //Yellow
const int YLED_Front_Left=51;
// const int RLED_Back_Right=58; //Red
// const int RLED_Back_Left=57;

//TODO
/*
1. How to make it turn around and do a 180* turn
2. Do the speed motor stuff, make direction change.


*/



void setup() {
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  

  //LOW is FORWARD, HIGH is BACK
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH); 
  digitalWrite(right_nslp_pin,HIGH);

  ECE3_Init();
  Serial.begin(9600); 
  delay(2000);

  RSpeed = BASE_SPD;
  LSpeed = BASE_SPD;
}

void loop() {
  error = 0;

  ECE3_read_IR(sensorValues);
  
  int normalizedError = 0;
  int weightedError = 0;
  for(int i = 0; i < 8; i++){
    normalizedError = (1000 * max(sensorValues[i] - MIN_VALUES[i], 0)) / (MAX_VALUES[i] - MIN_VALUES[i]);
    weightedError = (normalizedError * WEIGHTS[i])/8; //Using 15-14-12-8 / 8 Weight Scheme
    error += weightedError;
  }
  //accumlatedError += error;
  float kp_error = error * kp;
  float kd_error = (error - prevError) * kd;
  float pd_sum = kp_error + kd_error;
  //float ki_error = accumlatedError * ki;

  
  kpLED(kp_error); //Might make it slow....
  //kdLED(kd_error); //Might make it slow....

  LSpeed = BASE_SPD - pd_sum;
  RSpeed = BASE_SPD + pd_sum;

  if(RSpeed < 0){
    digitalWrite(right_dir_pin, LOW);
    RSpeed *= -1;
  }else if(LSpeed < 0){
    digitalWrite(left_dir_pin, LOW);
    LSpeed *= -1;
  }

  analogWrite(left_pwm_pin, constrain(LSpeed, 0, 255));
}
