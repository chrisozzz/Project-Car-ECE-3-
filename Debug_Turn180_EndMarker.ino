#include <ECE3.h>

uint16_t sensorValues[8];
const uint16_t MAX_VALUES[8] = { 1911, 1911, 1911, 1911, 1910, 1933, 1888, 1883 };
const uint16_t MIN_VALUES[8] = {  635,  589,  612,  635,  589,  589,  589,  544 };
const int8_t WEIGHTS[8] =      {  -15,  -14,  -12,   -8,    8,   12,   14,   15 };
int counter = 0;

//Best Drag Track: BASE_SPD = 200, Kd = 1.00, Kp = 0.1
//Best CoA Track: BASE_SPD = 20, Kd = 0.015, Kp = 0.05
const int BASE_SPD = 20;
int error = 0;
int prevError = 0;
int RSpeed;
int LSpeed;
float kp = 0.1;
float kd = 10;
const int MAX_SPD = 255;

const int left_nslp_pin=31;
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11;
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int YLED_Front_Right=41; //Yellow
const int YLED_Front_Left=51;
const int RLED_Back_Right=58; //Red
const int RLED_Back_Left=57;

bool trackEndTriggered = false;

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


  pinMode(PUSH1, INPUT_PULLUP);
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
  int totalNormError = 0;
  for(int i = 0; i < 8; i++){
    normalizedError = (1000 * max(sensorValues[i] - MIN_VALUES[i], 0)) / (MAX_VALUES[i] - MIN_VALUES[i]);
    totalNormError += abs(normalizedError) * 1000/1452;
    weightedError = (normalizedError * WEIGHTS[i])/8; //Using 15-14-12-8 / 8 Weight Scheme
    error += weightedError;
  }
  Serial.print(totalNormError / 8);Serial.print("\t");
  Serial.print(1000); Serial.print("\t");
  Serial.println(-200);


  if(checkEnd(totalNormError / 8)){
    if(trackEndTriggered){
      digitalWrite(RLED_Back_Left, HIGH);
      turn180();
      moveCar(BASE_SPD, BASE_SPD);
    }else{
      trackEndTriggered = true;
      digitalWrite(RLED_Back_Right, HIGH);
    }
  }else{
    digitalWrite(RLED_Back_Right, LOW);
    digitalWrite(RLED_Back_Left, LOW);
    trackEndTriggered = false;
  }
}

void moveCar(int RSpeed, int LSpeed){
  
  if(RSpeed < 0){
    digitalWrite(right_dir_pin, HIGH);
  }else{
    digitalWrite(right_dir_pin,LOW);
  }
  
  if(LSpeed < 0){
    digitalWrite(left_dir_pin, HIGH);
  }else{
    digitalWrite(left_dir_pin,LOW);
  }

  analogWrite(left_pwm_pin, constrain(abs(LSpeed), 0, 255));
  analogWrite(right_pwm_pin, constrain(abs(RSpeed), 0, 255));
}

void turn180(){
  delay(500); //just a magic # delay

  int leftEncoderCount = 0;
  resetEncoderCount_left();
  //305 experimentally determined to be most accurate @ 255 spd, 9 V
  // Might need to increase for less Volts.
  while(abs(leftEncoderCount) < 305){
    moveCar(MAX_SPD, -MAX_SPD);
    leftEncoderCount = getEncoderCount_left();
  }
  moveCar(0,0);
}

bool checkEnd(int e){
  return (e >= 800);
}


void kpLED(float kp_error){
  if(kp_error > 0){
    digitalWrite(YLED_Front_Right, HIGH);
    digitalWrite(YLED_Front_Left, LOW);
  }else if(kp_error < 0){
    digitalWrite(YLED_Front_Left, HIGH);
    digitalWrite(YLED_Front_Right, LOW);
  }
}