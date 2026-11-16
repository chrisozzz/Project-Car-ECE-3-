#include <ECE3.h>
#define TRACK_COTA 1
#define TRACK_DRAG 2
#define SELECTED_TRACK TRACK_DRAG

uint16_t sensorValues[8];
const uint16_t MAX_VALUES[8] = { 1911, 1911, 1911, 1911, 1910, 1933, 1888, 1883 };
const uint16_t MIN_VALUES[8] = {  635,  589,  612,  635,  589,  589,  589,  544 };
const int8_t WEIGHTS[8] =      {  -15,  -14,  -12,   -8,    8,   12,   14,   15 };

#if SELECTED_TRACK == TRACK_COTA
const float kp = 0.05;
const float kd = 0.015;
const int BASE_SPD = 20;
#elif SELECTED_TRACK == TRACK_DRAG
const float kp = 0.1;
const float kd = 1.00;
const int BASE_SPD = 200;
#endif

int setBaseSpeed(int);
void moveCar(int, int);
void turn180();
void kpLED(float);
void kdLED(float);

//Best Drag Track: BASE_SPD = 200, Kd = 1.00, Kp = 0.1
//Best CoA Track: BASE_SPD = 20, Kd = 0.015, Kp = 0.05
// const int BASE_SPD = 20;
const int MAX_SPD = 255;
int prevError = 0;
int error = 0;
int RSpeed;
int LSpeed;
// float kp = 0.1;
// float kd = 10;
bool trackEndedTriggered = false;
bool trackEndedReal = false;

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

//Bump pins not used.... yet
const int BUMP2_pin = 6;
const int BUMP3_pin = 27;
const int BUMP4_pin = 8;
const int BUMP5_pin = 28;
const int BUMP0_pin = 24;
const int BUMP1_pin = 25;



/* TODO
1. How to make it turn around and do a 180* turn
    a. there is a wide black line where that indicates the end.
    b. make that turn around condition, if avg sensors is high then turn around.

2. Do the speed motor stuff, make direction change.

3. Change base speed based on error?
    ie. small error increase
    large error : decrease


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
  int endMarkerError = 0;
  for(int i = 0; i < 8; i++){
    normalizedError = (1000 * max(sensorValues[i] - MIN_VALUES[i], 0)) / (MAX_VALUES[i] - MIN_VALUES[i]);
    endMarkerError += normalizedError;
    weightedError = (normalizedError * WEIGHTS[i])/8; //Using 15-14-12-8 / 8 Weight Scheme
    error += weightedError;
  }
  /*
  PSEUDO CODE:
  if( all black ){ somehow check error?
    if( first trigger = true ){
      second trigger = true; (We know its the end now)
    } else {
      first trigger = true;
    }
  }

  ACTUAL CODE PROTOTYPE:
  if((endMarkerError / 8) >= 800){ --> Average of all sensors > 800 (magic number) (do testing to find proper val)
    if(trackEndTriggered){
      trackEndReal = true;
    }else{
      trackEndTriggered = true;
      redLED();
    }
  }
  */

  float kp_error = error * kp;
  float kd_error = (error - prevError) * kd;
  
  float pd_sum = kp_error + kd_error;

  
  kpLED(kp_error); //Might make it slow....
  
  LSpeed = BASE_SPD - pd_sum;
  RSpeed = BASE_SPD + pd_sum;

  moveCar(RSpeed, LSpeed);
  /*if(trackEndReal){
      180turn();
      moveCar(BASE_SPD, BASE_SPD);

      trackEndTriggered = false;
      trackEndReal = false;
      }else{
        moveCar(RSpeed, LSpeed);
      }
  */
  prevError = error;
}

void moveCar(int RSpeed, int LSpeed){
  if(RSpeed < 0){
    digitalWrite(right_dir_pin, LOW);
  }else{
    digitalWrite(right_dir_pin,HIGH);
  }
  
  if(LSpeed < 0){
    digitalWrite(left_dir_pin, LOW);
  }else{
    digitalWrite(left_dir_pin,HIGH);
  }

  analogWrite(left_pwm_pin, constrain(abs(LSpeed), 0, 255));
  analogWrite(right_pwm_pin, constrain(abs(RSpeed), 0, 255));
}

void turn180(){
  delay(500); //just a magic # delay
  // @ 255 speed, 82 encoder counts / bin
  // bin length = 50 ms.
  // encoder count / revolution = 360 <-- Experimentally determined
  int leftEncoderCount = 0;
  resetEncoderCount_left();
  resetEncoderCount_right();

  while(leftEncoderCount < 360){ //MAY NEED TO TWEAK NUMBER to account for slipping, deceleration, etc...
    moveCar(MAX_SPD, -MAX_SPD);
    leftEncoderCount = getEncoderCount_left();
  }
}

int setBaseSpeed(int error){
  int e = abs(error);

  /* GPT Example:
  if (e < 15) return 150;
  if (e < 40) return 110;
  if (e < 80) return 75;

  return 50;
  */
  return 0;
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

void redLED(){
  static bool isOn = true;
  if(isOn){
    digitalWrite(RLED_Back_Left, LOW);
    isOn = false;
  }else{
    digitalWrite(RLED_Back_Left, HIGH);
    isOn = true;
  }
}

