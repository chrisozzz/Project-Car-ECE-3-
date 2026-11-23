#include <ECE3.h>
#define TRACK_COTA 1
#define TRACK_DRAG 2
#define SELECTED_TRACK TRACK_COTA

uint16_t sensorValues[8];
const uint16_t MAX_VALUES[8] = { 1911, 1911, 1911, 1911, 1910, 1933, 1888, 1883 };
const uint16_t MIN_VALUES[8] = {  635,  589,  612,  635,  589,  589,  589,  544 };
const int8_t WEIGHTS[8] =      {  -15,  -14,  -12,   -8,    8,   12,   14,   15 };
float Variable_Store[2000][2];
int arrCounter = 0;

// Kp is for TURNS
// Kd is for WOBBLE

#if SELECTED_TRACK == TRACK_COTA
const float kp = 0.065;
const float kd = 0.3; //decrease kd a bit more
const int BASE_SPD = 100;

#elif SELECTED_TRACK == TRACK_DRAG
const float kp = 0.1;
const float kd = 1.00;
const int BASE_SPD = 200;
#endif

int setBaseSpeed(int);
void moveCar(int, int);
void turn180();
void kpLED(float);
void redLED();

const int MAX_SPD = 255;
float prevError = 0;
float error = 0;
int RSpeed;
int LSpeed;
int curr_speed = 100;
bool trackEndTriggered = false;
bool insideEnd = false;


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

/* TODO
1. How to make it turn around and do a 180* turn
    a. there is a wide black line where that indicates the end.
    b. make that turn around condition, if avg sensors is high then turn around. //DONE
    c. turn around 180 // DONE

3. Change base speed based on error?
    ie. small error increase
    large error : decrease
*/

/*
Order to Implement:
1. Fine Tune kd & kp
2. Turn around at end
3. Variable Base Speed

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
  pinMode(PUSH2, INPUT_PULLUP);
  
  ECE3_Init();
  Serial.begin(9600); 
  delay(2000);

  RSpeed = BASE_SPD;
  LSpeed = BASE_SPD;
}

void loop() {
  ECE3_read_IR(sensorValues);
  
  float normalizedError = 0;
  float weightedError = 0;
  float endMarkerError = 0;
  for(int i = 0; i < 8; i++){
    normalizedError = (1000 * max(sensorValues[i] - MIN_VALUES[i], 0)) / (float)(MAX_VALUES[i] - MIN_VALUES[i]);
    endMarkerError += abs(normalizedError);
    weightedError = (normalizedError * WEIGHTS[i])/8; //Using 15-14-12-8 / 8 Weight Scheme
    error += weightedError;
  }
  //The minimum error is -19.0 for some reason. Might need to redo sensor fusion?

  // Not really helpful
  if(arrCounter < 2000){
    Variable_Store[arrCounter][0] = error;
    Variable_Store[arrCounter][1] = getBaseSpeed();
    arrCounter++;
  }

  if(endMarkerError >= 11000){ //End Marker Threshold; why 11,000? idk
    if(trackEndTriggered && insideEnd){
      digitalWrite(RLED_Back_Left, HIGH);
      moveCar(0,0);
      turn180();
      moveCar(BASE_SPD, BASE_SPD);
      trackEndTriggered = false;
    }else if(!insideEnd){
      trackEndTriggered = true;
      insideEnd = true;
      digitalWrite(RLED_Back_Right, HIGH);
    }
  }else{
    digitalWrite(RLED_Back_Right, LOW);
    digitalWrite(RLED_Back_Left, LOW);
    trackEndTriggered = false;
    insideEnd = false;
  }

  float kp_error = error * kp;
  float kd_error = (error - prevError) * kd;
  float pd_sum = kp_error + kd_error;
  kpLED(kp_error);
  

  LSpeed = BASE_SPD - pd_sum; // change to getBaseSpeed() - pd_sum
  RSpeed = BASE_SPD + pd_sum; // change to getBaseSpeed() + pd_sum
  //Need to capture data in turns and straights, need to make capture code
  //Bascially just copy the code from the "Advanced Project Tips" slides...
  
  if(!digitalRead(PUSH1)){
    readData();
  }

  moveCar(RSpeed, LSpeed);
  prevError = error;
  error = 0; 
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
  //delay(50); //just a magic # delay; can decrease to save time;

  int leftEncoderCount = 0;
  resetEncoderCount_left();
 
  while(abs(leftEncoderCount) < 305){
    moveCar(MAX_SPD, -MAX_SPD);
    leftEncoderCount = getEncoderCount_left();
  }
  moveCar(0,0);
}

int getBaseSpeed(){
  //abs(error) on the range 0 - 10000
  int e = abs(error);

  if (e < 700) curr_speed += 10;
  if (e < 3000) curr_speed += 2;
  if (e < 4000) curr_speed += -2;
  if (e < 8000) curr_speed += -5;
  // if (e < 3000) return 70;
  // if (e < 5000) return 40;
  // if (e < 8000) return 30;
  Serial.println(curr_speed);
  
  if(curr_speed > 240){
    curr_speed = 240;
  }else if(curr_speed < 0){
    curr_speed = 0;
  }
  
  return curr_speed;
}

void readData(){
  moveCar(0,0);
  while(true){
    if(!digitalRead(PUSH2)){
      for(int i = 0; i < 5000; i++){
        Serial.print(Variable_Store[i][0]);Serial.print("\t\t");
        Serial.print(Variable_Store[i][1]);Serial.print("\t\t");
        Serial.print(10000);Serial.print("\t");
        Serial.println(-200);
      }
    }
  }
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

