#include <util/atomic.h>
#include <Adafruit_PS2_Trackpad.h>

// Pins
#define ENCA1 22
#define ENCB1 23
#define ENCA2 24
#define ENCB2 25
#define ENCA3 26
#define ENCB3 27
#define ENCA4 28
#define ENCB4 29
#define ENCA5 30
#define ENCB5 31

#define PWM1 5
#define PWM2 6
#define PWM3 7
#define PWM4 8
#define PWM5 9

#define DIR1 44
#define DIR2 46
#define DIR3 48
#define DIR4 50
#define DIR5 52

#define PS2_DATA 10
#define PS2_CLK 11

// globals
long prevT = 0;
long prevT_touch = 0;
int posPrev1 = 0;
int posPrev2 = 0;
int posPrev3 = 0;
int posPrev4 = 0;
int posPrev5 = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i1 = 0;
volatile int pos_i2 = 0;
volatile int pos_i3 = 0;
volatile int pos_i4 = 0;
volatile int pos_i5 = 0;
volatile float velocity_i1 = 0;
volatile float velocity_i2 = 0;
volatile float velocity_i3 = 0;
volatile float velocity_i4 = 0;
volatile float velocity_i5 = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float v3Filt = 0;
float v3Prev = 0;
float v4Filt = 0;
float v4Prev = 0;
float v5Filt = 0;
float v5Prev = 0;

Adafruit_PS2_Mouse ps2(PS2_CLK, PS2_DATA);

int count = 0;
int x = 0;
float vx = 0;
float vxFilt = 0;
float vxPrev = 0;

void setup() {
  ps2.begin();

  pinMode(ENCA1,INPUT);
  pinMode(ENCB1,INPUT);
  pinMode(ENCA2,INPUT);
  pinMode(ENCB2,INPUT);
  pinMode(ENCA3,INPUT);
  pinMode(ENCB3,INPUT);
  pinMode(ENCA4,INPUT);
  pinMode(ENCB4,INPUT);
  pinMode(ENCA5,INPUT);
  pinMode(ENCB5,INPUT);
  
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);
  pinMode(PWM5,OUTPUT);
  
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(DIR3,OUTPUT);
  pinMode(DIR4,OUTPUT);
  pinMode(DIR5,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA1),
                  readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2),
                  readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3),
                  readEncoder3,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA4),
                  readEncoder4,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA5),
                  readEncoder5,RISING);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos1 = 0;
  int pos2 = 0;
  int pos3 = 0;
  int pos4 = 0;
  int pos5 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos1 = pos_i1;
    pos2 = pos_i2;
    pos3 = pos_i3;
    pos4 = pos_i4;
    pos5 = pos_i5;
  }

  // Compute velocity with method 1
  long currT = micros();

  // touchpad stuff
  if(count >= 5){
    count = 0;
  }
  if(count == 0){
    if(ps2.readData()){
      float deltaT = ((float)(currT-prevT_touch))/1.0e6;
      x = ps2.x;
      if(x>127){
        x=x-256;
      }
    
      vx = x/deltaT;
    
      prevT_touch = currT;
    }
  }
  count = count + 1;

  // now calculate velocity
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos1 - posPrev1)/deltaT;
  float velocity2 = (pos2 - posPrev2)/deltaT;
  float velocity3 = (pos3 - posPrev3)/deltaT;
  float velocity4 = (pos4 - posPrev4)/deltaT;
  float velocity5 = (pos5 - posPrev5)/deltaT;
  posPrev1 = pos1;
  posPrev2 = pos2;
  posPrev3 = pos3;
  posPrev4 = pos4;
  posPrev5 = pos5;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/4776.0*4*60.0;
  float v2 = velocity2/4776.0*4*60.0;
  float v3 = velocity3/4776.0*4*60.0;
  float v4 = velocity4/4776.0*4*60.0;
  float v5 = velocity5/4776.0*4*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;
  v3Filt = 0.854*v3Filt + 0.0728*v3 + 0.0728*v3Prev;
  v3Prev = v3;
  v4Filt = 0.854*v4Filt + 0.0728*v4 + 0.0728*v4Prev;
  v4Prev = v4;
  v5Filt = 0.854*v5Filt + 0.0728*v5 + 0.0728*v5Prev;
  v5Prev = v5;

  // Set a target
  // Low-pass filter (25 Hz cutoff)
  vxFilt = 0.854*vxFilt + 0.0728*vx + 0.0728*vxPrev;
  vxPrev = vx;
  float vt = vxFilt;

  // Compute the control signal u
  float kp = 0.5; // original: 5
  float e1 = vt-v1Filt; //error
  float e2 = vt-v2Filt;
  float e3 = vt-v3Filt;
  float e4 = vt-v4Filt;
  float e5 = vt-v5Filt;
  
  float u1 = kp*e1;
  float u2 = kp*e2;
  float u3 = kp*e3;
  float u4 = kp*e4;
  float u5 = kp*e5;

  // Set the motor speed and direction
  int dir1 = -1;
  int dir2 = -1;
  int dir3 = -1;
  int dir4 = -1;
  int dir5 = -1;
  if (u1<0){
    dir1 = 1;
  }
  if (u2<0){
    dir2 = 1;
  }
  if (u3<0){
    dir3 = 1;
  }
  if (u4<0){
    dir4 = 1;
  }
  if (u5<0){
    dir5 = 1;
  }
  int maxpwr = 192;
  int pwr1 = (int) fabs(u1)*0.5;
  if(pwr1 > maxpwr){
    pwr1 = maxpwr;
  }
  int pwr2 = (int) fabs(u2)*0.5;
  if(pwr2 > maxpwr){
    pwr2 = maxpwr;
  }
  int pwr3 = (int) fabs(u3)*0.5;
  if(pwr3 > maxpwr){
    pwr3 = maxpwr;
  }
  int pwr4 = (int) fabs(u4)*0.5;
  if(pwr4 > maxpwr){
    pwr4 = maxpwr;
  }
  int pwr5 = (int) fabs(u5)*0.5;
  if(pwr5 > maxpwr){
    pwr5 = maxpwr;
  }
  setMotor(dir1,pwr1,PWM1,DIR1);
  setMotor(dir2,pwr2,PWM2,DIR2);
  setMotor(dir3,pwr3,PWM3,DIR3);
  setMotor(dir4,pwr4,PWM4,DIR4);
  setMotor(dir5,pwr5,PWM5,DIR5);

  delayMicroseconds(1000);
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
  }
}

void readEncoder1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB1);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i1 = pos_i1 + increment;
}

void readEncoder2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB2);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i2 = pos_i2 + increment;
}

void readEncoder3(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB3);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i3 = pos_i3 + increment;
}

void readEncoder4(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB4);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i4 = pos_i4 + increment;
}

void readEncoder5(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB5);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i5 = pos_i5 + increment;
}
