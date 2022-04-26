#include <util/atomic.h>
#include <Adafruit_PS2_Trackpad.h>

// Pins
int ENCA [6] = {0, 22, 24, 26, 28, 30};
int ENCB [6] = {0, 23, 25, 27, 29, 31};
int PWM [6] = {0, 5, 6, 7, 8, 9};
int DIR [6] = {0, 44, 46, 48, 50, 52};
#define PS2_DATA 10
#define PS2_CLK 11

// globals
long prevT = 0;
long prevT_touch = 0;
int posPrev [6] = {0, 0, 0, 0, 0, 0};
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i [6] = {0, 0, 0, 0, 0, 0};
volatile float velocity_i [6] = {0, 0, 0, 0, 0, 0};
volatile long prevT_i = 0;

float vFilt [6] = {0, 0, 0, 0, 0, 0};
float vPrev [6] = {0, 0, 0, 0, 0, 0};

Adafruit_PS2_Mouse ps2(PS2_CLK, PS2_DATA);

int count = 0;
int x = 0;
float vx = 0;
float vxFilt = 0;
float vxPrev = 0;

void setup() {
  ps2.begin();

  for(int i=1; i<6; i++){
    pinMode(ENCA[i],INPUT);
    pinMode(ENCB[i],INPUT);
    pinMode(PWM[i],OUTPUT);
    pinMode(DIR[i],OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(ENCA[1]),
                  readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[2]),
                  readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[3]),
                  readEncoder3,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[4]),
                  readEncoder4,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[5]),
                  readEncoder5,RISING);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos [6] = {0, 0, 0, 0, 0, 0};
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int i=1; i<6; i++){
      pos[i] = pos_i[i];
    }
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
  float velocity [6];
  for(int i=1; i<6; i++){
    velocity[i] = (pos[i]-posPrev[i])/deltaT;
    posPrev[i] = pos[i];
  }
  prevT = currT;

  // Convert count/s to RPM
  float v [6];
  for(int i=1; i<6; i++){
    v[i] = velocity[i]/4776.384*4*60.0;
  }

  // Low-pass filter (25 Hz cutoff)
  for(int i=1; i<6; i++){
    vFilt[i] = 0.854*vFilt[i]+0.0728*v[i]+0.0728*vPrev[i];
    vPrev[i] = v[i];
  }

  // Set a target
  // Low-pass filter (25 Hz cutoff)
  vxFilt = 0.854*vxFilt + 0.0728*vx + 0.0728*vxPrev;
  vxPrev = vx;
  float vt = vxFilt;

  // Compute the control signal u
  float kp = 0.25; // original: 5
  float e [6];
  float u [6];
  for(int i=1; i<6; i++){
    e[i] = vt-vFilt[i];
    u[i] = kp*e[i];
  }

  // Set the motor speed and direction
  int dir [6] = {0, -1, -1, -1, -1, -1};
  for(int i=1; i<6; i++){
    if(u[i]<0){
      dir[i] = 1;
    }
  }
  int maxpwr = 192;
  int pwr [6];
  for(int i=1; i<6; i++){
    pwr[i] = (int) fabs(u[i]);
    if(pwr[i] > maxpwr){
      pwr[i] = maxpwr;
    }
  }

  // send signals to motors
  for(int i=1; i<6; i++){
    setMotor(dir[i], pwr[i], PWM[i], DIR[i]);
  }

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
  int index = 1;
  int b = digitalRead(ENCB[index]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[index] = pos_i[index] + increment;
}

void readEncoder2(){
  // Read encoder B when ENCA rises
  int index = 2;
  int b = digitalRead(ENCB[index]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[index] = pos_i[index] + increment;
}

void readEncoder3(){
  // Read encoder B when ENCA rises
  int index = 3;
  int b = digitalRead(ENCB[index]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[index] = pos_i[index] + increment;
}

void readEncoder4(){
  // Read encoder B when ENCA rises
  int index = 4;
  int b = digitalRead(ENCB[index]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[index] = pos_i[index] + increment;
}

void readEncoder5(){
  // Read encoder B when ENCA rises
  int index = 5;
  int b = digitalRead(ENCB[index]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i[index] = pos_i[index] + increment;
}
