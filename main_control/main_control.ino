#include <util/atomic.h>
#include <Adafruit_PS2_Trackpad.h>
#include <math.h>

// Pins
int ENCA [6] = {0, 22, 24, 26, 28, 30}; // encoder A 
int ENCB [6] = {0, 23, 25, 27, 29, 31}; // encoder B
int PWM [6] = {0, 5, 6, 7, 8, 9}; // PWM signal to control speed
int DIR [6] = {0, 44, 46, 48, 50, 52}; // direction for motors
#define PS2_DATA 10 // data from touchpad
#define PS2_CLK 11 // clock for touchpad

// globals
#define PI 3.14159265
long prevT = 0; // previous time for motor updates
long prevT_touch = 0; // previous time for touchpad updates
int posPrev [6] = {0, 0, 0, 0, 0, 0}; // previous position of encoder
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i [6] = {0, 0, 0, 0, 0, 0};
volatile float velocity_i [6] = {0, 0, 0, 0, 0, 0};
volatile long prevT_i = 0;
float vFilt [6] = {0, 0, 0, 0, 0, 0};
float vPrev [6] = {0, 0, 0, 0, 0, 0};

// paramters for motion
int touchpad_count = 1;
int count_to_rpm = 1/4776.384*4*60.0; // depends on motor resolution
float maxv = 60.0; // in RPM, should not be close to 84
float kp = 10.0; // proportional gain for P control
long unittime = 20000; // time for the loop in microseconds
float gain = 0.035; // gain from finger velocity to motor velocity
int maxpwr = 192; // 255 at max, smaller if battery V > motor V
float accrate = 240.0; // in RPM/s
float decrate = 120.0; // in RPM/s
float hardstoprate = 600.0; // in RPM/s
float unitacc = accrate*unittime/10e6; // acceleration in RPM per unit time
float unitdec = decrate*unittime/10e6; // deceleration in RPM per unit time
float uniths = hardstoprate*unittime/10e6; // hard stop rate in RPM per unit time

// creating the touchpad object
Adafruit_PS2_Mouse ps2(PS2_CLK, PS2_DATA);

// initializing variables
int count = 0;
int x = 0;
int y = 0;
float vx = 0;
float vy = 0;
float vxFilt = 0;
float vyFilt = 0;
float vxPrev = 0;
float vyPrev = 0;
float vxf = 0;
float vyf = 0;
float vxd = 0;
float vyd = 0;
float vxgoal = 0;
float vygoal = 0;
float vxlim = 0;
float vylim = 0;

void setup() {
  ps2.begin();
  
  // set pin modes for motors
  for(int i=1; i<6; i++){
    pinMode(ENCA[i],INPUT);
    pinMode(ENCB[i],INPUT);
    pinMode(PWM[i],OUTPUT);
    pinMode(DIR[i],OUTPUT);
  }

  // interrupt from encoder A of motor X
  // will call the function readEncoderX
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

  // measure current time
  long currT = micros();

  // touchpad signal
  if(count >= touchpad_count){
    count = 0;
  }
  if(count == 0){
    if(ps2.readData()){
      float deltaT = ((float)(currT-prevT_touch))/1.0e6;
      x = ps2.x;
      y = ps2.y;
      // x and y values exceeding 127 means that the finger moved
      // in negative direction
      if(x>127){
        x=x-256;
      }
      if(y>127){
        y=y-256;
      }
      vx = x/deltaT; // calculate x velocity
      vy = y/deltaT; // calculate y velocity
    
      prevT_touch = currT;
    }
  }
  count = count + 1;

  // now calculate current velocity of the motors
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
    v[i] = velocity[i]*count_to_rpm;
  }

  // Low-pass filter for motor velocities (25 Hz cutoff)
  for(int i=1; i<6; i++){
    vFilt[i] = 0.854*vFilt[i]+0.0728*v[i]+0.0728*vPrev[i];
    vPrev[i] = v[i];
  }

  // Set a target
  // Low-pass filter for touchpad signal (25 Hz cutoff)
  vxFilt = 0.854*vxFilt + 0.0728*vx + 0.0728*vxPrev;
  vxPrev = vx;
  vyFilt = 0.854*vyFilt + 0.0728*vy + 0.0728*vyPrev;
  vyPrev = vy;

  // multiply by gain
  float vxf = vxFilt*gain;
  float vyf = vyFilt*gain;

  // update goal
  if(fabs(vxf) > fabs(vxgoal) || vxf*vxgoal < 0){
    vxgoal = vxf;
  }
  if(fabs(vyf) > fabs(vygoal) || vyf*vygoal < 0){
    vygoal = vyf;
  }

  if(vxgoal*vxd < 0){ // hard stop for x
    if(fabs(vxd) > uniths){
      if(vxd > 0) vxd -= uniths;
      else vxd += uniths;
    }
    else vxd = 0;
  }
  else if(fabs(vxgoal) > fabs(vxd)){ // slow acceleration for x
    if(fabs(vxgoal-vxd) > unitacc){
      if(vxgoal > 0) vxd += unitacc;
      else vxd -= unitacc;
    }
    else{
      vxd = vxgoal;
      vxgoal = 0; // initialize vxgoal
    }
  }
  else{ // slow deceleration for x
    if(fabs(vxd) > unitdec){
      if(vxd > 0) vxd -= unitdec;
      else vxd += unitdec;
    }
    else vxd = 0;
  }

  // same thing for y
  if(vygoal*vyd < 0){ // hard stop for y
    if(fabs(vyd) > uniths){
      if(vyd > 0) vyd -= uniths;
      else vyd += uniths;
    }
    else vyd = 0;
  }
  else if(fabs(vygoal) > fabs(vyd)){ // slow acceleration for y
    if(fabs(vygoal-vyd) > unitacc){
      if(vygoal > 0) vyd += unitacc;
      else vyd -= unitacc;
    }
    else{
      vyd = vygoal;
      vygoal = 0; // initialize vygoal
    }
  }
  else{ // slow deceleration for y
    if(fabs(vyd) > unitdec){
      if(vyd > 0) vyd -= unitdec;
      else vyd += unitdec;
    }
    else vyd = 0;
  }
  
  
  // limits vx and vy with same ratio
  float overratio = sqrt(vxd*vxd+vyd*vyd)/maxv;
  if(overratio > 1.0){
    vxlim = vxd/overratio;
    vylim = vyd/overratio;
  }
  else{
    vxlim = vxd;
    vylim = vyd;
  }

  float vdesired [6];
  float angles [6] = {0, PI*0.2, PI*0.6, PI*1.0, PI*1.4, PI*1.8};
  // calculate desired velocity for each motors
  for(int i=1; i<6; i++){
    vdesired[i] = vxlim*cos(angles[i])+vylim*sin(angles[i]);
  }

  // Compute the control signal u, which decides the voltages through
  // the motors
  float e [6];
  float u [6];
  for(int i=1; i<6; i++){
    e[i] = vdesired[i]-vFilt[i];
    u[i] = kp*e[i];
  }

  // Set the motor speed and direction
  int dir [6] = {0, -1, -1, -1, -1, -1};
  for(int i=1; i<6; i++){
    if(u[i]<0){
      dir[i] = 1;
    }
  }
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
  long delaytime = unittime-(micros()-currT);
  if(delaytime > 0){
    delayMicroseconds(delaytime);
  }
  else{
    for(int i=1; i<6; i++){
      setMotor(dir[i], 0, PWM[i], DIR[i]);
    }
    delay(10000);
    // stop for 10 seconds. if this happens, increase the unittime!
  }
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

// Interrupt Service Routine Functions
// They cannot have parameters, so 5 functions are written separately

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
