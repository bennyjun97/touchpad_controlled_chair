#include <util/atomic.h>
#include <Adafruit_PS2_Trackpad.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5 // PWM will be 56789
#define IN1 6
#define IN2 7
#define PS2_DATA 10
#define PS2_CLK 11

// globals
long prevT = 0;
long prevT_touch = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;

float eintegral = 0;

Adafruit_PS2_Mouse ps2(PS2_CLK, PS2_DATA);

int count = 0;
int x = 0;
//int y = 0;
float vx = 0;
//float vy = 0;
float vxFilt = 0;
float vxPrev = 0;

void setup() {
  Serial.begin(115200);
  ps2.begin();

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
  Serial.println("Desired, Actual");
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  //float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    //velocity2 = velocity_i;
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
      //y = ps2.y;
      if(x>127){
        x=x-256;
      }
      //if(y>127){
      //  y=y-256;
      //}
    
      vx = x/deltaT;
      //vy = y/deltaT;
    
      prevT_touch = currT;
    }
  }
  count = count + 1;

  // now calculate velocity
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/4776.0*4*60.0;
  //float v2 = velocity2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  //v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  //v2Prev = v2;

  // Set a target
  // float vt = 500*(sin(currT/1e6*3.14)>0);
  // Low-pass filter (25 Hz cutoff)
  vxFilt = 0.854*vxFilt + 0.0728*vx + 0.0728*vxPrev;
  vxPrev = vx;
  float vt = vxFilt;

  Serial.print(vt);
  Serial.print(",");
  Serial.println(v1Filt);

  // Compute the control signal u
  float kp = 0.5; // original: 5
  float ki = 0; // original: 10
  float e = vt-v1Filt; //error
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u)*0.5;
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);

  //Serial.print(vt);
  //Serial.print(" ");
  //Serial.print(v1Filt);
  //Serial.println();
  delayMicroseconds(1000);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  //long currT = micros();
  //float deltaT = ((float) (currT - prevT_i))/1.0e6;
  //velocity_i = increment/deltaT;
  //prevT_i = currT;
}
