#include <Adafruit_PS2_Trackpad.h>
#include <movingAvg.h>

/*************************************************** 
  This is an example for the Adafruit Capacitive Trackpad

  Designed specifically to work with the Adafruit Capacitive Trackpad 
  ----> https://www.adafruit.com/products/837

  These devices use PS/2 to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
/*
We suggest using a PS/2 breakout adapter such as this
https://www.adafruit.com/products/804
for easy wiring!

PS/2 pin #1 (Brown)  - Data 
PS/2 pin #2 (White)  - N/C
PS/2 pin #3 (Black)  - Ground
PS/2 pin #4 (Green)  - 5V
PS/2 pin #5 (Yellow) - Clock
PS/2 pin #6 (Red)    - N/C
*/

// PS2 uses two digital pins
#define PS2_DATA 10
#define PS2_CLK 11

//Adafruit_PS2 ps2(PS2_CLK, PS2_DATA);

// Use this declaration when you want the trackpad to act like a 'mouse'
// with relative positioning
Adafruit_PS2_Mouse ps2(PS2_CLK, PS2_DATA);

// Use this declaration when you want 'absolute' tablet mode
//Adafruit_PS2_Trackpad ps2(PS2_CLK, PS2_DATA);
int interval = 5;
int vxFilt;
int vyFilt;
int decayRate = 50;
movingAvg filterX(interval);
movingAvg filterY(interval);

void setup() {
  Serial.begin(9600);
  Serial.println("adsfasfasef");
  if (ps2.begin()) 
    Serial.println("Successfully found PS2 mouse device");
  else
    Serial.println("Did not find PS2 mouse device");
  
  Serial.print("PS/2 Mouse with ID 0x");
  Serial.println(ps2.readID(), HEX);
  int cycles = 0;
  filterX.begin();
  filterY.begin();
}

uint16_t lasttap_x = 0, lasttap_y = 0;
long prevT = 0;
int cycles;

void loop() {
  if (! ps2.readData()){
    return;
  }
  long currT = micros();
  cycles += 1;
  float deltaT = ((float)(currT-prevT))/1.0e6;
  int x = ps2.x;
  int y = ps2.y;
  if(x>127){
    x=x-256;
  }
  if(y>127){
    y=y-256;
  }

  int vx = (int)(x/deltaT);
  int vy = (int)(y/deltaT);

  prevT = currT;
 
  if(vx!=0){
    vxFilt = filterX.reading(vx);
  }
  else if(vxFilt>=decayRate){
    vxFilt -= decayRate;
  }
  else{
    vxFilt=0;
  }
  vyFilt = filterY.reading(vy);

  Serial.print(vxFilt, DEC); Serial.print("\t");
  Serial.print(vyFilt, DEC); Serial.print("\t");
  Serial.println();
  delay(10);
}
