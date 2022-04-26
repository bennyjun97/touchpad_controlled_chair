#include <Adafruit_PS2_Trackpad.h>

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

void setup() {
  Serial.begin(9600);
  if (ps2.begin()) 
    Serial.println("Successfully found PS2 mouse device");
  else
    Serial.println("Did not find PS2 mouse device");
  
  Serial.print("PS/2 Mouse with ID 0x");
  Serial.println(ps2.readID(), HEX);

}

uint16_t lasttap_x = 0, lasttap_y = 0;
long prevT = 0;

void loop() {
  delay(5);

  if (! ps2.readData()) return;
  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  int x = ps2.x;
  int y = ps2.y;
  if(x>127){
    x=x-256;
  }
  if(y>127){
    y=y-256;
  }

  float vx = x/deltaT;
  float vy = y/deltaT;

  prevT = currT;

  //Serial.print("0x");
  //Serial.print(ps2.status, HEX); Serial.print("\t");
  Serial.print("X = "); Serial.print(vx, DEC); Serial.print("\t");
  Serial.print("Y = "); Serial.print(vy, DEC); Serial.print("\t");
  Serial.print("Z = "); Serial.print(ps2.z); 
  //if (ps2.right) Serial.print("\tRight");
  //if (ps2.middle) Serial.print("\tMiddle");
  //if (ps2.left) Serial.print("\tLeft");
  
  Serial.println();
  delay(5);
}
