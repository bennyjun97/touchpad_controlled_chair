#define ENCA 2
#define ENCB 3

volatile int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pos);
}

void readEncoder(){
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
  pos = pos + increment;
}
