int led_pin = 5;
void setup() {
  // put your setup code here, to run once:
  //Declaring LED pin as output
  pinMode(led_pin, OUTPUT);
  analogWrite(led_pin, 10);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Fading the LED
  for(int i=50; i<100; i++){
    analogWrite(led_pin, i);
    delay(60);
  }
  for(int i=70; i<140; i++){
    analogWrite(led_pin, i);
    delay(60);
  }
  for(int i=90; i<180; i++){
    analogWrite(led_pin, i);
    delay(60);
  }
  for(int i=110; i<220; i++){
    analogWrite(led_pin, i);
    delay(50);
  }
  for(int i=130; i<255; i++){
    analogWrite(led_pin, i);
    delay(40);
  }
}
