#include <Servo.h>
//#include <util/atomic.h>

Servo myservo;
int interruptPin = 2;
int servoPin = 4;

volatile unsigned long val;
volatile unsigned long rising_ts;
volatile bool lock = false;
void ppm_callback(){
  if (lock) {rising_ts = -1; return;}
  lock = true;
  if (digitalRead(interruptPin)==LOW){
    val = (rising_ts==-1)? -1:micros()-rising_ts;
  } else {
    rising_ts = micros();
  }
  lock = false;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(interruptPin,INPUT);
  pinMode(servoPin,OUTPUT);
  myservo.attach(servoPin);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ppm_callback, CHANGE);
}



void loop() {
  // put your main code here, to run repeatedly:
  while(lock);
  lock = true;
  if (val!=-1){
    Serial.println(val);
    //Serial.println(fmap(val,908.0,2140.0,260.0,1260.0));
  }
  //myservo.writeMicroseconds(fmap(val,908.0,2140.0,260.0,1260.0));
  lock = false;
  delay(10);
}
