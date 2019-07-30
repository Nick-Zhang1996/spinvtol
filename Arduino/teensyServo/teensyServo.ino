// test routine for PPM input and servo control (high freq tail servo)
// Nick Zhang Summer 2019
// for Teensy 3.5

// Uses PDB timer
#include <PWMServo.h>

PWMServo myservo;

// Teensy 3.5 usable pins 2-10, 14, 20-23, 29-30, 35-38
int interruptPin = 2;
int servoPin = 4;

volatile unsigned long val;
volatile unsigned long pending_val;
volatile unsigned long rising_ts;
volatile bool lock = false;
void ppm_callback(){
  if (lock) {rising_ts = -1; return;}
  lock = true;
  if (digitalRead(interruptPin)==LOW){
    pending_val = (rising_ts==(uint8_t)-1)? -1:micros()-rising_ts;
    if (pending_val<3000 and pending_val>500){
      val = pending_val;
    }
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
  myservo.attach(servoPin,560,930);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ppm_callback, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(lock);
  lock = true;
  if (val!=(uint8_t)-1){
    Serial.println(val);
    //Serial.println(fmap(val,908.0,2140.0,560.0,930.0);
  }
  myservo.writeMicroseconds(fmap(val,908.0,2140.0,560.0,930.0));
  lock = false;
  delay(10);
}
