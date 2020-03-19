/*
  Rotary Encoder Test
  Demonstrates operation of Rotary Encoder
  Displays results on Serial Monitor
*/
// Rotary Encoder Inputs
#define inputCLK 2
#define inputDT 5
#define Motor 6

#define tick 32.727272727272727
#define pi 3.14159265358979323846264338

void setup() { 
   
 // Set encoder pins as inputs  
  pinMode (inputCLK,INPUT);
  pinMode (inputDT,INPUT);
  pinMode (Motor,OUTPUT);
  
  // Setup Serial Monitor
  Serial.begin (9600);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(inputCLK), encoder, RISING);

  Serial.println("Set up completed");
} 

double rev = 0, temp_rev = 0, old_rev = 0, old_temp_rev = 0;
double elapsed_time = 0, angle = 0, target = 0;
long times[2] = {0, 0};
int p = 0, i = 0, d = 0, spd = 0;
String str; 

void encoder() {
  times[1] = millis();
  elapsed_time = (double)(times[1]-times[0])/1000;
  times[0] = times[1];
  
  old_temp_rev = temp_rev;
  temp_rev = tick/360.0/elapsed_time; // Simple Average filter
  
  // Discard unrealistic revs
  if (temp_rev>12)
    temp_rev = old_temp_rev;

  // Avg filter
  rev = (4*rev + temp_rev)/5;

  angle += tick;
  if (angle >= 360)
    angle -= 360;

  // Control
  if (target == 0){
    spd = 0;
    analogWrite(Motor, 0);
    return;
  }
  
  // PID
  p = (target-temp_rev)*800;
  d = (old_temp_rev-temp_rev)/elapsed_time*120;
  if (d>80)
    d = 80;
  else if (d<-80)
    d = -80;
  if (spd < 254 && spd > 0) // anti wind-up
    i += (target-temp_rev)*elapsed_time*70;

  spd = p + i + d;

  if (spd > target * 140 + 50)
    spd = target * 140 + 50;
  if (target != 0 && spd < 75)
    spd = 75;
  else if (spd > 254)
    spd = 254;
    
  // analogWrite(Motor, spd);
}

void loop() {

  analogWrite(Motor, spd);
  
   if (Serial.available() > 0){
     str = Serial.readString();
     target = str.toDouble();
     Serial.println(target);
     analogWrite(Motor, 254);
   }

//   Serial.print("elapsed_time = ");
//   Serial.print(elapsed_time);
//   Serial.print(", speed = ");
//   Serial.print(spd);
//   Serial.print(", p1 = ");
//   Serial.print(p1);
//   Serial.print(", p2 = ");
//   Serial.print(p2);
//   Serial.print(", i = ");
//   Serial.print(i);
//   Serial.print(", d = ");
//   Serial.print(d);
//
//   
//   Serial.print(", emp_rev = ");
//   Serial.print(temp_rev);
//   Serial.print(", rev = ");
//   Serial.print(rev);
//   Serial.print(", omega = ");
//   Serial.print(omega);
//   Serial.print(", angle = ");
//   Serial.println(angle);
    Serial.print("angle = ");
    Serial.print(angle);
    Serial.print(", rev = ");
    Serial.print(rev);
    Serial.print(", PWM = ");
    Serial.println(spd);
 }
