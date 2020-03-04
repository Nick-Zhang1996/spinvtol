/*
  Rotary Encoder Test
  Demonstrates operation of Rotary Encoder
  Displays results on Serial Monitor
*/
 
 // Rotary Encoder Inputs
 #define inputCLK 4
 #define inputDT 5
 
 // LED Outputs
 #define ledCW 8
 #define ledCCW 9
 #define tick 30
 #define pi 3.14159265358979323846264338

 int counter = 0; 
 int currentStateCLK;
 int previousStateCLK; 
 int omega, cur_time, prev_time;

 String encdir ="";

 void setup() { 
   
   // Set encoder pins as inputs  
   pinMode (inputCLK,INPUT);
   pinMode (inputDT,INPUT);
   
   // Set LED pins as outputs
   pinMode (ledCW,OUTPUT);
   pinMode (ledCCW,OUTPUT);
   
   // Setup Serial Monitor
   Serial.begin (9600);
   
   // Read the initial state of inputCLK
   // Assign to previousStateCLK variable
   previousStateCLK = digitalRead(inputCLK);

 } 

 void loop() { 
  
  // Read the current state of inputCLK
   currentStateCLK = digitalRead(inputCLK);
    
   // If the previous and the current state of the inputCLK are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 

     prev_time = cur_time;
     cur_time = millis();
     // If the inputDT state is different than the inputCLK state then 
     // the encoder is rotating counterclockwise
     if (digitalRead(inputDT) != currentStateCLK) { 
       counter --;
       encdir ="CCW";
       digitalWrite(ledCW, LOW);
       digitalWrite(ledCCW, HIGH);
       
     } else {
       // Encoder is rotating clockwise
       counter ++;
       encdir ="CW";
       digitalWrite(ledCW, HIGH);
       digitalWrite(ledCCW, LOW);
       
     }
     Serial.print("Direction: ");
     Serial.print(encdir);
     Serial.print(" -- Value: ");
     Serial.println(counter);
   } 
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 

   omega = tick/180*pi/()cur_time-prev_time);
 }
