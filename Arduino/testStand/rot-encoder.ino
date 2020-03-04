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
 #define tick 32.72727
 #define pi 3.14159265358979323846264338

 int counter = 0; 
 int currentStateCLK;
 int previousStateCLK;
 int toggle;

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

double omega, elapsed_time, rev = 0, temp_rev = 0;
long cur_time, prev_time; // elapsed_time;

 void loop() { 
  
  // Read the current state of inputCLK
   currentStateCLK = digitalRead(inputCLK);
    
   // If the previous and the current state of the inputCLK are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 

     if (toggle){
     prev_time = cur_time;
     cur_time = millis();
     }
     toggle = !toggle;
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
     elapsed_time = (double)(cur_time-prev_time)/1000;
     // Serial.println(elapsed_time);
     // omega = tick/180.0*pi/elapsed_time;
     temp_rev = tick/360.0/elapsed_time; // Simple Average filter
     // Serial.println(counter);
     if (counter > 1){
         rev = (99*rev + temp_rev)/100;
         Serial.println(rev);
         Serial.println(temp_rev);
     }
     
   } 
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 
   
 }
