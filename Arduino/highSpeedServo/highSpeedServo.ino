// high speed servo controller with timer. This sketch is able to control ppm on a per duty cycle basis

void enablePWM(){
    cli();//stop interrupts
    //set timer2 interrupt 

    TCCR2A = 0;// set entire TCCR2A register to 0
    TCCR2B = 0;// same for TCCR2B
    TCNT2  = 0;//initialize counter value to 0

    // Set CS bits for 256 prescaler
    // duty cycle: (16*10^6) / (256*256) Hz = 244Hz < 333Hz (servo max freq)
    // per count time: 16us
    TCCR2B |= (1 << CS21) | (1 << CS22) ; 

    // set compare target, this controls the on-time of PWM
    // for n% signal:
    // OCR2A = (uint8_t) 256.0*onTime (fraction (0-1) );
    // Note, OCR2A < 20 creates erratic behavior(on oscilloscope) worth investicating, it  does NOT set power to 0    
    OCR2A = 47; // = 745us signal, neutral of DS3005HV servo


    // enable timer compare interrupt and overflow interrupt
    TIMSK2 |= (1 << OCIE2A) | ( 1 << TOIE2);

    sei();//allow interrupts
  
}

// to ensure safety of the servo, the servo should always be given an active, safe command
void disablePWM(){
  
  
  cli();//stop interrupts
  //unset timer2 interrupt 
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  TIMSK2 = 0;

  sei();//allow interrupts
  
}


// https://www.arduino.cc/en/Reference/PortManipulation
// Called at the falling edge of on-time, enter off-time configuration here
ISR(TIMER2_COMPA_vect){
// digital write takes ~6us to execute
// inline assembly takes <1us
// use with caution, though

// D8
    asm (
      "cbi %0, %1 \n"
      : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB0)
    );
// D13
    asm (
      "cbi %0, %1 \n"
      : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB5)
    );
}

// Beginning of each Duty Cycle, enter on-time configuration here
ISR(TIMER2_OVF_vect){
// D8
    asm (
      "sbi %0, %1 \n"
      : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTD0)
    );
// D13
    asm (
      "sbi %0, %1 \n"
      : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB5)
    );

}

#define MAX_SERVO_PULSEWIDTH 930
#define MIN_SERVO_PULSEWIDTH 560
#define CENTRAL_SERVO_PULSEWIDTH 745

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setPulseWidth(float us){
    us = (us>MAX_SERVO_PULSEWIDTH)?MAX_SERVO_PULSEWIDTH:us;
    us = (us<MIN_SERVO_PULSEWIDTH)?MIN_SERVO_PULSEWIDTH:us;

    cli(); //XXX would this cause irratical behavior?
    OCR2A = (uint8_t) (us/16); // 16us per tick of clock
    sei();
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
/*
  cli();

  //set timer2 interrupt 
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCCR1C = 0; // PWM related stuff
  TIFR1 |= (1<<TOV1); // writing 1 to TOV1 clears the flag, preventing the ISR to be activated as soon as sei();


  // enable timer compare interrupt and overflow interrupt
  //TIMSK1 = (1 << OCIE1A) | ( 1 << TOIE1); // for reference, ovf interrupt
  TIMSK1 = (1 << TOIE1);

  uint16_t preload = 65536u - (ms/0.064);
  TCNT1H  = highByte(preload);
  TCNT1L  = lowByte(preload);

  // prescaler: 1024
  // duty cycle: (16*10^6) / (1024*65536) Hz = 0.238Hz (4.19s)
  // per count : 64us
  // this starts counting
  TCCR1B |= (1 << CS12) | (1 << CS10) ; 

  sei();
*/
  pinMode(13,OUTPUT);
  pinMode(8,OUTPUT);

  enablePWM();
}

void loop() {
  // put your main code here, to run repeatedly:
    if (Serial.available()>0) {
        float val = Serial.parseFloat();
        setPulseWidth(fmap(val,0,100,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
        Serial.print(val);
        Serial.print("  ");
        Serial.println(fmap(val,0,100,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
    }

}
