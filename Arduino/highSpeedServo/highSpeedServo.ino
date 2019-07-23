// this skwtch contains:
// 1. high speed servo controller with timer2. This sketch is able to control ppm on a per duty cycle basis
// 2. dual independent action sequence manager through timer 1
// the purpose of this skwtch is to test fast, frequent manipulation of timer settings and observe results

// actions
#define NONE 0
#define LED_OFF 1
#define LED_ON 2

#define RISING_NEUTRAL 1
#define FALLING_NEUTRAL 3
#define SERVO_MIN 4
#define SERVO_MAX 2

#define PIN_LED1 10
#define PIN_LED2 11
#define PIN_LED3 12

#define PIN_LED4 9

#define MAX_SERVO_PULSEWIDTH 930
#define MIN_SERVO_PULSEWIDTH 560
#define CENTRAL_SERVO_PULSEWIDTH 745

// for testing PWM output on LEDs
//#define MAX_SERVO_PULSEWIDTH 3000
//#define MIN_SERVO_PULSEWIDTH 100
//#define CENTRAL_SERVO_PULSEWIDTH 1500

const float pi = 3.1415926535898;

float ffmod(float x, float y){
  int quotient = floor(x/y);
  return x-y*quotient;
}

// init timer 1
// in this setup, timer 1 will support two independent action sequence with arbitrary delay
// between each action (max 4.19s)
// this is achieved by setting COMPA and COMPB interrupt
void timer1_init(){

    //set timer1 interrupt 
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCCR1C = 0; // PWM related stuff
    TIFR1 |= (1<<TOV1) | (1<<OCF1B) | (1<<OCF1A); // writing 1 to TOV1 clears the flag, preventing the ISR to be activated as soon as sei();
    TCNT1 = 0;
    // TODO clear action sequence next_action=NONE to prevent accident triggering

    // prescaler: 1024
    // duty cycle: (16*10^6) / (1024*65536) Hz = 0.238Hz (4.19s)
    // per count : 64us
    // this starts counting
    TCCR1B |= (1 << CS12) | (1 << CS10) ; 

    // enable timer compare interrupt and overflow interrupt
    //TIMSK1 = (1 << OCIE1A) | ( 1 << TOIE1); // for reference, ovf interrupt
    //TIMSK1 = (1 << TOIE1);



}

// this will set ISR(TIMER1_COMPA_vect) to fire in specified delay
// the ISR will determine the proper action to take depending on the value in pending_action
//WARNING this may ceize
void next_action_t1a(float ms){

    uint16_t ticks = TCNT1 + (ms/0.064);


    cli();// is this really necessary?

    OCR1A  = ticks;

    sei();

}

volatile byte pending_action_t1a = NONE;
volatile bool flash_in_progress = false;
volatile float on_time_ms = 20;
ISR(TIMER1_COMPA_vect) {
    switch (pending_action_t1a){

        case LED_ON:
            digitalWrite(PIN_LED1,LOW);
            digitalWrite(PIN_LED2,LOW);
            digitalWrite(PIN_LED3,LOW);
            pending_action_t1a = LED_OFF;
            next_action_t1a(on_time_ms);
            break;

        case LED_OFF:
            digitalWrite(PIN_LED1,HIGH);
            digitalWrite(PIN_LED2,HIGH);
            digitalWrite(PIN_LED3,HIGH);
            pending_action_t1a  = NONE;
            //disable_t1a();
            flash_in_progress = false;
            break;

        case NONE:
            break;
    }
} 


// this will set ISR(TIMER1_COMPB_vect) to fire in specified delay
// the ISR will determine the proper action to take depending on the value in pending_action
void next_action_t1b(float ms){

    // if we need to take next action immediately, 
    // we have to allow a time for exit out of thie function and ISR
    // to enter this state would require a sudden change of control phase of around -90deg
    uint16_t ticks = TCNT1 + max(ms/0.064,2);

    cli();// is this really necessary?
    OCR1B  = ticks;
    sei();
    

}

// ctrl_phase: phase to START command to max deflection
volatile float ctrl_phase;
// 0:azimuth, rad; 1:omega, rad/s
volatile float state_buffer[2];
// micros()
volatile uint16_t state_buffer_ts;
// ms to next control phase, time between each new ISR
volatile float quarter_period;
volatile float current_phase;
volatile int pending_action_t1b = NONE;
ISR(TIMER1_COMPB_vect) {
  //Serial.print("COMPB = ");
  //Serial.print(pending_action_t1b);

  switch(pending_action_t1b){

      case RISING_NEUTRAL:
          //Serial.println("RISING_NEUTRAL");
          setPulseWidth(CENTRAL_SERVO_PULSEWIDTH);
          pending_action_t1b = SERVO_MAX;
          // adjust control phase to sync with estimated state
          // 500 = 2(for 2pi) * 1000 (sec -> ms) / 4 (quarter period)
          quarter_period = 500.0*pi/state_buffer[1];
          current_phase = state_buffer[0] + ffmod(TCNT1-state_buffer_ts,65536)*6.4e-5*state_buffer[1];
          // make sure next_action_t1b gets a positive delay time
          //Serial.print("du(ms)=");
          //Serial.println(quarter_period-ffmod(ctrl_phase-current_phase,2*pi)/state_buffer[1]*1000);
          // experiment shows roughly 1ms error due to processing time,which results in a ~26ms correction periodically, not sure what to do
          next_action_t1b((ffmod(ctrl_phase-current_phase,2*pi)/state_buffer[1])*1000);
          //next_action_t1b(quarter_period);
          break;
          
      case SERVO_MAX:
          //Serial.println("SERVO_MAX");
          setPulseWidth(MAX_SERVO_PULSEWIDTH);
          pending_action_t1b = FALLING_NEUTRAL;
          next_action_t1b(quarter_period);
          break;

      case FALLING_NEUTRAL:
          //Serial.println("FALLING_NEUTRAL");
          setPulseWidth(CENTRAL_SERVO_PULSEWIDTH);
          pending_action_t1b = SERVO_MIN;
          next_action_t1b(quarter_period);
          break;

      case SERVO_MIN:
          //Serial.println("SERVO_MIN");
          setPulseWidth(MIN_SERVO_PULSEWIDTH);
          pending_action_t1b  = RISING_NEUTRAL;
          next_action_t1b(quarter_period);
          break;

      case NONE:
          //Serial.println("none");
          break;

      default:
          //Serial.println("default");
          break;
  }
  //Serial.println("done");
} 

void stop_timer1(){
    cli();
    //set timer2 interrupt 
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCCR1C = 0; // PWM related stuff
    sei();

}

void enable_t1a(){

    cli();
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

void enable_t1b(){
    cli();
    TIMSK1 |= (1 << OCIE1B); 
    sei();
}

void disable_t1a(){
    cli();
    TIMSK1 &= ~(1 << OCIE1A);
    sei();
}

void disable_t1b(){
    cli();
    TIMSK1 &= ~(1 << OCIE1B); 
    sei();
}

// timer2_init
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
// D13 - B5
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



float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setPulseWidth(float us){
    us = (us>MAX_SERVO_PULSEWIDTH)?MAX_SERVO_PULSEWIDTH:us;
    us = (us<MIN_SERVO_PULSEWIDTH)?MIN_SERVO_PULSEWIDTH:us;
    //Serial.print("pulsewidth set=");
    //Serial.println((uint8_t) (us/16));
    cli(); //XXX would this cause irratical behavior?
    OCR2A = (uint8_t) (us/16); // 16us per tick of clock
    sei();
}
void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(PIN_LED1,OUTPUT);
    pinMode(PIN_LED2,OUTPUT);
    pinMode(PIN_LED3,OUTPUT);
    pinMode(PIN_LED4,OUTPUT);

    pinMode(13,OUTPUT);
    pinMode(8,OUTPUT);
    //rad
    state_buffer[0] = 0;
    // rad/s
    state_buffer[1] = 2*pi;
    state_buffer_ts = TCNT1;
    timer1_init();
    enable_t1a();
    enable_t1b();
    pending_action_t1b = RISING_NEUTRAL;
    next_action_t1b(1);
    enablePWM();
}

void loop() {
  // put your main code here, to run repeatedly:
    if (Serial.available()>0) {
        float val = Serial.parseFloat();
        cli();
        ctrl_phase = fmap(val,0,360,0,2*pi);
        sei();
        
        Serial.print(val);
        //Serial.print("  ");
        //Serial.println(fmap(val,0,100,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
        Serial.println();
    }
    delay(100);
    
    pending_action_t1a = LED_ON;
    // Setup LED to blink when azimuth = 0
    if (!flash_in_progress){
      // time needed to travel 10 degrees, so that light will be on for 10 deg
      //on_time_ms = 10.0/180.0*pi/x[1][0]*1000.0;
      // XXX a bit hacky, but probably OK
      on_time_ms = 30;
      float t1a_delay = 50;
      if (t1a_delay>0){
        flash_in_progress = true;
        next_action_t1a(t1a_delay);
        pending_action_t1a = LED_ON;
        //Serial.println(F("LED on - "));
        //Serial.println(t1a_delay);
      } else {
        //Serial.print("neg omega - ");
        //Serial.println(t1a_delay);
      }
    } else {
      //Serial.println("flash blk");
    }


}
