// this skwtch contains:
// 1. high speed servo controller with timer2. This sketch is able to control ppm on a per duty cycle basis
// 2. dual independent action sequence manager through timer 1
// the purpose of this skwtch is to test fast, frequent manipulation of timer settings and observe results

// actions
// Uses PDB timer
//#include <PWMServo.h>

// uses FTM0 timer by default, interferes with PWM function on some pins, see https://www.pjrc.com/teensy/td_pulse.html
#include <TeensyDelay.h>
#define NONE 0
#define LED_OFF 1
#define LED_ON 2

#define RISING_NEUTRAL 1
#define FALLING_NEUTRAL 3
#define SERVO_MIN 4
#define SERVO_MAX 2

#define PIN_LED 13
#define PIN_LED1 10
#define PIN_LED2 11
#define PIN_LED3 12
#define PIN_LED4 9

#define PIN_SERVO 4
#define PIN_TEST 3



#define MAX_SERVO_PULSEWIDTH 930
#define MIN_SERVO_PULSEWIDTH 560
#define CENTRAL_SERVO_PULSEWIDTH 745

// for testing PWM output on LEDs
//#define MAX_SERVO_PULSEWIDTH 3000
//#define MIN_SERVO_PULSEWIDTH 100
//#define CENTRAL_SERVO_PULSEWIDTH 1500

const float pi = 3.1415926535898;

float ffmod(float x, float y){
  return x-y*floor(x/y);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//PWMServo myservo;

IntervalTimer blinkerTimer;
volatile byte pending_action_blinker = NONE;
volatile bool flash_in_progress = false;
volatile float on_time_ms = 20;

void blinker() {
    switch (pending_action_blinker){
        case LED_ON:
            digitalWrite(PIN_LED,HIGH);
            digitalWrite(PIN_LED1,LOW);
            digitalWrite(PIN_LED2,LOW);
            digitalWrite(PIN_LED3,LOW);
            pending_action_blinker = LED_OFF;
            blinkerTimer.begin(blinker,on_time_ms*1000);
            break;

        case LED_OFF:
            digitalWrite(PIN_LED,LOW);
            digitalWrite(PIN_LED1,HIGH);
            digitalWrite(PIN_LED2,HIGH);
            digitalWrite(PIN_LED3,HIGH);
            pending_action_blinker  = NONE;
            flash_in_progress = false;
            blinkerTimer.end();
            break;

        case NONE:
        default:
            break;
    }
} 


// ctrl_phase: phase to START command to max deflection
volatile float ctrl_phase;
// 0:azimuth, rad; 1:omega, rad/s
volatile float state_buffer[2];
// micros()
volatile unsigned long state_buffer_ts;
// us to next control phase, time between each new ISR
volatile float quarter_period;
volatile float current_phase;
volatile int pending_action_cyclic = NONE;
// TeensyDelay uses 16bit FTM timer that only goes to about 69ms, if we need a delay longer
// than that we need to handle this gracefully
volatile uint32_t remaining_delay_us;
void cyclic(){
  if (remaining_delay_us!=0){
    if (remaining_delay_us > 60000){
      TeensyDelay::trigger(60000,0);
      remaining_delay_us -= 60000;
    } else {
      TeensyDelay::trigger(remaining_delay_us);
      remaining_delay_us = 0;
    }
    return;
  }

  switch(pending_action_cyclic){

      case RISING_NEUTRAL:
          Serial.println("RISING_NEUTRAL");
          setPulsewidth(CENTRAL_SERVO_PULSEWIDTH);
          analogWrite(PIN_TEST,10);
          pending_action_cyclic = SERVO_MAX;
          // adjust control phase to sync with estimated state
          // 5e5 = 2(for 2pi) * 1e6 (sec -> us) / 4 (quarter period)
          quarter_period = 5e5*pi/state_buffer[1];
//XXX would micro() still work here?
          current_phase = state_buffer[0] + float(micros()-state_buffer_ts)*1e-6*state_buffer[1];
          remaining_delay_us = (ffmod(ctrl_phase-current_phase,2*pi)/state_buffer[1])*1e6;
          //Serial.print("current phase= ");
          //Serial.println(ffmod(current_phase,2*pi)/pi*180);
          //Serial.print("dist 2 ctrl phase= ");
          //Serial.println((ffmod(ctrl_phase-current_phase,2*pi))/pi*180);
          //Serial.print("wakeup after(s) ");
          //Serial.println((ffmod(ctrl_phase-current_phase,2*pi)/state_buffer[1]));
          //Serial.println(ffmod((ctrl_phase-current_phase)/2/pi,360));
          break;
          
      case SERVO_MAX:
          Serial.println("SERVO_MAX");
          setPulsewidth(MAX_SERVO_PULSEWIDTH);
          analogWrite(PIN_TEST,50);
          pending_action_cyclic = FALLING_NEUTRAL;
          remaining_delay_us = quarter_period;
          break;

      case FALLING_NEUTRAL:
          Serial.println("FALLING_NEUTRAL");
          setPulsewidth(CENTRAL_SERVO_PULSEWIDTH);
          analogWrite(PIN_TEST,150);
          pending_action_cyclic = SERVO_MIN;
          remaining_delay_us = quarter_period;
          break;

      case SERVO_MIN:
          Serial.println("SERVO_MIN");
          setPulsewidth(MIN_SERVO_PULSEWIDTH);
          analogWrite(PIN_TEST,200);
          pending_action_cyclic  = RISING_NEUTRAL;
          remaining_delay_us = quarter_period;
          break;

      case NONE:
      default:
          //Serial.println("fallthrough");
          break;
  }

  if (remaining_delay_us > 60000){
    TeensyDelay::trigger(60000,0);
    remaining_delay_us -= 60000;
  } else {
    TeensyDelay::trigger(remaining_delay_us);
    remaining_delay_us = 0;
  }
  return;
} 


int min_us = MIN_SERVO_PULSEWIDTH;
int max_us = MAX_SERVO_PULSEWIDTH;

volatile uint32_t oldres;
volatile int duty;
void setPulsewidth(float us)
{
  if (us<min_us) { us = min_us; }
  if (us>max_us) { us = max_us; }
	duty = us/(1e6/300.0)*65535;
#if TEENSYDUINO >= 137
	noInterrupts();
	oldres = analogWriteResolution(16);
	analogWrite(PIN_SERVO, duty);
  // restore resolution
	analogWriteResolution(oldres);
	interrupts();
#endif
}

volatile bool light_on = false;
void test(){
    if (!light_on){
      digitalWrite(13,HIGH);
      TeensyDelay::trigger(60e3,1);
      light_on = true;
    } else{
      digitalWrite(13,LOW);
      TeensyDelay::trigger(20e3,1);
      light_on = false;
    }
}
      

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(PIN_LED,OUTPUT);
    pinMode(PIN_LED1,OUTPUT);
    pinMode(PIN_LED2,OUTPUT);
    pinMode(PIN_LED3,OUTPUT);
    pinMode(PIN_LED4,OUTPUT);
    pinMode(PIN_TEST,OUTPUT);
    pinMode(6,OUTPUT);
    digitalWrite(6,HIGH);
    digitalWrite(PIN_LED, LOW);

    pinMode(PIN_SERVO,OUTPUT);
    analogWriteFrequency(PIN_SERVO, 300);

    pinMode(13,OUTPUT);
    pinMode(8,OUTPUT);
    //rad
    state_buffer[0] = 0;
    // rad/s
    state_buffer[1] = 2*pi;
    state_buffer_ts = 0;
    //myservo.attach(PIN_SERVO,560,930);

    pending_action_cyclic = RISING_NEUTRAL;
    blinkerTimer.begin(blinker,1e6);
    TeensyDelay::begin();
    TeensyDelay::addDelayChannel(cyclic, 0);
    //TeensyDelay::addDelayChannel(test, 1);
    TeensyDelay::trigger(10,0);
    //TeensyDelay::trigger(10,1);

    
}


void loop() {
  // put your main code here, to run repeatedly:
    if (Serial.available()>0) {
        float val = Serial.parseFloat();
        cli();
        // set speed, rev/s
        state_buffer[1] = val*2*pi;
        sei();
        Serial.print(val);
        //Serial.print("  ");
        //Serial.println(fmap(val,0,100,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
        Serial.println();

        pending_action_blinker = LED_ON;
        if (!flash_in_progress){
          // time needed to travel 10 degrees, so that light will be on for 10 deg
          //on_time_ms = 10.0/180.0*pi/x[1][0]*1000.0;
          // XXX a bit hacky, but probably OK
          on_time_ms = 3000;
          float blink_delay = 1000;
          if (blink_delay>0){
            flash_in_progress = true;
            pending_action_blinker = LED_ON;
            blinkerTimer.begin(blinker,blink_delay*1000);
          }
        }
    }
    delay(100);

}
