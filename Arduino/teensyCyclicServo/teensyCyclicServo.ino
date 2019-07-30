// this skwtch contains:
// 1. high speed servo controller with timer2. This sketch is able to control ppm on a per duty cycle basis
// 2. dual independent action sequence manager through timer 1
// the purpose of this skwtch is to test fast, frequent manipulation of timer settings and observe results

// actions
// Uses PDB timer
//#include <PWMServo.h>
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

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//PWMServo myservo;

IntervalTimer Timer1;
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
            Timer1.begin(blinker,on_time_ms*1000);
            break;

        case LED_OFF:
            digitalWrite(PIN_LED,LOW);
            digitalWrite(PIN_LED1,HIGH);
            digitalWrite(PIN_LED2,HIGH);
            digitalWrite(PIN_LED3,HIGH);
            pending_action_blinker  = NONE;
            flash_in_progress = false;
            Timer1.end();
            break;

        case NONE:
        default:
            break;
    }
} 


IntervalTimer Timer2;
// ctrl_phase: phase to START command to max deflection
volatile float ctrl_phase;
// 0:azimuth, rad; 1:omega, rad/s
volatile float state_buffer[2];
// micros()
volatile uint16_t state_buffer_ts;
// ms to next control phase, time between each new ISR
volatile float quarter_period;
volatile float current_phase;
volatile int pending_action_cyclic = NONE;
void cyclic(){

  switch(pending_action_cyclic){

      case RISING_NEUTRAL:
          Serial.println("RISING_NEUTRAL");
          setPulsewidth(CENTRAL_SERVO_PULSEWIDTH);
          pending_action_cyclic = SERVO_MAX;
          // adjust control phase to sync with estimated state
          // 500 = 2(for 2pi) * 1000 (sec -> ms) / 4 (quarter period)
          quarter_period = 500.0*pi/state_buffer[1];
//XXX would micro() still work here?
          current_phase = state_buffer[0] + ffmod(micros()-state_buffer_ts,65536)*6.4e-5*state_buffer[1];
          // make sure next_action_t2 gets a positive delay time
          //Serial.print("du(ms)=");
          //Serial.println(quarter_period-ffmod(ctrl_phase-current_phase,2*pi)/state_buffer[1]*1000);
          // experiment shows roughly 1ms error due to processing time,which results in a ~26ms correction periodically, not sure what to do
          Timer2.begin(cyclic,(ffmod(ctrl_phase-current_phase,2*pi)/state_buffer[1])*1e6);
          break;
          
      case SERVO_MAX:
          Serial.println("SERVO_MAX");
          setPulsewidth(MAX_SERVO_PULSEWIDTH);
          pending_action_cyclic = FALLING_NEUTRAL;
          Timer2.begin(cyclic,quarter_period);
          break;

      case FALLING_NEUTRAL:
          Serial.println("FALLING_NEUTRAL");
          setPulsewidth(CENTRAL_SERVO_PULSEWIDTH);
          pending_action_cyclic = SERVO_MIN;
          Timer2.begin(cyclic,quarter_period);
          break;

      case SERVO_MIN:
          Serial.println("SERVO_MIN");
          setPulsewidth(MIN_SERVO_PULSEWIDTH);
          pending_action_cyclic  = RISING_NEUTRAL;
          Timer2.begin(cyclic,quarter_period);
          break;

      case NONE:
      default:
          Serial.println("fallthrough");
          break;
  }
  //Serial.println("done");
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
  Serial.print(duty);
#endif
}




void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(PIN_LED,OUTPUT);
    pinMode(PIN_LED1,OUTPUT);
    pinMode(PIN_LED2,OUTPUT);
    pinMode(PIN_LED3,OUTPUT);
    pinMode(PIN_LED4,OUTPUT);
    digitalWrite(PIN_LED, LOW);

    pinMode(PIN_SERVO,OUTPUT);
    analogWriteFrequency(PIN_SERVO, 300);

    pinMode(13,OUTPUT);
    pinMode(8,OUTPUT);
    //rad
    state_buffer[0] = 0;
    // rad/s
    state_buffer[1] = 2*pi;
    state_buffer_ts = micros();
    //myservo.attach(PIN_SERVO,560,930);

    pending_action_cyclic = RISING_NEUTRAL;
    Timer1.begin(blinker,1e6);
    Timer2.begin(cyclic,1e6);
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
        // Setup LED to blink when azimuth = 0
        if (!flash_in_progress){
          // time needed to travel 10 degrees, so that light will be on for 10 deg
          //on_time_ms = 10.0/180.0*pi/x[1][0]*1000.0;
          // XXX a bit hacky, but probably OK
          on_time_ms = 3000;
          float blink_delay = 1000;
          if (blink_delay>0){
            flash_in_progress = true;
            pending_action_blinker = LED_ON;
            Timer1.begin(blinker,blink_delay*1000);
          }
        }
    }
    delay(100);

}
