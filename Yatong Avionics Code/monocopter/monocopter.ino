// monocopter controller
// Nick Zhang, Yatong Bai

// 3 red led array
#define PIN_LED1 10
#define PIN_LED2 11
#define PIN_LED3 12
// blue led
#define PIN_LED4 9
#define PIN_SERVO 8

// action sequence for T1_COMPA
#define NONE 0
#define LED_OFF 1
#define LED_ON 2

// action sequence for T1_COMPB
#define RISING_NEUTRAL 1
#define SERVO_MAX 2
#define FALLING_NEUTRAL 3
#define SERVO_MIN 4

#define MAX_SERVO_PULSEWIDTH 930
#define MIN_SERVO_PULSEWIDTH 560
#define CENTRAL_SERVO_PULSEWIDTH 745

// roll-ch1->ch0, pitch ch2->ch1, rudder ch4->ch2
#define PITCH_FULL_PULL 1930.0
#define PITCH_FULL_PUSH 1108.0
#define ROLL_FULL_LEFT 1108.0
#define ROLL_FULL_RIGHT 1930.0
#define RUDDER_FULL_LEFT 1110.0
#define RUDDER_FULL_RIGHT 1930.0
#define VR_MIN 932.0
#define VR_MAX 2032.0

#define EI_ARDUINO_INTERRUPTED_PIN
#include "/Users/Michael/Desktop/monocopter/EnableInterrupt/EnableInterrupt.h"

//  millis() use Timer 0
#include <Wire.h>
#include "/Users/Michael/Desktop/monocopter/SerialCommand.h"
#include "/Users/Michael/Desktop/monocopter/Adafruit_ADXL345_U.h"
#include "/Users/Michael/Desktop/monocopter/Filters.h"
#include "/Users/Michael/Desktop/monocopter/MatrixMath.h"

#define I2Cclock 400000
#define I2Cport Wire
#include "/Users/Michael/Desktop/monocopter/MPU9250.h"
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 // AD0 on MPU9250 pulled down to GND

typedef mtx_type matrix;
#define multiply(A,B,m,n,p,C) Matrix.Multiply((mtx_type*)A, (mtx_type*)B, m, n, p, (mtx_type*)C)
#define add(A,B,m,n,C) Matrix.Add((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define subtract(A,B,m,n,C) Matrix.Subtract((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define transpose(A,m,n,C) Matrix.Transpose((mtx_type*)A, m, n, (mtx_type*)C)
#define invert(A,m) Matrix.Invert((mtx_type*)A,m)
#define mtxprint(A,m,n,N) Matrix.Print((mtx_type*)A, m, n, N)
#define mtxcopy(A,m,n,B) Matrix.Copy((mtx_type*)A, m, n, (mtx_type*)B)
SerialCommand sCmd;

// MPU9250 init sets Wire I2c freq(adxl345 does not), let it run first
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

Adafruit_ADXL345_Unified accel_in = Adafruit_ADXL345_Unified(1);
Adafruit_ADXL345_Unified accel_out = Adafruit_ADXL345_Unified(2);

const float pi = 3.1415926535897932384626434;
const float two_pi = pi*2;

// the main loop runs at 50HZ, much slower than needed to actuate accurate command
// Precise timing and actuation of POV LEDs and flap servo is achieved by timer interrupts

// the several functions below enable the execution of a sequence of action in precise, asynchrous manner
// the ISR will be called multiple times with timer interrupt, sleeping in between calls to allow main program
// to continue. the ISR itself should determine
// the proper action to take every time it is called, usually by maintaining a sequence number
// it is also responsible for setting up the interrupt registers to it will be called again at proper time
// to execute the next action sequence. 
// upon execution of the last command in sequence, the ISR should stop the timer/disable the ISR from undesirable
// calls in the future. 

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
    TIMSK1 = 0;
}

// this will set ISR(TIMER1_COMPA_vect) to fire in specified delay
// the ISR will determine the proper action to take depending on the value in pending_action
void next_action_t1a(float ms){

    // if we need to take next action immediately, 
    // we have to allow a time for exit out of thie function and ISR
    // to enter this state would require a sudden change of control phase of around -90deg
    uint16_t ticks = TCNT1 + max(ms/0.064,2);


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
            // Use Port B to turn on red LEDs
            // PORTB &= B100011;
            pending_action_t1a = LED_OFF;
            next_action_t1a(on_time_ms);
            break;

        case LED_OFF:
            // Use Port B to turn off red LEDs
            // PORTB |= B011100;
            pending_action_t1a  = NONE;
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
volatile int pending_action_t1b = RISING_NEUTRAL;
volatile float ctrl_magnitude;
// Functionalities and pin layout
// RC in : 3 channels, on pin 4,5,6 (port D 0-7)
// RC out: 2 channels, flap on pin 8, ESC on pin 9 (port B 8-13)

// Timer usage:
// Timer 0 : delay, millis
// Timer 1 : Servo

uint8_t rc_in_pinno[] = {4, 5, 6, 7, 3}; // CH0,1,2,3,4
volatile int rc_in_val[5] = {0};
volatile unsigned long rc_rising_ts[5] = {0};
volatile float dynamic_servo_central = CENTRAL_SERVO_PULSEWIDTH;
bool flag_signal_loss = false;

ISR(TIMER1_COMPB_vect) {

  dynamic_servo_central = fmap(float(rc_in_val[3]),VR_MIN,VR_MAX,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH); 

  switch(pending_action_t1b){

      case RISING_NEUTRAL:
          // setPulseWidth(CENTRAL_SERVO_PULSEWIDTH);
          // allow an adjustable offset, this is mapped to VR in T8j transmitter
          setPulseWidth(dynamic_servo_central);
          pending_action_t1b = SERVO_MAX;
          // adjust control phase to sync with estimated state
          // 500 = 2(for 2pi) * 1000 (sec -> ms) / 4 (quarter period)
          quarter_period = 500.0*pi/state_buffer[1];
          current_phase = state_buffer[0] + ffmod(TCNT1-state_buffer_ts,65536)*6.4e-5*state_buffer[1];
          // make sure next_action_t1b gets a positive delay time
          // experiment shows roughly 1ms error(at 5rev/s) due to processing time,which results in a ~26ms correction periodically, not sure what to do
          next_action_t1b((ffmod(ctrl_phase-current_phase,two_pi)/state_buffer[1])*1000);
          // next_action_t1b(quarter_period);
          break;
          
      case SERVO_MAX:
          setPulseWidth(fmap(ctrl_magnitude,0.0,1.0,dynamic_servo_central, MAX_SERVO_PULSEWIDTH));
          pending_action_t1b = FALLING_NEUTRAL;
          next_action_t1b(quarter_period);
          // D9, blue LED
              asm (
                "sbi %0, %1 \n"
                : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB1)
              );
          break;

      case FALLING_NEUTRAL:
          //setPulseWidth(CENTRAL_SERVO_PULSEWIDTH);
          setPulseWidth(dynamic_servo_central);

          pending_action_t1b = SERVO_MIN;
          next_action_t1b(quarter_period);
          break;

      case SERVO_MIN:
          setPulseWidth(fmap(ctrl_magnitude,0.0,1.0,dynamic_servo_central, MIN_SERVO_PULSEWIDTH));
          pending_action_t1b  = RISING_NEUTRAL;
          next_action_t1b(quarter_period);
          // D9, blue LED, low enable
              asm (
                "cbi %0, %1 \n"
                : : "I" (_SFR_IO_ADDR(PORTB)), "I" (PORTB1)
              );
          break;

      case NONE:
          break;

      default:
          break;
  }
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
// TODO would probably benefit from PWM mode
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


float fmap_temp;
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  fmap_temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (out_max>out_min){
    fmap_temp = (fmap_temp>out_max)?out_max:fmap_temp;
    fmap_temp = (fmap_temp<out_min)?out_min:fmap_temp;
  } else{
    fmap_temp = (fmap_temp<out_max)?out_max:fmap_temp;
    fmap_temp = (fmap_temp>out_min)?out_min:fmap_temp;

  }
  return fmap_temp;
}

void setPulseWidth(float us){
    us = (us>MAX_SERVO_PULSEWIDTH)?MAX_SERVO_PULSEWIDTH:us;
    us = (us<MIN_SERVO_PULSEWIDTH)?MIN_SERVO_PULSEWIDTH:us;
    cli(); //TODO would this cause irratical behavior?
    OCR2A = (uint8_t) (us/16); // 16us per tick of clock
    sei();
}

void displaySensorDetails(Adafruit_ADXL345_Unified &accel)
{
//  sensor_t sensor;
//  accel.getSensor(&sensor);
//  // Serial.println(F("------------------------------------"));
//  // Serial.print  (F("Sensor:       ")); // Serial.println(sensor.name);
//  // Serial.print  (F("Driver Ver:   ")); // Serial.println(sensor.version);
//  // Serial.print  (F("Unique ID:    ")); // Serial.println(sensor.sensor_id);
//  // Serial.print  (F("Max Value:    ")); // Serial.print(sensor.max_value); // Serial.println(F(" m/s^2"));
//  // Serial.print  (F("Min Value:    ")); // Serial.print(sensor.min_value); // Serial.println(F(" m/s^2"));
//  // Serial.print  (F("Resolution:   ")); // Serial.print(sensor.resolution); // Serial.println(F(" m/s^2"));
//  // Serial.println(F("------------------------------------"));
//  // Serial.println("");
  delay(500);
}

void displayDataRate(Adafruit_ADXL345_Unified &accel)
{
  // Serial.print  (F("Data Rate:    "));

  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      // Serial.print  (F("3200 "));
      break;
    case ADXL345_DATARATE_1600_HZ:
      // Serial.print  (F("1600 "));
      break;
    case ADXL345_DATARATE_800_HZ:
      // Serial.print  (F("800 "));
      break;
    case ADXL345_DATARATE_400_HZ:
      // Serial.print  (F("400 "));
      break;
    case ADXL345_DATARATE_200_HZ:
      // Serial.print  (F("200 "));
      break;
    case ADXL345_DATARATE_100_HZ:
      // Serial.print  (F("100 "));
      break;
    case ADXL345_DATARATE_50_HZ:
      // Serial.print  (F("50 "));
      break;
    case ADXL345_DATARATE_25_HZ:
      // Serial.print  (F("25 "));
      break;
    case ADXL345_DATARATE_12_5_HZ:
      // Serial.print  (F("12.5 "));
      break;
    case ADXL345_DATARATE_6_25HZ:
      // Serial.print  (F("6.25 "));
      break;
    case ADXL345_DATARATE_3_13_HZ:
      // Serial.print  (F("3.13 "));
      break;
    case ADXL345_DATARATE_1_56_HZ:
      // Serial.print  (F("1.56 "));
      break;
    case ADXL345_DATARATE_0_78_HZ:
      // Serial.print  (F("0.78 "));
      break;
    case ADXL345_DATARATE_0_39_HZ:
      // Serial.print  (F("0.39 "));
      break;
    case ADXL345_DATARATE_0_20_HZ:
      // Serial.print  (F("0.20 "));
      break;
    case ADXL345_DATARATE_0_10_HZ:
      // Serial.print  (F("0.10 "));
      break;
    default:
      // Serial.print  (F("???? "));
      break;
  }
  // Serial.println(F(" Hz"));
}

void displayRange(Adafruit_ADXL345_Unified &accel)
{
  // Serial.print  (F("Range:         +/- "));

  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      // Serial.print  (F("16 "));
      break;
    case ADXL345_RANGE_8_G:
      // Serial.print  (F("8 "));
      break;
    case ADXL345_RANGE_4_G:
      // Serial.print  (F("4 "));
      break;
    case ADXL345_RANGE_2_G:
      // Serial.print  (F("2 "));
      break;
    default:
      // Serial.print  (F("?? "));
      break;
  }
  // Serial.println(F(" g"));
}


//note: measured dutycycle for futaba FHSS 13564us, 73Hz
void rc_in_callback() {
  unsigned long timestamp = micros();
  int channel = -1;
  switch (arduinoInterruptedPin) {
    case 4 : channel = 0; break;
    case 5 : channel = 1; break;
    case 6 : channel = 2; break;
    case 7 : channel = 3; break;
    case 3 : channel = 4; break;
    default : break;
  }
  if (channel == -1) {
    return;
  }
  if (digitalRead(arduinoInterruptedPin) == HIGH) {
    rc_rising_ts[channel] = timestamp;
  } else {
    rc_in_val[channel] = timestamp - rc_rising_ts[channel];
  }
}


int xbee_rssi_pin = 3;
volatile unsigned long xbee_rssi_last_rising;
volatile unsigned long xbee_rssi_val;
volatile bool xbee_rssi_newdata = false;
void xbee_rssi_callback() {
  unsigned long timestamp = micros();

  if (digitalRead(xbee_rssi_pin) == HIGH) {
    xbee_rssi_last_rising = timestamp;
  } else {
   xbee_rssi_val = timestamp - xbee_rssi_last_rising;
   xbee_rssi_newdata = true;

  }
}
// --------- Serial command handlers -------
int onoff(char* arg) {
  if (arg != NULL) {
    if (!strcmp(arg, "on")) {
      return 1;
    } else if (!strcmp(arg, "off")) {
      return 0;
    }
  }
  return -1;
}

void led() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(13, rval);
  }
}

void led1() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(PIN_LED1, !rval);
  }
}

void led2() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(PIN_LED2, !rval);
  }
}

void led3() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(PIN_LED3, !rval);
  }
}

bool mag_verbose = true;
void display_mag() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    mag_verbose = (bool)rval;
  }
}

bool acc_verbose = true;
void display_acc() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    acc_verbose = (bool)rval;
  }
}

bool rc_verbose = true;
void display_rc() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    rc_verbose = (bool)rval;
  }
}
bool sig_verbose = true;
void display_sig() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    sig_verbose = (bool)rval;
  }
}

bool rssi_verbose = true;
void display_rssi() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    rssi_verbose = (bool)rval;
  }
}

void display_all() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    rssi_verbose = acc_verbose = mag_verbose = rc_verbose = sig_verbose =  (bool)rval;
  }
}


bool human_readable_output = false;
void human(){
  human_readable_output = true;
}

void machine(){
  human_readable_output = false;
}

int synchro_pinno = A1;
unsigned long epoch;
// trigger a falling edge on pin, have the other system ready for sync signal
// to receive such signal, connect pin9 of avionics to pin4 on testStand, also connect both GND
void synchronize(){
  digitalWrite(synchro_pinno,LOW);
  epoch = millis();
  // Serial.print(F("Syncing ... epoch = "));
  // Serial.println(epoch);
  //delay(1);
  digitalWrite(synchro_pinno,HIGH);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int i = 0; i < sizeof(rc_in_pinno) / sizeof(uint8_t);  i++) {
    pinMode(rc_in_pinno[i], INPUT);
    enableInterrupt( rc_in_pinno[i], rc_in_callback, CHANGE);
    enableInterrupt( xbee_rssi_pin, xbee_rssi_callback,CHANGE);
  }
//  // Serial.println(REFRESH_INTERVAL);
//  flap.attach(8);
//  esc.attach(9);

  // ---------------- MPU9240 init ---------------
  Wire.begin();
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  // Serial.print(F("MPU9250 I AM 0x"));
  // Serial.print(c, HEX);
  // Serial.print(F(" I should be 0x"));
  // Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    // Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    // Serial.print(F("x-axis self test: acceleration trim within : "));
    // Serial.print(myIMU.selfTest[0], 1); // Serial.println(F("% of factory value"));
    // Serial.print(F("y-axis self test: acceleration trim within : "));
    // Serial.print(myIMU.selfTest[1], 1); // Serial.println(F("% of factory value"));
    // Serial.print(F("z-axis self test: acceleration trim within : "));
    // Serial.print(myIMU.selfTest[2], 1); // Serial.println(F("% of factory value"));
    // Serial.print(F("x-axis self test: gyration trim within : "));
    // Serial.print(myIMU.selfTest[3], 1); // Serial.println(F("% of factory value"));
    // Serial.print(F("y-axis self test: gyration trim within : "));
    // Serial.print(myIMU.selfTest[4], 1); // Serial.println(F("% of factory value"));
    // Serial.print(F("z-axis self test: gyration trim within : "));
    // Serial.print(myIMU.selfTest[5], 1); // Serial.println(F("% of factory value"));

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    // Serial.println(F("MPU9250 initialized for active data mode...."));

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    // Serial.print(F("AK8963 "));
    // Serial.print(F("I AM 0x"));
    // Serial.print(d, HEX);
    // Serial.print(F(" I should be 0x"));
    // Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      // Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    // Serial.println(F("AK8963 initialized for active data mode...."));

    //  // Serial.println("Calibration values: ");
    // Serial.print(F("X-Axis factory sensitivity adjustment value "));
    // Serial.println(myIMU.factoryMagCalibration[0], 2);
    // Serial.print(F("Y-Axis factory sensitivity adjustment value "));
    // Serial.println(myIMU.factoryMagCalibration[1], 2);
    // Serial.print(F("Z-Axis factory sensitivity adjustment value "));
    // Serial.println(myIMU.factoryMagCalibration[2], 2);


    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    // Serial.println(F("AK8963 mag biases (mG)"));
    // Serial.println(myIMU.magBias[0]);
    // Serial.println(myIMU.magBias[1]);
    // Serial.println(myIMU.magBias[2]);

    // Serial.println(F("AK8963 mag scale (mG)"));
    // Serial.println(myIMU.magScale[0]);
    // Serial.println(myIMU.magScale[1]);
    // Serial.println(myIMU.magScale[2]);

    // Serial.println(F("Magnetometer:"));
    // Serial.print(F("X-Axis sensitivity adjustment value "));
    // Serial.println(myIMU.factoryMagCalibration[0], 2);
    // Serial.print(F("Y-Axis sensitivity adjustment value "));
    // Serial.println(myIMU.factoryMagCalibration[1], 2);
    // Serial.print(F("Z-Axis sensitivity adjustment value "));
    // Serial.println(myIMU.factoryMagCalibration[2], 2);
  } else
  {
    // Serial.print(F("Could not connect to MPU9250: 0x"));
    // Serial.println(c, HEX);

    // Communication failed, stop here
    // Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  // ---------------- ADXL345 init ----------------
  // Serial.println(F("Accelerometer Init -- inner (0x53)")); // Serial.println("");
  /* Initialise the sensor */
  if (!accel_in.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    // Serial.println(F("Inner ADXL345 not detected"));
    while (1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel_in.setRange(ADXL345_RANGE_16_G);
  // accel_in.setRange(ADXL345_RANGE_8_G);
  // accel_in.setRange(ADXL345_RANGE_4_G);
  // accel_in.setRange(ADXL345_RANGE_2_G);

  /* Display some basic information on this sensor */
  //displaySensorDetails(accel_in);

  /* Display additional settings (outside the scope of sensor_t) */
  //displayDataRate(accel_in);
  //displayRange(accel_in);
  // Serial.println("");
  // ------ outer ADXL345
  // Serial.println(F("Accelerometer Init -- outer (0x1D)")); // Serial.println("");
  /* Initialise the sensor */
  if (!accel_out.begin(0x1D))
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    // Serial.println(F("Outer ADXL345 not detected"));
    while (1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel_out.setRange(ADXL345_RANGE_16_G);
  // accel_out.setRange(ADXL345_RANGE_8_G);
  // accel_out.setRange(ADXL345_RANGE_4_G);
  // accel_out.setRange(ADXL345_RANGE_2_G);

  /* Display some basic information on this sensor */
  //displaySensorDetails(accel_out);

  /* Display additional settings (outside the scope of sensor_t) */
  //displayDataRate(accel_out);
  //displayRange(accel_out);
  // Serial.println("");
  // ----------- end ADXL345 init -----------

  // --------  Register serial commands ------
  pinMode(13, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED4, OUTPUT);
  pinMode(PIN_SERVO,OUTPUT);
  digitalWrite(PIN_LED1,HIGH);
  digitalWrite(PIN_LED2,HIGH);
  digitalWrite(PIN_LED3,HIGH);
  digitalWrite(PIN_LED4,HIGH);

  pinMode(synchro_pinno,OUTPUT);
  digitalWrite(synchro_pinno,HIGH);
 
  sCmd.addCommand("led", led); // test function, takes 1 argument on or off
  sCmd.addCommand("led1", led1); // test function, takes 1 argument on or off
  sCmd.addCommand("led2", led2); // test function, takes 1 argument on or off
  sCmd.addCommand("led3", led3); // test function, takes 1 argument on or off
  sCmd.addCommand("mag", display_mag);
  sCmd.addCommand("acc", display_acc);
  sCmd.addCommand("rc", display_rc);
  sCmd.addCommand("sig", display_sig);
  sCmd.addCommand("rssi", display_rssi);
  sCmd.addCommand("all", display_all);
  sCmd.addCommand("sync",synchronize);
  sCmd.addCommand("human",human);
  sCmd.addCommand("machine",machine);

  timer1_init();
  enable_t1a();
  enable_t1b();
  enablePWM();
  
}
// ---------------- EKF ----------------
// x = [theta(azimuth), omega] unit: rad, rad/s

// actually matrix here logically means type of the elements in a matrix, an array of elements compose a matrix
matrix x[2][1],F[2][2],FT[2][2],P[2][2],Q[2][2],H[1][2],HT[2][1];
float S;

// unit: (rad/s2)^2, this is a rough estimate, need to see video footage, of actual monocopter
const float acc_variance=10.0;
// experiment shows 60 deg variation for 4sigma(95% ish) -> convert to 1 sigma in rad:pow(0.2618,2)
const float mag_variance= pow(0.2618,2);

void kf_predict(){
  static unsigned long last_update = millis();
  unsigned long ts = millis();
  matrix buff_col[2][1],buff_square[2][2];
  float delta_t = (ts - last_update) / 1000.0;
  last_update = ts;
  // because F.transpose is calculated in place, we need to re-init F each time
  F[0][0] = 1;
  F[1][0] = 0;
  F[1][1] = 1;
  F[0][1] = delta_t;
  // x = Fx + Bu (no control so no Bu here)

  Matrix.Multiply(*F,*x,2,2,1,*buff_col);
  mtxcopy(buff_col,2,1,x);

  // P = FPF.T + Q
  multiply(F,P,2,2,2,buff_square);
  transpose(F,2,2,FT);
  multiply(buff_square,FT,2,2,2,P);
  Q[0][0] = 0.25*delta_t*delta_t*delta_t*delta_t*acc_variance;
  Q[0][1] = 0.5*delta_t*delta_t*delta_t*acc_variance;
  Q[1][0] = Q[0][1];
  Q[1][1] = delta_t*delta_t*acc_variance;
  add(P,Q,2,2,P);

  return;
}

// z: acc observation, z = [delta_acc], unit: m/s2
void kf_update_acc(float z){

  matrix buff_single[1][1],buff_row[1][2],buff_col[2][1],buff_square[2][2],buff_square2[2][2];
  matrix I[2][2];
  I[0][0] = 1;
  I[1][1] = 1;  
  I[0][1] = 0;
  I[1][0] = 0;
  // y = z - H(x), where H is a function for EKF
  
  matrix y[1][1];
  // constant from experiment, corresbond to 11.5 in rev/s representation
  y[0][0] = z - 0.2913*x[1][0]*x[1][0];
  H[0][0] = 0;
  // Jacobian
  H[0][1] = 2*0.2913*x[1][0];

  // S = HPH.T + R
  
  multiply(H,P,1,2,2,buff_row);
  multiply(buff_row,HT,1,2,1,buff_single);
  
  // covariance of innovation, R = var of observation, calculated from gain variation
  S = buff_single[0][0] + pow(1*x[1][0]*x[1][0],2);
  

  // K = P HT S-1 ----------------------
  matrix K[2][1];
  multiply(P,HT,2,2,1,buff_col); 
  //mtxprint(P,2,2,"P");
  //mtxprint(HT,2,1,"HT");
  
  matrix mat_Sinv[1][1];
  mat_Sinv[0][0] = 1.0/S;
  multiply(buff_col,mat_Sinv,2,1,1,K);

  //mtxprint(K,2,1,"K");
  //mtxprint(x,2,1,"x");

  // x = x + Ky
  multiply(K,y,2,1,1,buff_col);
  //mtxprint(buff_col,2,1,"Ky");
  add(x,buff_col,2,1,x);
  //// Serial.print("update -> ");
  //mtxprint(x,2,1,"x");

  
  // P = (I-KH)P
  multiply(K,H,2,1,2,buff_square);
  subtract(I,buff_square,2,2,buff_square);
  multiply(buff_square,P,2,2,2,buff_square2);
  mtxcopy(buff_square,2,2,P);
  // y = z - Hx post fit residual, not needed
  return;
}

inline float acc_norm(sensors_event_t* event){
  return sqrt(event->acceleration.x*event->acceleration.x+event->acceleration.y*event->acceleration.y+event->acceleration.z*event->acceleration.z);
}


// --------------------- LOOP ---------------------
FilterTwoPole lowfilter1(2.8, 0.35);
FilterTwoPole lowfilter2(5, 0.5);
FilterTwoPole lowfilter3(2.8, 0.35);
FilterTwoPole lowfilter4(5, 0.5);
FilterOnePole lowfilter5(LOWPASS, 0.15);
FilterOnePole lowfilter6(LOWPASS, 0.001);
FilterOnePole lowfilter7(LOWPASS, 0.001);
unsigned long led_off_ts;
unsigned long loop_ts;
bool flag_reset_point = false;
bool flag_led_on = false;
bool flag_calibrated = false;
bool flag_running = false;
bool flag_servo_protect = false;
bool no_spin = true;
float pitch_normalized;
float roll_normalized;
float rudder_normalized;
// where the monocopter is pointed, this is only a visual reference for pilot
// roll/pitch will be based on this angle
float ref_heading;
unsigned long last_mag_update_ts, prev_check_point = 0;
int flap_val;
float dotx, doty, corrected_dotx, corrected_doty, filtered_dotx, filtered_doty, max_doty, min_doty;
float heading, raw_heading, corrected_heading;
float phase_lag, raw_phase_lag, last_phase_lag;

void update_mag(){
  flag_reset_point = true;
  last_mag_update_ts = millis();
  flag_running = true;

  // servo actuation / buffered phase update
  if (x[1][0] > two_pi*8){
    // Serial.println(F("Warning: omega too high"));
    flag_servo_protect = true;
    disable_t1b();
    setPulseWidth(fmap(float(rc_in_val[3]),VR_MIN,VR_MAX,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
    // digitalWrite(PIN_LED4,HIGH);
  } 
  else if (x[1][0]<two_pi) {
    // Serial.println(F("Omega too low, servo disabled"));
    flag_servo_protect = true;
    disable_t1b();
    setPulseWidth(fmap(float(rc_in_val[3]),VR_MIN,VR_MAX,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
    // digitalWrite(PIN_LED4,HIGH);
  } 
  else {
    // suitable omega for control
    if (flag_servo_protect){
      // disable ''failsafe''
      // Serial.println(F("Resume Servo Control"));
      flag_servo_protect = false;
      pending_action_t1b = RISING_NEUTRAL;
      enable_t1b();
    }

    cli();
    state_buffer_ts = TCNT1;
    state_buffer[0] = x[0][0];
    state_buffer[1] = x[1][0];
    sei();
  }

  // Serial.println("mag update--");
  // Setup LED to blink when azimuth = ref_heading
  if (!flash_in_progress){
    // time needed to travel 30 degrees, so that light will be on for 30 deg
    on_time_ms = 30.0/180.0*pi/x[1][0]*1000.0;
    // put a lower limit
    on_time_ms = (on_time_ms<20) ? 20:on_time_ms;
    float t1a_delay = (two_pi - ffmod(ref_heading-x[0][0], two_pi)) / x[1][0] * 1000.0;
    if (t1a_delay>0){
      flash_in_progress = true;
      pending_action_t1a = LED_ON;
      enable_t1a();
      next_action_t1a(t1a_delay);
    }
  }
}

// Actual Loop
void loop() {
  sCmd.readSerial();
  // block -- serial update
  static int serial_update_freq = 10;
  static unsigned long serial_update_ts = millis();
  // RC Status
  if ( (millis() - serial_update_ts) > (unsigned long) (1000 / float(serial_update_freq)) ) {
    serial_update_ts = millis();
    if (rc_verbose && human_readable_output) {
      // Serial.print(F("RC in : ch0 : "));
      // Serial.print(rc_in_val[0]);
      // Serial.print(F(" | ch1 : "));
      // Serial.print(rc_in_val[1]);
      // Serial.print(F(" | ch2 : "));
      // Serial.print(rc_in_val[2]);
      // Serial.print(F(" | ch3 : "));
      // Serial.print(rc_in_val[3]);
      // Serial.print(F(" -> "));
      // Serial.println(fmap(float(rc_in_val[3]),VR_MIN,VR_MAX,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
    }
  }
  // block ----

  // Read Magnetometer
  static int mag_update_freq = 10;
  static unsigned long mag_update_ts = millis();
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }
  
  if ( (millis() - mag_update_ts) > (unsigned long) (1000 / float(mag_update_freq)) ) {

      // Print mag values in degree/sec
      if (mag_verbose && human_readable_output) {
        // Serial.print(F("X-mag field: ")); // Serial.print(myIMU.mx);
        // Serial.print(F(" mG "));
        // Serial.print(F("Y-mag field: ")); // Serial.print(myIMU.my);
        // Serial.print(F(" mG "));
        // Serial.print(F("Z-mag field: ")); // Serial.print(myIMU.mz);
        // Serial.println(F(" mG"));
      } 
  }
  //block ----------

  /*
  // block -- Xbee rssi update
  static int rssi_update_freq = 10;
  static unsigned long rssi_update_ts = millis();
  if ( (millis() - rssi_update_ts) > (unsigned long) (1000 / float(rssi_update_freq)) ) {
    if (rssi_verbose && human_readable_output){
      if (xbee_rssi_newdata){
        // Serial.print(F("RSSI : "));
        // Serial.println(xbee_rssi_val);
        xbee_rssi_newdata = false;
      } 
      else{
        // Serial.println(F("No RSSI update"));
      }
    }
  }

  // block -- Accelerometer update
  static int accel_update_freq = 10;
  static unsigned long accel_update_ts = millis();
  /* Get a new sensor event */
  /*
  sensors_event_t event_in;
  accel_in.getEvent(&event_in);

  // repeat for the outer one
  /* Get a new sensor event */
  /*
  sensors_event_t event_out;
  accel_out.getEvent(&event_out);
  
  if ( (millis() - accel_update_ts) > (unsigned long) (1000 / float(accel_update_freq)) ) {
    accel_update_ts = millis();

    if (acc_verbose && human_readable_output) {
      /* Display the results (acceleration is measured in m/s^2) */
      /*
      // Serial.print(F("Innter - X: ")); // Serial.print(event_in.acceleration.x); // Serial.print(F("  "));
      // Serial.print(F("Y: ")); // Serial.print(event_in.acceleration.y); // Serial.print(F("  "));
      // Serial.print(F("Z: ")); // Serial.print(event_in.acceleration.z); // Serial.print(F("  ")); // Serial.print(F("m/s^2      "));
    }


    if (acc_verbose && human_readable_output) {
      /* Display the results (acceleration is measured in m/s^2) */
      /*
      // Serial.print(F("outer X: ")); // Serial.print(event_out.acceleration.x); // Serial.print(F("  "));
      // Serial.print(F("Y: ")); // Serial.print(event_out.acceleration.y); // Serial.print(F("  "));
      // Serial.print(F("Z: ")); // Serial.print(event_out.acceleration.z); // Serial.print(F("  ")); // Serial.println(F("m/s^2 "));
    }
  }

  // block -- rc out
  static int rc_out_freq = 100;
  static unsigned long rc_out_ts = millis();
  if ( (millis() - rc_out_ts) > (unsigned long) (1000 / float(rc_out_freq)) ) {
    rc_out_ts = millis();
//    flap.writeMicroseconds(map(rc_in_val[0], 1000, 2000, 260, 1260));
//    esc.writeMicroseconds(rc_in_val[1]);
  }
  // block ----


  //block -- loss of signal
  static int loss_of_signal_freq = 10;
  static unsigned long loss_of_signal_ts = millis();
  if ( (millis() - loss_of_signal_ts) > (unsigned long) (1000 / float(loss_of_signal_freq)) ) {
    loss_of_signal_ts = millis();

    // detect loss of signal (works with FHSS on a channel without fail safe enabled
    unsigned long us_since_last_rc_in = micros() - rc_rising_ts[0];
    if (us_since_last_rc_in > 500000) {
      if (sig_verbose && human_readable_output){
        // Serial.println(F("Signal loss"));
      }
      flag_signal_loss = true;
    } else {
      flag_signal_loss = false;
    }
  }
  // block ---

  // data processing and visualization
  kf_predict();

/*
  // acc update
  // WARNING FIXME two acc not aligned, implement same algo used in reader.py
  float delta_acc = acc_norm(&event_out)-acc_norm(&event_in);
  if (x[1][0]>two_pi){
    kf_update_acc(delta_acc);
  }
  */

  float m_norm = sqrt(myIMU.mx*myIMU.mx + myIMU.my*myIMU.my + myIMU.mz*myIMU.mz);
  dotx = myIMU.mx/m_norm * 300; // Scale to ease plotting
  doty = myIMU.my/m_norm * 300;

//  // Check of the vehicle is not spinning
//  if ((millis() - prev_check_point) > 2000 && prev_check_point > 1000){
//    if (max_doty - min_doty < 150)
//      no_spin = true;
//    else
//      no_spin = false;
//    prev_check_point = millis();
//    max_doty = doty - 30;
//    min_doty = doty + 30;
//  }
//  else{
//    if (doty > max_doty)
//      max_doty = doty;
//    else if (doty < min_doty)
//      min_doty = doty;
//  }
//
//  // Correct mag bias
//  if (no_spin == false){
//    lowfilter6.input(dotx);
//    lowfilter7.input(doty);
//  }
//  else{
//    lowfilter6.input(lowfilter6.output());
//    lowfilter7.input(lowfilter7.output());
//  }
//  corrected_dotx = dotx - lowfilter6.output();
//  corrected_doty = doty - lowfilter7.output();

  corrected_dotx = dotx;
  corrected_doty = doty;
  
  raw_heading = atan(corrected_doty / corrected_dotx) / two_pi * 360.;
  if (corrected_dotx < 0){
    if (corrected_doty > 0) // 90 ~ 180
      raw_heading += 180;
    else // -180 ~ -90
      raw_heading -= 180;
  }
  
  lowfilter1.input(corrected_doty);
  lowfilter2.input(lowfilter1.output());
  lowfilter3.input(corrected_dotx);
  lowfilter4.input(lowfilter3.output());
  filtered_dotx = lowfilter4.output() * 1.6;
  filtered_doty = lowfilter2.output() * 1.6;
  
  heading = atan (filtered_doty / filtered_dotx) / two_pi * 360.;
  if (filtered_dotx < 0){
    if (filtered_doty > 0) // 90 ~ 180
      heading += 180;
    else                 // -180 ~ -90
      heading -= 180;
  }

  // Makes sure that phase lag is always positive
  last_phase_lag = raw_phase_lag;
  raw_phase_lag = raw_heading - heading;
  if ((raw_phase_lag < 0 && raw_phase_lag - last_phase_lag < -90) || raw_phase_lag < -220)
    raw_phase_lag += 360;
  if ( (raw_heading < -120 || raw_heading > 120) && raw_phase_lag > 240)
    raw_phase_lag -= 360;
    
  // Filter phase lag
  lowfilter5.input(raw_phase_lag);
  phase_lag = lowfilter5.output();
  phase_lag = phase_lag * 1.86 - sq(phase_lag) * 0.0024;

  corrected_heading = heading + phase_lag;
  if (corrected_heading > 180)
    corrected_heading -= 360;

  // Light up the LEDs when the heading is between 55 deg and 65 deg
  if (corrected_heading > 55 && corrected_heading < 65)
    PORTB &= B100011; // Turn on LEDs
  else
    PORTB |= B011100; // Turn off LEDs

/*
  // Serial.print(corrected_doty);
  Serial.print(" -180 180 ");
  // Serial.print(lowfilter2.output() * 1.6);
  // Serial.print(corrected_dotx);
  // Serial.print(" ");
  // Serial.print(no_spin * 100);
  // Serial.print(" ");
  // Serial.println(doty);
  Serial.print(raw_heading);
  Serial.print(" ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(corrected_heading);
  Serial.print(" ");
  Serial.print(raw_phase_lag);
  Serial.print(" ");
  Serial.println(phase_lag);
  */
  
  // set target phase to pilot input
  // ctrl_phase: phase to START command to max deflection
  // volatile float ctrl_phase;
  // the control space is a 2 by 2 square (roll -1~1, pitch -1~1)
  // the actual output is a circle, inner tangent to the control space, so control 
  // will saturate at around 70% when going diagonally, this may result in undesirable handling characteristics

  /*
  // first find direction
  roll_normalized = fmap(float(rc_in_val[0]),ROLL_FULL_LEFT,ROLL_FULL_RIGHT,-1.0,1.0);
  pitch_normalized = fmap(float(rc_in_val[1]),PITCH_FULL_PULL,PITCH_FULL_PUSH,-1.0,1.0);
  rudder_normalized = fmap(float(rc_in_val[2]),RUDDER_FULL_LEFT,RUDDER_FULL_RIGHT,-1.0,1.0);
  // target control : at max rudder, heading change 150deg/sec, loop runs 50Hz, so
  // 1.5*pi/100 (unit: rad/loop)
  // apply a 5% deadzone on rudder so it doesn't drift
  if (abs(rudder_normalized)>0.1){
    ref_heading -= rudder_normalized * 1.5*pi/100.0;
    ref_heading = ffmod(ref_heading,two_pi);
  }
  // positive direction is the monocopter's spin direction, here its clock-wise
  ctrl_phase = ref_heading + atan2(roll_normalized,pitch_normalized);
  ctrl_magnitude = sqrt(roll_normalized*roll_normalized+pitch_normalized*pitch_normalized);
  ctrl_magnitude = ctrl_magnitude>1.0?1.0:ctrl_magnitude;

  // machine readable output
  if (!human_readable_output){
    //mag_x,y,z,(inner)acc1_x,y,z,(outer)acc2_x,y,z,
    // signify this is a line intended for machine parsing

    // Serial.print('#');

    // Serial.print(millis()-epoch);
    // Serial.print(",");
    // Serial.print(myIMU.mx,2);
    // Serial.print(",");
    // Serial.print(myIMU.my,2);
    // Serial.print(",");
    // Serial.print(myIMU.mz,2);
    // Serial.print(",");
    
    // Serial.print(event_in.acceleration.x,2);
    // Serial.print(",");
    // Serial.print(event_in.acceleration.y,2);
    // Serial.print(",");
    // Serial.print(event_in.acceleration.z,2);
    // Serial.print(",");

    // Serial.print(event_out.acceleration.x,2);
    // Serial.print(",");
    // Serial.print(event_out.acceleration.y,2);
    // Serial.print(",");
    // Serial.print(event_out.acceleration.z,2);

    // Serial.print(",");
    // unit: degree
    // may not need
    kf_predict();
    long offset = (long)(x[0][0]/pi*180)/360*360;
    // Serial.print(x[0][0]/pi*180-offset);
    // Serial.print(",");
    //unit: rev/s
    // Serial.println(x[1][0]/2.0/pi);

    //// Serial.println(millis()-loop_ts);
    loop_ts = millis();
  }
  
    flag_reset_point = false;
    // limit transmission rate to 50Hz
    while (millis()-loop_ts < 20){
      delayMicroseconds(100);
    }
      if (millis()-last_mag_update_ts>2000){
        x[1][0] = 0;
        x[0][0] = 0;
        P[0][0] = 0;
        P[1][0] = 0;
        P[0][1] = 0;
        P[1][1] = 0;
        if (flag_running){
          // Serial.println("Stopped");
          flag_running = false;
          // Serial.println(F("servo disabled"));
          flag_servo_protect = true;
          disable_t1b();
          digitalWrite(PIN_LED4,HIGH);
        }
      }
    
    if (!flag_running){
       //// Serial.println(fmap(float(rc_in_val[3]),VR_MIN,VR_MAX,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH));
       int newval = fmap(float(rc_in_val[3]),VR_MIN,VR_MAX,MIN_SERVO_PULSEWIDTH,MAX_SERVO_PULSEWIDTH);
       if (abs(newval-flap_val)>5){
         setPulseWidth(newval);
       }
    }
    */
}
