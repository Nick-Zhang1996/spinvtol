// monocopter controller, for teensy 3.5
// this firmware works with phase_gs.py in monocopter/mono
// it uses the 11 byte protocol that encodes throttle(pwm), max flap(pwm),phase(4-byte float)
// Nick Zhang 2019


// Pin connection table:
// most of these can be changed, though there are some restrictions
// LED1,LED2,LED3, LED4(blue): 9,10,11,12
// RSSI level from xbee: 3
// RC channel PWM input: 4,5,6,7,8, connect to Ch1,3,4,6 on the RC receiver
// flap servo output: 29
// throttle  output(reserved, not used): 30

// Communication
// see protocol.txt
#define BUFFER_SIZE 11

// Binary protocol
// normal update
#define MSG_UPDATE 1
// ping response, remaining data in packet undefined
#define MSG_PING 2
#define MSG_ERR 3

#define PIN_LED 13
// 3 red led array
#define PIN_LED1 9
#define PIN_LED2 10
#define PIN_LED3 11
// blue
#define PIN_LED4 12

#define PIN_FLAP_SERVO 29
#define PIN_THROTTLE_SERVO 30
#define PIN_VOLTAGE A3
// resistor values for voltage divider
#define R_VOLTAGE_UP 22e3
#define R_VOLTAGE_DOWN 4.7e3
#define VOLTAGE_BUFFER_LEN 100


// action sequence for T1_COMPA
#define NONE 0
#define LED_OFF 1
#define LED_ON 2

// action sequence for T1_COMPB
#define RISING_NEUTRAL 1
#define SERVO_MAX 2
#define FALLING_NEUTRAL 3
#define SERVO_MIN 4

#define CHANNEL_NO 6

// up/down range from neutral : 185us
#define MAX_FLAP_SERVO_PULSEWIDTH 930
#define MIN_FLAP_SERVO_PULSEWIDTH 560
#define NEUTRAL_FLAP_SERVO_PULSEWIDTH 745

#define MAX_THROTTLE_SERVO_PULSEWIDTH 1939
#define MIN_THROTTLE_SERVO_PULSEWIDTH 1098

// roll-ch1(pcb)->ch0(array index), pitch ch2->ch1, throttle ch3->ch2, rudder ch4->ch3, aux ch5->ch4
#define PITCH_FULL_PULL 1939.0
#define PITCH_FULL_PUSH 1098.0
#define ROLL_FULL_LEFT 1096.0
#define ROLL_FULL_RIGHT 1938.0
#define RUDDER_FULL_LEFT 1100.0
#define RUDDER_FULL_RIGHT 1940.0

#define VR_MIN 1099.0
#define VR_MAX 1939.0

#include <Wire.h>
#include <Filters.h>
#include <MatrixMath.h>
// uses FTM0 timer by default, interferes with PWM function on some pins, see https://www.pjrc.com/teensy/td_pulse.html
#include <TeensyDelay.h>

#define I2Cclock 400000
#define I2Cport Wire
#include <MPU9250.h>
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 // AD0 on MPU9250 pulled down to GND

typedef mtx_type matrix;
#define multiply(A,B,m,n,p,C) Matrix.Multiply((mtx_type*)A, (mtx_type*)B, m, n, p, (mtx_type*)C)
#define add(A,B,m,n,C) Matrix.Add((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define subtract(A,B,m,n,C) Matrix.Subtract((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define transpose(A,m,n,C) Matrix.Transpose((mtx_type*)A, m, n, (mtx_type*)C)
#define invert(A,m) Matrix.Invert((mtx_type*)A,m)
#define mtxprint(A,m,n,N) Matrix.Print((mtx_type*)A, m, n, N)
#define mtxcopy(A,m,n,B) Matrix.Copy((mtx_type*)A, m, n, (mtx_type*)B)
float dot;

// MPU9250 init sets Wire I2c freq(adxl345 does not), let it run first
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

const float pi = 3.1415926535898;
const float two_pi = 2*3.1415926535898;

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
  return x-y*floor(x/y);
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


uint8_t rc_in_pinno[] = {4, 5, 6, 7, 8, 24}; // CH0,1,2,3,4,5(soldered on)
volatile int rc_in_val[CHANNEL_NO] = {0};
volatile unsigned long rc_rising_ts[CHANNEL_NO] = {0};
bool flag_signal_loss = false;

//note: measured dutycycle for futaba FHSS 13564us, 73Hz
void rc0_in_callback() {
  unsigned long timestamp = micros();
  if (digitalRead(rc_in_pinno[0]) == HIGH) {
    rc_rising_ts[0] = timestamp;
  } else {
    rc_in_val[0] = timestamp - rc_rising_ts[0];
  }
}
void rc1_in_callback() {
  unsigned long timestamp = micros();
  if (digitalRead(rc_in_pinno[1]) == HIGH) {
    rc_rising_ts[1] = timestamp;
  } else {
    rc_in_val[1] = timestamp - rc_rising_ts[1];
  }
}
void rc2_in_callback() {
  unsigned long timestamp = micros();
  if (digitalRead(rc_in_pinno[2]) == HIGH) {
    rc_rising_ts[2] = timestamp;
  } else {
    rc_in_val[2] = timestamp - rc_rising_ts[2];
  }
}
void rc3_in_callback() {
  unsigned long timestamp = micros();
  if (digitalRead(rc_in_pinno[3]) == HIGH) {
    rc_rising_ts[3] = timestamp;
  } else {
    rc_in_val[3] = timestamp - rc_rising_ts[3];
  }
}
void rc4_in_callback() {
  unsigned long timestamp = micros();
  if (digitalRead(rc_in_pinno[4]) == HIGH) {
    rc_rising_ts[4] = timestamp;
  } else {
    rc_in_val[4] = timestamp - rc_rising_ts[4];
  }
}

void rc5_in_callback() {
  unsigned long timestamp = micros();
  if (digitalRead(rc_in_pinno[5]) == HIGH) {
    rc_rising_ts[5] = timestamp;
  } else {
    rc_in_val[5] = timestamp - rc_rising_ts[5];
  }
}

// timer routines
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

volatile uint32_t oldres_flap;
volatile int duty_flap;
void setFlapServoPulseWidth(float us)
{
  if (us<MIN_FLAP_SERVO_PULSEWIDTH) { us = MIN_FLAP_SERVO_PULSEWIDTH; }
  if (us>MAX_FLAP_SERVO_PULSEWIDTH) { us = MAX_FLAP_SERVO_PULSEWIDTH; }
	duty_flap = us/(1e6/300.0)*65535;
#if TEENSYDUINO >= 137
	noInterrupts();
	oldres_flap = analogWriteResolution(16);
	analogWrite(PIN_FLAP_SERVO, duty_flap);
  // restore resolution
	analogWriteResolution(oldres_flap);
	interrupts();
#endif
}

volatile uint32_t oldres_throttle;
volatile int duty_throttle;
void setThrottleServoPulseWidth(float us)
{
  if (us<MIN_THROTTLE_SERVO_PULSEWIDTH) { us = MIN_THROTTLE_SERVO_PULSEWIDTH; }
  if (us>MAX_THROTTLE_SERVO_PULSEWIDTH) { us = MAX_THROTTLE_SERVO_PULSEWIDTH; }
	duty_throttle = us/(1e6/300.0)*65535;
#if TEENSYDUINO >= 137
	noInterrupts();
	oldres_throttle = analogWriteResolution(16);
	analogWrite(PIN_THROTTLE_SERVO, duty_throttle);
  // restore resolution
	analogWriteResolution(oldres_throttle);
	interrupts();
#endif
}

// ctrl_phase: phase to START command to max deflection
volatile float ctrl_phase;
// us to next control phase, time between each new ISR
volatile float quarter_period;
volatile float current_phase;
volatile int pending_action_cyclic = NONE;
volatile float ctrl_magnitude;
// TeensyDelay uses 16bit FTM timer that only goes to about 69ms, if we need a delay longer
// than that we need to handle this gracefully
volatile uint32_t remaining_delay_us;
volatile float corrected_heading,corrected_heading_ts_us;

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
          //Serial1.println("RISING_NEUTRAL");
          setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
          //setFlapServoPulseWidth(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
          pending_action_cyclic = SERVO_MAX;
          // adjust control phase to sync with estimated state
          // 5e5 = 2(for 2pi) * 1e6 (sec -> us) / 4 (quarter period)
          quarter_period = 5e5*pi/rotation_speed_rads;
          current_phase = corrected_heading + float(micros()-corrected_heading_ts_us)*1e-6*rotation_speed_rads;
          remaining_delay_us = (ffmod(ctrl_phase-current_phase,2*pi)/rotation_speed_rads)*1e6;
          break;
          
      case SERVO_MAX:
          //Serial1.println("SERVO_MAX");
          //setFlapServoPulseWidth(fmap(ctrl_magnitude,0.0,1.0,NEUTRAL_FLAP_SERVO_PULSEWIDTH, MAX_FLAP_SERVO_PULSEWIDTH));
          setFlapServoPulseWidth(remote_flap+NEUTRAL_FLAP_SERVO_PULSEWIDTH)
          pending_action_cyclic = FALLING_NEUTRAL;
          remaining_delay_us = quarter_period;
          break;

      case FALLING_NEUTRAL:
          //Serial1.println("FALLING_NEUTRAL");
          //setFlapServoPulseWidth(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
          setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
          pending_action_cyclic = SERVO_MIN;
          remaining_delay_us = quarter_period;
          break;

      case SERVO_MIN:
          //Serial1.println("SERVO_MIN");
          //setFlapServoPulseWidth(fmap(ctrl_magnitude,0.0,1.0,NEUTRAL_FLAP_SERVO_PULSEWIDTH, MIN_FLAP_SERVO_PULSEWIDTH));
          setFlapServoPulseWidth(remote_flap-NEUTRAL_FLAP_SERVO_PULSEWIDTH)
          pending_action_cyclic  = RISING_NEUTRAL;
          remaining_delay_us = quarter_period;
          break;

      case NONE:
      default:
          //Serial1.println("fallthrough");
          //setFlapServoPulseWidth(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
          setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
          return;
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

const int xbee_rssi_pin = 3;
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

// update serial remote control command
// this is the processor for host computer generated control command
volatile bool manual_control = true;
volatile unsigned long remote_ts = 0; // use a timestamp to monitor if up to date control msg is available
volatile int remote_throttle = MIN_THROTTLE_SERVO_PULSEWIDTH;
volatile int remote_flap = 0;
volatile int remote_buffer = MIN_THROTTLE_SERVO_PULSEWIDTH;

uint8_t buffer[BUFFER_SIZE] = {0};
uint8_t buffer_index = 0;
union{
  float val;
  unint8_t bytes[4];
} float_helper;

// 8 bytes
struct monomsg{
    uint8_t msg_type;
    // 1: telemetry control, 2: RC control
    uint8_t telem_ctrl;
    // voltage * 100
    uint16_t voltage;
    uint16_t flapPWM;
    uint16_t throttlePWM;
};

void parseSerial(){
  // incoming packet length is 7
  // Byte 0,1: 9, used for detecting misaligned data
  // Byte 2: msg type 1->control update 2-> ping request, ignore following packet
  // 3-4: flapPWM
  // 5-6: throttlePWM
  // 7-10: phase at which to start moving to max deflection
  while (Serial1.available()>0){
      buffer[buffer_index%BUFFER_SIZE] = Serial1.read();
      buffer_index++;
  }

  if (buffer_index>=BUFFER_SIZE){
      // the package is not properly aligned
      if (not (buffer[0]==9 and buffer[1]==9)){
          // drop the first byte in buffer and shift remaining data forward
          for (int i=0;i<BUFFER_SIZE-1;i++){
              buffer[i] = buffer[i+1];
          }
          buffer_index--;
      } else{
        // process
        buffer_index = 0;

        int msgType = buffer[2];
        if (msgType==1){
          remote_flap = ((uint16_t)buffer[4]<<8)+buffer[3];
          remote_flap = (remote_flap>185)?185:remote_flap;
          remote_flap = (remote_flap<0)?0:remote_flap;

          remote_throttle = ((uint16_t)buffer[6]<<8)+buffer[5];
          remote_throttle = (remote_throttle>MAX_THROTTLE_SERVO_PULSEWIDTH)?MAX_THROTTLE_SERVO_PULSEWIDTH:remote_throttle;
          remote_throttle = (remote_throttle<MIN_THROTTLE_SERVO_PULSEWIDTH)?MIN_THROTTLE_SERVO_PULSEWIDTH:remote_throttle;
          float_helper.bytes[0] = buffer[7];
          float_helper.bytes[1] = buffer[8];
          float_helper.bytes[2] = buffer[9];
          float_helper.bytes[3] = buffer[10];
          remote_phase = float_helper.val;

          ctrl_phase = remote_phase + pi/2;
        }

        if (msgType==2){
          struct monomsg packet;
          packet.msg_type = MSG_PING;
          Serial1.write((const uint8_t *)&packet,sizeof(struct monomsg));
          digitalWrite(PIN_LED,HIGH);

        }

        remote_ts = millis();
      }
  }
}

FilterTwoPole lowfilter1(2.8, 0.35);
FilterTwoPole lowfilter2(5, 0.5);
FilterTwoPole lowfilter3(2.8, 0.35);
FilterTwoPole lowfilter4(5, 0.5);
FilterOnePole lowfilter5(LOWPASS, 0.15);
FilterOnePole lowfilter6(LOWPASS, 0.001);
FilterOnePole lowfilter7(LOWPASS, 0.001);
float dotx, doty, corrected_dotx, corrected_doty, filtered_dotx, filtered_doty, max_doty, min_doty;
float heading, raw_heading;
float phase_lag, raw_phase_lag, last_phase_lag;

// estimate speed from multiple updates
int speed_sample_count;
unsigned long est_speed_start_ts;
float rotation_speed_rads;
float last_heading,angle_traveled;

// are we rotating ccw or cw
// TODO verify this, assuming rotatin in mz axis
void update_mag(){
  float m_norm = sqrt(myIMU.mx*myIMU.mx + myIMU.my*myIMU.my + myIMU.mz*myIMU.mz);
  dotx = myIMU.mx/m_norm;
  doty = myIMU.my/m_norm;

  corrected_dotx = dotx;
  corrected_doty = doty;
  
  raw_heading = atan2(corrected_doty , corrected_dotx);
  
  lowfilter1.input(corrected_doty);
  lowfilter2.input(lowfilter1.output());
  lowfilter3.input(corrected_dotx);
  lowfilter4.input(lowfilter3.output());
  filtered_dotx = lowfilter4.output() * 1.6;
  filtered_doty = lowfilter2.output() * 1.6;
  
  heading = atan2(filtered_doty , filtered_dotx);

  // Makes sure that phase lag is always positive
  raw_phase_lag = raw_heading - heading;
  // TODO verify this is correct
  // wrap to (0,2pi)
  raw_phase_lag = ffmod(raw_phase_lag + two_pi, two_pi);
    
  // Filter phase lag
  lowfilter5.input(raw_phase_lag);
  phase_lag = lowfilter5.output();
  phase_lag = phase_lag * 1.86 - sq(phase_lag) * 0.0024;

  corrected_heading = heading + phase_lag;
  // wrap to (0,2pi)
  corrected_heading = ffmod(corrected_heading + two_pi, two_pi);
  corrected_heading_ts_us = micros();

  speed_sample_count++;
  // TODO verify rotation angle direction, here we assume positive
  angle_traveled += ffmod((corrected_heading - last_heading + two_pi), two_pi);


  // TODO how long is this? is it reasonable?
  if (speed_sample_count == 30){
    speed_sample_count = 0;
    rotation_speed_rads = angle_traveled / (((float)(millis()-est_speed_start_ts))/1000.0)
    angle_traveled = 0;
    est_speed_start_ts = millis()
  }

  last_heading = corrected_heading;

}

// unrecoverable error
// notify ground station
void sendErr(){
    struct monomsg packet;
    packet.msg_type = MSG_ERR;
    Serial1.write((const uint8_t *)&packet,sizeof(struct monomsg));
}

void setup() {
  Serial1.begin(115200);
  //while (!Serial);

  for (uint8_t i = 0; i < sizeof(rc_in_pinno) / sizeof(uint8_t);  i++) {
    pinMode(rc_in_pinno[i], INPUT);
  }
  pinMode(PIN_VOLTAGE,INPUT);

  attachInterrupt( digitalPinToInterrupt(xbee_rssi_pin), xbee_rssi_callback,CHANGE);
  attachInterrupt( digitalPinToInterrupt(rc_in_pinno[0]), rc0_in_callback, CHANGE);
  attachInterrupt( digitalPinToInterrupt(rc_in_pinno[1]), rc1_in_callback, CHANGE);
  attachInterrupt( digitalPinToInterrupt(rc_in_pinno[2]), rc2_in_callback, CHANGE);
  attachInterrupt( digitalPinToInterrupt(rc_in_pinno[3]), rc3_in_callback, CHANGE);
  attachInterrupt( digitalPinToInterrupt(rc_in_pinno[4]), rc4_in_callback, CHANGE);
  attachInterrupt( digitalPinToInterrupt(rc_in_pinno[5]), rc5_in_callback, CHANGE);

  // ---------------- MPU9250 init ---------------
  Wire.begin();
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial1.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0x48)
    {
      sendErr();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

  } else
  {
    sendErr();
    abort();
  }

  // --------  Register serial commands ------
  pinMode(13, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED4, OUTPUT);
  pinMode(PIN_FLAP_SERVO,OUTPUT);
  digitalWrite(PIN_LED1,HIGH);
  digitalWrite(PIN_LED2,HIGH);
  digitalWrite(PIN_LED3,HIGH);
  digitalWrite(PIN_LED4,HIGH);
  digitalWrite(PIN_FLAP_SERVO,LOW);

  pinMode(synchro_pinno,OUTPUT);
  digitalWrite(synchro_pinno,HIGH);

  analogWriteFrequency(PIN_FLAP_SERVO, 300);
  analogWriteFrequency(PIN_THROTTLE_SERVO, 300);
  pending_action_cyclic = RISING_NEUTRAL;
  TeensyDelay::begin();
  TeensyDelay::addDelayChannel(cyclic, 0);
  TeensyDelay::trigger(10,0);
  
}
// --------------------- LOOP ---------------------

float mag_hist[4];
unsigned long led_off_ts;
unsigned long main_loop_ts;
float main_loop_hz = 0.0;
bool flag_led_on = false;
bool flag_calibrated = false;
bool flag_running = false;
float pitch_normalized;
float roll_normalized;
float rudder_normalized;
// where the monocopter is pointed, this is only a visual reference for pilot
// roll/pitch will be based on this angle
float ref_heading;
unsigned long last_mag_update_ts;
void loop() {
  main_loop_hz = 1000.0/(millis()-main_loop_ts);
  main_loop_ts = millis();

  //block -- manual/telemetry control switch, no signal loss
  // TODO, test the actual switching PWM
  if (!flag_signal_loss){
    if (rc_in_val[5]<1500 and !manual_control){ // switch to manual mode
      manual_control = true;
      pending_action_cyclic = NONE;

    } else if (rc_in_val[5]>1500 and manual_control){ // switch to autonomous mode
      // ensure a recent remote control command is present
      // this avoids accidently switching control to remote when remote is not ready
      if ((millis()-remote_ts)<50){
        manual_control = false;
        pending_action_cyclic = RISING_NEUTRAL;
      } 
    }
  }
  //block --

  //block -- loss of signal
  static int loss_of_signal_freq = 10;
  static unsigned long loss_of_signal_ts = millis();
  if ( (millis() - loss_of_signal_ts) > (unsigned long) (1000 / float(loss_of_signal_freq)) ) {
    loss_of_signal_ts = millis();

    // detect loss of signal (works with FHSS on a channel with fail safe DISABLED)
    unsigned long us_since_last_rc_in = micros() - rc_rising_ts[0];
    if (us_since_last_rc_in > 500000) {
      if (!flag_signal_loss){ // enter failsafe, disable motor and set flap to neutral
        //Serial1.println(F("Signal loss - Failsafe enabled"));
        flag_signal_loss = true;
        setThrottleServoPulseWidth(MIN_THROTTLE_SERVO_PULSEWIDTH);
        setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
      }
    } else if (flag_signal_loss){ // recover from failsafe 
      flag_signal_loss = false;
      // Autonomous control will automatically resume by autonomous block
      // Manual throttle control will resume automatically by throttle block
      // Manual flap control need to be re-enabled here
      pending_action_cyclic = RISING_NEUTRAL;
      //Serial1.println(F("RC signal recovered"));
    }
  }
  // block ---

  //block -- autonomous control
  static int remote_update_freq = 50;
  static unsigned long remote_update_ts = millis();
  if (!manual_control and !flag_signal_loss and (millis()-remote_ts<500) and((millis() - remote_update_ts) > (unsigned long) (1000 / float(remote_update_freq)))){
    remote_update_ts = millis();
    setThrottleServoPulseWidth(remote_throttle);
  }
  //block --

  //block -- manual control
  static int throttle_update_freq = 50;
  static unsigned long throttle_update_ts = millis();
  // the third condition may be redundent
  if ( !flag_signal_loss && manual_control && ((millis() - throttle_update_ts) > (unsigned long) (1000 / float(throttle_update_freq))) ) {
    throttle_update_ts = millis();
    setThrottleServoPulseWidth(rc_in_val[2]);
    setFlapServoPulseWidth(rc_in_val[4]);
  }
  //block -- 


  //block -- voltage update
  static int voltage_hist[VOLTAGE_BUFFER_LEN] = {0};
  static int p_voltage_hist = 0;
  static float voltage_running_avg = 0.0;
  static float voltage_sum = 0.0;
  voltage_hist[p_voltage_hist%VOLTAGE_BUFFER_LEN] = analogRead(PIN_VOLTAGE);
  p_voltage_hist++;
  voltage_sum = 0.0;
  for (int i=0;i<VOLTAGE_BUFFER_LEN;i++){
    voltage_sum += voltage_hist[i];
  }
  voltage_running_avg = (float)voltage_sum/(float)VOLTAGE_BUFFER_LEN/1023.0/R_VOLTAGE_DOWN*(R_VOLTAGE_DOWN+R_VOLTAGE_UP)*3.3;
  //block -- 

  //block -- magnetometer update
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
    update_mag();
  }
  //block ----------
  
  // enter low speed protection
  if (rotation_speed_rads<2*two_pi and flag_running){
    // Serial1.println("Stopped");
    flag_running = false;
    pending_action_cyclic = NONE;
    //setFlapServoPulseWidth(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
    setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
    digitalWrite(PIN_LED4,HIGH);
  }

  // recover from low speed protection
  if (rotation_speed_rads>2*two_pi and !flag_running){
    flag_running = true;
    pending_action_cyclic = RISING_NEUTRAL;
  }

  if (!flag_running){
    //Serial1.println(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
    //setFlapServoPulseWidth(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
    setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
  }

  // block -- Serial output
  static unsigned long seq_no = 0;
  static int serial_loop_freq = 50;
  static unsigned long serial_loop_ts = millis();
  if ( (millis() - serial_loop_ts) > (unsigned long) (1000 / float(serial_loop_freq)) ) {
    serial_loop_ts = millis();
    seq_no++;

    struct monomsg packet;
    packet.msg_type = MSG_UPDATE;
    packet.voltage = (uint16_t)(voltage_running_avg*100);
    if (manual_control){
      packet.flapPWM = (uint16_t)rc_in_val[4];
      packet.throttlePWM = (uint16_t)rc_in_val[2];
    } else {
      packet.flapPWM = (uint16_t)remote_flap;
      packet.throttlePWM = (uint16_t)remote_throttle;
    }
    packet.telem_ctrl = (manual_control)?0:1;
    Serial1.write((const uint8_t *)&packet,sizeof(struct monomsg));

  }
  // block --

}
