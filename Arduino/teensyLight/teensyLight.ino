// This firmware achieves the following things
// 1. Relay control from telemetry to flap and throttle servo
// 2. Send over Serial1 current voltage measurement
// 3. Allow switching from manual control(RC receiver reading) to telemetry control
// 4. Ping

// Manual control:
// CH2(rc receiver) ->

//#define DEBUG

#define REMOTE_MAX_DELAY_MS 55

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

// RC related
#define CHANNEL_NO 6

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

#include <SerialCommand.h>

// map from one range to another
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

// --------- Serial command handlers -------
SerialCommand sCmd(Serial1);
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

void led4() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(PIN_LED4, !rval);
  }
}

bool rc_verbose = false;
void display_rc() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    rc_verbose = (bool)rval;
  }
}

bool loophz_verbose = false;
void display_loop_freq() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    loophz_verbose = (bool)rval;
  }
}

bool remote_verbose = false;
void display_remote_sig_delay() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    remote_verbose = (bool)rval;
  }
}

void display_all() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    loophz_verbose = rc_verbose = (bool)rval;
  }
}

// response to ping 
void ping(){
  Serial1.println(F("$ping"));
}

// unrecognized comand response
void badcommand(const char *command){
    Serial1.print(F("Bad Command:"));
    Serial1.println(command);
    return;
}

// update serial remote control command
// this is the processor for host computer generated control command
volatile bool manual_control = true;
volatile unsigned long remote_ts = 0; // use a timestamp to monitor if up to date control msg is available
volatile int remote_throttle = MIN_THROTTLE_SERVO_PULSEWIDTH;
volatile int remote_flap = NEUTRAL_FLAP_SERVO_PULSEWIDTH;
volatile int remote_buffer = MIN_THROTTLE_SERVO_PULSEWIDTH;
void ctrl(){
  // throttle
  remote_buffer = atoi(sCmd.next());
  remote_buffer = (remote_buffer>MAX_THROTTLE_SERVO_PULSEWIDTH)?MAX_THROTTLE_SERVO_PULSEWIDTH:remote_buffer;
  remote_throttle = (remote_buffer<MIN_THROTTLE_SERVO_PULSEWIDTH)?MIN_THROTTLE_SERVO_PULSEWIDTH:remote_buffer;

  // flap
  remote_buffer = atoi(sCmd.next());
  remote_buffer = (remote_buffer>VR_MAX)?VR_MAX:remote_buffer;
  remote_flap = (remote_buffer<VR_MIN)?VR_MIN:remote_buffer;

  remote_ts = millis();

#ifdef DEBUG
  Serial1.print(F("R T:"));
  Serial1.print(remote_throttle);
  Serial1.print(F("F:"));
  Serial1.println(remote_flap);
#endif

}

void setup() {
  Serial1.begin(115200);

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
  // --------  Register serial commands ------
  pinMode(13, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED4, OUTPUT);
  pinMode(PIN_FLAP_SERVO,OUTPUT);
  pinMode(PIN_THROTTLE_SERVO,OUTPUT);
  digitalWrite(PIN_LED1,HIGH);
  digitalWrite(PIN_LED2,HIGH);
  digitalWrite(PIN_LED3,HIGH);
  digitalWrite(PIN_LED4,HIGH);
  digitalWrite(PIN_FLAP_SERVO,LOW);
  digitalWrite(PIN_THROTTLE_SERVO,LOW);

  sCmd.addCommand("led", led); // test function, takes 1 argument on or off
  sCmd.addCommand("led1", led1); // test function, takes 1 argument on or off
  sCmd.addCommand("led2", led2); // test function, takes 1 argument on or off
  sCmd.addCommand("led3", led3); // test function, takes 1 argument on or off
  sCmd.addCommand("led4", led4); // test function, takes 1 argument on or off
  sCmd.addCommand("rc", display_rc);
  //sCmd.addCommand("rssi", display_rssi);
  sCmd.addCommand("loophz", display_loop_freq);
  sCmd.addCommand("all", display_all);
  sCmd.addCommand("ping",ping);
  sCmd.addCommand("ctrl",ctrl);
  sCmd.addCommand("remote",display_remote_sig_delay);
  sCmd.setDefaultHandler(badcommand);
  analogWriteFrequency(PIN_FLAP_SERVO, 300);
  analogWriteFrequency(PIN_THROTTLE_SERVO, 300);
}

unsigned long main_loop_ts;
float main_loop_hz = 0.0;
float main_loop_hz_array[50] = {0};
int p_main_loop_hz_array = 0;
unsigned long remote_delay_ms_array[50] = {0};
int p_remote_delay_ms_array = 0;

void loop() {
  main_loop_hz = 1000000.0/float(micros()-main_loop_ts);
  main_loop_ts = micros();
  main_loop_hz_array[p_main_loop_hz_array%50] = main_loop_hz;
  p_main_loop_hz_array++;

  // process commandd
  sCmd.readSerial();

  //block -- manual/telemetry control switch
  // TODO, test the actual switching PWM
  static bool flag_unable_to_switch = false;
  unsigned long remote_signal_delay = 0;
  remote_signal_delay = millis()-remote_ts;
  remote_delay_ms_array[p_remote_delay_ms_array++] = remote_signal_delay;
  if (!flag_signal_loss){
    if (rc_in_val[5]>1300){ // switch to manual mode
        if (!manual_control){ // if current mode is automatic, switch to manual
          manual_control = true;
          Serial1.println(F("Manual Override"));
        } else {
            flag_unable_to_switch = false; // this flag should stay false in manual mode
        }
    } else if (rc_in_val[5]<1300){ // switch to autonomous mode
        if (manual_control){
          // ensure a recent remote control command is present
          // this avoids accidently switching control to remote when remote is not ready
          if (remote_signal_delay<REMOTE_MAX_DELAY_MS){
            manual_control = false;
            Serial1.println(F("Autonomous Control"));
          } else {
            // if user request autonomous mode but there's no autonomous control signal
            if (!flag_unable_to_switch){
                Serial1.println(F("Unable to switch to Autonomous Mode, remote not updating"));
                // set a flag so we don't constantly show this message
                flag_unable_to_switch = true;
            }
          }
        } else if (remote_signal_delay>REMOTE_MAX_DELAY_MS){
            // always at autonomous mode, but control signal is lost
            manual_control = true;
            Serial1.print(F("Autonomous Control Signal Loss, manual override engaged, dt= "));
            Serial1.println(remote_signal_delay);
          }

    }
  }
  //block --

  //block -- autonomous control
  static int remote_update_freq = 50;
  static unsigned long remote_update_ts = millis();
  if (!manual_control and !flag_signal_loss and (remote_signal_delay<REMOTE_MAX_DELAY_MS) and((millis() - remote_update_ts) > (unsigned long) (1000 / float(remote_update_freq)))){
    remote_update_ts = millis();
    setThrottleServoPulseWidth(remote_throttle);
    setFlapServoPulseWidth(fmap(float(remote_flap),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
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

  //block -- loss of signal
  static int loss_of_signal_freq = 10;
  static unsigned long loss_of_signal_ts = millis();
  if ( (millis() - loss_of_signal_ts) > (unsigned long) (1000 / float(loss_of_signal_freq)) ) {
    loss_of_signal_ts = millis();

    // detect loss of signal (works with FHSS on a channel with fail safe DISABLED)
    unsigned long us_since_last_rc_in = micros() - rc_rising_ts[0];
    if (us_since_last_rc_in > 500000) {
      if (!flag_signal_loss){ // enter failsafe, disable motor and set flap to neutral
        Serial1.println(F("Signal loss - Failsafe enabled"));
        flag_signal_loss = true;
        setThrottleServoPulseWidth(MIN_THROTTLE_SERVO_PULSEWIDTH);
        setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
      }
    } else if (flag_signal_loss){ // recover from failsafe 
      flag_signal_loss = false;
      // Autonomous control will automatically resume by autonomous block
      // Manual throttle control will resume automatically by throttle block
      // Manual flap control need to be re-enabled here
      Serial1.println(F("RC signal recovered"));
    }
  }
  // block ---


  //block -- manual control relay
  static int throttle_update_freq = 50;
  static unsigned long throttle_update_ts = millis();
  //static int ctrl_seq_no = 0;
  if ( manual_control && ((millis() - throttle_update_ts) > (unsigned long) (1000 / float(throttle_update_freq))) ) {
    throttle_update_ts = millis();
    /*
    Serial1.print(F("control update "));
    Serial1.print(ctrl_seq_no++);
    Serial1.print(F("T:"));
    Serial1.print(rc_in_val[2]);
    Serial1.print(F("F:"));
    Serial1.println(rc_in_val[4]);
    */
    setThrottleServoPulseWidth(rc_in_val[2]);
    setFlapServoPulseWidth(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
  }
  //block -- 

  // block -- machine readable output
  static unsigned long seq_no = 0;
  static int serial_loop_freq = 50;
  static unsigned long serial_loop_ts = millis();
  if ( (millis() - serial_loop_ts) > (unsigned long) (1000 / float(serial_loop_freq)) ) {
    serial_loop_ts = millis();
    seq_no++;

    // signify this is a line intended for machine parsing
    // 26 bytes, -> max 500Hz
    Serial1.print('#');
    Serial1.print(seq_no);
    Serial1.print(",");
    Serial1.print(voltage_running_avg,2); // voltage
    Serial1.print(",");
    Serial1.print(rc_in_val[4]); //flap, raw pwm
    Serial1.print(",");
    Serial1.print(rc_in_val[2]); //throttle, raw pwm
    Serial1.print(",");
    Serial1.print((manual_control)?0:1);
    Serial1.println();
  }
  // block --

  //block -- human readable debug info
  static int debug_output_freq = 10;
  static unsigned long debug_output_ts = millis();
  if ( (millis() - debug_output_ts) > (unsigned long) (1000 / float(debug_output_freq)) ) {
    debug_output_ts = millis();
    if (loophz_verbose){
        float temp_main_loop_hz = 9999.0;
        //for (int i=0;i<50;i++){ temp_main_loop_hz+= main_loop_hz_array[i];}
        for (int i=0;i<50;i++){ temp_main_loop_hz = (temp_main_loop_hz<main_loop_hz_array[i])?temp_main_loop_hz:main_loop_hz_array[i];}
        Serial1.print(F("Loop freq (Hz) : "));
        Serial1.println(temp_main_loop_hz/50.0,2);
    }

    if (remote_verbose){
        unsigned long temp_remote_sig_delay_ms = 0;
        for (int i=0;i<50;i++){ temp_remote_sig_delay_ms = (temp_remote_sig_delay_ms>remote_delay_ms_array[i])?temp_remote_sig_delay_ms:remote_delay_ms_array[i];}
        Serial1.print(F("tele_sig_dt(ms) = "));
        Serial1.println(temp_remote_sig_delay_ms);
    }
  }
  //block --
}
