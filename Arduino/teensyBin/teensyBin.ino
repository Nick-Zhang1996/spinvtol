// This firmware achieves the following things
// 1. Relay control from telemetry to flap and throttle servo
// 2. Send over Serial1 current voltage measurement
// 3. Allow switching from manual control(RC receiver reading) to telemetry control
// 4. Ping
// NOTE this uses the new binary protocol

// Manual control:
// CH2(rc receiver) ->

//#define DEBUG

#define REMOTE_MAX_DELAY_MS 80
#define BUFFER_SIZE 7

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

// Binary protocol
// normal update
#define MSG_UPDATE 1
// ping response, remaining data in packet undefined
#define MSG_PING 2

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

// update serial remote control command
// this is the processor for host computer generated control command
volatile bool manual_control = true;
volatile unsigned long remote_ts = 0; // use a timestamp to monitor if up to date control msg is available
volatile uint16_t remote_throttle = MIN_THROTTLE_SERVO_PULSEWIDTH;
volatile uint16_t remote_flap = NEUTRAL_FLAP_SERVO_PULSEWIDTH;
uint8_t buffer[BUFFER_SIZE] = {0};
uint8_t buffer_index = 0;
void parseSerial(){
  // incoming packet length is 7
  // Byte 0,1: 9, used for detecting misaligned data
  // Byte 2: msg type 1->control update 2-> ping request, ignore following packet
  // 3-4: flapPWM
  // 5-6: throttlePWM
  while (Serial.available()>0){
      buffer[buffer_index%BUFFER_SIZE] = Serial.read();
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
          remote_flap = (remote_flap>VR_MAX)?VR_MAX:remote_flap;
          remote_flap = (remote_flap<VR_MIN)?VR_MIN:remote_flap;

          remote_throttle = ((uint16_t)buffer[6]<<8)+buffer[5];
          remote_throttle = (remote_throttle>MAX_THROTTLE_SERVO_PULSEWIDTH)?MAX_THROTTLE_SERVO_PULSEWIDTH:remote_throttle;
          remote_throttle = (remote_throttle<MIN_THROTTLE_SERVO_PULSEWIDTH)?MIN_THROTTLE_SERVO_PULSEWIDTH:remote_throttle;
        }

        if (msgType==2){
          struct monomsg packet;
          packet.msg_type = MSG_PING;
          Serial.write((const uint8_t *)&packet,sizeof(struct monomsg));
          digitalWrite(PIN_LED,HIGH);

        }

        remote_ts = millis();
      }
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
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED4, OUTPUT);
  pinMode(PIN_FLAP_SERVO,OUTPUT);
  pinMode(PIN_THROTTLE_SERVO,OUTPUT);
  digitalWrite(PIN_LED,LOW);
  digitalWrite(PIN_LED1,HIGH);
  digitalWrite(PIN_LED2,HIGH);
  digitalWrite(PIN_LED3,HIGH);
  digitalWrite(PIN_LED4,HIGH);
  digitalWrite(PIN_FLAP_SERVO,LOW);
  digitalWrite(PIN_THROTTLE_SERVO,LOW);

  analogWriteFrequency(PIN_FLAP_SERVO, 300);
  analogWriteFrequency(PIN_THROTTLE_SERVO, 300);
}

unsigned long main_loop_ts;
float main_loop_hz = 0.0;
float main_loop_hz_array[50] = {0};
int p_main_loop_hz_array = 0;
unsigned long remote_delay_ms_array[100] = {0};
int p_remote_delay_ms_array = 0;

void loop() {
  main_loop_hz = 1000000.0/float(micros()-main_loop_ts);
  main_loop_ts = micros();
  main_loop_hz_array[p_main_loop_hz_array%50] = main_loop_hz;
  p_main_loop_hz_array++;
  //block -- read command
  parseSerial();

  //block -- manual/telemetry control switch
  static bool flag_unable_to_switch = false;
  unsigned long remote_signal_delay = 0;
  remote_signal_delay = millis()-remote_ts;
  remote_delay_ms_array[(p_remote_delay_ms_array++)%100] = remote_signal_delay;
  if (!flag_signal_loss){
    if (rc_in_val[5]>1300){ // switch to manual mode
        if (!manual_control){ // if current mode is automatic, switch to manual
          manual_control = true;
        } else {
            flag_unable_to_switch = false; // this flag should stay false in manual mode
        }
    } else if (rc_in_val[5]<1300){ // switch to autonomous mode
        if (manual_control){
          // ensure a recent remote control command is present
          // this avoids accidently switching control to remote when remote is not ready
          if (remote_signal_delay<REMOTE_MAX_DELAY_MS){
            manual_control = false;
          } else {
            // if user request autonomous mode but there's no autonomous control signal
            if (!flag_unable_to_switch){
                // set a flag so we don't constantly show this message
                flag_unable_to_switch = true;
            }
          }
        } else if (remote_signal_delay>REMOTE_MAX_DELAY_MS){
            // always at autonomous mode, but control signal is lost
            manual_control = true;
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
        flag_signal_loss = true;
        setThrottleServoPulseWidth(MIN_THROTTLE_SERVO_PULSEWIDTH);
        setFlapServoPulseWidth(NEUTRAL_FLAP_SERVO_PULSEWIDTH);
        digitalWrite(PIN_LED,HIGH);
      }
    } else if (flag_signal_loss){ // recover from failsafe 
      flag_signal_loss = false;
      digitalWrite(PIN_LED,LOW);
      // Autonomous control will automatically resume by autonomous block
      // Manual throttle control will resume automatically by throttle block
    }
  }
  // block ---
  // process commandd


  //block -- manual control relay
  static int throttle_update_freq = 50;
  static unsigned long throttle_update_ts = millis();
  //static int ctrl_seq_no = 0;
  if ( manual_control && ((millis() - throttle_update_ts) > (unsigned long) (1000 / float(throttle_update_freq))) ) {
    throttle_update_ts = millis();
    setThrottleServoPulseWidth(rc_in_val[2]);
    setFlapServoPulseWidth(fmap(float(rc_in_val[4]),VR_MIN,VR_MAX,MIN_FLAP_SERVO_PULSEWIDTH,MAX_FLAP_SERVO_PULSEWIDTH));
  }
  //block -- 

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
    Serial.write((const uint8_t *)&packet,sizeof(struct monomsg));

  }
  // block --

}
