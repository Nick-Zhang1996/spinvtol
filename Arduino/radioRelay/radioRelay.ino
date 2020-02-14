// read RC PWM input, send to serial 
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>
uint8_t rc_in_pinno[] = {4, 5}; // CH0(throttle),1(flap)
volatile int rc_in_val[2] = {0};
volatile unsigned long rc_rising_ts[2] = {0};

//note: measured dutycycle for futaba FHSS 13564us, 73Hz
void rc_in_callback() {
  unsigned long timestamp = micros();
  int channel = -1;
  switch (arduinoInterruptedPin) {
    case 4 : channel = 0; break;
    case 5 : channel = 1; break;
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

int onboard_led_pin = 13;
bool flag_signal_loss = true;
void setup() {
  Serial.begin(115200);
  pinMode(onboard_led_pin,OUTPUT);
  digitalWrite(onboard_led_pin,LOW);
  while (!Serial);

  for (int i = 0; i < sizeof(rc_in_pinno) / sizeof(uint8_t);  i++) {
    pinMode(rc_in_pinno[i], INPUT);
    enableInterrupt( rc_in_pinno[i], rc_in_callback, CHANGE);
  }
}
unsigned long last_serial_ts = 0;
void loop(){
  //block -- loss of signal
  static int loss_of_signal_freq = 10;
  static unsigned long loss_of_signal_ts = millis();
  if ( (millis() - loss_of_signal_ts) > (unsigned long) (1000 / float(loss_of_signal_freq)) ) {
    loss_of_signal_ts = millis();

    // detect loss of signal (works with FHSS on a channel without fail safe enabled
    unsigned long us_since_last_rc_in = micros() - rc_rising_ts[0];
    // 0.5s
    if (us_since_last_rc_in > 500000) {
      digitalWrite(onboard_led_pin,LOW);
      flag_signal_loss = true;
    } else if(flag_signal_loss){
      flag_signal_loss = false;
      digitalWrite(onboard_led_pin,HIGH);
    }
  }
  // block ---
  // this gives 300Hz output rate
  while ((micros()-last_serial_ts)<1860) {;}

  if (true) {
      last_serial_ts = micros();
      Serial.print("#");
      //Serial.print(millis());
      //Serial.print(",");
      Serial.print(rc_in_val[0]);
      Serial.print(",");
      Serial.println(rc_in_val[1]);
  }
}
  
