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
int led = 13;

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


void setup() {
    Serial1.begin(115200);
      analogWriteFrequency(PIN_FLAP_SERVO, 300);
      analogWriteFrequency(PIN_THROTTLE_SERVO, 300);
}

int seq_no = 0;
void loop(){
    Serial1.print("Hello World");
    Serial1.println(seq_no++);
    delay(100);
    setFlapServoPulseWidth(300);
}
