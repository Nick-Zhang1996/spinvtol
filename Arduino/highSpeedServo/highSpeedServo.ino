// high speed servo controller with timer. This sketch is able to control ppm on a per duty cycle basis

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
}

void loop() {
  // put your main code here, to run repeatedly:

}
