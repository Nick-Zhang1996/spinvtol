#include <SerialCommand.h>
SerialCommand sCmd;
#define NONE 0
#define LED_OFF 1
#define LED_ON 2


void precise_delay_ms(float ms){
    //Serial.println("test routine started");
    cli();

    //set timer1 interrupt 
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCCR1C = 0; // PWM related stuff
    // per 328p datasheet, writing 1 to TOV1 clears the flag
    TIFR1 &= 1<<TOV1;



    // enable timer compare interrupt and overflow interrupt
    //TIMSK1 = (1 << OCIE1A) | ( 1 << TOIE1); // for reference, ovf interrupt

    TIMSK1 = 0;
    TIMSK1 |= (1 << TOIE1);

    uint16_t preload = 65536u - (ms/0.064);
    TCNT1H  = highByte(preload);
    TCNT1L  = lowByte(preload);
    Serial.println(uint16_t(TCNT1H)<<8 |uint16_t(TCNT1L));
 

    // duty cycle: (16*10^6) / (1024*65536) Hz = 0.238Hz = 4.194s
    // per count : 64us
    TCCR1B |= (1 << CS12) | (1 << CS10) ; 

    sei();
}

void stop_timer(){
    //Serial.println("test routine stopped");
    cli();

    //set timer2 interrupt 
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCCR1C = 0; // PWM related stuff

    sei();

}

volatile bool led_is_on = false;
volatile byte pending_action = NONE;
ISR(TIMER1_OVF_vect) {
  switch (pending_action){
    case NONE:
      break;

    case LED_ON:
      digitalWrite(13,HIGH);
      pending_action = LED_OFF;
      precise_delay_ms(1000);
      break;

    case LED_OFF:
      digitalWrite(13,LOW);
      pending_action  = NONE;
      stop_timer();
      break;
  }
} 

void led_sequence(){
  pending_action = LED_ON;
  precise_delay_ms(2000);
        Serial.println(uint16_t(TCNT1H)<<8 |uint16_t(TCNT1L));

}

void setup() {
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  sCmd.addCommand("led",led_sequence);

  // put your setup code here, to run once:
  //precise_delay_ms();

}

void loop() {
  // put your main code here, to run repeatedly:
      Serial.println(uint16_t(TCNT1H)<<8 |uint16_t(TCNT1L));
    delay(10);
    sCmd.readSerial();


}
