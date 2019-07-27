#define LED_PIN 13
// action sequence for blinker
#define NONE 0
#define LED_OFF 1
#define LED_ON 2

IntervalTimer myTimer;

volatile byte pending_action_blinker = NONE;
void blinker() {
    
  switch (pending_action_blinker){

    case LED_ON:
      digitalWrite(LED_PIN,HIGH);
      //Serial.println("ISR led on");
      pending_action_blinker = LED_OFF;
      myTimer.begin(blinker,1e6);
      break;

    case LED_OFF:
      digitalWrite(LED_PIN,LOW);
      pending_action_blinker  = LED_ON;
      myTimer.begin(blinker,3e6);
      //Serial.println("ISR led off");
      break;

    case NONE:
      //Serial.println("ISR NONE");
      break;
  }
} 

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN,OUTPUT);
  pending_action_blinker = LED_ON;
  myTimer.begin(blinker,1e6);
}

void loop() {
  // put your main code here, to run repeatedly:
}
