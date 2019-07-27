#define LED_PIN 13
// action sequence for blinker
#define NONE 0
#define LED_OFF 1
#define LED_ON 2

volatile byte pending_action_t1a = NONE;
volatile bool flash_in_progress = false;
volatile float on_time_ms = 20;
void blinker() {
    
    switch (pending_action_t1a){

        case LED_ON:
            digitalWrite(PIN_LED,HIGH);
            //Serial.println("ISR led on");
            pending_action_t1a = LED_OFF;
            next_action_t1a(on_time_ms);
            break;

        case LED_OFF:
            digitalWrite(PIN_LED,LOW);
            pending_action_t1a  = NONE;
            flash_in_progress = false;
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
}

void loop() {
  // put your main code here, to run repeatedly:

}
