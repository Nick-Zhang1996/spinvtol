void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);

}

unsigned long timestamp = 0;
int value = 1000;
void loop() {
  // put your main code here, to run repeatedly:
  if (value > 2000) {
    value = 1000;
  }

  digitalWrite(7, HIGH);
  delayMicroseconds(value);
  digitalWrite(7, LOW);
  delayMicroseconds(30000 - value);
  value++;


}
