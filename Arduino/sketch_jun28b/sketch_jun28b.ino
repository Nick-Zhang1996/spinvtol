void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello");
  delay(100);
  digitalWrite(13,HIGH);
  delay(100);
  digitalWrite(13,LOW);
}
