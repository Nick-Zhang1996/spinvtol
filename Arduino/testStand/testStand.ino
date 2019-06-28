//Test platform firmware
#include <Encoder.h>

Encoder enc(2,3);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
}

long oldPosition;
void loop() {
  // put your main code here, to run repeatedly:
  long newPosition = enc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
}
