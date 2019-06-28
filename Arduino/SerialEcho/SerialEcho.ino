/*
  Serial Call and Response
 Language: Wiring/Arduino

 This program sends an ASCII A (byte of value 65) on startup
 and repeats that until it gets some data in.
 Then it waits for a byte in the serial port, and
 sends three sensor values whenever it gets a byte in.

 Thanks to Greg Shakar and Scott Fitzgerald for the improvements

   The circuit:
 * potentiometers attached to analog inputs 0 and 1
 * pushbutton attached to digital I/O 2

 Created 26 Sept. 2005
 by Tom Igoe
 modified 24 April 2012
 by Tom Igoe and Scott Fitzgerald

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/SerialCallResponse

 */

int inByte = 0;         // incoming serial byte

void setup()
{
  // start serial port at bps:
  Serial.begin(38400);
  pinMode(13,OUTPUT);
  pinMode(0,INPUT);
  // disable all pullup function
  MCUCR |= (1u << PUD);
}

void loop()
{
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    
    //Serial.print("character recieved: ");
    Serial.write(inByte);
    
    if (inByte=='l'){
      digitalWrite(13,LOW);
    } else if (inByte == 'h'){
      digitalWrite(13,HIGH);
    }
    
  }
}
