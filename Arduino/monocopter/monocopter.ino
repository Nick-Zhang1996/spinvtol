// monocopter controller
// Nick Zhang 2019

#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel_in = Adafruit_ADXL345_Unified(1);
Adafruit_ADXL345_Unified accel_out = Adafruit_ADXL345_Unified(2);

void displaySensorDetails(Adafruit_ADXL345_Unified &accel)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(Adafruit_ADXL345_Unified &accel)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(Adafruit_ADXL345_Unified &accel)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}



// Functionalities and pin layout
// RC in : 3 channels, on pin 4,5,6 (port D 0-7)
// RC out: 2 channels, flap on pin 8, ESC on pin 9 (port B 8-13)

// Timer usage:
// Timer 0 : delay, millis
// Timer 1 : Servo 

uint8_t rc_in_pinno[] = {4,5,6}; // CH0,1,2
volatile int rc_in_val[3] = {0};
volatile unsigned long rc_rising_ts[3] = {0};
bool flag_signal_loss = false;

Servo flap;
Servo esc;

//note: measured dutycycle for futaba FHSS 13564us, 73Hz
void rc_in_callback(){
  unsigned long timestamp = micros();
  int channel = -1;
  switch (arduinoInterruptedPin) {
    case 4 : channel = 0; break;
    case 5 : channel = 1; break;
    case 6 : channel = 2; break;
    default : break;
  }
  if (channel == -1){return;}
  if (digitalRead(arduinoInterruptedPin)==HIGH) {
    rc_rising_ts[channel] = timestamp;
  } else { 
    rc_in_val[channel] = timestamp - rc_rising_ts[channel];

  }
}

void setup() {
  Serial.begin(57600);
  while (!Serial);
  
  for (int i=0;i<sizeof(rc_in_pinno)/sizeof(uint8_t);  i++){
    pinMode(rc_in_pinno[i],INPUT);
    enableInterrupt( rc_in_pinno[i], rc_in_callback, CHANGE);
  }
  Serial.println(REFRESH_INTERVAL);
  flap.attach(8);
  esc.attach(9);

// ---------------- ADXL345 init ----------------
  Serial.println("Accelerometer Init -- inner (0x53)"); Serial.println("");
  /* Initialise the sensor */
  if(!accel_in.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Inner ADXL345 not detected");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel_in.setRange(ADXL345_RANGE_16_G);
  // accel_in.setRange(ADXL345_RANGE_8_G);
  // accel_in.setRange(ADXL345_RANGE_4_G);
  // accel_in.setRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails(accel_in);
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate(accel_in);
  displayRange(accel_in);
  Serial.println("");
// ------ outer ADXL345
  Serial.println("Accelerometer Init -- outer (0x1D)"); Serial.println("");
  /* Initialise the sensor */
  if(!accel_out.begin(0x1D))
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Outer ADXL345 not detected");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel_out.setRange(ADXL345_RANGE_16_G);
  // accel_out.setRange(ADXL345_RANGE_8_G);
  // accel_out.setRange(ADXL345_RANGE_4_G);
  // accel_out.setRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails(accel_out);
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate(accel_out);
  displayRange(accel_out);
  Serial.println("");
// ----------- end ADXL345 init -----------



}

void loop() {

//block -- serial update
  static int serial_update_freq = 10;
  static unsigned long serial_update_ts = millis();
  if ( (millis()-serial_update_ts) > (unsigned long) (1000/float(serial_update_freq)) ){
    serial_update_ts = millis();
    Serial.print("RC in : ch0 : ");
    Serial.print(rc_in_val[0]);
    Serial.print(" | ch1 : ");
    Serial.print(rc_in_val[1]);
    Serial.print(" | ch2 : ");
    Serial.println(rc_in_val[2]);
  }
// block ---- 

//block -- Accelerometer update
  static int accel_update_freq = 10;
  static unsigned long accel_update_ts = millis();
  if ( (millis()-accel_update_ts) > (unsigned long) (1000/float(accel_update_freq)) ){
    accel_update_ts = millis();

    /* Get a new sensor event */ 
    sensors_event_t event; 
    accel_in.getEvent(&event);
 
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("Innter - X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.print("m/s^2      ");

   // repeat for the outer one
   /* Get a new sensor event */ 
    //sensors_event_t event; 
    accel_out.getEvent(&event);
 
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("outer X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  }

//block -- rc out
  static int rc_out_freq = 100;
  static unsigned long rc_out_ts = millis();
  if ( (millis()-rc_out_ts) > (unsigned long) (1000/float(rc_out_freq)) ){
    rc_out_ts = millis();
    flap.writeMicroseconds(map(rc_in_val[0],1000,2000,260,1260));
    esc.writeMicroseconds(rc_in_val[1]);
  }
// block ---- 


//block -- loss of signal
  static int loss_of_signal_freq = 10;
  static unsigned long loss_of_signal_ts = millis();
  if ( (millis()-loss_of_signal_ts) > (unsigned long) (1000/float(loss_of_signal_freq)) ){
    loss_of_signal_ts = millis();
    
    // detect loss of signal (works with FHSS on a channel without fail safe enabled
    unsigned long us_since_last_rc_in = micros()-rc_rising_ts[0];
    if (us_since_last_rc_in > 500000){
      Serial.print("Signal loss");
      flag_signal_loss = true;
    } else{
      flag_signal_loss = false;
    }
  }
// block ---  

  

}
