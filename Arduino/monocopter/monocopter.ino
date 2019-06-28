// monocopter controller
// Nick Zhang 2019

#define PIN_LED1 10
#define PIN_LED2 11
#define PIN_LED3 12

#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>

#include <Servo.h>
#include <Wire.h>
#include <SerialCommand.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define I2Cclock 400000
#define I2Cport Wire
#include <MPU9250.h>
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 // AD0 on MPU9250 pulled down to GND

SerialCommand sCmd;

// MPU9250 init sets Wire I2c freq(adxl345 does not), let it run first
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

Adafruit_ADXL345_Unified accel_in = Adafruit_ADXL345_Unified(1);
Adafruit_ADXL345_Unified accel_out = Adafruit_ADXL345_Unified(2);

void displaySensorDetails(Adafruit_ADXL345_Unified &accel)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println("");
  delay(500);
}

void displayDataRate(Adafruit_ADXL345_Unified &accel)
{
  Serial.print  (F("Data Rate:    "));

  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  (F("3200 "));
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  (F("1600 "));
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  (F("800 "));
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  (F("400 "));
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  (F("200 "));
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  (F("100 "));
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  (F("50 "));
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  (F("25 "));
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  (F("12.5 "));
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  (F("6.25 "));
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  (F("3.13 "));
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  (F("1.56 "));
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  (F("0.78 "));
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  (F("0.39 "));
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  (F("0.20 "));
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  (F("0.10 "));
      break;
    default:
      Serial.print  (F("???? "));
      break;
  }
  Serial.println(F(" Hz"));
}

void displayRange(Adafruit_ADXL345_Unified &accel)
{
  Serial.print  (F("Range:         +/- "));

  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  (F("16 "));
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  (F("8 "));
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  (F("4 "));
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  (F("2 "));
      break;
    default:
      Serial.print  (F("?? "));
      break;
  }
  Serial.println(F(" g"));
}



// Functionalities and pin layout
// RC in : 3 channels, on pin 4,5,6 (port D 0-7)
// RC out: 2 channels, flap on pin 8, ESC on pin 9 (port B 8-13)

// Timer usage:
// Timer 0 : delay, millis
// Timer 1 : Servo

uint8_t rc_in_pinno[] = {4, 5, 6}; // CH0,1,2
volatile int rc_in_val[3] = {0};
volatile unsigned long rc_rising_ts[3] = {0};
bool flag_signal_loss = false;

Servo flap;
Servo esc;

//note: measured dutycycle for futaba FHSS 13564us, 73Hz
void rc_in_callback() {
  unsigned long timestamp = micros();
  int channel = -1;
  switch (arduinoInterruptedPin) {
    case 4 : channel = 0; break;
    case 5 : channel = 1; break;
    case 6 : channel = 2; break;
    default : break;
  }
  if (channel == -1) {
    return;
  }
  if (digitalRead(arduinoInterruptedPin) == HIGH) {
    rc_rising_ts[channel] = timestamp;
  } else {
    rc_in_val[channel] = timestamp - rc_rising_ts[channel];

  }
}


int xbee_rssi_pin = 3;
volatile unsigned long xbee_rssi_last_rising;
volatile unsigned long xbee_rssi_val;
volatile bool xbee_rssi_newdata = false;
void xbee_rssi_callback() {
  unsigned long timestamp = micros();

  if (digitalRead(xbee_rssi_pin) == HIGH) {
    xbee_rssi_last_rising = timestamp;
  } else {
   xbee_rssi_val = timestamp - xbee_rssi_last_rising;
   xbee_rssi_newdata = true;

  }
}
// --------- Serial command handlers -------
int onoff(char* arg) {
  if (arg != NULL) {
    if (!strcmp(arg, "on")) {
      return 1;
    } else if (!strcmp(arg, "off")) {
      return 0;
    }
  }
  return -1;
}
void led() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(13, rval);
  }
}

void led1() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(PIN_LED1, !rval);
  }
}
void led2() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(PIN_LED2, !rval);
  }
}
void led3() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    digitalWrite(PIN_LED3, !rval);
  }
}

bool mag_verbose = true;
void display_mag() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    mag_verbose = (bool)rval;
  }
}

bool acc_verbose = true;
void display_acc() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    acc_verbose = (bool)rval;
  }
}

bool rc_verbose = true;
void display_rc() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    rc_verbose = (bool)rval;
  }
}
bool sig_verbose = true;
void display_sig() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    sig_verbose = (bool)rval;
  }
}

bool rssi_verbose = true;
void display_rssi() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    rssi_verbose = (bool)rval;
  }
}

void display_all() {
  int rval = onoff(sCmd.next());
  if (rval != -1) {
    rssi_verbose = acc_verbose = mag_verbose = rc_verbose = sig_verbose =  (bool)rval;
  }
}

void setup() {
  Serial.begin(38400);
  while (!Serial);

  for (int i = 0; i < sizeof(rc_in_pinno) / sizeof(uint8_t);  i++) {
    pinMode(rc_in_pinno[i], INPUT);
    enableInterrupt( rc_in_pinno[i], rc_in_callback, CHANGE);
    enableInterrupt( xbee_rssi_pin, xbee_rssi_callback,CHANGE);
  }
  Serial.println(REFRESH_INTERVAL);
  flap.attach(8);
  esc.attach(9);

  // ---------------- MPU9240 init ---------------
  Wire.begin();
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1); Serial.println(F("% of factory value"));
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1); Serial.println(F("% of factory value"));

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println(F("MPU9250 initialized for active data mode...."));

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print(F("AK8963 "));
    Serial.print(F("I AM 0x"));
    Serial.print(d, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println(F("AK8963 initialized for active data mode...."));

    //  Serial.println("Calibration values: ");
    Serial.print(F("X-Axis factory sensitivity adjustment value "));
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print(F("Y-Axis factory sensitivity adjustment value "));
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print(F("Z-Axis factory sensitivity adjustment value "));
    Serial.println(myIMU.factoryMagCalibration[2], 2);


    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println(F("AK8963 mag biases (mG)"));
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println(F("AK8963 mag scale (mG)"));
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);

    Serial.println(F("Magnetometer:"));
    Serial.print(F("X-Axis sensitivity adjustment value "));
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print(F("Y-Axis sensitivity adjustment value "));
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print(F("Z-Axis sensitivity adjustment value "));
    Serial.println(myIMU.factoryMagCalibration[2], 2);
  } else
  {
    Serial.print(F("Could not connect to MPU9250: 0x"));
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  // ---------------- ADXL345 init ----------------
  Serial.println(F("Accelerometer Init -- inner (0x53)")); Serial.println("");
  /* Initialise the sensor */
  if (!accel_in.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Inner ADXL345 not detected"));
    while (1);
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
  Serial.println(F("Accelerometer Init -- outer (0x1D)")); Serial.println("");
  /* Initialise the sensor */
  if (!accel_out.begin(0x1D))
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Outer ADXL345 not detected"));
    while (1);
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

  // --------  Register serial commands ------
  pinMode(13, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  digitalWrite(PIN_LED1,HIGH);
  digitalWrite(PIN_LED2,HIGH);
  digitalWrite(PIN_LED3,HIGH);
 
  sCmd.addCommand("led", led); // test function, takes 1 argument on or off
  sCmd.addCommand("led1", led1); // test function, takes 1 argument on or off
  sCmd.addCommand("led2", led2); // test function, takes 1 argument on or off
  sCmd.addCommand("led3", led3); // test function, takes 1 argument on or off
  sCmd.addCommand("mag", display_mag);
  sCmd.addCommand("acc", display_acc);
  sCmd.addCommand("rc", display_rc);
  sCmd.addCommand("sig", display_sig);
  sCmd.addCommand("rssi", display_rssi);
  sCmd.addCommand("all", display_all);
}

void loop() {
  sCmd.readSerial();
  //block -- serial update
  static int serial_update_freq = 10;
  static unsigned long serial_update_ts = millis();
  if ( (millis() - serial_update_ts) > (unsigned long) (1000 / float(serial_update_freq)) ) {
    serial_update_ts = millis();
    if (rc_verbose) {
      Serial.print(F("RC in : ch0 : "));
      Serial.print(rc_in_val[0]);
      Serial.print(F(" | ch1 : "));
      Serial.print(rc_in_val[1]);
      Serial.print(F(" | ch2 : "));
      Serial.println(rc_in_val[2]);
    }
  }
  // block ----

  //block -- magnetometer update
  static int mag_update_freq = 10;
  static unsigned long mag_update_ts = millis();
  if ( (millis() - mag_update_ts) > (unsigned long) (1000 / float(mag_update_freq)) ) {
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {

      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
                 * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
      myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
                 * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
      myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
                 * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];

      // Print mag values in degree/sec
      if (mag_verbose) {
        Serial.print(F("X-mag field: ")); Serial.print(myIMU.mx);
        Serial.print(F(" mG "));
        Serial.print(F("Y-mag field: ")); Serial.print(myIMU.my);
        Serial.print(F(" mG "));
        Serial.print(F("Z-mag field: ")); Serial.print(myIMU.mz);
        Serial.println(F(" mG"));
      }
    }
  }
  //block ----------
  
 //block -- Xbee rssi update
  static int rssi_update_freq = 10;
  static unsigned long rssi_update_ts = millis();
  if ( (millis() - rssi_update_ts) > (unsigned long) (1000 / float(rssi_update_freq)) ) {
    if (rssi_verbose){
      if (xbee_rssi_newdata){
        Serial.print(F("RSSI : "));
        Serial.println(xbee_rssi_val);
        xbee_rssi_newdata = false;
      } else{
        Serial.print(F("No RSSI update"));
      }
      }
    }

    
  }

  
  //block -- Accelerometer update
  static int accel_update_freq = 10;
  static unsigned long accel_update_ts = millis();
  if ( (millis() - accel_update_ts) > (unsigned long) (1000 / float(accel_update_freq)) ) {
    accel_update_ts = millis();

    /* Get a new sensor event */
    sensors_event_t event;
    accel_in.getEvent(&event);
    if (acc_verbose) {
      /* Display the results (acceleration is measured in m/s^2) */
      Serial.print(F("Innter - X: ")); Serial.print(event.acceleration.x); Serial.print(F("  "));
      Serial.print(F("Y: ")); Serial.print(event.acceleration.y); Serial.print(F("  "));
      Serial.print(F("Z: ")); Serial.print(event.acceleration.z); Serial.print(F("  ")); Serial.print(F("m/s^2      "));
    }

    // repeat for the outer one
    /* Get a new sensor event */
    //sensors_event_t event;
    accel_out.getEvent(&event);
    if (acc_verbose) {
      /* Display the results (acceleration is measured in m/s^2) */
      Serial.print(F("outer X: ")); Serial.print(event.acceleration.x); Serial.print(F("  "));
      Serial.print(F("Y: ")); Serial.print(event.acceleration.y); Serial.print(F("  "));
      Serial.print(F("Z: ")); Serial.print(event.acceleration.z); Serial.print(F("  ")); Serial.println(F("m/s^2 "));
    }
  }

  //block -- rc out
  static int rc_out_freq = 100;
  static unsigned long rc_out_ts = millis();
  if ( (millis() - rc_out_ts) > (unsigned long) (1000 / float(rc_out_freq)) ) {
    rc_out_ts = millis();
    flap.writeMicroseconds(map(rc_in_val[0], 1000, 2000, 260, 1260));
    esc.writeMicroseconds(rc_in_val[1]);
  }
  // block ----


  //block -- loss of signal
  static int loss_of_signal_freq = 10;
  static unsigned long loss_of_signal_ts = millis();
  if ( (millis() - loss_of_signal_ts) > (unsigned long) (1000 / float(loss_of_signal_freq)) ) {
    loss_of_signal_ts = millis();

    // detect loss of signal (works with FHSS on a channel without fail safe enabled
    unsigned long us_since_last_rc_in = micros() - rc_rising_ts[0];
    if (us_since_last_rc_in > 500000) {
      if (sig_verbose){
        Serial.println(F("Signal loss"));
      }
      flag_signal_loss = true;
    } else {
      flag_signal_loss = false;
    }
  }
  // block ---

}
