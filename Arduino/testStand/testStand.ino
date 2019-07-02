//Test platform firmware
// Nick Zhang Summer 2019
#include <Encoder.h>
#include <MatrixMath.h>
#include <SerialCommand.h>

typedef mtx_type matrix;
#define multiply(A,B,m,n,p,C) Matrix.Multiply((mtx_type*)A, (mtx_type*)B, m, n, p, (mtx_type*)C)
#define add(A,B,m,n,C) Matrix.Add((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define subtract(A,B,m,n,C) Matrix.Subtract((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define transpose(A,m,n,C) Matrix.Transpose((mtx_type*)A, m, n, (mtx_type*)C)
#define invert(A,m) Matrix.Invert((mtx_type*)A,m)
#define mtxprint(A,m,n,N) Matrix.Print((mtx_type*)A, m, n, N)
#define mtxcopy(A,m,n,B) Matrix.Copy((mtx_type*)A, m, n, (mtx_type*)B)
Encoder enc(2,3);
// internally pulled up, down activate, release after use
const int synchro_pinno = 4;

// actually matrix here logically means type of the elements in a matrix, an array of elements compose a matrix
matrix x[2][1],F[2][2],P[2][2],Q[2][2],H[1][2],HT[2][1];
float S;


// (deg/s^2)^2 stddev^2
const float acc_variance=10.0;
// observation stddev, deg
const float R = 360.0/2400;

SerialCommand sCmd;

unsigned long epoch;
// reset offset timeframe, for synchronizing data with other electronics
void synchronize(){
  digitalWrite(13,HIGH);
  Serial.println(F("Pending Synchronize signal"));
  if (digitalRead(synchro_pinno)==LOW){
    Serial.println(F("Error, input pin not in high state, try again when ready"));
    digitalWrite(13,LOW);
    return;
  }
  unsigned long timeout_ts = millis();
  while (millis()-timeout_ts <10000 && digitalRead(synchro_pinno)==HIGH);
  unsigned long temp = millis();
  if (temp-timeout_ts >=10000){
    Serial.println(F(" Timeout "));
    digitalWrite(13,LOW);
    return;
  }
  epoch = temp;
  Serial.print(F(" Reset timestamp - "));
  Serial.println(epoch);
  digitalWrite(13,LOW);
}


void kf_predict(  unsigned long ts){
  static unsigned long last_update = millis();
  
  matrix buff_col[2][1],buff_square[2][2];
  float delta_t = (ts - last_update) / 1000.0;
  last_update = ts;
  // because F.invert is calculated in plate, we need to re-init F each time
  F[0][0] = 1;
  F[1][0] = 0;
  F[1][1] = 1;
  F[0][1] = delta_t;
  // x = Fx + Bu (no control so no Bu here)
  
//  Serial.print("entering = ");
//  Serial.print("kf_dt : ");
//  Serial.print(delta_t);
//  Serial.print(" x : ");
//  Serial.print(x[0][0]);
//  Serial.print(" "); 
//  Serial.println(x[1][0]);

  multiply(F,x,2,2,1,buff_col);
  mtxcopy(buff_col,2,1,x);
  
  //mtxprint(F,2,2,"F");
  //mtxprint(x,2,1,"x");
//  Serial.print("prediction -> ");
//  Serial.print(x[0][0]);
//  Serial.print(" ");
//  Serial.println(x[1][0]);
  //mtxprint(x,2,1,"x");

  // P = FPF.T + Q
  multiply(F,P,2,2,2,buff_square);
  invert(F,2);
  multiply(buff_square,F,2,2,2,P);
  Q[0][0] = 0.25*delta_t*delta_t*delta_t*delta_t*acc_variance;
  Q[0][1] = 0.5*delta_t*delta_t*delta_t*acc_variance;
  Q[1][0] = Q[0][1];
  Q[1][1] = delta_t*delta_t*acc_variance;
  add(P,Q,2,2,P);
  return;
}

//z:observation
void kf_update(float z){
  // buffer, one size fits all
  matrix buff_single[1][1],buff_row[1][2],buff_col[2][1],buff_square[2][2],buff_square2[2][2];
  matrix I[2][2];
  I[0][0] = 1;
  I[1][1] = 1;  
  I[0][1] = 0;
  I[1][0] = 0;
  // y = z - Hx
  H[0][0] = 1;
  H[0][1] = 0;
  multiply(H,x,1,2,1,buff_single);
  matrix y[1][1];
  y[0][0] = z - buff_single[0][0];
  // should be observed angle - estimated angle
  
  // S = HPH.T + R
  //mtxprint(P,2,2,"P");
  
  multiply(H,P,1,2,2,buff_row);
  multiply(buff_row,HT,1,2,1,buff_single);
  
  //mtxprint(buff_single,1,1,"buffer");
  // covariance of innovation
  S = buff_single[0][0] + R;

  // K = P HT S-1 ----------------------
  matrix K[2][1];
  multiply(P,HT,2,2,1,buff_col); 
  matrix mat_Sinv[1][1];
  mat_Sinv[0][0] = 1.0/S;
  multiply(buff_col,mat_Sinv,2,1,1,K);
  // x = x + Ky
  multiply(K,y,2,1,1,buff_col);
  add(x,buff_col,2,1,x);
//  Serial.print("update -> ");
//  Serial.print(" K ");
//  Serial.print(K[0][0]);
//  Serial.print(" ");
//  Serial.print(K[1][0]);
//  Serial.print(" x: ");
//  Serial.print(x[0][0]);
//  Serial.print(" ");
//  Serial.println(x[1][0]);
  
  // P = (I-KH)P
  multiply(K,H,2,1,2,buff_square);
  subtract(I,buff_square,2,2,buff_square);
  multiply(buff_square,P,2,2,2,buff_square2);
  mtxcopy(buff_square,2,2,P);
  // y = z - Hx post fit residual, not needed
  return;
}

long oldPosition;

//simulation variables:
float azimuth, omega, alpha;

bool human_readable_output = false;
void human(){
  human_readable_output = true;
}

void machine(){
  human_readable_output = false;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  // x = [azimuth angle,angular velocity].T
  // x = 0
  
  // P covariance matrix of state estimation
  // P = 0

  // F state transition model matrix

  // H observation model
  H[0][0] = 1;
  H[0][1] = 0;
  // H transpose
  transpose(H,1,2,HT);
  pinMode(synchro_pinno,INPUT_PULLUP);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  sCmd.addCommand("human",human);
  sCmd.addCommand("machine",machine);
  sCmd.addCommand("sync",synchronize);
}



//unsigned long sim_last_update;
unsigned long loop_ts;
void loop() {
  sCmd.readSerial();
  // overflow: 24hr over 10rev/s
  // Warning, numerically unstable

  long newPosition = (enc.read())/2400.0*360;
  unsigned long timestamp = millis();
  kf_predict(timestamp);

  kf_update(newPosition);

  
  static int update_freq = 10;
  static unsigned long update_ts = millis();

  if (human_readable_output){
    if ( (millis() - update_ts) > (unsigned long) (1000 / float(update_freq)) ) {
      Serial.print(timestamp-epoch);
      Serial.print(": Raw position : ");
      long offset = (int)newPosition / 360 *360;
      Serial.print(newPosition-offset);
      Serial.print("(deg) Estimated position : ");
      offset = (int)x[0][0] / 360 *360;
      Serial.print(x[0][0]-offset);
      Serial.print("(deg) Estimated velocity : ");
      Serial.print(x[1][0]/360.0);
      Serial.println("rev/s");
    }
  } else {
    // ts, azimuth angle(deg), angular velocity(rad/s)
    long offset = (int)newPosition / 360 *360;
    while (millis()-loop_ts < 20); // limit transmission rate to 50Hz
    loop_ts = millis();
    
    //indicates start of a line for machine reading
    Serial.print("#");
    Serial.print(timestamp-epoch);
    Serial.print(",");
    Serial.print((newPosition-offset),2);
    Serial.print(",");
    // rad/s
    Serial.println(x[1][0]*2*3.14159265359,2);

  }
  
  
// simulation code
//  unsigned long sim_ts = millis();
//  float sim_dt = (sim_ts - sim_last_update)/1000.0;
//  // really the alpha of last frame
//  alpha = random(-100,100)/10.0;
//  azimuth += omega*sim_dt + 0.5*alpha*sim_dt*sim_dt;
//  omega += alpha*sim_dt;
//  sim_last_update = sim_ts;

}
