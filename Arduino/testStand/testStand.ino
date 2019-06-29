//Test platform firmware
// Nick Zhang Summer 2019
#include <Encoder.h>
#include <MatrixMath.h>
typedef mtx_type matrix;
#define multiply(A,B,m,n,p,C) Matrix.Multiply((mtx_type*)A, (mtx_type*)B, m, n, p, (mtx_type*)C)
#define add(A,B,m,n,C) Matrix.Add((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define subtract(A,B,m,n,C) Matrix.Subtract((mtx_type*)A, (mtx_type*)B, m, n,  (mtx_type*)C)
#define transpose(A,m,n,C) Matrix.Transpose((mtx_type*)A, m, n, (mtx_type*)C)
#define invert(A,m) Matrix.Invert((mtx_type*)A,m)
Encoder enc(2,3);

// actually matrix here logically means type of the elements in a matrix, an array of elements compose a matrix
matrix x[2][1],F[2][2],P[2][2],Q[2][2],H[1][2],HT[2][1];
float S;
unsigned long last_update;

// (deg/s^2)^2 stddev^2
const float acc_variance=0.1;
// observation stddev, deg
const float R = 180.0/600;

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
  
}

void predict(){
  unsigned long ts = millis();
  float delta_t = (ts - last_update) / 1000.0;
  last_update = ts;
  // because F.invert is calculated in plate, we need to re-init F each time
  F[0][0] = 1;
  F[1][0] = 0;
  F[1][1] = 1;
  F[0][1] = delta_t;
  // x = Fx + Bu (no control so no Bu here)
  multiply(F,x,2,2,1,x);
  // P = FPF.T + Q
  multiply(F,P,2,2,2,P);
  invert(F,2);
  multiply(P,F,2,2,2,P);
  Q[0][0] = 0.25*delta_t*delta_t*delta_t*delta_t*acc_variance;
  Q[0][1] = 0.5*delta_t*delta_t*delta_t*acc_variance;
  Q[1][0] = Q[0][1];
  Q[1][1] = delta_t*delta_t*acc_variance;
  add(P,Q,2,2,P);
  return;
}

void update(float z){
  // buffer, one size fits all
  matrix buff[2][2];
  matrix I[2][2];
  I[0][0] = 1;
  I[1][1] = 1;  
  I[0][1] = 0;
  I[1][0] = 0;
  // y = z - Hx
  H[0][0] = 1;
  H[0][1] = 0;
  multiply(H,x,1,2,1,buff);
  matrix y[1][1];
  y[0][0] = z - buff[0][0];

  // S = HPH.T + R
  multiply(H,P,1,2,2,buff);
  multiply(buff,HT,1,2,1,buff);
  S = buff[0][0] + R;
  // K = P HT S-1
  matrix K[2][1];
  multiply(P,HT,2,2,1,buff); 
  matrix mat_S[1][1];
  mat_S[0][0] = S;
  multiply(buff,mat_S,2,1,1,K);
  // x = x + Ky
  multiply(K,y,2,1,1,buff);
  add(x,buff,2,1,x);
  // P = (I-KH)P
  multiply(K,H,2,1,2,buff);
  subtract(I,buff,2,2,buff);
  multiply(buff,P,2,2,2,P);
  // y = z - Hx post fit residual, not needed
  return;
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
