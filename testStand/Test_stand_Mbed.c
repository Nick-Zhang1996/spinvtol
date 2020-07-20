/*
  Rotary Encoder Test
  Demonstrates operation of Rotary Encoder
  Displays results on Serial Monitor
*/

#include "mbed.h"

#define tick 32.727272727272727
#define pi 3.14159265358979323846264338

InterruptIn encoder (p22);
PwmOut motor (p21);
Serial pc (USBTX, USBRX);
DigitalOut led(LED1);
DigitalOut blinker(LED2);
Timer t;
Timer t2;
Ticker ticker;

volatile double rev = 0, old_rev = 0, raw_rev = 0, print_rev = 0;
volatile double elapsed_time = 99999999999, cur_elapsed_time = 0;
volatile double angle = 0, raw_angle = 0, target = 0;
volatile double cur_angle = 0, print_angle = 0, num = 0, err = 0;

volatile int p = 0, i = 0, d = 0, spd = 0, up_lim = 255;
volatile int cntr = 0, print_spd = 0;
char buf[5];

void PID(){
    // PID
    err = target-rev;
    p = err*(110*target+500) + target*5 + 40;
    double temp = err*elapsed_time*(target*330+110);
    if (temp < 20 && temp > -20)
        i += temp;
    else if (temp >= 20)
        i += 20;
    else
        i -= 20;
    if (i < -75)
        i = -75;
    else if (i > 100+target*70)
        i = 100 + target * 70;
    d = (old_rev-rev)/elapsed_time*7;
    spd = p + i + d;
  
    if (target == 0)
        spd = 0;
    
    else{
        up_lim = target * 25 + 160;
    
        if (spd < 0)
            spd = 0;
        else{
            if (spd > up_lim)
                spd = up_lim;
            if (spd > 255)
                spd = 255;
        }
    }
    
    motor = spd / 255.0;
}

void control() {
    t.stop();
    elapsed_time = t.read();
    t.reset();
    t.start();
    
    blinker = !blinker;
  
    if (elapsed_time == 0)
        raw_rev = 7;
    else
        raw_rev = tick/360.0/elapsed_time;
    rev = raw_rev;
  
    // Discard unrealistic revs
    if (rev - 1.5*print_rev > 1.3 || rev > 6){
        rev = print_rev;
        cntr++;
    }
    else{
        old_rev = rev;
        raw_angle += tick;
        angle = (raw_angle + 2*cur_angle) / 3.;
    }
    if (rev<6.5)
        print_rev = (print_rev * 29. + rev) / 30.;
  
    PID();
}

void output(){
    pc.printf("Heading: %3.0f deg, Speed: %3.1f revs/sec\n", print_angle, print_rev);
}

int main() {
    encoder.rise(&control);
    pc.baud(38400);
    pc.printf("Hello\n");
    motor.period_us(800);
    t2.reset();
    t2.start();
    ticker.attach(&output, 0.1);
  
    while (true){
  
        if (pc.readable()){
            pc.scanf("%s", &buf);
            num = atof(buf);
            if (num > 6){
                angle -= cur_angle;
                raw_angle -= cur_angle;
                cur_angle = 0;
                elapsed_time = 99999999999;
                rev = 0;
            }
            else{
                target = num * 1.005;
                motor = (target*40.0+50.0) / 255.0;
                i = 0;
            }
        }
        
        if (t2 > 6){
            t2.stop();
            t2.reset();
            t2.start();
            i = 50;
        }

        cur_elapsed_time = t.read();
        if ((target<5 && cur_elapsed_time>elapsed_time) || cur_elapsed_time>1){
            rev = tick/360.0/cur_elapsed_time;
            if (rev - 1.2*print_rev > 0.2 || rev > 5)
                rev = print_rev;
            PID();
        }
       
        cur_angle = angle + print_rev*360*cur_elapsed_time;
        if (cur_angle > angle + tick)
            cur_angle = angle + tick;
        if (cur_angle > 360.){
            raw_angle -= 360.;
            angle -= 360.;
            cur_angle -= 360.;
        }
    
        if (rev < 6.5){
            print_rev = (print_rev * 349. + rev) / 350.;
            print_angle = cur_angle;
            if (print_angle < 10 || print_angle > 350)
                led = 1;
            else
                led = 0;
        }
    }
}