int counter = 0;
String str;
unsigned long time;
float mx, my, mz, acc1_x, acc1_y, acc1_z, acc2_x, acc2_y, acc2_z, voltage, kf_azimuth, kf_omega;


void setup() {
    Serial.begin(115200);
    randomSeed(0);
}

void loop() {
    if (counter % 5 != 0) {
        // machine readable generation
        str = "#";
        time = millis();
        mx = random(33000, 35000) / 100.0;
        my = random(33000, 35000) / 100.0;
        mz = random(33000, 35000) / 100.0;
        acc1_x = random(300) / 100.0;
        acc1_y = random(300) / 100.0;
        acc1_z = random(300) / 100.0;
        acc2_x = random(300) / 100.0;
        acc2_y = random(300) / 100.0;
        acc2_z = random(300) / 100.0;
        voltage = random(100, 330) / 100.0;
        kf_azimuth = random(360);
        kf_omega = random(600) / 100.0;
        str = String(str + time + ',' + mx + ',' + my + ',' + mz + ','+
                acc1_x + ',' + acc1_y + ',' + acc1_z + ',' + acc2_x + ',' +
                acc2_y + ',' + acc2_z + ',' + voltage  + ',' + kf_azimuth + ',' +
                kf_omega + "\r\n");
        
    } else {
        //human readable gen
        str = "";
    }
    Serial.print(str);
    counter++;
    delay(1000);
}