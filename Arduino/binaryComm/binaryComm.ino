// normal update
#define MSG_UPDATE 1
// ping response, remaining data in packet useless
#define MSG_PING 2

// 8 bytes
struct monomsg{
    uint8_t msg_type;
    // voltage * 100
    uint16_t voltage;
    uint16_t flapPWM;
    uint16_t throttlePWM;
    // 1: telemetry control, 2: RC control
    uint8_t telem_ctrl;
};

void setup(){
    Serial.begin(115200);
    pinMode(13,OUTPUT);
    digitalWrite(13,LOW);
}
uint8_t buffer[5] = {0};

void loop(){
    struct monomsg packet;
    packet.msg_type = 1;
    packet.voltage = 1105;
    packet.flapPWM = 1500;
    packet.throttlePWM = 1707;
    packet.telem_ctrl = 1;
    //Serial.write((const uint8_t *)&packet,sizeof(struct monomsg));
    Serial.println("Hello");

    if (Serial.available()>=5){
        for (int i=0;i<5;i++){
            buffer[i] = Serial.read();
        }
        int msgType = buffer[0];
        // lower byte is sent first
        uint16_t flap = ((uint16_t)buffer[2]<<8)+buffer[1];
        //Serial.println(flap);
        if (flap==1500){
            digitalWrite(13,HIGH);
        } else{
            digitalWrite(13,LOW);
        }

    }
    delay(20);
}
