// normal update
#define MSG_UPDATE 1
// ping response, remaining data in packet useless
#define MSG_PING 2

#define BUFFER_SIZE 7

// 8 bytes
struct monomsg{
    // always x09,x09,x09,x09, serves to mark the begin of a msg
    //uint32_t alignment;
    uint8_t msg_type;
    // 1: telemetry control, 2: RC control
    uint8_t telem_ctrl;
    // voltage * 100
    uint16_t voltage;
    uint16_t flapPWM;
    uint16_t throttlePWM;
};

void setup(){
    Serial.begin(115200);
    pinMode(13,OUTPUT);
    digitalWrite(13,LOW);
}
uint8_t buffer[BUFFER_SIZE] = {0};
uint8_t buffer_index = 0;
uint16_t received = 0;
unsigned long ts = 0;


void loop(){
    struct monomsg packet;
    packet.msg_type = 1;
    packet.telem_ctrl = 1;
    packet.voltage = 1108;
    packet.flapPWM = received;
    packet.throttlePWM = 1600;
    if (millis()-ts>20){
        Serial.write((const uint8_t *)&packet,sizeof(struct monomsg));
        ts = millis();
    }
    //Serial.println("Hello");
    //Serial.println(sizeof(struct monomsg));

    while (Serial.available()>0){
        buffer[buffer_index%BUFFER_SIZE] = Serial.read();
        buffer_index++;
    }

    if (buffer_index>=BUFFER_SIZE){
        // the package is not properly aligned
        if (not (buffer[0]==9 and buffer[1]==9)){
            // drop the first byte in buffer and shift remaining data forward
            for (int i=0;i<BUFFER_SIZE-1;i++){
                buffer[i] = buffer[i+1];
            }
            buffer_index--;
        } else{
            // process
            buffer_index = 0;

            int msgType = buffer[2];
            // lower byte is sent first
            uint16_t flap = ((uint16_t)buffer[4]<<8)+buffer[3];
            received = flap;
            //Serial.println(flap);
            if (flap==1500u){
                digitalWrite(13,HIGH);
            } else{
                digitalWrite(13,LOW);
            }

        }
    }
}
