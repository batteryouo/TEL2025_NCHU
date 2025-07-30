#include "mecunum.h"

#define DATA_LENGTH 16
mec::Mecanum mecanum;

bool BULLT_IN_LED_STATE = true;

float speed = 0.0;
float angle = 0.0;
float w = 0.0;

void setup() {
	Serial.begin(115200);
	pinMode(13, OUTPUT);
	digitalWrite(13, BULLT_IN_LED_STATE);
}

void loop() {
	if(tmp_communication()){
		speed *= 5*sqrt(MAX_VX*MAX_VX + MAX_VY*MAX_VY);
		w *= MAX_W;
		mecanum.movePolar(speed, angle, w);
	}


}



bool tmp_communication() {
    static int index = 0;
    static char recv_buffer[DATA_LENGTH] = {0};
	while(Serial.available() > 0) {
		char c = Serial.read();
		if(index == 0 && c != '$') {
			continue;
		}
		recv_buffer[index] = c;
		index++;

		if(index >= DATA_LENGTH){
			if(recv_buffer[1] == 0x14 && recv_buffer[DATA_LENGTH - 2] == '\r' && recv_buffer[DATA_LENGTH - 1] == '\n'){
				
				index = 0;  // Reset index after processing a complete message

				speed = *((float*)(recv_buffer+2));
				angle = *((float*)(recv_buffer+6));
				w = *((float*)(recv_buffer+10));
				Serial.flush();
				return true;
			}
			else{
				index = 0;
			}
		}

	}

	return false;

}