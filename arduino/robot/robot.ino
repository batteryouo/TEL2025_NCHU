#include "mecunum.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "IMU.h"

#define DATA_LENGTH 16

mec::Mecanum mecanum;
MPU6050 mpu;
IMU imu;

bool LED_BLINK_STATE = true;

float speed = 0.0;
float angle = 0.0;
float w = 0.0;

void setup() {
	Serial.begin(115200);
	imu.init(&mpu);
	pinMode(13, OUTPUT);
	digitalWrite(13, LED_BLINK_STATE);

}

void loop() {
	float ypr[3];
	imu.getYPR(ypr);
	if(tmp_communication()){
		speed *= 2.5*sqrt(MAX_VX*MAX_VX + MAX_VY*MAX_VY);
		w *= MAX_W;
		mecanum.movePolar(speed, angle, w);
	}
	// delay(10);

}

int index = 0;
char recv_buffer[DATA_LENGTH] = {0};

bool tmp_communication() {

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

				LED_BLINK_STATE = !LED_BLINK_STATE;
				digitalWrite(13, LED_BLINK_STATE);
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