#include "MPU6050_6Axis_MotionApps20.h"

#include "mecunum.h"
#include "LaunchSystem.h"
#include "SerialCommunicate.h"

mec::Mecanum mecanum;
// MPU6050 mpu;
// IMU imu;
SerialCommunicate serialCommunicate(&Serial);

bool LED_BLINK_STATE = true;

void setup() {
	serialCommunicate.init();
	// imu.init(&mpu);
	pinMode(13, OUTPUT);
	digitalWrite(13, LED_BLINK_STATE);
	Serial.flush();
}

void loop() {
	// float ypr[3];
	// imu.getYPR(ypr);
	// if(tmp_communication()){
	// 	speed *= 2.5*sqrt(MAX_VX*MAX_VX + MAX_VY*MAX_VY);
	// 	w *= MAX_W;
	// 	mecanum.movePolar(speed, angle, w);
	// }
	vector<uint8_t> readData;
	cmd::Command_Type command = serialCommunicate.read(readData);
	if(command == cmd::Command_Type::MOVE_POLAR){
		float speed, angle, w;
		uint8_t tmp_num[4];
		for(int i = 0; i< 4; ++i){
			tmp_num[i] = readData[i];
		}
		speed = ((float*)tmp_num)[0];
		speed *= 2.5*sqrt(MAX_VX*MAX_VX + MAX_VY*MAX_VY);

		for(int i = 0; i< 4; ++i){
			tmp_num[i] = readData[i + 4];
		}
		angle = ((float*)tmp_num)[0];
		
		for(int i = 0; i< 4; ++i){
			tmp_num[i] = readData[i + 8];
		}
		w = ((float*)tmp_num)[0];
		w *= MAX_W;

		mecanum.movePolar(speed, angle, w);
	}

}