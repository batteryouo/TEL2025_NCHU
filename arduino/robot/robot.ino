#include "MPU6050_6Axis_MotionApps20.h"

#include "mecunum.h"
#include "LaunchSystem.h"
#include "SerialCommunicate.h"

mec::Mecanum mecanum;
// MPU6050 mpu;
// IMU imu;
SerialCommunicate serialCommunicate(&Serial);
BYJ48 byj28_left(22, 24, 26, 28);
BYJ48 byj28_right(30, 32, 34, 36);

bool LED_BLINK_STATE = true;

void setup() {
	serialCommunicate.init();
	// imu.init(&mpu);
	pinMode(13, OUTPUT);
	digitalWrite(13, LED_BLINK_STATE);
	Serial.flush();

	// mecanum.movePolar(5, -PI, 0);
	// delay(5000);
	// mecanum.movePolar(5, -PI/2, 0);
	// delay(5000);
	// mecanum.movePolar(5, PI/2, 0);
	// delay(5000);
}

void loop() {
	// float ypr[3];
	// imu.getYPR(ypr);

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