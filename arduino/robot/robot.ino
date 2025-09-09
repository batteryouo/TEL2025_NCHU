#include "MPU6050_6Axis_MotionApps20.h"


#include "mecunum.h"
#include "LaunchSystem.h"
#include "SerialCommunicate.h"

mec::Mecanum mecanum;
MPU6050 mpu;
IMU imu;
SerialCommunicate serialCommunicate(&Serial);

bool LED_BLINK_STATE = true;

void setup() {
	serialCommunicate.init();
	imu.init(&mpu);
	pinMode(13, OUTPUT);
	digitalWrite(13, LED_BLINK_STATE);
	Serial.flush();
}

void loop() {
	float ypr[3];
	imu.getYPR(ypr);
	// if(tmp_communication()){
	// 	speed *= 2.5*sqrt(MAX_VX*MAX_VX + MAX_VY*MAX_VY);
	// 	w *= MAX_W;
	// 	mecanum.movePolar(speed, angle, w);
	// }

}