#include "MPU6050_6Axis_MotionApps20.h"

#include "mecunum.h"
#include "LaunchSystem.h"
#include "SerialCommunicate.h"
#include "PID.h"
mec::Mecanum mecanum;
MPU6050 mpu;
IMU imu;
SerialCommunicate serialCommunicate(&Serial);
BYJ48 byj28_left(22, 24, 26, 28);
BYJ48 byj28_right(23, 25, 27, 29);
TB6600 tb6600(37, 36);

PID launch_angle_PID(2, 0, 0, 0.2);

bool LED_BLINK_STATE = true;
float launch_angle = 0;

template <class _T>
_T uint8Vector2Value(vector<uint8_t> data, int bias){
	uint8_t tmp_num[sizeof(_T)];
	for(int i = 0; i< sizeof(_T); ++i){
		tmp_num[i] = data[i + bias];
	}
	return ((_T *)tmp_num)[0];
}

void setup() {
	serialCommunicate.init();
	imu.init(&mpu);
	tb6600.reset();
	pinMode(13, OUTPUT);
	digitalWrite(13, LED_BLINK_STATE);
	Serial.println("start!");
	Serial.flush();
	
}

void loop() {
	float ypr[3];
	imu.getYPR(ypr);

	vector<uint8_t> readData;
	cmd::Command_Type command = serialCommunicate.read(readData);

	if(command == cmd::Command_Type::MOVE_POLAR){
		float speed, angle, w;
		
		speed = uint8Vector2Value<float>(readData, 0);
		speed *= 2.5*sqrt(MAX_VX*MAX_VX + MAX_VY*MAX_VY);

		angle = uint8Vector2Value<float>(readData, 4);
		
		w = uint8Vector2Value<float>(readData, 8);
		w *= MAX_W;

		mecanum.movePolar(speed, angle, w);

	}

	if(command == cmd::Command_Type::LAUNCH_ANGLE_NORMALIZE){
		launch_angle = uint8Vector2Value<float>(readData, 0) * MAX_LAUNCH_ANGLE;
	}

	if(command == cmd::Command_Type::LAUNCH && tb6600.getStepAmount() == 0){
		if(readData[0] != 0){
			tb6600.setAngle(52);
		}
	}

	tb6600.run(2000);

	float byj_speed = launch_angle_PID.calculatePID(launch_angle - ypr[0]);

	if(fabs(byj_speed)< 0.05){
		byj_speed = 0;
	}
	byj28_left.run(byj_speed);
	byj28_right.run(byj_speed);
	
}