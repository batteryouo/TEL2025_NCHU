#include "MPU6050_6Axis_MotionApps20.h"

#include "mecunum.h"
#include "LaunchSystem.h"
#include "SerialCommunicate.h"
#include "PID.h"
mec::Mecanum mecanum;
// MPU6050 mpu;
// IMU imu;
SerialCommunicate serialCommunicate(&Serial);
BYJ48 byj28_left(22, 24, 26, 28);
BYJ48 byj28_right(23, 25, 27, 29);
TB6600 tb6600(37, 36);
ElevationAngleSWState elevationAngleSWState(30, 32);
AngleReader angleReader(A0, 0, 1023);

PID launch_angle_PID(1, 0, 0.1, 0.2);

bool LED_BLINK_STATE = true;
float launch_angle = 0;

unsigned long reset_timer = 0;

template <class _T>
_T uint8Vector2Value(vector<uint8_t> data, int bias){
	uint8_t tmp_num[sizeof(_T)];
	for(int i = 0; i< sizeof(_T); ++i){
		tmp_num[i] = data[i + bias];
	}
	return ((_T *)tmp_num)[0];
}

void float2uint8_t(float num, uint8_t *outputArray){
	for(int i = 0; i< 4; ++i){
		outputArray[i] = ((uint8_t *)&num)[i];
	}
}

void setup() {
	serialCommunicate.init();
	// imu.init(&mpu);
	tb6600.reset();
	pinMode(13, OUTPUT);
	digitalWrite(13, LED_BLINK_STATE);
	Serial.println("start!");
	Serial.flush();

	int _bias = 0;
	int _maxValue = 1023;

	// tb6600.setAngle(60);
	// while(tb6600.getStepAmount()){
	// 	tb6600.run(1000);
	// 	if(!tb6600.getStepAmount()){
	// 		tb6600.setAngle(60);
	// 	}
	// }
	
	uint8_t state[2];
	elevationAngleSWState.getState(state);
	while(state[1] == SWITCH_OFF){
		elevationAngleSWState.getState(state);
		byj28_left.run(-1.0);
	}
	_bias = angleReader.read();
	byj28_left.stop();

	while(state[0] == SWITCH_OFF){
		elevationAngleSWState.getState(state);
		byj28_left.run(1.0);
	}

	_maxValue = angleReader.read();
	byj28_right.stop();
	angleReader.setBias(_bias);
	angleReader.setMaxValue(_maxValue);
}

void loop() {

	vector<uint8_t> readData;
	cmd::Command_Type command = serialCommunicate.read(readData);
	// process command
	if(command == cmd::Command_Type::MOVE_POLAR){
		float speed, angle, w;
		
		speed = uint8Vector2Value<float>(readData, 0);
		speed *= 4*sqrt(MAX_VX*MAX_VX + MAX_VY*MAX_VY);

		angle = uint8Vector2Value<float>(readData, 4);
		
		w = uint8Vector2Value<float>(readData, 8);
		w *= MAX_W;

		mecanum.movePolar(speed, angle, w);

	}

	if(command == cmd::Command_Type::LAUNCH_ANGLE_NORMALIZE){
		launch_angle = uint8Vector2Value<float>(readData, 0);
	}

	if(command == cmd::Command_Type::LAUNCH && tb6600.getStepAmount() == 0){
		if(readData[0] != 0){
			tb6600.setAngle(360.0/7.0);
		}
	}

	if(command == cmd::Command_Type::None && micros() - reset_timer > 1000000){
		LED_BLINK_STATE = !LED_BLINK_STATE;
		digitalWrite(LED_BUILTIN, LED_BLINK_STATE);
		reset_timer = micros();
	}
	// process launch system
	tb6600.run(2000);

	float readAngle = angleReader.readNormalized();
	float byj_speed = launch_angle_PID.calculatePID(launch_angle - readAngle);
	
	if(fabs(byj_speed)< 0.05){
		byj_speed = 0;
	}
	uint8_t state[2];
	elevationAngleSWState.getState(state);
	if(state[0] == SWITCH_ON && byj_speed > 0 || state[1] == SWITCH_ON && byj_speed < 0){
		byj_speed = 0;
	}
	byj28_left.run(byj_speed);

	// send data back
	uint8_t tmp[4] = {0};
	vector<uint8_t> readAngle_vector;

	float2uint8_t(readAngle, tmp);
	for(int i = 0; i< 4; ++i){
		readAngle_vector.push_back(tmp[i]);
	}
	for(int i = 0; i< 8; ++i){
		readAngle_vector.push_back( (uint8_t)0 );
	}
	serialCommunicate.write(readAngle_vector, cmd::Command_Type::IMU_YPR);

	if(tb6600.getStepAmount() == 0){
		vector<uint8_t> launch_state_vector;
		launch_state_vector.push_back((uint8_t)0);
		serialCommunicate.write(launch_state_vector, cmd::Command_Type::LAUNCH);
	}
	else{
		vector<uint8_t> launch_state_vector;
		launch_state_vector.push_back((uint8_t)1);
		serialCommunicate.write(launch_state_vector, cmd::Command_Type::LAUNCH);
	}
	
}