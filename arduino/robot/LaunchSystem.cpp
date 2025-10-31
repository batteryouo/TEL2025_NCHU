#include <Arduino.h>

#include "MPU6050_6Axis_MotionApps20.h"

#include "LaunchSystem.h"

BYJ48::BYJ48(int IN1, int IN2, int IN3, int IN4, int minStepTime, int maxStepTime, int step):
_in1(IN1), _in2(IN2), _in3(IN3), _in4(IN4), _step(step), _minStepTime(minStepTime), _maxStepTime(maxStepTime){

	_pinArray[0] = _in1;
	_pinArray[1] = _in2;
	_pinArray[2] = _in3;
	_pinArray[3] = _in4;
	
	_currentPhase = 0;

	for(int i = 0; i< 4; ++i){
		pinMode(_pinArray[i], OUTPUT);
		digitalWrite(_pinArray[i], LOW);
	}

	_phaseSet(0);

	_lastTime = micros();
}

BYJ48::~BYJ48(){

}

void BYJ48::stop(){
	for(int i = 0; i< 4; ++i){
		digitalWrite(_pinArray[i], LOW);
	}	
	_currentPhase = 0;
}

void BYJ48::run(float speed){
	int dir = byj::DIR::COUNTERCLOCKWISE;

	if(speed > 0){
		dir = byj::DIR::COUNTERCLOCKWISE;
	}
	else if(speed < 0){
		dir = byj::DIR::CLOCKWISE;
	}
	else{
		stop();
		return;
	}

	speed = fabs(speed);

	int timeStep = (1.0-speed) * (float)(_maxStepTime - _minStepTime) + _minStepTime;

	if(timeStep < _minStepTime){
		timeStep = _minStepTime;
	}
	if(timeStep > _maxStepTime){
		timeStep = _maxStepTime;
	}

	if(dir == byj::DIR::COUNTERCLOCKWISE){
		_CCW(timeStep);
	}
	if(dir == byj::DIR::CLOCKWISE){
		_CW(timeStep);
	}
}

void BYJ48::_CCW(int timeStep){
	if(!_ok2next(timeStep)){
		return;
	}

	_currentPhase++;
	_phaseSet(_currentPhase);
}

void BYJ48::_CW(int timeStep){
	if(!_ok2next(timeStep)){
		return;
	}

	_currentPhase--;
	_phaseSet(_currentPhase);
}

bool BYJ48::_ok2next(int timeStep){
	unsigned long currentTime = micros();
	
	if(currentTime - _lastTime < (unsigned long)timeStep){
		return false;
	}
	else{
		_lastTime = currentTime;
	}
	return true;
}

void BYJ48::_phaseSet(int phase){
	_currentPhase = phase;

	if(_currentPhase > 7){
		_currentPhase = 0;
	}
	if(_currentPhase < 0){
		_currentPhase = 7;
	}

	int pinState[4] = {0};

	_phaseToPinState(_currentPhase, pinState);

	for(int i = 0; i< 4; ++i){
		digitalWrite(_pinArray[i], pinState[i]);
	}
}

void BYJ48::_phaseToPinState(int phase, int *pinState){
	
	int stateNum_0 = (phase + 1) % 8; // H H L L L L L H
	int stateNum_1 = (phase + 7) % 8; // L H H H L L L L
	int stateNum_2 = (phase + 5) % 8; // L L L H H H L L
	int stateNum_3 = (phase + 3) % 8; // L L L L L H H H
	pinState[0] = stateNum_0 <= 2 ? HIGH: LOW;
	pinState[1] = stateNum_1 <= 2 ? HIGH: LOW;
	pinState[2] = stateNum_2 <= 2 ? HIGH: LOW;
	pinState[3] = stateNum_3 <= 2 ? HIGH: LOW;

}

volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

void DMPDataReady() {
	MPUInterrupt = true;
}

IMU::IMU(){
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
		Wire.setWireTimeout(3000, true);
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif
}

IMU::~IMU(){

}

void IMU::init(MPU6050 *mpu){
	_mpu = mpu;

	// while (!Serial);
	// Serial.println(F("Initializing I2C devices..."));
	
	_mpu->initialize();

	pinMode(INTERRUPT_PIN, INPUT);
	// // verify connection
	// Serial.println(F("Testing device connections..."));
	// Serial.println(_mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	// Serial.println(F("Initializing DMP..."));
	_devStatus = _mpu->dmpInitialize();
	_mpu->CalibrateGyro(15);
	_mpu->CalibrateAccel(15);
	_mpu->PrintActiveOffsets();

	// make sure it worked (returns 0 if so)
	if (_devStatus == 0) {
		// turn on the DMP, now that it's ready
		// Serial.println(F("Enabling DMP..."));
		_mpu->setDMPEnabled(true);

		// enable Arduino interrupt detection
		// Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
		_MPUIntStatus = _mpu->getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		// Serial.println(F("DMP ready! Waiting for first interrupt..."));
		_DMPReady = true;

		// get expected DMP packet size for later comparison
		_packetSize = _mpu->dmpGetFIFOPacketSize();
	}
	else {
		// // 1 = initial memory load failed
		// // 2 = DMP configuration updates failed
		// // (if it's going to break, usually the code will be 1)
		// Serial.print(F("DMP Initialization failed (code "));
		// Serial.print(_devStatus);
		// Serial.println(F(")"));
	}
}

bool IMU::getYPR(float *ypr){
	if (!_DMPReady){
		return false;
	} 
	if(_mpu->dmpGetCurrentFIFOPacket(_FIFOBuffer)) {
		Quaternion q;           // [w, x, y, z]         Quaternion container
		_mpu->dmpGetQuaternion(&q, _FIFOBuffer);
		_mpu->dmpGetEuler(ypr, &q);
	}
	return true;
}

MicroSwitch::MicroSwitch(){

}

MicroSwitch::MicroSwitch(uint8_t pin){
	_init(pin);
}

MicroSwitch::~MicroSwitch(){

}

bool MicroSwitch::state(){

	if(!_isInit){
		return _state;
	}

	bool current_state = digitalRead(_pin);
	
	if(current_state != _state && millis() - _triggerTime > _debouceTime){
		_state = current_state;
		_triggerTime = millis();
	}

	return _state;
}

void MicroSwitch::initPin(uint8_t pin){
	_init(pin);
}

void MicroSwitch::_init(uint8_t pin){
	_pin = pin;
	_triggerTime = millis();
	pinMode(pin, INPUT_PULLUP);
	_isInit = true;
}

ElevationAngleSWState::ElevationAngleSWState(uint8_t pinA, uint8_t pinB){
	_sw[0].initPin(pinA);	
	_sw[1].initPin(pinB);	
}

ElevationAngleSWState::~ElevationAngleSWState(){

}

void ElevationAngleSWState::getState(uint8_t *state){
	for(int i = 0; i< 2; ++i){
		state[i] = _sw[i].state();
	}
}


TB6600::TB6600(uint8_t pulsePin, uint8_t directionPin, unsigned int stepsPerRevolution){
	_pulsePin = pulsePin;
	_directionPin = directionPin;
	_stepsPerRevolution = stepsPerRevolution;

	_lastTimeRun = micros();

	pinMode(directionPin, OUTPUT);
	pinMode(pulsePin, OUTPUT);
}

TB6600::~TB6600(){

}

void TB6600::setAngle(float angle){
	_settingStep = (int)((angle/360.0) * _stepsPerRevolution);
}

void TB6600::run(unsigned long timeStamp){
	unsigned long currentTime = micros();
	if(_settingStep == 0 || (currentTime-_lastTimeRun) < timeStamp){
		return;
	}

	if(_settingStep > 0){
		digitalWrite(_directionPin, HIGH);
		_settingStep--;
	}
	else if(_settingStep < 0){
		digitalWrite(_directionPin, LOW);
		_settingStep++;
	}

	digitalWrite(_pulsePin, HIGH);
	delayMicroseconds(10);
	digitalWrite(_pulsePin, LOW);
	_lastTimeRun = currentTime;
}

int TB6600::getStepAmount(){
	return _settingStep;
}

void TB6600::reset(){
	_settingStep = 0;
	_lastTimeRun = 0;
}

AngleReader::AngleReader(uint8_t pin, int bias, int maxValue){
	_pin = pin;
	_bias = bias;
	_maxValue = maxValue;

	pinMode(_pin, INPUT);
}

AngleReader::~AngleReader(){

}

float AngleReader::readNormalized(){
	int rawValue = analogRead(_pin);
	float normalizedValue = ((float)(rawValue - _bias)) / ((float)_maxValue - _bias);
	
	return normalizedValue;
}

int AngleReader::read(){
	int rawValue = analogRead(_pin);
	return (float)(rawValue);
}

void AngleReader::setPin(uint8_t pin){
	_pin = pin;
	pinMode(_pin, INPUT);
}

void AngleReader::setBias(int bias){
	_bias = bias;
}

void AngleReader::setMaxValue(int maxValue){
	_maxValue = maxValue;
}