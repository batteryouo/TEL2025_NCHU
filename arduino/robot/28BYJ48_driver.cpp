#include "Arduino.h"
#include "28BYJ48_driver.h"

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
