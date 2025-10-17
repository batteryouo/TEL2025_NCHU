#include "PID.h"

PID::PID(float p, float i, float d, float maxErrorCum):_p(p), _i(i), _d(d){
	_maxErrorCum = fabs(maxErrorCum);
}

PID::~PID(){

}

float PID::calculatePID(float error){
	float delta_error = error - _lastError;
	_cumulativeError += error;
	_cumulativeError = constrain(_cumulativeError, -1*_maxErrorCum, _maxErrorCum);

	_lastError = error;

	return _p*error + _i*_cumulativeError + _d*delta_error;
}