#ifndef PID_H
#define PID_H

class PID{
	public:
		PID(float p, float i, float d, float maxErrorCum = 1);
		~PID();

		float calculatePID(float error);
	private:
		float _p = 0;
		float _i = 0;
		float _d = 0;

		float _lastError = 0;
		float _cumulativeError = 0;
		float _maxErrorCum = 0;

};

#endif
