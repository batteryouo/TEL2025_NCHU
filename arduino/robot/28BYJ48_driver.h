#ifndef BYJ48_DRIVER_H
#define BYJ48_DRIVER_H

#include "Arduino.h"

namespace byj{
	enum DIR{
		CLOCKWISE = 0,
		COUNTERCLOCKWISE
	};
};

class BYJ48{
	public:
		BYJ48(int IN1, int IN2, int IN3, int IN4, int minStepTime = 800, int maxStepTime = 2000, int step = 4096);
		~BYJ48();

		void stop();
		void run(float speed = 0);

	private:
		int _in1, _in2, _in3, _in4;
		int _pinArray[4];
		int _currentPhase = 0;

		int _step = 0;
		int _minStepTime = 0;
		int _maxStepTime = 0;

		void _phaseSet(int phase);
		void _phaseToPinState(int phase, int *pinState);
		void _CCW(int stepTime);
		void _CW(int stepTime);
		bool _ok2next(int stepTime);

		unsigned long _lastTime = 0;
}; 

#endif