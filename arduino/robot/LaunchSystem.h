#ifndef LAUNCH_SYSTEM_H
#define LAUNCH_SYSTEM_H

#define SWITCH_ON 1
#define SWITCH_OFF 0

#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

class IMU{
	public:
		IMU();
		~IMU();
		void init(MPU6050 *mpu);
		bool getYPR(float *ypr);

	private:
		MPU6050 *_mpu;
		/*---MPU6050 Control/Status Variables---*/
		bool _DMPReady = false;  // Set true if DMP init was successful
		uint8_t _MPUIntStatus;   // Holds actual interrupt status byte from MPU
		uint8_t _devStatus;      // Return status after each device operation (0 = success, !0 = error)
		uint16_t _packetSize;    // Expected DMP packet size (default is 42 bytes)
		uint8_t _FIFOBuffer[64]; // FIFO storage buffer

};

namespace byj{
	enum DIR{
		CLOCKWISE = 0,
		COUNTERCLOCKWISE
	};
};

class BYJ48{
	public:
		/**
		 * @brief Construct a new BYJ48 motor controller.
		 *
		 * @param IN1 Arduino pin connected to IN1 on driver board.
		 * @param IN2 Arduino pin connected to IN2 on driver board.
		 * @param IN3 Arduino pin connected to IN3 on driver board.
		 * @param IN4 Arduino pin connected to IN4 on driver board.
		 * @param minStepTime Minimum delay per step (microseconds). Default = 800.
		 * @param maxStepTime Maximum delay per step (microseconds). Default = 2000.
		 * @param step Steps per revolution (default = 4096 for 28BYJ-48).
		 */
		BYJ48(int IN1, int IN2, int IN3, int IN4, int minStepTime = 800, int maxStepTime = 2000, int step = 4096);
		~BYJ48();
		/**
		 * @brief Stop the motor (set all coils LOW).
		 */
		void stop();
		/**
		 * @brief Run the motor with a given speed.
		 *
		 * @param speed Normalized speed value:
		 *              - Negative → clockwise
		 *              - Positive → counterclockwise
		 *              - Zero → stop
		 *              - Magnitude range: 0.0 ~ 1.0
		 */		
		void run(float speed = 0);

	private:
		// -------------------- Motor Pins --------------------
		int _in1, _in2, _in3, _in4;  ///< Arduino pins connected to driver board
		int _pinArray[4];            ///< Internal pin storage
		int _currentPhase = 0;       ///< Current stepping phase index (0–7)

		// -------------------- Motor Parameters --------------------
		int _step = 0;         ///< Steps per revolution
		int _minStepTime = 0;  ///< Minimum microseconds per step
		int _maxStepTime = 0;  ///< Maximum microseconds per step

		void _phaseSet(int phase);
		void _phaseToPinState(int phase, int *pinState);
		void _CCW(int stepTime);
		void _CW(int stepTime);
		bool _ok2next(int stepTime);

		unsigned long _lastTime = 0;
}; 

class MicroSwitch{
	public:
		MicroSwitch::MicroSwitch();
		MicroSwitch(uint8_t pin);
		~MicroSwitch();
		bool state();
		void initPin(uint8_t pin);

	private:
		unsigned long _triggerTime;
		bool _state = false;
		uint8_t _pin;
		unsigned int _debouceTime = 50;
		bool _isInit = false;
		void _init(uint8_t pin);
};

class ElevationAngleSWState{
	public:
		ElevationAngleSWState(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t pinD);
		~ElevationAngleSWState();
		void getState(uint8_t* state);
	private:
		MicroSwitch _sw[4];
};

#endif