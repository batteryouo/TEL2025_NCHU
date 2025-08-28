#ifndef IMU_6050_H
#define IMU_6050_H

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

#endif
