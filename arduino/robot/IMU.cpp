#include "Arduino.h"
#include "IMU.h"

#include "MPU6050_6Axis_MotionApps20.h"

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
	// mpu.PrintActiveOffsets();

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