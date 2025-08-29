#ifndef ROBOT_H
#define ROBOT_H

#define RPWM_OUTPUT_FL 4
#define LPWM_OUTPUT_FL 3

#define RPWM_OUTPUT_FR 6
#define LPWM_OUTPUT_FR 5

#define RPWM_OUTPUT_RL 9
#define LPWM_OUTPUT_RL 10

#define RPWM_OUTPUT_RR 11
#define LPWM_OUTPUT_RR 12

#define Chassis_LX 0.21 // unit: (m)
#define Chassis_LY 0.19 // unit: (m)
#define Mecanum_Wheel_Radius 0.07 // unit: (m)

#define MAX_VX 1 // unit: (m/s)
#define MAX_VY 1 // unit: (m/s)
#define MAX_W 16 // unit: (rad/s)


namespace mec{
	/**
	 * @brief Structure representing a motor’s PWM pins.
	 */
	typedef struct _MotorPin {
		int RPWM_OUTPUT;
		int LPWM_OUTPUT;
	} MotorPin;
	
	class Mecanum {
	
			public:

					Mecanum();
					~Mecanum();

					/**
					 * @brief Move the chassis with velocity components (vx, vy, w).
					 *
					 * @param vx Velocity in x direction (m/s).
					 * @param vy Velocity in y direction (m/s).
					 * @param w  Angular velocity (rad/s).
					 */
					void move(float vx, float vy, float w);

					/**
					 * @brief Stop all motors immediately.
					 */					
					void stop();

					/**
					 * @brief Move using polar coordinates.
					 *
					 * @param speed Linear speed (m/s).
					 * @param angle Direction angle (rad).
					 * @param w     Angular velocity (rad/s). Default = 0.
					 */					
					void movePolar(float speed, float angle, float w = 0);
					
			private:
					/**
					 * @brief Control one motor with PWM signals.
					 *
					 * @param pinR Forward PWM pin.
					 * @param pinL Reverse PWM pin.
					 * @param speed Speed input (-255 ~ 255).
					 */
					void _motor(int pinR, int pinL, int speed);

					/**
					 * @brief Compute individual motor PWM values from chassis velocity.
					 *
					 * @param vx Velocity in x direction (m/s).
					 * @param vy Velocity in y direction (m/s).
					 * @param w  Angular velocity (rad/s).
					 * @param motor_pwm Array to store computed PWM values for 4 motors.
					 */
					void _computeMecanumParam(float vx, float vy, float w, int *motor_pwm);

					MotorPin motorPins[4];  ///< Array of motor pin configurations.
			
	};

	/**
	 * @brief Motor ID corresponding to chassis position.
	 */
	enum MotorID {
		MOTOR_FL = 0,
		MOTOR_RL,
		MOTOR_FR,
		MOTOR_RR
	};

	
};

#endif