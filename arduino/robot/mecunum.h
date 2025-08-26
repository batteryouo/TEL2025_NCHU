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

    typedef struct _MotorPin {
        int RPWM_OUTPUT;
        int LPWM_OUTPUT;
    } MotorPin;

    class Mecanum {
                    
            public:
                    Mecanum();
                    ~Mecanum();
                    void move(float vx, float vy, float w);
                    void stop();
                    void movePolar(float speed, float angle, float w = 0);
                    
            private:
                    void _motor(int pinR, int pinL, int speed);
                    void _computeMecanumParam(float vx, float vy, float w, int *motor_pwm);
                    MotorPin motorPins[4];
            
    };

    enum MotorID {
        MOTOR_FL = 0,
        MOTOR_RL,
        MOTOR_FR,
        MOTOR_RR
    };


    
};

#endif