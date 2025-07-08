#include <Arduino.h>
#include "mecunum.h"

mec::Mecanum::Mecanum() {
    motorPins[0] = { RPWM_OUTPUT_FL, LPWM_OUTPUT_FL };
    motorPins[1] = { RPWM_OUTPUT_FR, LPWM_OUTPUT_FR };
    motorPins[2] = { RPWM_OUTPUT_RL, LPWM_OUTPUT_RL };
    motorPins[3] = { RPWM_OUTPUT_RR, LPWM_OUTPUT_RR };

    for (int i = 0; i < 4; ++i) {
        pinMode(motorPins[i].RPWM_OUTPUT, OUTPUT);
        pinMode(motorPins[i].LPWM_OUTPUT, OUTPUT);
    }
}

mec::Mecanum::~Mecanum() {
}

void mec::Mecanum::move(float vx, float vy, float w){


    int motor_pwm[4] = {0};

    _computeMecanumParam(vx, vy, w, motor_pwm);

    for(int i = 0; i< 4; ++i){
        _motor(motorPins[i].RPWM_OUTPUT, motorPins[i].LPWM_OUTPUT, motor_pwm[i]);
    }
    
}

void mec::Mecanum::_motor(int pinR, int pinL, int speed) {

    int pwm = constrain(abs(speed), 0, 255);

    if (speed >= 0) {
        analogWrite(pinR, pwm);
        analogWrite(pinL, 0);
    }
    else{
        analogWrite(pinR, 0);
        analogWrite(pinL, pwm);
    }
}

void mec::Mecanum::_computeMecanumParam(float vx, float vy, float w, int *motor_pwm){
    
    float length_x_y = Chassis_LX + Chassis_LY;

    float max_motor_w = fabs( (MAX_VX + MAX_VY + length_x_y*MAX_W)/Mecanum_Wheel_Radius ); // add abs
    float min_motor_w = -max_motor_w;
    
    float tmp_motor_w[4] = {0};
    tmp_motor_w[0] = (vx - vy - length_x_y*w )/Mecanum_Wheel_Radius;
    tmp_motor_w[1] = (vx + vy + length_x_y*w )/Mecanum_Wheel_Radius;
    tmp_motor_w[2] = (vx + vy - length_x_y*w )/Mecanum_Wheel_Radius;
    tmp_motor_w[3] = (vx - vy + length_x_y*w )/Mecanum_Wheel_Radius;

    // normalize
    for(int i = 0; i< 4; ++i){
        float tmp_w = (tmp_motor_w[i] - min_motor_w) / ( max_motor_w - min_motor_w);
        motor_pwm[i] = (tmp_w - 0.5) * 255;

        motor_pwm[i] = motor_pwm[i] > 255? 255: motor_pwm[i];
        motor_pwm[i] = motor_pwm[i] < -255? -255: motor_pwm[i];
    }


}