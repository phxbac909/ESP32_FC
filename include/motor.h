// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Khai báo hàm
void motor_init();
void motor_set_pulse(int pwm1, int pwm2, int pwm3, int pwm4 );
void motor_receive_command(String command);
void motor_stop();
void motor_detach();
void motor_calibrate();
boolean motor_is_active();

void motor_rotate(int throttle , float roll, float pitch, float yaw ) ;
void motor_set_throttle(int throttle);
#endif