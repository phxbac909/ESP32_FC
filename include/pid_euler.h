#ifndef PID_EULER_H
#define PID_EULER_H

#include <Arduino.h>
#include <data_struct.h>


// Function declarations
void pid_euler_receive_command(int roll_command, int pitch_command);
void pid_euler_stop_task();
void pid_euler_set_base_throttle(int throttle);
void pid_euler_set_tuning(PidData data);
void pid_euler_init(void);
void pid_euler_set_angle(float roll, float pitch, float yaw);

#endif