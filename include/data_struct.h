#pragma once
#include <Arduino.h>

typedef struct  {
    int16_t speed;
    float roll;
    float pitch;
    float yaw;
} __attribute__((packed)) ControlData;

struct __attribute__((packed)) PidData {

 uint16_t version;
  
  // VÒNG GÓC (ANGLE CONTROL)
  double angle_kp_roll,  angle_ki_roll,  angle_kd_roll;
  double angle_kp_pitch, angle_ki_pitch, angle_kd_pitch;
  double angle_kp_yaw,   angle_ki_yaw,   angle_kd_yaw;

  // VÒNG TỐC ĐỘ (RATE CONTROL)
  double rate_kp_roll,   rate_ki_roll,   rate_kd_roll;
  double rate_kp_pitch,  rate_ki_pitch,  rate_kd_pitch;
  double rate_kp_yaw,    rate_ki_yaw,    rate_kd_yaw;
};
struct __attribute__((packed)) StopSignal {
    byte id;
};

struct DroneLog {
    int16_t version_setting;
    int16_t time;
    float roll_target;
    float pitch_target;
    float yaw_target;
    float roll;
    float pitch;
    float yaw;
    float gyro_roll_target;
    float gyro_pitch_target;
    float gyro_yaw_target;
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
    int16_t pwm1;
    int16_t pwm2;
    int16_t pwm3;
    int16_t pwm4;
}__attribute__((packed));