#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

typedef struct {
    float roll;
    float pitch;
    float yaw;
    
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
    
    float acc_roll;
    float acc_pitch;
} imu_data_t;

void mpu6050_init();
bool IMU_Update_And_Read(imu_data_t *out_data);

#endif