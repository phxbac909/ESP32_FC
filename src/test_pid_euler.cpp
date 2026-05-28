#include <Arduino.h>
#include "mpu6050.h"
#include "motor.h"
#include <ESP32_Servo.h>
#include "pid_euler.h"
#include <Wire.h>
#include <esp32_now.h>
#include "config.h"
#include "pid.h"
#include "config.h"

#ifdef TEST_PID

// --- TUNING PARAMETERS ---
double TEST_PID_ROLL_KP = 2;  double TEST_PID_ROLL_KI = 0.0;  double TEST_PID_ROLL_KD = 0.003;
double TEST_PID_PITCH_KP = 2; double TEST_PID_PITCH_KI = 0.0; double TEST_PID_PITCH_KD = 0.003;
double TEST_PID_YAW_KP = 2;   double TEST_PID_YAW_KI = 0.0;   double TEST_PID_YAW_KD = 0.003;

double TEST_RATE_ROLL_KP = 2;  double TEST_RATE_ROLL_KI = 0.0;  double TEST_RATE_ROLL_KD = 0.0;
double TEST_RATE_PITCH_KP = 2; double TEST_RATE_PITCH_KI = 0.0; double TEST_RATE_PITCH_KD = 0.0;
double TEST_RATE_YAW_KP = 2;   double TEST_RATE_YAW_KI = 0.0;   double TEST_RATE_YAW_KD = 0.0;

// --- KHỞI TẠO CÁC OBJECT PID (Thêm tiền tố test_) ---
static PID test_pid_angle_roll;
static PID test_pid_angle_pitch;
static PID test_pid_angle_yaw;

static PID test_pid_rate_roll;
static PID test_pid_rate_pitch;
static PID test_pid_rate_yaw;

// --- CONTROL VARIABLES (Thêm tiền tố test_) ---
static float test_roll_cmd = 0;
static float test_pitch_cmd = 0;
static float test_yaw_setpoint = 0; 
static int test_base_throttle = 1000;

// --- TASK ĐÃ ĐỔI TÊN ---
void pid_euler_test_task(void *parameter) {
    int log_counter = 0;
    imu_data_t current_imu; 
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 4 / portTICK_PERIOD_MS; // 250Hz -> dt = 4ms
    const float dt = 0.004f; 
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    IMU_Update_And_Read(&current_imu);
    test_yaw_setpoint = current_imu.yaw;

    while(1) {
        IMU_Update_And_Read(&current_imu);

        // Safety Stop
        // if (abs(current_imu.roll) >= 45 || abs(current_imu.pitch) >= 45) {
        //     motor_stop();
            
        //     test_pid_rate_roll.reset(); 
        //     test_pid_rate_pitch.reset();
        //     test_pid_rate_yaw.reset();
            
        //     vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS); 
        //     continue;
        // }
        
        // 1. TÍNH TOÁN VÒNG NGOÀI (ANGLE)
        float target_rate_roll = test_pid_angle_roll.compute(test_roll_cmd, current_imu.roll, dt);
        float target_rate_pitch = test_pid_angle_pitch.compute(test_pitch_cmd, current_imu.pitch, dt);
        float target_rate_yaw = test_pid_angle_yaw.compute(test_yaw_setpoint, current_imu.yaw, dt);

        // 2. TÍNH TOÁN VÒNG TRONG (RATE)
        float out_roll = test_pid_rate_roll.compute(target_rate_roll, current_imu.gyro_roll, dt);
        float out_pitch = test_pid_rate_pitch.compute(target_rate_pitch, current_imu.gyro_pitch, dt);
        float out_yaw = test_pid_rate_yaw.compute(target_rate_yaw, current_imu.gyro_yaw, dt);

        // 3. MIXING MOTOR
        int pwm1 = test_base_throttle + (int)out_pitch + (int)out_roll + (int)out_yaw; 
        int pwm2 = test_base_throttle + (int)out_pitch - (int)out_roll - (int)out_yaw; 
        int pwm3 = test_base_throttle - (int)out_pitch - (int)out_roll + (int)out_yaw;
        int pwm4 = test_base_throttle - (int)out_pitch + (int)out_roll - (int)out_yaw; 
        
        if (test_base_throttle < 1050) {
            pwm1 = 1000; pwm2 = 1000; pwm3 = 1000; pwm4 = 1000;
            
            test_pid_rate_roll.reset();
            test_pid_rate_pitch.reset();
            test_pid_rate_yaw.reset();
            
            test_yaw_setpoint = current_imu.yaw;
        } else {
            pwm1 = constrain(pwm1, 1000, 2000);
            pwm2 = constrain(pwm2, 1000, 2000);
            pwm3 = constrain(pwm3, 1000, 2000);
            pwm4 = constrain(pwm4, 1000, 2000);
        }
       
       
        // 4. LOGGING (10Hz)
        log_counter++;
        if (log_counter >= 100) { 
            DEBUG_PRINT("R: "); 
            DEBUG_PRINT(current_imu.roll); 
            DEBUG_PRINT(" | P: "); 
            DEBUG_PRINT(current_imu.pitch);
            DEBUG_PRINT(" | Y: "); 
            DEBUG_PRINT(current_imu.yaw);
            DEBUG_PRINT(" : ");
            DEBUG_PRINT(pwm1); DEBUG_PRINT("...");
            DEBUG_PRINT(pwm2); DEBUG_PRINT("...");
            DEBUG_PRINT(pwm3); DEBUG_PRINT("...");
            DEBUG_PRINT(pwm4);
            DEBUG_PRINTLN();
            log_counter = 0;
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// --- INIT ĐÃ ĐỔI TÊN ---
void pid_euler_test_init(void) {
    test_pid_angle_roll.init(TEST_PID_ROLL_KP, TEST_PID_ROLL_KI, TEST_PID_ROLL_KD, -200, 200);
    test_pid_angle_pitch.init(TEST_PID_PITCH_KP, TEST_PID_PITCH_KI, TEST_PID_PITCH_KD, -200, 200);
    test_pid_angle_yaw.init(TEST_PID_YAW_KP, TEST_PID_YAW_KI, TEST_PID_YAW_KD, -200, 200);

    test_pid_rate_roll.init(TEST_RATE_ROLL_KP, TEST_RATE_ROLL_KI, TEST_RATE_ROLL_KD, -120, 120);
    test_pid_rate_pitch.init(TEST_RATE_PITCH_KP, TEST_RATE_PITCH_KI, TEST_RATE_PITCH_KD, -120, 120);
    test_pid_rate_yaw.init(TEST_RATE_YAW_KP, TEST_RATE_YAW_KI, TEST_RATE_YAW_KD, -120, 120);
    
    test_roll_cmd = 0;
    test_pitch_cmd = 0;
    test_yaw_setpoint = 0;
    
    // Lưu ý: Đặt base_throttle là 1400 motor sẽ quay khá mạnh ngay khi khởi động
    test_base_throttle = 1400; 
    
    xTaskCreate(pid_euler_test_task, "PID Test Task", 4096, NULL, 5, NULL);
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting PID test. ...");  

  mpu6050_init();

//   esp32_now_init();

  pid_euler_test_init();

}

void loop() {

}




#endif