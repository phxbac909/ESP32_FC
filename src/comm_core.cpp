#include "comm_core.h"
#include "pid_euler.h"
#include "motor.h"
#include "config.h" 

 volatile unsigned long last_packet_time; 
 ControlData data;
 PidData receivedPid;

void proceedReceivedData(uint8_t *incomingData, size_t length) {
    if (length < 1 || incomingData == nullptr) return;
    
    uint8_t id = incomingData[0];
    
    switch (id) {
        case 1: { // ControlData
            if (length < 15) return;
            
            data.speed = (int16_t)(incomingData[1] | (incomingData[2] << 8));
            memcpy(&data.roll,  &incomingData[3], 4);
            memcpy(&data.pitch, &incomingData[7], 4);
            memcpy(&data.yaw,   &incomingData[11], 4);
            
            pid_euler_set_base_throttle(data.speed);
            pid_euler_set_angle(data.roll, data.pitch, data.yaw);
            break;
        }
       case 2: { // PidEulerData
            if (length < (sizeof(PidData) + 1)) return;
            
            memcpy(&receivedPid, &incomingData[1], sizeof(PidData));
            pid_euler_set_tuning(receivedPid);
            
            DEBUG_PRINTF("=== PID Tuned : %.2f\n", receivedPid.angle_kd_pitch);
            break;
        }
        case 0: { // Khẩn cấp
            motor_detach();
            break;
        }
    }
}

// =======================================================
// FAILSAFE TASK (DÙNG CHO MỌI LOẠI SÓNG)
// =======================================================
void failsafeTask(void *pvParameters) {
    unsigned long last_step_time = 0; 
    
    for (;;) { 
        unsigned long current_time = millis();

        // Kiểm tra mất tín hiệu (>1000ms)
        if (current_time - last_packet_time >= 1000) {
            
            if (current_time - last_step_time >= 500) {
                last_step_time = current_time;

                // Trả góc nghiêng về 0 để tự cân bằng
                data.roll = 0.0f;
                data.pitch = 0.0f;
                data.yaw = 0.0f;

                // Hạ ga từ từ
                if (data.speed > 1000) {
                    data.speed -= 4;
                    if (data.speed < 1000) data.speed = 1000; 
                }

                pid_euler_set_angle(data.roll, data.pitch, data.yaw);
                pid_euler_set_base_throttle(data.speed);

                DEBUG_PRINTF("[FAILSAFE] Mất sóng! Hạ cánh. Throttle: %d\n", data.speed);
            }
        } else {
            last_step_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
