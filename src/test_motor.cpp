#include <Arduino.h>
#include "motor.h" 
#include "config.h"
// Đảm bảo file motor.h/cpp của bạn đã cấu hình đúng pin
#ifdef TEST_MOTOR_VIA_SERIAL

void setup() {
    Serial.begin(115200);
    motor_init();
    motor_set_pulse(1000, 1000, 1000, 1000);
    Serial.println("--- MOTOR TEST ---");
    Serial.println("Send 1, 2, 3, 4 to spin motor. x to stop.");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        motor_set_pulse(1000, 1000, 1000, 1000); // Stop all first
        delay(100);
        
        int spin_speed = 1050; // Tốc độ test
        
        if (c == '1') {
            Serial.println("Spinning Motor 1 (FR - CW)");
            motor_set_pulse(spin_speed, 1000, 1000, 1000);
        } else if (c == '2') {
            Serial.println("Spinning Motor 2 (FL - CCW)");
            motor_set_pulse(1000, spin_speed, 1000, 1000);
        } else if (c == '3') {
            Serial.println("Spinning Motor 3 (RL - CW)");
            motor_set_pulse(1000, 1000, spin_speed, 1000);
        } else if (c == '4') {
            Serial.println("Spinning Motor 4 (RR - CCW)");
            motor_set_pulse(1000, 1000, 1000, spin_speed);
        }else if (c == '5') {
            Serial.println("Spinning All");
            motor_set_pulse(spin_speed, spin_speed, spin_speed, spin_speed);
        } else if (c == 'x') {
            Serial.println("STOP");
            motor_set_pulse(1000, 1000, 1000, 1000);
        }
    }
}

#endif