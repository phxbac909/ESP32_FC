#include <Arduino.h>
#include <ESP32_Servo.h>
#include "config.h"

#ifdef TEST_PIN
// Khai báo đối tượng Servo cho ESC
Servo motor;

// Cấu hình chân và mức throttle
const int MOTOR_PIN = 25;
const int THROTTLE_STOP = 1000;
const int THROTTLE_RUN = 1100;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== TEST 1 MOTOR - CHAN 25 ===");
    Serial.println("Dang khoi tao ESC...");

    // Gắn motor vào chân 25, dải xung 1000us - 2000us
    motor.attach(MOTOR_PIN, 1000, 2000);

    // Gửi tín hiệu 1000us để ESC nhận dạng mức ga thấp nhất (Arming)
    motor.writeMicroseconds(THROTTLE_STOP);
    
    // Đợi 3 giây để ESC phát tiếng beep khởi động xong
    delay(3000); 

    Serial.println("✅ Khoi tao thanh cong!");
    Serial.println("-> Nhap '1' de BAT motor (1100us)");
    Serial.println("-> Nhap '0' de TAT motor (1000us)");
}

void loop() {
    // Kiểm tra xem có dữ liệu từ Serial Monitor không
    if (Serial.available() > 0) {
        char command = Serial.read();

        // Xóa các ký tự thừa như \n hay \r (Enter)
        if (command == '\n' || command == '\r') {
            return; 
        }

        if (command == '1') {
            motor.writeMicroseconds(THROTTLE_RUN);
            Serial.println("Motor: ON (Throttle: 1100)");
        } 
        else if (command == '0') {
            motor.writeMicroseconds(THROTTLE_STOP);
            Serial.println("Motor: OFF (Throttle: 1000)");
        } 
        else {
            Serial.println("Lenh khong hop le. Chi nhap 0 hoac 1.");
        }
    }
}
#endif