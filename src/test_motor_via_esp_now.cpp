#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "motor.h" 
#include "config.h"

#ifdef TEST_MOTOR_VIA_ESPNOW

// Đảm bảo file motor.h/cpp của bạn đã cấu hình đúng pin

// --- CẤU TRÚC GÓI TIN ĐIỀU KHIỂN MOTOR ---
// Phải khớp hoàn toàn với cấu trúc bên TX
typedef struct {
    uint8_t motor_cmd; 
} MotorTestPacket;

// Các biến cờ để giao tiếp giữa Callback và Loop
volatile uint8_t incoming_cmd = 255; // 255 là trạng thái chờ, không có lệnh mới
volatile bool cmd_received = false;

// --- CALLBACK NHẬN DỮ LIỆU ---
void on_data_receive(const uint8_t * mac, const uint8_t *incomingData, int len) {
    // Kể từ ESP32 Core v3.x, bạn có thể cần đổi tham số đầu tiên thành `const esp_now_recv_info_t *esp_now_info` 
    // Nếu bị lỗi identifier, hãy giữ nguyên `const uint8_t * mac` như thế này.
    
    if (len == sizeof(MotorTestPacket)) {
        MotorTestPacket *recvData = (MotorTestPacket *)incomingData;
        incoming_cmd = recvData->motor_cmd;
        cmd_received = true; // Bật cờ báo hiệu có lệnh mới
    }
}

// --- KHỞI TẠO ESP-NOW ---
void esp32_now_init() {
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Lỗi khởi tạo ESP-NOW");
        return;
    }
    
    esp_now_register_recv_cb(on_data_receive);
    Serial.println("ESP-NOW Initialized. Waiting for TX commands...");
}

// ==========================================
// CHƯƠNG TRÌNH CHÍNH
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Khởi tạo Motor và set PWM về mức an toàn (1000)
    motor_init();
    motor_set_pulse(1000, 1000, 1000, 1000);
    
    // Khởi tạo giao tiếp không dây
    esp32_now_init();
    
    Serial.println("--- MOTOR TEST (ESP-NOW RX) ---");
    Serial.println("Sử dụng Serial Monitor bên TX để gửi lệnh.");
}

void loop() {
    // Kiểm tra xem có lệnh mới từ TX gửi tới không
    if (cmd_received) {
        cmd_received = false; // Hạ cờ xuống
        uint8_t cmd = incoming_cmd; // Lưu lệnh ra biến cục bộ
        
        // Dừng tất cả trước khi chuyển trạng thái (giống logic cũ của bạn)
        motor_set_pulse(1000, 1000, 1000, 1000); 
        delay(100); 
        
        int spin_speed = 1050; // Tốc độ test an toàn
        
        // Phân tích lệnh từ TX gửi sang
        if (cmd == 1) {
            Serial.println("Spinning Motor 1 (FR - CW)");
            motor_set_pulse(spin_speed, 1000, 1000, 1000);
        } 
        else if (cmd == 2) {
            Serial.println("Spinning Motor 2 (FL - CCW)");
            motor_set_pulse(1000, spin_speed, 1000, 1000);
        } 
        else if (cmd == 3) {
            Serial.println("Spinning Motor 3 (RL - CW)");
            motor_set_pulse(1000, 1000, spin_speed, 1000);
        } 
        else if (cmd == 4) {
            Serial.println("Spinning Motor 4 (RR - CCW)");
            motor_set_pulse(1000, 1000, 1000, spin_speed);
        }
        else if (cmd == 5) {
            Serial.println("Spinning All Motors");
            motor_set_pulse(spin_speed, spin_speed, spin_speed, spin_speed);
        } 
        else if (cmd == 0) { // TX gửi 0 khi bạn bấm phím 'x'
            Serial.println("STOP ALL MOTORS");
            motor_set_pulse(1000, 1000, 1000, 1000);
        }
    }
}

#endif