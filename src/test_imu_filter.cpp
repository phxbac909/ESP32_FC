#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "motor.h" 
#include "mpu6050.h" 
#include "config.h"


#ifdef TEST_IMU_FILTER

// --- Cấu hình WiFi ---
const char* ssid = "Dung co bat";
const char* password = "hoilamgi123";

// --- Cấu hình UDP ---
WiFiUDP udp;
const unsigned int localPort = 8888;
IPAddress remoteIP; 
unsigned int remotePort = 0;
bool clientConnected = false;

// --- Biến toàn cục ---
int current_throttle = 1000;
unsigned long last_packet_time_ = 0;
unsigned long last_telemetry_time = 0;
imu_data_t imu_data;

// --- BIẾN QUẢN LÝ LOOPTIME 250Hz ---
const unsigned long LOOP_TIME_US = 4000; // 4000 us = 4ms = 250Hz
unsigned long last_loop_time = 0;

void setup() {
    Serial.begin(115200);

 

    // 2. Khởi tạo IMU
    mpu6050_init();

    unsigned long tare_start = millis();
    while(millis() - tare_start < 3000) {

        IMU_Update_And_Read(&imu_data);  // Mahony chạy bình thường, Ki tích phân
        delayMicroseconds(4000);      // 250Hz
    }

    mpu6050_tare(); // Set điểm 0

    IPAddress local_IP(192, 168, 100, 207); 
    IPAddress gateway(192, 168, 100, 1);    
    IPAddress subnet(255, 255, 255, 0);
    
    // Áp dụng cấu hình IP tĩnh
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("STA Failed to configure");
    }

    // 3. Kết nối WiFi
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

       // 1. Khởi tạo Motor
    motor_init();
    motor_set_pulse(1000, 1000, 1000, 1000); 

    Serial.println("\nWiFi connected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());

    // 4. Mở port UDP
    udp.begin(localPort);
    Serial.printf("Listening on UDP port %d\n", localPort);
}

void loop() {
    unsigned long current_micros = micros();

    // KIỂM TRA ĐÚNG 4000us (250Hz) MỚI CHO CHẠY VÒNG LẶP
    if (current_micros - last_loop_time >= LOOP_TIME_US) {
        last_loop_time = current_micros; // Cập nhật mốc thời gian

        // ==========================================
        // 1. READ SENSORS & FILTER (Mất ~1.5ms)
        // ==========================================
        IMU_Update_And_Read(&imu_data);

        // ==========================================
        // 2. NHẬN LỆNH ĐIỀU KHIỂN UDP (Rất nhanh, vài us)
        // ==========================================
        int packetSize = udp.parsePacket();
        if (packetSize) {
            char packetBuffer[32];
            int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
            if (len > 0) packetBuffer[len] = '\0';
            
            remoteIP = udp.remoteIP();
            remotePort = udp.remotePort();
            clientConnected = true;

            if (packetBuffer[0] == 'T') {
                int new_throttle = atoi(&packetBuffer[1]);
                if (new_throttle >= 1000 && new_throttle <= 2000) {
                    current_throttle = new_throttle;
                    last_packet_time_ = millis(); 
                }
            }
        }

        // Failsafe: Mất kết nối > 1s -> Tắt motor
        if (millis() - last_packet_time_ > 1000) {
            current_throttle = 1000; 
        }

        // ==========================================
        // 3. PID CONTROLLER SẼ NẰM Ở ĐÂY SAU NÀY
        // ==========================================
        // pid_calculate(setpoint, imu_data); 
        // motor_mix(current_throttle, pid_output);
        
        // Tạm thời xuất trực tiếp ga ra 4 motor để test rung động
        motor_set_pulse(current_throttle, current_throttle, current_throttle, current_throttle);

        // ==========================================
        // 4. TELEMETRY (Gửi về PC ở tần số thấp hơn: 20Hz = 50ms)
        // ==========================================
        // Không gửi UDP ở 250Hz vì sẽ làm tràn băng thông WiFi gây nghẽn mạng
        if (clientConnected && (millis() - last_telemetry_time >= 50)) {
            last_telemetry_time = millis();
            
            char replyBuffer[80];
            snprintf(replyBuffer, sizeof(replyBuffer), "D,%d,%.2f,%.2f,%.3f",
         current_throttle, imu_data.pitch, imu_data.roll, imu_data.acc_mag);
                     
            udp.beginPacket(remoteIP, remotePort);
            udp.print(replyBuffer);
            udp.endPacket();
        }
    }
    
    // Nếu chưa đủ 4000us, ESP32 sẽ rảnh rỗi (idle) làm việc khác như duy trì WiFi
}

#endif