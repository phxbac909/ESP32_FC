#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "config.h"

#ifdef TEST_ESPNOW
// --- MACROS IN ẤN ---
#ifndef DEBUG_PRINT
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#endif

// --- BIẾN TOÀN CỤC ---
esp_now_peer_info_t peerInfo;
// Nhập địa chỉ MAC của mạch TX (tay điều khiển) vào đây
uint8_t broadcastAddress[] = {0x14, 0x33, 0x5C, 0x2F, 0x1F, 0xB4}; 

// --- BIẾN PHỤC VỤ TEST GÓI TIN ---
volatile uint32_t received_count = 0;

// Khai báo prototype với cấu trúc cũ (Core v2.x)
void on_data_receive(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

// --- HÀM KHỞI TẠO ---
void esp32_now_init() {
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        DEBUG_PRINTLN("Error initializing ESP-NOW");
        return;
    }
    
    esp_now_register_recv_cb(on_data_receive);
    
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        DEBUG_PRINTLN("Failed to add peer");
        return;
    }
    DEBUG_PRINTLN("\n=== ESP-NOW RX READY FOR PING & PACKET LOSS TEST ===");
    DEBUG_PRINTLN("Đang chờ TX gửi gói tin...");
}

// --- CALLBACK NHẬN DỮ LIỆU ---
void on_data_receive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (len > 0) {
        // 1. Dội ngược (echo) ngay lập tức về TX để TX tính Ping
        esp_now_send(broadcastAddress, incomingData, len);
        
        // 2. Tăng biến đếm số gói tin nhận được
        received_count++;

        // 3. Báo cáo khi chạm mốc 1000 gói
        if (received_count >= 1000) {
            DEBUG_PRINTLN("\n------------------------------------------------");
            DEBUG_PRINTF("[TEST HOÀN TẤT] Đã nhận đủ: %d gói\n", received_count);
            DEBUG_PRINTLN("------------------------------------------------\n");
            
            // Reset đếm để tự động chạy lại bài test tiếp theo
            received_count = 0;
        }
    }
}

// ==========================================
// CHƯƠNG TRÌNH CHÍNH
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Khởi tạo ESP-NOW
    esp32_now_init();
}

void loop() {
    // Để trống hoàn toàn để CPU ưu tiên tối đa cho ngắt nhận ESP-NOW
}

#endif