#include "comm_core.h"
#include "comm_interface.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "config.h"
#include "comm_core.h"


#if defined(ESPNOW) && defined(MAIN)

// =======================================================
// BIẾN TOÀN CỤC CHO ESP-NOW
// =======================================================
esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = {0x14, 0x33, 0x5C, 0x2F, 0x1F, 0xB4};

bool data_received = false;
uint8_t received_data[250]; 
int received_data_len = 0;



// Khai báo prototype nội bộ
void on_data_receive(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

// =======================================================
// THỰC THI 3 HÀM TỪ COMMUNICATION.H
// =======================================================

// 1. Hàm khởi tạo
void comm_init(void) {
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
    
    DEBUG_PRINTLN("ESP-NOW Initialized Successfully");
    last_packet_time = millis();

    // Khởi tạo Task Failsafe chạy ngầm trên Core 0
    xTaskCreatePinnedToCore(
        failsafeTask, "Failsafe_Task", 2048, NULL, 1, NULL, 0
    );
}

// 2. Hàm gửi dữ liệu (Log/Telemetry)
bool comm_send(const uint8_t *data_payload, size_t len) {
    if (len > 250) {
        DEBUG_PRINTLN("Data too long (max 250 bytes)");
        return false;
    }
    
    esp_err_t result = esp_now_send(broadcastAddress, data_payload, len);
    return (result == ESP_OK);
}

// 3. Hàm kiểm tra kết nối
bool comm_connect(void) {
    // Nếu thời gian từ lần cuối nhận tín hiệu < 1000ms -> Đang kết nối tốt
    return (millis() - last_packet_time < 1000);
}


void on_data_receive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    last_packet_time = millis(); // Refresh mốc thời gian an toàn
    if (len <= 250) {
       proceedReceivedData((uint8_t*) incomingData, len);
    }
}


#endif