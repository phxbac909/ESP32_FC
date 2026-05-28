#include "comm_core.h"
#include "comm_interface.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "data_struct.h"
#include "config.h"
#include "comm_core.h"

// Sử dụng #define này trong config.h để kích hoạt file này
#if defined(UDP) && defined(MAIN)


 volatile unsigned long last_receive_time;
// =======================================================
// CẤU HÌNH WIFI & UDP
// =======================================================
const char* AP_SSID = "QUADCOPTER_WIFI"; // Tên WiFi do Quad phát ra
const char* AP_PASS = "12345678";        // Mật khẩu WiFi (ít nhất 8 ký tự)

const uint16_t LOCAL_PORT = 8888;
WiFiUDP udp;

// Biến lưu thông tin tay cầm (để biết đường gửi log trả về)
IPAddress remoteIP;
uint16_t remotePort;
bool isRemoteKnown = false; 

void udpReceiveTask(void *pvParameters) {
    uint8_t buffer[250]; 
    
    for(;;) {
        int packetSize = udp.parsePacket();
        
        if (packetSize > 0) { 
            last_receive_time = millis();
            
            if (!isRemoteKnown){
                remoteIP = udp.remoteIP();
                remotePort = udp.remotePort();
                isRemoteKnown = true;
                DEBUG_PRINTLN("✅ Đã nhận diện được App điều khiển!"); // Thêm log này
            }
            
            int len = udp.read(buffer, sizeof(buffer));
            
            if (len > 0 && len < 250) {
                buffer[len] = '\0'; // Ép kiểu chuỗi để in ra kiểm tra
                DEBUG_PRINTF("📥 Nhận: %s\n", (char*)buffer);
                
                // Nếu là gói Hello từ App, lập tức gửi trả lời để App thông luồng
                if (strncmp((char*)buffer, "Hello", 5) == 0) {
                    const char* reply = "Hi_App";
                    comm_send((const uint8_t*)reply, strlen(reply));
                    DEBUG_PRINTLN("Đã phản hồi Hi_App");
                } else {
                    // Nếu không phải Hello thì mới đưa vào xử lý logic bay
                    proceedReceivedData(buffer, len);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }
}


// 1. Hàm khởi tạo
void comm_init(void) {
    // 1. Cấu hình IP tĩnh (192.168.4.1)
    IPAddress local_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    // 2. Phát WiFi (Access Point)
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(AP_SSID, AP_PASS);
    
    DEBUG_PRINT("WiFi AP Started. IP: ");
    DEBUG_PRINTLN(WiFi.softAPIP());

    // 3. Khởi tạo UDP lắng nghe ở port 8888
    udp.begin(LOCAL_PORT);
    DEBUG_PRINTF("UDP Server Listening on Port: %d\n", LOCAL_PORT);

    last_receive_time = millis();

    // 4. Khởi tạo các Task chạy ngầm
    // Task Failsafe chạy trên Core 0
    xTaskCreatePinnedToCore(failsafeTask, "Failsafe_Task", 2048, NULL, 1, NULL, 0);
    
    // Task lắng nghe UDP chạy trên Core 1 (hoặc Core 0 tùy bạn phân bổ)
    xTaskCreatePinnedToCore(udpReceiveTask, "UDP_Rx_Task", 4096, NULL, 2, NULL, 1); 
}

// 2. Hàm gửi dữ liệu (Log/Telemetry)
bool comm_send(const uint8_t *data_payload, size_t len) {
    // Chỉ gửi khi đã biết IP và Port của tay điều khiển (đã nhận được ít nhất 1 gói tin)
    if (!isRemoteKnown) return false; 
    
    // Bắt đầu gói tin tới IP và Port của Remote
    udp.beginPacket(remoteIP, remotePort);
    udp.write(data_payload, len);
    
    // endPacket trả về 1 nếu gửi thành công
    return (udp.endPacket() == 1);
}

// 3. Hàm kiểm tra kết nối (Dùng cho vòng lặp chính để cảnh báo)
bool comm_connect(void) {
    return (millis() - last_receive_time < 1000);
}

// =======================================================
// XỬ LÝ NHẬN DỮ LIỆU BẰNG TASK NGẦM
// =======================================================


#endif