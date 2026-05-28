#ifndef COMM_CORE_H
#define COMM_CORE_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include "data_struct.h"

// --- BIẾN TOÀN CỤC DÙNG CHUNG ---
// Dùng extern để các file khác (ESP-NOW, UDP) có thể truy cập và cập nhật


// --- HÀM XỬ LÝ DÙNG CHUNG ---
/**
 * @brief Phân tích mảng byte nhận được và đưa vào hệ thống điều khiển
 */
void proceedReceivedData(uint8_t *incomingData, size_t length);

/**
 * @brief Task chạy ngầm trên RTOS để giám sát mất tín hiệu
 */
void failsafeTask(void *pvParameters);

#endif // COMM_COMMON_H
