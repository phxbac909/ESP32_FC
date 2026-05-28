#ifndef COMM_INTERFACE_H
#define COMM_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <data_struct.h>

// --- Khai báo các hàm Core (3 hàm chính) ---

/**
 * @brief Khởi tạo phần cứng và giao thức truyền thông.
 * Hậu tố _Comm để tránh trùng lặp với các module khác.
 */
void comm_init(void);

/**
 * @brief Thiết lập và kiểm tra trạng thái kết nối với tay cầm.
 * @return true nếu đã kết nối (handshake thành công), false nếu mất tín hiệu.
 */
bool connect_Comm(void);

/**
 * @brief Gửi dữ liệu log/telemetry từ Quad về tay điều khiển.
 * @param log_data Con trỏ tới struct chứa dữ liệu cần gửi.
 */
bool comm_send(const uint8_t *data_payload, size_t len);

#endif