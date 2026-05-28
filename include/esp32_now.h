#ifndef ESP32_NOW_H
#define ESP32_NOW_H

#include <Arduino.h>

// Khai báo các hàm công khai
void esp32_now_init();
bool esp32_now_send(const uint8_t *data, int len);
bool esp32_now_get_received_data(uint8_t *buffer, int *len);

#endif