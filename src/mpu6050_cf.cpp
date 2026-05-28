#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <mpu6050.h>
#include "config.h"

#ifdef CF

// --- CẤU HÌNH HỆ THỐNG ---
#define MPU_ADDR 0x68
#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f

// --- CẤU HÌNH TWEAKING ---
// Hằng số thời gian của Complementary Filter (Giây).
#define COMP_FILTER_TAU 10.0f 

// LPF cho Gyro Input
#define GYRO_INPUT_LPF_OLD 0.7f
#define GYRO_INPUT_LPF_NEW 0.3f

// Scale factor cho Gyro ở dải +/- 500 deg/s
#define GYRO_SCALE 65.5f


// --- BIẾN TOÀN CỤC ---
static float roll_deg = 0, pitch_deg = 0, yaw_deg = 0; 
static float gyro_roll_input = 0, gyro_pitch_input = 0, gyro_yaw_input = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;

static float tare_offset_roll = 0.0f;
static float tare_offset_pitch = 0.0f; 

static unsigned long last_time_micros = 0;

// --- HÀM GHI I2C ---
void i2c_write_reg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

// --- KHỞI TẠO MPU6050 ---
void mpu6050_init() {
    Wire.begin(21, 22); 
    Wire.setClock(400000);

    i2c_write_reg(0x6B, 0x80); delay(50); 
    i2c_write_reg(0x6B, 0x01); delay(50); 
    i2c_write_reg(0x1A, 0x03);            
    i2c_write_reg(0x1B, 0x08);            
    i2c_write_reg(0x1C, 0x10);            
    delay(200);

    float gx_s = 0, gy_s = 0, gz_s = 0;
    for (int i = 0; i < 2000; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6);
        gx_s += (int16_t)(Wire.read() << 8 | Wire.read());
        gy_s += (int16_t)(Wire.read() << 8 | Wire.read());
        gz_s += (int16_t)(Wire.read() << 8 | Wire.read());
        delayMicroseconds(1000);
    }
    gyro_off_x = gx_s / 2000.0f;
    gyro_off_y = gy_s / 2000.0f;
    gyro_off_z = gz_s / 2000.0f;


    DEBUG_PRINTLN("MPU6050 init !");
    last_time_micros = micros();
}

// --- CẬP NHẬT DỮ LIỆU ---
bool IMU_Update_And_Read(imu_data_t *out_data) {
    // 1. TÍNH TOÁN dt ĐỘNG BẰNG micros()
    unsigned long current_time = micros();
    float dt = (current_time - last_time_micros) / 1000000.0f; // Chuyển sang giây
    last_time_micros = current_time;

    // Bảo vệ dt (Nếu loop bị kẹt quá lâu hoặc lỗi tràn micros)
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.004f; // Mặc định về 1ms nếu lỗi
    }
;
    // --- TÍNH TOÁN CÁC HỆ SỐ PHỤ THUỘC dt ---
    // a. Hằng số tích phân (dt / Scale)
    float dt_integral = dt / GYRO_SCALE;
    
    // b. Hằng số bù trừ Yaw (dt / Scale * chuyển sang radian)
    float dt_yaw_transfer = dt_integral * DEG_TO_RAD;
    
    // c. Alpha động cho Complementary Filter
    float alpha = COMP_FILTER_TAU / (COMP_FILTER_TAU + dt);

    // 2. Đọc dữ liệu Raw
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14) != 14) return false;

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    // 3. Xử lý Gyro Angle (Trừ offset)
    float pitch_gyro_raw = (float)gx - gyro_off_x;
    float roll_gyro_raw  = (float)gy - gyro_off_y;
    float yaw_gyro_raw   = (float)gz - gyro_off_z;

    // 4. LPF cho Gyro Input (Đưa vào PID)
    gyro_pitch_input = (gyro_pitch_input * GYRO_INPUT_LPF_OLD) + ((pitch_gyro_raw / GYRO_SCALE) * GYRO_INPUT_LPF_NEW);
    gyro_roll_input  = (gyro_roll_input  * GYRO_INPUT_LPF_OLD) + ((roll_gyro_raw  / GYRO_SCALE) * GYRO_INPUT_LPF_NEW);
    gyro_yaw_input   = (gyro_yaw_input   * GYRO_INPUT_LPF_OLD) + ((yaw_gyro_raw   / GYRO_SCALE) * GYRO_INPUT_LPF_NEW);

    // 5. Tích phân góc bằng dt động
    roll_deg  += roll_gyro_raw  * dt_integral;
    pitch_deg += pitch_gyro_raw * dt_integral;

    // 6. Quy đổi roll, pitch khi góc yaw quay (Kinematic Coupling)
    float yaw_rad_delta = yaw_gyro_raw * dt_yaw_transfer;
    float sin_yaw = sin(yaw_rad_delta);
    
    float roll_temp = roll_deg;
    roll_deg  += pitch_deg * sin_yaw;
    pitch_deg -= roll_temp * sin_yaw;

    // 7. Tính góc từ Accel 
    float acc_total_vector = sqrt((float)ax*ax + (float)ay*ay + (float)az*az);
    float acc_pitch = asin((float)ay / acc_total_vector) * RAD_TO_DEG;
    float acc_roll  = asin((float)ax / acc_total_vector) * RAD_TO_DEG * -1.0f;

    // 8. Bộ lọc bù (Complementary Filter) với alpha động
    roll_deg  = roll_deg  * alpha + acc_roll  * (1.0f - alpha);
    pitch_deg = pitch_deg * alpha + acc_pitch * (1.0f - alpha);
    
    yaw_deg += yaw_gyro_raw * dt_integral; // Tích phân Yaw thuần túy

    // 9. Xuất dữ liệu cuối cùng
    if (out_data != nullptr) {
        out_data->gyro_roll  = gyro_roll_input;
        out_data->gyro_pitch = gyro_pitch_input;
        out_data->gyro_yaw   = gyro_yaw_input;

        out_data->roll  = roll_deg - tare_offset_roll;
        out_data->pitch = pitch_deg - tare_offset_pitch;
        out_data->yaw   = yaw_deg;
    }

    return true;
}

#endif