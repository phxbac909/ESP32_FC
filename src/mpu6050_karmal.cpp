#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "mpu6050.h"
#include "config.h"

#ifdef KARMAL
// --- CẤU HÌNH TWEAKING ---
#define MPU_ADDR 0x68
#define RAD_TO_DEG 57.29577951f

// 1. Deadzone (Vùng chết) cho Gyro (Đã chuyển sang ĐỘ/GIÂY):
// Nếu Gyro nhỏ hơn giá trị này (deg/s), coi như bằng 0 để chống trôi.
#define GYRO_DEADZONE 0.1f 

// 2. Lọc Gyro
#define GYRO_SOFT_LPF_HZ 30.0f 

// ==========================================
// CLASS KALMAN FILTER (Hệ quy chiếu: Độ - Degree)
// ==========================================
class KalmanFilter {
private:
    float Q_angle;   
    float Q_bias;    
    float R_measure; 

    float angle; 
    float bias;  
    float rate;  
    float P[2][2]; 

public:
    KalmanFilter() {
        /* CẤU HÌNH TỐI ĐA ĐỘ ỔN ĐỊNH (Đã quy đổi cho đơn vị ĐỘ)
         * - R_measure rất lớn (10.0 thay vì 0.03 mặc định): Lọc nhiễu rung cơ học cực mạnh.
         */
        Q_angle = 0.001f; 
        Q_bias = 0.003f;
        R_measure = 30.0f; // Chống rung lắc tối đa

        angle = 0.0f; bias = 0.0f; rate = 0.0f;
        P[0][0] = 0.0f; P[0][1] = 0.0f; P[1][0] = 0.0f; P[1][1] = 0.0f;
    }

    float getAngle(float newAngle, float newRate, float dt) {
        rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        float S = P[0][0] + R_measure; 
        float K[2]; 
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = newAngle - angle; 

        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    };

    void setAngle(float newAngle) { angle = newAngle; };
};

static KalmanFilter kalmanRoll;
static KalmanFilter kalmanPitch;
// ==========================================

// --- BIẾN TOÀN CỤC ---
// Đổi toàn bộ sang đuôi _deg để tránh nhầm lẫn
static float roll_deg = 0, pitch_deg = 0, yaw_deg = 0; 
static float last_gyro_x = 0, last_gyro_y = 0, last_gyro_z = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;

// Offset cân bằng (Tư thế của MPU so với Quad)
static float tare_offset_roll = 0.0f, tare_offset_pitch = 0.0f; 

static unsigned long last_time_micros = 0;
static bool is_initialized = false;

// --- HÀM HỖ TRỢ ---
void i2c_write_reg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

// --- KHỞI TẠO & HIỆU CHUẨN ---
void mpu6050_init() {
    Wire.begin(21, 22);
    Wire.setClock(400000);

    // 1. Reset & Config
    i2c_write_reg(0x6B, 0x80); delay(50);
    i2c_write_reg(0x6B, 0x00); delay(50);
    i2c_write_reg(0x1A, 0x05); // DLPF ~42Hz
    i2c_write_reg(0x1B, 0x18); // Gyro +/- 2000 dps (16.4 LSB/deg/s)
    i2c_write_reg(0x1C, 0x10); // Accel +/- 8g

    delay(500); // Đợi ổn định nhiệt

    // 2. HIỆU CHUẨN TRÔI GYRO BAN ĐẦU (Rất quan trọng, không được bỏ)
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    for (int i = 0; i < 1000; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true);
        
        gx_sum += (int16_t)(Wire.read() << 8 | Wire.read());
        gy_sum += (int16_t)(Wire.read() << 8 | Wire.read());
        gz_sum += (int16_t)(Wire.read() << 8 | Wire.read());
        delay(1);
    }
    gyro_off_x = gx_sum / 1000.0f;
    gyro_off_y = gy_sum / 1000.0f;
    gyro_off_z = gz_sum / 1000.0f;

    // 3. MỒI GÓC BAN ĐẦU CHO KALMAN (Bằng Độ)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true);
    int16_t ax_init = Wire.read() << 8 | Wire.read();
    int16_t ay_init = Wire.read() << 8 | Wire.read();
    int16_t az_init = Wire.read() << 8 | Wire.read();
    
    float acc_roll_init = atan2((float)ay_init, (float)az_init) * RAD_TO_DEG;
    float acc_pitch_init = atan2((float)-ax_init, sqrt((float)ay_init*ay_init + (float)az_init*az_init)) * RAD_TO_DEG;
    kalmanRoll.setAngle(acc_roll_init);
    kalmanPitch.setAngle(acc_pitch_init);

    // 4. HIỆU CHUẨN TƯ THẾ (TARE)
    // Lấy trung bình 2000 mẫu góc đã qua bộ lọc Kalman để làm gốc 0 cho Quadcopter
    last_time_micros = micros();
    
    // float sum_roll = 0.0f;
    // float sum_pitch = 0.0f;

    // for(int i = 0; i < 2000; i++) {
    //     imu_data_t temp;
    //     delay(3); 
    //     IMU_Update_And_Read(&temp); // Gọi update để filter chạy và cập nhật roll_deg, pitch_deg
        
    //     sum_roll += roll_deg;
    //     sum_pitch += pitch_deg;
    // }

    // Chốt góc lệch vật lý của cảm biến so với quadcopter
    tare_offset_roll = -0.8f;
    tare_offset_pitch = 0.55f;
    
    // Reset Yaw
    yaw_deg = 0; 
    
    is_initialized = true;
}

// --- HÀM UPDATE ---
bool IMU_Update_And_Read(imu_data_t *out_data) {
    // 1. Tính DT
    unsigned long now = micros();
    float dt = (now - last_time_micros) / 1000000.0f;
    last_time_micros = now;
    if (dt > 0.04f || dt <= 0.0f) dt = 0.004f;

    // 2. Đọc Raw
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true) != 14) return false;

    int16_t ax_raw = Wire.read() << 8 | Wire.read();
    int16_t ay_raw = Wire.read() << 8 | Wire.read();
    int16_t az_raw = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t gx_raw = Wire.read() << 8 | Wire.read();
    int16_t gy_raw = Wire.read() << 8 | Wire.read();
    int16_t gz_raw = Wire.read() << 8 | Wire.read();

    // 3. Xử lý Gyro (Tính trực tiếp ra Độ/Giây - Deg/s)
    float gyro_x = ((float)gx_raw - gyro_off_x) / 16.4f;
    float gyro_y = ((float)gy_raw - gyro_off_y) / 16.4f;
    float gyro_z = ((float)gz_raw - gyro_off_z) / 16.4f;

    // Deadzone
    if (fabs(gyro_x) < GYRO_DEADZONE) gyro_x = 0;
    if (fabs(gyro_y) < GYRO_DEADZONE) gyro_y = 0;
    if (fabs(gyro_z) < GYRO_DEADZONE) gyro_z = 0;

    // LPF cho Gyro Rate
    float rc = 1.0f / (2.0f * PI * GYRO_SOFT_LPF_HZ);
    float alpha_lpf = dt / (rc + dt);
    
    last_gyro_x += alpha_lpf * (gyro_x - last_gyro_x);
    last_gyro_y += alpha_lpf * (gyro_y - last_gyro_y);
    last_gyro_z += alpha_lpf * (gyro_z - last_gyro_z);

    // 4. Tính toán góc Accel (Trực tiếp ra Độ)
    float acc_roll = atan2((float)ay_raw, (float)az_raw) * RAD_TO_DEG;
    float acc_pitch = atan2((float)-ax_raw, sqrt((float)ay_raw*ay_raw + (float)az_raw*az_raw)) * RAD_TO_DEG;

    // --- MỚI: BIẾN ĐỔI ĐỘNG HỌC (KINEMATIC COUPLING) ---
    // Cần đổi góc hiện tại (của chu kỳ trước) ra Radian để tính sin()
    // Lưu ý: Mặc định hàm sin() trong C/C++ yêu cầu đầu vào là Radian
    float phi = roll_deg * 0.01745329f;   // roll_deg * (PI / 180)
    float theta = pitch_deg * 0.01745329f; // pitch_deg * (PI / 180)

    // Tính toán lại vận tốc góc thực tế (Euler Rates) bằng xấp xỉ góc nhỏ
    // Đơn vị đầu ra vẫn là ĐỘ/GIÂY (vì last_gyro đang là Độ/Giây)
    float euler_rate_roll  = last_gyro_x + last_gyro_z * sin(theta);
    float euler_rate_pitch = last_gyro_y - last_gyro_z * sin(phi);

    // 5. Cập nhật Kalman Filter (Sử dụng Euler Rates thay vì Body Rates)
    roll_deg  = kalmanRoll.getAngle(acc_roll, euler_rate_roll, dt);
    pitch_deg = kalmanPitch.getAngle(acc_pitch, euler_rate_pitch, dt);
    
    // Yaw vẫn tích phân theo Body Rate Z (để giữ tốc độ điều khiển phản hồi nhanh)
    yaw_deg  += last_gyro_z * dt;

    // 6. XUẤT DỮ LIỆU CUỐI CÙNG
    if (out_data != nullptr) {        
        // --- RATE (Vận tốc góc) : Đã là Deg/s ---
        out_data->gyro_roll  = last_gyro_x;
        out_data->gyro_pitch = last_gyro_y;
        out_data->gyro_yaw   = last_gyro_z;

        // --- ANGLE (Góc nghiêng) : Trừ đi Offset tư thế ---
        out_data->roll  = roll_deg - tare_offset_roll;
        out_data->pitch = pitch_deg - tare_offset_pitch;
        out_data->yaw   = yaw_deg;
    }
    
    return true;
}

#endif