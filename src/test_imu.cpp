// #include <Arduino.h>
// #include <Wire.h>
// #include <math.h>

// // ==========================================
// // CẤU HÌNH HỆ THỐNG
// // ==========================================
// #define MPU_ADDR 0x68
// #define M_PI_F 3.1415926535f

// // Thang đo MPU6050
// #define GYRO_SCALE 16.4f    // Cho dải +/- 2000 deg/s
// #define ACCEL_SCALE 2048.0f // Cho dải +/- 16G

// // Thông số vòng lặp
// #define SAMPLE_FREQ 250.0f      // Tần số vòng lặp mong muốn (250Hz)
// #define LOOP_TIME_US 4000       // 4000 micro-giây = 4ms = 250Hz
// #define GYRO_CUTOFF_FREQ 80.0f  // Tần số cắt Gyro
// #define ACCEL_CUTOFF_FREQ 30.0f // Tần số cắt Accel

// // ==========================================
// // CẤU TRÚC VÀ BIẾN TOÀN CỤC
// // ==========================================
// unsigned long last_loop_time = 0;
// unsigned long last_print_time = 0;

// float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;
// float tare_offset_roll = 0.0f, tare_offset_pitch = 0.0f;

// // Biến cho Mahony
// float twoKp = (2.0f * 0.4f);
// float twoKi = (2.0f * 0.001f);
// float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
// float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

// // Biến lưu kết quả cuối cùng
// float roll_filtered = 0.0f, pitch_filtered = 0.0f, yaw_filtered = 0.0f;

// // Cấu trúc Biquad Filter
// typedef struct {
//     float a1, a2, b0, b1, b2;
//     float delay_element_1, delay_element_2;
// } lpf2pData;

// lpf2pData gyroFilterX, gyroFilterY, gyroFilterZ;
// lpf2pData accelFilterX, accelFilterY, accelFilterZ;

// // ==========================================
// // HÀM HỖ TRỢ
// // ==========================================
// void i2c_write_reg(uint8_t reg, uint8_t data) {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(reg);
//     Wire.write(data); // Đã có hàm write data :D
//     Wire.endTransmission();
// }

// float invSqrt(float x) {
//     float halfx = 0.5f * x;
//     float y = x;
//     long i = *(long*)&y;
//     i = 0x5f3759df - (i >> 1);
//     y = *(float*)&i;
//     y = y * (1.5f - (halfx * y * y));
//     return y;
// }

// void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq) {
//     float fr = sample_freq / cutoff_freq;
//     float ohm = tanf(M_PI_F / fr);
//     float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
//     lpfData->b0 = ohm * ohm / c;
//     lpfData->b1 = 2.0f * lpfData->b0;
//     lpfData->b2 = lpfData->b0;
//     lpfData->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
//     lpfData->a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
//     lpfData->delay_element_1 = 0.0f;
//     lpfData->delay_element_2 = 0.0f;
// }

// float lpf2pApply(lpf2pData* lpfData, float sample) {
//     float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
//     if (!isfinite(delay_element_0)) delay_element_0 = sample; 
//     float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;
//     lpfData->delay_element_2 = lpfData->delay_element_1;
//     lpfData->delay_element_1 = delay_element_0;
//     return output;
// }

// // ==========================================
// // SETUP & LOOP
// // ==========================================
// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("\n--- KHỞI TẠO MPU6050 & LỌC MAHONY ---");

//     Wire.begin(); 
//     Wire.setClock(400000);

//     // 1. Reset & Clock
//     i2c_write_reg(0x6B, 0x80); delay(100); 
//     i2c_write_reg(0x6B, 0x01); delay(100); 

//     // 2. Cấu hình thang đo & Lọc phần cứng
//     i2c_write_reg(0x1B, 0x18); // GYRO: +/- 2000 deg/s 
//     i2c_write_reg(0x1C, 0x18); // ACCEL: +/- 16G 
//     i2c_write_reg(0x1A, 0x03); // DLPF: Cắt ở 42Hz 
//     i2c_write_reg(0x19, 0x00); // Tốc độ lấy mẫu nội bộ 1000Hz
//     delay(100);

//     // 3. Khởi tạo Biquad
//     lpf2pInit(&gyroFilterX, SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
//     lpf2pInit(&gyroFilterY, SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
//     lpf2pInit(&gyroFilterZ, SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
    
//     lpf2pInit(&accelFilterX, SAMPLE_FREQ, ACCEL_CUTOFF_FREQ);
//     lpf2pInit(&accelFilterY, SAMPLE_FREQ, ACCEL_CUTOFF_FREQ);
//     lpf2pInit(&accelFilterZ, SAMPLE_FREQ, ACCEL_CUTOFF_FREQ);

//     // 4. Calibrate (Giữ mạch đứng im tuyệt đối)
//     Serial.println("Đang Calibrate Gyro... (Vui lòng không chạm vào mạch)");
//     float gx_s = 0, gy_s = 0, gz_s = 0;
//     for (int i = 0; i < 2000; i++) {
//         Wire.beginTransmission(MPU_ADDR);
//         Wire.write(0x43);
//         Wire.endTransmission(false);
//         Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6);
//         gx_s += (int16_t)(Wire.read() << 8 | Wire.read());
//         gy_s += (int16_t)(Wire.read() << 8 | Wire.read());
//         gz_s += (int16_t)(Wire.read() << 8 | Wire.read());
//         delayMicroseconds(1000); // Đợi 1ms giữa mỗi lần đọc
//     }
//     gyro_off_x = (gx_s / 2000.0f) / GYRO_SCALE;
//     gyro_off_y = (gy_s / 2000.0f) / GYRO_SCALE;
//     gyro_off_z = (gz_s / 2000.0f) / GYRO_SCALE;

//     Serial.println("Calibrate OK! Bắt đầu chạy vòng lặp 250Hz...");
//     last_loop_time = micros();
// }

// void loop() {
//     unsigned long current_micros = micros();

//     // KIỂM SOÁT VÒNG LẶP CHÍNH XÁC 250Hz (4000 micro giây)
//     if (current_micros - last_loop_time >= LOOP_TIME_US) {
//         last_loop_time = current_micros; 
//         float dt = 1.0f / SAMPLE_FREQ; // dt = 0.004

//         // ==========================================
//         // 1. ĐỌC DỮ LIỆU I2C
//         // ==========================================
//         Wire.beginTransmission(MPU_ADDR);
//         Wire.write(0x3B);
//         Wire.endTransmission(false);
//         if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14) == 14) {
//             int16_t ax_raw = Wire.read() << 8 | Wire.read();
//             int16_t ay_raw = Wire.read() << 8 | Wire.read();
//             int16_t az_raw = Wire.read() << 8 | Wire.read();
//             Wire.read(); Wire.read(); // Skip Temp
//             int16_t gx_raw = Wire.read() << 8 | Wire.read();
//             int16_t gy_raw = Wire.read() << 8 | Wire.read();
//             int16_t gz_raw = Wire.read() << 8 | Wire.read();

//             // ==========================================
//             // 2. CHUYỂN ĐỔI & LỌC BIQUAD
//             // ==========================================
//             float ax = (float)ax_raw / ACCEL_SCALE;
//             float ay = (float)ay_raw / ACCEL_SCALE;
//             float az = (float)az_raw / ACCEL_SCALE;

//             float gx = ((float)gx_raw / GYRO_SCALE) - gyro_off_x;
//             float gy = ((float)gy_raw / GYRO_SCALE) - gyro_off_y;
//             float gz = ((float)gz_raw / GYRO_SCALE) - gyro_off_z;

//             // Lọc Gyro (Cho D-term của PID)
//             float gx_f = lpf2pApply(&gyroFilterX, gx);
//             float gy_f = lpf2pApply(&gyroFilterY, gy);
//             float gz_f = lpf2pApply(&gyroFilterZ, gz);

//             // Lọc Accel (Cho Mahony tính góc)
//             float ax_f = lpf2pApply(&accelFilterX, ax);
//             float ay_f = lpf2pApply(&accelFilterY, ay);
//             float az_f = lpf2pApply(&accelFilterZ, az);

//             // ==========================================
//             // 3. THUẬT TOÁN MAHONY
//             // ==========================================
//             float gx_rad = gx_f * M_PI_F / 180.0f;
//             float gy_rad = gy_f * M_PI_F / 180.0f;
//             float gz_rad = gz_f * M_PI_F / 180.0f;

//             if(!((ax_f == 0.0f) && (ay_f == 0.0f) && (az_f == 0.0f))) {
//                 float recipNorm = invSqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
//                 ax_f *= recipNorm; ay_f *= recipNorm; az_f *= recipNorm;

//                 float halfvx = qx * qz - qw * qy;
//                 float halfvy = qw * qx + qy * qz;
//                 float halfvz = qw * qw - 0.5f + qz * qz;

//                 float halfex = (ay_f * halfvz - az_f * halfvy);
//                 float halfey = (az_f * halfvx - ax_f * halfvz);
//                 float halfez = (ax_f * halfvy - ay_f * halfvx);

//                 if(twoKi > 0.0f) {
//                     integralFBx += twoKi * halfex * dt;
//                     integralFBy += twoKi * halfey * dt;
//                     integralFBz += twoKi * halfez * dt;
//                     gx_rad += integralFBx;
//                     gy_rad += integralFBy;
//                     gz_rad += integralFBz;
//                 }

//                 gx_rad += twoKp * halfex;
//                 gy_rad += twoKp * halfey;
//                 gz_rad += twoKp * halfez;
//             }

//             gx_rad *= (0.5f * dt);
//             gy_rad *= (0.5f * dt);
//             gz_rad *= (0.5f * dt);
            
//             float qa = qw, qb = qx, qc = qy;
//             qw += (-qb * gx_rad - qc * gy_rad - qz * gz_rad);
//             qx += (qa * gx_rad + qc * gz_rad - qz * gy_rad);
//             qy += (qa * gy_rad - qb * gz_rad + qz * gx_rad);
//             qz += (qa * gz_rad + qb * gy_rad - qc * gx_rad);

//             float norm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
//             qw *= norm; qx *= norm; qy *= norm; qz *= norm;

//             // ==========================================
//             // 4. QUY ĐỔI RA EULER ANGLE (PITCH, ROLL, YAW)
//             // ==========================================
//             float gravX = 2 * (qx * qz - qw * qy);
//             float gravY = 2 * (qw * qx + qy * qz);
//             float gravZ = qw * qw - qx * qx - qy * qy + qz * qz;

//             if (gravX > 1.0f) gravX = 1.0f;
//             if (gravX < -1.0f) gravX = -1.0f;

//             yaw_filtered   = atan2f(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz) * 180.0f / M_PI_F;
//             pitch_filtered = asinf(gravX) * 180.0f / M_PI_F;
//             roll_filtered  = atan2f(gravY, gravZ) * 180.0f / M_PI_F;
//         }

//         // ==========================================
//         // 5. IN RA SERIAL PLOTTER Ở 20Hz (50ms)
//         // ==========================================
//         if (millis() - last_print_time >= 50) {
//             last_print_time = millis();
            
//             // Format này dùng để mở Serial Plotter (Arduino IDE) vẽ đồ thị
//             Serial.print("Pitch:");
//             Serial.print(pitch_filtered);
//             Serial.print(",");
//             Serial.print("Roll:");
//             Serial.println(roll_filtered);
//         }
//     }
// }