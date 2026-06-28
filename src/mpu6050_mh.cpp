#include "mpu6050.h" // Chỉnh lại include này theo tên file header của bạn
#include <Wire.h>
#include <math.h>
#include <config.h>

#ifdef MH

// ==========================================
// CẤU HÌNH HỆ THỐNG
// ==========================================
#define MPU_ADDR 0x68
#define M_PI_F 3.1415926535f
#define INTEGRAL_LIMIT 0.3f


// Thang đo MPU6050
#define GYRO_SCALE 16.4f    // Cho dải +/- 2000 deg/s
#define ACCEL_SCALE 2048.0f // Cho dải +/- 16G (CHỐNG NHIỄU MOTOR)

// Thông số vòng lặp
#define SAMPLE_FREQ 250.0f  // Tần số vòng lặp 
#define GYRO_CUTOFF_FREQ 80.0f  // Tần số cắt nhiễu Gyro
#define ACCEL_CUTOFF_FREQ 10.0f // Tần số cắt nhiễu Accel 

// ==========================================
// CẤU TRÚC VÀ BIẾN TOÀN CỤC
// ==========================================
static unsigned long last_time_micros = 0;
static float gyro_off_x = 0, gyro_off_y = 0, gyro_off_z = 0;
static float tare_offset_roll = 0.0f, tare_offset_pitch = 0.0f;

// --- BIẾN CHO MAHONY FILTER ---
static float twoKp = (2.0f * 0.1f);    // Hệ số P
static float twoKi = (2.0f * 0.005f);  // Hệ số I
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
static float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f; 

// --- BIẾN CHO BỘ LỌC BIQUAD ---
typedef struct {
    float a1, a2, b0, b1, b2;
    float delay_element_1, delay_element_2;
} lpf2pData;

static lpf2pData gyroFilterX, gyroFilterY, gyroFilterZ;
static lpf2pData accelFilterX, accelFilterY, accelFilterZ;

// ==========================================
// CÁC HÀM HỖ TRỢ NỘI BỘ 
// ==========================================
static void i2c_write_reg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data); // Đã fix lỗi thiếu dòng này
    Wire.endTransmission();
}

static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq) {
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(M_PI_F / fr);
    float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
    lpfData->b0 = ohm * ohm / c;
    lpfData->b1 = 2.0f * lpfData->b0;
    lpfData->b2 = lpfData->b0;
    lpfData->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    lpfData->a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
    lpfData->delay_element_1 = 0.0f;
    lpfData->delay_element_2 = 0.0f;
}

static float lpf2pApply(lpf2pData* lpfData, float sample) {
    float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
    if (!isfinite(delay_element_0)) delay_element_0 = sample; 
    float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;
    lpfData->delay_element_2 = lpfData->delay_element_1;
    lpfData->delay_element_1 = delay_element_0;
    return output;
}

// ==========================================
// CÁC HÀM CHÍNH (PUBLIC)
// ==========================================
void mpu6050_init() {
    Wire.begin(21, 22); 
    Wire.setClock(400000);

    i2c_write_reg(0x6B, 0x80); delay(50); 
    i2c_write_reg(0x6B, 0x01); delay(50); 

    i2c_write_reg(0x1B, 0x18); // GYRO: +/- 2000 deg/s 
    i2c_write_reg(0x1C, 0x18); // ACCEL: +/- 16G 
    i2c_write_reg(0x1A, 0x02); // DLPF: 98Hz  — viết trước
    i2c_write_reg(0x19, 0x03); // SMPLRT_DIV: 250Hz output
    delay(200);

    lpf2pInit(&gyroFilterX, SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
    lpf2pInit(&gyroFilterY, SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
    lpf2pInit(&gyroFilterZ, SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
    
    lpf2pInit(&accelFilterX, SAMPLE_FREQ, ACCEL_CUTOFF_FREQ);
    lpf2pInit(&accelFilterY, SAMPLE_FREQ, ACCEL_CUTOFF_FREQ);
    lpf2pInit(&accelFilterZ, SAMPLE_FREQ, ACCEL_CUTOFF_FREQ);

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
    
    gyro_off_x = (gx_s / 2000.0f) / GYRO_SCALE;
    gyro_off_y = (gy_s / 2000.0f) / GYRO_SCALE;
    gyro_off_z = (gz_s / 2000.0f) / GYRO_SCALE;

    last_time_micros = micros();
}

bool IMU_Update_And_Read(imu_data_t *out_data) {
    // ==========================================
    // 1. TÍNH dt VÀ ĐỌC I2C RAW DATA
    // ==========================================
    unsigned long current_time = micros();
    float dt = (current_time - last_time_micros) / 1000000.0f; 
    last_time_micros = current_time;

    if (dt <= 0.0f || dt > 0.05f) {
        dt = 1.0f / SAMPLE_FREQ; 
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14) != 14) return false;

    int16_t ax_raw = Wire.read() << 8 | Wire.read();
    int16_t ay_raw = Wire.read() << 8 | Wire.read();
    int16_t az_raw = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Skip Temp
    int16_t gx_raw = Wire.read() << 8 | Wire.read();
    int16_t gy_raw = Wire.read() << 8 | Wire.read();
    int16_t gz_raw = Wire.read() << 8 | Wire.read();

    // ==========================================
    // 2. CHUYỂN ĐỔI UNIT & LỌC BIQUAD
    // ==========================================
    float ax = (float)ax_raw / ACCEL_SCALE;
    float ay = (float)ay_raw / ACCEL_SCALE;
    float az = (float)az_raw / ACCEL_SCALE;

    float gx = ((float)gx_raw / GYRO_SCALE) - gyro_off_x;
    float gy = ((float)gy_raw / GYRO_SCALE) - gyro_off_y;
    float gz = ((float)gz_raw / GYRO_SCALE) - gyro_off_z;

    float gx_filtered = lpf2pApply(&gyroFilterX, gx);
    float gy_filtered = lpf2pApply(&gyroFilterY, gy);
    float gz_filtered = lpf2pApply(&gyroFilterZ, gz);

    float ax_filtered = lpf2pApply(&accelFilterX, ax);
    float ay_filtered = lpf2pApply(&accelFilterY, ay);
    float az_filtered = lpf2pApply(&accelFilterZ, az);

    // ==========================================
    // 3. BETAFLIGHT SMART MAHONY AHRS
    // ==========================================
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Chuyển Gyro sang Radian/s
    float gx_rad = gx_filtered * M_PI_F / 180.0f;
    float gy_rad = gy_filtered * M_PI_F / 180.0f;
    float gz_rad = gz_filtered * M_PI_F / 180.0f;

    // TÍNH ĐỘ LỚN CỦA GIA TỐC TỔNG (Vectơ G)
    float acc_mag = sqrtf(ax_filtered * ax_filtered + ay_filtered * ay_filtered + az_filtered * az_filtered);
    out_data->acc_mag = acc_mag;

    // BỘ CHỐT CHẶN RUNG ĐỘNG (ADAPTIVE GAIN)
    // Chỉ tin Accel khi nó nằm trong dải 0.75G đến 1.25G
    if(acc_mag > 0.2f && acc_mag < 1.8f) {
        
        // CÓ GIA TỐC HỢP LỆ: Chuẩn hóa vector gia tốc
        recipNorm = 1.0f / acc_mag;
        ax_filtered *= recipNorm; 
        ay_filtered *= recipNorm; 
        az_filtered *= recipNorm;

        // Ước lượng hướng trọng lực từ Quaternion hiện tại
        halfvx = qx * qz - qw * qy;
        halfvy = qw * qx + qy * qz;
        halfvz = qw * qw - 0.5f + qz * qz;

        // Tính sai số (Lỗi giữa trọng lực thực tế và ước lượng)
        halfex = (ay_filtered * halfvz - az_filtered * halfvy);
        halfey = (az_filtered * halfvx - ax_filtered * halfvz);
        halfez = (ax_filtered * halfvy - ay_filtered * halfvx);

        // Tích phân lỗi (Khử trôi Gyro)
        if(twoKi > 0.0f) {

            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;

            if(integralFBx >  INTEGRAL_LIMIT) integralFBx =  INTEGRAL_LIMIT;
            if(integralFBx < -INTEGRAL_LIMIT) integralFBx = -INTEGRAL_LIMIT;
            if(integralFBy >  INTEGRAL_LIMIT) integralFBy =  INTEGRAL_LIMIT;
            if(integralFBy < -INTEGRAL_LIMIT) integralFBy = -INTEGRAL_LIMIT;
            if(integralFBz >  INTEGRAL_LIMIT) integralFBz =  INTEGRAL_LIMIT;
            if(integralFBz < -INTEGRAL_LIMIT) integralFBz = -INTEGRAL_LIMIT;
            
            gx_rad += integralFBx;
            gy_rad += integralFBy;
            gz_rad += integralFBz;
        }

        // Cộng lỗi P vào tốc độ góc Gyro
        gx_rad += twoKp * halfex;
        gy_rad += twoKp * halfey;
        gz_rad += twoKp * halfez;
    } 
    // Nếu acc_mag vọt ra ngoài ngưỡng (rung động cực mạnh), KHÔNG làm gì cả!
    // Hệ thống sẽ bỏ qua toàn bộ phần bù nhiễu Accel, gx_rad/gy_rad chỉ chứa dữ liệu Gyro.
    
    // TÍCH PHÂN TỐC ĐỘ GÓC VÀO QUATERNION (Luôn luôn chạy)
    gx_rad *= (0.5f * dt);
    gy_rad *= (0.5f * dt);
    gz_rad *= (0.5f * dt);
    qa = qw; qb = qx; qc = qy;
    qw += (-qb * gx_rad - qc * gy_rad - qz * gz_rad);
    qx += (qa * gx_rad + qc * gz_rad - qz * gy_rad);
    qy += (qa * gy_rad - qb * gz_rad + qz * gx_rad);
    qz += (qa * gz_rad + qb * gy_rad - qc * gx_rad);

    // Chuẩn hóa lại Quaternion
    recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= recipNorm; qx *= recipNorm; qy *= recipNorm; qz *= recipNorm;

    // ==========================================
    // 4. CHUYỂN ĐỔI SANG EULER (PITCH, ROLL, YAW)
    // ==========================================
    float gravX = 2 * (qx * qz - qw * qy);
    float gravY = 2 * (qw * qx + qy * qz);
    float gravZ = qw * qw - qx * qx - qy * qy + qz * qz;

    if (gravX > 1.0f) gravX = 1.0f;
    if (gravX < -1.0f) gravX = -1.0f;

    if (out_data != nullptr) {
        out_data->gyro_roll  = gx_filtered;
        out_data->gyro_pitch = gy_filtered;
        out_data->gyro_yaw   = gz_filtered;

        out_data->yaw   = atan2f(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz) * 180.0f / M_PI_F;
        out_data->pitch = asinf(gravX) * 180.0f / M_PI_F - tare_offset_pitch;
        out_data->roll  = atan2f(gravY, gravZ) * 180.0f / M_PI_F - tare_offset_roll;
    }

    return true;
}

void mpu6050_tare() {
    float r_sum = 0, p_sum = 0;
    imu_data_t temp;
    for(int i=0; i<100; i++) {
        IMU_Update_And_Read(&temp);
        r_sum += temp.roll;
        p_sum += temp.pitch;
        delay(4); 
    }
    tare_offset_roll = r_sum / 100.0f;
    tare_offset_pitch = p_sum / 100.0f;
}

#endif