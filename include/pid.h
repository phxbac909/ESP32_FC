#ifndef PID_H
#define PID_H

class PID {
private:
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;

public:
    // Constructor mặc định
    PID();

    // Khởi tạo toàn bộ thông số
    void init(float p, float i, float d, float min, float max);

    // Cập nhật lại PID (dùng khi bạn muốn tune thông số trực tiếp lúc đang chạy)
    void setTunings(float p, float i, float d);

    // Đặt giới hạn đầu ra
    void setOutputLimits(float min, float max);

    // Reset bộ tích phân và sai số cũ (dùng khi ngã hoặc khởi động lại motor)
    void reset();

    // Tính toán đầu ra
    float compute(float setpoint, float measured, float dt);
};

#endif // PID_H