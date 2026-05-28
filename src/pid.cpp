#include "pid.h"

// Constructor khởi tạo mọi thứ bằng 0
PID::PID() : kp(0), ki(0), kd(0), integral(0), prev_error(0), out_min(0), out_max(0) {}

void PID::init(float p, float i, float d, float min, float max) {
    kp = p;
    ki = i;
    kd = d;
    out_min = min;
    out_max = max;
    integral = 0.0f;
    prev_error = 0.0f;
}

void PID::setTunings(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void PID::setOutputLimits(float min, float max) {
    out_min = min;
    out_max = max;
}

void PID::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
}

float PID::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // 1. Sai số
    float error = setpoint - measured;
    
    // 2. Proportional (P)
    float p_out = kp * error;
    
    // 3. Integral (I) với Anti-Windup
    integral += error * dt;
    float i_out = ki * integral;
    
    if (ki > 0.0f) {
        if (i_out > out_max) {
            integral = out_max / ki;
            i_out = out_max;
        } else if (i_out < out_min) {
            integral = out_min / ki;
            i_out = out_min;
        }
    }
    
    // 4. Derivative (D)
    float derivative = (error - prev_error) / dt;
    float d_out = kd * derivative;
    
    prev_error = error; // Lưu sai số
    
    // 5. Tổng hợp
    float output = p_out + i_out + d_out;
    
    // 6. Constrain
    if (output > out_max) return out_max;
    if (output < out_min) return out_min;
    
    return output;
}