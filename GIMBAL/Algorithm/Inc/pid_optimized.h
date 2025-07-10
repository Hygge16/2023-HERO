//
// Optimized PID Controller for High-Performance Control Loops
// Created for performance optimization
//

#ifndef PID_OPTIMIZED_H
#define PID_OPTIMIZED_H

#include <stdint.h>
#include <stdbool.h>

// Optimized PID structure for critical control loops
typedef struct {
    float kp, ki, kd;
    float max_out;
    float max_integral;
    float deadband;
    
    float error[3];        // Current, previous, previous-previous error
    float integral;
    float output;
    
    // Performance optimization: pre-computed values
    float ki_dt;           // Ki * dt pre-computed
    float kd_dt;           // Kd / dt pre-computed
} pid_optimized_t;

// Inline PID calculation for maximum performance
static inline float pid_calculate_optimized(pid_optimized_t *pid, float setpoint, float feedback) {
    // Shift error history
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = setpoint - feedback;
    
    // Skip calculation if within deadband
    if (pid->error[0] < pid->deadband && pid->error[0] > -pid->deadband) {
        return pid->output;
    }
    
    // Proportional term
    float p_out = pid->kp * pid->error[0];
    
    // Integral term with windup protection
    if (pid->ki_dt > 0.0f) {
        pid->integral += pid->error[0];
        
        // Integral windup protection
        if (pid->integral > pid->max_integral) {
            pid->integral = pid->max_integral;
        } else if (pid->integral < -pid->max_integral) {
            pid->integral = -pid->max_integral;
        }
    }
    float i_out = pid->ki_dt * pid->integral;
    
    // Derivative term
    float d_out = pid->kd_dt * (pid->error[0] - pid->error[1]);
    
    // Combine all terms
    pid->output = p_out + i_out + d_out;
    
    // Output limiting
    if (pid->output > pid->max_out) {
        pid->output = pid->max_out;
    } else if (pid->output < -pid->max_out) {
        pid->output = -pid->max_out;
    }
    
    return pid->output;
}

// Fast PID initialization
static inline void pid_init_optimized(pid_optimized_t *pid, float kp, float ki, float kd, 
                                     float max_out, float max_integral, float deadband, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_integral = max_integral;
    pid->deadband = deadband;
    
    // Pre-compute time-dependent terms
    pid->ki_dt = ki * dt;
    pid->kd_dt = kd / dt;
    
    // Clear state
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

// Fast PID reset
static inline void pid_reset_optimized(pid_optimized_t *pid) {
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

#endif // PID_OPTIMIZED_H