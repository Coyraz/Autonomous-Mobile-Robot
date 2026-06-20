#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"

// PID state for one wheel
typedef struct {
    float kp;
    float ki;
    float kd;
    float target_speed;   // mm/s, set by ROS command
    float actual_speed;   // mm/s, measured from encoder
    float error;
    float prev_error;
    float integral;
    float output;         // final PWM output
} PID_State_t;

typedef struct {
    int16_t left_speed;
    int16_t right_speed;
    uint32_t last_cmd_time;
} MotorCommand_t;

// These are read by PID loop to measure actual speed
// They must be updated by encoder reading code
extern volatile int32_t g_encoder_left_delta;   // ticks since last PID cycle
extern volatile int32_t g_encoder_right_delta;  // ticks since last PID cycle

extern PID_State_t g_pid_left;
extern PID_State_t g_pid_right;
extern MotorCommand_t g_motor_cmd;

void Motor_Init(void);
void Motor_SetTargetSpeed(int16_t left_speed_mmps, int16_t right_speed_mmps);
void Motor_PID_Update(void);    // Call this from timer interrupt at 100Hz
void Motor_Emergency_Stop(void);
uint8_t Motor_CheckTimeout(uint32_t current_time, uint32_t timeout_ms);

#endif
