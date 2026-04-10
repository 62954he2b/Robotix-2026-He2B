#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "cli_input.h"
#include "stepper_motors.h"

#define revolution 360

extern const float minimum_frequency;
extern const float maximum_frequency;

extern bool forward_flag;
extern bool turn_flag;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float previousError;
    unsigned long previousTime;
} PIDController;

enum MotorState {
    IDLE,
    RUNNING,
    DONE
};

enum ControlState {
    MANUAL,
    AUTOMATIC,
    EMERGENCY_STOP,
    NONE
};

extern volatile MotorState left_motor_state;
extern volatile MotorState right_motor_state;

extern volatile ControlState motors_control_state;

void right_motor_position_control_task(void *parameter);
void left_motor_position_control_task(void *parameter);
void right_motor_velocity_control_task(void *parameter);
void left_motor_velocity_control_task(void *parameter);
float PID_control(float error, float feedforward_frequency, PIDController* pid);

#endif


