#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include "cli_input.h"
#include "stepper_motors.h"

#define revolution 360
#define motor_step_per_rev 200
#define step_size 8

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

extern volatile MotorState left_motor_state;
extern volatile MotorState right_motor_state;

void azimuth_control_task(void *parameter);
void right_motor_control_task(void *parameter);
void left_motor_control_task(void *parameter);
void distance_control_task(void *parameter);
float motor_speed_control(float error, float currentFreq, PIDController* pid);
float motor_rotation_speed_control(float error, float currentFreq);

#endif


