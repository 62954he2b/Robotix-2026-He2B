#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <Arduino.h>
#include "cli_input.h"
#include "motor_control.h"

typedef struct {
    float distance_travelled_left_wheel, distance_travelled_right_wheel; 
    float robot_distance_travelled;   
    float robot_orientation_deg;                 
} MotionState;

enum FixedWheel {
    FIXED_LEFT,
    FIXED_RIGHT
};

extern MotionState current_position;

void move_forward(float distance);
void move_backward(float distance);
void rotate(float angle);
void pivot(float angle, bool left_motor_enabled, bool right_motor_enabled);
void stop_motion();

float angle_to_distance_converter(float angle);
float pivot_angle_to_distance_converter(float angle);

#endif