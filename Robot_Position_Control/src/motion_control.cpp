#include "motion_control.h"

MotionState current_position {
    .relative_distance_travelled_left_wheel = 0, 
    .relative_distance_travelled_right_wheel = 0,
    .absolute_distance_travelled_left_wheel = 0,
    .absolute_distance_travelled_right_wheel = 0,          
    .robot_orientation_deg = 0
};

FixedWheel fixed_wheel;

void move_forward(float distance) {

    left_motor_state = IDLE;
    right_motor_state = IDLE;

    right_motor_enabled = false;
    left_motor_enabled = false;

    forward_flag = true;

    right_distance_reference = distance;
    left_distance_reference = distance;

    right_motor_enabled = true;
    left_motor_enabled = true;

}

void move_backward(float distance) {

    left_motor_state = IDLE;
    right_motor_state = IDLE;

    right_motor_enabled = false;
    left_motor_enabled = false;

    right_distance_reference = -distance;
    left_distance_reference = -distance;

    right_motor_enabled = true;
    left_motor_enabled = true;

}

void rotate(float angle) {

    left_motor_state = IDLE;
    right_motor_state = IDLE;
    
    right_motor_enabled = false;
    left_motor_enabled = false;

    right_distance_reference = angle_to_distance_converter(angle);
    left_distance_reference = -(angle_to_distance_converter(angle));

    right_motor_enabled = true;
    left_motor_enabled = true;
}

void pivot(float angle, FixedWheel fixed_wheel) {

    right_motor_enabled = false;
    left_motor_enabled = false;

    if (fixed_wheel == FIXED_RIGHT) {
        left_motor_state = IDLE;
        left_distance_reference = -(pivot_angle_to_distance_converter(angle));
        left_motor_enabled = true;
        right_motor_enabled = false;

    }
    else if (fixed_wheel == FIXED_LEFT) {
        right_motor_state = IDLE;
        right_distance_reference = pivot_angle_to_distance_converter(angle);
        left_motor_enabled = false;
        right_motor_enabled = true;

    }

    //current_position.robot_orientation_deg += angle;

}

void stop_motion(){

    left_motor_state = IDLE;
    right_motor_state = IDLE;
    
    right_motor_enabled = false;
    left_motor_enabled = false;

}


float angle_to_distance_converter(float angle) {
    return ((LENGTH_BETWEEN_ENCODERS_MM/2) * angle/180.0f * PI) / 10;
}

float pivot_angle_to_distance_converter(float angle) {
    return ((LENGTH_BETWEEN_ENCODERS_MM) * angle/180.0f * PI) / 10;
}