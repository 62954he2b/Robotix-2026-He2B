#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "rotary_encoder_AS5048.h"
#include "motor_control.h"

typedef struct {
    float x;       
    float y;       
    float theta;   
} Odometry_t;

extern Odometry_t x_y_position;

void odometry_task(void *parameter);
void odometry_update(Odometry_t *odom, float delta_left, float delta_right, float wheel_base);

#endif