#ifndef ROTARYENCODERAS5048_H
#define ROTARYENCODERAS5048_H

#include <Arduino.h>
#include "motor_control.h"

#define LEFT_PWM_PIN 25
#define RIGHT_PWM_PIN 26

#define DELTA_MAX_ALLOWED 60

#define MEDIAN_SIZE 5
#define AVG_SIZE 10

#define WHEEL_RADIUS_MM 22.50f
#define WHEEL_BASE_MM 180.00f

typedef struct {
    volatile uint32_t rise_time;
    volatile uint32_t last_rise_time;
    volatile uint32_t high_time;
    volatile uint32_t period_us;
    volatile bool sampled;

    float initial_angular_position;
    float absolute_angular_position;
    float current_angular_position;
    float filtered_absolute_angular_position;
    float previous_filtered_absolute_angular_position;
    float filtered_relative_angular_position;
    float filtered_angular_velocity;
    float previous_filtered_angular_velocity;
    float filtered_linear_velocity;
    float velocity_reference;

    bool initialized;
    
} EncoderAS5048;

extern EncoderAS5048 left_encoder;
extern EncoderAS5048 right_encoder;

extern portMUX_TYPE LeftEncoderMutex;
extern portMUX_TYPE RightEncoderMutex;

void left_encoder_reading_task(void *parameter);
void right_encoder_reading_task(void *parameter);

#endif