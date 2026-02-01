#ifndef ROTARYENCODERAS5048_H
#define ROTARYENCODERAS5048_H

#include <Arduino.h>
#include "motor_control.h"

#define LEFT_PWM_PIN 25
#define RIGHT_PWM_PIN 26

#define MEDIAN_SIZE 5
#define AVG_SIZE 10

#define WHEEL_RADIUS 22.50  //0.637 pour engrenage
#define LENGTH_BETWEEN_ENCODERS_MM 165

typedef struct {
    volatile uint32_t rise_time;
    volatile uint32_t last_rise_time;
    volatile uint32_t high_time;
    volatile uint32_t period_us;
    volatile bool new_sample;

    float median_buf[MEDIAN_SIZE];
    float average_buf[AVG_SIZE]; 
    int median_index;
    int average_index;

    float initial_angular_position;
    float absolute_angular_position;
    float current_angular_position;
    int revolution_counter;
    float filtered_absolute_angular_position;
    float filtered_relative_angular_position;

    bool initialized;
    
} EncoderAS5048;

extern EncoderAS5048 left_encoder;
extern EncoderAS5048 right_encoder;

extern portMUX_TYPE encoderMutex;

void initialize_encoder(EncoderAS5048 *encoder);
void filter_angle_value(EncoderAS5048 *encoder);
void left_encoder_reading_task(void *parameter);
void right_encoder_reading_task(void *parameter);

#endif