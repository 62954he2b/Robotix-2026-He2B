#ifndef CLI_INPUT_H
#define CLI_INPUT_H

#include <Arduino.h>
#include "access_point.h"
#include "rotary_encoder_AS5048.h"
#include "odometry.h"
#include "motion_control.h"
#include "SPI_wrapper.h"

#define START_BYTE 0XAA

extern float orientation_reference;
extern float right_distance_reference;
extern float left_distance_reference;
extern float angular_velocity_reference;
extern float linear_velocity_reference;

extern bool orientation_command;
extern bool forward_command; 
extern bool backward_command; 
extern bool rightMotorCommand;
extern bool leftMotorCommand;
extern bool print_command;

extern bool angle_command;
extern bool distance_command; 

extern volatile bool right_motor_enabled;
extern volatile bool left_motor_enabled;

typedef struct __attribute__((packed)) {
    uint8_t start_byte;
    float target_linear_velocity;
    float target_angular_velocity;
    float emergency_stop;
} DataFromPi;

extern portMUX_TYPE HSPIMutex;

void command_handler(String command);

void read_serial_input_task(void *parameter);
void read_wifi_input_task(void *parameter);
void read_write_HSPI_task(void *parameter);

uint8_t compute_checksum(const uint8_t *data, size_t len);
bool validate_frame(const uint8_t *frame, size_t buffer_size);


#endif