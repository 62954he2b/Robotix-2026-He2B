#ifndef INPUTTASKS_H
#define INPUTTASKS_H

#include <Arduino.h>
#include "access_point.h"
#include "driver/spi_slave.h"
#include "rotary_encoder_AS5048.h"
#include "odometry.h"
#include "motion_control.h"

#define HSPI_MOSI 13
#define HSPI_MISO 12
#define HSPI_SCLK 14
#define HSPI_CS   15

#define SPI_SLAVE_HOST HSPI

extern float orientation_reference;
extern float right_distance_reference;
extern float left_distance_reference;

extern bool orientation_command;
extern bool forward_command; 
extern bool backward_command; 
extern bool rightMotorCommand;
extern bool leftMotorCommand;

extern bool angle_command;
extern bool distance_command; 

extern volatile bool right_motor_enabled;
extern volatile bool left_motor_enabled;

void command_handler(String command);
void HSPIInitialisation();

void read_serial_input_task(void *parameter);
void read_wifi_input_task(void *parameter);
void read_HSPI_input_task(void *parameter);

#endif