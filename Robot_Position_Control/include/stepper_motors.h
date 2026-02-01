#ifndef STEPPERMOTORS_H
#define STEPPERMOTORS_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

#define EN_PIN_RIGHT    32
#define CS_PIN_RIGHT    5
#define STEP_PIN_RIGHT 22
#define DIR_PIN_RIGHT   21

#define EN_PIN_LEFT   33
#define CS_PIN_LEFT   4
#define STEP_PIN_LEFT  2
#define DIR_PIN_LEFT  15

#define SCK 18
#define MISO 19
#define MOSI 23

#define R_SENSE  0.075f  // Valeur du sense resistor sur les drivers

extern hw_timer_t * left_stepper_timer;
extern hw_timer_t * right_stepper_timer;

extern TMC5160Stepper left_driver;
extern TMC5160Stepper right_driver;
extern AccelStepper left_stepper;  
extern AccelStepper right_stepper;

void setupDriver(TMC5160Stepper &driver);

#endif