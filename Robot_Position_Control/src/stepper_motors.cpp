#include "stepper_motors.h"

hw_timer_t * left_stepper_timer = NULL;
hw_timer_t * right_stepper_timer = NULL;

TMC5160Stepper left_driver = TMC5160Stepper(CS_PIN_LEFT, R_SENSE);
TMC5160Stepper right_driver = TMC5160Stepper(CS_PIN_RIGHT, R_SENSE);
AccelStepper left_stepper(AccelStepper::DRIVER, STEP_PIN_LEFT, DIR_PIN_LEFT);  
AccelStepper right_stepper(AccelStepper::DRIVER, STEP_PIN_RIGHT, DIR_PIN_RIGHT);

void setupDriver(TMC5160Stepper &driver) {
  driver.begin();
  driver.toff(4); 
  driver.rms_current(1000); // en mA
  driver.microsteps(4);
  driver.en_pwm_mode(false);
  driver.pwm_autoscale(true); // Auto courant
  driver.VSTART(10);
  driver.VSTOP(10);
}