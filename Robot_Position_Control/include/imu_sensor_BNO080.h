#ifndef BNO080_H
#define BNO080_H

#include <Arduino.h>
#include "Adafruit_BNO08x_RVC.h"
#include "cli_input.h"

#define BNO08X_TX 16 
#define BNO08X_RX 17

extern HardwareSerial BNO080x_Serial;
extern Adafruit_BNO08x_RVC rvc;

extern float angle_value_IMU;

void initiate_BNO080();
void imu_BNO080_read_task(void *parameter);

#endif