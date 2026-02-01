#include "imu_sensor_BNO080.h"

HardwareSerial BNO080x_Serial(2);

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

float angle_value_IMU = 0;

void initiate_BNO080(){
  Serial.println("Adafruit BNO08x IMU - UART-RVC mode");

  BNO080x_Serial.begin(115200); // This is the baud rate specified by the datasheet
  while (!BNO080x_Serial)
    delay(10);

  if (!rvc.begin(&BNO080x_Serial)) { // connect to the sensor over hardware serial
    Serial.println("Could not find BNO08x!");
    while (1)
      delay(10);
  }

  Serial.println("BNO08x found!");

}

void imu_BNO080_read_task(void *parameter) {

	while (1) {
		BNO08x_RVC_Data heading;

		if (!rvc.read(&heading)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
      		continue; 
		}

		angle_value_IMU = heading.yaw;

		// Serial.print(F("Yaw: "));
		// Serial.println(heading.yaw);
		
		// Serial.print(F("rotation: "));
		// Serial.println(rotationAngle);
		
	  vTaskDelay(1 / portTICK_PERIOD_MS);  
	}
	vTaskDelay(10 / portTICK_PERIOD_MS); 
}