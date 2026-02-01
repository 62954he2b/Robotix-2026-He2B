#include "stepper_motors.h"
#include "cli_input.h"
#include "access_point.h"
#include "rotary_encoder_AS5048.h"
#include "motor_control.h"
#include "odometry.h"

volatile bool leftStepState = false;
volatile bool rightStepState = false;

// Fonctions d'interruption pour les canaux PWM de l'encodeur gauche
void IRAM_ATTR leftEncoder_handleEdge() {
	portENTER_CRITICAL_ISR(&encoderMutex);
    uint32_t left_now = micros();

    if (digitalRead(LEFT_PWM_PIN) == HIGH) {
        left_encoder.period_us = left_now - left_encoder.last_rise_time;
        left_encoder.last_rise_time = left_now;
        left_encoder.rise_time = left_now;

    } else {
        left_encoder.high_time = left_now - left_encoder.rise_time;
        left_encoder.new_sample = true;
    }
	portEXIT_CRITICAL_ISR(&encoderMutex);
}

// Fonctions d'interruption pour les canaux PWM de l'encodeur gauche
void IRAM_ATTR rightEncoder_handleEdge() {
	portENTER_CRITICAL_ISR(&encoderMutex);
    uint32_t right_now = micros();

    if (digitalRead(RIGHT_PWM_PIN) == HIGH) {
        right_encoder.period_us = right_now - right_encoder.last_rise_time;
        right_encoder.last_rise_time = right_now;
        right_encoder.rise_time = right_now;

    } else {
        right_encoder.high_time = right_now - right_encoder.rise_time;
        right_encoder.new_sample = true;
    }
	portEXIT_CRITICAL_ISR(&encoderMutex);
}

void IRAM_ATTR leftStepperISR() {
  leftStepState = !leftStepState;
  gpio_set_level((gpio_num_t)STEP_PIN_LEFT, leftStepState);
}

void IRAM_ATTR rightStepperISR() {
  rightStepState = !rightStepState;
  gpio_set_level((gpio_num_t)STEP_PIN_RIGHT, rightStepState);
}

void setup()
{
	Serial.begin(115200);
	initiate_access_point(ssid, password);
	//initiate_BNO080(); 

	// VSPI - DRIVERS STEPPER MOTORS
	SPI.begin();

	// HSPI - COMMUNICATION WITH RPI
	HSPIInitialisation();

	pinMode(EN_PIN_LEFT, OUTPUT);
	digitalWrite(EN_PIN_LEFT, LOW);
	pinMode(EN_PIN_RIGHT, OUTPUT);
	digitalWrite(EN_PIN_RIGHT, LOW);
	pinMode(STEP_PIN_LEFT, OUTPUT); 
	pinMode(DIR_PIN_LEFT, OUTPUT);
	pinMode(STEP_PIN_RIGHT, OUTPUT);
	pinMode(DIR_PIN_RIGHT, OUTPUT);

	setupDriver(left_driver);
	left_stepper.setMaxSpeed(2500);
	left_stepper.setAcceleration(2500);

	setupDriver(right_driver);
	right_stepper.setMaxSpeed(2500);
	right_stepper.setAcceleration(2500);

	// initialize buffers
	memset(left_encoder.median_buf, 0, sizeof(left_encoder.median_buf));
    memset(left_encoder.average_buf, 0, sizeof(left_encoder.average_buf));
	memset(right_encoder.median_buf, 0, sizeof(right_encoder.median_buf));
    memset(right_encoder.average_buf, 0, sizeof(right_encoder.average_buf));

	// Declare pins of encoders as inputs
    pinMode(LEFT_PWM_PIN, INPUT);
    pinMode(RIGHT_PWM_PIN, INPUT);

	// Configurer les interruptions pour les encodeurs
	attachInterrupt(digitalPinToInterrupt(LEFT_PWM_PIN), leftEncoder_handleEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_PWM_PIN), rightEncoder_handleEdge, CHANGE);

	// left Stepper Motor Timer 0 64-bit
	left_stepper_timer = timerBegin(0, 80, true); // Timer 0, prescaler de 80 (1 tick = 1 µs)
	timerAttachInterrupt(left_stepper_timer, &leftStepperISR, true);
	timerAlarmWrite(left_stepper_timer, 1000000 / maximum_frequency, true); // 400 µs = 2,5 kHz

	// right Stepper Motor Timer 1 64-bit
	right_stepper_timer = timerBegin(1, 80, true); // Timer 1, prescaler de 80 (1 tick = 1 µs)
	timerAttachInterrupt(right_stepper_timer, &rightStepperISR, true);
	timerAlarmWrite(right_stepper_timer, 1000000 / maximum_frequency, true); // 400 µs = 2,5 kHz

	// Démarrer les tâches sur les cœurs respectifs
	//xTaskCreatePinnedToCore(azimuth_control_task, "AzimuthControl", 4096, NULL, 2, NULL, 0);
	//xTaskCreatePinnedToCore(distance_control_task, "DistanceControl", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(right_motor_control_task, "RigMotorDistanceControl", 4096, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(left_motor_control_task, "LeftMotorDistanceControl", 4096, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(left_encoder_reading_task, "LeftEncoderRead", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(right_encoder_reading_task, "RightEncoderRead", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(read_serial_input_task, "SerialInputTask", 4096, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(read_wifi_input_task, "WifiInputTask", 4096, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(read_HSPI_input_task, "RPItoESP32Task", 4096, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(odometry_task, "Odometry", 4096, NULL, 0, NULL, 1);
	//xTaskCreatePinnedToCore(imu_BNO080_read_task, "IMUReadTask", 4096, NULL, 1, NULL, 1);
	}

void loop()
{
	if (left_motor_enabled) {
		Serial.printf(
			"LEFT  | init:%8.2f° cur:%8.2f° abs:%8.2f° rel:%8.2f° ref:%8.2f dist:%8.2f  x : %8.2f  y : %8.2f theta = %8.2f\n",
			left_encoder.initial_angular_position,
			left_encoder.current_angular_position,
			left_encoder.filtered_absolute_angular_position,
			left_encoder.filtered_relative_angular_position,
			left_distance_reference,
			current_position.distance_travelled_left_wheel,
			x_y_position.x,
			x_y_position.y,
			x_y_position.theta
		);
	}

	if (right_motor_enabled) {
		Serial.printf(
			"RIGHT | init:%8.2f° cur:%8.2f° abs:%8.2f° rel:%8.2f° ref:%8.2f dist:%8.2f  x : %8.2f  y : %8.2f  theta = %8.2f\n",
			right_encoder.initial_angular_position,
			right_encoder.current_angular_position,
			right_encoder.filtered_absolute_angular_position,
			right_encoder.filtered_relative_angular_position,
			right_distance_reference,
			current_position.distance_travelled_right_wheel,
			x_y_position.x,
			x_y_position.y,
			x_y_position.theta
		);
	}

}









