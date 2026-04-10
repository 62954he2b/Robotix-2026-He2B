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
	portENTER_CRITICAL_ISR(&LeftEncoderMutex);
    uint32_t left_now = micros();

    if (digitalRead(LEFT_PWM_PIN) == HIGH) {
        left_encoder.period_us = left_now - left_encoder.last_rise_time;
        left_encoder.last_rise_time = left_now;

    } else {
        left_encoder.high_time = left_now - left_encoder.last_rise_time;
        left_encoder.new_sample = true;
    }
	portEXIT_CRITICAL_ISR(&LeftEncoderMutex);
}

// Fonctions d'interruption pour les canaux PWM de l'encodeur gauche
void IRAM_ATTR rightEncoder_handleEdge() {
	portENTER_CRITICAL_ISR(&RightEncoderMutex);
    uint32_t right_now = micros();

    if (digitalRead(RIGHT_PWM_PIN) == HIGH) {
        right_encoder.period_us = right_now - right_encoder.last_rise_time;
        right_encoder.last_rise_time = right_now;

    } else {
        right_encoder.high_time = right_now - right_encoder.last_rise_time;
        right_encoder.new_sample = true;
    }
	portEXIT_CRITICAL_ISR(&RightEncoderMutex);
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
	HSPI_initialisation();

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
	xTaskCreatePinnedToCore(right_motor_velocity_control_task, "RigMotorVelocityControl", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(left_motor_velocity_control_task, "LeftMotorVelocityControl", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(right_motor_position_control_task, "RigMotorPositionControl", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(left_motor_position_control_task, "LeftMotorPositionControl", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(left_encoder_reading_task, "LeftEncoderRead", 4096, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(right_encoder_reading_task, "RightEncoderRead", 4096, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(read_serial_input_task, "SerialInputTask", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(read_wifi_input_task, "WifiInputTask", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(read_write_HSPI_task, "RPItoESP32Task", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(odometry_task, "Odometry", 4096, NULL, 1, NULL, 0);
 
}

void loop()
{	
	static bool cleared_screen = false;

	if (motors_control_state == MANUAL) {
		if(print_command){
			if (!cleared_screen) {
				Serial.print("\033[2J"); 
				cleared_screen = true;
			}

			Serial.print("\033[H");  // home

			Serial.println("=========== ROBOT STATE ===========");

			Serial.printf("POSITION   | X:%7.2f  Y:%7.2f  TH:%7.2f\n",
				robot_state.x,
				robot_state.y,
				robot_state.theta
			);

			Serial.printf("VELOCITY   | LINEAR - REF: %7.4f  CUR: %7.4f m/s\n",
				linear_velocity_reference,
				robot_state.current_linear_velocity
			);
			Serial.printf("VELOCITY   | ANGULAR - REF: %7.4f  CUR: %7.4f  rad/s\n",
				angular_velocity_reference,
				robot_state.current_angular_velocity
			);

			Serial.println("-----------------------------------");

			Serial.println("LEFT WHEEL");
			Serial.printf("ANG   | init    :%7.2f  cur :%7.2f  abs     :%7.2f  rel :%7.2f\n",
				left_encoder.initial_angular_position,
				left_encoder.current_angular_position,
				left_encoder.filtered_absolute_angular_position,
				left_encoder.filtered_relative_angular_position
			);
			Serial.printf("CTRL  | abs_dist:%7.2f  ref :%7.2f  rel_dist:%7.2f\n",
				current_position.absolute_distance_travelled_left_wheel,
				left_distance_reference,
				current_position.relative_distance_travelled_left_wheel
			);

			Serial.println("-----------------------------------");

			Serial.println("RIGHT WHEEL");
			Serial.printf("ANG   | init    :%7.2f  cur :%7.2f  abs     :%7.2f  rel :%7.2f\n",
				right_encoder.initial_angular_position,
				right_encoder.current_angular_position,
				right_encoder.filtered_absolute_angular_position,
				right_encoder.filtered_relative_angular_position
			);
			Serial.printf("CTRL  | abs_dist:%7.2f  ref :%7.2f  rel_dist:%7.2f\n",
				current_position.absolute_distance_travelled_right_wheel,
				right_distance_reference,
				current_position.relative_distance_travelled_right_wheel
			);
		}
		else{
			cleared_screen = false;
		}

	}
	else if (motors_control_state == AUTOMATIC){

		if(print_command){

			if (!cleared_screen) {
				Serial.print("\033[2J");
				cleared_screen = true;
			}

			Serial.println("\033[H");
			Serial.printf("current position - x : %8.2f y : %8.2f theta : %8.2f ang_vel : %8.5f °/s lin_vel : %8.5f m/s",
				robot_state.x, 
				robot_state.y, 
				robot_state.theta, 
				angular_velocity_reference,
				linear_velocity_reference
			);
		}
		else{
			cleared_screen = false;
		}
	}
	else if (motors_control_state == EMERGENCY_STOP){

		if (!cleared_screen) {
				Serial.print("\033[2J");
				cleared_screen = true;
			}

			Serial.println("\033[H");
			Serial.println("EMERGENCY STOP");
	}

}









