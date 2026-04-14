#include "motor_control.h"
#include "rotary_encoder_AS5048.h"
#include "odometry.h"

#define MOTOR_PERIOD_MS 5.0f
#define DT_SEC (MOTOR_PERIOD_MS / 1000.0f)

#define CLOCKWISE LOW
#define ANTI_CLOCKWISE HIGH
#define ACCEPTABLE_ERROR 0.1
#define KP 200

#define STOP_THRESHOLD 5.0f
#define FREQUENCY_UPDATE_THRESHOLD 1.0f

const float minimum_frequency = 0;
const float maximum_frequency = 4000;
bool forward_flag = false;
bool turn_flag = false;

PIDController left_position_PID_coefficients = {500, 0.25, 0.1, 0, 0};
PIDController right_position_PID_coefficients = {500, 0.25, 0.1, 0, 0};
PIDController left_velocity_PID_coefficients = {2000, 0.25, 0.1, 0, 0};
PIDController right_velocity_PID_coefficients = {2000, 0.25, 0.1, 0, 0};

volatile MotorState left_motor_state;
volatile MotorState right_motor_state;

volatile ControlState motors_control_state = AUTOMATIC;

void right_motor_position_control_task(void *parameter) {

	const TickType_t period = pdMS_TO_TICKS(MOTOR_PERIOD_MS);
	TickType_t lastWakeTime = xTaskGetTickCount();

	float current_frequency = maximum_frequency;
	float previous_frequency = 0;
	float correction = 0;
	float last_absolute_angular_position = 0;

	while(true){
		if (motors_control_state == MANUAL) {

			portENTER_CRITICAL(&RightEncoderMutex);
			current_position.absolute_distance_travelled_right_wheel = ((right_encoder.filtered_absolute_angular_position)/revolution*2*PI*WHEEL_RADIUS_MM)/10;
			portEXIT_CRITICAL(&RightEncoderMutex);
				
			if (right_motor_enabled == true) {

				if (!right_encoder.initialized) {
					initialize_encoder(&right_encoder);
					right_encoder.initial_angular_position = right_encoder.current_angular_position;
					last_absolute_angular_position = right_encoder.filtered_absolute_angular_position;
					right_motor_state = RUNNING;
				}

				portENTER_CRITICAL(&RightEncoderMutex);
				right_encoder.filtered_relative_angular_position = (right_encoder.filtered_absolute_angular_position - last_absolute_angular_position);
				current_position.relative_distance_travelled_right_wheel = ((right_encoder.filtered_relative_angular_position)/revolution*2*PI*WHEEL_RADIUS_MM)/10;
				portEXIT_CRITICAL(&RightEncoderMutex);

				float distance_error = right_distance_reference - current_position.relative_distance_travelled_right_wheel;
				
				if(right_distance_reference != 0 && distance_error != 0) {
					current_frequency = PID_control(distance_error, current_frequency, &right_position_PID_coefficients);
					timerAlarmEnable(right_stepper_timer);

					if (forward_flag && left_motor_enabled && right_motor_enabled) {
						float driftError = 0 - robot_state.theta;
						float kP = KP;
						if (driftError > 0) {
							correction = driftError * kP;
						}
						
					}
					// else if (turn_flag && left_motor_enabled && right_motor_enabled) {
					// 	float driftError = -(current_position.distance_travelled_right_wheel + current_position.distance_travelled_left_wheel);
					// 	float kP = KP;
					// 	if (driftError > 0) {
					// 		correction = driftError * kP;
					// 	}
					// }
					else {
						correction = 0;
					}

					float corrected_frequency = current_frequency - correction;
					corrected_frequency = constrain(corrected_frequency, minimum_frequency, maximum_frequency);

					if (previous_frequency != corrected_frequency) {
						previous_frequency = corrected_frequency;
						timerAlarmWrite(right_stepper_timer, 1000000.0f / corrected_frequency, true);
					}
					
					if(distance_error > 0 && abs(distance_error) > ACCEPTABLE_ERROR){
						digitalWrite(DIR_PIN_RIGHT, CLOCKWISE);
						//vTaskDelay(1 / portTICK_PERIOD_MS);
					}
					else if (distance_error < 0 && abs(distance_error) > ACCEPTABLE_ERROR){
						digitalWrite(DIR_PIN_RIGHT, ANTI_CLOCKWISE);
						//vTaskDelay(1 / portTICK_PERIOD_MS); 
					}
					else {
						timerAlarmDisable(right_stepper_timer); 
						//right_distance_reference = 0;
						correction = 0;
						right_motor_enabled = false;
						right_encoder.initialized = false;
						right_motor_state = DONE;
						//vTaskDelay(100 / portTICK_PERIOD_MS);
					}
				}
				else {
					timerAlarmDisable(right_stepper_timer);
					//vTaskDelay(10 / portTICK_PERIOD_MS);
				}	
			}		
			else {
				timerAlarmDisable(right_stepper_timer);
				if (!left_motor_enabled && !right_motor_enabled && forward_flag) {
					forward_flag = false;
				}
				right_motor_state = IDLE;
				//vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			vTaskDelayUntil(&lastWakeTime, period);
		}
		else {
			vTaskDelayUntil(&lastWakeTime, period);
		}
	}
}

void left_motor_position_control_task(void *parameter) {

	const TickType_t period = pdMS_TO_TICKS(MOTOR_PERIOD_MS);
	TickType_t lastWakeTime = xTaskGetTickCount();
	
	float current_frequency = maximum_frequency;
	float previous_frequency = 0;
	float correction = 0;
	float last_absolute_angular_position = 0;

	while(true){
		if (motors_control_state == MANUAL) {	

			portENTER_CRITICAL(&LeftEncoderMutex);
			current_position.absolute_distance_travelled_left_wheel = ((left_encoder.filtered_absolute_angular_position)/revolution*2*PI*WHEEL_RADIUS_MM)/10;
			portEXIT_CRITICAL(&LeftEncoderMutex);

			if (left_motor_enabled == true) {

				if (!left_encoder.initialized) {
					initialize_encoder(&left_encoder);
					left_encoder.initial_angular_position = left_encoder.current_angular_position;
					last_absolute_angular_position = left_encoder.filtered_absolute_angular_position;
					left_motor_state = RUNNING;
				}

				portENTER_CRITICAL(&LeftEncoderMutex);
				left_encoder.filtered_relative_angular_position = (left_encoder.filtered_absolute_angular_position - last_absolute_angular_position);// - left_encoder.initial_angular_position;
				current_position.relative_distance_travelled_left_wheel = ((left_encoder.filtered_relative_angular_position)/revolution*2*PI*WHEEL_RADIUS_MM)/10;
				portEXIT_CRITICAL(&LeftEncoderMutex);

				float distance_error = left_distance_reference - current_position.relative_distance_travelled_left_wheel;
				
				if(left_distance_reference != 0 && distance_error != 0) {
					current_frequency = PID_control(distance_error, current_frequency, &left_position_PID_coefficients);

					timerAlarmEnable(left_stepper_timer); 

					if (forward_flag && left_motor_enabled && right_motor_enabled) {
						float driftError = 0 + robot_state.theta;
						float kP = KP;
						if (driftError > 0) {
							correction = driftError * kP;
						}
					}
					// else if (turn_flag && left_motor_enabled && right_motor_enabled) {
					// 	float driftError = -(current_position.distance_travelled_left_wheel + current_position.distance_travelled_right_wheel);
					// 	float kP = KP;
					// 	if (driftError > 0) {
					// 		correction = driftError * kP;
					// 	}
					//}
					else {
						correction = 0;
					}

					float corrected_frequency = current_frequency - correction;
					corrected_frequency = constrain(corrected_frequency, minimum_frequency, maximum_frequency);

					if (previous_frequency != corrected_frequency) {
						previous_frequency = corrected_frequency;
						timerAlarmWrite(left_stepper_timer, 10000000.0f / corrected_frequency, true);
					}	

					if(distance_error > 0 && abs(distance_error) > ACCEPTABLE_ERROR){
						digitalWrite(DIR_PIN_LEFT, CLOCKWISE);
					}
					else if (distance_error < 0 && abs(distance_error) > ACCEPTABLE_ERROR){
						digitalWrite(DIR_PIN_LEFT, ANTI_CLOCKWISE);
					}
					else {
						timerAlarmDisable(left_stepper_timer);
						//left_distance_reference = 0;
						correction = 0;
						left_motor_enabled = false;
						left_encoder.initialized = false;
						left_motor_state = DONE;
					}
				}
				else {
					timerAlarmDisable(left_stepper_timer);
				}	
			}		
			else {
				timerAlarmDisable(left_stepper_timer);
				if (!left_motor_enabled && !right_motor_enabled && forward_flag) {
					forward_flag = false;
				}
				left_motor_state = IDLE;
			}
			vTaskDelayUntil(&lastWakeTime, period);
		}
		else {
			vTaskDelayUntil(&lastWakeTime, period);
		}
	}
}

void right_motor_velocity_control_task(void *parameter) {

	const TickType_t period = pdMS_TO_TICKS(MOTOR_PERIOD_MS);
	TickType_t lastWakeTime = xTaskGetTickCount();
	
	float current_frequency = maximum_frequency;
	float previous_frequency = 0;

	while(true){
		if (motors_control_state == AUTOMATIC) {

			portENTER_CRITICAL(&HSPIMutex);
			float right_velocity_reference = linear_velocity_reference + (angular_velocity_reference * ((WHEEL_BASE_MM / 1000.0f) / 2.0f));
			portEXIT_CRITICAL(&HSPIMutex);

			float right_angular_velocity_reference  = right_velocity_reference  / ( WHEEL_RADIUS_MM / 1000.0f);
			current_frequency  = (right_angular_velocity_reference  / (2 * PI)) * STEPS_PER_REV;

			portENTER_CRITICAL(&RightEncoderMutex);
			current_position.absolute_distance_travelled_right_wheel = ((right_encoder.filtered_absolute_angular_position)/revolution*2*PI*WHEEL_RADIUS_MM) / 10;
			float right_current_velocity = right_encoder.filtered_linear_velocity;
			portEXIT_CRITICAL(&RightEncoderMutex);

			float velocity_error = right_velocity_reference - right_current_velocity;
			
			current_frequency = PID_control(velocity_error, current_frequency, &right_velocity_PID_coefficients);
			current_frequency = constrain(current_frequency, -maximum_frequency, maximum_frequency);

			if (fabsf(current_frequency) < STOP_THRESHOLD) {
				timerAlarmDisable(right_stepper_timer);
				right_velocity_PID_coefficients.integral = 0.0f;
				right_velocity_PID_coefficients.previousError = 0.0f;
				vTaskDelayUntil(&lastWakeTime, period);
				continue;
			}

			if( current_frequency >= 0 ){ 
				digitalWrite(DIR_PIN_RIGHT, CLOCKWISE);
			}
			else if ( current_frequency < 0 ){
				digitalWrite(DIR_PIN_RIGHT, ANTI_CLOCKWISE);
			}

			timerAlarmEnable(right_stepper_timer);

			if (fabsf(previous_frequency - current_frequency) > FREQUENCY_UPDATE_THRESHOLD) {
				previous_frequency = current_frequency;
				timerAlarmWrite(right_stepper_timer, 1000000.0f / fabsf(current_frequency), true);
			}	

			vTaskDelayUntil(&lastWakeTime, period);
		}
		else if (motors_control_state == EMERGENCY_STOP) {
			timerAlarmDisable(right_stepper_timer);
			vTaskDelayUntil(&lastWakeTime, period);
		}
		else {
			vTaskDelayUntil(&lastWakeTime, period);
		}
	}
}

void left_motor_velocity_control_task(void *parameter) {

	const TickType_t period = pdMS_TO_TICKS(MOTOR_PERIOD_MS);
	TickType_t lastWakeTime = xTaskGetTickCount();
	
	float current_frequency = maximum_frequency;
	float previous_frequency = 0;

	while(true){

		if(motors_control_state == AUTOMATIC){

			portENTER_CRITICAL(&HSPIMutex);
			float left_velocity_reference  = linear_velocity_reference - (angular_velocity_reference * (( WHEEL_BASE_MM / 1000.0f) / 2.0f));
			portEXIT_CRITICAL(&HSPIMutex);
			
			float left_angular_velocity_reference  = left_velocity_reference  / ( WHEEL_RADIUS_MM / 1000.0f);

			current_frequency  = (left_angular_velocity_reference  / (2 * PI)) * STEPS_PER_REV;

			portENTER_CRITICAL(&LeftEncoderMutex);
			current_position.absolute_distance_travelled_left_wheel = ((left_encoder.filtered_absolute_angular_position)/revolution*2*PI*WHEEL_RADIUS_MM) / 10;
			float left_current_velocity = left_encoder.filtered_linear_velocity;
			portEXIT_CRITICAL(&LeftEncoderMutex);

			float velocity_error = left_velocity_reference - left_current_velocity;
			
			current_frequency = PID_control(velocity_error, current_frequency, &left_velocity_PID_coefficients);
			current_frequency = constrain(current_frequency, -maximum_frequency, maximum_frequency);

			if (fabsf(current_frequency) < STOP_THRESHOLD) {
				timerAlarmDisable(left_stepper_timer);
				left_velocity_PID_coefficients.integral = 0.0f;
				left_velocity_PID_coefficients.previousError = 0.0f;
				vTaskDelayUntil(&lastWakeTime, period);
				continue;
			}
			
			if( current_frequency >= 0 ){ 
				digitalWrite(DIR_PIN_LEFT, CLOCKWISE);
			}
			else if ( current_frequency < 0 ){
				digitalWrite(DIR_PIN_LEFT, ANTI_CLOCKWISE);
			}

			timerAlarmEnable(left_stepper_timer);


			if (fabsf(previous_frequency - current_frequency) > FREQUENCY_UPDATE_THRESHOLD) {
				previous_frequency = current_frequency;
				timerAlarmWrite(left_stepper_timer, 1000000.0f / fabsf(current_frequency), true);
			}	

			
			vTaskDelayUntil(&lastWakeTime, period);
		}
		else if (motors_control_state == EMERGENCY_STOP) {
			timerAlarmDisable(left_stepper_timer);
			vTaskDelayUntil(&lastWakeTime, period);
		}
		else {
			vTaskDelayUntil(&lastWakeTime, period);
		}
	}
}

float PID_control(float error, float feedforward_frequency, PIDController* pid) {

    float dt = DT_SEC;

    if (motors_control_state == MANUAL) {
        error = fabsf(error);
    }
    
    pid->integral += error * dt;
    pid->integral = constrain(pid->integral, -2000.0f, 2000.0f); // ANTI-WINDUP Security
    float derivative = (error - pid->previousError) / dt;
    pid->previousError = error;

    float pid_correction = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    float corrected_frequency = feedforward_frequency + pid_correction;

    return corrected_frequency;
}