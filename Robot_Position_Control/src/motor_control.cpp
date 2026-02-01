#include "motor_control.h"
#include "rotary_encoder_AS5048.h"
#include "odometry.h"

#define CLOCKWISE LOW
#define ANTI_CLOCKWISE HIGH
#define ACCEPTABLE_ERROR 0.5
#define KP 20

const float minimum_frequency = 500;
const float maximum_frequency = 4000;
bool forward_flag = false;
bool turn_flag = false;

PIDController leftPID = {500, 0.25, 0.1, 0, 0, 0};
PIDController rightPID = {500, 0.25, 0.1, 0, 0, 0};

volatile MotorState left_motor_state;
volatile MotorState right_motor_state;

/*void azimuth_control_task(void *parameter) {
  float currentFreq = maximum_frequency; 
  float previousFreq = 0;
  float maxAngleError = 0;
  bool initCaptured = false;

  while (1) {
	if (!turn_flag) {
		vTaskDelay(10 / portTICK_PERIOD_MS); 
		continue;
	}

    if (turn_flag) {
      float angleError = orientation_reference - current_angle;

		if (turn_flag && !initCaptured) {
		maxAngleError = angleError;
		if (abs(maxAngleError) < 20) {
			currentFreq = maximum_frequency/2; 
		}
		initCaptured = true;
		}				

		if (abs(angleError) > 1) {
			currentFreq = motor_rotation_speed_control(angleError, currentFreq);
			
			if (previousFreq != currentFreq) {
				previousFreq = currentFreq;
				uint32_t period_us = 1000000 / currentFreq;
				timerAlarmWrite(left_stepper_timer, period_us, true);
				timerAlarmWrite(right_stepper_timer, period_us, true);
			}

			timerAlarmEnable(left_stepper_timer);
			timerAlarmEnable(right_stepper_timer);

			if (angleError > 0) {
				digitalWrite(DIR_PIN_RIGHT, HIGH);
				digitalWrite(DIR_PIN_LEFT, LOW);
			} else {
				digitalWrite(DIR_PIN_RIGHT, LOW);
				digitalWrite(DIR_PIN_LEFT, HIGH);
			}

			vTaskDelay(1 / portTICK_PERIOD_MS);
		} else {
				timerAlarmDisable(left_stepper_timer);
				timerAlarmDisable(right_stepper_timer);
				turn_flag = false;
				initCaptured = false;
				currentFreq = maximum_frequency;
		}
  	}
  }
}

void distance_control_task(void *parameter) {
	float currentFreq = maximum_frequency;
	static bool initDriftCaptured = false;
	static float driftReference = 0;
	float previousRightFreq = 0;
	float rightFreq = 0;
	float previousLeftFreq = 0;
	float leftFreq = 0;

	while(1){
		if (!forward_flag) {
			vTaskDelay(10 / portTICK_PERIOD_MS); 
		}
		if (forward_flag == true) {

			float distanceReached = current_position.robot_distance_travelled;
			float distance_error = distanceReference - distanceReached;

			if (forward_flag && !initDriftCaptured) {
				driftReference = current_angle;
				initDriftCaptured = true;
			}

			if(distanceReference != 0 && distance_error != 0) {
				currentFreq = motor_speed_control(distance_error, currentFreq);
				timerAlarmEnable(left_stepper_timer); 
				timerAlarmEnable(right_stepper_timer); 

				float driftError = driftReference - angle_value_IMU;
				float kP = 250;
				float correction = driftError * kP;

				if(distanceReference > 0) {
					rightFreq = currentFreq + correction;
					leftFreq  = currentFreq - correction;
				} 
				else {
					rightFreq = currentFreq - correction;
					leftFreq  = currentFreq + correction;					
				}

				rightFreq = constrain(rightFreq, minimum_frequency, maximum_frequency);
				leftFreq  = constrain(leftFreq, minimum_frequency, maximum_frequency);
				
				if (previousLeftFreq != leftFreq) {
					previousLeftFreq = leftFreq;
					timerAlarmWrite(left_stepper_timer, 1000000 / leftFreq, true);
				}

				if (previousRightFreq != rightFreq) {
					previousRightFreq = rightFreq;
					timerAlarmWrite(right_stepper_timer, 1000000 / rightFreq, true);
				}

			
				if(distance_error > 0 && abs(distance_error) > ACCEPTABLE_ERROR){
					digitalWrite(DIR_PIN_LEFT, HIGH);
					digitalWrite(DIR_PIN_RIGHT, HIGH);
					vTaskDelay(1 / portTICK_PERIOD_MS);  
				}
				else if(distance_error < 0 && abs(distance_error) > ACCEPTABLE_ERROR){
					digitalWrite(DIR_PIN_LEFT, LOW);
					digitalWrite(DIR_PIN_RIGHT, LOW);
					vTaskDelay(1 / portTICK_PERIOD_MS);  
				}
				else {
          			timerAlarmDisable(left_stepper_timer); 
					timerAlarmDisable(right_stepper_timer);
					currentFreq = maximum_frequency;
					distanceReference = 0;
					forward_flag = false;
					initDriftCaptured = false;
					vTaskDelay(100 / portTICK_PERIOD_MS);  
				}
			}
			else {
				timerAlarmDisable(left_stepper_timer); 
				timerAlarmDisable(right_stepper_timer); 
				vTaskDelay(10 / portTICK_PERIOD_MS);  
			}	
		}		

	}
}*/

void right_motor_control_task(void *parameter) {

	float current_frequency = maximum_frequency;
	float previous_frequency = 0;
	float correction = 0;

	while(1){
		if (right_motor_enabled == true) {

			if (!right_encoder.initialized) {
				initialize_encoder(&right_encoder);
				right_encoder.initial_angular_position = right_encoder.current_angular_position;
				right_motor_state = RUNNING;
			}

			right_encoder.filtered_relative_angular_position = (-right_encoder.absolute_angular_position + right_encoder.initial_angular_position);

			current_position.distance_travelled_right_wheel = ((right_encoder.filtered_relative_angular_position)/revolution*2*PI*WHEEL_RADIUS)/10;
			float distance_error = right_distance_reference - current_position.distance_travelled_right_wheel;
			
			if(right_distance_reference != 0 && distance_error != 0) {
				current_frequency = motor_speed_control(distance_error, current_frequency, &rightPID);
				timerAlarmEnable(right_stepper_timer);

				if (forward_flag && left_motor_enabled && right_motor_enabled) {
					float driftError = current_position.distance_travelled_right_wheel - current_position.distance_travelled_left_wheel;
					float kP = KP;
					if (driftError > 0) {
						correction = driftError * kP;
					}
					
				}
				else if (turn_flag && left_motor_enabled && right_motor_enabled) {
					float driftError = -(current_position.distance_travelled_right_wheel + current_position.distance_travelled_left_wheel);
					float kP = KP;
					if (driftError > 0) {
						correction = driftError * kP;
					}
				}
				else {
					correction = 0;
				}

				float corrected_frequency = current_frequency - correction;
				corrected_frequency = constrain(corrected_frequency, minimum_frequency, maximum_frequency);

				if (previous_frequency != corrected_frequency) {
					previous_frequency = corrected_frequency;
					timerAlarmWrite(right_stepper_timer, 1000000 / corrected_frequency, true);
				}
				
				if(distance_error > 0 && abs(distance_error) > ACCEPTABLE_ERROR){
					digitalWrite(DIR_PIN_RIGHT, CLOCKWISE);
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				else if (distance_error < 0 && abs(distance_error) > ACCEPTABLE_ERROR){
					digitalWrite(DIR_PIN_RIGHT, ANTI_CLOCKWISE);
					vTaskDelay(1 / portTICK_PERIOD_MS); 
				}
				else {
					timerAlarmDisable(right_stepper_timer); 
					right_distance_reference = 0;
					correction = 0;
					//right_motor_enabled = false;
					right_encoder.initialized = false;
					right_motor_state = DONE;
					vTaskDelay(100 / portTICK_PERIOD_MS);
				}
			}
			else {
				timerAlarmDisable(right_stepper_timer);
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}	
		}		
		else {
			timerAlarmDisable(right_stepper_timer);
			if (!left_motor_enabled && !right_motor_enabled && forward_flag) {
				forward_flag = false;
			}
			right_motor_state = IDLE;
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}

	}

}

void left_motor_control_task(void *parameter) {
	
	float current_frequency = maximum_frequency;
	float previous_frequency = 0;
	float correction = 0;

	while(1){
		if (left_motor_enabled == true) {

			if (!left_encoder.initialized) {
				initialize_encoder(&left_encoder);
				left_encoder.initial_angular_position = left_encoder.current_angular_position;
				left_motor_state = RUNNING;
			}

			left_encoder.filtered_relative_angular_position = (left_encoder.absolute_angular_position - left_encoder.initial_angular_position);
			
			current_position.distance_travelled_left_wheel = ((left_encoder.filtered_relative_angular_position)/revolution*2*PI*WHEEL_RADIUS)/10;
			float distance_error = left_distance_reference - current_position.distance_travelled_left_wheel;

			if(left_distance_reference != 0 && distance_error != 0) {
				current_frequency = motor_speed_control(distance_error, current_frequency, &leftPID);
				timerAlarmEnable(left_stepper_timer); 

				if (forward_flag && left_motor_enabled && right_motor_enabled) {
					float driftError = current_position.distance_travelled_left_wheel - current_position.distance_travelled_right_wheel;
					float kP = KP;
					if (driftError > 0) {
						correction = driftError * kP;
					}
				}
				else if (turn_flag && left_motor_enabled && right_motor_enabled) {
					float driftError = -(current_position.distance_travelled_left_wheel + current_position.distance_travelled_right_wheel);
					float kP = KP;
					if (driftError > 0) {
						correction = driftError * kP;
					}
				}
				else {
					correction = 0;
				}

				float corrected_frequency = current_frequency - correction;
				corrected_frequency = constrain(corrected_frequency, minimum_frequency, maximum_frequency);

				if (previous_frequency != corrected_frequency) {
					previous_frequency = corrected_frequency;
					timerAlarmWrite(left_stepper_timer, 1000000 / corrected_frequency, true);
				}	

				if(distance_error > 0 && abs(distance_error) > ACCEPTABLE_ERROR){
					digitalWrite(DIR_PIN_LEFT, CLOCKWISE);
					vTaskDelay(1 / portTICK_PERIOD_MS); 
				}
				else if (distance_error < 0 && abs(distance_error) > ACCEPTABLE_ERROR){
					digitalWrite(DIR_PIN_LEFT, ANTI_CLOCKWISE);
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				else {
					timerAlarmDisable(left_stepper_timer);
					left_distance_reference = 0;
					correction = 0;
					//left_motor_enabled = false;
					left_encoder.initialized = false;
					left_motor_state = DONE;
					vTaskDelay(100 / portTICK_PERIOD_MS); 
				}
			}
			else {
				timerAlarmDisable(left_stepper_timer);
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}	
		}		
		else {
			timerAlarmDisable(left_stepper_timer);
			if (!left_motor_enabled && !right_motor_enabled && forward_flag) {
				forward_flag = false;
			}
			left_motor_state = IDLE;
			vTaskDelay(10 / portTICK_PERIOD_MS); 
		}

	}

}

float motor_speed_control(float error, float currentFreq, PIDController* pid) {

    unsigned long currentTime = millis();
    float dt = (currentTime - pid->previousTime) / 1000.0;
	if (dt <= 0.0f) dt = 0.001f;
    pid->previousTime = currentTime;

    error = abs(error);
    pid->integral += error * dt;
	//pid->integral = constrain(pid->integral, -1000.0f, 1000.0f);
    float derivative = (error - pid->previousError) / dt;
    pid->previousError = error;

    currentFreq = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    return currentFreq;
}

float motor_rotation_speed_control(float error, float currentFreq) {	
		
		float Kp = 100.0;
		float Ki = 0.25;
		float Kd = 0.1;

		static unsigned long previousTime = 0;
		static float integral = 0;
		static float previousError = 0;
		float derivative = 0;

		unsigned long currentTime = millis();
		float dt = (currentTime - previousTime) / 1000.0;
		previousTime = currentTime;

		error = abs(error);
		integral += error * dt;
		derivative = (error - previousError) / dt;
		previousError = error;

		currentFreq = Kp * error + Ki * integral + Kd * derivative;
		currentFreq = constrain(currentFreq, minimum_frequency, maximum_frequency);

		return currentFreq;

}

