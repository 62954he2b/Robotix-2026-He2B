#include "rotary_encoder_AS5048.h"

portMUX_TYPE encoderMutex = portMUX_INITIALIZER_UNLOCKED;

 EncoderAS5048 left_encoder {
    .rise_time = 0,
    .last_rise_time = 0,
    .high_time = 0,
    .period_us = 0,
    .new_sample = false,

    .median_buf = {0},
    .average_buf = {0}, 

    .median_index = 0,
    .average_index = 0,

    .initial_angular_position = 0.0,
    .absolute_angular_position = 0.0,
    .current_angular_position = 0.0,
    .revolution_counter = 0,
    .filtered_absolute_angular_position = 0.0,
    .filtered_relative_angular_position = 0.0,

    .initialized = false  
};

EncoderAS5048 right_encoder {
    .rise_time = 0,
    .last_rise_time = 0,
    .high_time = 0,
    .period_us = 0,
    .new_sample = false,

    .median_buf = {0},
    .average_buf = {0}, 

    .median_index = 0,
    .average_index = 0,

    .initial_angular_position = 0.0,
    .absolute_angular_position = 0.0,
    .current_angular_position = 0.0,
    .revolution_counter = 0,
    .filtered_absolute_angular_position = 0.0,
    .filtered_relative_angular_position = 0.0,

    .initialized = false
};

void left_encoder_reading_task(void *parameter) {

    static float previousAngularPosition = 0;
    static bool initialized = false;

    while(1){

         if (!left_encoder.new_sample) {
            vTaskDelay(1);
            continue;
        }

        if (left_encoder.new_sample) {
        portENTER_CRITICAL(&encoderMutex);
        uint32_t h = left_encoder.high_time;
        uint32_t p = left_encoder.period_us;
        left_encoder.new_sample = false;
        portEXIT_CRITICAL(&encoderMutex);

        if (p < 500 || p > 5000) goto delay;
        if (h > p) goto delay;

        if (p > 0) {
            float duty = (float)h / (float)p;
            float rawAngle = duty * 360.0;
            left_encoder.current_angular_position = rawAngle;

            if (!initialized) {
                    previousAngularPosition = rawAngle;
                    initialized = true;
                    goto delay;
            }

            // Unwrap angle (continu)
            float delta = left_encoder.current_angular_position - previousAngularPosition;
            if (delta > 300.0f) {
               left_encoder.revolution_counter--;
            }
            else if (delta < -300.0f) {
                left_encoder.revolution_counter++;
            }

            previousAngularPosition = left_encoder.current_angular_position;

            left_encoder.absolute_angular_position = left_encoder.current_angular_position + left_encoder.revolution_counter * 360.0f;
            filter_angle_value(&left_encoder);

            }
        }
        delay :
            vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void right_encoder_reading_task(void *parameter) {

    const TickType_t period = pdMS_TO_TICKS(1);
    TickType_t lastWakeTime = xTaskGetTickCount();
    static float previousAngularPosition = 0;
    static bool initialized = false;

    while(1){

        if (!right_encoder.new_sample) {
            vTaskDelay(1);
            continue;
        }

        if (right_encoder.new_sample) {
        portENTER_CRITICAL(&encoderMutex);
        uint32_t h = right_encoder.high_time;
        uint32_t p = right_encoder.period_us;
        right_encoder.new_sample = false;
        portEXIT_CRITICAL(&encoderMutex);

        if (p < 500 || p > 5000) goto delay;
        if (h > p) goto delay;

        if (p > 0) {
            float duty = (float)h / (float)p;
            float rawAngle = duty * 360.0;
            right_encoder.current_angular_position = rawAngle;

            if (!initialized) {
                    previousAngularPosition = rawAngle;
                    initialized = true;
                    goto delay;
            }

            // Unwrap angle (continu)
            float delta = right_encoder.current_angular_position - previousAngularPosition;
            if (delta > 300.0f) {
               right_encoder.revolution_counter--;
            }
            else if (delta < -300.0f) {
                right_encoder.revolution_counter++;
            }

            previousAngularPosition = right_encoder.current_angular_position;

            right_encoder.absolute_angular_position = right_encoder.current_angular_position + right_encoder.revolution_counter * 360.0f;
            
            filter_angle_value(&right_encoder);
            }
        }
        delay :
            vTaskDelayUntil(&lastWakeTime, period);
    }
}


void filter_angle_value(EncoderAS5048 *encoder) {
    
    //Filtre médian
    encoder->median_buf[encoder->median_index] = encoder->absolute_angular_position;
    (encoder->median_index)++;
    if (encoder->median_index >= MEDIAN_SIZE) encoder->median_index = 0;

    float temp[MEDIAN_SIZE];

    memcpy(temp, encoder->median_buf, sizeof(temp));

    for (int i = 0; i < MEDIAN_SIZE - 1; i++) {
        for (int j = i + 1; j < MEDIAN_SIZE; j++) {
            if (temp[j] < temp[i]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    float median = temp[MEDIAN_SIZE / 2];

    //Moyenne Glissante
    encoder->average_buf[encoder->average_index] = median;
    encoder->average_index++;
    if (encoder->average_index >= AVG_SIZE) encoder->average_index = 0;

    float sum = 0.0;
    for (int i = 0; i < AVG_SIZE; i++) sum += encoder->average_buf[i];

    float averaged = sum / AVG_SIZE;
    float alpha = 0.1;   

    //filtre IIR    
    encoder->filtered_absolute_angular_position = alpha * averaged + (1 - alpha) * encoder->filtered_absolute_angular_position;

}



void initialize_encoder(EncoderAS5048 *encoder) {
    encoder->median_index = 0;
    encoder->average_index = 0;

    //encoder->revolution_counter = 0;
    encoder->initialized = true;
}