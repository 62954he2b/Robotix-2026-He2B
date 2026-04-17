#include "rotary_encoder_AS5048.h"

portMUX_TYPE LeftEncoderMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE RightEncoderMutex = portMUX_INITIALIZER_UNLOCKED;

EncoderAS5048 left_encoder {
    .rise_time = 0,
    .last_rise_time = 0,
    .high_time = 0,
    .period_us = 0,
    .sampled = false,

    .median_buf = {0},
    .average_buf = {0}, 
    .median_index = 0,
    .average_index = 0,

    .initial_angular_position = 0.0,
    .absolute_angular_position = 0.0,
    .current_angular_position = 0.0,
    .revolution_counter = 0,
    .angular_velocity = 0.0,
    .filtered_absolute_angular_position = 0.0,
    .previous_filtered_absolute_angular_position = 0.0,
    .filtered_relative_angular_position = 0.0,
    .filtered_angular_velocity = 0.0,
    .previous_filtered_angular_velocity = 0.0,
    .filtered_linear_velocity = 0.0,
    .velocity_reference = 0.0,

    .initialized = false  
};

EncoderAS5048 right_encoder {
    .rise_time = 0,
    .last_rise_time = 0,
    .high_time = 0,
    .period_us = 0,
    .sampled = false,

    .median_buf = {0},
    .average_buf = {0}, 
    .median_index = 0,
    .average_index = 0,

    .initial_angular_position = 0.0,
    .absolute_angular_position = 0.0,
    .current_angular_position = 0.0,
    .revolution_counter = 0,
    .angular_velocity = 0.0,
    .filtered_absolute_angular_position = 0.0,
    .previous_filtered_absolute_angular_position = 0.0,
    .filtered_relative_angular_position = 0.0,
    .filtered_angular_velocity = 0.0,
    .previous_filtered_angular_velocity = 0.0,
    .filtered_linear_velocity = 0.0,
    .velocity_reference = 0.0,

    .initialized = false
};

void left_encoder_reading_task(void *parameter) {

    static float initialAngularPosition = 0.0f;
    static float previousAngularPosition = 0.0f;
    static bool initialized = false;
    bool new_sample = false;
    static uint32_t current_time = 0;
    static uint32_t previous_time = 0;

    // Etat local du filtre
    static float median_buf[MEDIAN_SIZE] = {0.0f};
    static float average_buf[AVG_SIZE] = {0.0f};
    static int median_index = 0;
    static int average_index = 0;
    static float continuousRawAngle = 0.0f;
    static float filteredContinuousAngle = 0.0f;

    while(1){

        portENTER_CRITICAL(&LeftEncoderMutex);
        uint32_t h = left_encoder.high_time;
        uint32_t p = left_encoder.period_us;
        new_sample = left_encoder.sampled;
        left_encoder.sampled = false;
        portEXIT_CRITICAL(&LeftEncoderMutex);

        if (!new_sample) {
            vTaskDelay(1);
            continue;
        }

        if (new_sample) {

            if (p < 500 || p > 5000) goto delay;
            if (h > p) goto delay;
            if (p == 0) goto delay;

            float duty = (float)h / (float)p;
            float rawAngle = duty * 360.0f;

            if (!initialized) {
                initialAngularPosition = rawAngle;
                previousAngularPosition = rawAngle;
                continuousRawAngle = rawAngle;
                filteredContinuousAngle = rawAngle;

                for (int i = 0; i < MEDIAN_SIZE; i++) {
                    median_buf[i] = rawAngle;
                }
                for (int i = 0; i < AVG_SIZE; i++) {
                    average_buf[i] = rawAngle;
                }

                initialized = true;
                previous_time = micros();

                portENTER_CRITICAL(&LeftEncoderMutex);
                left_encoder.current_angular_position = rawAngle;
                portEXIT_CRITICAL(&LeftEncoderMutex);

                goto delay;
            }

            // 1) Unwrap du raw angle pour obtenir un angle continu
            float deltaRaw = rawAngle - previousAngularPosition;

            if (deltaRaw > 180.0f) {
                deltaRaw -= 360.0f;
            }
            else if (deltaRaw < -180.0f) {
                deltaRaw += 360.0f;
            }

            if (fabsf(deltaRaw) > DELTA_MAX_ALLOWED) {
                previousAngularPosition = rawAngle;
                previous_time = micros();
                goto delay;
            } 

            continuousRawAngle += deltaRaw;
            previousAngularPosition = rawAngle;

            // 2) Filtre médian sur l’angle continu
            median_buf[median_index] = continuousRawAngle;
            median_index++;
            if (median_index >= MEDIAN_SIZE) median_index = 0;

            float temp[MEDIAN_SIZE];
            memcpy(temp, median_buf, sizeof(temp));

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

            // 3) Moyenne glissante
            average_buf[average_index] = median;
            average_index++;
            if (average_index >= AVG_SIZE) average_index = 0;

            float sum = 0.0f;
            for (int i = 0; i < AVG_SIZE; i++) {
                sum += average_buf[i];
            }
            float averaged = sum / AVG_SIZE;

            // 4) Filtre IIR
            const float alpha = 0.1f;
            filteredContinuousAngle =
                alpha * averaged + (1.0f - alpha) * filteredContinuousAngle;

            // Position absolue gauche à partir de l’angle filtré
            float filteredAbsoluteAngle = (filteredContinuousAngle - initialAngularPosition);

            current_time = micros();
            float dt = (current_time - previous_time) / 1000000.0f;

            float filteredAngularVelocity = 0.0f;
            float filteredLinearVelocity = 0.0f;
            float previousFilteredAbsolute = left_encoder.previous_filtered_absolute_angular_position;

            if (dt > 1e-6f) {
                filteredAngularVelocity = ((filteredAbsoluteAngle - previousFilteredAbsolute) / dt) * PI / 180.0f;
                filteredLinearVelocity = filteredAngularVelocity * WHEEL_RADIUS_MM / 1000.0f;
            }

            portENTER_CRITICAL(&LeftEncoderMutex);
            left_encoder.current_angular_position = rawAngle;
            left_encoder.absolute_angular_position = filteredAbsoluteAngle;
            left_encoder.filtered_absolute_angular_position = filteredAbsoluteAngle;
            left_encoder.filtered_angular_velocity = filteredAngularVelocity;
            left_encoder.filtered_linear_velocity = filteredLinearVelocity;
            left_encoder.previous_filtered_absolute_angular_position = filteredAbsoluteAngle;
            left_encoder.previous_filtered_angular_velocity = filteredAngularVelocity;
            portEXIT_CRITICAL(&LeftEncoderMutex);

            previous_time = current_time;
            
        }
        delay:
            vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void right_encoder_reading_task(void *parameter) {

    static float initialAngularPosition = 0.0f;
    static float previousAngularPosition = 0.0f;
    static bool initialized = false;
    bool new_sample = false;
    static uint32_t current_time = 0;
    static uint32_t previous_time = 0;
    int revolutionCounter = 0;

    // Etat local du filtre
    static float median_buf[MEDIAN_SIZE] = {0.0f};
    static float average_buf[AVG_SIZE] = {0.0f};
    static int median_index = 0;
    static int average_index = 0;
    static float continuousRawAngle = 0.0f;
    static float filteredContinuousAngle = 0.0f;

    while(1){

        portENTER_CRITICAL(&RightEncoderMutex);
        uint32_t h = right_encoder.high_time;
        uint32_t p = right_encoder.period_us;
        new_sample = right_encoder.sampled;
        right_encoder.sampled = false;
        portEXIT_CRITICAL(&RightEncoderMutex);

        if (!new_sample) {
            vTaskDelay(1);
            continue;
        }

        if (new_sample) {

            if (p < 500 || p > 5000) goto delay;
            if (h > p) goto delay;
            if (p == 0) goto delay;

            float duty = (float)h / (float)p;
            float rawAngle = duty * 360.0f;

            if (!initialized) {
                initialAngularPosition = rawAngle;
                previousAngularPosition = rawAngle;
                continuousRawAngle = rawAngle;
                filteredContinuousAngle = rawAngle;

                for (int i = 0; i < MEDIAN_SIZE; i++) {
                    median_buf[i] = rawAngle;
                }
                for (int i = 0; i < AVG_SIZE; i++) {
                    average_buf[i] = rawAngle;
                }

                initialized = true;
                previous_time = micros();

                portENTER_CRITICAL(&RightEncoderMutex);
                right_encoder.current_angular_position = rawAngle;
                portEXIT_CRITICAL(&RightEncoderMutex);

                goto delay;
            }

            // 1) Unwrap du raw angle pour obtenir un angle continu
            float deltaRaw = rawAngle - previousAngularPosition;

            if (deltaRaw > 180.0f) {
                deltaRaw -= 360.0f;
            }
            else if (deltaRaw < -180.0f) {
                deltaRaw += 360.0f;
            }

            // Optionnel mais conseillé
            if (fabsf(deltaRaw) > DELTA_MAX_ALLOWED) {
                previousAngularPosition = rawAngle;
                previous_time = micros();
                goto delay;
            } 

            continuousRawAngle += deltaRaw;
            previousAngularPosition = rawAngle;

            // 2) Filtre médian sur l'angle continu
            median_buf[median_index] = continuousRawAngle;
            median_index++;
            if (median_index >= MEDIAN_SIZE) median_index = 0;

            float temp[MEDIAN_SIZE];
            memcpy(temp, median_buf, sizeof(temp));

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

            // 3) Moyenne glissante
            average_buf[average_index] = median;
            average_index++;
            if (average_index >= AVG_SIZE) average_index = 0;

            float sum = 0.0f;
            for (int i = 0; i < AVG_SIZE; i++) {
                sum += average_buf[i];
            }
            float averaged = sum / AVG_SIZE;

            // 4) Filtre IIR
            const float alpha = 0.1f;
            filteredContinuousAngle =
                alpha * averaged + (1.0f - alpha) * filteredContinuousAngle;

            // Position absolue droite à partir de l'angle filtré
            float filteredAbsoluteAngle = -(filteredContinuousAngle - initialAngularPosition);

            current_time = micros();
            float dt = (current_time - previous_time) / 1000000.0f;

            float filteredAngularVelocity = 0.0f;
            float filteredLinearVelocity = 0.0f;
            float previousFilteredAbsolute = right_encoder.previous_filtered_absolute_angular_position;

            if (dt > 1e-6f) {
                filteredAngularVelocity = ((filteredAbsoluteAngle - previousFilteredAbsolute) / dt) * PI / 180.0f;
                filteredLinearVelocity = filteredAngularVelocity * WHEEL_RADIUS_MM / 1000.0f;
            }

            portENTER_CRITICAL(&RightEncoderMutex);
            right_encoder.current_angular_position = rawAngle;
            right_encoder.absolute_angular_position = filteredAbsoluteAngle;
            right_encoder.filtered_absolute_angular_position = filteredAbsoluteAngle;
            right_encoder.filtered_angular_velocity = filteredAngularVelocity;
            right_encoder.filtered_linear_velocity = filteredLinearVelocity;
            right_encoder.previous_filtered_absolute_angular_position = filteredAbsoluteAngle;
            right_encoder.previous_filtered_angular_velocity = filteredAngularVelocity;
            portEXIT_CRITICAL(&RightEncoderMutex);

            previous_time = current_time;
        }

        delay :
            vTaskDelay(10 / portTICK_PERIOD_MS);
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
    float alpha = 0.5;   

    //filtre IIR
    encoder->filtered_absolute_angular_position = alpha * averaged + (1 - alpha) * encoder->filtered_absolute_angular_position;

}

void initialize_encoder(EncoderAS5048 *encoder) {
    encoder->median_index = 0;
    encoder->average_index = 0;

    encoder->initialized = true;
}