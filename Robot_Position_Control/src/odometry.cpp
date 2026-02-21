#include "odometry.h"
#include "cli_input.h"

Odometry_t x_y_position {
    .x = 0,
    .y = 0,
    .theta = 0
};

void odometry_task(void *parameter){
    
    static float prev_left = 0.0f;
    static float prev_right = 0.0f;

    while (1) {

        float curr_left  = current_position.absolute_distance_travelled_left_wheel;
        float curr_right = current_position.absolute_distance_travelled_right_wheel;

        float delta_left  = curr_left  - prev_left;
        float delta_right = curr_right - prev_right;

        prev_left  = curr_left;
        prev_right = curr_right;

        odometry_update(&x_y_position, delta_left, delta_right, LENGTH_BETWEEN_ENCODERS_MM/10);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

void odometry_update(Odometry_t *odom, float delta_left, float delta_right, float wheel_base)
{
    float delta_s = (delta_left + delta_right) / 2.0f;
    float delta_theta = (delta_right - delta_left) / wheel_base;

    float theta_mid = odom->theta + delta_theta / 2.0f;  //Odometrie point milieu
    odom->x += delta_s * (-sinf(theta_mid));
    odom->y += delta_s * ( cosf(theta_mid));

    odom->theta += delta_theta;

    if (odom->theta > PI)  odom->theta -= 2 * PI;
    if (odom->theta < -PI) odom->theta += 2 * PI;

    // //Odométrie classique
    // odom->x += delta_s * (-sinf(odom->theta));
    // odom->y += delta_s * (cosf(odom->theta));
}