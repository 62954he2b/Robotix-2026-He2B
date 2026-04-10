#include "odometry.h"
#include "cli_input.h"

Odometry_t robot_state {
    .x = 0.0,
    .y = 0.0,
    .theta = 0.0,
    .current_linear_velocity = 0.0,
    .current_angular_velocity = 0.0
};

void odometry_task(void *parameter){
    
    static float previous_left = 0.0f;
    static float previous_right = 0.0f;

    while (1) {

        float curr_left  = current_position.absolute_distance_travelled_left_wheel;
        float curr_right = current_position.absolute_distance_travelled_right_wheel;

        float delta_left  = curr_left  - previous_left;
        float delta_right = curr_right - previous_right;

        previous_left  = curr_left;
        previous_right = curr_right;

        odometry_update(&robot_state, delta_left, delta_right, WHEEL_BASE_MM/10);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

void odometry_update(Odometry_t *odom, float delta_left, float delta_right, float wheel_base)
{
    float delta_s = (delta_left + delta_right) / 2.0f;
    float delta_theta = (delta_right - delta_left) / wheel_base;

    float theta_mid = odom->theta + delta_theta / 2.0f;  //Odometrie point milieu
    odom->x += delta_s * ( cosf(theta_mid));
    odom->y += delta_s * ( sinf(theta_mid));

    // //Odométrie classique
    // odom->x += delta_s * (cosf(odom->theta));
    // odom->y += delta_s * (sinf(odom->theta));

    odom->theta += delta_theta;

    if (odom->theta > PI)  odom->theta -= 2 * PI;
    if (odom->theta < -PI) odom->theta += 2 * PI;

    odom->current_angular_velocity = (right_encoder.filtered_linear_velocity - left_encoder.filtered_linear_velocity) / wheel_base;
    odom->current_linear_velocity = (left_encoder.filtered_linear_velocity + right_encoder.filtered_linear_velocity) / 2.0f;


}