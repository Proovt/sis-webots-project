// Implemented by: Linus
#pragma once

#include "pioneer_interface/pioneer_interface.hpp"

#include <iostream>
#include <string>

#define VERBOSE_STATE_NAME false

/*

Pioneer sensor layout:

                Front
                3   4
        1   2           5   6
    0                           7
Left                             Right
    15                          8
        14  13          10  9
                12  11
                 Back

Response: >5m -> 0, 0m -> 1024
*/

/**
 * @brief      This function implements the Braitenberg algorithm
 *              to control the robot's velocity.
 * @param      ps           Proximity sensor readings (NUM_SENSORS values)
 * @param      vel_left     The left velocity
 * @param      vel_right    The right velocity
 */

void braitenberg(double *ps, double &vel_left, double &vel_right)
{

    double braitenberg_coefficients[16][2] = {
        {0.1, -0.0}, // 0: slight steer
        {0.2, -0.4}, // 1
        {0.2, -0.3}, // 2
        {0.2, -0.3}, // 3
        {-0.3, 0.2}, // 4
        {-0.3, 0.2}, // 5
        {-0.4, 0.2}, // 6
        {-0.0, 0.1}, // 7
        {0.0, 0.0},  // rear sensors ignored
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}};

    const double base_speed = 1.0;

    double influence_sum_l = 0.0;
    double influence_sum_r = 0.0;

    vel_left = base_speed;
    vel_right = base_speed;

    for (int i = 0; i < 8; ++i)
    {
        if (ps[i] < 900)
            ps[i] = 800;                                                // Ignore far sensors
        double influence = pow(((double)ps[i] - 800) / (960 - 800), 4); // Stronger nonlinear amplification

        influence_sum_l += std::abs(braitenberg_coefficients[i][0] * influence);
        influence_sum_r += std::abs(braitenberg_coefficients[i][1] * influence);

        vel_left += braitenberg_coefficients[i][0] * influence;
        vel_right += braitenberg_coefficients[i][1] * influence;
    }

    // Optionally normalize to avoid overcorrection
    if (influence_sum_l > 1.0)
        vel_left /= influence_sum_l;
    if (influence_sum_r > 1.0)
        vel_right /= influence_sum_r;
}