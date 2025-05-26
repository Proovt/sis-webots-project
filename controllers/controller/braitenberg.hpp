#pragma once

#include "pioneer_interface/pioneer_interface.hpp"

#include <iostream>
#include <string>

#define VERBOSE_STATE_NAME false
#define VERBOSE_EMERGENCY_BACKUP false
#define VERBOSE_TURN false

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
enum RobotState
{
    NAVIGATE,
    TURNING,
    EMERGENCY_BACKUP,
    STRAIGHT,
    STOP_2
};
const char *getStateName(RobotState state)
{
    switch (state)
    {
    case NAVIGATE:
        return "NAVIGATE";
    case TURNING:
        return "TURNING";
    case EMERGENCY_BACKUP:
        return "EMERGENCY_BACKUP";
    default:
        return "UNKNOWN";
    }
}

void braitenberg(double *ps, double &vel_left, double &vel_right, double pose[4])
{

    // TODO: implement your Braitenberg algorithm here

    static RobotState state = NAVIGATE;
    static RobotState prev_state = NAVIGATE;
    static int backup_counter = 0;
    const int backup_duration = 70;
    static bool is_emergency = false;
    static bool turn_right = false;
    static bool turn_left = true;
    static bool no_turn = true;
    static bool is_end_of_corridor = false;

    /*     double braitenberg_coefficients[16][2] = {
            {-0.6, -0.2}, {-0.5, -0.2}, {-0.4, -0.2}, {-0.3, -0.1},   // Front-left
            {-0.1, -0.3}, {-0.2, -0.4}, {-0.2, -0.5}, {-0.2, -0.6},   // Front-right
            {0.0,  0.0},  {0.0,  0.0},  {0.0,  0.0},  {0.0,  0.0},    // Rear-right
            {0.0,  0.0},  {0.0,  0.0},  {0.0,  0.0},  {0.0,  0.0}};     // Rear-left
     */

    double braitenberg_coefficients[16][2] = {
        {0.0, -0.0}, // 0: slight steer
        {0.2, -0.4}, // 1
        {0.2, -0.3}, // 2
        {0.2, -0.3}, // 3
        {-0.3, 0.2}, // 4
        {-0.3, 0.2}, // 5
        {-0.4, 0.2}, // 6
        {-0.0, 0.0}, // 7
        {0.0, 0.0},  // rear sensors ignored
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}};

    const double base_speed = 1.0;

    // heading variable
    double h = pose[2];
    double x = pose[0];
    double y = pose[1];

    if (x < 1)
    {
        turn_right = true;
        turn_left = false;
    }
    if (x > 5)
    {
        turn_right = false;
        turn_left = true;
    }
    else
    {
        no_turn = true;
    }

    ////////////////////// STOP ///////////////////////////////////////


    bool at_final_stop = (y > 3.5 && y < 4.5 && x < 0);  // adapt to map
    static int stop_counter = 0;
    const int stop_confirm_duration = 150;

    if (at_final_stop) {
        stop_counter++;
        if (stop_counter > stop_confirm_duration) {
            state = STOP_2;
            return;
        }
    } else {
        stop_counter = 0;
    }




    /////////////////turn left ///////////////////////////
    if (turn_left)
    {
        if (is_end_of_corridor)
        {

            if (h > 3)
            {
                is_end_of_corridor = false;
                state = NAVIGATE;
            }
        }
        else
        {
            bool out_cor = true;
            for (int i = 0; i <= 7; ++i)
            {

                if (ps[i] > 850)
                {
                    out_cor = false;
                    break;
                }
            }
            if (out_cor && !at_final_stop)
            {
                state = TURNING;
                is_end_of_corridor = true;
            }
        }
    }

    /////////////////turn right ///////////////////////////
    if (turn_right)
    {
        if (is_end_of_corridor)
        {

            if (h < 0.3)
            {
                is_end_of_corridor = false;
                state = NAVIGATE;
            }
        }
        else
        {
            bool out_cor = true;
            for (int i = 0; i <= 7; ++i)
            {

                if (ps[i] > 850)
                {
                    out_cor = false;
                    break;
                }
            }
            if (out_cor && !at_final_stop)
            {
                state = TURNING;
                is_end_of_corridor = true;
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////

    double influence_sum_l = 0.0;
    double influence_sum_r = 0.0;

    ////////////////emergency///////////////////////////////////

    if (is_emergency)
    {
        backup_counter++;
        state = EMERGENCY_BACKUP;
        if (VERBOSE_EMERGENCY_BACKUP)
        {
            printf("adf %d\n", backup_counter);
        }

        if (backup_counter > backup_duration)
        {
            state = prev_state;
            backup_counter = 0;
            is_emergency = false;
        }
    }
    else
    {
        for (int i = 1; i <= 6; ++i)
        {
            if (ps[i] > 1005)
            {
                prev_state = state;
                state = EMERGENCY_BACKUP;
                is_emergency = true;

                break;
            }
        }
    }

    if (state == TURNING)
    {
        if (x > 0 && x < 5)
        {
            state = NAVIGATE;
            is_end_of_corridor = false;
        }
    }

    /////////////////////////////////////////               /////////////////////////////////////////
    /////////////////////////////////////////     SWITCH    /////////////////////////////////////////
    /////////////////////////////////////////               /////////////////////////////////////////

    switch (state)
    {
    case NAVIGATE:

        vel_left = base_speed;
        vel_right = base_speed;

        for (int i = 0; i < 8; ++i)
        {
            if (ps[i] < 900)
                ps[i] = 800;                                                // Ignore far sensors
            double influence = pow(((double)ps[i] - 800) / (990 - 800), 4); // Stronger nonlinear amplification

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

        // if (vel_left < 0) vel_left = 0;
        // if (vel_right < 0) vel_right = 0;
        break;

    case TURNING:

        if (turn_left)
        {
            if (VERBOSE_TURN)
            {

                printf("\nturnleft");
            }

            double target = 990; // target value for ~0.5m distance
            double sensor_left = std::max(ps[0], ps[15]);
            double error = sensor_left - target;
            double Kp = 0.005;
            double correction = Kp * error;

            if (VERBOSE_TURN)
            {

                printf("\ncorrection = %f\n", correction);
            }

            if (sensor_left < 850)
            {
                vel_left = 0.5;
                vel_right = 0.5;
            }
            else if (ps[0] < ps[15])
            {
                vel_left = 0.2 + correction / 2;
                vel_right = 0.7 - correction;
            }
            else if (ps[0] > ps[15])
            {
                vel_left = 0.3;
                vel_right = 0.5 - correction;
            }
        }

        if (turn_right)
        {
            if (VERBOSE_TURN)
            {

                printf("\nturnright");
            }

            double target = 990; // target value for ~0.5m distance
            double sensor_right = std::max(ps[7], ps[8]);
            double error = sensor_right - target;
            double Kp = 0.005;
            double correction = Kp * error;

            if (VERBOSE_TURN)
            {

                printf("\ncorrection = %f\n", correction);
            }

            if (sensor_right < 850)
            {
                vel_left = 0.5;
                vel_right = 0.5;
            }
            else if (ps[7] < ps[8])
            {
                vel_right = 0.2 + correction;
                vel_left = 0.7 - correction;
            }
            else if (ps[7] > ps[8])
            {
                vel_right = 0.3;
                vel_left = 0.5 - correction;
            }
        }

        break;

    case EMERGENCY_BACKUP:
        vel_left = -0.5;
        vel_right = -0.5;

        break;

    case STRAIGHT:

        vel_left = 0.5;
        vel_right = 0.5;
        break;

    

    case STOP_2:

        vel_left = 0.0;
        vel_right = 0.0;
        break;
    }

    if (VERBOSE_STATE_NAME)
    {
        std::cout << "State name: " << getStateName(state) << std::endl;
    }

    /*     bool danger_close = false;
        for (int i = 0; i <= 7; ++i) {
            if (ps[i] > 1000) danger_close = true;
        }
        if (!danger_close) {
            vel_left += 0.2;
            vel_right += 0.2;
        } */

    // if (vel_left > 1.0) vel_left = 1.0;
    // if (vel_left < 0) vel_left = 0;
    // if (vel_right > 1.0) vel_right = 1.0;
    // if (vel_right < 0) vel_right = 0;
}
