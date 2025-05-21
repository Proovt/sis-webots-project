#pragma once

#include "pioneer_interface/pioneer_interface.hpp"

#define PACKET_SIZE 5 // Number of doubles in a packet

/**
 * @brief      Print an array of doubles in the terminal
 * @param[in]  array The array
 */
void print_array(const double *array)
{
    printf("[");
    for (int i = 0; i < PACKET_SIZE; i++)
    {
        printf("%.3lf, ", array[i]);
    }
    printf("]\n");
}

/**
 * @brief      Get a message from the serial port
 * @param[in]  robot The robot object
 * @param[out] data  The data destination
 * @return     signal strength of the received packet, 0.0 if no packet was received
 */
double serial_get_data(Pioneer &robot, double *data)
{
    if (robot.serial_get_queue_length() == 0)
        return 0.0;

    const double *robot_data = robot.serial_read_msg();

    memcpy(data, robot_data, PACKET_SIZE * sizeof(double));

    double signal_strength = robot.serial_get_signal_strength();

    robot.serial_next_msg();

    return signal_strength;
}
