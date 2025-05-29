#pragma once

#include "kalman.hpp"
#include "serial.hpp"

#define VERBOSE_SIGNAL_RADIUS false // Prints the calculated horizontal distance from the sensor node to the robot for a given signal strength

#define PROPORTIONALITY_FACTOR 1.08  // [-] (empirical)
#define SENSOR_NODE_DST 0.723        // [m] Distance between light sensor and sensor node
#define SENSOR_NODE_BIAS_FACTOR 0.01 // [m] Bias term factor (empirical)
#define SENSOR_NODE_VARIANCE 0.01    // [m^2] (empirical)

static Vec2D last_pos, sensor_pos, last_pos_direction, bias, estimated_pos;

void handle_sensor_node(double data[PACKET_SIZE], double signal_strength, Vec &mu, Mat &Sigma)
{
    double dst_sqr = PROPORTIONALITY_FACTOR / signal_strength;
    double radius = sqrt(dst_sqr - SENSOR_NODE_DST * SENSOR_NODE_DST);

    if (VERBOSE_SIGNAL_RADIUS)
    {
        printf("Horizontal Distance: %f m, Signal strength: %f\n", radius, signal_strength);
    }

    last_pos << mu(0), mu(1);
    sensor_pos << data[1], data[2];

    // Calculate the direction of the last position from the sensor node
    last_pos_direction = last_pos - sensor_pos;
    last_pos_direction.normalize();

    // Calculate bias term in forward direction
    bias << cos(mu(2)), sin(mu(2));
    bias *= SENSOR_NODE_BIAS_FACTOR;

    estimated_pos << sensor_pos + last_pos_direction * radius + bias;

    update_step_sensor_node(mu, estimated_pos, Sigma, SENSOR_NODE_VARIANCE);
}