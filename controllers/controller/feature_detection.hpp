#pragma once

#include "kalman.hpp"
#include "serial.hpp"

#define VERBOSE_SIGNAL_RADIUS false // Prints the calculated distance for a given signal strength

#define PROPORTIONALITY_FACTOR 1.08 // [-]
#define SENSOR_NODE_DST 0.723 // [m] distance between light sensor and sensor node
#define SENSOR_NODE_VARIANCE 0.01 // [m^2] (empirical)

void handle_sensor_node(double data[PACKET_SIZE], double signal_strength, Vec &mu, Mat &Sigma)
{
    double d_sqr = PROPORTIONALITY_FACTOR / signal_strength;
    double radius = sqrt(d_sqr - SENSOR_NODE_DST * SENSOR_NODE_DST);

    if (VERBOSE_SIGNAL_RADIUS)
    {
        printf("Radius: %f, Signal strength: %f\n", radius, signal_strength);
    }

    Vec2D last_pos(mu(0), mu(1));
    Vec2D sensor_pos(data[1], data[2]);

    Vec2D diff = last_pos - sensor_pos;
    diff.normalize();

    Vec2D bias = Vec2D::Ones() * .01;
    bias[0] *= cos(mu(2));
    bias[1] *= sin(mu(2));

    Vec2D estimated_pos = sensor_pos + diff * radius + bias;

    update_step_sensor_node(mu, estimated_pos, Sigma, SENSOR_NODE_VARIANCE);
}