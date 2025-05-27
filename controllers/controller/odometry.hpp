#pragma once

#include "utils/log_data.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"
#include "moving_window.hpp"

#define RAD2DEG(X) X / M_PI * 180.0 // Convert radians to degrees

typedef struct
{
	double x;
	double y;
	double heading;
} pose_t;

static MovingAverage acc_x_avg, acc_y_avg;

void odo_compute_acc(pose_t &odo_speed, const double imu[6], const double imu_mean[6], double delta_time)
{
	// Remove static bias
	double acc_normalized_x = imu[0] - imu_mean[0];
	double acc_normalized_y = imu[1] - imu_mean[1];
	double gyro_normalized_z = imu[5] - imu_mean[5];

	acc_x_avg.slide(acc_normalized_x);
	acc_y_avg.slide(acc_normalized_y);

	odo_speed.x += acc_x_avg.compute_average() * delta_time;
	odo_speed.y += acc_y_avg.compute_average() * delta_time;
	odo_speed.heading = gyro_normalized_z;
}

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t &odo_speed, double Aleft_enc, double Aright_enc, double delta_time)
{
	// Rad to meter: Convert the wheel encoders units into meters
	Aleft_enc *= pioneer_info.wheel_radius;
	Aright_enc *= pioneer_info.wheel_radius;

	// Comupute speeds: Compute the forward and the rotational speed
	double omega = (Aright_enc - Aleft_enc) / ((pioneer_info.width + 0.092) * delta_time);
	double speed = (Aright_enc + Aleft_enc) / (2.0 * delta_time);

	odo_speed.x = speed;
	odo_speed.heading = omega;
}