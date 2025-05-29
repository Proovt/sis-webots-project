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

/* CONSTANTS */
#define TIME_INIT_ACC 5 // [s] Time for imu static bias collection

/* VERBOSE_FLAGS */
#define VERBOSE_ACC_MEAN false // Prints imu mean values

/* VARIABLES */
static double imu_mean[6] = {0};
static double axis_width; // [m] (empirical)

static MovingAverage acc_x_avg(50), acc_y_avg(50), gyr_z_avg(18);

pose_t measurement;

void odo_init()
{
	axis_width = pioneer_info.width + 0.092;
}

/**
 * @brief      Compute the mean of the 3-axis accelerometer for about TIME_INIT_ACC seconds. The result is stored in array imu_mean
 */
bool controller_compute_mean_acc(double imu[6], float time, double delta_time)
{
	static int count = 0;

	count++;

	// Remove the effects of strong acceleration at the begining
	if (count > 20)
	{
		for (int i = 0; i < 5; i++)
			imu_mean[i] += imu[i];
	}

	if (count == (int)((double)(TIME_INIT_ACC) / delta_time))
	{
		for (int i = 0; i < 5; i++)
			imu_mean[i] /= (double)(count - 20);

		if (VERBOSE_ACC_MEAN)
		{
			printf("Accelerometer initialization Done!\n");
			printf("ROBOT accelerometer mean: %g %g %g, gyroscope mean: %g %g %g\n", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
		}

		return true;
	}

	return false;
}

void odo_compute_acc(pose_t &odo_speed, const double imu[6], double delta_time)
{
	// Remove static bias
	double acc_normalized_x = imu[0] - imu_mean[0];
	double acc_normalized_y = imu[1] - imu_mean[1];
	double gyro_normalized_z = imu[5] - imu_mean[5];

	acc_x_avg.slide(acc_normalized_x);
	acc_y_avg.slide(acc_normalized_y);

	gyr_z_avg.slide(gyro_normalized_z);

	odo_speed.x += acc_x_avg.compute_average() * delta_time;
	odo_speed.y += acc_y_avg.compute_average() * delta_time;
	odo_speed.heading = gyr_z_avg.compute_average();
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
	double omega = (Aright_enc - Aleft_enc) / (axis_width * delta_time);
	double speed = (Aright_enc + Aleft_enc) / (2.0 * delta_time);

	odo_speed.x = speed;
	odo_speed.heading = omega;
}