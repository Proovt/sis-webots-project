#pragma once

#include "utils/log_data.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"
#include "moving_window.hpp"
#include "kalman.hpp"

#define RAD2DEG(X) X / M_PI * 180.0 // Convert radians to degrees

/* CONSTANTS */
#define STRONG_ACCELERATION_DELAY 20 // [timestep] Timesteps to wait for strong acceleration
#define TIME_INIT_ACC 5				 // [s] Time for imu static bias collection
#define GYRO_Z_IDX 5				 // Index of gyroscope z measurement

/* VERBOSE_FLAGS */
#define VERBOSE_GYRO_MEAN false // Prints imu mean values

/* VARIABLES */
static double gyro_z_bias = 0.0; // [rad/s]
static double axis_width;		 // [m] (empirical)

static MovingAverage gyr_z_avg(20);

void odo_init()
{
	axis_width = pioneer_info.width + 0.092;
}

/**
 * @brief      Compute the mean of the 3-axis accelerometer for about TIME_INIT_ACC seconds. The result is stored in array imu_mean
 */
bool compute_gyro_bias(double imu[6], float time, double delta_time)
{
	static int count = 0;

	count++;

	// Remove the effects of strong acceleration at the begining
	if (count > STRONG_ACCELERATION_DELAY)
	{
		gyro_z_bias += imu[GYRO_Z_IDX];
	}

	if (count == (int)((double)(TIME_INIT_ACC) / delta_time))
	{
		gyro_z_bias /= (double)(count - STRONG_ACCELERATION_DELAY);

		if (VERBOSE_GYRO_MEAN)
		{
			printf("Gyroscope bias computation: Done!\n");
			printf("Gyroscope z mean: %g\n", gyro_z_bias);
		}

		return true;
	}

	return false;
}

double odo_compute_gyroscope(const double imu[6], double delta_time)
{
	// Remove static bias
	double gyro_normalized_z = imu[GYRO_Z_IDX] - gyro_z_bias;

	gyr_z_avg.slide(gyro_normalized_z);

	return gyr_z_avg.compute_average();
}

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(Vec2D &odo_speed, double Aleft_enc, double Aright_enc, double delta_time)
{
	// Rad to meter: Convert the wheel encoders units into meters
	Aleft_enc *= pioneer_info.wheel_radius;
	Aright_enc *= pioneer_info.wheel_radius;

	// Comupute speeds: Compute the forward and the rotational speed
	double omega = (Aright_enc - Aleft_enc) / (axis_width * delta_time);
	double speed = (Aright_enc + Aleft_enc) / (2.0 * delta_time);

	odo_speed << speed, omega;
}