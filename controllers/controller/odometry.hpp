// Implemented by: Nico
#pragma once

#include "utils/log_data.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"
#include "moving_average.hpp"
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

/**
 * @brief      Initializes odometry subsystem
 */
void odo_init()
{
	axis_width = pioneer_info.width + 0.092;
}

/**
 * @brief      Computes gyroscope bias over time for about TIME_INIT_ACC seconds
 * @param[in]  imu        	 IMU sensor array
 * @param[in]  time       	 Current timestamp
 * @param[in]  delta_time    Time step
 * @return     True if bias estimation is complete
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

		printf("Gyroscope bias computation: Done!\n");

		if (VERBOSE_GYRO_MEAN)
		{
			printf("Gyroscope z mean: %g\n", gyro_z_bias);
		}

		return true;
	}

	return false;
}

/**
 * @brief      Computes angular velocity from gyroscope data
 * @param[in]  imu       	IMU sensor array
 * @param[in]  delta_time   Time difference
 * @return     Angular velocity (z-axis)
 */
double odo_compute_gyroscope(const double imu[6], double delta_time)
{
	// Remove static bias
	double gyro_normalized_z = imu[GYRO_Z_IDX] - gyro_z_bias;

	gyr_z_avg.slide(gyro_normalized_z);

	return gyr_z_avg.compute_average();
}

/**
 * @brief      Computes robot velocity based on encoder values
 * @param[out] odo_speed  	 Vector to store linear and angular speed
 * @param[in]  Aleft_enc     Left wheel encoder difference
 * @param[in]  Aright_enc    Right wheel encoder difference
 * @param[in]  delta_time    Time difference
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