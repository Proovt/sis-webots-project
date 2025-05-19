#pragma once 


#include "utils/log_data.hpp"


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"

#define RAD2DEG(X)      X / M_PI * 180.0 // Convert radians to degrees

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

/* CONSTANTS */
/* - */

/* VARIABLES */
static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;

/* VERBOSE_FLAGS */
#define VERBOSE_ODO_ENC false     	// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC false    	// Print odometry values computed with accelerometer

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */
void odo_compute_acc(pose_t* odo, const double acc[6], const double acc_mean[6], double delta_time, std::string fname, int fcols, double time)
{
	// Remove bias
	double gyro_normalized_z = acc[5] - acc_mean[5];
	double acc_normalized_x = acc[0] - acc_mean[0];
	double acc_normalized_y = acc[1] - acc_mean[1];

	// Adjust heading with gyroscope
	_odo_pose_acc.heading += gyro_normalized_z * delta_time;

	// Compute the acceleration in world frame (Assume 2-D motion)
	double acc_wx = cos(_odo_pose_acc.heading) * acc_normalized_x - sin(_odo_pose_acc.heading) * acc_normalized_y;
	double acc_wy = sin(_odo_pose_acc.heading) * acc_normalized_x + cos(_odo_pose_acc.heading) * acc_normalized_y;

	// Motion model (Integration: Euler method)
	_odo_speed_acc.x += acc_wx * delta_time;
	_odo_pose_acc.x += _odo_speed_acc.x * delta_time;

	_odo_speed_acc.y += acc_wy * delta_time;
	_odo_pose_acc.y += _odo_speed_acc.y * delta_time;

	log_csv(fname, fcols, time, acc[0], acc_wy, acc_mean[0], _odo_speed_acc.x, _odo_pose_acc.x, acc[1], acc_wx, acc_mean[1], _odo_speed_acc.y, _odo_pose_acc.y, _odo_pose_acc.heading);

	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
	
	if(VERBOSE_ODO_ACC)
    {
		printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));
    }
}

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t* odo_speed, double Aleft_enc, double Aright_enc, double delta_time)
{
	// Rad to meter: Convert the wheel encoders units into meters
	Aleft_enc  *= pioneer_info.wheel_radius;
	Aright_enc *= pioneer_info.wheel_radius;

	// Comupute speeds: Compute the forward and the rotational speed
	double omega = ( Aright_enc - Aleft_enc ) / ( pioneer_info.width * delta_time );
	double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * delta_time );

	odo_speed->x = speed;
	odo_speed->heading = omega;
}

/**
 * @brief      Reset the odometry to zeros
 */
void odo_reset()
{
 	memset(&_odo_pose_acc, 0, sizeof(pose_t));

	memset(&_odo_speed_acc, 0, sizeof(pose_t));

	memset(&_odo_pose_enc, 0, sizeof(pose_t));
}