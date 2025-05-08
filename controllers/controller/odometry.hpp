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

/*CONSTANTS*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter

/*GLOBAL*/
static double _T;

static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC false     	// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC false    	// Print odometry values computed with accelerometer

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */
void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3], std::string fname, int fcols, float time)
{
	// Compute the acceleration in frame A + remove biais (Assume 2-D motion)
	double acc_wx = acc[0] - acc_mean[0];
	// double acc_wy = acc[1] - acc_mean[1];

	// Motion model (Assume 2-D motion)
	_odo_speed_acc.x += acc_wx *_T;
	_odo_pose_acc.x += _odo_speed_acc.x * _T;

	// _odo_speed_acc.y += acc_wy *_T;
	// _odo_pose_acc.y += _odo_speed_acc.y * _T;


	log_csv(fname, fcols, time, acc[0], acc_wx, _odo_speed_acc.x, _odo_pose_acc.x, acc_mean[0]);

	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
	
	if(VERBOSE_ODO_ACC)
    {
		printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));
    }
}

/**
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step)
{
 	memset(&_odo_pose_acc, 0 , sizeof(pose_t));

	memset(&_odo_speed_acc, 0 , sizeof(pose_t));

	memset(&_odo_pose_enc, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
}

// TODO: You can implement your wheel odometry here if relevant for your project

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc)
{
	//  Rad to meter : Convert the wheel encoders units into meters
	
	Aleft_enc  *= WHEEL_RADIUS;

	Aright_enc *= WHEEL_RADIUS;

	
	// Comupute speeds : Compute the forward and the rotational speed
	
	double omega = ( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * _T );

	double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * _T );

	
	//  Compute the speed into the world frame (A) 
	
	double speed_wx = speed * cos(_odo_pose_enc.heading);

	double speed_wy = speed * sin(_odo_pose_enc.heading);

	double omega_w  = omega;


	// Integration : Euler method
	
	_odo_pose_enc.x += speed_wx * _T;

	_odo_pose_enc.y += speed_wy * _T;

	_odo_pose_enc.heading += omega_w * _T;

	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

	if(VERBOSE_ODO_ENC)
    	printf("ODO with wheel encoders : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading) );
}
