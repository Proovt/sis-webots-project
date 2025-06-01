// Implemented by: Nico
#pragma once

#include <math.h>
#include <memory.h>

#include "odometry.hpp"

///////////////////
// Eigen library //     DO NOT MODIFY THIS PART
///////////////////

#include <Eigen/Dense>

/* CONSTANTS */
#define DIM 3                           // State dimension
#define SIGMA_GYR 0.025                 // [rad/s]
#define SIGMA_V_ENC 0.05                // [m/s] (empirical)
#define SIGMA_OMEGA_ENC 2.0 * SIGMA_GYR // [rad/s] (empirical)

using namespace std;

typedef Eigen::Matrix<double, DIM, DIM> Mat; // DIMxDIM matrix
typedef Eigen::Matrix<double, -1, -1> MatX;  // Arbitrary size matrix
typedef Eigen::Matrix<double, DIM, 1> Vec;   // DIMx1 column vector
typedef Eigen::Matrix<double, -1, 1> VecX;   // Arbitrary size column vector
typedef Eigen::Matrix<double, 2, 1> Vec2D;   // 2D column vector

static const Mat I = MatX::Identity(DIM, DIM); // DIMxDIM identity matrix

//////////////////////////////////
// Kalman filter base functions //    DO NOT MODIFY THIS PART
//////////////////////////////////

// State vector mu (x,y,heading) to be updated by the Kalman filter functions
// static Vec mu = Vec::Zero();
// State covariance sigma to be updated by the Kalman filter functions
// static Mat sigma = Mat::Zero();

static Mat Sigma_u = Mat::Zero();

static double var_omega_enc = 0;
static double heading_enc = 0;

/**
 * @brief      Get the dimensionality of the Kalman state vector
 * @return     Number of dimensions (usually 3)
 */
int kal_get_dim()
{
    return DIM;
}

/**
 * @brief      Extracts pose [x, y, heading] from Kalman state vector
 * @param[in]  mu     State vector
 * @param[out] state  Output array to store pose
 */
void kal_get_state(Vec &mu, double state[3])
{
    for (int i = 0; i < DIM; i++)
    {
        state[i] = mu(i);
    }
}

/**
 * @brief      Checks matrix for NaN values
 * @param[in]  m  Matrix to check
 * @return     True if any value is NaN
 */
bool kal_check_nan(const MatX &m)
{
    for (int i = 0; i < m.rows(); i++)
    {
        for (int j = 0; j < m.cols(); j++)
        {
            if (isnan(m(i, j)))
            {
                printf("FATAL: matrix has NaN values, exiting...\n");
                return true;
            }
        }
    }
    return false;
}

///////////////////////////////////////////////////
// TODO: implement your Kalman filter here after //
///////////////////////////////////////////////////
/**
 * @brief      Initializes Kalman filter parameters
 */
void kalman_init()
{
    Sigma_u << SIGMA_V_ENC * SIGMA_V_ENC, 0, 0,
        0, 0, 0,
        0, 0, SIGMA_GYR * SIGMA_GYR;
}

/**
 * @brief      Calculates transformation matrix T based on heading
 * @param[out] T        Output transformation matrix
 * @param[in]  heading  Current robot heading
 */
void calculate_T(Mat &T, double heading)
{
    T << cos(heading), -sin(heading), 0,
        sin(heading), cos(heading), 0,
        0, 0, 1;
}

/* Prediction step */
/**
 * @brief      Performs Kalman prediction step with control input
 * @param[in,out] mu         State vector
 * @param[in,out] Sigma      Covariance matrix
 * @param[in]     Sigma_u    Process noise matrix
 * @param[in]     u          Control input
 * @param[in]     delta_time Time difference since last update
 */
void prediction_step(Vec &mu, Mat &Sigma, const Mat &Sigma_u, const Vec &u, double delta_time)
{
    // Process matrix
    Mat F;

    // (- sin(heading) * v_x - cos(heading) * v_y) * dt
    F << 1, 0, -(sin(mu(2)) * u(0) + cos(mu(2)) * u(1)) * delta_time,
        // (cos(heading) * v_x - sin(heading) * v_y) * dt
        0, 1, (cos(mu(2)) * u(0) - sin(mu(2)) * u(1)) * delta_time,
        0, 0, 1;

    Mat T;
    // initialize T
    calculate_T(T, mu(2));

    Mat G = T * delta_time;

    Mat R = G * Sigma_u * G.transpose();

    mu = mu + G * u;

    Sigma = F * Sigma * F.transpose() + R;
}

/**
 * @brief      Updates the encoder-derived heading and its variance
 * @param[in]  omega_enc     Angular velocity from encoders
 * @param[in]  delta_time    Time step duration
 */
void prediction_step_heading_encoder(double omega_enc, double delta_time)
{
    heading_enc += omega_enc * delta_time;
    var_omega_enc += delta_time * SIGMA_OMEGA_ENC * SIGMA_OMEGA_ENC * delta_time;
}

/* Update step */
/**
 * @brief      Generic update step with measurement and Jacobians
 * @param[in,out] mu          State vector
 * @param[in]     measurement Observed measurement
 * @param[in,out] Sigma       Covariance matrix
 * @param[in]     Q           Measurement noise covariance
 * @param[in]     H           Measurement matrix
 */
void update_step(Vec &mu, const Vec2D &measurement, Mat &Sigma, const MatX &Q, const MatX &H)
{
    MatX K = Sigma * H.transpose() * (H * Sigma * H.transpose() + Q).inverse();

    if (kal_check_nan(K))
        return;

    mu = mu + K * (measurement - H * mu);
    Sigma = (I - K * H) * Sigma;
}

/**
 * @brief      Updates state using sensor fusion step
 * @param[in,out] mu     State vector
 * @param[in,out] Sigma  Covariance matrix
 */
void update_step_sensors(Vec &mu, Mat &Sigma)
{
    double K = Sigma(2, 2) / (Sigma(2, 2) + var_omega_enc);

    if (isnan(K))
        return;

    mu(2) += K * (heading_enc - mu(2));
    Sigma(2, 2) -= K * Sigma(2, 2);
}

/**
 * @brief      Updates state with sensor node data (e.g., x/y)
 * @param[in,out] mu          State vector
 * @param[in]     measurement Sensor node 2D position measurement
 * @param[in,out] Sigma       Covariance matrix
 * @param[in]     sensor_var  Variance of sensor measurement
 */
void update_step_sensor_node(Vec &mu, Vec2D &measurement, Mat &Sigma, double sensor_var)
{
    MatX H(2, DIM);
    H << 1, 0, 0,
        0, 1, 0;

    MatX Q(2, 2);
    Q << sensor_var, 0,
        0, sensor_var;

    update_step(mu, measurement, Sigma, Q, H);
}