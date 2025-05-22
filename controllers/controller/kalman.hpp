
#include <math.h>
#include <memory.h>

#include "odometry.hpp"

///////////////////
// Eigen library //     DO NOT MODIFY THIS PART
///////////////////

#include <Eigen/Dense>

/* CONSTANTS */
#define DIM 3                          // State dimension
#define SIGMA_ACC 0.05                 // [m/s^2]
#define SIGMA_GYR 0.025                // [rad/s]
#define SIGMA_V_ENC 0.05               // [m/s] (empirical)
#define SIGMA_OMEGA_ENC 10 * SIGMA_GYR // [rad/s] (empirical)

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
static Vec mu = Vec::Zero();
// State covariance sigma to be updated by the Kalman filter functions
static Mat sigma = Mat::Zero();

static MatX H(2, DIM);

static double sigma_acc_v = 0;

/**
 * @brief      Get the state dimension
 */
int kal_get_dim()
{
    return DIM;
}

/**
 * @brief      Copy the state vector into a 1D array
 */
void kal_get_state(double *state)
{
    for (int i = 0; i < DIM; i++)
    {
        state[i] = mu(i);
    }
}

/**
 * @brief      Copy the state covariance matrix into a 2D array
 */
void kal_get_state_covariance(double **cov)
{
    for (int i = 0; i < sigma.rows(); i++)
    {
        for (int j = 0; j < sigma.cols(); j++)
        {
            cov[i][j] = sigma(i, j);
        }
    }
}

/**
 * @brief      Check if a matrix contains any NaN values
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
/* void init_kalman_filter() {
    Mat Sigma = Mat::Zero();
    Vec mu = Vec::Zero();

    H << 1, 0, 0,
         0, 1, 0;
} */

void calculate_sigma_u(Mat &Sigma, double sigma_acc_vx, double sigma_acc_vy, double sigma_gyr)
{
    Sigma << sigma_acc_vx * sigma_acc_vx, 0, 0,
        0, sigma_acc_vy * sigma_acc_vy, 0,
        0, 0, sigma_gyr * sigma_gyr;
}

void calculate_T(Mat &T, double heading)
{
    T << cos(heading), -sin(heading), 0,
        sin(heading), cos(heading), 0,
        0, 0, 1;
}

/* Prediction step */
void prediction_step(Vec &mu, Mat &Sigma, pose_t &odo_speed, Mat &Sigma_u, double delta_time)
{
    // local frame of reference: speed_x = speed, speed_y = 0, angular_speed = omega
    Vec u(odo_speed.x, 0, odo_speed.heading);

    // Process matrix
    Mat F;

    // cos(v_x) * heading * dt
    F << 1, 0, cos(u(0)) * mu(2) * delta_time,
        // sin(v_x) * heading * dt
        0, 1, sin(u(0)) * mu(2) * delta_time,
        0, 0, 1;

    Mat T;
    // initialize T
    calculate_T(T, mu(2));

    Mat G = T * delta_time;

    Mat R = G * Sigma_u * G.transpose();

    mu = mu + G * u;

    Sigma = F * Sigma * F.transpose() + R;
}

void prediction_step_acc(Vec &mu, Mat &Sigma, pose_t &odo_speed_acc, double delta_time)
{
    // initialize Sigma_u
    Mat Sigma_u;
    calculate_sigma_u(Sigma_u, sigma_acc_v, sigma_acc_v, SIGMA_GYR);

    prediction_step(mu, Sigma, odo_speed_acc, Sigma_u, delta_time);

    sigma_acc_v += SIGMA_ACC * delta_time;
}

void prediction_step_enc(Vec &mu, Mat &Sigma, pose_t &odo_speed_enc, double delta_time)
{
    // initialize Sigma_u
    Mat Sigma_u;
    calculate_sigma_u(Sigma_u, SIGMA_V_ENC, 0, SIGMA_OMEGA_ENC);

    prediction_step(mu, Sigma, odo_speed_enc, Sigma_u, delta_time);
}

/* Update step */
void update_step(Vec &mu, Vec2D &measurement, Mat &Sigma, MatX &Q, MatX &C)
{
    MatX K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Q).inverse();
    mu = mu + K * (measurement - C * mu);
    Sigma = (I - K * C) * Sigma;
}

void update_step_sensors(Vec &mu_enc, Vec &mu_acc, Mat &Sigma_enc, Mat &Sigma_acc)
{
    // Only consider gyroscope
    Mat C;
    C << 0, 0, 0,
        0, 0, 0,
        0, 0, 1;
    Mat K = Sigma_enc * C.transpose() * (C * Sigma_enc * C.transpose() + Sigma_acc).inverse();
    // Mat K = Sigma_enc * (Sigma_enc + Sigma_acc).inverse();

    // std::cout << "K: " << std::endl;
    if (kal_check_nan(K))
        return;

    // Print the matrix
    // std::cout << "Matrix K is:\n" << K << std::endl;

    mu = mu + K * (mu_acc - C * mu);
    Sigma_enc = (I - K * C) * Sigma_enc;

    // mu_enc = mu_enc + K * (mu_acc - mu_enc);
    // Sigma_enc = (I - K) * Sigma_enc;
}

void update_step_sensor_node(Vec &mu, Vec2D &measurement, Mat &Sigma, double sensor_var)
{
    MatX C(2, DIM);
    C << 1, 0, 0,
        0, 1, 0;

    MatX Q(2, 2);
    Q << sensor_var, 0,
        0, sensor_var;

    update_step(mu, measurement, Sigma, Q, C);
}

void update_step_wall(Mat &Sigma, Vec &mu, Vec2D &measurement, double sensor_var)
{
    MatX C(2, DIM);
    C << 1, 0, 0,
        0, 1, 0;

    MatX Q(2, 2);
    Q << sensor_var, 0,
        0, sensor_var;

    update_step(mu, measurement, Sigma, Q, C);
}

/* void extended_kalman_filter_enc() {


} */