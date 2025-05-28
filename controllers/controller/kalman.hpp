
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
#define SIGMA_OMEGA_ENC .3 * SIGMA_GYR // [rad/s] (empirical)

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
/* void kal_get_state(double *state)
{
    for (int i = 0; i < DIM; i++)
    {
        state[i] = mu(i);
    }
} */

/**
 * @brief      Copy the state covariance matrix into a 2D array
 */
/* void kal_get_state_covariance(double **cov)
{
    for (int i = 0; i < sigma.rows(); i++)
    {
        for (int j = 0; j < sigma.cols(); j++)
        {
            cov[i][j] = sigma(i, j);
        }
    }
} */

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
    Vec u(odo_speed.x, odo_speed.y, odo_speed.heading);

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
void update_step(Vec &mu, Vec2D &measurement, Mat &Sigma, MatX &Q, MatX &H)
{
    MatX K = Sigma * H.transpose() * (H * Sigma * H.transpose() + Q).inverse();

    if (kal_check_nan(K))
        return;

    mu = mu + K * (measurement - H * mu);
    Sigma = (I - K * H) * Sigma;
}

void update_step_sensors_mat(Vec &mu_enc, Vec &mu_acc, Mat &Sigma_enc, Mat &Sigma_acc)
{
    // Only consider gyroscope (z value) and accelerometer (x and y values)
    Mat H = Mat::Identity();

    // Only consider gyroscope because of imprecise values
    /* Mat H;
    H << 0, 0, 0,
        0, 0, 0,
        0, 0, 1; */

    Mat K = Sigma_enc * H.transpose() * (H * Sigma_enc * H.transpose() + Sigma_acc).inverse();

    if (kal_check_nan(K))
        return;

    mu_enc = mu_enc + K * (mu_acc - H * mu_enc);
    Sigma_enc = (I - K * H) * Sigma_enc;
}

void update_step_sensors(Vec &mu_enc, double heading_acc, Mat &Sigma_enc, double var_heading_acc)
{
    double K = Sigma_enc(2, 2) / (Sigma_enc(2, 2) + var_heading_acc);

    if (isnan(K))
        return;

    mu_enc(2) += K * (heading_acc - mu_enc(2));
    Sigma_enc(2, 2) -= K * Sigma_enc(2, 2);
    // mu_enc(2) = heading_acc;
    // Sigma_enc(2, 2) = var_heading_acc;
}

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