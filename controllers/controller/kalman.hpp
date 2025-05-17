
#include <math.h>
#include <memory.h>

#include "odometry.hpp"

///////////////////
// Eigen library //     DO NOT MODIFY THIS PART
///////////////////

#include <Eigen/Dense>


/* CONSTANTS */
#define DIM 3                                       // State dimension 
#define SIGMA_ACC 0.05 // m/s^2
#define SIGMA_GYR 0.025 // rad/s
#define SIGMA_V 0.05 // empirical
#define SIGMA_OMEGA 10 * SIGMA_GYR // empirical


typedef Eigen::Matrix<double,DIM,DIM>   Mat;        // DIMxDIM matrix  
typedef Eigen::Matrix<double, -1, -1>   MatX;       // Arbitrary size matrix 
typedef Eigen::Matrix<double,DIM,  1>   Vec;        // DIMx1 column vector  
typedef Eigen::Matrix<double, -1,  1>   VecX;       // Arbitrary size column vector  

static const Mat I = MatX::Identity(DIM,DIM);       // DIMxDIM identity matrix  

//////////////////////////////////
// Kalman filter base functions //    DO NOT MODIFY THIS PART
//////////////////////////////////

// State vector mu (x,y,heading) to be updated by the Kalman filter functions
static Vec mu = Vec::Zero();
// State covariance sigma to be updated by the Kalman filter functions
static Mat sigma = Mat::Zero();

static MatX H(2,3);

static double sigma_acc_v = 0;

/**
 * @brief      Get the state dimension 
*/
int kal_get_dim(){
    return DIM;
}

/**
 * @brief      Copy the state vector into a 1D array
*/
void kal_get_state(double* state){
    for(int i=0; i<DIM; i++){
        state[i] = mu(i);
    }
}

/**
 * @brief      Copy the state covariance matrix into a 2D array
*/
void kal_get_state_covariance(double** cov){
    for(int i=0;i<sigma.rows();i++){
        for(int j=0; j<sigma.cols(); j++){
            cov[i][j] = sigma(i,j);
        }
    }
}

/**
 * @brief      Check if a matrix contains any NaN values 
*/
bool kal_check_nan(const MatX& m){
    for(int i=0;i<m.rows();i++){
        for(int j=0; j<m.cols(); j++){
            if(isnan(m(i,j))){
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
void init_kalman_filter() {
    H << 1, 0, 0,
         0, 1, 0;
}

void calculate_sigma_u(Mat Sigma, double sigma_acc_v, double sigma_gyr) {
    double var_acc_v = sigma_acc_v * sigma_acc_v;
    
    Sigma << var_acc_v, 0,         0,
                0,       var_acc_v,   0,
                0,       0,         sigma_gyr * sigma_gyr;
}

void calculate_T(Mat T, double heading) {
    T << cos(heading), -sin(heading),         0,
          sin(heading),       cos(heading),   0,
          0,       0,         1;
}

void prediction_step_acc(Mat Sigma, pose_t odo_pose_acc, pose_t odo_speed_acc, double gyro_z, double heading, double delta_time) {
    // initialize Sigma_u
    Mat Sigma_u;
    calculate_sigma_u(Sigma_u, sigma_acc_v, SIGMA_GYR);

    Vec mu, u;
    mu << odo_pose_acc.x, odo_pose_acc.y, odo_pose_acc.heading;
    u << odo_speed_acc.x, odo_speed_acc.y, gyro_z;
    
    Mat T, R;

    // initialize T
    calculate_T(T, heading);

    Mat B = T * delta_time;

    R = B * Sigma * B.transpose();

    mu = mu + B * u;

    Sigma = Sigma + R;

    sigma_acc_v += SIGMA_ACC * delta_time;
}

void extended_kalman_filter() {
    Mat Sigma = Mat::Zero();
}