
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
#define SIGMA_V_ENC 0.05 // empirical
#define SIGMA_OMEGA_ENC 10 * SIGMA_GYR // empirical


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
/* void init_kalman_filter() {
    Mat Sigma = Mat::Zero();
    Vec mu = Vec::Zero();

    H << 1, 0, 0,
         0, 1, 0;
} */

void calculate_sigma_u(Mat *Sigma, double sigma_acc_vx, double sigma_acc_vy, double sigma_gyr) {
    *Sigma << sigma_acc_vx * sigma_acc_vx, 0,         0,
                0,       sigma_acc_vy * sigma_acc_vy,   0,
                0,       0,         sigma_gyr * sigma_gyr;
}

void calculate_T(Mat *T, double heading) {
    *T << cos(heading), -sin(heading),         0,
          sin(heading),       cos(heading),   0,
          0,       0,         1;
}

void prediction_step_acc(Vec *mu, Mat *Sigma, pose_t *odo_speed_acc, double delta_time) {
    // initialize Sigma_u
    Mat Sigma_u;
    calculate_sigma_u(&Sigma_u, sigma_acc_v, sigma_acc_v, SIGMA_GYR);

    Vec u(odo_speed_acc->x, odo_speed_acc->y, odo_speed_acc->heading);
    
    Mat T;
    // initialize T
    calculate_T(&T, (*mu)(2));

    Mat B = T * delta_time;

    Mat R = B * Sigma_u * B.transpose();

    (*mu) = (*mu) + B * u;

    (*Sigma) = (*Sigma) + R;


    sigma_acc_v += SIGMA_ACC * delta_time;
}

void prediction_step_enc(Vec *mu, Mat *Sigma, pose_t *odo_speed, double delta_time) {
    // initialize Sigma_u
    Mat Sigma_u;
    calculate_sigma_u(&Sigma_u, SIGMA_V_ENC, 0, SIGMA_OMEGA_ENC);

    // local frame of reference: speed_x = speed, speed_y = 0, angular_speed = omega
    Vec u(odo_speed->x, 0, odo_speed->heading);
    
    Mat T;
    // initialize T
    calculate_T(&T, (*mu)(2));

    Mat B = T * delta_time;

    Mat R = B * Sigma_u * B.transpose();

    (*mu) = (*mu) + B * u;

    (*Sigma) = (*Sigma) + R;
}

/* void extended_kalman_filter_enc() {
    

} */