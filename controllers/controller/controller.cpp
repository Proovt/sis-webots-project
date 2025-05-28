// Controller for the robot robot

// Provided libraries
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"

// Files to implement your solutions
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"

/*CONSTANTS*/
#define TIME_INIT_ACC 5 // Time in seconds
#define MIN_SIGNAL_STRENGTH 1.50 // Minimum considered signal strength
#define MAX_SIGNAL_STRENGTH 2.04 // Maximum considered signal strength

/*VERBOSE_FLAGS*/
#define VERBOSE_ACC_MEAN false        // Prints accelerometer mean values
#define VERBOSE_ACC false             // Prints accelerometer values
#define VERBOSE_PS false              // Prints proximity sensor values
#define VERBOSE_SIGNAL_STRENGTH false // Prints signal stregth and packet data
#define VERBOSE_SIGNAL_RADIUS true   // Prints the sqrt

/*VARIABLES*/
static pose_t _odo_speed_acc, _odo_speed_enc;
static double imu_mean[6] = {0, 0, 0, 0, 0, 0};
static bool acc_mean_computed = false;
static double odo_enc_prev[2] = {0, 0};

/* variables for computing delta time */
static float last_robot_time = -INFINITY;

void controller_init(Pioneer &robot);
void set_position(Vec &mu, double x, double y);
void odo_reset();
void controller_compute_mean_acc(double imu[6], float time, double delta_time);
double compute_delta_time(double last_time, double current_time);

int main(int argc, char **argv)
{
  // Initialize the robot
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  // Initialize an example log file
  std::string f_example = "example.csv";
  int f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo_enc = "odo_enc.csv";
  int f_odo_enc_cols = init_csv(f_odo_enc, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo_acc = "odo_acc.csv";
  int f_odo_acc_cols = init_csv(f_odo_acc, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo_sigma = "odo_sigma.csv";
  int f_odo_sigma_cols = init_csv(f_odo_sigma, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo_enc_sigma = "odo_enc_sigma.csv";
  int f_odo_enc_sigma_cols = init_csv(f_odo_enc_sigma, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo_acc_sigma = "odo_acc_sigma.csv";
  int f_odo_acc_sigma_cols = init_csv(f_odo_acc_sigma, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo = "odo.csv";
  int f_odo_cols = init_csv(f_odo, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_sensor = "sensor_data.csv";                                                // Linus hat kaputt gemacht
  int f_sensor_cols = init_csv(f_sensor, "time, ID, signal_strength, x, y, T_in, T_out,"); // <-- don't forget the comma at the end of the string!!

  std::string f_sensor_node = "sensor_node.csv";
  int f_sensor_node_cols = init_csv(f_sensor_node, "id, signal_stregth, distance_calc, real_distance, C,"); // <-- don't forget the comma at the end of the string!!

  // init Kalman
  Mat Sigma = Mat::Zero();
  Vec mu = Vec::Zero();

  Mat Sigma_enc = Mat::Zero();
  Vec mu_enc = Vec::Zero();
  Mat Sigma_acc = Mat::Zero();
  Vec mu_acc = Vec::Zero();

  // reset odometry
  controller_init(robot);

  while (robot.step() != -1)
  {
    double truth_pose[4];
    robot.get_ground_truth_pose(truth_pose);

    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////

    double time = robot.get_time();             // Current time in seconds
    double *ps_values = robot.get_proximity();  // Measured proximity sensor values (16 values)
    double *wheel_rot = robot.get_encoders();   // Wheel rotations (left, right)
    double light = robot.get_light_intensity(); // Light intensity
    double *imu = robot.get_imu();              // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
    ////////////////////
    // Implementation //
    ////////////////////

    // DEBUG
    if (time > 150)
    {
      robot.set_motors_velocity(0, 0); // set the wheel velocities

      break;
    }

    // delta time computation
    double delta_time = compute_delta_time(last_robot_time, time);

    // Update previous time step
    last_robot_time = time;

    if (delta_time == INFINITY)
    {
      // first timestep, skip calculations
      continue;
    }

    // Accelerometer and Gyroscope bias computation
    if (!acc_mean_computed)
    {
      controller_compute_mean_acc(imu, time, delta_time);
      // skip odometry as long as mean is computed
      continue;
    }

    // Sensor value logging
    if (VERBOSE_ACC)
      printf("acceleration: %g %g %g, gyroscope: %g %g %g\n", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);

    if (VERBOSE_PS)
    {
      for (int i = 0; i < 16; i++)
      {
        printf("%d: %f; ", i, ps_values[i]);
      }
      printf("\n");
    }

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // Localization
    odo_compute_encoders(_odo_speed_enc, wheel_rot[0] - odo_enc_prev[0], wheel_rot[1] - odo_enc_prev[1], delta_time);
    odo_compute_acc(_odo_speed_acc, imu, imu_mean, delta_time);

    heading_enc += _odo_speed_enc.heading * delta_time;
    var_omega_enc += SIGMA_OMEGA_ENC * SIGMA_OMEGA_ENC * delta_time * delta_time;

    Vec u(_odo_speed_enc.x, 0, _odo_speed_acc.heading);

    prediction_step(mu, Sigma, u, delta_time);


    // for logging difference
    prediction_step_acc(mu_acc, Sigma_acc, _odo_speed_acc, delta_time);
    prediction_step_enc(mu_enc, Sigma_enc, _odo_speed_enc, delta_time);

    // Fuse sensor values
    // update_step_sensors_mat(mu, mu_acc, Sigma, Sigma_acc);
    update_step_sensors(mu, heading_enc, Sigma, var_omega_enc);

    if(signal_strength >= MAX_SIGNAL_STRENGTH) {
      std::cout << "help: " << signal_strength << std::endl;
    }

    // signal strength below threshold to avoid negative sqrt
    if (signal_strength > MIN_SIGNAL_STRENGTH && signal_strength < MAX_SIGNAL_STRENGTH)
    {
      double C = 1.08;
      double h = 1 - 0.277;
      double d_sqr = C / signal_strength;
      double radius = sqrt(d_sqr - h * h);

      if (VERBOSE_SIGNAL_RADIUS)
      {
        printf("Radius: %f, Signal strength: %f\n", radius, signal_strength);
      }
      // printf("radius: %f, radius^2: %f ", radius, radius * radius);

      Vec2D last_pos(mu(0), mu(1));
      Vec2D sensor_pos(data[1], data[2]);

      Vec2D diff = last_pos - sensor_pos;
      diff.normalize();

      Vec2D bias = Vec2D::Ones() * .01;
      bias[0] *= cos(mu(2));
      bias[1] *= sin(mu(2));

      Vec2D estimated_pos = sensor_pos + diff * radius + bias;

      double var = 0.01; // empirical

      update_step_sensor_node(mu, estimated_pos, Sigma, var);
    }

    // Update values
    for (int i = 0; i < 2; i++)
      odo_enc_prev[i] = wheel_rot[i];

    // NAVIGATION
    double lws = 0.0, rws = 0.0; // left and right wheel speeds
    double pose[4] = {mu(0), mu(1), mu(2), time};

    bool final_stop = fsm(ps_values, lws, rws, pose); // finite state machine
    robot.set_motors_velocity(lws, rws);              // set the wheel velocities

    //////////////////
    // Data logging //
    //////////////////

    // Log the time and light and IMU data in a csv file
    log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);

    // Log pose
    log_csv(f_odo, f_odo_cols, time, mu(0), mu(1), mu(2));

    // Log reference pose
    log_csv(f_odo_enc, f_odo_enc_cols, time, mu_enc(0), mu_enc(1), mu_enc(2));
    log_csv(f_odo_acc, f_odo_acc_cols, time, mu_acc(0), mu_acc(1), mu_acc(2));

    // Log uncertainty
    log_csv(f_odo_sigma, f_odo_sigma_cols, time, Sigma(0, 0), Sigma(1, 1), Sigma(2, 2));
    log_csv(f_odo_enc_sigma, f_odo_enc_sigma_cols, time, Sigma_enc(0, 0), Sigma_enc(1, 1), Sigma_enc(2, 2));
    log_csv(f_odo_acc_sigma, f_odo_acc_sigma_cols, time, Sigma_acc(0, 0), Sigma_acc(1, 1), Sigma_acc(2, 2));

    if (signal_strength > 0)
    {
      // Log sensor
      log_csv(f_sensor, f_sensor_cols, time, data[0], signal_strength, data[1], data[2], data[3], data[4]);

      if (VERBOSE_SIGNAL_STRENGTH)
      {
        printf("Signal [%f]: ", signal_strength);
        print_array(data);
      }
    }

    if (final_stop)
      break;
  }

  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}

void controller_init(Pioneer &robot)
{
  odo_init();
  odo_reset();
}

/**
 * @brief      Compute the delta time between two time steps
 */
double compute_delta_time(double last_time, double current_time)
{
  return current_time - last_time;
}

/**
 * @brief      Compute the mean of the 3-axis accelerometer for about TIME_INIT_ACC seconds. The result is stored in array imu_mean
 */
void controller_compute_mean_acc(double imu[6], float time, double delta_time)
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

    acc_mean_computed = true;

    if (VERBOSE_ACC_MEAN)
    {
      printf("Accelerometer initialization Done!\n");
      printf("ROBOT accelerometer mean: %g %g %g, gyroscope mean: %g %g %g\n", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
    }
  }
}

/**
 * @brief      Reset the odometry to zeros
 */
void odo_reset()
{
  memset(&_odo_speed_enc, 0, sizeof(pose_t));
  memset(&_odo_speed_acc, 0, sizeof(pose_t));
}

void set_position(Vec &mu, double x, double y)
{
  mu(0) = x;
  mu(1) = y;
}