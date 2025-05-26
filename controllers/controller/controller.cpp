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
#define TIME_INIT_ACC 5          // Time in seconds
#define MIN_SIGNAL_STRENGTH 1.50 // Threshold for updating robot position
#define MAX_SIGNAL_STRENGTH 2.02 // Threshold for updating robot position

/*VERBOSE_FLAGS*/
#define VERBOSE_ACC_MEAN true         // Prints accelerometer mean values
#define VERBOSE_ACC false             // Prints accelerometer values
#define VERBOSE_PS false              // Prints proximity sensor values
#define VERBOSE_SIGNAL_STRENGTH false // Prints signal stregth and packet data

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

  std::string f_odo_enc_sigma = "odo_enc_sigma.csv";
  int f_odo_enc_sigma_cols = init_csv(f_odo_enc_sigma, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo = "odo.csv";
  int f_odo_cols = init_csv(f_odo, "time, x, y,"); // <-- don't forget the comma at the end of the string!!

  std::string f_sensor = "sensor_data.csv";
  int f_sensor_cols = init_csv(f_sensor, "time, id, signal_strength, x, y, Ti, To,"); // <-- don't forget the comma at the end of the string!!

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

    // Kalman Filter
    // mu_enc = mu;
    // mu_acc = mu;

    // Mat Sigma_enc = Sigma;
    // Mat Sigma_acc = Sigma;

    prediction_step_acc(mu_acc, Sigma_acc, _odo_speed_acc, delta_time);
    prediction_step_enc(mu, Sigma, _odo_speed_enc, delta_time);

    // Fuse sensor values

    // S = C / d^2
    update_step_sensors(mu, mu_acc, Sigma, Sigma_acc);

    // signal strength below threshold to avoid negative sqrt
    if (signal_strength > MIN_SIGNAL_STRENGTH && signal_strength < MAX_SIGNAL_STRENGTH)
    {
      double C = 1.08;
      double h = 1 - 0.277;
      double d_sqr = C / signal_strength;
      double radius = sqrt(d_sqr - h * h);
      printf("sqrt: %f, strength: %f\n", radius, signal_strength);
      // printf("radius: %f, radius^2: %f ", radius, radius * radius);

      Vec2D last_pos(mu(0), mu(1));
      Vec2D sensor_pos(data[1], data[2]);

      Vec2D diff = last_pos - sensor_pos;
      diff.normalize();

      Vec2D bias = Vec2D::Ones() * .01;
      bias[0] *= cos(mu(2));
      bias[1] *= sin(mu(2));

      /* MatX head_change;
      head_change << cos(mu(2)), 0,
          0, sin(mu(2)); */

      // std::cout << head_change * bias << std::endl;

      Vec2D estimated_pos = sensor_pos + diff * radius + bias;

      // printf("cur position: %f, %f; ", last_pos(0), last_pos(1));
      // printf("est position: %f, %f\n", estimated_pos(0), estimated_pos(1));

      // std::cout << "\n" << estimated_pos << " vs. \n" << last_pos << "\n" << std::endl;

      // double var = radius * radius; // 0.8 - 1.06 / signal_strength;
      double var = 0.05;
      // Vec2D measurements(data[1], data[2]);
      // printf("position before: %f, %f; ", mu(0), mu(1));
      update_step_sensor_node(mu, estimated_pos, Sigma, var);
      // printf("position after: %f, %f\n", mu(0), mu(1));
      // set_position(mu_enc, data[1], data[2]);
      // printf("Set robot position: [%f, %f]\n", mu_enc(0), mu_enc(1));
    }

    /* bool in_corridor = true;

    if (!in_corridor)
    {
      // test walls
      // double front_left = se
    } */

    // for logging difference
    prediction_step_enc(mu_enc, Sigma_enc, _odo_speed_enc, delta_time);

    // Update values
    for (int i = 0; i < 2; i++)
      odo_enc_prev[i] = wheel_rot[i];

    // NAVIGATION
    double lws = 0.0, rws = 0.0; // left and right wheel speeds
    // fsm(ps_values, lws, rws);            // finite state machine

    double pose[4] = {mu(0), mu(1), mu(2), time};

    braitenberg(ps_values, lws, rws, truth_pose);
    robot.set_motors_velocity(lws, rws); // set the wheel velocities

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
    // log_csv(f_odo_enc_sigma, f_odo_enc_sigma_cols, time, Sigma_enc(0, 0), Sigma_enc(1, 1), Sigma_enc(2, 2));

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
  }

  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}

void controller_init(Pioneer &robot)
{
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

  if (count > 20) // Remove the effects of strong acceleration at the begining
  {
    printf("computing acc\n");
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
      printf("Accelerometer initialization Done! \n");
      printf("ROBOT acc mean : %g %g %g\n", imu_mean[0], imu_mean[1], imu_mean[2]);
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