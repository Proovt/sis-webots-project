// Implemented by: Linus, Miguel, Nico

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
#include "feature_detection.hpp"
#include "detect_light.hpp"

/* CONSTANTS */
#define MIN_SIGNAL_STRENGTH 1.50 // Minimum considered signal strength
#define MAX_SIGNAL_STRENGTH 2.04 // Maximum considered signal strength

/* VERBOSE_FLAGS */
#define VERBOSE_IMU false             // Prints accelerometer values
#define VERBOSE_PS false              // Prints proximity sensor values
#define VERBOSE_SIGNAL_STRENGTH false // Prints signal stregth and packet data

/* VARIABLES */
static double odo_enc_prev[2] = {0};
static bool imu_mean_computed = false;

// Odometry
static Vec2D odo_speed = Vec2D::Zero(); // prediction vector with (x speed, angular speed)

// Kalman
static Mat Sigma = Mat::Zero(); // state covariance matrix
static Vec mu = Vec::Zero();    // state vector
static Vec u = Vec::Zero();     // input vector
static double pose[3] = {0};    // pose array (x, y, heading)

/* Delta time */
static float last_robot_time = -INFINITY;

void controller_init(Pioneer &robot);
double compute_delta_time(double last_time, double current_time);

int main(int argc, char **argv)
{
  // Initialize the robot
  Pioneer robot = Pioneer(argc, argv);

  // Initialize controller and robot
  controller_init(robot);

  // Initialize log files
  std::string f_odo = "odo.csv";
  int f_odo_cols = init_csv(f_odo, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo_sigma = "odo_sigma.csv";
  int f_odo_sigma_cols = init_csv(f_odo_sigma, "time, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_sensor = "sensor_data.csv";
  int f_sensor_cols = init_csv(f_sensor, "time, ID, T_in, T_out,"); // <-- don't forget the comma at the end of the string!!

  std::string f_example = "Lights_detected.csv";
  int f_example_cols = init_csv(f_example, "x, y, status,"); // <-- don't forget the comma at the end of the string!!

  std::string f_amp_t = "light_data.csv";
  int f_amp_t_cols = init_csv(f_amp_t, "ID, amplitude, frequency, magnitude,"); // <-- don't forget the comma at the end of the string!!

  while (robot.step() != -1)
  {
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

    // update previous time step
    last_robot_time = time;

    if (delta_time == INFINITY)
    {
      // first timestep, skip calculations
      continue;
    }

    // gyroscope bias computation
    if (!imu_mean_computed)
    {
      imu_mean_computed = compute_gyro_bias(imu, time, delta_time);
      // skip odometry as long as mean is being computed
      continue;
    }

    // sensor value logging
    if (VERBOSE_IMU)
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
    odo_compute_encoders(odo_speed, wheel_rot[0] - odo_enc_prev[0], wheel_rot[1] - odo_enc_prev[1], delta_time);
    double gyro_z = odo_compute_gyroscope(imu, delta_time);

    // prediction
    u << odo_speed(0), 0, gyro_z;
    prediction_step(mu, Sigma, Sigma_u, u, delta_time);

    // update measurement
    prediction_step_heading_encoder(odo_speed(1), delta_time);

    // fuse sensor values
    update_step_sensors(mu, Sigma);

    // signal strength below threshold to avoid negative sqrt
    if (signal_strength > MIN_SIGNAL_STRENGTH && signal_strength < MAX_SIGNAL_STRENGTH)
    {
      handle_sensor_node(data, signal_strength, mu, Sigma);
    }

    // Update values
    for (int i = 0; i < 2; i++)
      odo_enc_prev[i] = wheel_rot[i];

    kal_get_state(mu, pose); // load pose from state vector

    // LIGHT DETECTION
    bool stop_for_light = detectLight(robot, light, f_example, f_example_cols, f_amp_t, f_amp_t_cols, pose);

    // NAVIGATION
    double lws = 0.0, rws = 0.0; // left and right wheel speeds

    bool final_stop = fsm(ps_values, lws, rws, pose, stop_for_light); // finite state machine
    robot.set_motors_velocity(lws, rws);                              // set the wheel velocities

    //////////////////
    // Data logging //
    //////////////////

    // Log pose
    log_csv(f_odo, f_odo_cols, time, mu(0), mu(1), mu(2));

    // Log uncertainty
    log_csv(f_odo_sigma, f_odo_sigma_cols, time, Sigma(0, 0), Sigma(1, 1), Sigma(2, 2));

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
  robot.init();

  odo_init();
  kalman_init();
}

/**
 * @brief      Compute the delta time between two time steps
 */
double compute_delta_time(double last_time, double current_time)
{
  return current_time - last_time;
}