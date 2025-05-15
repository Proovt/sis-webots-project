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
#define TIME_INIT_ACC 5 	      // Time in seconds
#define DELTA_TIME 32        // Delta time in milliseconds

/*VERBOSE_FLAGS*/
#define VERBOSE_ACC_MEAN true       // Print accelerometer mean values
#define VERBOSE_ACC false       // Print accelerometer values

/*VARIABLES*/
static pose_t _odo_acc, _odo_enc;
static double acc_mean[3] = {0, 0, 0};
static bool acc_mean_computed = false;
static double odo_enc_prev[2] = {0, 0};

// static double delta_time = 0.0;
// static float last_time = -100.;

void controller_init(Pioneer* robot);
void controller_compute_mean_acc(double* imu, float time, std::string fname, int fcols, double delta_time);
// void compute_delta_time(double current_time);
// void controller_compute_mean_acc(double* imu, float time);


void debug_compare_delta_time(Pioneer* robot) {
  int robot_timestep = robot->get_timestep(); // [ms]
  double delta_time = - robot->get_time(); // [s]
  // go one step forward to compute delta time
  robot->step();
  // delta time = time_1 - time_0
  delta_time += robot->get_time();

  int update_timestep = delta_time * 1000;

  printf("Robot timestep / world timestep %d ms\n", robot_timestep);
  printf("Sensor / Robot step (update) timestep: %d ms\n", update_timestep);
}

int main(int argc, char **argv) {
  // Initialize the robot 
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  // Initialize an example log file
  std::string f_example = "example.csv";
  int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!
  
  std::string f_odo_acc = "odo_acc.csv";
  int         f_odo_acc_cols = init_csv(f_odo_acc, "time, accx, acc_wx, acc_mean_x, velx, x, accy, acc_wy, acc_mean_y, vely, y, heading,"); // <-- don't forget the comma at the end of the string!!

  std::string f_odo_enc = "odo_enc.csv";
  int         f_odo_enc_cols = init_csv(f_odo_enc, "time, aleft_enc, aright_enc, speed_wx, speed_wy, omega_w, x, y, heading,"); // <-- don't forget the comma at the end of the string!!

    // log_csv(fname, fcols, time, Aleft_enc, Aright_enc, speed_wx, speed_wy, omega_w, _odo_pose_enc.x, _odo_pose_enc.y, _odo_pose_enc.heading);


  // reset odometry
  controller_init(&robot);

  while (robot.step() != -1) {
    /* debug_compare_delta_time(&robot);
    break; */
    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////

    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity sensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity();  // Light intensity
    double* imu = robot.get_imu();                // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    odo_compute_encoders(&_odo_acc, wheel_rot[0] - odo_enc_prev[0], wheel_rot[1] - odo_enc_prev[1], f_odo_enc, f_odo_enc_cols, time);

    for(int i = 0; i < 2; i++)
      odo_enc_prev[i] = wheel_rot[i];

    if(!acc_mean_computed) {
      controller_compute_mean_acc(imu, time, f_odo_acc, f_odo_acc_cols, robot.get_timestep());
      continue;
    } else {
      
      if(VERBOSE_ACC)
        printf("acceleration : %g %g %g\n", imu[0], imu[1], imu[2]);

      // 2. Localization
      odo_compute_acc(&_odo_acc, imu, acc_mean, f_odo_acc, f_odo_acc_cols, time);
    }


    /* for(int i = 0; i < 16; i++) {
      printf("%d: %f; ", i, ps_values[i]);
    }
    printf("\n"); */

    ////////////////////
    // Implementation //
    ////////////////////

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // NAVIGATION
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    fsm(ps_values, lws, rws);     // finite state machine 
    robot.set_motors_velocity(lws, rws); // set the wheel velocities

    //////////////////
    // Data logging //
    //////////////////

    // Log the time and light and IMU data in a csv file 
    log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);

  }

  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}


void controller_init(Pioneer* robot) {
  odo_reset(robot->get_timestep());
}

/* void compute_delta_time(double current_time) {
  if(delta_time == 0.0) {
    // printf("delta time 0 at time: %f\n", current_time);
    if (last_time != -100.) {
      delta_time = current_time - last_time;
      // printf("computing delta time: %f\n", delta_time);
    } else {
      // printf("Setting last time at %f\n", current_time);
      last_time = current_time;
    }
  }
} */

/**
 * @brief      Compute the mean of the 3-axis accelerometer. The result is stored in array _meas.acc_mean
 */
void controller_compute_mean_acc(double* imu, float time, std::string fname, int fcols, double delta_time)
{ 
  static int count = 0;

  count++;
  
  if(count > 20) // Remove the effects of strong acceleration at the begining
  {
    for(int i = 0; i < 3; i++)
        acc_mean[i] += imu[i];

    log_csv(fname, fcols, time, imu[0], 0., 0., 0., 0., imu[1], 0., 0., 0., 0.);
  }

  if(count == (int) (TIME_INIT_ACC / (double) delta_time * 1000)) {
    for(int i = 0; i < 3; i++)  
        acc_mean[i] /= (double) (count - 20);

    acc_mean_computed = true;

    if(VERBOSE_ACC_MEAN) {
      printf("Accelerometer initialization Done! \n");
      printf("ROBOT acc mean : %g %g %g\n", acc_mean[0], acc_mean[1] , acc_mean[2]);
    }
  }
}