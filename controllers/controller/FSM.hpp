// Implemented by: Linus
#pragma once

#include "pioneer_interface/pioneer_interface.hpp"
#include "braitenberg.hpp"
#include <iostream>
#include <cmath>

#define VERBOSE_STATE_NAME false

//////////////////////
// Global variables //
//////////////////////

enum RobotState
{
  NAVIGATE,
  TURNING,
  EMERGENCY_BACKUP,
  STRAIGHT,
  STOP
};

static RobotState state = NAVIGATE;
static RobotState prev_state = NAVIGATE;

/**
 * @brief      Get string representation of a robot state
 * @param[in]  s  Robot state enum
 * @return     Corresponding state name as a string
 */
const char *getStateName(RobotState s)
{
  switch (s)
  {
  case NAVIGATE:
    return "NAVIGATE";
  case TURNING:
    return "TURNING";
  case EMERGENCY_BACKUP:
    return "EMERGENCY_BACKUP";
  case STRAIGHT:
    return "STRAIGHT";
  case STOP:
    return "STOP";
  default:
    return "UNKNOWN";
  }
}

////////////////////////
// Behavior Functions //
////////////////////////

/**
 * @brief      Clamp a value between two limits
 * @param[in]  val       Value to clamp
 * @param[in]  min_val   Minimum allowed value
 * @param[in]  max_val   Maximum allowed value
 * @return     Clamped value
 */
double clamp(double val, double min_val, double max_val)
{
  return std::max(min_val, std::min(val, max_val));
}

/**
 * @brief      Executes navigate behavior using Braitenberg logic
 * @param[in]  ps          Proximity sensor values
 * @param[out] vel_left    Output left wheel speed
 * @param[out] vel_right   Output right wheel speed
 * @param[in]  pose        Robot pose array
 */
void navigateBehavior(double *ps, double &vel_left, double &vel_right, double pose[3])
{
  braitenberg(ps, vel_left, vel_right);
}

/**
 * @brief      Executes emergency backup behavior
 * @param[out] vel_left    Output left wheel speed
 * @param[out] vel_right   Output right wheel speed
 */
void emergencyBackupBehavior(double &vel_left, double &vel_right)
{
  vel_left = -0.5;
  vel_right = -0.5;
}

/**
 * @brief      Executes straight forward movement behavior
 * @param[out] vel_left    Output left wheel speed
 * @param[out] vel_right   Output right wheel speed
 */
void straightBehavior(double &vel_left, double &vel_right)
{
  vel_left = 0.5;
  vel_right = 0.5;
}

/**
 * @brief      Executes final stop behavior
 * @param[out] vel_left    Output left wheel speed
 * @param[out] vel_right   Output right wheel speed
 */
void stopBehavior(double &vel_left, double &vel_right)
{
  vel_left = 0;
  vel_right = 0;
}

/**
 * @brief      Executes turning behavior when detecting corners or ends
 * @param[in]  ps                    Proximity sensor values
 * @param[out] vel_left              Output left wheel speed
 * @param[out] vel_right             Output right wheel speed
 * @param[in]  pose                  Robot pose
 * @param[in]  turn_left             Whether a left turn is expected
 * @param[in]  turn_right            Whether a right turn is expected
 * @param[in]  pot_found             Whether a potential turn was already found
 * @param[out] search_pot            Whether robot is in search mode
 * @param[out] search_pot_counter    Counter for search mode duration
 * @param[in]  search_pot_duration   Maximum duration for search mode
 */
void turningBehavior(
    double *ps,
    double &vel_left,
    double &vel_right,
    double pose[3],
    bool turn_left,
    bool turn_right,
    bool pot_found,
    bool &search_pot,
    int &search_pot_counter,
    const int search_pot_duration)
{

  if (turn_left)
  {

    double target = 1000;
    double sensor_left = std::max(ps[0], ps[15]);
    double error = sensor_left - target;
    double Kp = 0.015;
    double correction = Kp * error;

    if (pot_found && sensor_left < 850)
    {
      search_pot = true;
    }

    if (search_pot)
    {
      vel_left = 1;
      vel_right = 1;
      search_pot_counter++;
      if (search_pot_counter > search_pot_duration)
      {
        search_pot = false;
        search_pot_counter = 0;
      }
    }
    else if (ps[0] < ps[15])
    {
      vel_left = clamp(0.2 + correction / 2, -3.0, 3.0);
      vel_right = clamp(0.7 - correction, -3.0, 3.0);
    }
    else if (ps[0] > ps[15])
    {
      vel_left = 0.3;
      vel_right = clamp(0.5 - correction, -3.0, 3.0);
    }
  }

  if (turn_right)
  {
    double target = 1000;
    double sensor_right = std::max(ps[7], ps[8]);
    double error = sensor_right - target;
    double Kp = 0.015;
    double correction = Kp * error;

    if (pot_found && sensor_right < 850)
    {
      search_pot = true;
    }

    if (search_pot)
    {
      vel_left = 1;
      vel_right = 1;
      search_pot_counter++;
      if (search_pot_counter > search_pot_duration)
      {
        search_pot = false;
        search_pot_counter = 0;
      }
    }
    else if (ps[7] < ps[8])
    {
      vel_right = clamp(0.2 + correction / 2, -3.0, 3.0);
      vel_left = clamp(0.7 - correction, -3.0, 3.0);
    }
    else if (ps[7] > ps[8])
    {
      vel_right = 0.3;
      vel_left = clamp(0.5 - correction, -3.0, 3.0);
    }
  }
}

///////////////////////
// Main FSM function //
///////////////////////

/**
 * @brief      Finite State Machine that manages the robot's behavior
 * @param[in]  ps            Proximity sensor values
 * @param[out] vel_left      Output left wheel speed
 * @param[out] vel_right     Output right wheel speed
 * @param[in]  pose          Current robot pose
 * @param[in]  stop_for_light Whether light detection should force stop
 * @return     True if robot is at the end of the last corridor
 */
bool fsm(double *ps, double &vel_left, double &vel_right, double pose[3], bool stop_for_light)
{
  static int backup_counter = 0;
  static bool is_emergency = false;
  static bool turn_right = false;
  static bool turn_left = false;
  static bool is_straight = false;
  static bool is_end_of_corridor = false;
  static bool pot_found = false;
  static bool prev_state_stop;

  static int search_pot_counter = 0;
  static bool search_pot = false;
  static int stop_counter = 0;

  const int backup_duration = 70;
  const int search_pot_duration = 80;
  const int stop_confirm_duration = 150;

  double x = pose[0];
  double y = pose[1];
  double h = pose[2];

  turn_left = x > 5;
  turn_right = x < 0.0;
  bool at_final_stop = (y > 3.5 && y < 4.5 && x < 0);

  if (stop_for_light)
  {
    state = STOP;
    prev_state_stop = true;
  }
  else
  {
    if (prev_state_stop)
    {
      state = NAVIGATE;
      prev_state_stop = false;
    }

    if (at_final_stop)
    {

      stop_counter++;
      if (stop_counter > stop_confirm_duration)
      {
        state = STOP;
        printf("Stopped at the end of the last corridor!\n");
        return true;
      }
    }
    else
    {
      stop_counter = 0;
    }

    // EMERGENCY BACKUP
    if (is_emergency)
    {
      state = EMERGENCY_BACKUP;
      backup_counter++;
      bool rear_blocked = false;
      for (int i = 9; i <= 11; ++i)
      {
        if (ps[i] > 1010)
        {
          rear_blocked = true;
          break;
        }
      }
      if (rear_blocked || backup_counter > backup_duration)
      {
        state = prev_state;
        backup_counter = 0;
        is_emergency = false;
      }
    }
    else
    {
      for (int i = 1; i <= 6; ++i)
      {
        if (ps[i] > 1005)
        {
          prev_state = state;
          state = EMERGENCY_BACKUP;
          is_emergency = true;
          break;
        }
      }
    }

    // STRAIGHT CORRIDOR
    if (is_straight)
    {
      state = STRAIGHT;
      backup_counter++;
      if (backup_counter > backup_duration)
      {
        state = prev_state;
        backup_counter = 0;
        is_straight = false;
      }
    }
    else if (ps[15] > 1010 || ps[8] > 1010)
    {

      prev_state = state;
      state = STRAIGHT;
      is_straight = true;
    }

    // TURN DETECTION (LEFT)
    if (turn_left)
    {
      if (is_end_of_corridor)
      {
        pot_found = pot_found || ps[0] > 800 || ps[15] > 800;
        if (h > 2.8)
        {
          is_end_of_corridor = false;
          state = NAVIGATE;
          pot_found = false;
        }
      }
      else
      {
        bool out_cor = true;
        for (int i = 0; i <= 7; ++i)
        {
          if (ps[i] > 850)
          {

            out_cor = false;
            break;
          }
        }
        if (out_cor && !at_final_stop)
        {
          state = TURNING;
          is_end_of_corridor = true;
        }
      }
    }

    // TURN DETECTION (RIGHT)
    if (turn_right)
    {
      if (is_end_of_corridor)
      {
        pot_found = pot_found || ps[0] > 800 || ps[15] > 800;
        if (h < 0.3)
        {
          is_end_of_corridor = false;
          state = NAVIGATE;
          pot_found = false;
        }
      }
      else
      {
        bool out_cor = true;
        for (int i = 0; i <= 7; ++i)
        {
          if (ps[i] > 850)
          {
            out_cor = false;
            break;
          }
        }
        if (out_cor && !at_final_stop)
        {
          state = TURNING;
          is_end_of_corridor = true;
        }
      }
    }
  }
  // EXECUTE STATE
  switch (state)
  {
  case NAVIGATE:
    navigateBehavior(ps, vel_left, vel_right, pose);
    break;
  case TURNING:
    turningBehavior(ps, vel_left, vel_right, pose, turn_left, turn_right, pot_found, search_pot, search_pot_counter, search_pot_duration);
    break;
  case EMERGENCY_BACKUP:
    emergencyBackupBehavior(vel_left, vel_right);
    break;
  case STRAIGHT:
    straightBehavior(vel_left, vel_right);
    break;
  case STOP:
    stopBehavior(vel_left, vel_right);
    break;
  default:
    if (VERBOSE_STATE_NAME)
      std::cout << "Unknown FSM state!" << std::endl;
    vel_left = vel_right = 0.0;
    break;
  }

  if (VERBOSE_STATE_NAME)
    std::cout << "Current FSM state: " << getStateName(state) << std::endl;

  return false;
}