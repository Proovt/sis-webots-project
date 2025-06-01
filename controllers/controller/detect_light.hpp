// Implemented by: Miguel
#pragma once

#include "pioneer_interface/pioneer_interface.hpp"
#include "signal_analysis.hpp"

#define PEAK_THRESHOLD 2.22

static double prevval = 0, currval = 0, nextval = 0, light_count = 0, raw = 0;
static bool in_light = false;
static int lowSignalCounter = 0;
const int MAX_LOW_SIGNAL_COUNT = 40;
bool Defective = false;
static double signal_buffer[1024] = {0};
static int signal_index = 0;
static bool is_analyzed = false;
const float Light_calibration = 0.28;

/**
 * @brief      Detects light based on intensity and logs detection events
 * @param[in]  robot            The Pioneer robot
 * @param[in]  light            Measured light intensity
 * @param[in]  f_light_fft        CSV file path to store detections
 * @param[in]  f_light_fft_cols   Column count for f_light_fft
 * @param[in]  f_light_data          CSV file path to store signal characteristics
 * @param[in]  f_light_data_cols     Column count for f_light_data
 * @param[in]  pose             Robot pose array (x, y, heading)
 * @return     True if robot should stop for light, false otherwise
 */
bool detectLight(Pioneer &robot, double light, std::string f_light_fft, int f_light_fft_cols, std::string f_light_data, int f_light_data_cols, double pose[3])
{

    // For a smoother light reading due to the noise
    raw = light;
    nextval = 0.2 * currval + 0.8 * raw;

    //  Shift values
    prevval = currval;
    currval = nextval;

    double x = pose[0];
    double y = pose[1];

    // Debounce logic
    if (!in_light && currval > PEAK_THRESHOLD)
    {

        // reseting the buffer
        signal_index = 0;
        in_light = true;
        lowSignalCounter = 0; // Reset flicker counter when entering light
    }

    if (in_light)
    {
        if (currval > PEAK_THRESHOLD * 0.9)
        {
            // Still in light
            lowSignalCounter = 0;
            signal_buffer[signal_index++] = currval;

            if (signal_index >= SIGNAL_LENGTH) // if the signal buffer is full
            {
                signal_index = 0;                // reset the signal index
                if (pose[2] < 1 && pose[2] > -1) // calibrate the position of the light
                    x = x + Light_calibration;

                if (pose[2] < 4 && pose[2] > 2) // calibrate the position of the light
                    x = x - Light_calibration;
                light_count += 1; // count one more light

                if (Defective)
                {
                    // If the light is already known to be defective, log it and reset
                    printf("Detected light n°%.0f, status: Defective, location: (%.1f, %.1f)\n", light_count, x, y);
                    log_csv(f_light_fft, f_light_fft_cols, x, y, (double)DEFECTIVE);
                    Defective = false;
                }
                else
                {
                    // further analyze signal using FFT
                    fft_light_analysis(signal_buffer, f_light_fft, f_light_fft_cols, x, y, f_light_data, f_light_data_cols, light_count);
                }
                is_analyzed = true;
            }
        }
        else // no light detected
        {
            // Possibly defective light, increment flicker counter
            lowSignalCounter++;
            Defective = true; // assume defective light until proven wrong

            if (lowSignalCounter >= MAX_LOW_SIGNAL_COUNT) // if true then defective light proven wrong
            {
                Defective = false;
                // Light is truly gone
                in_light = false;
                is_analyzed = false;
                lowSignalCounter = 0; // reset
            }
        }
    }
    // returns true if an unseen light is detected
    return in_light && !is_analyzed;
}