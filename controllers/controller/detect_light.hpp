#pragma once

#include "pioneer_interface/pioneer_interface.hpp"
#include "signal_analysis.hpp"

#define PEAK_THRESHOLD 2.22

static double prevval = 0, currval = 0, nextval = 0, light_count = 0, raw = 0;
static bool in_light = false;
static int lowSignalCounter = 0;
const int MAX_LOW_SIGNAL_COUNT = 40;
bool Defective = false;
static bool buffer_full = false;
static double signal_buffer[1024] = {0};
static int signal_index = 0;
static bool go_next = false;
const float Light_calibration = 0.28;

bool detectLight(Pioneer &robot, double light, std::string f_example, int f_example_cols, std::string f_amp_t, int f_amp_t_cols, double pose[3])
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

            if (signal_index >= SIGNAL_LENGTH) //if the signal buffer is full
            {
                signal_index = 0; //reset the signal index
                buffer_full = true;

                if (buffer_full)
                {
                    if (pose[2] < 1 && pose[2] > -1) //calibrate the position of the light
                        x = x + Light_calibration;

                    if (pose[2] < 4 && pose[2] > 2) //calibrate the position of the light
                        x = x - Light_calibration;
                    light_count += 1; //count one more light

                    kiss_fft_demo(signal_buffer, f_example, f_example_cols, x, y, f_amp_t, f_amp_t_cols, light_count, Defective);
                    buffer_full = false; // Reset if you want one-shot$
                    go_next = true;
                }
            }
        }
        else //no light detected
        {
            // Possibly defective light, increment flicker counter
            lowSignalCounter++;
            Defective = true; //assume defective light until proven wrong

            if (lowSignalCounter >= MAX_LOW_SIGNAL_COUNT) //if true then defective light proven wrong
            {
                Defective = false;
                // Light is truly gone
                in_light = false;
                go_next = false;
                lowSignalCounter = 0; //reset
            }
        }
    }
    return in_light && !go_next;
}