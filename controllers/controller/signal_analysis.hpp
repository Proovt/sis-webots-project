// Implemented by: Miguel
#pragma once

#include <math.h>
#include "kiss_fft/kiss_fft.h" // FFT library
#include <iostream>
#include <cmath>

#define SIGNAL_LENGTH 1024 // length of the signal to be analyzed

enum LightStatus
{
    NOMINAL = 1,
    FLICKERING = 2,
    DEFECTIVE = 3
};

/**
 * @brief KISS FFT USAGE EXAMPLE
 * @param signal_data pointer to the array containing the signal to be analyzed
 */

void kiss_fft_demo(double *signal_data, std::string f_example, int f_example_cols, double x, double y, std::string f_amp_t, int f_amp_t_cols, double count, bool &Defective)
{

    /* fft preparation */
    kiss_fft_cfg cfg = kiss_fft_alloc(SIGNAL_LENGTH, 0, NULL, NULL);

    /* fft variables */
    kiss_fft_cpx cx_in[SIGNAL_LENGTH], // input signal (time domain)
        cx_out[SIGNAL_LENGTH];         // output signal (frequency domain)
    // prepare the input (add signal in the 'in' structure of the kiss_fft)
    for (int n = 0; n < SIGNAL_LENGTH; n++)
    {
        cx_in[n].r = signal_data[n]; // the real part of the signal is the data
        cx_in[n].i = 0.;             // set the imaginary part to zero
    }
    // run the fft (the fourier transform is stored in the 'out' structure of the kiss_fft)
    kiss_fft(cfg, cx_in, cx_out);

    // compute the magnitude of the complex numbers
    double mag[SIGNAL_LENGTH];
    for (int n = 0; n < SIGNAL_LENGTH; n++)
    {
        mag[n] = sqrt(cx_out[n].r * cx_out[n].r + cx_out[n].i * cx_out[n].i);
    }

    // free fft memory once done with it
    free(cfg);

    for (int i = 1; i < SIGNAL_LENGTH; i++)
    {
        log_csv(f_amp_t, f_amp_t_cols, count, signal_data[i], (double)i - 512, mag[i]); // Used to plot FFTs, so one can analyze the signal more precisely
    }

    if (Defective) // If already known that light is defective, save it and no need to test if it is nominal or flickering
    {
        printf("Detected light n°%.0f, status: Defective, location: (%.1f, %.1f)\n", count, x, y);
        log_csv(f_example, f_example_cols, x, y, (double)DEFECTIVE);
        Defective = false;
        return;
    }
    else
    {
        for (int i = 1; i < SIGNAL_LENGTH; ++i)
        {
            if (mag[i] > 10.0f) // if there is a frequency with a magnitude bigger than 10, then the light is flickering
            {
                printf("Detected light n°%.0f, status: Flickering, location: (%.1f, %.1f)\n", count, x, y);
                log_csv(f_example, f_example_cols, x, y, (double)FLICKERING);
                return;
            }
            else // if not, the FFT doesn't have any peaks over 10 and the light isn't defective, then the light is nominal
            {
                printf("Detected light n°%.0f, status: Good, location: (%.1f, %.1f)\n", count, x, y);
                log_csv(f_example, f_example_cols, x, y, (double)NOMINAL);
                return;
            }
        }
    }
}
