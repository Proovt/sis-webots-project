#pragma once

#include <deque>
#include <iostream>

#define MAX_WINDOW_SIZE 2000

typedef struct
{
private:
    std::deque<double> moving_avg;
    double cur_sum = 0;

public:
    void slide(double new_value)
    {
        if (moving_avg.size() >= MAX_WINDOW_SIZE)
        {
            cur_sum -= moving_avg.front();
            moving_avg.pop_front();
        }

        moving_avg.push_back(new_value);
        cur_sum += new_value;
    }

    double compute_average()
    {
        return cur_sum / (double)moving_avg.size();
    }
} MovingAverage;