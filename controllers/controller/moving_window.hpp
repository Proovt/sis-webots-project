// Implemented by: Nico
#pragma once

#include <deque>
#include <iostream>

class MovingAverage
{
private:
    std::deque<double> values_;
    double cur_sum_;
    size_t max_window_size_;

public:
    MovingAverage(size_t window_size = 50)
        : cur_sum_(0.0), max_window_size_(window_size)
    {
        /*
     if (window_size <= 0) {
         throw std::invalid_argument("Window size must be greater than 0");
     }
  */
    }

    void slide(double new_value)
    {
        if (values_.size() >= max_window_size_)
        {
            cur_sum_ -= values_.front();
            values_.pop_front();
        }

        values_.push_back(new_value);
        cur_sum_ += new_value;
    }

    double compute_average()
    {
        return cur_sum_ / (double)values_.size();
    }
};