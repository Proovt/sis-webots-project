// Implemented by: Nico
#pragma once

#include <deque>
#include <iostream>

/**
 * @brief      Maintains a sliding window of values and computes their average
 */
class MovingAverage
{
private:
    std::deque<double> values_;
    double cur_sum_;
    size_t max_window_size_;

public:
    /**
     * @brief      Constructs the moving average with optional window size
     * @param[in]  window_size  Number of elements in the moving window
     */
    MovingAverage(size_t window_size = 50)
        : cur_sum_(0.0), max_window_size_(window_size)
    {
    }

    /**
     * @brief      Add a new value and update the average
     * @param[in]  new_value  New value to include in average
     */
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

    /**
     * @brief      Compute the current average
     * @return     Mean value of current window
     */
    double compute_average()
    {
        return cur_sum_ / (double)values_.size();
    }
};