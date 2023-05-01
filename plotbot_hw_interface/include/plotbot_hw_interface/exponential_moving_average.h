#ifndef PLOTBOT_HW_INTERFACE_EMA_H
#define PLOTBOT_HW_INTERFACE_EMA_H

// Discrete transfer function of low pass filter in: "y[n] = (1 - α) * y[n-1] + α * x[n]"
// where α = T / (T + Ts)
// Ts sampling time and T = 1 / (2πf_c), f_c is cutoff frequency.
// the cutoff frequency should be set to half of the sampling rate.
//  EMA and low pass filter actually is the same thing.
namespace plotbot_hw_interface
{
template <typename T1>
class ExpMovingAverage
{
private:
  T1 average_;
  T1 average_old_;
  T1 coefficent_;

public:
  /**
   * @brief Construct a new Exponantial Moving Average (EMA) object
   *
   * @param coefficient
   */
  // For more information about EMA : https://tinyurl.com/5awhj9sh
  explicit ExpMovingAverage(T1 coefficient) : average_(), average_old_(), coefficent_(coefficient)
  {
  }

  ExpMovingAverage() : average_(), average_old_(), coefficent_(0.7)
  {
  }

  T1 operator()(T1 measurement)
  {
    average_ = coefficent_ * measurement + (1 - coefficent_) * average_old_;
    average_old_ = average_;
    return average_;
  }
};
}  // namespace deliverbot_hw_interface

#endif  // PLOTBOT_HW_INTERFACE_EMA_H
