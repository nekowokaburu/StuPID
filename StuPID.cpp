#include "StuPID.hpp"

StuPID::StuPID(double *input,
               double *setpoint,
               double *output,
               double outputMin,
               double outputMax,
               double *Kp,
               double *Ki,
               double *Kd,
               unsigned long windowSize)
    : outputMin_{outputMin},
      outputMax_{outputMax},
      windowSize_{windowSize},
      integralError_{0},
      previousError_{0},
      lastComputationStartTime_{millis()}
{
    input_ = input;
    setpoint_ = setpoint;
    output_ = output;
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

bool StuPID::inRange(double range)
{
    return abs(*setpoint_ - *input_) <= range;
}

void StuPID::run()
{
    const unsigned long timeInteval = millis() - lastComputationStartTime_;
    if (timeInteval >= windowSize_)
    {
        lastComputationStartTime_ = millis();
        const double error = *setpoint_ - *input_;

        // Use time interval of seconds for better scalability with the user input parameters
        integralError_ += (error + previousError_) / 2.0 * (timeInteval / 1000.0);

        const double derivativeError = (error - previousError_) / (timeInteval / 1000.0);
        previousError_ = error;

        const double PID = (*Kp_ * error) + (*Ki_ * integralError_) + (*Kd_ * derivativeError);
        *output_ = constrain(PID, outputMin_, outputMax_);
    }
}

void StuPID::reset()
{
    lastComputationStartTime_ = millis();
    integralError_ = 0;
    previousError_ = 0;
}

void StuPIDRelay::run()
{
    // Update output_
    StuPID::run();

    // Set relay state to be on for an output [0.0, 1.0] percentage of the windowSize.
    // E.g. an output of 0.5 for a windowSize (= update interval) of 5 s would turn
    // the SSR on for 2.5 s and off for 2.5s.
    // This also adds additional safety if the output computes to HIGH for input > setpoint.
    *relayState_ = *input_ >= *setpoint_ ? false : ((millis() - lastComputationStartTime_) < (*output_ * windowSize_));
}
