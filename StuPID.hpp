#ifndef __INCLUDE_STUPID__
#define __INCLUDE_STUPID__

#include <Arduino.h>

class StuPID
{
public:
  StuPID(double *input,
         double *setpoint,
         double *output,
         double outputMin,
         double outputMax,
         double *Kp,
         double *Ki,
         double *Kd,
         unsigned long windowSize);

  // Returns true when the setpoint +-range is reached
  bool inRange(double range);

  // run PID calculation
  virtual void run();

  // reset integral part and previous error
  void reset();

protected:
  double *input_, *setpoint_, *output_;
  unsigned long windowSize_, lastComputationStartTime_;

private:
  double *Kp_, *Ki_, *Kd_;
  double integralError_, previousError_;
  double outputMin_, outputMax_;
};

class StuPIDRelay : public StuPID
{
public:
  StuPIDRelay(double *input,
              double *setpoint,
              bool *relayState,
              double windowSize,
              double *Kp,
              double *Ki,
              double *Kd)
      : StuPID(input, setpoint, output_, 0, 1.0, Kp, Ki, Kd, windowSize)
  {
    relayState_ = relayState;
  };

  void run() override;

private:
  bool *relayState_;
};

#endif
