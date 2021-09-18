#ifndef PID_H
#define PID_H

#include <math.h>

class PID
{
 public:
  PID(double, double, double);
  PID(double, double, double, double);
  void setP(double);
  void setI(double);
  void setD(double);
  void setF(double);
  void setPID(double, double, double);
  void setPID(double, double, double, double);
  void setMaxIOutput(double);
  void setInputLimits(double);
  void setInputLimits(double, double);
  void setOutputLimits(double);
  void setOutputLimits(double, double);
  void setReversed(bool);
  void setSetpoint(double);
  void reset();
  void setOutputRampRate(double);
  void setSetpointRange(double);
  void setOutputFilter(double);
  void setContinuous(bool);
  double getOutput();
  double getOutput(double);
  double getOutput(double, double);

 private:
  double clamp(double, double, double);
  bool bounded(double, double, double);
  void checkSigns();
  void init();
  double P;
  double I;
  double D;
  double F;

  double maxIOutput;
  double maxError;
  double errorSum;

  double maxInput;
  double minInput;

  double maxOutput;
  double minOutput;

  double setpoint;

  double lastActual;

  bool firstRun;
  bool reversed;
  bool continuous;

  double outputRampRate;
  double lastOutput;

  double outputFilter;

  double setpointRange;
};
#endif
