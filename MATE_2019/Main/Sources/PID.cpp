#include "..\Headers\PID.h"

// Constructor functions

PID::PID(double p, double i=0, double d=0, double f=0)
{
  init();
  P = p;
  I = i;
  D = d;
  F = f;
}
void PID::init()
{
  P = 0;
  I = 0;
  D = 0;
  F = 0;

  maxIOutput = 0;
  maxError = 0;
  errorSum = 0;
  maxOutput = 0;
  minOutput = 0;
  setpoint = 0;
  lastActual = 0;
  firstRun = true;
  reversed = false;
  continuous = true;
  outputRampRate = 0;
  lastOutput = 0;
  outputFilter = 0;
  setpointRange = 0;
}

// Configuration functions

// Configure the Proportional gain parameter. 
// This responds quicly to changes in setpoint, and provides most of the initial driving force to make corrections. 
// Some systems can be used with only a P gain, and many can be operated with only PI.
// For position based controllers, this is the first parameter to tune, with I second. 
// For rate controlled systems, this is often the second after F.
// p is for Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)</b>

void PID::setP(double p)
{
  P = p;
  checkSigns();
}

// Changes the I parameter
// This is used for overcoming disturbances, and ensuring that the controller always gets to the control mode. 
// Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. 
// Affects output through <b>output+=previous_errors*Igain ;previous_errors+=current_error</b>
// See {@link #setMaxIOutput(double) setMaxIOutput} for how to restrict @param i New gain value for the Integral term

void PID::setI(double i)
{
  if (I != 0)
  {
    errorSum = errorSum * I / i;
  }
  if (maxIOutput != 0)
  {
    maxError = maxIOutput / i;
  }
  I = i;
  checkSigns();
  // Implementation note:
  // This Scales the accumulated error to avoid output errors.
  // As an example doubling the I term cuts the accumulated error in half, 
  // Which results in the output change due to the I term constant during the transition.
}

void PID::setD(double d)
{
  D = d;
  checkSigns();
}

// Configure the FeedForward parameter. 
// This is excellent for Velocity, rate,
// Other	continuous control modes where you can expect a rough output value based solely on the setpoint.
// Should not be used in "position" based control modes.
// f is for Feed forward gain. Affects output according to <b>output+=F*Setpoint</b>;

void PID::setF(double f)
{
  F = f;
  checkSigns();
}

// Creates a new PID object.
// p, Proportional gain. Large if large difference between setpoint and target.
// i is for Integral gain.	Becomes large if setpoint cannot reach target quickly.
// d is for Derivative gain. Responds quickly to large changes in error. 
// Small values prevent P and I terms from causing overshoot.

void PID::setPID(double p, double i, double d)
{
  P = p;
  I = i;
  D = d;
  checkSigns();
}

void PID::setPID(double p, double i, double d, double f)
{
  P = p;
  I = i;
  D = d;
  F = f;
  checkSigns();
}

// Set the maximum output value contributed by the I component of the system
// This can be used to prevent large windup issues and make tuning simpler
// For maximum: Units are the same as the expected output value

void PID::setMaxIOutput(double maximum)
{
  // Internally maxError and Izone are similar, but scaled for different purposes.
  // The maxError is generated for simplifying math,
  // Since calculations against the max error are far more common than changing the I term or Izone.
 
  maxIOutput = maximum;
  if (I != 0)
  {
    maxError = maxIOutput / I;
  }
}

// Specify a maximum input. 
// If a single parameter is specified, the minimum is set to (-maximum).

void PID::setInputLimits(double input) { setInputLimits(-input, input); }

// Specify a maximum input
// minimum possible input value
// maximum possible input value
 
void PID::setInputLimits(double minimum, double maximum)
{
  if (maximum < minimum)
  {
    return;
  }
  maxInput = maximum;
  minInput = minimum;
}

// Specify a maximum output. 
// If a single parameter is specified, the minimum is set to (-maximum).

void PID::setOutputLimits(double output) { setOutputLimits(-output, output); }

// Specify a maximum output.
// minimum possible output value
// maximum possible output value
 
void PID::setOutputLimits(double minimum, double maximum)
{
  if (maximum < minimum)
  {
    return;
  }
  maxOutput = maximum;
  minOutput = minimum;

  // Ensure the bounds of the I term are within the bounds of the allowable output swing
  if (maxIOutput == 0 || maxIOutput > (maximum - minimum))
  {
    setMaxIOutput(maximum - minimum);
  }
}

// Set the operating direction of the PID controller
// reversed: Set true to reverse PID output

void PID::setReversed(bool reversed) { this->reversed = reversed; }

void PID::setContinuous(bool continuous) { this->continuous = continuous; }

// Primary operating functions
// Set the target for the PID calculations

void PID::setSetpoint(double setpoint) { this->setpoint = setpoint; }

// Calculate the PID value needed to hit the target setpoint.
// Automatically re-calculates the output at each call.
// actual: The monitored value
// target: The target value
// @return calculated output value for driving the actual to the target

double PID::getOutput(double actual, double setpoint)
{
  double output;
  double Poutput;
  double Ioutput;
  double Doutput;
  double Foutput;

  this->setpoint = setpoint;

  // Ramp the setpoint used for calculations if user has opted to do so
  if (setpointRange != 0)
  {
    setpoint = clamp(setpoint, actual - setpointRange, actual + setpointRange);
  }

  // Do the simple parts of the calculations
  double error = setpoint - actual;

  // Calculate F output. Notice, this depends only on the setpoint, and not the error.
  Foutput = F * setpoint;

  // Calculate P term
  Poutput = P * error;

  // If continuous is set to true allow wrap around
  if (continuous)
  {
    if (fabs(error) > (maxInput - minInput) / 2.0)
    {
      if (error > 0.0)
      {
        error = error - maxInput + minInput;
      }
      else
      {
        error = error + maxInput - minInput;
      }
    }
  }

  // If this is our first time running this we don't actually _have_ a previous input or output. 
  // For sensor, sanely assume it was exactly where it is now. 
  // For last output, we can assume it's the current time-independent outputs.
  
  if (firstRun)
  {
    lastActual = actual;
    lastOutput = Poutput + Foutput;
    firstRun = false;
  }

  // Calculate D Term
  // Note, this is negative. this actually "slows" the system if it's doing
  // The correct thing, and small values helps prevent output spikes and overshoot

  Doutput = -D * (actual - lastActual);
  lastActual = actual;

  // The Iterm is more complex. There's several things to factor in to make it easier to deal with.
  // 1. maxIoutput restricts the amount of output contributed by the Iterm.
  // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
  // 3. prevent windup by not increasing errorSum if output is output=maxOutput
  Ioutput = I * errorSum;
  if (maxIOutput != 0)
  {
    Ioutput = clamp(Ioutput, -maxIOutput, maxIOutput);
  }

  // And, finally, we can just add the terms up
  output = Foutput + Poutput + Ioutput + Doutput;

  // Figure out what we're doing with the error.
  if (minOutput != maxOutput && !bounded(output, minOutput, maxOutput))
  {
    errorSum = error;
    // Reset the error sum to a sane level
    // Setting to current error ensures a smooth transition when the P term
    // Decreases enough for the I term to start acting upon the controller
    // From that point the I term will build up as would be expected
  }
  else if (outputRampRate != 0 && !bounded(output, lastOutput - outputRampRate,
                                           lastOutput + outputRampRate))
  {
    errorSum = error;
  }
  else if (maxIOutput != 0)
  {
    errorSum = clamp(errorSum + error, -maxError, maxError);
    // In addition to output limiting directly, we also want to prevent I term buildup
    // SAo restrict the error directly
  }
  else
  {
    errorSum += error;
  }

  // Restrict output to our specified output and ramp limits
  if (outputRampRate != 0)
  {
    output =
        clamp(output, lastOutput - outputRampRate, lastOutput + outputRampRate);
  }
  if (minOutput != maxOutput)
  {
    output = clamp(output, minOutput, maxOutput);
  }
  if (outputFilter != 0)
  {
    output = lastOutput * outputFilter + output * (1 - outputFilter);
  }

  lastOutput = output;
  return output;
}

// Calculates the PID value using the last provided setpoint and actual values
// @return calculated output value for driving the actual to the target

double PID::getOutput() { return getOutput(lastActual, setpoint); }

// @return calculated output value for driving the actual to the target

double PID::getOutput(double actual) { return getOutput(actual, setpoint); }

// Resets the controller
// This erases the I term buildup and removes D gain on the next loop.

void PID::reset()
{
  firstRun = true;
  errorSum = 0;
}

// Set the maximum rate the output can increase per cycle.

void PID::setOutputRampRate(double rate) { outputRampRate = rate; }

// Set a limit on how far the setpoint can be from the current position
// Can simplify tuning by helping tuning over a small range applies to a much larger range. 
// This limits the reactivity of P term, 
// And restricts impact of large D term during large setpoint adjustments. 
// Increases lag and I term if range is too small.

void PID::setSetpointRange(double range) { setpointRange = range; }

// Set a filter on the output to reduce sharp oscillations.
// 0.1 is likely a sane starting value. Larger values P and D oscillations, 
// But force larger I values. 
// Uses an exponential rolling sum filter, 
// According to a simple <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
// Output is valid between [0..1), 
// meaning [current output only.. historical output only)

void PID::setOutputFilter(double strength)
{
  if (strength == 0 || bounded(strength, 0, 1))
  {
    outputFilter = strength;
  }
}

// Helper functions

// Forces a value into a specific range
// value input value
// maximum returned value
// maximum value in range
 // return Value if it's within provided range, min or max otherwise

double PID::clamp(double value, double min, double max)
{
  if (value > max)
  {
    return max;
  }
  if (value < min)
  {
    return min;
  }
  return value;
}

// Test it the value is within the min and max, inclusive
// Value to test
// Minimum value of range
// Maximum value of range

bool PID::bounded(double value, double min, double max)
{
  return (min < value) && (value < max);
}

// To operate correctly, all PID parameters require the same sign,
// with that sign depending on the {@literal}reversed value

void PID::checkSigns()
{
  if (reversed)
  {  // all values should be below zero
    if (P > 0) P *= -1;
    if (I > 0) I *= -1;
    if (D > 0) D *= -1;
    if (F > 0) F *= -1;
  }
  else
  {  // all values should be above zero
    if (P < 0) P *= -1;
    if (I < 0) I *= -1;
    if (D < 0) D *= -1;
    if (F < 0) F *= -1;
  }
}
