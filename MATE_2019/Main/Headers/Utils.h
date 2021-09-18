#ifndef UTILS_H
#define UTILS_H

#include <string>

class Utils
{
 public:
  // Converts a number from range a-b to range c-d
  static double convertRange(double oldMin, double oldMax, double newMin,
                             double newMax, double oldValue);

  // Finds the Nth occurance of a string in another string and returns its
  // position
  static int findNth(const std::string& str, const std::string& findMe,
                     int nth);
};

#endif
