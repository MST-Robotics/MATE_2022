#include "..\Headers\Utils.h"

double Utils::convertRange(double oldMin, double oldMax, double newMin,
                           double newMax, double oldValue)
{
  double oldRange = (oldMax - oldMin);
  double newRange = (newMax - newMin);

  return (((oldValue - oldMin) * newRange) / oldRange) + newMin;
}

int Utils::findNth(const std::string& str, const std::string& findMe, int nth)
{
  size_t pos = -1;
  int count = 0;

  while (count != nth)
  {
    ++pos;
    pos = str.find(findMe, pos);
    if (pos == std::string::npos)
    {
      return -1;
    }
    ++count;
  }
  return pos;
}
