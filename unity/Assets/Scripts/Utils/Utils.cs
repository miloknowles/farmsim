using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

public class Utils {
  // If a value goes past a bound, set it to the other bound (i.e like the modulo operation).
  public static float CircularWrap(float value, float minValue, float maxValue)
  {
    if (value >= minValue && value <= maxValue) {
      return value;
    }
    return (value > maxValue) ? minValue : maxValue;
  }
}
}
