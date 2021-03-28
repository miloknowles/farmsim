using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

public class ListUtils {
  // Average value in a list of floats.
  public static float Average(List<float> values)
  {
    float total = 0;
    foreach (float v in values) {
      total += v;
    }
    return total / (float)values.Count;
  }
}

}
