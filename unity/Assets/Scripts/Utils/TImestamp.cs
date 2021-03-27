using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

public class Timestamp {
  public static long UnityNanoseconds()
  {
    return (long)(Time.fixedTime * 1e9);
  }

  public static double UnitySeconds()
  {
    return Time.fixedTime;
  }
}

}
