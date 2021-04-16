using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

// https://bluerobotics.com/store/sensors-sonars-cameras/sensors/bar30-sensor-r1/
public struct DepthMeasurement
{
  public DepthMeasurement(long timestamp, float depth)
  {
    this.timestamp = timestamp;
    this.depth = depth;
  }

  public long timestamp;
  public float depth;
}


public class DepthSensor : MonoBehaviour
{
  public GameObject depthSensorObject;

  public bool enableDepthNoise = true;
  public float noiseSigma = 0.02f;

  // Call Read() and then access this to get data.
  public DepthMeasurement data = new DepthMeasurement(0, 0);

  public void Read()
  {
    data.timestamp = Timestamp.UnityNanoseconds();

    // NOTE(milo): Unity uses a y-up convention, so flip the sign.
    data.depth = -1.0f * this.depthSensorObject.transform.position.y;

    // Optionally add sensor noise.
    if (this.noiseSigma > 0 && this.enableDepthNoise) {
      data.depth += Gaussian.Sample1D(0, this.noiseSigma);
    }
  }
}

}
