using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

readonly public struct DepthMeasurement
{
  public DepthMeasurement(long timestamp, float depth)
  {
    this.timestamp = timestamp;
    this.depth = depth;
  }

  public readonly long timestamp;
  public readonly float depth;
}


public class DepthSensor : MonoBehaviour
{
  public GameObject depthSensorObject;

  public bool enableDepthNoise = true;
  public float noiseSigma = 0.05f;

  public DepthMeasurement Read()
  {
    long nsec = (long)(Time.fixedTime * 1e9);

    // NOTE(milo): Unity uses a y-up convention, so flip the sign.
    float depth = -1.0f * this.depthSensorObject.transform.position.y;

    // Optionally add sensor noise.
    if (this.noiseSigma > 0 && this.enableDepthNoise) {
      depth += Utils.Gaussian(0, this.noiseSigma);
    }

    return new DepthMeasurement(nsec, depth);
  }
}

}
