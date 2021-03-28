using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

readonly public struct RangeMeasurement
{
  public RangeMeasurement(long timestamp, float range, Vector3 world_t_beacon)
  {
    this.timestamp = timestamp;
    this.range = range;
    this.world_t_beacon = world_t_beacon;
  }

  public readonly long timestamp;
  public readonly float range;
  public readonly Vector3 world_t_beacon;
}


// Attach this to a robot to simulate range measurements.
public class RangeSensor : MonoBehaviour
{
  public GameObject apsBeaconObject;
  public GameObject apsReceiverObject;

  public bool enableApsNoise = true;
  public float apsNoiseSigma = 0.1f;

  // Lazy read: only get sensor data when called.
  public RangeMeasurement Read()
  {
    Vector3 world_t_beacon = this.apsBeaconObject.transform.position;
    float range = (this.apsReceiverObject.transform.position - world_t_beacon).magnitude;

    if (this.enableApsNoise && this.apsNoiseSigma > 0) {
      range += Gaussian.Sample1D(0, this.apsNoiseSigma);
    }

    // NOTE(milo): Switch to right-handed coordinates!
    return new RangeMeasurement(Timestamp.UnityNanoseconds(), range, TransformUtils.ToRightHandedTranslation(world_t_beacon));
  }
}

}
