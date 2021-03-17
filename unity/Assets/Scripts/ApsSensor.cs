using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

readonly public struct ApsMeasurement
{
  public ApsMeasurement(long timestamp, float range, Vector3 t_world_beacon)
  {
    this.timestamp = timestamp;
    this.range = range;
    this.t_world_beacon = t_world_beacon;
  }

  public readonly long timestamp;
  public readonly float range;
  public readonly Vector3 t_world_beacon;
}


public class ApsSensor : MonoBehaviour
{
  public GameObject apsBeaconObject;
  public GameObject apsReceiverObject;

  public bool enableApsNoise = true;
  public float apsNoiseSigma = 0.1f;

  // Lazy read: only get sensor data when called.
  public ApsMeasurement Read()
  {
    long nsec = (long)(Time.fixedTime * 1e9);
    Vector3 t_world_beacon = this.apsBeaconObject.transform.position;
    float range = (this.apsReceiverObject.transform.position - t_world_beacon).magnitude;

    if (this.enableApsNoise && this.apsNoiseSigma > 0) {
      range += Utils.Gaussian(0, this.apsNoiseSigma);
    }

    // NOTE(milo): Switch to right-handed coordinates!
    return new ApsMeasurement(nsec, range, TransformUtils.ToRightHandedTranslation(t_world_beacon));
  }
}

}
