using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

public struct RangeMeasurement
{
  public RangeMeasurement(long timestamp, float range, Vector3 world_t_beacon)
  {
    this.timestamp = timestamp;
    this.range = range;
    this.world_t_beacon = world_t_beacon;
  }

  public long timestamp;
  public float range;
  public Vector3 world_t_beacon;
}


// Attach this to a robot to simulate range measurements.
public class RangeSensor : MonoBehaviour
{
  public GameObject apsBeaconObject;
  public GameObject apsReceiverObject;

  public bool enableApsNoise = true;
  public float apsNoiseSigma = 0.1f;

  public RangeMeasurement data = new RangeMeasurement(0, 0, Vector3.zero);

  // Lazy read: only get sensor data when called.
  public void Read()
  {
    data.timestamp = Timestamp.UnityNanoseconds();
    data.world_t_beacon = this.apsBeaconObject.transform.position;
    data.range = (this.apsReceiverObject.transform.position - data.world_t_beacon).magnitude;
    data.world_t_beacon = TransformUtils.ToRightHandedTranslation(data.world_t_beacon);

    if (this.enableApsNoise && this.apsNoiseSigma > 0) {
      data.range += Gaussian.Sample1D(0, this.apsNoiseSigma);
    }
  }
}

}
