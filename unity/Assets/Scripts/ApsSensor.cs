﻿using System.Collections;
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

  // Lazy read: only get sensor data when called.
  public ApsMeasurement Read()
  {
    long nsec = (long)(Time.fixedTime * 1e9);
    Vector3 t_world_beacon = this.apsBeaconObject.transform.position;
    float range = (this.apsReceiverObject.transform.position - t_world_beacon).magnitude;
    return new ApsMeasurement(nsec, range, t_world_beacon);
  }
}


}
