using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Simulator {

class Animation
{
  /**
   * Animates the vehicle moving between two waypoints.
   */
  IEnumerator AnimateMotion(GameObject vehicle,
                            Vector3 t_start,
                            Quaternion q_start,
                            Vector3 t_end,
                            Quaternion q_end,
                            float transit_sec)
  {
    float startTime = Time.time;
    float elap = (Time.time - startTime);
    while (elap < transit_sec) {
      elap = (Time.time - startTime);
      float T = Mathf.Clamp(elap / transit_sec, 0, 1); // Compute interpolation amount.

      // Linear interpolation between the two endpoints, slerp between quaternions.
      vehicle.transform.position = (1 - T)*t_start + T*t_end;
      vehicle.transform.rotation = Quaternion.Slerp(q_start, q_end, T);

      // Yield progress.
      yield return T;
    }
  }

  /**
   * Animates the vehicle moving from its current pose to a waypoint pose.
   */
  IEnumerator AnimateWaypoint(GameObject vehicle, Vector3 t_end, Quaternion q_end, float transit_sec, float rotate_sec)
  {
    // NOTE(milo): Need to grab the start transform HERE so that it's up-to-date when this coroutine
    // starts. If it was an argument, it would be pinned to whatever location the vehicle was at
    // upon instantiating the coroutine.
    Vector3 t_start = vehicle.transform.position;
    Quaternion q_start = vehicle.transform.rotation;

    // Align the vehicle with the direction of motion.
    Quaternion q_transit = Simulator.TransformUtils.RotateAlignVectors(new Vector3(0, 0, 1), t_end - t_start);

    // Rotate in place.
    float startTime = Time.time;
    while ((Time.time - startTime) < rotate_sec) {
      float elap = (Time.time - startTime);
      float T = Mathf.Clamp(elap / rotate_sec, 0, 1); // Compute interpolation amount.
      vehicle.transform.position = t_start;
      vehicle.transform.rotation = Quaternion.Slerp(q_start, q_transit, T);

      yield return T;
    }

    startTime = Time.time;
    while ((Time.time - startTime) < transit_sec) {
      float elap = (Time.time - startTime);

      // First half of the trajectory (accelerating).
      float T = 0;
      if (elap < (transit_sec / 2.0f)) {
        T = (2.0f / Mathf.Pow(transit_sec, 2.0f)) * Mathf.Pow(elap, 2.0f);

      // Second half of the trajectory (decelerating).
      } else {
        T = 1 - (2.0f / Mathf.Pow(transit_sec, 2.0f)) * Mathf.Pow((transit_sec - elap), 2.0f);
      }

      T = Mathf.Clamp(T, 0, 1); // Compute interpolation amount.

      // Linear interpolation between the two endpoints, slerp between quaternions.
      vehicle.transform.position = (1 - T)*t_start + T*t_end;
      vehicle.transform.rotation = q_transit;

      // Yield progress.
      yield return T;
    }

    // Rotate to goal orientation.
    startTime = Time.time;
    while ((Time.time - startTime) < rotate_sec) {
      float elap = (Time.time - startTime);
      float T = Mathf.Clamp(elap / rotate_sec, 0, 1); // Compute interpolation amount.
      vehicle.transform.position = t_end;
      vehicle.transform.rotation = Quaternion.Slerp(q_transit, q_end, T);
      yield return T;
    }
  }

  // IEnumerator AnimateFollowWinch(GameObject vehicle, int row, char buoy)
  // {
  //   GameObject winch_to_follow = this.farm.GetWinchesAtAddress(row, buoy)[0];
  //   this.farm.ToggleDepth(row, buoy);
  //   float prev_depth = 123;
  //   while (this.farm.winchInProgress) {
  //     prev_depth = winch_to_follow.transform.position.y;
  //     vehicle.transform.position = new Vector3(vehicle.transform.position.x, prev_depth, vehicle.transform.position.z);
  //     yield return null;
  //   }
  // }

  /**
   * Executes a sequence of coroutines. As soon as one finishes, the next one is started.
   */
  // IEnumerator ChainCoroutines(List<IEnumerator> routines)
  // {
  //   foreach (IEnumerator r in routines) {
  //     yield return StartCoroutine(r);
  //   }
  // }
};

}
