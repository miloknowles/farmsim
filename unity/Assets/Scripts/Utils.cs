using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

class Utils {
  // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
  public static Quaternion RotateAlignVectors(Vector3 v1, Vector3 v2)
  {
    Vector3 hat1 = v1.normalized;
    Vector3 hat2 = v2.normalized;

    Vector3 xyz = Vector3.Cross(hat1, hat2);
    float w = 1 + Vector3.Dot(hat1, hat2);

    Quaternion q = new Quaternion(xyz.x, xyz.y, xyz.z, w).normalized;
    return q;
  }

  // If a value goes past a bound, set it to the other bound (i.e like the modulo operation).
  public static float CircularWrap(float value, float minValue, float maxValue)
  {
    if (value >= minValue && value <= maxValue) {
      return value;
    }
    return (value > maxValue) ? minValue : maxValue;
  }

  // Rotate an object towards a look direction.
  public static void ApplyRotation(GameObject obj, Vector3 rotation, float rotateSpeed)
  {
    Quaternion targetRotation = Quaternion.LookRotation(rotation.normalized);
    obj.transform.rotation = Quaternion.RotateTowards(obj.transform.rotation, targetRotation, rotateSpeed * Time.deltaTime);
  }

  /**
   * Samples a direction for a fish to travel in. Chooses a uniformly randomly bearing to travel in
   * but limits the up/down (azimuth) angle of the fish to a shallow range, since fish probably
   * aren't going to swim straight up or down.
   */
  public static Vector3 SampleDirectionShallowAzimuth(float minAzimuth, float maxAzimuth)
  {
    float bearingAngle = Random.Range(0, 2.0f*Mathf.PI);
    float azimuthAngle = Random.Range(minAzimuth, maxAzimuth);

    // How much to elevate a unit vector in the XZ plane.
    float y = Mathf.Sin(Mathf.Deg2Rad * azimuthAngle);
    Vector3 direction = new Vector3(Mathf.Cos(bearingAngle), y, Mathf.Sin(bearingAngle));
    return direction.normalized;
  }

  // Generate a sample from a Gaussian distribution using the Box-Muller transform.
  // https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
  public static float Gaussian(float mu, float sigma)
  {
    float u1 = 1.0f - Random.Range(0.0f, 1.0f);
    float u2 = 1.0f - Random.Range(0.0f, 1.0f);
    float standardNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);

    return mu + sigma*standardNormal;
  }

  /**
   * Converts a Unity3D left-handed transform to a right-handed one (for compatibility with
   * everything else in the robotics world).
   *
   * Keeps the x and z axes the same, and flips the y axis.
   */
  public static void ToRightHandedTransform(Transform lh, out Vector3 t_rh, out Quaternion q_rh)
  {
    t_rh = new Vector3(lh.position.x, -lh.position.y, lh.position.z);

    // Flip the y-component of the rotation axis.
    float angle_lh = 0.0f;
    Vector3 axis_rh = Vector3.zero;
    lh.rotation.ToAngleAxis(out angle_lh, out axis_rh);

    // NOTE(milo): I can't quite wrap my head around this, but we DON'T need to flip the direction
    // of the y-axis for axis-angle rotations. Regardless of the handedness of the coordinate
    // system, a rotation around +y (LH) will have the same effect as a rotation around +y (RH).
    // axis_rh.y *= -1;

    q_rh = Quaternion.AngleAxis(angle_lh, axis_rh);
  }
}
}
