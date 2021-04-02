using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Simulator {

public class TransformUtils {
  /**
    * Converts a Unity3D left-handed transform to a right-handed one (for compatibility with
    * everything else in the robotics world).
    *
    * To this, we flip the y-component of the rotation axis, and change the sign of the rotation
    * direction, since LH/RH coordinate frames define positive rotation in opposite ways.
    *
    *  *LEFT-HANDED*     *RIGHT-HANDED*
    *    (y) (z)             (z)
    *     | /               /
    *     |/_____(x)       /_____(x)
    *                      |
    *                      |
    *                     (y)
    */
  public static void ToRightHandedTransform(Transform lh, ref Vector3 t_rh, ref Quaternion q_rh)
  {
    ToRightHandedTranslation(lh.position, ref t_rh);
    ToRightHandedQuaternion(lh.rotation, ref q_rh);
  }

  public static void ToRightHandedTranslation(Vector3 t_lh, ref Vector3 rh)
  {
    rh.x = t_lh.x;
    rh.y = -t_lh.y;
    rh.z = t_lh.z;
  }

  public static void ToRightHandedQuaternion(Quaternion q_lh, ref Quaternion rh)
  {
    // Flip the y-component of the rotation axis.
    float angle_rh = 0.0f;
    Vector3 axis_rh = Vector3.zero;
    q_lh.ToAngleAxis(out angle_rh, out axis_rh);

    axis_rh.y *= -1;
    angle_rh *= -1;

    rh = Quaternion.AngleAxis(angle_rh, axis_rh).normalized;
  }

  /**
    * Converts a left-handed angular velocity to a right-handed coordinate frame.
    *
    * To do this, we flip the direction of the y axis, and change the direction of rotation for the
    * x and z axes. The y axis does NOT need the rotation direction switched, because we've already
    * flipped its orientation.
    *
    *  *LEFT-HANDED*     *RIGHT-HANDED*
    *    (y) (z)             (z)
    *     | /               /
    *     |/_____(x)       /_____(x)
    *                      |
    *                      |
    *                     (y)
    */
  public static void ToRightHandedAngularVelocity(Vector3 w_lh, ref Vector3 rh)
  {
    rh.x = -1.0f * w_lh.x;
    rh.y = w_lh.y;
    rh.z = -1.0f * w_lh.z;
  }

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
}

}
