using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Simulator {
  class TransformUtils {
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
    public static void ToRightHandedTransform(Transform lh, out Vector3 t_rh, out Quaternion q_rh)
    {
      t_rh = ToRightHandedTranslation(lh.position);
      q_rh = ToRightHandedQuaternion(lh.rotation);
    }

    public static Vector3 ToRightHandedTranslation(Vector3 t_lh)
    {
      return new Vector3(t_lh.x, -t_lh.y, t_lh.z);
    }

    public static Quaternion ToRightHandedQuaternion(Quaternion q_lh)
    {
      // Flip the y-component of the rotation axis.
      float angle_rh = 0.0f;
      Vector3 axis_rh = Vector3.zero;
      q_lh.ToAngleAxis(out angle_rh, out axis_rh);

      axis_rh.y *= -1;
      angle_rh *= -1;

      return Quaternion.AngleAxis(angle_rh, axis_rh).normalized;
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
    public static Vector3 ToRightHandedAngularVelocity(Vector3 w_lh)
    {
      return new Vector3(-1.0f * w_lh.x, w_lh.y, -1.0f * w_lh.z);
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
  }
}
