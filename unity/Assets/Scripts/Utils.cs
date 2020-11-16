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
}

}
