using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using vehicle;


namespace Simulator {

public class LCMUtils {
  public static header_t pack_header_t(long timestamp, long seq, string frame_id)
  {
    header_t header = new header_t();
    header.timestamp = timestamp;
    header.seq = seq;
    header.frame_id = frame_id;
    return header;
  }

  public static vector3_t pack_vector3_t(Vector3 v)
  {
    vector3_t msg = new vector3_t();
    msg.x = v.x;
    msg.y = v.y;
    msg.z = v.z;
    return msg;
  }

  public static quaternion_t pack_quaternion_t(Quaternion q)
  {
    quaternion_t msg = new quaternion_t();
    msg.w = q.w;
    msg.x = q.x;
    msg.y = q.y;
    msg.z = q.z;
    return msg;
  }

  public static pose3_t pack_pose3_t(Quaternion q, Vector3 t)
  {
    pose3_t msg = new pose3_t();
    msg.orientation = pack_quaternion_t(q);
    msg.position = pack_vector3_t(t);
    return msg;
  }
}

}
