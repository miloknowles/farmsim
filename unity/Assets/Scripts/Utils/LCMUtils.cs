using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using vehicle;


namespace Simulator {

public class LCMUtils {
  public static void pack_header_t(long timestamp, long seq, string frame_id, ref header_t header)
  {
    header.timestamp = timestamp;
    header.seq = seq;
    header.frame_id = frame_id;
  }

  public static void pack_vector3_t(Vector3 v, ref vector3_t msg)
  {
    msg.x = v.x;
    msg.y = v.y;
    msg.z = v.z;
  }

  public static void unpack_vector3_t(vector3_t msg, ref Vector3 v)
  {
    v.x = (float)msg.x;
    v.y = (float)msg.y;
    v.z = (float)msg.z;
  }

  public static void pack_quaternion_t(Quaternion q, ref quaternion_t msg)
  {
    msg.w = q.w;
    msg.x = q.x;
    msg.y = q.y;
    msg.z = q.z;
  }

  public static void unpack_quaternion_t(quaternion_t msg, ref Quaternion q)
  {
    q.w = (float)msg.w;
    q.x = (float)msg.x;
    q.y = (float)msg.y;
    q.z = (float)msg.z;
  }

  public static void pack_pose3_t(Quaternion q, Vector3 t, ref pose3_t msg)
  {
    pack_quaternion_t(q, ref msg.orientation);
    pack_vector3_t(t, ref msg.position);
  }

  public static void unpack_pose3_t(pose3_t msg, ref Quaternion q, ref Vector3 t)
  {
    unpack_quaternion_t(msg.orientation, ref q);
    unpack_vector3_t(msg.position, ref t);
  }

  // Packs a Unity Texture2D into an LCM image_t type.
  public static void pack_image_t(ref Texture2D im, ref image_t msg)
  {
    msg.height = im.height;
    msg.width = im.width;
    msg.channels = 3;
    msg.format = "bgr8";
    msg.encoding = "jpg";

    // This seems to use a "BGR" channel ordering like OpenCV.
    msg.data = ImageConversion.EncodeToJPG(im, 75);
    msg.size = msg.data.Length;
  }
}

}
