using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Simulator {

readonly public struct ImuMeasurement
{
  public ImuMeasurement(long nsec, Vector3 a, Vector3 w)
  {
    this.nsec = nsec;
    this.imu_acceleration_rh = a;
    this.imu_angular_velocity_rh = w;
  }

  public readonly long nsec;
  public readonly Vector3 imu_acceleration_rh;
  public readonly Vector3 imu_angular_velocity_rh;
};


/**
 * Simulates measurements from an IMU (accelerometer and gyroscope).
 * NOTE(milo): The "IMU" and "Body" frames are the same for now!
 */
public class ImuSensor : MonoBehaviour
{
  public Rigidbody imu_rigidbody;   // Rigidbody that the IMU should experience forces from.

  // Used to calculate accleration with finite-differencing.
  private Vector3 prev_world_velocity;

  private ImuMeasurement _latest;

  public ImuMeasurement Read()
  {
    return this._latest;
  }

  void FixedUpdate()
  {
    // Rotation from the world to the local IMU frame.
    Quaternion q_imu_world = Quaternion.Inverse(this.imu_rigidbody.transform.rotation);

    Vector3 v_imu_cur = q_imu_world * this.imu_rigidbody.velocity;
    Vector3 v_imu_pre = q_imu_world * this.prev_world_velocity;
    Vector3 imu_a = (v_imu_cur - v_imu_pre) / Time.fixedDeltaTime;
    this.prev_world_velocity = this.imu_rigidbody.velocity;

    Vector3 imu_w = q_imu_world * this.imu_rigidbody.angularVelocity;

    // Rotate the gravity vector into the IMU's frame, then add it to acceleration.
    Vector3 imu_a_gravity = q_imu_world * Physics.gravity;
    Vector3 imu_a_total = imu_a - imu_a_gravity;

    // NOTE(milo): The IMU "feels" an upward acceleration due to gravity!
    Vector3 imu_a_rh = TransformUtils.ToRightHandedTranslation(imu_a_total);
    Vector3 imu_w_rh = TransformUtils.ToRightHandedAngularVelocity(imu_w);

    long nsec = (long)(Time.fixedTime * 1e9);
    this._latest = new ImuMeasurement(nsec, imu_a_rh, imu_w_rh);
  }
}

}
