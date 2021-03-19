using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Simulator {

readonly public struct ImuMeasurement
{
  public ImuMeasurement(long timestamp, Vector3 a, Vector3 w)
  {
    this.timestamp = timestamp;
    this.imu_acceleration_rh = a;
    this.imu_angular_velocity_rh = w;
  }

  public readonly long timestamp;
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

  public bool enableImuNoise = true;
  public bool enableImuBias = true;

  public float accelNoiseSigma = 0.003924f;
  public float gyroNoiseSigma = 0.000205689024915f;
  public float accelBiasRandomWalkSigma = 0.004905f;
  public float gyroBiasRandomWalkSigma = 0.000001454441043f;
  public float accelBiasSigma = 0.001f;
  public float gyroBiasSigma = 0.001f;

  private Vector3 accelBias = new Vector3(0, 0, 0);
  private Vector3 gyroBias = new Vector3(0, 0, 0);

  // Used to calculate accleration with finite-differencing.
  private Vector3 prev_world_v_imu;

  private ImuMeasurement _latest;

  public ImuMeasurement Read()
  {
    return this._latest;
  }

  void Start()
  {
    // For now, just sample a bias when the simulation starts, and hold it constant throughout.
    if (this.accelBiasSigma > 0 && this.enableImuBias) {
      this.accelBias.x += Utils.Gaussian(0, this.accelBiasSigma);
      this.accelBias.y += Utils.Gaussian(0, this.accelBiasSigma);
      this.accelBias.z += Utils.Gaussian(0, this.accelBiasSigma);
    }

    if (this.gyroBiasSigma > 0 && this.enableImuBias) {
      this.gyroBias.x += Utils.Gaussian(0, this.gyroBiasSigma);
      this.gyroBias.y += Utils.Gaussian(0, this.gyroBiasSigma);
      this.gyroBias.z += Utils.Gaussian(0, this.gyroBiasSigma);
    }
  }

  void FixedUpdate()
  {
    // TODO(milo): You forgot to add the bias!
    // Rotation from the world to the local IMU frame.
    Quaternion world_q_imu = Quaternion.Inverse(this.imu_rigidbody.transform.rotation);

    // NOTE(milo): Really important to do velocity-differencing in the WORLD frame!
    Vector3 world_a_imu = (this.imu_rigidbody.velocity - this.prev_world_v_imu) / Time.fixedDeltaTime;
    Vector3 imu_a = world_q_imu * world_a_imu;
    this.prev_world_v_imu = this.imu_rigidbody.velocity;

    Vector3 imu_w = world_q_imu * this.imu_rigidbody.angularVelocity;

    // Rotate the gravity vector into the IMU's frame, then add it to acceleration.
    Vector3 imu_a_gravity = world_q_imu * Physics.gravity;
    Vector3 imu_a_total = imu_a - imu_a_gravity;

    // NOTE(milo): The IMU "feels" an upward acceleration due to gravity!
    Vector3 imu_a_rh = TransformUtils.ToRightHandedTranslation(imu_a_total);
    Vector3 imu_w_rh = TransformUtils.ToRightHandedAngularVelocity(imu_w);

    // Optionally add zero-mean noise.
    if (this.accelNoiseSigma > 0 && this.enableImuNoise) {
      imu_a_rh.x += Utils.Gaussian(0, this.accelNoiseSigma);
      imu_a_rh.y += Utils.Gaussian(0, this.accelNoiseSigma);
      imu_a_rh.z += Utils.Gaussian(0, this.accelNoiseSigma);
    }

    if (this.gyroNoiseSigma > 0 && this.enableImuNoise) {
      imu_w_rh.x += Utils.Gaussian(0, this.gyroNoiseSigma);
      imu_w_rh.y += Utils.Gaussian(0, this.gyroNoiseSigma);
      imu_w_rh.z += Utils.Gaussian(0, this.gyroNoiseSigma);
    }

    long nsec = (long)(Time.fixedTime * 1e9);
    this._latest = new ImuMeasurement(nsec, imu_a_rh, imu_w_rh);
  }
}

}
