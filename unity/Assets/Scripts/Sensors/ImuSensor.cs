using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Simulator {

public struct ImuMeasurement
{
  public ImuMeasurement(long timestamp, Vector3 a, Vector3 w)
  {
    this.timestamp = timestamp;
    this.imu_a_rh = a;
    this.imu_w_rh = w;
  }

  public long timestamp;
  public Vector3 imu_a_rh;
  public Vector3 imu_w_rh;
};


// Simulates measurements from an IMU (accelerometer and gyroscope).
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

  private Vector3 accelBias = Vector3.zero;
  private Vector3 gyroBias = Vector3.zero;
  private Vector3 accelNoiseSigmaVec;
  private Vector3 gyroNoiseSigmaVec;

  // Used to calculate accleration with finite-differencing.
  private Vector3 prev_world_v_imu;

  public ImuMeasurement data = new ImuMeasurement(0, Vector3.zero, Vector3.zero);

  void Start()
  {
    // For now, just sample a bias when the simulation starts, and hold it constant throughout.
    if (this.accelBiasSigma > 0 && this.enableImuBias) {
      this.accelBias += Gaussian.Sample3D(Vector3.zero, new Vector3(this.accelBiasSigma, this.accelBiasSigma, this.accelBiasSigma));
    }

    if (this.gyroBiasSigma > 0 && this.enableImuBias) {
      this.gyroBias += Gaussian.Sample3D(Vector3.zero, new Vector3(this.gyroBiasSigma, this.gyroBiasSigma, this.gyroBiasSigma));
    }

    this.accelNoiseSigmaVec = new Vector3(this.accelNoiseSigma, this.accelNoiseSigma, this.accelNoiseSigma);
    this.gyroNoiseSigmaVec = new Vector3(this.gyroNoiseSigma, this.gyroNoiseSigma, this.gyroNoiseSigma);

    Debug.Log($"** [ImuSensor] Accelerometer bias (m/s^2): {this.accelBias.x} {this.accelBias.y} {this.accelBias.z}");
    Debug.Log($"** [ImuSensor] Gyroscope bias: (rad/s): {this.gyroBias.x} {this.gyroBias.y} {this.gyroBias.z}");
  }

  void FixedUpdate()
  {
    // Rotation from the world to the local IMU frame.
    Quaternion imu_q_world = Quaternion.Inverse(this.imu_rigidbody.transform.rotation);

    // NOTE(milo): Really important to do velocity-differencing in the WORLD frame!
    Vector3 world_a_imu = (this.imu_rigidbody.velocity - this.prev_world_v_imu) / Time.fixedDeltaTime;
    Vector3 imu_a = imu_q_world * world_a_imu;
    this.prev_world_v_imu = this.imu_rigidbody.velocity;

    Vector3 imu_w = imu_q_world * this.imu_rigidbody.angularVelocity;

    // Rotate the gravity vector into the IMU's frame, then add it to acceleration.
    Vector3 imu_a_gravity = imu_q_world * Physics.gravity;
    Vector3 imu_a_total = imu_a - imu_a_gravity;

    // NOTE(milo): The IMU "feels" an upward acceleration due to gravity!
    TransformUtils.ToRightHandedTranslation(imu_a_total, ref data.imu_a_rh);
    TransformUtils.ToRightHandedAngularVelocity(imu_w, ref data.imu_w_rh);

    // Optionally add zero-mean noise.
    if (this.accelNoiseSigma > 0 && this.enableImuNoise) {
      data.imu_a_rh += Gaussian.Sample3D(Vector3.zero, this.accelNoiseSigmaVec);
    }

    if (this.gyroNoiseSigma > 0 && this.enableImuNoise) {
      data.imu_w_rh += Gaussian.Sample3D(Vector3.zero, this.accelNoiseSigmaVec);
    }

    // Add bias. If enableImuBias is OFF, this will just add a zero vector.
    data.imu_a_rh += this.accelBias;
    data.imu_w_rh += this.gyroBias;

    data.timestamp = Timestamp.UnityNanoseconds();
  }
}

}
