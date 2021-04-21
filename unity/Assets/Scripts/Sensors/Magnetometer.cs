using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {


// Measures a magnetic field "bM" in the body frame.
// Reference: https://en.wikipedia.org/wiki/Magnetometer#Types_of_magnetometer
// bM is measurement in "tesla" units, typically in the 20k-80k nT range.
// To keep the numerical range reasonable, using uT (microtesla), so that we're in the 20-80 range.
public struct MagMeasurement
{
  public MagMeasurement(long timestamp, Vector3 bM)
  {
    this.timestamp = timestamp;
    this.bM = bM;                 // Expressed in right-handed body frame (RDF).
  }

  public long timestamp;
  public Vector3 bM;
};


/**
 * Simulates measurements from a vector magnetometer.
 *
 * The magnetometer measurements are modelled as:
 *
 * body_M = scale * body_R_nav * direction + bias + noise
 *   - body_M is the measured 3D magnetic field vector
 *   - scale is 3x3 matrix that scales a unit vector to magnetometer units
 *   - body_R_nav is the rotation of the "nav" frame in the "body" frame
 *   - direction is the true magnetic field unit vector
 *   - bias is 3D vector that offsets readings
 *   - noise is 3D gaussian noise
 *
 * In Unity3D, the "nav" or "world" frame follows a "Right-Up-Forward" convention.
 * We assign North to the z axis and East to the x axis.
 */
public class Magnetometer : MonoBehaviour
{
  public GameObject body;   // Rigidbody that the magnetometer is attached to.

  public bool enableNoise = true;
  public bool enableBias = true;

  public float noiseSigma = 0.001f;
  public float biasSigma = 0.001f;

  public Vector3 fieldDirection = new Vector3(0, 0, 1);   // Points North (z) by default.
  public float fieldStrength = 50.0f; // In microtesla (uT).
  private Vector3 biasVector = Vector3.zero;
  public MagMeasurement data;
  private Vector3 noiseSigmaVector;

  void Start()
  {
    if (this.enableBias && this.biasSigma > 0) {
      this.biasVector += Gaussian.Sample3D(Vector3.zero, new Vector3(this.biasSigma, this.biasSigma, this.biasSigma));
    }

    this.data = new MagMeasurement(0, this.fieldStrength * this.fieldDirection);
    this.noiseSigmaVector = new Vector3(this.noiseSigma, this.noiseSigma, this.noiseSigma);
  }

  // Call Read() to store the latest measurement, then access it using the "data" property.
  public void Read()
  {
    this.data.timestamp = Timestamp.UnityNanoseconds();

    this.data.bM = this.fieldStrength * (this.body.transform.rotation * this.fieldDirection) + this.biasVector;
    TransformUtils.ToRightHandedTranslation(this.data.bM, ref this.data.bM);

    if (this.enableNoise) {
      this.data.bM += Gaussian.Sample3D(Vector3.zero, this.noiseSigmaVector);
    }
  }
}

}
