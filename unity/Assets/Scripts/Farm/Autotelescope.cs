using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Add this to a cylinder as a cheap way to simulate elasticity.
// The object with lengthen so that its endpoints stay near the specified game objects.
// IMPORTANT: This object that this script is attached to should position itself exactly at the
// midpoint of the endpoint objects.
public class Autotelescope : MonoBehaviour
{
  public enum LongitudinalAxis { X, Y, Z }

  public GameObject telescope;

  // Attach the - end of the cylinder to this object.
  public GameObject endpoint0;
  public Vector3 offset0;

  // Attach the + end of the cylinder to this object.
  public GameObject endpoint1;
  public Vector3 offset1;

  public LongitudinalAxis longitudinalAxis = LongitudinalAxis.Y;

  private float originalScaleX;
  private float originalScaleZ;
  // private Vector3 originalPosition0;
  // private Vector3 originalPosition1;

  // By default, assume cylinder is aligned with the +y axis.
  private Vector3 alignVector = new Vector3(0, 1, 0);

  // Preallocated variables.
  private Vector3 _midpoint;
  private Vector3 _localScale = Vector3.zero;

  void Start()
  {
    this.originalScaleX = telescope.transform.localScale.x;
    this.originalScaleZ = telescope.transform.localScale.z;

    if (this.longitudinalAxis == LongitudinalAxis.X) {
      this.alignVector = new Vector3(1, 0, 0);
    } else if (this.longitudinalAxis == LongitudinalAxis.Z) {
      this.alignVector = new Vector3(0, 0, 1);
    }
  }

  void Update()
  {
    // Average of the two endpoints.
    this._midpoint = 0.5f * (endpoint0.transform.position + offset0 + endpoint1.transform.position + offset1);

    Vector3 vector_01 = (endpoint1.transform.position - offset1) - (endpoint0.transform.position + offset0);
    Vector3 unit_01 = Vector3.Normalize(vector_01);
    float length_01 = vector_01.magnitude;

    Quaternion q_align_long = Simulator.TransformUtils.RotateAlignVectors(alignVector, unit_01);

    // Position the attached game object at the center of the two endpoints.
    this.gameObject.transform.SetPositionAndRotation(this._midpoint, q_align_long);

    // Make the attached telescope scale to fit between the endpoints.
    // NOTE(milo): Should be 0.5f * scale for cylinders!
    this._localScale.x = this.originalScaleX;
    this._localScale.y = length_01;
    this._localScale.z = this.originalScaleZ;
    telescope.transform.localScale = this._localScale;
  }
}
