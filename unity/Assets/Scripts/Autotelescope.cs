using System.Collections;
using System.Collections.Generic;
using UnityEngine;


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

  void Start()
  {
    originalScaleX = telescope.transform.localScale.x;
    originalScaleZ = telescope.transform.localScale.z;

    if (longitudinalAxis == LongitudinalAxis.X) {
      alignVector = new Vector3(1, 0, 0);
    } else if (longitudinalAxis == LongitudinalAxis.Z) {
      alignVector = new Vector3(0, 0, 1);
    }
  }

  void Update()
  {
    // Average of the two endpoints.
    Vector3 midpoint = 0.5f * (endpoint0.transform.position + offset0 + endpoint1.transform.position + offset1);

    Vector3 vector_01 = (endpoint1.transform.position - offset1) - (endpoint0.transform.position + offset0);
    Vector3 unit_01 = Vector3.Normalize(vector_01);
    float length_01 = vector_01.magnitude;

    Quaternion q_align_long = Simulator.Utils.RotateAlignVectors(alignVector, unit_01);

    // Position the attached game object at the center of the two endpoints.
    this.gameObject.transform.position = midpoint;
    this.gameObject.transform.rotation = q_align_long;

    // Make the attached telescope scale to fit between the endpoints.
    telescope.transform.localScale = new Vector3(this.originalScaleX, 0.5f*length_01, this.originalScaleZ);
  }
}
