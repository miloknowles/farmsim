using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Autotelescope : MonoBehaviour
{
  // Attach the -y end of the cylinder to this object.
  public GameObject endpoint0;
  public Vector3 offset0;

  // Attach the +y end of the cylinder to this object.
  public GameObject endpoint1;
  public Vector3 offset1;

  private float originalScaleX;
  private float originalScaleZ;

  void Start()
  {
    originalScaleX = this.gameObject.transform.localScale.x;
    originalScaleZ = this.gameObject.transform.localScale.z;
  }

  // Update is called once per frame
  void Update()
  {
    // Average of the two endpoints.
    Vector3 midpoint = 0.5f * (endpoint0.transform.position + offset0 + endpoint1.transform.position + offset1);

    Vector3 vector_01 = (endpoint1.transform.position - offset1) - (endpoint0.transform.position + offset0);
    Vector3 unit_01 = Vector3.Normalize(vector_01);
    float length_01 = vector_01.magnitude;

    Vector3 y_axis = new Vector3(0, 1, 0);
    Quaternion q_align_y = RotateAlignVectors(y_axis, unit_01);

    this.gameObject.transform.position = midpoint;
    this.gameObject.transform.rotation = q_align_y;
    this.gameObject.transform.localScale = new Vector3(this.originalScaleX, 0.5f*length_01, this.originalScaleZ);
  }

  // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
  Quaternion RotateAlignVectors(Vector3 v1, Vector3 v2)
  {
    Vector3 hat1 = v1.normalized;
    Vector3 hat2 = v2.normalized;

    Vector3 xyz = Vector3.Cross(hat1, hat2);
    float w = 1 + Vector3.Dot(hat1, hat2);

    Quaternion q = new Quaternion(xyz.x, xyz.y, xyz.z, w).normalized;
    return q;
  }
}
