using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateInPlace : MonoBehaviour
{
  public float degPerSec = 90.0f;
  private Vector3 rotationAxis = new Vector3(0f, 1f, 0f);

  void Update()
  {
    this.transform.Rotate(this.rotationAxis * this.degPerSec * Time.deltaTime);
  }
}
