using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Simulator {

public class SharkController : MonoBehaviour {
  public Vector3 relativeOrbitCenter = new Vector3(-20f, 0f, 0f);
  public float orbitPeriod = 60.0f;  // time to complete an orbit (sec)

  private float radius = 20.0f;
  private Vector3 eulerInitial = new Vector3(0f, 0f, 0f);
  private Vector3 worldOrbitCenter = new Vector3(0f, 0f, 0f);
  private Quaternion world_q_body = new Quaternion();

  void Start()
  {
    this.worldOrbitCenter = this.transform.TransformPoint(this.relativeOrbitCenter);
    this.radius = this.relativeOrbitCenter.magnitude;
    this.world_q_body = this.transform.rotation;
    this.eulerInitial = this.transform.eulerAngles;
  }

  void Update()
  {
    float omega = 2.0f*Mathf.PI / this.orbitPeriod;
    float theta = omega * (float)Timestamp.UnitySeconds();

    // Get the RELATIVE offset from center in ORIGINAL frame.
    float body_x = this.radius * Mathf.Cos(theta);
    float body_z = this.radius * Mathf.Sin(theta);

    Vector3 world_t_body = this.worldOrbitCenter + this.world_q_body * new Vector3(body_x, 0.0f, body_z);

    this.transform.position = world_t_body;
    Vector3 newEuler = this.eulerInitial;
    newEuler.y -= 180.0f * theta / Mathf.PI;  // Convert to radians.
    this.transform.eulerAngles = newEuler;
  }
}

}
