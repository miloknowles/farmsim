using System;
using UnityEngine;
using System.Collections;


public class VehicleDynamics : MonoBehaviour {
  private Rigidbody rigidbody;
  private float F_lt = 0.0f;  // Force of left thruster.
  private float F_rt = 0.0f;  // Force of right thruster.
  private float F_ct = 0.0f;  // Force of center thruster.

  public float C_drag_body = 0.5f * 1027.0f * 1.0f * 0.1f;   // Lump terms: 1/2 * rho * Cd * A
  public float C_angular_drag_body = 1.0f; // Drag = Cd * w^2

  private Vector3 t_lt_body = new Vector3(-0.15f, 0.0f, -0.3f);
  private Vector3 t_rt_body = new Vector3(0.15f, 0.0f, -0.3f);
  private Vector3 t_ct_body = new Vector3(0.0f, 0.0f, 0.2f);
  private Vector3 t_CP_body = new Vector3(0.0f, 0.0f, -0.05f);

  void Start()
  {
    this.rigidbody = this.GetComponent<Rigidbody>();
  }

  void Update()
  {
    // Keys: Up = +1, Down = -1;
		float forwardBackwardSign = Input.GetAxis("Vertical");

		// Keys: Right = +1, Left = -1
		float leftRightSign = Input.GetAxis("Horizontal");

    this.F_lt = (leftRightSign < 0) ? 100.0f : 0.0f;
    this.F_rt = (leftRightSign > 0) ? 100.0f : 0.0f;
    this.F_ct = -forwardBackwardSign * 100.0f;
  }

  void FixedUpdate()
  {
    Vector3 v_body_world = this.rigidbody.velocity;
    Vector3 w_body_world = this.rigidbody.angularVelocity;

    // Rear motors create forward (+z thrust).
    Vector3 f_lt = new Vector3(0.0f, 0.0f, this.F_lt);
    Vector3 f_rt = new Vector3(0.0f, 0.0f, this.F_rt);

    // Center motor creates upward (+y) thrust.
    Vector3 f_ct = new Vector3(0.0f, this.F_ct, 0.0f);

    // Drag = 1/2 * rho * Cd * A * v^2
    Vector3 F_drag = -1.0f * this.C_drag_body * Vector3.Scale(v_body_world, v_body_world);

    // Compute torques due to motor thrust.
    Vector3 tau_lt = Vector3.Cross(this.t_lt_body, f_lt);
    Vector3 tau_rt = Vector3.Cross(this.t_rt_body, f_rt);
    Vector3 tau_ct = Vector3.Cross(this.t_ct_body, f_ct);

    // Compute torque due to drag.
    Vector3 tau_linear_drag = Vector3.Cross(this.t_CP_body, F_drag);
    Vector3 tau_angular_drag = this.C_angular_drag_body * Vector3.Scale(w_body_world, w_body_world);

    // Apply forces and torques to the vehicle.
    this.rigidbody.AddRelativeForce(f_lt + f_rt + f_ct + F_drag);
    this.rigidbody.AddRelativeTorque(tau_lt + tau_rt + tau_ct + tau_linear_drag + tau_angular_drag);
  }
}
