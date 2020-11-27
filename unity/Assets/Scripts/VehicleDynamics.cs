using System;
using UnityEngine;
using System.Collections;


public class VehicleDynamics : MonoBehaviour {
  private Rigidbody rigidbody;
  private float F_lt = 0.0f;  // Force of left thruster.
  private float F_rt = 0.0f;  // Force of right thruster.
  private float F_ct = 0.0f;  // Force of center thruster.

  public float linearDragCoefficient = 0.5f * 1027.0f * 0.1f * 0.1f;   // Lump terms: 1/2 * rho * Cd * A
  public float angularDragCoefficient = 0.01f; // Drag = Cd * w^2

  private Vector3 t_lt_body = new Vector3(-0.1f, 0.0f, -0.2f);
  private Vector3 t_rt_body = new Vector3(0.1f, 0.0f, -0.2f);
  private Vector3 t_ct_body = new Vector3(0.0f, 0.0f, 0.05f);
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

    this.F_lt = (leftRightSign < -0.5f) ? 5.0f : 0.0f;
    this.F_rt = (leftRightSign > 0.5f) ? 5.0f : 0.0f;
    this.F_ct = (Mathf.Abs(forwardBackwardSign) > 0.5f) ? -forwardBackwardSign * 10.0f : 0.0f;
  }

  void FixedUpdate()
  {
    // Get velocity and angular velocity in the body frame.
    Vector3 v_body = this.transform.InverseTransformDirection(this.rigidbody.velocity);
    Vector3 w_body = this.transform.InverseTransformDirection(this.rigidbody.angularVelocity);

    Debug.Log(w_body);

    // Rear motors create forward (+z thrust).
    Vector3 f_lt = new Vector3(0.0f, 0.0f, this.F_lt);
    Vector3 f_rt = new Vector3(0.0f, 0.0f, this.F_rt);

    // Center motor creates upward (+y) thrust.
    Vector3 f_ct = new Vector3(0.0f, this.F_ct, 0.0f);

    // Drag = 1/2 * rho * Cd * A * v^2
    Vector3 F_drag = -1.0f * v_body.normalized * this.linearDragCoefficient * Mathf.Pow(v_body.magnitude, 2);

    // NOTE(milo): Need to clamp drag force for stability. Make sure that the force can't causes
    // an acceration that would change the direction of motion. This prevents drag forces from
    // wild positive feedback effects.
    // dv = a * dt = (F / m) * dt < Magnitude(v)
    F_drag = Vector3.ClampMagnitude(F_drag, v_body.magnitude * this.rigidbody.mass * Time.fixedDeltaTime);

    // Compute torques due to motor thrust.
    Vector3 tau_lt = Vector3.Cross(this.t_lt_body, f_lt);
    Vector3 tau_rt = Vector3.Cross(this.t_rt_body, f_rt);
    Vector3 tau_ct = Vector3.Cross(this.t_ct_body, f_ct);

    // Compute torque due to drag.
    Vector3 tau_linear_drag = Vector3.Cross(this.t_CP_body, F_drag);
    Vector3 tau_angular_drag = -1.0f * w_body.normalized * this.angularDragCoefficient * Mathf.Pow(w_body.magnitude, 2);

    // Apply forces and torques to the vehicle in its body frame.
    this.rigidbody.AddRelativeForce(f_lt + f_rt + f_ct + F_drag);
    this.rigidbody.AddRelativeTorque(tau_lt + tau_rt + tau_ct + tau_angular_drag);
  }
}
