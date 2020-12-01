using System;
using UnityEngine;
using System.Collections;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.sensor_msgs;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.CustomMessages;
using ROSBridgeLib;


using ROSCallback = System.Action<ROSBridgeMsg>;

public class VehicleDynamics : MonoBehaviour {
  private Rigidbody rigidBody;
  private float _F_lt = 0.0f;  // Force of left thruster.
  private float _F_rt = 0.0f;  // Force of right thruster.
  private float _F_ct = 0.0f;  // Force of center thruster.

  // Assume Cd of cube: https://www.engineeringtoolbox.com/drag-coefficient-d_627.html
  // Lump terms: 1/2 * rho * Cd * A
  private float linearDragCoefficient = 0.5f * 1027.0f * 0.9f * (0.08f * 0.201f * 0.41f);
  public float angularDragCoefficient = 0.7f; // Drag = Cd * w^2
  public float maxThrust = 10.0f; // N

  private Vector3 t_lt_body = new Vector3(-0.1f, 0.0f, -0.2f);
  private Vector3 t_rt_body = new Vector3(0.1f, 0.0f, -0.2f);
  private Vector3 t_ct_body = new Vector3(0.0f, 0.0f, 0.05f);
  private Vector3 t_CP_body = new Vector3(0.0f, 0.0f, -0.01f);

  private ROSMessageHolder roslink;

  void Start()
  {
    this.rigidBody = this.GetComponent<Rigidbody>();

    this.roslink = GameObject.Find("ROSMessageHolder").GetComponent<ROSMessageHolder>();
    this.roslink.RegisterCallback(TridentThrustCallback.GetMessageTopic(), this.Callback);
    this.roslink.ros.AddSubscriber(typeof(TridentThrustCallback));
  }

  void Update()
  {
    this.roslink.ros.Render();
  }

  void FixedUpdate()
  {
    // Get velocity and angular velocity in the body frame.
    Vector3 v_body = this.transform.InverseTransformDirection(this.rigidBody.velocity);
    Vector3 w_body = this.transform.InverseTransformDirection(this.rigidBody.angularVelocity);

    // Rear motors create forward (+z thrust).
    Vector3 flt = new Vector3(0.0f, 0.0f, this._F_lt);
    Vector3 frt = new Vector3(0.0f, 0.0f, this._F_rt);

    // Center motor creates upward (+y) thrust.
    Vector3 fct = new Vector3(0.0f, this._F_ct, 0.0f);

    // Drag = 1/2 * rho * Cd * A * v^2
    Vector3 F_drag = -1.0f * v_body.normalized * this.linearDragCoefficient * Mathf.Pow(v_body.magnitude, 2);

    // NOTE(milo): Need to clamp drag force for stability. Make sure that the force can't causes
    // an acceration that would change the direction of motion. This prevents drag forces from
    // wild positive feedback effects.
    // dv = a * dt = (F / m) * dt < Magnitude(v)
    F_drag = Vector3.ClampMagnitude(F_drag, v_body.magnitude * this.rigidBody.mass / Time.fixedDeltaTime);

    // Compute torques due to motor thrust.
    Vector3 tau_lt = Vector3.Cross(this.t_lt_body, flt);
    Vector3 tau_rt = Vector3.Cross(this.t_rt_body, frt);
    Vector3 tau_ct = Vector3.Cross(this.t_ct_body, fct);

    // Compute torque due to drag.
    Vector3 tau_linear_drag = Vector3.Cross(this.t_CP_body, F_drag);
    Vector3 tau_angular_drag = -1.0f * w_body.normalized * this.angularDragCoefficient * Mathf.Pow(w_body.magnitude, 2);

    // Apply forces and torques to the vehicle in its body frame.
    this.rigidBody.AddRelativeForce(flt + frt + fct + F_drag);
    this.rigidBody.AddRelativeTorque(tau_lt + tau_rt + tau_ct + tau_linear_drag + tau_angular_drag);
  }

  public void Callback(ROSBridgeMsg msg)
  {
    TridentThrustMsg typed = (TridentThrustMsg)msg;
    this._F_lt = Mathf.Clamp(typed.GetFlt(), -this.maxThrust, this.maxThrust);
    this._F_rt = Mathf.Clamp(typed.GetFrt(), -this.maxThrust, this.maxThrust);
    this._F_ct = Mathf.Clamp(typed.GetFct(), -this.maxThrust, this.maxThrust);
  }
}
