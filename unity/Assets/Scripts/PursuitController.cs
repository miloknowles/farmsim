using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Simulator;

public class PursuitController : MonoBehaviour
{
  public Rigidbody rigidBody;
  public GameObject followObject;
  public float pGainPitch = 0.03f;
  public float pGainYaw = 0.03f;
  public float pGainThrust = 3.0f;
  private Vector3 unit_forward = new Vector3(0f, 0f, 1f);
  private Vector3 world_t_goal = Vector3.zero;
  private Vector3 body_t_goal = Vector3.zero;
  private Quaternion body_q_goal = Quaternion.identity;
  private Vector3 body_euler_goal = Vector3.zero;
  private Vector3 torque_command = Vector3.zero;
  private Vector3 thrust_command = Vector3.zero;

  // Start is called before the first frame update
  void Start()
  {

  }

  public static float ClampAngle(float deg)
	{
    return deg - 360.0f*Mathf.Floor((deg + 180.0f) * (1.0f / 360.0f));
 	}


  public float CalcForwardVelocity(float angle)
  {
    if (angle >= 30.0f) {
      return 0.0f;
    }
    // Goes 2 m/s at 0 angle error.
    return 2.0f * (30.0f - angle) / 30.0f;
  }

  // Update is called once per frame
  void Update()
  {
    if (this.followObject == null) {
      return;
    }

    this.world_t_goal = this.followObject.transform.position;

    this.body_t_goal = this.transform.InverseTransformPoint(this.world_t_goal);
    this.body_q_goal = TransformUtils.RotateAlignVectors(unit_forward, body_t_goal);
    this.body_euler_goal = this.body_q_goal.eulerAngles;

    // Apply torques to align with the goal.
    float pitch_error = ClampAngle(this.body_euler_goal.x);
    float yaw_error = ClampAngle(this.body_euler_goal.y);

		this.torque_command.x = this.pGainPitch*pitch_error;
    this.torque_command.y = this.pGainYaw*yaw_error;
    this.torque_command.z = 0.0f;

		this.rigidBody.AddRelativeTorque(this.torque_command);

    // Forward velocity proportional to angular distance from goal.
    float desired_vel = CalcForwardVelocity(Quaternion.Angle(this.body_q_goal, Quaternion.identity));
    float vel_error = desired_vel - this.rigidBody.velocity.magnitude;

    this.thrust_command.z = this.pGainThrust * vel_error;
    this.rigidBody.AddRelativeForce(this.thrust_command);
  }
}
