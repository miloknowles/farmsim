using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Simulator;

public class PursuitController : MonoBehaviour
{
  public Rigidbody rigidBody;
  public GameObject followObject;
  public WaypointSequence waypointSeq;
  public float pGainPitch = 0.03f;
  public float pGainYaw = 0.02f;
  public float pGainThrust = 10.0f;
  private Vector3 unit_forward = new Vector3(0f, 0f, 1f);
  private Vector3 world_t_goal = Vector3.zero;
  private Vector3 body_t_goal = Vector3.zero;
  private Quaternion body_q_goal = Quaternion.identity;
  private Vector3 body_euler_goal = Vector3.zero;
  private Vector3 torque_command = Vector3.zero;
  private Vector3 thrust_command = Vector3.zero;
  private float turnInPlaceAngle = 20.0f;
  private float maxForwardVelocity = 5.0f;
  private float goalPositionTol = 0.2f;
  private float distVelocityGain = 1.5f;
  private float maxThrust = 30.0f;

  void Start()
  {
    // Check if we're following a single waypoint or circuit.
    if (this.waypointSeq != null) {
      Debug.Log("Attached to waypoint sequence, going to first object");
      this.followObject = this.waypointSeq.GetCurrentWaypoint();
    }
  }

  public static float ClampAngle(float deg)
	{
    return deg - 360.0f*Mathf.Floor((deg + 180.0f) * (1.0f / 360.0f));
 	}


  // Max velocity when pointing directly at target, zero velocity if > turnInPlaceAngle error.
  public float CalcForwardVelocityFromAngle(float angle)
  {
    if (angle >= this.turnInPlaceAngle) {
      return 0.0f;
    }
    return this.maxForwardVelocity * (turnInPlaceAngle - angle) / turnInPlaceAngle;
  }

  // Set velocity to zero if at target.
  public float CalcForwardVelocityFromDistance(Vector3 body_t_goal)
  {
    float dist = body_t_goal.magnitude;
    float mag = Mathf.Min(this.maxForwardVelocity, this.distVelocityGain*dist);
    float sign = body_t_goal.z >= 0.0f ? 1.0f : -1.0f;
    return sign * mag;
  }

  private void MaybeUpdateWaypoint()
  {
    if (this.waypointSeq == null) {
      return;
    }

    this.world_t_goal = this.followObject.transform.position;
    this.body_t_goal = this.transform.InverseTransformPoint(this.world_t_goal);
    float distance_to_goal = this.body_t_goal.magnitude;

    if (distance_to_goal < this.goalPositionTol) {
      Debug.Log("Going to next waypoint");
      this.followObject = this.waypointSeq.GetNextWaypoint();
    }
  }

  // Update is called once per frame
  void Update()
  {
    if (this.followObject == null) {
      return;
    }

    // If following a waypoint circuit, check if we've reached the current waypoint.
    MaybeUpdateWaypoint();

    this.world_t_goal = this.followObject.transform.position;
    this.body_t_goal = this.transform.InverseTransformPoint(this.world_t_goal);
    this.body_q_goal = TransformUtils.RotateAlignVectors(this.unit_forward, this.body_t_goal);

    float distance_to_goal = this.body_t_goal.magnitude;

    // If at goal, rotate in place to desired orientation.
    if (distance_to_goal < this.goalPositionTol && this.rigidBody.velocity.magnitude < 0.1f) {
      // Compute: body_q_goal = body_q_world * world_q_goal;
      this.body_q_goal = Quaternion.Inverse(this.transform.rotation) * this.followObject.transform.rotation;
    }

    this.body_euler_goal = this.body_q_goal.eulerAngles;

    // Apply torques to align with the goal.
    float pitch_error = ClampAngle(this.body_euler_goal.x);
    float yaw_error = ClampAngle(this.body_euler_goal.y);

		this.torque_command.x = this.pGainPitch*pitch_error;
    this.torque_command.y = this.pGainYaw*yaw_error;
    this.torque_command.z = 0.0f;

		this.rigidBody.AddRelativeTorque(this.torque_command);

    // Forward velocity proportional to angular distance from goal.
    float desired_vel_ang = CalcForwardVelocityFromAngle(Quaternion.Angle(this.body_q_goal, Quaternion.identity));

    // Forward velocity proportional to metric distance from goal.
    float desired_vel_dist = CalcForwardVelocityFromDistance(this.body_t_goal);
    float desired_vel_z = Mathf.Min(desired_vel_ang, desired_vel_dist);

    // Track the desired velocity.
    float vel_error_z = desired_vel_z - this.transform.InverseTransformDirection(this.rigidBody.velocity).z;

    // Track the desired depth using y-axis thrust.
    float desired_vel_y = 0.2f * this.body_t_goal.y;
    float error_vel_y = desired_vel_y - this.transform.InverseTransformDirection(this.rigidBody.velocity).y;

    this.thrust_command.x = 0.0f;
    this.thrust_command.y = this.pGainThrust * error_vel_y;
    this.thrust_command.z = this.pGainThrust * vel_error_z;
    this.thrust_command = Vector3.ClampMagnitude(this.thrust_command, this.maxThrust);
    this.rigidBody.AddRelativeForce(this.thrust_command);
  }
}
