using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class KeyboardMove : MonoBehaviour {
	// Choose how to drive the vehicle.
	public enum ControlMode {
		OFF = 0,
		THRUST_COMMANDS = 1,
		DRIVE_COMMMANDS = 2
	}

	public bool activeRollStabilization = true;

	public Rigidbody rigidBody;
	public float keyThrust = 5.0f; 											// Amount of force applied by each key (N).
	public ControlMode controlMode = ControlMode.OFF;		// Turn key commands off by default.

	void FixedUpdate()
	{
		if (this.controlMode == ControlMode.DRIVE_COMMMANDS) {
			KeyboardDriveCommand();
		}
	}

	// Run the control loop at a low rate to be more lightweight.
	IEnumerator ListenForKeyCommands()
	{
		while (true) {
			yield return new WaitForSeconds(0.2f);
			if (this.controlMode == ControlMode.THRUST_COMMANDS) {
				// KeyboardThrustCommand();
			}
		}
	}

	// Returns -1 or +1 depending on which of two keys is pressed. 0 if neither.
	private float SignFromKeyPair(KeyCode key_positive, KeyCode key_negative)
	{
		if (Input.GetKey(key_positive)) {
			return 1.0f;
		} else if (Input.GetKey(key_negative)) {
			return -1.0f;
		}
		return 0.0f;
	}

	/**
	 * Send thrust commands to motors (through ROS).
	 * NOTE(milo): Need to have rosbridge_server running for this to work!
	 * Command: roslaunch rosbridge_server rosbridge_websocket.launch
	 */
	// private void KeyboardThrustCommand()
	// {
	// 	float Flt = this.keyThrust * SignFromKeyPair(KeyCode.A, KeyCode.Q);
	// 	float Frt = this.keyThrust * SignFromKeyPair(KeyCode.D, KeyCode.E);
	// 	float Fct = this.keyThrust * SignFromKeyPair(KeyCode.S, KeyCode.W);
	// 	TridentThrustMsg msg = new TridentThrustMsg(Flt, Frt, Fct);
	// 	this.roslink.ros.Publish(TridentThrustPublisher.GetMessageTopic(), msg);
	// 	this.roslink.ros.Render();
	// }

	public static float ClampAngle(float deg)
	{
    return deg - 360.0f*Mathf.Floor((deg + 180.0f) * (1.0f / 360.0f));
 	}

	private void KeyboardDriveCommand()
	{
		float w_pitch = Input.GetAxis("Vertical") * 1.0f;
		float w_yaw = Input.GetAxis("Horizontal") * 1.0f;
		float thrust = Input.GetKey(KeyCode.Space) ? 3.0f : 0.0f;

		this.rigidBody.AddRelativeForce(new Vector3(0, 0, thrust) * this.rigidBody.mass);
		this.rigidBody.AddRelativeTorque(new Vector3(0.1f*w_pitch, 0.1f*w_yaw, 0.0f));

		// Disable roll, since we always want the vehicle level.
		// TODO(milo): More sophisticated PID control (just P right now).
		Vector3 euler_angles = this.rigidBody.transform.eulerAngles;
		float roll_error = -ClampAngle(euler_angles.z);
		float P_gain = 0.005f;
		float torque_command_roll = P_gain*roll_error;
		this.rigidBody.AddRelativeTorque(new Vector3(0, 0, torque_command_roll));
	}
}
