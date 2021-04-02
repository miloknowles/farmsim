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

	// Preallocated variables.
	Vector3 _force = Vector3.zero;
	Vector3 _torque = Vector3.zero;

	void FixedUpdate()
	{
		if (this.controlMode == ControlMode.DRIVE_COMMMANDS) {
			KeyboardDriveCommand();
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

	public static float ClampAngle(float deg)
	{
    return deg - 360.0f*Mathf.Floor((deg + 180.0f) * (1.0f / 360.0f));
 	}

	private void KeyboardDriveCommand()
	{
		float w_pitch = Input.GetAxis("Vertical") * 1.0f;
		float w_yaw = Input.GetAxis("Horizontal") * 1.0f;
		float thrust = Input.GetKey(KeyCode.Space) ? 3.0f : 0.0f;

		this._force.z = thrust * this.rigidBody.mass;
		this._force.x = 0;
		this._force.y = 0;
		this.rigidBody.AddRelativeForce(this._force);

		this._torque.x = 0.1f*w_pitch;
		this._torque.y = 0.1f*w_yaw;

		// Disable roll, since we always want the vehicle level.
		// TODO(milo): More sophisticated PID control (just P right now).
		Vector3 euler_angles = this.rigidBody.transform.eulerAngles;
		float roll_error = -ClampAngle(euler_angles.z);
		float P_gain = 0.005f;
		float torque_command_roll = P_gain*roll_error;

		this._torque.z = torque_command_roll;
		this.rigidBody.AddRelativeTorque(this._torque);
	}
}
