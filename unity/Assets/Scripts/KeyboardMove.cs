using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ROSBridgeLib;
using ROSBridgeLib.CustomMessages;


public class KeyboardMove : MonoBehaviour {
	// Choose how to drive the vehicle.
	public enum ControlMode {
		OFF = 0,
		THRUST_COMMANDS = 1,
		DRIVE_COMMMANDS = 2
	}

	public Rigidbody rigidBody;
	public float keyThrust = 5.0f; 											// Amount of force applied by each key (N).
	public ControlMode controlMode = ControlMode.OFF;		// Turn key commands off by default.
	private ROSMessageHolder roslink;

	void Start()
	{
		this.roslink = GameObject.Find("ROSMessageHolder").GetComponent<ROSMessageHolder>();
		this.roslink.ros.AddPublisher(typeof(TridentThrustPublisher));
		StartCoroutine(ListenForKeyCommands());
	}

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
				KeyboardThrustCommand();
			}
		}
	}

	/**
	 * Returns -1 or +1 depending on which of two keys is pressed. 0 if neither.
	 */
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
	 */
	private void KeyboardThrustCommand()
	{
		float Flt = this.keyThrust * SignFromKeyPair(KeyCode.A, KeyCode.Q);
		float Frt = this.keyThrust * SignFromKeyPair(KeyCode.D, KeyCode.E);
		float Fct = this.keyThrust * SignFromKeyPair(KeyCode.S, KeyCode.W);
		TridentThrustMsg msg = new TridentThrustMsg(Flt, Frt, Fct);
		this.roslink.ros.Publish(TridentThrustPublisher.GetMessageTopic(), msg);
		this.roslink.ros.Render();
	}

	private void KeyboardDriveCommand()
	{
		float w_pitch = Input.GetAxis("Vertical") * 5.0f;
		float w_yaw = Input.GetAxis("Horizontal") * 5.0f;
		float thrust = Input.GetKey(KeyCode.Space) ? 3.0f : 0.0f;

		// this.rigidBody.transform.Translate(0, 0, speed * Time.fixedDeltaTime);
		// this.rigidBody.transform.Rotate(w_pitch * Time.fixedDeltaTime, w_yaw * Time.fixedDeltaTime, 0.0f);

		this.rigidBody.AddRelativeForce(new Vector3(0, 0, thrust) * this.rigidBody.mass);
		this.rigidBody.AddRelativeTorque(new Vector3(w_pitch, w_yaw, 0.0f));

		// Disable roll, since we always want the vehicle level.
		Vector3 euler = this.rigidBody.transform.eulerAngles;
		this.rigidBody.transform.eulerAngles = new Vector3(euler.x, euler.y, 0.0f);
	}
}
