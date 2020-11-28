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

	public Rigidbody rigidbody;
	public float keyThrust = 5.0f; 											// Amount of force applied by each key (N).
	public ControlMode controlMode = ControlMode.OFF;		// Turn key commands off by default.

  private ROSBridgeWebSocketConnection _ros = null;

	void Start()
	{
		this._ros = new ROSBridgeWebSocketConnection("ws://localhost", Config.ROS_BRIDGE_PORT);
		this._ros.AddPublisher(typeof(TridentThrustPublisher));
    this._ros.Connect();

		StartCoroutine(ListenForKeyCommands());
	}

	void OnApplicationQuit()
  {
    if (this._ros != null) {
      this._ros.Disconnect();
    }
  }

	// Run the control loop at a low rate to be more lightweight.
	IEnumerator ListenForKeyCommands()
	{
		while (true) {
			yield return new WaitForSeconds(0.2f);
			if (this.controlMode == ControlMode.THRUST_COMMANDS) {
				KeyboardThrustCommand();
			} else if (this.controlMode == ControlMode.DRIVE_COMMMANDS) {
				KeyboardDriveCommand();
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
	 * Send thrust commands to motors.
	 */
	private void KeyboardThrustCommand()
	{
		float Flt = this.keyThrust * SignFromKeyPair(KeyCode.A, KeyCode.Q);
		float Frt = this.keyThrust * SignFromKeyPair(KeyCode.D, KeyCode.E);
		float Fct = this.keyThrust * SignFromKeyPair(KeyCode.S, KeyCode.W);
		TridentThrustMsg msg = new TridentThrustMsg(Flt, Frt, Fct);
		this._ros.Publish(TridentThrustPublisher.GetMessageTopic(), msg);
		this._ros.Render();
	}

	private void KeyboardDriveCommand()
	{

	}
}
