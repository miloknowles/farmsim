using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class KeyboardMove : MonoBehaviour {
	public Rigidbody controlledRigidBody;

	// Force applied when moving forwards.
	public float surgeThrustForce = 20000f;

	// Force applied when moving sideways.
	public float swayThrustForce = 20000f;

	// Force applied when moving up and down.
	public float heaveThrustForce = 20000f;

	// Torque applied when yawing.
	public float yawTorque = 20000.0f;
	public float rollTorque = 1000.0f;
	public float pitchTorque = 10000.0f;

	void Update()
	{
		// Keys: Up = +1, Down = -1;
		float forwardBackwardSign = Input.GetAxis("Vertical");

		// Keys: Right = +1, Left = -1
		float leftRightSign = Input.GetAxis("Horizontal");

		// The "+" key goes towards surface, "-" key goes downward.
		float upDownSign = 0.0f;
		if (Input.GetKey("=")) {
			upDownSign = 1;
		} else if (Input.GetKey("-")) {
			upDownSign = -1;
		}

		float yawSign = 0.0f;
		if (Input.GetKey("a")) {
			yawSign = -1;
		} else if (Input.GetKey("d")) {
			yawSign = 1;
		}

		float rollSign = 0.0f;
		if (Input.GetKey("q")) {
			rollSign = 1;
		} else if (Input.GetKey("e")) {
			rollSign = -1;
		}

		float pitchSign = 0.0f;
		if (Input.GetKey("s")) {
			pitchSign = -1;
		} else if (Input.GetKey("w")) {
			pitchSign = 1;
		}

		float roll = controlledRigidBody.transform.eulerAngles.z;
		float pitch = controlledRigidBody.transform.eulerAngles.x;
		float yaw = controlledRigidBody.transform.eulerAngles.y;

		// NOTE(milo): Vehicle uses a "RUF" (Right-Up-Forward) coordinate frame.
		Vector3 forceVector = new Vector3(leftRightSign * swayThrustForce,
																			upDownSign * heaveThrustForce,
																			forwardBackwardSign * surgeThrustForce);

		// Apply force in the controlledRigidBody frame.
		controlledRigidBody.AddRelativeForce(forceVector);

		// NOTE(milo): Vector3.forward makes a unit Z vector.
		controlledRigidBody.AddRelativeTorque(new Vector3(pitchSign * pitchTorque, yawSign * yawTorque, rollSign * rollTorque));
	}
}
