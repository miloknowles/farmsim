using ROSBridgeLib;
using ROSBridgeLib.CustomMessages;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class TridentThrustCallback : ROSBridgeSubscriber {
  public static TridentThrustMsg _msg;

	public new static string GetMessageTopic() {
		return "/auv/controls/trident_thrust";
	}

	public new static string GetMessageType() {
		return "control/TridentThrust";
	}

	public new static ROSBridgeMsg ParseMessage(JSONNode msg) {
		return new TridentThrustMsg(msg);
	}

	public new static void CallBack(ROSBridgeMsg msg) {
    ROSMessageHolder holder = GameObject.Find("ROSMessageHolder").GetComponent<ROSMessageHolder>();
    // holder.UpdateTridentThrustMsg((TridentThrustMsg)msg);
		Debug.Log("callback");
		holder.UpdateTopic(TridentThrustPublisher.GetMessageTopic(), msg);
	}
}
