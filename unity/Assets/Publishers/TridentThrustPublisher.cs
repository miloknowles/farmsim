using ROSBridgeLib;
using ROSBridgeLib.CustomMessages;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class TridentThrustPublisher : ROSBridgePublisher {
	public static string GetMessageTopic()
	{
		return "/auv/controls/trident_thrust";
	}

	public static string GetMessageType()
	{
		return "control/TridentThrust";
	}

	public static string ToYAMLString(TridentThrustMsg msg)
	{
		return msg.ToYAMLString();
	}
}
