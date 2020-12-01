using ROSBridgeLib;
using ROSBridgeLib.CustomMessages;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class TridentThrustPublisher : ROSBridgePublisher {
	public static new string GetMessageTopic()
	{
		return "/auv/controls/trident_thrust";
	}

	public static new string GetMessageType()
	{
		return "auv/TridentThrust";
	}

	public static string ToYAMLString(TridentThrustMsg msg)
	{
		return msg.ToYAMLString();
	}
}
