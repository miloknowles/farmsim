using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class ImuPublisher : ROSBridgePublisher {
	public static new string GetMessageTopic()
	{
		return "/sim/imu0";
	}

	public static new string GetMessageType()
	{
		return "sensor_msgs/Imu";
	}

	public static string ToYAMLString(ImuMessage msg)
	{
		return msg.ToYAMLString();
	}
}
