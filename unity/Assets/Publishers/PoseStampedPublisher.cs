using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class PoseStampedPublisher : ROSBridgePublisher {
	public static new string GetMessageTopic()
	{
		return "/simulator/groundtruth/pose_auv_imu";
	}

	public static new string GetMessageType()
	{
		return "geometry_msgs/PoseStamped";
	}

	public static string ToYAMLString(ImuMessage msg)
	{
		return msg.ToYAMLString();
	}
}
