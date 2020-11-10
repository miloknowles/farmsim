using ROSBridgeLib;
using ROSBridgeLib.CustomMessages;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class DepthPublisher : ROSBridgePublisher {
	public static string GetMessageTopic()
	{
		return "/simulator/sensors/depth";
	}

	public static string GetMessageType()
	{
		return "std_msgs/Float64";
	}

  public static string ToYAMLString(PressureMessage msg)
	{
		return msg.ToYAMLString();
	}
}


public class GroundtruthDepthPublisher : DepthPublisher {
	public static string GetMessageTopic()
	{
		return "/simulator/groundtruth/depth_auv_imu";
	}
}
