using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class CompressedImagePublisher : ROSBridgePublisher {
	public static new string GetMessageTopic()
	{
		return "/image/compressed";
	}

	public static new string GetMessageType()
	{
		return "sensor_msgs/CompressedImage";
	}

	public static string ToYAMLString(CompressedImageMsg msg)
	{
		return msg.ToYAMLString();
	}
}


public class StereoCamLeftPublisherCmp : CompressedImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/sim/cam0/compressed";
	}
}


public class StereoCamRightPublisherCmp : CompressedImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/sim/cam1/compressed";
	}
}
