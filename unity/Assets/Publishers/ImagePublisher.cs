using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class ImagePublisher : ROSBridgePublisher {
	public static new string GetMessageTopic()
	{
		return "/image";
	}

	public static new string GetMessageType()
	{
		return "sensor_msgs/Image";
	}

	public static string ToYAMLString(ImageMsg msg)
	{
		return msg.ToYAMLString();
	}
}


public class StereoCamLeftPublisher : ImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/cam0/image_raw";
	}
}


public class StereoCamRightPublisher : ImagePublisher {
	public static new string GetMessageTopic()
	{
		return "/cam1/image_raw";
	}
}
