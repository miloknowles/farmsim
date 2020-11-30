using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;


public class CompressedImagePublisher : ROSBridgePublisher {
	public static string GetMessageTopic()
	{
		return "/image/compressed";
	}

	public static string GetMessageType()
	{
		return "sensor_msgs/CompressedImage";
	}

	public static string ToYAMLString(CompressedImageMsg msg)
	{
		return msg.ToYAMLString();
	}
}


public class CameraForwardLeftPublisher : CompressedImagePublisher {
	public static string GetMessageTopic()
	{
		return "/simulator/sensors/camera_fl/compressed";
	}
}


public class CameraForwardRightPublisher : CompressedImagePublisher {
	public static string GetMessageTopic()
	{
		return "/simulator/sensors/camera_fr/compressed";
	}
}


public class CameraDownwardLeftPublisher : CompressedImagePublisher {
	public static string GetMessageTopic()
	{
		return "/simulator/sensors/camera_dl/compressed";
	}
}


public class CameraUpwardLeftPublisher : CompressedImagePublisher {
	public static string GetMessageTopic()
	{
		return "/simulator/sensors/camera_ul/compressed";
	}
}
